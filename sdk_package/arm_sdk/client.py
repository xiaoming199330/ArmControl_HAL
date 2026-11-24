# -*- coding: utf-8 -*-
"""
client.py — 协议级客户端（网口 / 串口二进制 / 模拟器通用）

`ArmClient` 把「发一个逻辑包、等它的响应」这件事做对：
  · **SEQ 自增 + 按 SEQ 配对**（UDP 无顺序保证；固件只回显不校验，主机必须自己过滤）
  · 独立 RX 泵线程：查询/控制响应按 SEQ 派发；流帧的 ACK 进 `stream_acks`
  · 超时重发（**只对幂等命令默认重发**；运动命令重发会再规划一次同目标轨迹）
  · 错误码 → 人可读；E-Stop / 读失败 / 静默钳制如实上报，**不兜底**

用法：
    from arm_sdk import ArmClient, open_arm

    with open_arm("net://192.168.7.120:5000") as arm:      # 或 serial:///dev/ttyUSB0
        print(arm.ping())
        st = arm.get_status()
        print(st.joints)                                    # 舵机度；None = 读失败
        arm.move_joints([180]*6, v=20)
        arm.stream_joints([0, 10, 20, 30, 40, 50])
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .protocol import (CMD_FK_QUERY, CMD_GET_STATUS, CMD_LOG_SET_LEVEL,
                       CMD_MOVE_CART, CMD_MOVE_JOINTS, CMD_PARAM_COUNT,
                       CMD_PARAM_READ, CMD_PARAM_WRITE, CMD_PING,
                       CMD_SET_CORRECTION, CMD_SET_GRAVITY, CMD_SET_TORQUE,
                       CMD_STOP, CMD_STREAM_JOINTS, ERR_DESC, ErrCode, FkResult,
                       JointCalib, PingInfo, Status, cmd_name, decode_fk,
                       decode_param_count, decode_param_read, decode_status,
                       payload_log_set_level, payload_move_cart, payload_move_joints,
                       payload_param_read, payload_param_write, payload_set_correction,
                       payload_set_gravity, payload_set_torque, payload_stream_joints,
                       RSP_ACK, RSP_NACK, QUERY_RSP)

# 2026-09-23 扩展：网口全能力命令（设计基准 docs/网口指令扩展设计_2026-09-23.md）
from .protocol import (CMD_CALIB_ENTER, CMD_CENTER_CALIB, CMD_GET_CORRECTION,
                       CMD_GET_FLAGS, CMD_GET_GRAVITY, CMD_GET_INFO,
                       CMD_GET_JOINT_DETAIL, CMD_GET_LIMITS, CMD_GET_PID,
                       CMD_IK_QUERY, CMD_MOVE_JOINT_ONE, CMD_PARAM_LIST,
                       CMD_PARAM_SAVE, CMD_SERVO_INIT, CMD_SET_BUS_MODE,
                       CMD_SYS_RESET, CMD_TEXT_EXEC, CMD_TEXT_FETCH,
                       CMD_TEACH_COMPACT, CMD_TEACH_DELETE, CMD_TEACH_LIST,
                       CMD_TEACH_READ, CMD_TEACH_TOTAL, CMD_ZERO_RESET,
                       ArmFlags, ArmInfo, ArmLimits, CorrectionCfg, JointDetail,
                       ParamMeta, PidTriple, TeachFile, TeachTotals, TextOut,
                       TEACH_READ_MAX, TEXT_BUF_SZ, TEXT_CHUNK, TEXT_EXEC_MAX,
                       decode_correction, decode_flags,
                       decode_gravity, decode_ik_query, decode_info,
                       decode_joint_detail, decode_limits, decode_param_list,
                       decode_pid, decode_teach_list, decode_teach_read,
                       decode_teach_total, decode_text_out, payload_get_joint_detail,
                       payload_get_pid, payload_ik_query, payload_magic,
                       payload_move_joint_one, payload_param_list,
                       payload_servo_init, payload_set_bus_mode,
                       payload_teach_delete, payload_teach_list,
                       payload_teach_read, payload_text_exec, payload_text_fetch,
                       payload_zero_reset)
from .transport import NetTransport, SerialTransport, Transport, make_transport


class ArmCommandError(RuntimeError):
    """命令被固件明确拒绝（NACK）或超时。"""
    def __init__(self, reply: "Reply"):
        self.reply = reply
        super().__init__("%s 失败: %s" % (reply.label or cmd_name(reply.cmd),
                                        reply.desc if not reply.timeout else "超时"))


@dataclass
class Reply:
    """一条命令的结果。"""
    seq: int = -1
    req_cmd: int = -1
    cmd: int = -1                 # 响应码
    payload: bytes = b""
    ok: bool = False
    err: int = 0
    err_name: str = "OK"
    desc: str = "OK"
    latency_ms: float = 0.0
    attempts: int = 1
    timeout: bool = False
    label: str = ""
    dry_run: bool = False

    def raise_if_failed(self) -> "Reply":
        if not self.ok:
            raise ArmCommandError(self)
        return self

    def __str__(self) -> str:
        if self.dry_run:
            return "[dry-run] %s (未发送)" % (self.label or cmd_name(self.req_cmd))
        if self.timeout:
            return "✗ %s 超时(%.0fms, %d 次)" % (self.label or cmd_name(self.req_cmd),
                                                self.latency_ms, self.attempts)
        return "%s %s (%.1fms)%s" % ("✓" if self.ok else "✗",
                                     self.label or cmd_name(self.req_cmd),
                                     self.latency_ms,
                                     "" if self.ok else "  " + self.desc)


@dataclass
class ClientStats:
    sent: int = 0
    recv: int = 0
    timeout: int = 0
    retries: int = 0
    bad: int = 0
    unsolicited: int = 0
    stream_acks: int = 0
    stream_limit: int = 0
    stream_estop: int = 0


class _Slot:
    __slots__ = ("event", "reply")

    def __init__(self):
        self.event = threading.Event()
        self.reply: Optional[Reply] = None


# ============================================================================
# ArmClient
# ============================================================================

class ArmClient:
    """协议级客户端。线程安全（同一时刻只允许一条命令在途）。

    参数
    ----
    transport   : Transport 实例（NetTransport / SerialTransport / SimArmTransport）
    timeout     : 默认单条命令超时（秒）。建议 >= 0.3（MOVE_JOINTS 内部要读舵机总线）
    retries     : 默认重发次数（仅幂等命令默认启用）
    auto_open   : 构造时立即 open
    dry_run     : 只打印不发送（联调前校验脚本用；**安全开关放在最底层出口**）
    """

    def __init__(self, transport: Transport, timeout: float = 0.3, retries: int = 1,
                 auto_open: bool = True, dry_run: bool = False):
        self.transport = transport
        self.timeout = timeout
        self.retries = retries
        self.dry_run = dry_run

        self.stats = ClientStats()
        self.last_status: Optional[Status] = None
        self.last_fk: Optional[FkResult] = None
        self.stream_acks: deque = deque(maxlen=32)
        self.text_lines: deque = deque(maxlen=200)

        self.on_text: Optional[Callable[[str], None]] = None
        self.on_unsolicited: Optional[Callable[["Reply"], None]] = None

        self._seq = 0
        self._slots: Dict[int, _Slot] = {}
        self._lock = threading.RLock()
        self._pending_lock = threading.Lock()      # 串行化命令
        self._running = False
        self._thread: Optional[threading.Thread] = None

        if auto_open:
            self.open()

    # ---------------------------------------------------------------- 生命周期
    def open(self) -> None:
        if self._running:
            return
        self.transport.open()
        self._running = True
        self._thread = threading.Thread(target=self._pump, name="arm-rx", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self.transport.close()
        except Exception:
            pass

    def __enter__(self) -> "ArmClient":
        if not self._running:
            self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._running and self.transport.is_open

    # ---------------------------------------------------------------- RX 泵
    def _pump(self) -> None:
        while self._running:
            try:
                ev = self.transport.recv_event(0.05)
            except Exception:
                time.sleep(0.01)
                continue
            if ev.kind == "timeout":
                continue
            if ev.kind == "text":
                self.text_lines.append(ev.line)
                if self.on_text:
                    try:
                        self.on_text(ev.line)
                    except Exception:
                        pass
                continue
            if ev.kind == "bad":
                self.stats.bad += 1
                continue
            # kind == "packet"
            self.stats.recv += 1
            with self._lock:
                slot = self._slots.get(ev.seq)
                if slot is not None:
                    slot.reply = Reply(seq=ev.seq, cmd=ev.cmd, payload=ev.payload)
                    slot.event.set()
                    continue
            # 没人在等这个 seq：可能是流帧的迟到 ACK，或旧命令的残包
            if ev.cmd in (RSP_ACK, RSP_NACK) and len(ev.payload) >= 2:
                r = Reply(seq=ev.seq, req_cmd=ev.payload[0], cmd=ev.cmd,
                          payload=ev.payload, ok=(ev.cmd == RSP_ACK),
                          err=ev.payload[1], err_name=_err_name(ev.payload[1]),
                          desc=ERR_DESC.get(ev.payload[1], "?"))
                self.stream_acks.append(r)
                self.stats.stream_acks += 1
                if r.req_cmd == CMD_STREAM_JOINTS:
                    if r.err == int(ErrCode.LIMIT):
                        self.stats.stream_limit += 1
                    elif r.err == int(ErrCode.ESTOP):
                        self.stats.stream_estop += 1
            else:
                self.stats.unsolicited += 1
                if self.on_unsolicited:
                    try:
                        self.on_unsolicited(Reply(seq=ev.seq, cmd=ev.cmd, payload=ev.payload))
                    except Exception:
                        pass

    # ---------------------------------------------------------------- 发送 / 请求
    def next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def send_async(self, cmd: int, payload: bytes = b"") -> int:
        """只发不等。返回 seq（流模式用；配合 `poll_stream_acks()`）。"""
        seq = self.next_seq()
        if self.dry_run:
            print("[dry-run] send cmd=0x%02X seq=%d len=%d" % (cmd, seq, len(payload)))
            return seq
        self.transport.send(seq, cmd, payload)
        self.stats.sent += 1
        return seq

    def request(self, cmd: int, payload: bytes = b"", timeout: Optional[float] = None,
                retries: Optional[int] = None, label: str = "",
                idempotent: bool = True) -> Reply:
        """发一条命令并等响应。返回 `Reply`（**不抛异常**；要抛用 `.raise_if_failed()`）。"""
        timeout = self.timeout if timeout is None else timeout
        tries = (self.retries if retries is None else retries) + 1
        if not idempotent:
            tries = 1                      # 运动命令不自动重发
        label = label or cmd_name(cmd)

        if self.dry_run:
            return Reply(seq=-1, req_cmd=cmd, cmd=-1, ok=True, err=0,
                         err_name="DRY", desc="dry-run 未发送", label=label, dry_run=True)

        last: Reply = Reply(seq=-1, req_cmd=cmd, cmd=-1, ok=False, timeout=True, label=label)
        with self._pending_lock:
            for attempt in range(1, tries + 1):
                seq = self.next_seq()
                slot = _Slot()
                with self._lock:
                    self._slots[seq] = slot
                t0 = time.monotonic()
                try:
                    self.transport.send(seq, cmd, payload)
                    self.stats.sent += 1
                except Exception as e:                      # noqa: BLE001
                    with self._lock:
                        self._slots.pop(seq, None)
                    return Reply(seq=seq, req_cmd=cmd, ok=False, timeout=False,
                                 desc="发送失败: %s" % e, label=label, attempts=attempt)
                got = slot.event.wait(timeout)
                dt = (time.monotonic() - t0) * 1000.0
                with self._lock:
                    self._slots.pop(seq, None)

                if not got:
                    self.stats.timeout += 1
                    last = Reply(seq=seq, req_cmd=cmd, ok=False, timeout=True,
                                 latency_ms=dt, attempts=attempt, label=label,
                                 desc="超时（检查：目的端口 5000 / 5+LEN 是否等于报文长度 / 链路）")
                    if attempt < tries:
                        self.stats.retries += 1
                        continue
                    return last

                r = slot.reply
                assert r is not None
                r.req_cmd = cmd
                r.label = label
                r.latency_ms = dt
                r.attempts = attempt
                # 判定
                exp = QUERY_RSP.get(cmd)
                if r.cmd == exp:
                    r.ok, r.err, r.err_name, r.desc = True, 0, "OK", "OK"
                elif r.cmd == RSP_ACK and len(r.payload) >= 2:
                    r.ok = True
                    r.err = r.payload[1]
                    r.err_name, r.desc = _err_name(r.err), ERR_DESC.get(r.err, "?")
                elif r.cmd == RSP_NACK and len(r.payload) >= 2:
                    r.ok = False
                    r.err = r.payload[1]
                    r.err_name, r.desc = _err_name(r.err), ERR_DESC.get(r.err, "?")
                else:
                    r.ok = False
                    r.desc = "意外的响应码 0x%02X（期望 %s 或 ACK/NACK）" % (
                        r.cmd, ("0x%02X" % exp) if exp else "ACK/NACK")
                last = r
                # 超时才重发；明确 NACK 不重发
                if r.ok or r.timeout is False:
                    return r
            return last

    def poll_stream_acks(self, drain: bool = True) -> List[Reply]:
        """取走流帧的 ACK（异步流模式用）。"""
        out = list(self.stream_acks)
        if drain:
            self.stream_acks.clear()
        return out

    # ================================================================
    # 高层 API —— 全部 14 条命令
    # ================================================================

    # ---- 0x01 PING ----
    def ping(self, **kw) -> Optional[PingInfo]:
        r = self.request(CMD_PING, label="PING", **kw)
        if not r.ok or len(r.payload) < 2:
            return None
        # ⚠ 第 2 字节是 run_id（本次上电随机），**不是错误码**
        return PingInfo(proto_ver=r.payload[0], run_id=r.payload[1])

    # ---- 0x02 GET_STATUS ----
    def get_status(self, **kw) -> Status:
        r = self.request(CMD_GET_STATUS, label="GET_STATUS", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        st = decode_status(r.payload)
        self.last_status = st
        return st

    # ---- 0x21 FK_QUERY ----
    def fk_query(self, **kw) -> FkResult:
        r = self.request(CMD_FK_QUERY, label="FK_QUERY", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        fk = decode_fk(r.payload)
        self.last_fk = fk
        return fk

    # ---- 0x03 LOG_SET_LEVEL ----
    def log_set_level(self, level: int, **kw) -> Reply:
        return self.request(CMD_LOG_SET_LEVEL, payload_log_set_level(level),
                            label="LOG_SET_LEVEL", **kw)

    # ---- 0x10 MOVE_JOINTS ----
    def move_joints(self, servo_deg: Sequence[float], v: float = 0.0, a: float = 0.0,
                    **kw) -> Reply:
        """六关节 S 曲线运动（**舵机度**）。

        ⚠ err=0 只代表"命令被收下"：读起点失败时固件也回 ACK 但轨迹没起步。
           判断"动没动"必须用 `GET_STATUS` 看角度是否真的变化。
        ⚠ 超限是**静默钳制**（不报错，只打 WARNING 到串口）。
        """
        return self.request(CMD_MOVE_JOINTS, payload_move_joints(servo_deg, v, a),
                            label="MOVE_JOINTS", idempotent=False, **kw)

    # ---- 0x13 STREAM_JOINTS ----
    def stream_joints(self, logical_deg: Sequence[float], wait: bool = True,
                      timeout: Optional[float] = None) -> Optional[Reply]:
        """主从实时直写（**逻辑角**，相对本臂 Home）。

        `wait=False` 时只发不等（50 Hz 流的推荐用法），ACK 走 `poll_stream_acks()`。
        ⚠ 断流 >500ms 从臂自动退出流模式并原地自锁 ⇒ 频率必须 >2 Hz，建议 20~50 Hz。
        ⚠ 单帧单轴跳变 >25° ⇒ **整帧拒收** err=4，且被拒时不更新基准（会一直拒）
           —— `TeleopSession` 用 slew 限速从源头避免这个坑。
        """
        pl = payload_stream_joints(logical_deg)
        if wait:
            return self.request(CMD_STREAM_JOINTS, pl, label="STREAM_JOINTS",
                                timeout=timeout, idempotent=False)
        self.send_async(CMD_STREAM_JOINTS, pl)
        return None

    # ---- 0x11 STOP ----
    def stop(self, **kw) -> Reply:
        """立即停当前轨迹（平滑减速，**不是急停断电**）。总是成功。"""
        return self.request(CMD_STOP, label="STOP", **kw)

    # ---- 0x12 SET_TORQUE ----
    def set_torque(self, joint: int = 0xFF, enable: bool = True, **kw) -> Reply:
        """joint: 0~5 单轴 / 0xFF 全部。⚠ 6 是非法值（回 err=1）。"""
        return self.request(CMD_SET_TORQUE, payload_set_torque(joint, enable),
                            label="SET_TORQUE", idempotent=False, **kw)

    # ---- 0x20 MOVE_CART ----
    def move_cart(self, x: float, y: float, z: float, pitch: float = 0.0,
                  v: float = 0.0, a: float = 0.0, **kw) -> Reply:
        return self.request(CMD_MOVE_CART, payload_move_cart(x, y, z, pitch, v, a),
                            label="MOVE_CART", idempotent=False, **kw)

    # ---- 0x30 SET_GRAVITY ----
    def set_gravity(self, k_deg_per_mm: float, **kw) -> Reply:
        return self.request(CMD_SET_GRAVITY, payload_set_gravity(k_deg_per_mm),
                            label="SET_GRAVITY", **kw)

    # ---- 0x31 SET_CORRECTION ----
    def set_correction(self, enable: bool = True, threshold_deg: float = 2.0,
                       max_rounds: int = 10, **kw) -> Reply:
        return self.request(CMD_SET_CORRECTION,
                            payload_set_correction(enable, threshold_deg, max_rounds),
                            label="SET_CORRECTION", **kw)

    # ---- 0x40 PARAM_READ ----
    def param_read(self, param_id: int, **kw) -> Optional[float]:
        r = self.request(CMD_PARAM_READ, payload_param_read(param_id),
                         label="PARAM_READ #%d" % param_id, **kw)
        if not r.ok:
            return None
        if len(r.payload) >= 1 and r.payload[0] != (param_id & 0xFF):
            return None                       # id 回显不符 ⇒ 串包
        try:
            _pid, val = decode_param_read(r.payload)
        except Exception:
            return None
        return val

    # ---- 0x41 PARAM_WRITE ----
    def param_write(self, param_id: int, value: float, **kw) -> Reply:
        """⚠ 超范围**静默钳制**且仍回 err=0 —— 要确认真正写进去的值必须 `param_read` 回读。"""
        return self.request(CMD_PARAM_WRITE, payload_param_write(param_id, value),
                            label="PARAM_WRITE #%d=%.3f" % (param_id, value), **kw)

    def param_write_verified(self, param_id: int, value: float,
                             tol: float = 1e-3) -> Tuple[bool, float]:
        """写 + 回读确认。返回 (是否等于目标, 实际值)。"""
        self.param_write(param_id, value)
        back = self.param_read(param_id)
        if back is None:
            return False, float("nan")
        return abs(back - value) <= tol, back

    # ---- 0x42 PARAM_COUNT ----
    def param_count(self, **kw) -> int:
        r = self.request(CMD_PARAM_COUNT, label="PARAM_COUNT", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_param_count(r.payload)

    # ---------------------------------------------------------------- 便利方法
    def logical_angles(self, calib: JointCalib, **kw) -> Optional[List[float]]:
        """读 GET_STATUS 并转成**逻辑角**（STREAM_JOINTS 用的量纲）。

        读失败返回 None（**不兜底**）。
        """
        st = self.get_status(**kw)
        sv = st.as_servo_deg()
        return None if sv is None else calib.to_logical(sv)

    def check_read_ok(self, verbose: bool = False) -> bool:
        st = self.last_status
        if st is None:
            return False
        if verbose and st.failed_joints():
            print("  ⚠ 读失败的轴: J%s" % st.failed_joints())
        return st.read_ok

    # ================================================================
    # 2026-09-23 扩展：网口全能力命令
    # 设计基准 docs/网口指令扩展设计_2026-09-23.md
    #
    # 命名约定：会改硬件基准 / 会松开力矩 / 会改 Flash 的，方法名带
    # `_danger` 后缀 —— 让调用点在 review 时一眼可见。
    # ================================================================

    # ---- 0x04 GET_JOINT_DETAIL（温度 / 负载 / 电压 / 硬件错误） ----
    def get_joint_detail(self, joint: int = 0xFF, **kw) -> List[JointDetail]:
        """读关节遥测。joint=0xFF 读六轴，否则单轴。

        ⚠ 总线是**逐字段瞬时丢读**的（实测 Angle≈5% / Temp≈20%，各字段独立），
          所以别看单次结果下结论：
            · d.ok          本轴整组是否读成功
            · d.temp_lost   温度这一拍丢了（不是 0 度，也不是故障）
            · d.hw_err_lost 硬件错误寄存器读失败（≠ 硬件真的报错）
          要稳值就多读几次取多数，别把单次 0 当成真实温度。
        """
        r = self.request(CMD_GET_JOINT_DETAIL, payload_get_joint_detail(joint),
                         label="GET_JOINT_DETAIL", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_joint_detail(r.payload)

    # ---- 0x05 GET_LIMITS（限位与 Home 零点） ----
    def get_limits(self, **kw) -> ArmLimits:
        """读 6 轴的 raw 限位与 Home 零点（来自 Flash 校准记录）。

        ⚠ 这是**校准记录**，不是当前运行时限位。运行时限位（舵机度）在固件里
          是 JOINT_MIN/JOINT_MAX，本协议暂未开放读取。
        ⚠ zero_raw[k] == 0 表示该轴没做过 Home 零点校准 —— 此时固件回落
          180°/raw 2048，量纲基准是脆的，别在上面做精度动作。
        """
        r = self.request(CMD_GET_LIMITS, label="GET_LIMITS", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_limits(r.payload)

    # ---- 0x07 GET_INFO ----
    def get_info(self, **kw) -> ArmInfo:
        """读固件 tag / 构建类型 / 当前链路 / 参数条目数。

        ⚠ link_mode 是**启动冻结值**（不是 Flash 偏好）—— 「改了偏好但还没
          RESET」时两者会分叉，这正是判断"到底跑在哪条链路上"的依据。
        """
        r = self.request(CMD_GET_INFO, label="GET_INFO", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_info(r.payload)

    # ---- 0x0A GET_FLAGS（最轻的状态查询） ----
    def get_flags(self, **kw) -> ArmFlags:
        """读运行状态位图。**不碰舵机总线** ⇒ 适合 10~50Hz 轮询做 UI 状态灯。

        对照：get_status() 要读 6 轴舵机（约 4~6ms），这条只是读几个变量。
        """
        r = self.request(CMD_GET_FLAGS, label="GET_FLAGS", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_flags(r.payload)

    # ---- 0x14 MOVE_JOINT_ONE ----
    def move_joint_one(self, joint: int, servo_deg: float,
                       v: float = 0.0, a: float = 0.0, **kw) -> Reply:
        """单轴移动，其余五轴保持当前位置。

        ⚠ 角度是**舵机度**（0~360），不是 STREAM_JOINTS 的逻辑角。
        ⚠ 固件先读全场快照、只改这一轴目标 —— 任一轴丢读则回 ERR_READ(7)。
          这是有意的：拿不准位置时不动，比动错强。
        """
        return self.request(CMD_MOVE_JOINT_ONE,
                            payload_move_joint_one(joint, servo_deg, v, a),
                            label="MOVE_JOINT_ONE J%d" % (joint + 1),
                            idempotent=False, **kw)

    # ---- 0x17 SET_BUS_MODE ----
    def set_bus_mode(self, mode: int, **kw) -> Reply:
        """0=正常 A:AABBB / 1=全A / 2=全B。调试用，别放进生产路径。"""
        return self.request(CMD_SET_BUS_MODE, payload_set_bus_mode(mode),
                            label="SET_BUS_MODE %d" % mode, **kw)

    # ---- 0x18 SERVO_INIT ----
    def servo_init(self, joint: int = 0xFF, **kw) -> Reply:
        """舵机初始化。⚠ 会重写该轴 PID/限位/中位并 RESET，同文本命令 `i`。"""
        return self.request(CMD_SERVO_INIT, payload_servo_init(joint),
                            label="SERVO_INIT", idempotent=False, **kw)

    # ---- 0x22 IK_QUERY（只解不算） ----
    def ik_query(self, x: float, y: float, z: float,
                 pitch: float = 0.0, **kw) -> List[float]:
        """只做 IK 解算 + 限位检查 + FK 回代验证，**不启动运动**。

        返回六关节舵机度。失败抛 ArmCommandError，错误码区分原因：
        ERR_UNREACH(3) 不可达 / ERR_LIMIT(4) 超关节限位 / ERR_FK_VERIFY(6) 回代超差。
        用于「先问能不能到，再决定动不动」—— 比 move_cart 撞失败更省事。

        ⚠ 读取类方法约定：**失败一律抛 ArmCommandError**（不返回 None）。调用方
          不必再判 None，也就不会把 None 当成合法结果往下算。
        """
        r = self.request(CMD_IK_QUERY, payload_ik_query(x, y, z, pitch),
                         label="IK_QUERY", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_ik_query(r.payload)

    # ---- 0x32 / 0x33 补偿读回 ----
    def get_gravity(self, **kw) -> float:
        """读重力前馈增益 K（deg/mm）。失败抛 ArmCommandError。"""
        r = self.request(CMD_GET_GRAVITY, label="GET_GRAVITY", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_gravity(r.payload)

    def get_correction(self, **kw) -> CorrectionCfg:
        """读自适应修正配置（SET_CORRECTION 的反向）。失败抛 ArmCommandError。"""
        r = self.request(CMD_GET_CORRECTION, label="GET_CORRECTION", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_correction(r.payload)

    # ---- 0x36 GET_PID ----
    def get_pid(self, joint: int, **kw) -> PidTriple:
        """读单轴 PID（读自舵机 EEPROM，权威值）。失败抛 ArmCommandError。

        from_eeprom=False 表示**读失败**，p/i/d 不可信（是那个轴的从机没答，
        不是"PID 就是 0"）。
        """
        r = self.request(CMD_GET_PID, payload_get_pid(joint),
                         label="GET_PID J%d" % (joint + 1), **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_pid(r.payload)

    # ---- 0x44 PARAM_LIST（自动翻页） ----
    def param_list(self, **kw) -> List[ParamMeta]:
        """列举全部参数元数据（内部按 6 条/页自动翻页）。

        ⚠ 只有元数据（id/type/范围），**值要逐个 param_read 取** ——
          带上 i32 值会让响应帧超过 64B 上限，所以固件刻意拆开了。
        返回顺序 = 固件参数表顺序，列表索引即 PARAM_LIST 的 offset。
        """
        out: List[ParamMeta] = []
        offset = 0
        while True:
            r = self.request(CMD_PARAM_LIST, payload_param_list(offset),
                             label="PARAM_LIST @%d" % offset, **kw)
            if not r.ok:
                break
            total, page = decode_param_list(r.payload)
            if not page:
                break
            out.extend(page)
            offset += len(page)
            if offset >= total:
                break
        return out

    # ---- 0x50~0x55 示教文件存储 ----
    def teach_total(self, **kw) -> TeachTotals:
        """读存储总览：文件数 / 容量 / 总样本 / 剩余字节。失败抛 ArmCommandError。"""
        r = self.request(CMD_TEACH_TOTAL, label="TEACH_TOTAL", **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_teach_total(r.payload)

    def teach_list(self, idx: int, **kw) -> TeachFile:
        """读第 idx 个文件的信息（名字 / 样本数 / 是否已删）。失败抛 ArmCommandError。"""
        r = self.request(CMD_TEACH_LIST, payload_teach_list(idx),
                         label="TEACH_LIST #%d" % idx, **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_teach_list(r.payload)

    def teach_list_all(self, **kw) -> List[TeachFile]:
        """列举全部文件（按 teach_total 的文件数逐条取）。"""
        tot = self.teach_total(**kw)
        n = tot.n_files
        out: List[TeachFile] = []
        for i in range(n):
            f = self.teach_list(i, **kw)
            if f is not None:
                out.append(f)
        return out

    def teach_read(self, idx: int, off: int,
                   n: int = TEACH_READ_MAX, **kw) -> List[List[int]]:
        """读一页样本（n 上限 5 = 60B）。返回 n=0 表示 off 越界 = 正常读完。"""
        r = self.request(CMD_TEACH_READ, payload_teach_read(idx, off, n),
                         label="TEACH_READ #%d @%d" % (idx, off), **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_teach_read(r.payload)

    def teach_trace(self, idx: int, max_samples: int = 20000,
                    **kw) -> List[List[int]]:
        """⭐ 把一整条轨迹取下来（内部按 5 样本/页翻页）。

        1000 点轨迹 = 200 次请求；网口 p50 1.61ms ⇒ 约 0.3s 取完。
        max_samples 是安全上限 —— 防 samples 字段异常导致死循环。
        """
        out: List[List[int]] = []
        off = 0
        while len(out) < max_samples:
            page = self.teach_read(idx, off, TEACH_READ_MAX, **kw)
            if not page:
                break                      # n == 0 ⇒ 正常读完
            out.extend(page)
            off += len(page)
            if len(page) < TEACH_READ_MAX:
                break                      # 尾页不满 = 到底了
        return out

    def teach_delete(self, idx: int, **kw) -> Reply:
        """删除文件（写 tombstone，空间用 teach_compact 回收）。"""
        return self.request(CMD_TEACH_DELETE, payload_teach_delete(idx),
                            label="TEACH_DELETE #%d" % idx, **kw)

    def teach_compact(self, **kw) -> Reply:
        """回收 tombstone 占用的空间。⚠ 会擦写 Flash，耗时且不该频繁调。"""
        return self.request(CMD_TEACH_COMPACT, label="TEACH_COMPACT",
                            idempotent=False, **kw)

    # ---- 0x70~0x73 + 0x43 危险操作（MAGIC 由本层自动填） ----
    def center_calib_danger(self, **kw) -> Reply:
        """⚠ 等价文本 `M a`：六轴推到 raw 2048 中位。**会改变量纲基准**，不可逆。

        固件实现就是"执行一行 `M a`"，所以与串口行为逐字一致。
        """
        return self.request(CMD_CENTER_CALIB, payload_magic(),
                            label="CENTER_CALIB", idempotent=False, **kw)

    def calib_enter_danger(self, **kw) -> Reply:
        """⚠ 相位校准向导（文本 `c`）。**固件一律回 NACK(ERR_PARAM)** —— 见下。

        这条**设计上不可用**，不是实现漏了：cmdCalibrate() 里是 `for(;;)`
        等人在**本机串口**上敲 '0'/'s'/'q'。从网口进 = 主循环永不返回，而本
        工程**没有看门狗**（红线 5）⇒ 板子彻底失联但 USB 还在枚举，最难查的
        一种死法。它本来就要求人手摆位，是控制台专属命令。
        保留这个方法是为了让"不可用"这件事在 API 上**可见**，而不是让调用方
        收到一个没头没尾的参数错。要校准请接 COM11。
        """
        return self.request(CMD_CALIB_ENTER, payload_magic(),
                            label="CALIB_ENTER", idempotent=False, **kw)

    def zero_reset_danger(self, save: bool = False, **kw) -> Reply:
        """⚠ 等价文本 `zr` / `zr save`：把 Home 零点重置回出厂中位。

        ⚠ 一次复位**全部六轴** —— 文本 `zr` 没有单轴变体，所以这里也没有关节号。
        save=True 写 Flash 固化（= `zr save`，重启保持）；False 仅本次运行有效。
        量纲基准会随之改变，做精度动作前必须重新校准。
        """
        return self.request(CMD_ZERO_RESET, payload_zero_reset(save),
                            label="ZERO_RESET%s" % (" save" if save else ""),
                            idempotent=False, **kw)

    def param_save(self, **kw) -> Reply:
        """等价文本 `F`：把当前参数（PID/修正/速度/补偿…）落盘到 Flash。"""
        return self.request(CMD_PARAM_SAVE, payload_magic(),
                            label="PARAM_SAVE", idempotent=False, **kw)

    def sys_reset_danger(self, **kw) -> Reply:
        """⚠ 软复位。**先回 ACK、150 ms 后才真正复位**（固件刻意如此：顺序反了
        上位机只会看到超时，分不清"复位成功"和"包丢了"）。

        复位后按 **Flash 里的链路偏好**重建 —— 若偏好指向另一条链路（`link can`
        且已 `link save`），这台臂就不会再在网口上回话了。这正是远程切链路的
        完整序列：`text_exec("link can")` → `text_exec("link save")` → 本方法。
        """
        return self.request(CMD_SYS_RESET, payload_magic(),
                            label="SYS_RESET", idempotent=False, **kw)

    # ---- 0x0B/0x0C 文本命令透传：整个控制台（124 条） ----
    def text_exec(self, command: str, **kw) -> str:
        """在臂上执行**任意一条控制台命令**，返回它的输出（自动翻页）。

        这是"所有指令都支持网口下发"的入口：`client.text_exec("s")` 与在 COM11
        上敲 `s` 是**同一条** ProcessCommand，连回显文本都一样。

        能到什么程度：
          · 124 条控制台命令里 **113 条**可用；
          · 11 条抢 ETH 引脚的命令（cx/cp/cy/cz/cw/ps/can/mg/mp/mo/ml）会被固件
            拦下并**把原因当输出回给你**（不是静默失败）；
          · `c`（校准向导）会被安全闸门拒绝 —— 它是交互式的，会卡死主循环。
          · 急停时只放行 `s` / `h`（与主循环白名单一致）。

        ⚠ 命令最长 58 字符（TEXT_EXEC_MAX）；超长在这里就抛 ValueError，不让
          固件把命令钳断静默执行半条（半条往往是**另一条命令**）。
        ⚠ 输出上限 2048 B（TEXT_BUF_SZ）。超了会在末尾追加一条 [截断] 提示 ——
          `h`（帮助表 5406 B）就是这种情况，要看完整帮助请翻协议文档。
        ⚠ 会改硬件基准/松力矩的命令（`M a`/`zr`/`i`/`Y`）建议走**带 `_danger`
          后缀的类型化方法**，不要在这里随手拼字符串。
        """
        r = self.request(CMD_TEXT_EXEC, payload_text_exec(command),
                         label="TEXT_EXEC %r" % command, idempotent=False, **kw)
        if not r.ok:
            raise ArmCommandError(r)

        first = decode_text_out(r.payload)
        if first.gated:
            # 闸门拒绝：正文是**原因说明**，不是命令输出。原样返回更有用。
            # (这条路径 total 只等于原因文本长度, 下面的翻页循环自然不会进。)
            return first.text

        # ⚠ 按**字节**拼接，最后统一 decode 一次 —— 分页边界可能正好切在一个
        #   多字节 UTF-8 字符中间（固件控制台的中文提示是 UTF-8 字节），
        #   逐页 decode 会在缝上拼出坏字符。
        parts = [first.raw]
        off = first.off + len(first.raw)
        truncated = first.truncated
        guard = TEXT_BUF_SZ // TEXT_CHUNK + 2      # 防固件异常时死循环
        while off < first.total and len(parts) < guard:
            r2 = self.request(CMD_TEXT_FETCH, payload_text_fetch(off, TEXT_CHUNK),
                              label="TEXT_FETCH @%d" % off, **kw)
            if not r2.ok:
                break
            page = decode_text_out(r2.payload)
            if not page.raw:
                break
            parts.append(page.raw)
            off += len(page.raw)
            truncated = truncated or page.truncated

        text = b"".join(parts).decode("utf-8", "replace")
        if truncated:
            text += ("\n[TEXT_EXEC] 输出超过 %d B 已截断。这条命令的回显太长, "
                     "请改用结构化命令(如 GET_JOINT_DETAIL / GET_INFO / "
                     "TEACH_*)。" % TEXT_BUF_SZ)
        return text

    def text_out(self, command: str, **kw) -> TextOut:
        """同 text_exec，但只取**第一页**并保留分页元信息（total/truncated/gated）。

        给需要自己控节奏的调用方：比如只要 `s` 的前几行，或想区分"被闸门拒绝"
        与"命令正常执行但没输出"。
        """
        r = self.request(CMD_TEXT_EXEC, payload_text_exec(command),
                         label="TEXT_EXEC %r" % command, idempotent=False, **kw)
        if not r.ok:
            raise ArmCommandError(r)
        return decode_text_out(r.payload)

    def describe(self) -> str:
        s = self.stats
        return ("%s sent=%d recv=%d timeout=%d bad=%d stream_ack=%d(limit=%d estop=%d)"
                % (self.transport.name, s.sent, s.recv, s.timeout, s.bad,
                   s.stream_acks, s.stream_limit, s.stream_estop))


def _err_name(code: int) -> str:
    try:
        return ErrCode(code).name
    except ValueError:
        return "UNKNOWN(%d)" % code


# ============================================================================
# URI 解析：让 CLI / 示例一行接入
# ============================================================================

def open_arm(uri: str, timeout: float = 0.3, dry_run: bool = False, **kw) -> ArmClient:
    """按 URI 打开机械臂客户端。

        net://192.168.7.120:5000      UDP（首选）
        net://192.168.7.120           端口默认 5000
        serial:///dev/ttyUSB0         串口二进制（Linux）
        serial://COM11                （Windows）
        sim://                        内存假臂（离线）
    """
    uri = (uri or "").strip()
    if "://" not in uri:
        uri = "net://" + uri
    scheme, rest = uri.split("://", 1)
    scheme = scheme.lower()

    if scheme in ("net", "udp"):
        host, _, port = rest.partition(":")
        t = NetTransport(host or "192.168.7.120", int(port or 5000), **kw)
    elif scheme in ("serial", "ser"):
        port = rest.lstrip("/") if rest.startswith("/") and not rest.startswith("///") else rest
        # serial:///dev/ttyUSB0 -> rest == "/dev/ttyUSB0"
        if rest.startswith("/dev/"):
            port = rest
        elif ":" in rest:                        # serial://COM11:115200
            port, _, baud = rest.partition(":")
            kw.setdefault("baudrate", int(baud))
        t = SerialTransport(port, int(kw.pop("baudrate", 115200)))
    elif scheme in ("sim", "simulated", "mock"):
        from .sim import SimArmTransport
        t = SimArmTransport(**kw)
    else:
        raise ValueError("未知 URI scheme: %r" % scheme)

    return ArmClient(t, timeout=timeout, dry_run=dry_run)


__all__ = ["ArmClient", "ArmCommandError", "Reply", "ClientStats", "open_arm"]
