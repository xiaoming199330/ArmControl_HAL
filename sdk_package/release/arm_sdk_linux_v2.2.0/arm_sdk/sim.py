# -*- coding: utf-8 -*-
"""
sim.py — 纯内存「假臂」：完整二进制协议模拟器（离线开发 / 自检，不需要硬件）

它不是动力学仿真器，而是**协议行为模拟器**：忠实复刻固件在文档里写明的
每一条应答语义与陷阱，让上位机可以在没有机械臂的情况下把 UI / 遥操 / 录制
全链路调通，并用来做回归测试。

已复刻的固件行为（都对得上 docs/网口指令大全_2026-09-21.md）：
  · 查询类回 `CMD|0x80`；控制类回 ACK 0x7F / NACK 0x7E + {原CMD, err}
  · `PING` 回 `0x81` + {proto_ver=1, run_id}（**不是 ACK**，第 2 字节不是错误码）
  · 读失败：`GET_STATUS` 单轴置 `-32768` 哨兵、位姿整体置 0、flags bit3=0
  · `MOVE_JOINTS` 目标角**静默钳制**（超限不报错）；急停回 err=5
  · `STREAM_JOINTS` 跳变 >25° **整帧拒收** err=4，且**被拒时不更新基准**（会一直拒）
  · `PARAM_WRITE` 超范围**静默钳制**仍回 err=0（只有回读才能确认真值）
  · `CMD >= 0x80` **故意不回**（防两端互回 NACK 死循环）
  · `5 + LEN != 报文长度` **静默丢弃**（连 NACK 都没有）
  · `CRC 错` 是唯一"坏包也回"的情况 → NACK err=2

2026-09-23 扩展新增复刻（见 docs/网口指令扩展设计_2026-09-23.md）：
  · `TEXT_EXEC/TEXT_FETCH`：**文本命令透传**。假臂里有一个小命令解释器，
    复刻 `s`/`h`/`zi`/`F`/`M a`/`zr`/`link`/`lk` 的输出与固件的安全闸门：
      - magic 不符 → NACK err=8(MAGIC)
      - 单独 `c`（校准向导）→ 闸门拒绝 + flags bit1，正文是**原因说明**
      - 急停中只放行 s/h（其余 flags bit1）
      - 11 条抢 ETH 引脚的命令 → 打印 "[NET] 已停用" 原因（**不是**禁止标记）
      - 输出 > 2048 B 只截断并置 flags bit0（绝不静默丢字符）
  · `GET_JOINT_DETAIL` 用**逐字段丢读**语义：`fail_joints` 里的轴 ok=0，
    temp/hw_err 回 `0xFF` 哨兵（不是 0 —— 0 是有效读数）。
  · `GET_LIMITS` 的 `zero_raw == 0` 表示未做 Home 零点校准（固件回落 180°）。
  · `IK_QUERY` 复用 `kinematics.ik_checked()`（与固件同闸门），把
    reason 映射成 UNREACH(3)/LIMIT(4)/FK_VERIFY(6)。
  · `CENTER_CALIB`/`ZERO_RESET`/`PARAM_SAVE` 走**同一套文本路径**（与固件
    一致：它们就是执行 `M a` / `zr` / `F`）。
  · `CALIB_ENTER` **一律 NACK err=1** —— 与固件一样，校准向导是控制台专属
    （它是交互式的，从网口进会卡死主循环）。
  · `SYS_RESET` 回 ACK 并记一次复位（假臂不复位自己，只计数）。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import List, Optional, Sequence

from .protocol import (AP_PARAM_COUNT, CMD_FK_QUERY, CMD_GET_STATUS,
                       CMD_LOG_SET_LEVEL, CMD_MOVE_CART, CMD_MOVE_JOINTS,
                       CMD_PARAM_COUNT, CMD_PARAM_READ, CMD_PARAM_WRITE, CMD_PING,
                       CMD_SET_CORRECTION, CMD_SET_GRAVITY, CMD_SET_TORQUE,
                       CMD_STOP, CMD_STREAM_JOINTS, ERR_DESC, ErrCode, MAX_PAYLOAD,
                       PARAM_TABLE, ProtocolError, RAW_READ_FAIL, RSP_ACK, RSP_FK,
                       RSP_NACK, RSP_PARAM_COUNT, RSP_PARAM_READ, RSP_PING,
                       RSP_STATUS, STREAM_RC_ESTOP, STREAM_RC_LIMIT, crc16, rd_i16,
                       rd_i32, rd_u16, unpack_logical)
# 2026-09-23 扩展：全指令覆盖（见 docs/网口指令扩展设计_2026-09-23.md）
from .protocol import (CMD_CALIB_ENTER, CMD_CENTER_CALIB, CMD_GET_CORRECTION,
                       CMD_GET_FLAGS, CMD_GET_GRAVITY, CMD_GET_INFO,
                       CMD_GET_JOINT_DETAIL, CMD_GET_LIMITS, CMD_GET_PID,
                       CMD_IK_QUERY, CMD_MOVE_JOINT_ONE, CMD_PARAM_LIST,
                       CMD_PARAM_SAVE, CMD_SERVO_INIT, CMD_SET_BUS_MODE,
                       CMD_SYS_RESET, CMD_TEXT_EXEC, CMD_TEXT_FETCH,
                       CMD_TEACH_COMPACT, CMD_TEACH_DELETE, CMD_TEACH_LIST,
                       CMD_TEACH_READ, CMD_TEACH_TOTAL, CMD_ZERO_RESET,
                       DETAIL_SIZE, HWERR_LOST, MAGIC, PARAM_LIST_MAX,
                       RSP_CORRECTION, RSP_FLAGS, RSP_GRAVITY, RSP_IK_QUERY,
                       RSP_INFO, RSP_JOINT_DETAIL, RSP_LIMITS, RSP_PARAM_LIST,
                       RSP_PID, RSP_TEACH_LIST, RSP_TEACH_READ, RSP_TEACH_TOTAL,
                       RSP_TEXT_FETCH, RSP_TEXT_OUT, TEACH_READ_MAX, TEMP_LOST,
                       TEXT_BUF_SZ, TEXT_CHUNK, TEXT_FLAG_GATE, TEXT_FLAG_TRUNC)
from .transport import RxEvent, Transport, UdpCarrier


class SimArmTransport(Transport):
    """内存假臂。也可当 `Transport` 用（send/recv_event 同步配对）。

    可配置的故障/行为：
        latency_s      每个响应的延迟（默认 0.0008 ≈ 0.8 ms，接近实测网口 p50）
        loss           丢包率 0~1（模拟 UDP 丢包）
        estop          急停状态
        fail_joints    指定读失败的轴（0 基），如 [2] 表示 J3 读失败
        unreachable    让 MOVE_CART 一律回 err=3（IK 不可达）
        clock          可选：注入时间源
    """

    carrier = UdpCarrier()
    name = "sim"

    def __init__(self, latency_s: float = 0.0008, loss: float = 0.0,
                 estop: bool = False, fail_joints: Optional[Sequence[int]] = None,
                 unreachable: bool = False, **kw):
        self.latency_s = latency_s
        self.loss = loss
        self.estop = estop
        self.fail_joints = list(fail_joints or [])
        self.unreachable = unreachable

        self.servo_deg: List[float] = [180.0] * 6        # 当前"实际"舵机度
        self.target_deg: List[float] = [180.0] * 6       # 轨迹目标
        self.torque = [1] * 6
        self.params = {k: float(v[4]) for k, v in PARAM_TABLE.items()}
        self.log_level = 2
        self.run_id = 0x5A
        self.proto_ver = 1
        self._seq = 0
        self._rng = 12345

        # 流模式
        self.stream_active = False
        self.stream_base: Optional[List[float]] = None   # 进入流那一帧的实际位置
        self.stream_last: Optional[List[float]] = None   # 上一帧成功的目标（逻辑角→舵机度）
        self.stream_frames = 0
        self.stream_rejects = 0
        self.stream_clamps = 0

        # 轨迹（简单的按速度插值）
        self.traj_speed = 25.0
        self.traj_start = 0.0

        # 校准：逻辑角 ⇄ 舵机度。默认零位 180°
        self.offset = [180.0] * 6
        self.dir = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        # raw 限位（0~4095）：默认全范围
        self.min_raw = [0] * 6
        self.max_raw = [4095] * 6

        # ---- 2026-09-23 扩展: 新命令需要的状态 ----
        self.pos_raw = [2048] * 6          # GET_LIMITS 的"当前/中位" raw
        self.zero_raw = [0] * 6            # 0 = 未做 Home 零点校准（与固件一致）
        self.bus_mode = 0                  # 0=正常 1=全A 2=全B
        self.pid = {j: (32, 0, 1) for j in range(6)}   # (p, i, d) 默认值
        self.corr_pending = 0
        self.build = 0                     # 0=net 1=can 2=both 3=uart
        self.link_mode = 0                 # 启动冻结的链路模式
        self.fw_tag = "SIM-NET-0923A"
        self.flags_extra = 0               # 额外状态位（示教录制/回放…）
        self.sys_reset_count = 0
        # 示教文件: idx -> {"name", "valid", "pts"(每点 6×u16 raw)}
        self.teach = {
            0: {"name": "demo", "valid": True,
                "pts": [[2048, 2048, 2048, 2048, 2048, 2048] for _ in range(12)]},
            1: {"name": "gone", "valid": False, "pts": [[1024] * 6]},
        }
        self.teach_capacity = 32
        self.teach_free = 60 * 1024
        # 文本透传的输出缓冲（复刻固件的 2048 B 捕获缓冲）
        self._text_buf = ""
        self._text_trunc = False
        self._text_gate = False

        self._rx: deque = deque()
        self._lock = threading.Lock()
        self._opened = False
        self._t0 = time.monotonic()

    # ---------------------------------------------------------------- Transport
    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    @property
    def is_open(self) -> bool:
        return self._opened

    def _rand(self) -> float:
        self._rng = (1103515245 * self._rng + 12345) & 0x7FFFFFFF
        return self._rng / 0x7FFFFFFF

    def send(self, seq: int, cmd: int, payload: bytes = b"") -> None:
        """把"发出去"的包当成"到达假臂"，生成响应压入接收队列。"""
        if not self._opened:
            raise RuntimeError("SimArmTransport 未打开")
        wire = self.carrier.build(seq, cmd, payload)
        try:
            seq2, cmd2, pl = unpack_logical(wire)      # 顺带验证编解码自洽
        except ProtocolError as e:
            if cmd < 0x80:
                self._push(seq, RSP_NACK, bytes([cmd & 0xFF, int(ErrCode.CRC)]))
            return
        if self.loss > 0 and self._rand() < self.loss:
            return                                     # 丢包
        resp = self._handle(seq2, cmd2, pl)
        if resp is None:
            # 故意不回（CMD>=0x80 / 坏长度）—— 只可能是超时
            return
        rcmd, rpl = resp
        if self.latency_s > 0:
            # 同步阻塞模拟延迟（自检用；异步场景请配合随包线程使用）
            time.sleep(self.latency_s)
        self._push(seq2, rcmd, rpl)

    def _push(self, seq: int, cmd: int, payload: bytes) -> None:
        with self._lock:
            self._rx.append(self.carrier.build(seq, cmd, payload))

    def recv_event(self, timeout: float = 0.2) -> RxEvent:
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            with self._lock:
                raw = self._rx.popleft() if self._rx else None
            if raw is not None:
                try:
                    seq, cmd, pl = self.carrier.parse(raw)
                    return RxEvent("packet", seq=seq, cmd=cmd, payload=pl, raw=raw)
                except ProtocolError as e:
                    return RxEvent("bad", error=str(e), raw=raw)
            if time.monotonic() >= deadline:
                return RxEvent("timeout")
            time.sleep(0.001)

    # ---------------------------------------------------------------- 内部
    def _now(self) -> float:
        return time.monotonic() - self._t0

    def _step_traj(self) -> None:
        """把实际位置朝目标位置按速度推进（让 GET_STATUS 有"在动"的样子）。"""
        dt = self._now() - self.traj_start
        self.traj_start = self._now()
        if dt <= 0:
            return
        max_step = max(1.0, self.traj_speed) * dt
        moving = False
        for j in range(6):
            d = self.target_deg[j] - self.servo_deg[j]
            if abs(d) > 0.05:
                moving = True
                self.servo_deg[j] += max(-max_step, min(max_step, d))
        self._traj_active = moving

    _traj_active = False

    def _clamp_servo(self, deg: Sequence[float]) -> List[float]:
        out = []
        for j in range(6):
            r = int(round(deg[j] / 360.0 * 4095)) % 4096
            if r < self.min_raw[j] or r > self.max_raw[j]:
                r = max(self.min_raw[j], min(self.max_raw[j], r))
            out.append(r / 4095.0 * 360.0)
        return out

    def _handle(self, seq: int, cmd: int, pl: bytes):
        # 响应码 >= 0x80 的包：固件故意不处理、不回（防死循环）
        if cmd >= 0x80:
            return None

        if cmd == CMD_PING:
            return RSP_PING, bytes([self.proto_ver, self.run_id])

        if cmd == CMD_GET_STATUS:
            self._step_traj()
            body = bytearray()
            for j in range(6):
                if j in self.fail_joints:
                    body += int(RAW_READ_FAIL).to_bytes(2, "little", signed=True)
                else:
                    body += int(round(self.servo_deg[j] * 10)).to_bytes(2, "little", signed=True)
            if self.fail_joints:
                # 固件：任一轴失败 ⇒ 跳过 FK ⇒ 位姿整体置 0，read_ok=0
                body += bytes(8)
                flags = 0x01 if self.estop else 0x00
            else:
                x, y, z, pitch = self._fk()
                body += int(round(x * 10)).to_bytes(2, "little", signed=True)
                body += int(round(y * 10)).to_bytes(2, "little", signed=True)
                body += int(round(z * 10)).to_bytes(2, "little", signed=True)
                body += int(round(pitch * 10)).to_bytes(2, "little", signed=True)
                flags = 0x08
                if self.estop:
                    flags |= 0x01
                if self._traj_active:
                    flags |= 0x02
            return RSP_STATUS, bytes(body) + bytes([flags])

        if cmd == CMD_FK_QUERY:
            self._step_traj()
            body = bytearray()
            if self.fail_joints:
                body += bytes(8)
                for j in range(6):
                    if j in self.fail_joints:
                        body += int(RAW_READ_FAIL).to_bytes(2, "little", signed=True)
                    else:
                        body += int(round(self.servo_deg[j] * 10)).to_bytes(2, "little", signed=True)
                return RSP_FK, bytes(body) + bytes([0])
            x, y, z, pitch = self._fk()
            body += int(round(x * 10)).to_bytes(2, "little", signed=True)
            body += int(round(y * 10)).to_bytes(2, "little", signed=True)
            body += int(round(z * 10)).to_bytes(2, "little", signed=True)
            body += int(round(pitch * 10)).to_bytes(2, "little", signed=True)
            for j in range(6):
                body += int(round(self.servo_deg[j] * 10)).to_bytes(2, "little", signed=True)
            return RSP_FK, bytes(body) + bytes([1])

        if cmd == CMD_LOG_SET_LEVEL:
            if len(pl) < 1:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self.log_level = min(4, pl[0])
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_MOVE_JOINTS:
            if len(pl) < 16:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self.estop:
                return RSP_NACK, bytes([cmd, int(ErrCode.ESTOP)])
            tgt = [rd_i16(pl, j * 2) / 10.0 for j in range(6)]
            v = rd_u16(pl, 12) / 10.0
            self.traj_speed = v if v >= 5 else 25.0
            self.target_deg = self._clamp_servo(tgt)   # 静默钳制
            for j in range(6):
                self.torque[j] = 1                     # MOVE_JOINTS 自动上力矩
            self.stream_active = False                 # 轨迹与流互斥
            self.traj_start = self._now()
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_STOP:
            self.target_deg = list(self.servo_deg)
            self.stream_active = False
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_SET_TORQUE:
            if len(pl) < 2:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            jn, en = pl[0], pl[1]
            if jn == 0xFF:
                self.torque = [1 if en else 0] * 6
            elif 0 <= jn <= 5:
                self.torque[jn] = 1 if en else 0
            else:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_STREAM_JOINTS:
            if len(pl) < 12:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self.estop:
                return RSP_NACK, bytes([cmd, int(ErrCode.ESTOP)])
            logical = [rd_i16(pl, j * 2) / 10.0 for j in range(6)]
            tgt = [(self.offset[j] + self.dir[j] * logical[j]) % 360.0 for j in range(6)]
            base = self.stream_base if self.stream_base is not None else self.servo_deg
            # 跳变判据：任一首超 25° ⇒ **整帧丢弃**，且基准不动（会一直拒）
            if any(abs(tgt[j] - base[j]) > 25.0 for j in range(6)):
                self.stream_rejects += 1
                return RSP_NACK, bytes([cmd, int(ErrCode.LIMIT)])
            clamped = self._clamp_servo(tgt)
            if any(abs(clamped[j] - tgt[j]) > 1e-6 for j in range(6)):
                self.stream_clamps += 1
            self.stream_base = list(tgt)               # 成功 ⇒ 基准前移
            self.stream_last = list(tgt)
            self.servo_deg = list(clamped)
            self.target_deg = list(clamped)
            self.stream_active = True
            self.stream_frames += 1
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_MOVE_CART:
            if len(pl) < 12:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self.estop:
                return RSP_NACK, bytes([cmd, int(ErrCode.ESTOP)])
            if self.unreachable:
                return RSP_NACK, bytes([cmd, int(ErrCode.UNREACH)])
            x, y, z, pitch = (rd_i16(pl, j * 2) / 10.0 for j in range(4))
            r = math.hypot(x, y)
            if r > 418.8 or z < 0:
                return RSP_NACK, bytes([cmd, int(ErrCode.UNREACH)])
            # 简化：直接用 FK 逆推不了就当作已接受（本模拟器不追求 IK 精度）
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_SET_GRAVITY:
            if len(pl) < 2:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            k = rd_i16(pl, 0) / 1000.0
            self.params[32] = max(0.0, min(0.5, k))    # 静默钳制
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_SET_CORRECTION:
            if len(pl) < 4:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self.params[33] = float(1 if pl[0] else 0)
            self.params[34] = max(0.5, min(10.0, rd_u16(pl, 1) / 100.0))
            self.params[35] = float(max(1, min(20, pl[3])))
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_PARAM_READ:
            if len(pl) < 1 or pl[0] not in PARAM_TABLE:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            pid = pl[0]
            val = int(round(self.params.get(pid, PARAM_TABLE[pid][4]) * 1000))
            return RSP_PARAM_READ, bytes([pid]) + val.to_bytes(4, "little", signed=True)

        if cmd == CMD_PARAM_WRITE:
            if len(pl) < 5:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            pid = pl[0]
            if pid not in PARAM_TABLE:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            v = rd_i32(pl, 1) / 1000.0
            _n, _u, vmin, vmax, _d = PARAM_TABLE[pid]
            self.params[pid] = max(vmin, min(vmax, v))    # 静默钳制，仍回 err=0
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_PARAM_COUNT:
            return RSP_PARAM_COUNT, bytes([AP_PARAM_COUNT])

        # ============================================================
        # 2026-09-23 扩展：全指令覆盖（见 docs/网口指令扩展设计_2026-09-23.md）
        # ============================================================

        # ---- 0x04 GET_JOINT_DETAIL ----
        if cmd == CMD_GET_JOINT_DETAIL:
            if len(pl) < 1:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            j = pl[0]
            if j == 0xFF:
                js = list(range(6))
            elif j <= 5:
                js = [j]
            else:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self._step_traj()
            body = bytearray([len(js), js[0]])
            for k in js:
                if k in self.fail_joints:
                    # 本轴整组丢读: pos/speed/load 无意义, temp/hw_err 回哨兵
                    body += bytes(2) + bytes(4)
                    body += bytes([0, TEMP_LOST, HWERR_LOST, 0])
                else:
                    raw = int(round(self.servo_deg[k] / 360.0 * 4095)) % 4096
                    body += raw.to_bytes(2, "little")                  # pos_raw
                    body += int(0).to_bytes(2, "little", signed=True)  # speed
                    body += int(0).to_bytes(2, "little", signed=True)  # load
                    body += bytes([120, 42, 0, 1])   # 12.0V, 42°C, hw_err=0, ok=1
            return RSP_JOINT_DETAIL, bytes(body)

        # ---- 0x05 GET_LIMITS ----
        if cmd == CMD_GET_LIMITS:
            body = bytearray()
            for k in range(6):
                body += int(self.min_raw[k]).to_bytes(2, "little")
                body += int(self.pos_raw[k]).to_bytes(2, "little")
                body += int(self.max_raw[k]).to_bytes(2, "little")
                body += int(self.zero_raw[k]).to_bytes(2, "little")
            return RSP_LIMITS, bytes(body)

        # ---- 0x07 GET_INFO ----
        if cmd == CMD_GET_INFO:
            tag = self.fw_tag[:16]
            body = bytearray([len(tag)])
            body += tag.encode("ascii", "replace") + bytes(16 - len(tag))
            body += bytes([self.build & 0xFF, self.link_mode & 0xFF,
                           AP_PARAM_COUNT & 0xFF])
            return RSP_INFO, bytes(body)

        # ---- 0x0A GET_FLAGS（只读变量，不碰总线） ----
        if cmd == CMD_GET_FLAGS:
            f = 0
            if self.estop:           f |= 0x01
            if self._traj_active:    f |= 0x02
            if self.stream_active:   f |= 0x04
            f |= self.flags_extra & 0xFF
            return RSP_FLAGS, bytes([f, self.run_id & 0xFF])

        # ---- 0x14 MOVE_JOINT_ONE ----
        if cmd == CMD_MOVE_JOINT_ONE:
            if len(pl) < 7:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self.estop:
                return RSP_NACK, bytes([cmd, int(ErrCode.ESTOP)])
            j = pl[0]
            if j > 5:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            # 固件先读全场快照、只改这一轴 ⇒ **任一轴丢读整条拒绝**
            # （拿不准位置时不动，比动错强）
            if self.fail_joints:
                return RSP_NACK, bytes([cmd, int(ErrCode.READ)])
            deg = rd_i16(pl, 1) / 10.0
            v = rd_u16(pl, 3) / 10.0
            if v >= 5.0:
                self.traj_speed = v
            tgt = list(self.target_deg)
            tgt[j] = deg
            self.target_deg = self._clamp_servo(tgt)     # 静默钳制
            self.torque = [1] * 6
            self.stream_active = False
            self.traj_start = self._now()
            return RSP_ACK, bytes([cmd, 0])

        # ---- 0x17 SET_BUS_MODE ----
        if cmd == CMD_SET_BUS_MODE:
            if len(pl) < 1 or pl[0] > 2:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self.bus_mode = pl[0]
            return RSP_ACK, bytes([cmd, 0])

        # ---- 0x18 SERVO_INIT（会重写 PID/限位/中位） ----
        if cmd == CMD_SERVO_INIT:
            if len(pl) < 1:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            j = pl[0]
            if j != 0xFF and j > 5:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self.estop:
                return RSP_NACK, bytes([cmd, int(ErrCode.ESTOP)])
            self.offset = [180.0] * 6       # 中位回 180°
            return RSP_ACK, bytes([cmd, 0])

        # ---- 0x22 IK_QUERY（只解不算） ----
        if cmd == CMD_IK_QUERY:
            if len(pl) < 8:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            x = rd_i16(pl, 0) / 10.0
            y = rd_i16(pl, 2) / 10.0
            z = rd_i16(pl, 4) / 10.0
            p = rd_i16(pl, 6) / 10.0
            if self.unreachable:
                return RSP_NACK, bytes([cmd, int(ErrCode.UNREACH)])
            from . import kinematics as K
            try:
                r = K.ik_checked(x, y, z, p,
                                 min_raw=self.min_raw, max_raw=self.max_raw,
                                 servo_offset=self.offset,
                                 keep_j5j6_servo_deg=self.servo_deg)
            except Exception:
                # 与 _fk 同一条纪律: 求解器异常不许兜底成"成功"
                return RSP_NACK, bytes([cmd, int(ErrCode.UNREACH)])
            if not r.ok:
                rc = {"unreachable": ErrCode.UNREACH,
                      "joint_limit": ErrCode.LIMIT,
                      "raw_limit": ErrCode.LIMIT,
                      "fk_verify": ErrCode.FK_VERIFY,
                      "pitch_not_met": ErrCode.FK_VERIFY}.get(r.reason, ErrCode.UNREACH)
                return RSP_NACK, bytes([cmd, int(rc)])
            body = b"".join(int(round(r.servo_deg[k] * 10)).to_bytes(2, "little",
                                                                     signed=True)
                            for k in range(6))
            return RSP_IK_QUERY, body

        # ---- 0x32/0x33 补偿补读侧 ----
        if cmd == CMD_GET_GRAVITY:
            k = self.params.get(32, 0.03)
            return RSP_GRAVITY, int(round(k * 1000)).to_bytes(2, "little", signed=True)

        if cmd == CMD_GET_CORRECTION:
            en = 1 if self.params.get(33, 1.0) else 0
            thr = self.params.get(34, 2.0)
            rounds = int(self.params.get(35, 6.0))
            body = (bytes([en])
                    + int(round(thr * 100)).to_bytes(2, "little")
                    + bytes([rounds & 0xFF, 1 if self.corr_pending else 0]))
            return RSP_CORRECTION, body

        # ---- 0x36 GET_PID（协议顺序固定 p/i/d） ----
        if cmd == CMD_GET_PID:
            if len(pl) < 1 or pl[0] > 5:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            jp, ji, jd = self.pid[pl[0]]
            ee = 0 if pl[0] in self.fail_joints else 1
            return RSP_PID, bytes([jp, ji, jd, ee])

        # ---- 0x43 PARAM_SAVE（等价文本 F） ----
        if cmd == CMD_PARAM_SAVE:
            if len(pl) < 4 or self._u32(pl) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM if len(pl) < 4
                                                       else ErrCode.MAGIC)])
            return RSP_ACK, bytes([cmd, 0])

        # ---- 0x44 PARAM_LIST（只列元数据，值用 PARAM_READ 取） ----
        if cmd == CMD_PARAM_LIST:
            if len(pl) < 1:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            ids = sorted(PARAM_TABLE.keys())
            page = ids[pl[0]:(pl[0] + PARAM_LIST_MAX)]
            body = bytearray([len(ids) & 0xFF, len(page)])
            for pid in page:
                _n, _u, vmin, vmax, _d = PARAM_TABLE[pid]
                body += bytes([pid & 0xFF, 0])       # type: 假臂不细分
                body += int(round(vmin * 1000)).to_bytes(4, "little", signed=True)
                body += int(round(vmax * 1000)).to_bytes(4, "little", signed=True)
            return RSP_PARAM_LIST, bytes(body)

        # ---- 0x50~0x55 示教/轨迹存储 ----
        if cmd == CMD_TEACH_LIST:
            if len(pl) < 1:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            idx = pl[0]
            if idx not in self.teach:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            f = self.teach[idx]
            name = f["name"][:11].encode("ascii", "replace")
            body = bytes([len(self.teach) & 0xFF, idx, 1 if f["valid"] else 0])
            body += name + bytes(12 - len(name))
            body += int(f["samples"] if "samples" in f else len(f["pts"])).to_bytes(
                4, "little", signed=True)
            return RSP_TEACH_LIST, body

        if cmd == CMD_TEACH_READ:
            if len(pl) < 4:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            idx = pl[0]
            off = rd_u16(pl, 1)
            n = min(pl[3], TEACH_READ_MAX)
            tr = self.teach.get(idx)
            if tr is None:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            page = tr["pts"][off:off + n]            # off 越界 → 空页（正常读完）
            body = bytearray([len(page)])
            for pt in page:
                for k in range(6):
                    body += int(pt[k]).to_bytes(2, "little")
            return RSP_TEACH_READ, bytes(body)

        if cmd == CMD_TEACH_DELETE:
            if len(pl) < 1 or pl[0] not in self.teach:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self.teach[pl[0]]["valid"] = False       # 写 tombstone
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_TEACH_COMPACT:
            for k in [k for k, v in self.teach.items() if not v["valid"]]:
                del self.teach[k]
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_TEACH_TOTAL:
            tot = sum(len(v["pts"]) for v in self.teach.values())
            body = (bytes([len(self.teach) & 0xFF, self.teach_capacity & 0xFF])
                    + int(tot).to_bytes(4, "little", signed=True)
                    + int(self.teach_free).to_bytes(4, "little", signed=True))
            return RSP_TEACH_TOTAL, body

        # ---- 0x70~0x73 危险操作（强制 MAGIC；与固件一样走文本路径） ----
        if cmd == CMD_CENTER_CALIB:
            if len(pl) < 4 or self._u32(pl) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM if len(pl) < 4
                                                       else ErrCode.MAGIC)])
            self._exec_text_line("M a")              # 与固件同一条路径
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_ZERO_RESET:
            if len(pl) < 5 or self._u32(pl, 1) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM if len(pl) < 5
                                                       else ErrCode.MAGIC)])
            if pl[0] > 1:
                # 文本 `zr` 没有单轴变体 —— 不猜, 直接报参数错（与固件一致）
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            self._exec_text_line("zr save" if pl[0] else "zr")
            return RSP_ACK, bytes([cmd, 0])

        if cmd == CMD_CALIB_ENTER:
            if len(pl) < 4 or self._u32(pl) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM if len(pl) < 4
                                                       else ErrCode.MAGIC)])
            # 固件**一律拒绝**：校准向导是交互式的, 从网口进会卡死主循环
            return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])

        if cmd == CMD_SYS_RESET:
            if len(pl) < 4 or self._u32(pl) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM if len(pl) < 4
                                                       else ErrCode.MAGIC)])
            # 真板子: 回 ACK 后 150ms 复位。假臂不复位自己, 只记账并清理运行态
            self.sys_reset_count += 1
            self.stream_active = False
            self.target_deg = list(self.servo_deg)
            return RSP_ACK, bytes([cmd, 0])

        # ---- 0x0B/0x0C 文本命令透传 ----
        if cmd == CMD_TEXT_EXEC:
            if len(pl) < 5:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            if self._u32(pl) != MAGIC:
                return RSP_NACK, bytes([cmd, int(ErrCode.MAGIC)])
            raw = pl[4:]
            nul = raw.find(b"\x00")
            if nul >= 0:
                raw = raw[:nul]
            line = raw[:58].decode("latin-1")
            out, gated = self._text_run(line, from_link=True)
            self._set_text_buf(out, gated)
            return RSP_TEXT_OUT, self._text_page(0, TEXT_CHUNK)

        if cmd == CMD_TEXT_FETCH:
            if len(pl) < 3 or pl[2] == 0:
                return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])
            return RSP_TEXT_FETCH, self._text_page(rd_u16(pl, 0),
                                                   min(pl[2], TEXT_CHUNK))

        return RSP_NACK, bytes([cmd, int(ErrCode.PARAM)])

    # ---------------------------------------------------------------- 文本透传
    # 复刻固件 ArmMain_TextGate + ArmMain_ExecText：一个**够用**的小解释器，
    # 只覆盖自检要用到的那些命令。真实固件跑的是完整的 124 条 ProcessCommand,
    # 这里不追求全 —— 但**闸门语义必须一致**（那才是上位机要防的坑）。
    _TEXT_DENY_BLOCKING = ("c",)                       # 交互式, 会卡死主循环
    _TEXT_DENY_PINS = ("cx", "cp", "cy", "cz", "cw", "ps", "can",
                       "mg", "mp", "mo", "ml")         # 抢 ETH 引脚（NET 构建）

    @staticmethod
    def _u32(b, o=0):
        return int.from_bytes(b[o:o + 4], "little", signed=False)

    def _text_page(self, off, n):
        """复刻 0x8B/0x8C 的分页负载：u16 total, u8 flags, u16 off, u8 n, text[n]

        ⚠ 偏移与总长都是**字节**（固件那边就是一块字节缓冲）。控制台的中文提示
          是 UTF-8 字节 ⇒ 这里也必须按字节分页，否则一页正好切在字符中间时
          上位机拼出来就是坏字符。
        """
        data = self._text_buf.encode("utf-8")
        chunk = data[off:off + n]
        flags = 0
        if self._text_trunc:
            flags |= TEXT_FLAG_TRUNC
        if self._text_gate:
            flags |= TEXT_FLAG_GATE
        return (int(len(data)).to_bytes(2, "little") + bytes([flags])
                + int(off).to_bytes(2, "little") + bytes([len(chunk)]) + chunk)

    def _set_text_buf(self, out, gated):
        """把一段输出放进捕获缓冲（**按字节**截到 TEXT_BUF_SZ，与固件一致）。"""
        raw = out.encode("utf-8")
        self._text_trunc = len(raw) > TEXT_BUF_SZ
        self._text_buf = raw[:TEXT_BUF_SZ].decode("utf-8", "ignore")
        self._text_gate = gated

    def _exec_text_line(self, line):
        """给 CENTER_CALIB/ZERO_RESET/PARAM_SAVE 复用的"执行一行文本"入口。"""
        out, _gated = self._text_run(line)
        self._set_text_buf(out, False)
        return out

    def _text_run(self, line, from_link=False):
        """返回 (输出文本, 是否被闸门拒绝)。"""
        t = line.strip()
        if from_link and self.estop and t not in ("s", "h"):
            return ("  [GATE] E-Stop active: only 's' and 'h' pass over the link.\r\n",
                    True)
        if from_link and (not t or t[0] == '#' or t in self._TEXT_DENY_BLOCKING):
            return ("  [GATE] refused over link: interactive/blocking command.\r\n",
                    True)
        tok = t.split()
        if not tok:
            return ("  Unknown command. Type 'h' for help.\r\n", False)
        t0 = tok[0]

        if t0 in self._TEXT_DENY_PINS:
            # 与 main.c 里那段 #if ARM_LINK_NET 的拦截文案同义（不是闸门拒绝，
            # 是 ProcessCommand 自己的"接住 + 说明"）
            return ("  [NET] '%s' 在当前链路(网口)下已停用 —— 它会启用 CAN。\r\n"
                    "  原因: 网口占用了 PB12/PB13(ETH_RXD1/ETH_TXD1),\r\n"
                    "        而 CAN2 正是同一对脚 ⇒ 允许切换/扫描会直接掐断网口。\r\n"
                    "  要用 CAN: `link can` → 再 `link save` → RESET 后生效。\r\n" % t0,
                    False)
        if t0 == "s":
            self._step_traj()
            cols = " ".join("%7.1f" % self.servo_deg[k] for k in range(6))
            return ("  J1       J2       J3       J4       J5       J6\r\n"
                    "  %s\r\n" % cols, False)
        if t0 == "h":
            # ⚠ 固件这里吐 5406 B（会被 2048 B 缓冲截断）。假臂只给摘要，
            #   并故意做得比 2048 长一点点 *不* 需要 —— 保持"短"更利于自检。
            return ("  === ArmControl_HAL console (simulated) ===\r\n"
                    "  s        status / joint angles\r\n"
                    "  zi/zr    Home zero check / reset\r\n"
                    "  F        save params to flash\r\n"
                    "  M a      center-calibrate all six\r\n"
                    "  link ... link preference (can/net/off, save)\r\n"
                    "  (real firmware lists all 124 commands here)\r\n", False)
        if t0 == "zi":
            zr = " ".join("%4d" % v for v in self.zero_raw)
            ok = all(v != 0 for v in self.zero_raw)
            return ("\r\n  ===== zi: Home 零点 / IK 限位 状态 =====\r\n"
                    "  calib_zero_raw (0=未校准 ⇒ 出厂中位 180°)\r\n"
                    "     [%s]\r\n"
                    "  %s\r\n\r\n" % (zr, "已校准。" if ok else
                                   "⚠ 有轴为 0 ⇒ 未校准, 量纲基准是脆的。"), False)
        if t0 == "F":
            return ("  Parameters saved to flash.\r\n", False)
        if t0 == "M" and len(tok) > 1 and tok[1] == "a":
            self.pos_raw = [2048] * 6
            return ("  J1 (bus1 id1): writing 40=128... pos=2048 OK (≈2048)\r\n"
                    "  ... (six joints) ...\r\n"
                    "  All 6 centered. WARNING: re-run 'c' Phase1+2 and save now!\r\n",
                    False)
        if t0 == "zr":
            save = len(tok) > 1 and tok[1] == "save"
            self.zero_raw = [0] * 6
            return ("\r\n  ===== zr: Home 零点已复位为出厂中位 (SERVO_OFFSET=180°) =====\r\n"
                    "  %s\r\n" % ("已写入 Flash (重启保持)。" if save else
                                "仅本次运行有效 — 重启会恢复 Flash 里的旧值; "
                                "要固化请敲 `zr save`。"), False)
        if t0 == "link":
            if len(tok) >= 2 and tok[1] in ("can", "net", "off"):
                return ("  Link preference -> %s (not saved yet; run `link save`, "
                        "then RESET)\r\n" % tok[1], False)
            if len(tok) >= 2 and tok[1] == "save":
                return ("  Link preference saved to flash.\r\n", False)
            return ("  Link preference: net (saved) / active: net\r\n", False)
        if t0 == "lk":
            return ("  Link UART4 (PA0/PA1 @115200): %s\r\n"
                    % ("STREAMING" if self.stream_active else "idle"), False)
        if t0 == "P" and len(tok) >= 2:
            # 固件 cmdPose（main.c case 'P'）：0=HOME(真折叠收纳位) 1=FOLD 2=READY 3/4=WAVE
            try:
                pid = int(float(tok[1]))
            except ValueError:
                pid = -1
            names = {0: "HOME", 1: "FOLD", 2: "READY", 3: "WAVE1", 4: "WAVE2"}
            if pid not in names:
                return ("  Unknown pose %d. Available: 0=HOME 1=FOLD 2=READY "
                        "3=WAVE1 4=WAVE2\r\n" % pid, False)
            return ("  Pose %d (%s): 180 180 180 180 180 180\r\n  Move started.\r\n"
                    % (pid, names[pid]), False)
        if t0 == "f":
            x, y, z, p = self._fk()
            return ("  FK: x=%.1f y=%.1f z=%.1f pitch=%.2f\r\n" % (x, y, z, p), False)
        return ("  Unknown command '%c'. Type 'h' for help.\r\n" % t0[0], False)

    # ---------------------------------------------------------------- 正向运动学
    def _fk(self):
        """用 kinematics 的参考实现算 FK（与固件同几何/同 JOINT_DIR）。

        ⚠ 这里**不吞异常**: 以前 `except: return 0,0,0,0` 会把"实现写错"伪装成
           "位姿 = 原点", 这是最坏的一种兜底(见工程红线: 读失败不得兜底)。
        """
        from . import kinematics as K
        p = K.fk_from_servo_deg(self.servo_deg, servo_offset=self.offset)
        return p.x, p.y, p.z, p.pitch_deg

    # ---------------------------------------------------------------- 测试辅助
    def set_joint_fail(self, joints: Sequence[int]) -> None:
        """设置读失败的轴（0 基）。空列表 = 全部正常。"""
        self.fail_joints = list(joints)

    def set_estop(self, on: bool) -> None:
        self.estop = bool(on)

    def stream_stats(self) -> dict:
        return {"frames": self.stream_frames, "rejects": self.stream_rejects,
                "clamps": self.stream_clamps}


__all__ = ["SimArmTransport"]
