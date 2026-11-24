# -*- coding: utf-8 -*-
"""
teleop.py — 主从臂遥操（leader–follower teleoperation）

三条可用路线
------------
1. **网口直写（首选，本模块 `TeleopSession`）**
   主臂 `GET_STATUS`(50 Hz) → 逻辑角 → **限速** → 从臂 `STREAM_JOINTS 0x13`。
   UDP 实测 p50 1.61 ms、零丢包 ⇒ 50 Hz 有 15× 余量。两臂之间一根线都不用接。
2. **固件直连链路（UART4 PA0/PA1，`FirmwareLink`）**
   主臂自己把 0x13 帧转发给从臂，PC 只发开关命令（`mf`/`ma`/`sf`/`lk`）。
   需要两板 PA0↔PA1 交叉接线；本模块只做命令封装 + 状态读取。
3. **官网式 PC 中转（串口，`PcRelay`）**
   主臂 `sp <Hz>` 在控制台吐 `#P` 位置行 → PC 原样喂给从臂 → 从臂走同一
   `ArmApi_StreamJoints()` 通路。**不需要两臂之间接线**，只需两根 USB 串口。

固件硬约束（不遵守必然出症状，全部来自 `docs/网口指令大全_2026-09-21.md`）
--------------------------------------------------------------------
* 断流 **>500 ms** ⇒ 从臂自动退出流模式并在原地自锁 ⇒ 必须 >2 Hz，建议 20~50 Hz。
* 单帧单轴跳变 **>25°** ⇒ **整帧拒收** err=4，**且不更新跳变基准**（会一直拒 = 自锁）。
  ⇒ 本模块用 `slew_deg_per_s` 从源头限速；被拒帧不会被误当成"已生效"。
* 单帧 >64 B 装不下 ⇒ 6 轴 i16 共 12 B，安全。
* 主臂某轴读失败（`read_ok=0` 或哨兵 -32768）⇒ **不能**把 0 当成真位置发下去。
  ⇒ 本模块 hold 住上一个有效目标（既避免暴走，又保持流不超时）。

安全设计
--------
* 读失败 → hold（可配 `hold_on_leader_loss`）+ `link_loss_timeout_s` 后中止并停臂。
* 主臂 E-Stop → 立刻 `STOP` 从臂（可配 `estop_trips=False` 观察而不动）。
* 软限位（逻辑角）在限速**之前**夹，保证任何时候都不会命令臂走出包线。
* 非幂等命令不重发：`STREAM_JOINTS` 丢一帧只是少走一步，重发会重复规划。
* `dry_run` ⇒ 一个字节都不发（安全开关放在最底层出口，见 docs 里的教训）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .client import ArmClient, ArmCommandError
from .protocol import (JOINT_DIR, NUM_JOINTS, ErrCode, JointCalib, Status, cmd_name,
                       wrap180)
from .console import ArmSDK

__all__ = [
    "TeleopConfig", "TeleopStats", "TeleopSession",
    "TeleopError", "TeleopLinkLost", "TeleopAbort",
    "FirmwareLink", "PcRelay", "PcRelayStats",
    "FW_STREAM_TIMEOUT_MS", "FW_MAX_JUMP_DEG",
]

# ---------------------------------------------------------------- 固件常量
FW_STREAM_TIMEOUT_MS = 500.0     # 断流超时（固件侧硬编码）
FW_MAX_JUMP_DEG = 25.0           # 单帧单轴跳变拒帧门限
FW_MIN_HZ = 3.0                  # 低于这个必掉流（500ms → 2Hz，留余量）


# ============================================================================
# 异常
# ============================================================================

class TeleopError(RuntimeError):
    """遥操相关错误基类。"""


class TeleopLinkLost(TeleopError):
    """主臂连续读失败超过 `link_loss_timeout_s`（或超过 `max_leader_fail` 次）。"""


class TeleopAbort(TeleopError):
    """主臂 E-Stop / 从臂报 E-Stop，遥操已中止并停臂。"""


# ============================================================================
# 配置 / 统计
# ============================================================================

@dataclass
class TeleopConfig:
    """遥操参数。默认值按「两臂同型号、同朝向、都已用 `c` 校准」调好。"""

    # ---- 节拍 ----
    hz: float = 50.0
    """流频率。固件要求 >2 Hz；50 Hz 下 UDP 余量 15×，串口也够。"""

    # ---- 限速（防止 25° 拒帧自锁）----
    slew_deg_per_s: float = 240.0
    """每轴最大角速度。240°/s @50Hz = 4.8°/帧，离 25° 门限有 5× 余量。
    手猛拽主臂时靠它把速度削下来，而不是靠固件拒帧（拒帧会自锁）。"""

    max_step_deg: float = 0.0
    """>0 时额外限制单帧步长（度）。0 = 只用 `slew_deg_per_s`。"""

    # ---- 映射（主臂 → 从臂）----
    gain: float = 1.0
    reverse: object = False
    """False = 全轴同向；True = 全轴反向；也可以给 6 元素序列，
    **该位为真表示这一轴反向**（例：`[0,0,1,0,0,0]` 只反 J3）。
    镜像安装的从臂一般 J1/J4/J5/J6 需要反。"""

    # ---- 启动同步 ----
    sync: str = "hold"
    """`hold`   : 不动从臂，从"当前姿态"开始按增量跟随（最安全，默认）
    `align`  : 先把从臂 `MOVE_JOINTS` 到主臂当前姿态，再开始跟随
    `relative`/`absolute` 分别是 hold/align 的别名（兼容老写法）"""

    sync_speed: float = 20.0
    """`align` 时的 MOVE_JOINTS 速度（0~100）。"""

    # ---- 存活 / 安全 ----
    read_timeout: float = 0.2
    """单次 GET_STATUS 超时。50 Hz 时 0.2 s 已很宽松（实测 1.6 ms）。"""

    hold_on_leader_loss: bool = True
    """主臂读失败时把最后一个有效目标继续发下去（保住流，不超时自锁）。"""

    link_loss_timeout_s: float = 1.0
    """持续读失败超过这个时长 ⇒ 抛 `TeleopLinkLost`（并停从臂）。"""

    max_leader_fail: int = 50
    """连续失败帧数上限（0=不看这个判据，只看时长）。"""

    estop_trips: bool = True
    """主臂报 E-Stop 时是否真停从臂。False = 只记录不动作（观察模式）。"""

    enable_torque: bool = True
    """启动时给从臂 `SET_TORQUE 0xFF ON`（掉力矩的臂必须先上力）。"""

    soft_limits_deg: Optional[List[Tuple[float, float]]] = None
    """6 轴**逻辑角**软限位 [(min,max)]；None = 不限。
    运行时限位可先用 `L` 读出舵机度再换算，见 docs。"""

    wait_ack: bool = False
    """True = 每帧等 ACK（~1.6 ms，会掉到 ~200 Hz 上限）；False = 只发不等，
    异步收 ACK 做统计（推荐）。"""

    verbose: bool = False


@dataclass
class TeleopStats:
    frames: int = 0           # 循环次数
    sent: int = 0             # 真正发出的流帧
    held: int = 0             # 因主臂读失败而重发上一帧
    leader_fail: int = 0      # 主臂读失败次数
    ack_ok: int = 0
    ack_limit: int = 0        # 被固件拒帧 (err=4)
    ack_estop: int = 0
    soft_clamped: int = 0     # 被本模块软限位夹过
    slew_clamped: int = 0     # 被限速削过
    t_start: float = 0.0
    t_end: float = 0.0

    @property
    def elapsed(self) -> float:
        if not self.t_start:
            return 0.0
        return (self.t_end or time.monotonic()) - self.t_start

    @property
    def fps(self) -> float:
        e = self.elapsed
        return (self.frames / e) if e > 0 else 0.0

    def summary(self) -> str:
        return ("发送 %d 帧 / %.1fs (%.1f Hz) | hold %d 读失败 %d | "
                "ACK ok=%d limit=%d estop=%d | 软限位 %d 限速 %d"
                % (self.sent, self.elapsed, self.fps, self.held, self.leader_fail,
                   self.ack_ok, self.ack_limit, self.ack_estop,
                   self.soft_clamped, self.slew_clamped))


# ============================================================================
# TeleopSession —— 网口主从遥操
# ============================================================================

class TeleopSession:
    """一路主从遥操：`leader` 拖到哪，`follower` 跟到哪。

    最小用法::

        from arm_sdk import open_arm, TeleopSession, TeleopConfig

        lead = open_arm("net://192.168.7.121:5000")
        foll = open_arm("net://192.168.7.120:5000")
        with TeleopSession(lead, foll, hz=50) as tp:
            tp.run(duration=30)          # 拖 30 秒主臂
        print(tp.stats.summary())

    主从 **不需要** 两臂之间接线 —— PC 就是那根线（UDP 往返 1.61 ms）。

    参数
    ----
    leader / follower : `ArmClient`
    lead_calib / foll_calib : 各自的 `JointCalib`（逻辑角⇄舵机度）。
        ⚠ 只有两臂都做过 `c` 校准、Home 零点对应同一物理姿态，两边的逻辑角才可比。
        不校准也能跑（默认 offset=180），但会整体偏移，表现为"一开就从臂猛冲"。
    cfg : `TeleopConfig`
    on_frame : 每帧回调 `(lead_logical, target_logical)`，示教录制挂这里
    on_event : 文本事件回调（供 UI 日志）
    """

    def __init__(self, leader: ArmClient, follower: ArmClient,
                 lead_calib: Optional[JointCalib] = None,
                 foll_calib: Optional[JointCalib] = None,
                 cfg: Optional[TeleopConfig] = None,
                 on_frame: Optional[Callable[[List[float], List[float]], None]] = None,
                 on_event: Optional[Callable[[str], None]] = None):
        self.leader = leader
        self.follower = follower
        self.lead_calib = lead_calib or JointCalib()
        self.foll_calib = foll_calib or JointCalib()
        self.cfg = cfg or TeleopConfig()
        self.on_frame = on_frame
        self.on_event = on_event
        self.stats = TeleopStats()

        self._signs = self._resolve_reverse(self.cfg.reverse)
        self._lead_ref: Optional[List[float]] = None
        self._foll_ref: Optional[List[float]] = None
        self._last_sent: Optional[List[float]] = None
        self._last_good_t: float = 0.0
        self._last_t: float = 0.0
        self._active = False
        self.aborted = False

    # ---------------------------------------------------------------- 工具
    @staticmethod
    def _resolve_reverse(reverse) -> List[float]:
        if isinstance(reverse, (list, tuple)):
            if len(reverse) != NUM_JOINTS:
                raise ValueError("reverse 序列必须 6 个")
            return [(-1.0 if v else 1.0) for v in reverse]
        s = -1.0 if reverse else 1.0
        return [s] * NUM_JOINTS

    def _event(self, msg: str) -> None:
        """事件出口：有 on_event 就交给它（UI/日志），否则 verbose 时自己打。

        两者都给会重复打印，所以这里是**二选一**。
        """
        if self.on_event is not None:
            try:
                self.on_event(msg)
                return
            except Exception:
                pass
        if self.cfg.verbose:
            print("[teleop] " + msg)

    def _sync_mode(self) -> str:
        s = (self.cfg.sync or "hold").lower()
        if s in ("hold", "relative", "none", "off", ""):
            return "hold"
        if s in ("absolute", "align", "match"):
            return "align"
        raise ValueError("sync 只能是 hold / align（或别名）")

    # ---------------------------------------------------------------- 映射
    def _map(self, lead_logical: Sequence[float]) -> List[float]:
        """主臂逻辑角 → 从臂目标逻辑角（含增量参考、增益、反向）。

        target[j] = foll_ref[j] + gain * sign[j] * (lead[j] - lead_ref[j])
        即「从臂从它自己的起始姿态开始，按比例的位移量跟随主臂」。
        两臂 Home 零点差异被 `foll_ref` 吸收 ⇒ 不要求两臂起始姿态一致。
        """
        g = float(self.cfg.gain)
        out: List[float] = []
        have_ref = self._lead_ref is not None and self._foll_ref is not None
        for j in range(NUM_JOINTS):
            if have_ref:
                out.append(self._foll_ref[j]
                           + g * self._signs[j] * (lead_logical[j] - self._lead_ref[j]))
            else:
                out.append(g * self._signs[j] * lead_logical[j])
        return out

    def _clamp_soft(self, target: List[float]) -> List[float]:
        lim = self.cfg.soft_limits_deg
        if not lim:
            return target
        out = list(target)
        for j in range(NUM_JOINTS):
            if j >= len(lim):
                break
            lo, hi = lim[j]
            if out[j] < lo:
                out[j] = lo
                self.stats.soft_clamped += 1
            elif out[j] > hi:
                out[j] = hi
                self.stats.soft_clamped += 1
        return out

    def _slew(self, target: List[float], dt: float) -> List[float]:
        """按时间限速。dt<=0 时退化为按标称周期算。"""
        if self._last_sent is None:
            return list(target)
        if dt <= 0:
            dt = 1.0 / max(1.0, self.cfg.hz)
        cap = max(0.0, self.cfg.slew_deg_per_s) * dt
        if self.cfg.max_step_deg > 0:
            cap = min(cap, self.cfg.max_step_deg)
        out = []
        for j in range(NUM_JOINTS):
            d = target[j] - self._last_sent[j]
            if cap > 0 and abs(d) > cap:
                d = math.copysign(cap, d)
                self.stats.slew_clamped += 1
            out.append(self._last_sent[j] + d)
        return out

    # ---------------------------------------------------------------- 生命周期
    def start(self) -> "TeleopSession":
        """读两臂姿态 + 建参考 + （可选）对齐从臂。失败会抛 `TeleopError`。"""
        if self.cfg.hz < FW_MIN_HZ:
            raise TeleopError("hz=%.1f 太低（固件断流 500ms，必须 >2Hz）" % self.cfg.hz)
        mode = self._sync_mode()

        self._event("读取主臂姿态…")
        try:
            ls = self.leader.get_status(timeout=self.cfg.read_timeout)
        except ArmCommandError as e:
            raise TeleopError("主臂不可达：%s" % e) from e
        lsv = ls.as_servo_deg()
        if lsv is None:
            raise TeleopError("主臂读轴失败 J%s —— 先排总线，别带着瞎读数开遥操"
                              % ls.failed_joints())

        try:
            fs = self.follower.get_status(timeout=self.cfg.read_timeout)
        except ArmCommandError as e:
            raise TeleopError("从臂不可达：%s" % e) from e
        fsv = fs.as_servo_deg()

        lead_log = self.lead_calib.to_logical(lsv)

        if mode == "align":
            if fsv is None:
                raise TeleopError("从臂读轴失败 J%s —— 无法对齐" % fs.failed_joints())
            target_servo = self.foll_calib.to_servo(
                self._abs_mapped(lead_log))
            self._event("对齐从臂到主臂姿态（MOVE_JOINTS v=%.0f）…" % self.cfg.sync_speed)
            r = self.follower.move_joints(target_servo, v=self.cfg.sync_speed)
            if not r.ok:
                raise TeleopError("对齐失败：%s" % r)
            fsv = self._wait_settle(target_servo)

        if fsv is None:
            # hold 模式下从臂读失败：允许用"零参考"，但必须显式告知
            self._event("⚠ 从臂读轴失败，参考取 0（从臂真实姿态未知，先小范围试动）")
            foll_log = [0.0] * NUM_JOINTS
        else:
            foll_log = self.foll_calib.to_logical(fsv)

        self._lead_ref = list(lead_log)
        self._foll_ref = list(foll_log)
        self._last_sent = list(foll_log)
        self._last_good_t = time.monotonic()
        self._last_t = 0.0

        if self.cfg.enable_torque:
            self.follower.set_torque(0xFF, True)

        self._active = True
        self.aborted = False
        self.stats = TeleopStats()
        self.stats.t_start = time.monotonic()
        self._event("遥操开始：%.0f Hz，限速 %.0f°/s，sync=%s" %
                    (self.cfg.hz, self.cfg.slew_deg_per_s, mode))
        return self

    def _abs_mapped(self, lead_log: Sequence[float]) -> List[float]:
        """不带增量参考的绝对映射（对齐用）。"""
        g = float(self.cfg.gain)
        return [lead_log[j] * g * self._signs[j] for j in range(NUM_JOINTS)]

    def _wait_settle(self, target_servo: Sequence[float]) -> Optional[List[float]]:
        """等 MOVE_JOINTS 走完（读轮到稳定为止），最多 5 s。返回实际舵机度。"""
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
            try:
                st = self.follower.get_status(timeout=self.cfg.read_timeout)
            except ArmCommandError:
                continue
            sv = st.as_servo_deg()
            if sv is None:
                continue
            err = max(abs(wrap180(sv[j] - target_servo[j])) for j in range(NUM_JOINTS))
            if last is not None and err < 1.5:
                return sv
            last = sv
        self._event("⚠ 对齐未在 5s 内收敛，按当前读数继续")
        return last

    def stop(self, stop_follower: bool = True) -> TeleopStats:
        """结束遥操。默认给从臂发一条 `STOP`（平滑减速，不是断电急停）。"""
        self._active = False
        self.stats.t_end = time.monotonic()
        if stop_follower and not self.follower.dry_run:
            try:
                self.follower.stop()
            except Exception:
                pass
        self._event("遥操结束：" + self.stats.summary())
        return self.stats

    def __enter__(self) -> "TeleopSession":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # ---------------------------------------------------------------- 单帧
    def step(self) -> Optional[List[float]]:
        """走一帧。返回实际发给从臂的 6 个逻辑角；hold/中止时返回 None。"""
        if not self._active:
            return None
        now = time.monotonic()
        dt = (now - self._last_t) if self._last_t else (1.0 / max(1.0, self.cfg.hz))
        self._last_t = now
        self.stats.frames += 1

        # ---- 1) 读主臂 ----
        try:
            st = self.leader.get_status(timeout=self.cfg.read_timeout)
        except ArmCommandError:
            return self._on_leader_lost(dt, "读超时")
        if st.estop:
            self._on_estop("主臂报 E-Stop")
            return None
        lsv = st.as_servo_deg()
        if lsv is None:
            return self._on_leader_lost(dt, "读轴失败 J%s" % st.failed_joints())

        # ---- 2) 映射 + 软限位 + 限速 ----
        lead_log = self.lead_calib.to_logical(lsv)
        target = self._clamp_soft(self._map(lead_log))
        target = self._slew(target, dt)

        # ---- 3) 下发 ----
        self._send(target)
        self._last_sent = target
        self._last_good_t = now
        self.stats.leader_fail = 0

        if self.on_frame:
            try:
                self.on_frame(lead_log, target)
            except Exception:
                pass
        return target

    def _send(self, target: Sequence[float]) -> None:
        if self.cfg.wait_ack:
            r = self.follower.stream_joints(target, wait=True,
                                            timeout=self.cfg.read_timeout)
            if r is None:
                return
            self._account_ack(r.ok, r.err)
        else:
            self.follower.stream_joints(target, wait=False)
            self.stats.sent += 1
            for a in self.follower.poll_stream_acks():
                self._account_ack(a.ok, a.err)

    def _account_ack(self, ok: bool, err: int) -> None:
        if ok or err == int(ErrCode.OK):
            self.stats.ack_ok += 1
        elif err == int(ErrCode.LIMIT):
            self.stats.ack_limit += 1
            if self.stats.ack_limit in (1, 10, 100):
                self._event("⚠ 从臂拒帧 err=4（跳变>25° 或超限）第 %d 次"
                            % self.stats.ack_limit)
        elif err == int(ErrCode.ESTOP):
            self.stats.ack_estop += 1
            self._event("⚠ 从臂报 E-Stop")
        elif err == int(ErrCode.UNREACH):
            # 从臂没进入流模式（常见：sf 0 关过跟随，或 500ms 掉流后尚未恢复）
            self._event("⚠ 从臂未接受流帧 err=3（检查 `sf 1` / 断流）")

    def _on_leader_lost(self, dt: float, why: str) -> Optional[List[float]]:
        self.stats.leader_fail += 1
        gone = time.monotonic() - self._last_good_t
        if self.cfg.verbose and self.stats.leader_fail in (1, 25, 100):
            self._event("主臂读失败(%s) x%d，已 %.2fs" %
                        (why, self.stats.leader_fail, gone))
        if (self.cfg.max_leader_fail and
                self.stats.leader_fail >= self.cfg.max_leader_fail) or \
                gone > self.cfg.link_loss_timeout_s:
            self.stop(stop_follower=True)
            raise TeleopLinkLost("主臂链路丢失 %.2fs（%s）" % (gone, why))
        if self.cfg.hold_on_leader_loss and self._last_sent is not None:
            self._send(self._last_sent)      # 保住流，别让从臂 500ms 后自锁
            self.stats.held += 1
        return None

    def _on_estop(self, why: str) -> None:
        self._event("🛑 %s —— 停从臂" % why)
        if self.cfg.estop_trips:
            self.aborted = True
            self.stop(stop_follower=True)
            raise TeleopAbort(why)
        self.stats.ack_estop += 1

    # ---------------------------------------------------------------- 主循环
    def run(self, duration: Optional[float] = None,
            until: Optional[Callable[[], bool]] = None,
            on_tick: Optional[Callable[[TeleopStats], None]] = None,
            driver: Optional[Callable[[float], None]] = None) -> TeleopStats:
        """按 `hz` 跑到 `duration` 秒 / `until()` 为真 / Ctrl-C。

        超时抖动由"补偿式休眠"吸收（不是固定 sleep(1/hz)，否则实测只有 ~35 Hz）。

        `driver(elapsed_s)` 每帧先调一次 —— 给自动化脚本/演示用（真的遥操时留空，
        由人手拖主臂）。
        """
        if not self._active:
            self.start()
        period = 1.0 / max(1.0, self.cfg.hz)
        t0 = time.monotonic()
        nxt = t0
        try:
            while True:
                el = time.monotonic() - t0
                if until is not None and until():
                    break
                if duration is not None and el >= duration:
                    break
                if driver is not None:
                    try:
                        driver(el)
                    except Exception:
                        pass
                try:
                    self.step()
                except (TeleopLinkLost, TeleopAbort):
                    raise
                if on_tick:
                    on_tick(self.stats)
                nxt += period
                slp = nxt - time.monotonic()
                if slp > 0:
                    time.sleep(slp)
                else:
                    nxt = time.monotonic()      # 落后了就放弃追帧，避免雪崩
        except KeyboardInterrupt:
            self._event("用户中断 (Ctrl-C)")
        finally:
            self.stop()
        return self.stats


# ============================================================================
# 固件直连链路（UART4 PA0/PA1）—— 串口文本命令封装
# ============================================================================

class FirmwareLink:
    """主从 **固件直连** 路线的串口命令封装（走 `ConsoleClient`）。

    这条路线的数据流在两块板之间（PA0↔PA1），PC 只负责开开关 + 看诊断::

        lead = ConsoleClient("COM9");  lead.open()
        foll = ConsoleClient("COM11"); foll.open()
        L, F = FirmwareLink(lead), FirmwareLink(foll)
        print(L.link_status())        # lk  主从链路诊断
        L.forwarding(True)            # mf 1 开转发（上电默认 ON）
        F.follow(True)                # sf 1
        L.active(False)               # ma 0 手拖主臂

    ⚠ 这条路线的"能不能通"取决于 PA0/PA1 的物理接线与焊接（见
      `docs/主从臂联调步骤_2026-09-11.md` §13）—— 命令不回错 ≠ 线是通的，
      必须用 `link_status()` 看字节数/帧数/CRC 错，或 `gpio_loopback()`。
    """

    def __init__(self, console: ArmSDK):
        self.c = console

    # ---- 命令 ----
    def link_status(self):
        """`lk` 主从链路诊断：字节数 / 帧数 / CRC 错 / 收到的是哪种 cmd。"""
        return self.c.send_command("lk")

    def snapshot(self):
        """`ms` 两块板通用：打 6 轴 rel-home 逻辑角（对拍两臂校准一致性）。"""
        return self.c.send_command("ms")

    def forwarding(self, on: bool):
        """`mf 0|1` 主臂：暂停/恢复 0x13 直连转发（暂停时链路静默、主臂仍可拖）。"""
        return self.c.send_command("mf %d" % (1 if on else 0))

    def active(self, on: bool):
        """`ma 0|1` 主臂：1=上力矩+放行动作命令，0=松力矩手拖（上电默认 0）。"""
        return self.c.send_command("ma %d" % (1 if on else 0))

    def follow(self, on: bool):
        """`sf 0|1` 从臂：0=退出流模式并忽略后续流帧（原地自锁），1=恢复。"""
        return self.c.send_command("sf %d" % (1 if on else 0))

    def stream(self, hz: int):
        """`sp <Hz>` 主臂：按节拍往控制台吐 `#P` 位置行（PC 中转的输出端）；`sp 0` 停。"""
        return self.c.send_command("sp %d" % max(0, min(100, int(hz))))

    def stream_off(self):
        return self.stream(0)

    # ---- 自检 ----
    def loopback(self, n: int = 200):
        """`ml N` 主臂自环自检：不看舵机发 N 帧，用 CRC 正确的回环帧数判通路。"""
        return self.c.send_command("ml %d" % int(n), timeout=15.0)

    def gpio_loopback(self):
        """`mg` 主臂 GPIO 级自环（PA0<->PA1 跳线，链路要拔掉）。"""
        return self.c.send_command("mg", timeout=10.0)

    def pin_probe(self, port: Optional[str] = None, pin: Optional[int] = None):
        """`mp [端口 引脚]` 主臂引脚直流探测（不需跳线）。"""
        return self.c.send_command("mp" if port is None else "mp %s %s" % (port, pin),
                                   timeout=15.0)

    def pin_hold(self, port: str, pin: int, level: int):
        """`mo A 1 0|1` 主臂：把某脚输出并保持（配合万用表找脚）；保持期间 UART4 暂停。"""
        return self.c.send_command("mo %s %s %d" % (port, pin, 1 if level else 0))

    def pin_release(self):
        """`mo r` 释放引脚，恢复 UART4 AF8。"""
        return self.c.send_command("mo r")

    # ---- 链路开关（工程红线 69：运行期只能用启动冻结的模式）----
    def link_show(self):
        """`link` 打印本次生效 / 下次开机的链路选择。"""
        return self.c.send_command("link")

    def link_set(self, mode: str):
        """`link can|net|off` 改**偏好**（不立即生效！）。"""
        m = mode.lower()
        if m not in ("can", "net", "off"):
            raise ValueError("mode 只能是 can / net / off")
        return self.c.send_command("link %s" % m)

    def link_save(self):
        """`link save` 把偏好写 Flash；**必须 RESET 才生效**。"""
        return self.c.send_command("link save", timeout=10.0)


# ============================================================================
# PC 中转（官网式串口路线）
# ============================================================================

@dataclass
class PcRelayStats:
    lines_in: int = 0          # 收到的主臂 #P 行
    forwarded: int = 0         # 转发给从臂的帧
    slew_clamped: int = 0
    parse_bad: int = 0
    t_start: float = 0.0
    t_end: float = 0.0

    @property
    def elapsed(self) -> float:
        if not self.t_start:
            return 0.0
        return (self.t_end or time.monotonic()) - self.t_start

    @property
    def fps(self) -> float:
        e = self.elapsed
        return (self.lines_in / e) if e > 0 else 0.0

    def summary(self) -> str:
        return ("主臂 %d 行 / %.1fs (%.1f Hz) | 转发 %d | 限速 %d"
                % (self.lines_in, self.elapsed, self.fps, self.forwarded,
                   self.slew_clamped))


class PcRelay:
    """官网式「PC 中转」主从遥操（**串口**版，即 `tools/pc_relay.py` 的 SDK 化）。

    为什么需要它：某些板子 PA0/PA1 焊点不良，固件直连链路不通；此时两臂之间
    一根线都不接，由 PC 用两根 USB 串口当中转::

        主臂 `sp 50`  ──控制台吐──▶  PC  ──`#P ...`──▶  从臂 0x13 通路

    ⚠ **必须先 `mf 0`**（静默固件直连链路），否则两路各读一遍舵机、总线负载翻倍。
    ⚠ `sp` 的行会走异步通道（`ConsoleClient.on_position`）；若 `sp` 开着同时下发
      文本命令，位置行不再污染响应（2026-09-22 修）。
    """

    def __init__(self, leader_console: ArmSDK, follower_console: ArmSDK,
                 hz: int = 30, slew_deg_per_s: float = 240.0,
                 dry_run: bool = False, auto_gate: bool = True,
                 on_event: Optional[Callable[[str], None]] = None):
        self.lc = leader_console
        self.fc = follower_console
        self.hz = int(max(1, min(100, hz)))
        self.slew = float(slew_deg_per_s)
        self.dry_run = dry_run
        self.auto_gate = auto_gate
        self.on_event = on_event
        self.stats = PcRelayStats()
        self._last: Optional[List[float]] = None
        self._last_t: float = 0.0
        self._running = False

    def _event(self, m: str) -> None:
        if self.on_event:
            try:
                self.on_event(m)
            except Exception:
                pass

    def _on_pos(self, logical: List[float]) -> None:
        self.stats.lines_in += 1
        now = time.monotonic()
        dt = (now - self._last_t) if self._last_t else (1.0 / self.hz)
        self._last_t = now
        tgt = list(logical)
        if self._last is not None:
            cap = self.slew * dt
            for j in range(NUM_JOINTS):
                d = tgt[j] - self._last[j]
                if cap > 0 and abs(d) > cap:
                    d = math.copysign(cap, d)
                    self.stats.slew_clamped += 1
                tgt[j] = self._last[j] + d
        self._last = tgt
        self._forward(tgt)

    def _forward(self, tgt: Sequence[float]) -> None:
        # 与 0x13 同语义；从臂成功时**完全静默** ⇒ 不需要等响应
        line = "#P " + " ".join("%d" % int(round(v * 10.0)) for v in tgt)
        if self.dry_run:
            return
        self.fc.write_raw(line + "\r")
        self.stats.forwarded += 1

    def start(self) -> "PcRelay":
        if self.auto_gate:
            self._event("主臂 mf 0（静默固件直连链路，避免两路同时读总线）")
            try:
                self.lc.send_command("mf 0", timeout=2.0)
            except Exception:
                self._event("⚠ mf 0 无响应（板子上没有该命令？）")
            try:
                self.fc.send_command("sf 1", timeout=2.0)
            except Exception:
                self._event("⚠ sf 1 无响应")
            try:
                self.fc.send_command("ma 0", timeout=2.0)
            except Exception:
                pass
        self.stats = PcRelayStats()
        self.stats.t_start = time.monotonic()
        self._last = None
        self._last_t = 0.0
        self.lc.on_position = self._on_pos
        self._running = True
        self._event("主臂 sp %d 开流…" % self.hz)
        self.lc.send_command("sp %d" % self.hz, timeout=3.0)
        return self

    def stop(self) -> PcRelayStats:
        self._running = False
        self.stats.t_end = time.monotonic()
        self.lc.on_position = None
        try:
            self.lc.send_command("sp 0", timeout=2.0)
        except Exception:
            pass
        self._event("中转结束：" + self.stats.summary())
        return self.stats

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def run(self, duration: Optional[float] = None,
            until: Optional[Callable[[], bool]] = None) -> PcRelayStats:
        """保持运行。数据靠串口读线程推入 `on_position`，这里只做存活循环。"""
        if not self._running:
            self.start()
        t0 = time.monotonic()
        try:
            while True:
                if until is not None and until():
                    break
                if duration is not None and (time.monotonic() - t0) >= duration:
                    break
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
        return self.stats

    def poll(self) -> int:
        """处理已到达的位置行（**非阻塞**用法：外部自己跑循环时调它）。

        注意：`on_position` 是由串口读线程回调的，所以纯轮询用法其实不需要本函数；
        它在这里只为给"想自己控制节奏"的宿主一个显式入口 —— 返回累计行数。
        """
        return self.stats.lines_in


# ============================================================================
# 自检（离线，不需要硬件）
# ============================================================================

def _selftest() -> int:
    import queue

    fails: List[str] = []
    TOTAL = [0]

    def chk(name, cond, extra=""):
        TOTAL[0] += 1
        if not cond:
            fails.append("%s %s" % (name, extra))
            print("  FAIL %s %s" % (name, extra))
        else:
            print("  ok   %s" % name)

    # ---- 反向解析 ----
    chk("reverse=False", TeleopSession._resolve_reverse(False) == [1.0] * 6)
    chk("reverse=True", TeleopSession._resolve_reverse(True) == [-1.0] * 6)
    chk("reverse=list", TeleopSession._resolve_reverse([1, 0, 1, 1, 1, 1])
        == [-1.0, 1.0, -1.0, -1.0, -1.0, -1.0])

    # ---- 映射：hold 模式增量跟随 ----
    class _FakeArm:
        def __init__(self, servo):
            self.servo = list(servo)
            self.sent: List[List[float]] = []
            self.dry_run = True
            self.estop = False

        def get_status(self, **kw):
            st = Status()
            st.joints = list(self.servo)
            st.read_ok = True
            st.estop = self.estop
            return st

        def stream_joints(self, t, wait=True, timeout=None):
            self.sent.append(list(t))
            return None

        def poll_stream_acks(self, drain=True):
            return []

        def set_torque(self, *a, **k):
            return None

        def stop(self, **k):
            return None

        def move_joints(self, *a, **k):
            class R:
                ok = True
            return R()

    lead = _FakeArm([180.0] * 6)
    foll = _FakeArm([180.0] * 6)
    cfg = TeleopConfig(hz=50.0, slew_deg_per_s=1e9, wait_ack=False)
    tp = TeleopSession(lead, foll, cfg=cfg)
    tp.start()
    chk("start 建立参考", tp._lead_ref == [0.0] * 6 and tp._foll_ref == [0.0] * 6)

    # 主臂 J2 走 +10°（舵机度 190，dir=+1 → 逻辑角 +10）→ 从臂目标 +10
    lead.servo[1] = 190.0
    t1 = tp.step()
    chk("增量跟随 +10", t1 is not None and abs(t1[1] - 10.0) < 1e-6, str(t1))
    chk("J1 不动", t1 is not None and abs(t1[0]) < 1e-6)

    # ---- 限速：一帧跳 60°，被削到 slew/hz ----
    cfg2 = TeleopConfig(hz=50.0, slew_deg_per_s=120.0)   # 2.4°/帧
    tp2 = TeleopSession(_FakeArm([180.0] * 6), _FakeArm([180.0] * 6), cfg=cfg2)
    tp2.start()
    tp2.leader.servo[1] = 240.0                          # 逻辑角 +60
    t2 = tp2.step()
    chk("限速削到 2.4°", t2 is not None and abs(t2[1] - 2.4) < 0.05, str(t2[1]))
    chk("限速计数", tp2.stats.slew_clamped == 1)

    # ---- 25° 门限：限速保证永不触发 ----
    chk("60° 跳变被削到 <25°", t2 is not None and abs(t2[1]) < FW_MAX_JUMP_DEG)

    # ---- 软限位 ----
    cfg3 = TeleopConfig(hz=50.0, slew_deg_per_s=1e9,
                        soft_limits_deg=[(-5, 5)] * 6)
    tp3 = TeleopSession(_FakeArm([180.0] * 6), _FakeArm([180.0] * 6), cfg=cfg3)
    tp3.start()
    tp3.leader.servo[1] = 240.0
    t3 = tp3.step()
    chk("软限位夹到 5°", t3 is not None and abs(t3[1] - 5.0) < 1e-6, str(t3))
    chk("软限位计数", tp3.stats.soft_clamped == 1)

    # ---- gain / reverse ----
    cfg4 = TeleopConfig(hz=50.0, slew_deg_per_s=1e9, gain=2.0,
                        reverse=[0, 0, 1, 0, 0, 0])       # 只反 J3
    tp4 = TeleopSession(_FakeArm([180.0] * 6), _FakeArm([180.0] * 6), cfg=cfg4)
    tp4.start()
    tp4.leader.servo[1] = 190.0        # +10 → gain2 → +20
    tp4.leader.servo[2] = 190.0        # +10, J3 反向 → -20
    t4 = tp4.step()
    chk("gain=2", t4 is not None and abs(t4[1] - 20.0) < 1e-6, str(t4))
    chk("reverse J3", t4 is not None and abs(t4[2] + 20.0) < 1e-6)
    chk("J1 不受 gain 影响？(gain 是全轴)", t4 is not None and abs(t4[0]) < 1e-6)

    # ---- 读失败 hold ----
    class _FlakyArm(_FakeArm):
        def __init__(self, servo):
            super().__init__(servo)
            self.fail = False

        def get_status(self, **kw):
            if self.fail:
                st = Status()
                st.joints = [None] * 6
                st.read_ok = False
                return st
            return super().get_status(**kw)

    fl = _FlakyArm([180.0] * 6)
    fl2 = _FakeArm([180.0] * 6)
    tp5 = TeleopSession(fl, fl2, cfg=TeleopConfig(hz=50.0, slew_deg_per_s=1e9))
    tp5.start()
    fl.servo[1] = 190.0
    tp5.step()
    n_before = len(fl2.sent)
    fl.fail = True
    r = tp5.step()
    chk("读失败 hold 返回 None", r is None)
    chk("读失败仍重发上一帧", len(fl2.sent) == n_before + 1)
    chk("hold 计数", tp5.stats.held == 1)

    # ---- 链路丢失 → 抛异常 ----
    tp5.cfg.link_loss_timeout_s = -1.0        # 立刻判丢
    tp5._last_good_t = time.monotonic() - 5
    try:
        tp5.step()
        chk("链路丢失抛 TeleopLinkLost", False)
    except TeleopLinkLost:
        chk("链路丢失抛 TeleopLinkLost", True)

    # ---- E-Stop ----
    es = _FakeArm([180.0] * 6)
    es.estop = True
    tp6 = TeleopSession(es, _FakeArm([180.0] * 6), cfg=TeleopConfig(hz=50.0))
    tp6.start()
    try:
        tp6.step()
        chk("E-Stop 抛 TeleopAbort", False)
    except TeleopAbort:
        chk("E-Stop 抛 TeleopAbort", True)
    chk("E-Stop 后 aborted", tp6.aborted)

    # ---- hz 太低拒绝 ----
    tp7 = TeleopSession(_FakeArm([180.0] * 6), _FakeArm([180.0] * 6),
                        cfg=TeleopConfig(hz=1.0))
    try:
        tp7.start()
        chk("hz<3 拒绝启动", False)
    except TeleopError:
        chk("hz<3 拒绝启动", True)

    # ---- PcRelay 编码 ----
    class _FakeConsole:
        def __init__(self):
            self.written = []
            self.cmds = []
            self.on_position = None
            self.position_lines = 0
            self.dry_run = False

        def send_command(self, c, timeout=None, echo=True):
            self.cmds.append(c)
            class R:
                ok = True
            return R()

        def write_raw(self, s):
            self.written.append(s)

    lc, fc = _FakeConsole(), _FakeConsole()
    rel = PcRelay(lc, fc, hz=30, slew_deg_per_s=1e9, auto_gate=False)
    rel.start()
    rel._on_pos([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    chk("PcRelay 收行", rel.stats.lines_in == 1)
    chk("PcRelay 转发", rel.stats.forwarded == 1)
    chk("PcRelay 编码 #P", fc.written and
        fc.written[0] == "#P 10 20 30 40 50 60\r", repr(fc.written[:1]))
    chk("PcRelay sp 已开", "sp 30" in lc.cmds)
    rel.stop()
    chk("PcRelay sp 0", "sp 0" in lc.cmds)

    # ---- PcRelay 限速 ----
    rel2 = PcRelay(_FakeConsole(), _FakeConsole(), hz=50, slew_deg_per_s=100.0,
                   auto_gate=False)
    rel2.start()
    rel2._last = [0.0] * 6
    rel2._last_t = time.monotonic()
    rel2._on_pos([50.0] * 6)          # 一帧跳 50°，被削
    chk("PcRelay 限速生效", rel2.stats.slew_clamped == 6, str(rel2.stats.slew_clamped))

    print("")
    print("  teleop 自检：%d 项，%s" % (TOTAL[0],
                                    "全部通过" if not fails else "失败 %d" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
