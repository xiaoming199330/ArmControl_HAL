#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网口版运动控制性能实测 —— 阶跃响应 / 到位稳定性 / 重复定位精度

与老的串口工具（pid_step_test.py / overshoot_test.py 等，走 COM8 + 文本命令）
的区别：**走网口逻辑包**（MOVE_JOINT_ONE 0x14 + GET_STATUS 0x02 高频采样），
测的是当前交付链路的真实性能。

测什么
  step    逐关节阶跃响应 —— 上升时间 tr / 峰值时间 tp / 超调量 OS% /
          稳态误差 ess / 调节时间 ts(±2%)
  settle  到位稳定性 —— 到位后连续采样，算 std 与峰峰（抖动）
  repeat  重复定位精度 —— 同一目标点往返 N 次，看落点分布
  ctrl    以上全做（默认）

安全设计（务必先读）
  * 开场先读 `L` 限位表，**所有目标点先校验在限位内**，超了就跳过该关节并说明
  * **一次只动一个关节**（MOVE_JOINT_ONE，其余五轴保持）
  * 默认幅值 10°、速度 60°/s —— 保守值；`--amp` / `--vel` 可调
  * `--dry-run` 只打印计划、不发任何运动指令
  * Ctrl+C ⇒ finally 里发 STOP + 回中位
  * ⚠ 绝不发 `M a`（那是标定，会废掉量纲基准）

用法
  python tools/motion_perf_bench.py --dry-run --target-ip 192.168.7.120     # 先看计划
  python tools/motion_perf_bench.py --mode ctrl --target-ip 192.168.7.120   # 正式跑
  python tools/motion_perf_bench.py --selftest                              # 离线自检（对假臂）
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
SDK = os.path.join(HERE, "..", "sdk_package")
if SDK not in sys.path:
    sys.path.insert(0, SDK)

NUM_JOINTS = 6
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

# 固件里的运行时限位（RAM calib 为准）—— 仅作兜底；
# 运行时优先用 `L` 读到的真值，读不到才用这个。
FALLBACK_LIMITS: List[Tuple[int, int]] = [
    (63, 303), (75, 284), (76, 271), (71, 284), (12, 351), (138, 266),
]

# 每轴方向符号：+1 = 正方向增加角度更安全。仅用于自动挑选更宽松的一侧。
# 这里不猜，改成"往余量更大的一侧动"的动态策略，见 pick_direction()。


# ============================================================
# 控制指标分析
# ============================================================

@dataclass
class StepMetrics:
    """一段阶跃响应的经典控制指标。角度单位统一为**舵机度**。"""
    joint: int = 0
    start_deg: float = 0.0
    target_deg: float = 0.0
    step_deg: float = 0.0              # 目标 - 起点（带符号）
    settled_deg: Optional[float] = None
    t_rise_s: Optional[float] = None    # 10% → 90%
    t_peak_s: Optional[float] = None
    overshoot_pct: Optional[float] = None
    t_settle_s: Optional[float] = None  # 进入 ±2% 带且不再出去
    ess_deg: Optional[float] = None     # 稳态误差（末段均值 - 目标）
    samples: int = 0
    dropouts: int = 0                   # 丢读拍数（None）
    notes: List[str] = field(default_factory=list)

    def ok(self) -> bool:
        return self.t_rise_s is not None and self.settled_deg is not None


def analyze_step(joint: int, samples: Sequence[Tuple[float, float]],
                 start_deg: float, target_deg: float,
                 tail_frac: float = 0.15, band: float = 0.02) -> StepMetrics:
    """从 (t, deg) 序列算阶跃指标。

    samples: 已剔除丢读点，按时间升序，t 是相对秒数。
    tail_frac: 取末尾多大比例当"稳态"。
    band: 调节时间用的误差带（相对幅值）。
    """
    m = StepMetrics(joint=joint, start_deg=start_deg, target_deg=target_deg,
                    step_deg=target_deg - start_deg, samples=len(samples))
    if len(samples) < 5:
        m.notes.append("采样点太少（%d），无法分析" % len(samples))
        return m

    delta = target_deg - start_deg
    if abs(delta) < 1e-6:
        m.notes.append("阶跃幅值为 0")
        return m

    ts = [s[0] for s in samples]
    vs = [s[1] for s in samples]

    # 稳态值 = 末段均值
    n_tail = max(3, int(len(vs) * tail_frac))
    settled = sum(vs[-n_tail:]) / n_tail
    m.settled_deg = settled
    m.ess_deg = settled - target_deg

    # 只看向目标方向前进的部分，避免反向抖动干扰
    sign = 1.0 if delta > 0 else -1.0
    prog = [(t, (v - start_deg) * sign / abs(delta)) for t, v in samples]  # 归一化 0→1

    def cross(level: float) -> Optional[float]:
        """第一次穿过 level 的时间（线性插值）。"""
        for k in range(1, len(prog)):
            t0, p0 = prog[k - 1]
            t1, p1 = prog[k]
            if p0 < level <= p1:
                if p1 == p0:
                    return t1
                f = (level - p0) / (p1 - p0)
                return t0 + f * (t1 - t0)
        return None

    t10, t90 = cross(0.10), cross(0.90)
    if t10 is not None and t90 is not None and t90 >= t10:
        m.t_rise_s = t90 - t10
    else:
        m.notes.append("未完整走完 10%→90%（可能被限位拦下或采样不足）")

    # 峰值：取 prog 的最大值对应时刻（只在上升阶段找）
    pk_i = max(range(len(prog)), key=lambda i: prog[i][1])
    m.t_peak_s = prog[pk_i][0]
    peak_ov = prog[pk_i][1] - 1.0
    m.overshoot_pct = peak_ov * 100.0 if peak_ov > -1.0 else None

    # 调节时间：从末尾往回找，最后一次离开 ±band 的时刻
    t_settle = None
    for k in range(len(prog) - 1, -1, -1):
        if abs(prog[k][1] - 1.0) > band:
            if k + 1 < len(prog):
                t_settle = prog[k + 1][0]
            break
    else:
        t_settle = prog[0][0] if prog else None
    m.t_settle_s = t_settle
    return m


@dataclass
class SettleResult:
    """到位稳定性：到位后连续采样，看是否真的停住了。"""
    joint: int = 0
    target_deg: float = 0.0
    mean_deg: Optional[float] = None
    std_deg: Optional[float] = None
    pkpk_deg: Optional[float] = None
    samples: int = 0
    dropouts: int = 0


def analyze_settle(joint: int, target_deg: float,
                   samples: Sequence[float]) -> SettleResult:
    vals = [v for v in samples if v is not None]
    r = SettleResult(joint=joint, target_deg=target_deg,
                     samples=len(vals), dropouts=len(samples) - len(vals))
    if not vals:
        return r
    r.mean_deg = sum(vals) / len(vals)
    if len(vals) > 1:
        mu = r.mean_deg
        r.std_deg = math.sqrt(sum((v - mu) ** 2 for v in vals) / (len(vals) - 1))
    else:
        r.std_deg = 0.0
    r.pkpk_deg = max(vals) - min(vals)
    return r


@dataclass
class RepeatResult:
    """重复定位精度：同一个目标点多次到达，看落点分布。"""
    joint: int = 0
    target_deg: float = 0.0
    landings: List[float] = field(default_factory=list)
    mean_deg: Optional[float] = None
    std_deg: Optional[float] = None
    range_deg: Optional[float] = None    # max - min
    max_err_deg: Optional[float] = None  # 相对目标的绝对误差最大值


def analyze_repeat(joint: int, target_deg: float,
                   landings: Sequence[float]) -> RepeatResult:
    r = RepeatResult(joint=joint, target_deg=target_deg, landings=list(landings))
    if not landings:
        return r
    r.mean_deg = sum(landings) / len(landings)
    if len(landings) > 1:
        mu = r.mean_deg
        r.std_deg = math.sqrt(sum((v - mu) ** 2 for v in landings) / (len(landings) - 1))
    else:
        r.std_deg = 0.0
    r.range_deg = max(landings) - min(landings)
    r.max_err_deg = max(abs(v - target_deg) for v in landings)
    return r


# ============================================================
# 采样
# ============================================================

class Sampler:
    """高频轮询 GET_STATUS，记录某一轴的角度时间序列。

    ⚠ GET_STATUS 会碰舵机总线，本工程实测 6 轴连读有 2.5~8% 丢读
      （随读取频率升高）。丢读点记为 None，分析时剔除 —— **绝不兜底成 0**。
    """

    def __init__(self, arm, joint: int):
        self.arm = arm
        self.joint = joint
        self.t0 = 0.0
        self.ts: List[float] = []
        self.vals: List[Optional[float]] = []
        self.dropouts = 0
        self.exc = 0

    def reset(self) -> None:
        self.t0 = time.perf_counter()
        self.ts.clear()
        self.vals.clear()
        self.dropouts = 0
        self.exc = 0

    def poll(self) -> Optional[float]:
        try:
            st = self.arm.get_status()
        except Exception:                                    # noqa: BLE001
            self.exc += 1
            return None
        v = st.joints[self.joint] if self.joint < len(st.joints) else None
        if v is None:
            self.dropouts += 1
        self.ts.append(time.perf_counter() - self.t0)
        self.vals.append(v)
        return v

    def run_for(self, seconds: float, until_settled: bool = False,
                settle_band: float = 0.3, settle_hold: float = 0.3) -> None:
        """采 seconds 秒。until_settled=True 时，连续 settle_hold 秒落在 ±settle_band
        内就提前收工（用于等一个动作到位）。"""
        t_end = time.perf_counter() + seconds
        last_move_t = time.perf_counter()
        prev = None
        while time.perf_counter() < t_end:
            v = self.poll()
            now = time.perf_counter()
            if until_settled:
                if v is not None and prev is not None and abs(v - prev) <= settle_band:
                    if now - last_move_t >= settle_hold:
                        break
                else:
                    last_move_t = now
                prev = v

    def series(self) -> List[Tuple[float, float]]:
        """剔除 None 的 (t, deg) 序列。"""
        return [(t, v) for t, v in zip(self.ts, self.vals) if v is not None]

    def last_good(self) -> Optional[float]:
        for v in reversed(self.vals):
            if v is not None:
                return v
        return None


# ============================================================
# 主流程
# ============================================================

class MotionBench:
    def __init__(self, ip: str, port: int = 5000, *,
                 vel: float = 60.0, acc: float = 0.0,
                 verbose: bool = True, timeout: float = 2.0):
        self.uri = "net://%s:%d" % (ip, port)
        self.vel = vel
        self.acc = acc
        self.verbose = verbose
        self.timeout = timeout
        self.arm = None
        self.limits: List[Tuple[int, int]] = list(FALLBACK_LIMITS)
        self.limits_source = "fallback"

    # ---------- 日志 ----------
    def log(self, msg: str = "") -> None:
        if self.verbose:
            print(msg, flush=True)

    # ---------- 连接 ----------
    def open(self):
        from arm_sdk import open_arm
        self.arm = open_arm(self.uri, timeout=self.timeout)
        self.arm.open()
        p = self.arm.ping()
        self.log("  已连接 %s  (%s)" % (self.uri, p))
        self.read_limits()
        return self

    def close(self) -> None:
        if self.arm is not None:
            try:
                self.arm.close()
            except Exception:                                # noqa: BLE001
                pass
            self.arm = None

    def read_limits(self) -> List[Tuple[int, int]]:
        """优先用 `L`（内存里当前生效的运行时限位）。读不到就用兜底表并说明。"""
        try:
            txt = self.arm.text_exec("L")
        except Exception as e:                               # noqa: BLE001
            self.log("  ⚠ 读 `L` 失败（%s），用兜底限位表" % e)
            return self.limits
        import re
        rows = re.findall(r"\|\s*J(\d)\s*\|\s*(-?\d+)\s*\|\s*(-?\d+)\s*\|", txt)
        if len(rows) >= NUM_JOINTS:
            self.limits = [(int(r[1]), int(r[2])) for r in rows[:NUM_JOINTS]]
            self.limits_source = "L"
            self.log("  限位（来自 `L`）: " +
                     "  ".join("%s %d~%d" % (JOINT_NAMES[i], lo, hi)
                               for i, (lo, hi) in enumerate(self.limits)))
        else:
            self.log("  ⚠ `L` 解析不到 6 行，用兜底限位表")
        return self.limits

    def read_pose(self) -> List[Optional[float]]:
        st = self.arm.get_status()
        return list(st.joints)

    # ---------- 安全 ----------
    def margin(self, joint: int, deg: float) -> Tuple[float, float]:
        lo, hi = self.limits[joint]
        return deg - lo, hi - deg

    def pick_target(self, joint: int, cur: float, amp: float) -> Optional[float]:
        """在限位内挑一个目标：往**余量更大**的一侧动，且离限位至少留 amp/2 的余量。"""
        lo, hi = self.limits[joint]
        d_lo, d_hi = cur - lo, hi - cur
        for sign in ((1.0, -1.0) if d_hi >= d_lo else (-1.0, 1.0)):
            tgt = cur + sign * amp
            if lo + 2.0 <= tgt <= hi - 2.0:
                return tgt
        return None

    def precheck(self) -> Dict[str, Any]:
        st = self.arm.get_status()
        e = st.estop
        flags = st.flags
        pose = list(st.joints)
        self.log("  当前姿态: " + "  ".join(
            "%s %s" % (JOINT_NAMES[i], "--" if v is None else "%.1f" % v)
            for i, v in enumerate(pose)))
        self.log("  flags=0x%02X  estop=%s  read_ok=%s" % (flags, e, st.read_ok))
        if e:
            raise RuntimeError("急停激活中（flags bit0=1），拒绝运动测试")
        return {"pose": pose, "flags": flags, "estop": e, "read_ok": st.read_ok}

    # ---------- 单次移动（带丢读重试） ----------
    def move_one(self, joint: int, deg: float, retries: int = 4) -> bool:
        """下发单轴移动。⚠ MOVE_JOINT_ONE 在**任一轴丢读**时回 ERR_READ(7) 且不动
        —— 这是固件的安全设计（拿不准位置就不动），所以这里必须重试。"""
        last = None
        for k in range(retries):
            try:
                r = self.arm.move_joint_one(joint, deg, v=self.vel, a=self.acc)
            except Exception as e:                           # noqa: BLE001
                last = str(e)
                time.sleep(0.05)
                continue
            if getattr(r, "err", 0) == 0 or getattr(r, "ok", False):
                return True
            last = "err=%s(%s)" % (getattr(r, "err", "?"), getattr(r, "err_name", "?"))
            # 错误码 7 = 读失败（丢读）⇒ 重试有意义；其它错误立即放弃
            if getattr(r, "err", None) != 7:
                break
            time.sleep(0.05)
        self.log("    ⚠ 下发失败（已重试 %d 次）: %s" % (retries, last))
        return False

    def move_joints_center(self, deg: float = 180.0, v: float = 15.0) -> bool:
        """回中位（六轴同步）。⚠ 用的是 MOVE_JOINTS(0x10) = **舵机度**。"""
        try:
            r = self.arm.move_joints([deg] * NUM_JOINTS, v=v, a=0.0)
            return getattr(r, "err", 0) == 0 or getattr(r, "ok", False)
        except Exception as e:                               # noqa: BLE001
            self.log("    ⚠ 回中位失败: %s" % e)
            return False

    def stop(self) -> None:
        try:
            self.arm.stop()
        except Exception:                                    # noqa: BLE001
            pass

    # ---------- 三个测试 ----------
    def test_step(self, joint: int, amp: float) -> Optional[StepMetrics]:
        self.log("\n  ── %s 阶跃响应（±%.1f°）──" % (JOINT_NAMES[joint], amp))
        pose = self.read_pose()
        cur = pose[joint]
        if cur is None:
            # 丢读就重读几次
            for _ in range(5):
                time.sleep(0.08)
                cur = self.read_pose()[joint]
                if cur is not None:
                    break
        if cur is None:
            self.log("    ⚠ 起点丢读（重读 5 次仍失败），跳过 %s" % JOINT_NAMES[joint])
            return None

        tgt = self.pick_target(joint, cur, amp)
        if tgt is None:
            d_lo, d_hi = self.margin(joint, cur)
            self.log("    ⚠ 余量不足（下%.1f° 上%.1f°），跳过 %s" % (d_lo, d_hi, JOINT_NAMES[joint]))
            return None
        self.log("    %.1f° → %.1f°（舵机度）" % (cur, tgt))

        s = Sampler(self.arm, joint)
        s.reset()
        s.run_for(0.30)                                      # 阶跃前基线
        if not self.move_one(joint, tgt):
            self.log("    ⚠ 下发失败，跳过")
            return None
        s.run_for(3.0, until_settled=True)                   # 阶跃后跟踪到停
        dur = s.ts[-1] if s.ts else 0.0

        series = s.series()
        m = analyze_step(joint, series, cur, tgt)
        m.dropouts = s.dropouts
        if s.exc:
            m.notes.append("采样异常 %d 次" % s.exc)
        self.log("    采样 %.2fs  %d 点（丢读 %d）  实际 %.0f Hz"
                 % (dur, len(s.ts), s.dropouts,
                    len(s.ts) / dur if dur > 0 else 0))
        self.log("    tr=%s  tp=%s  OS=%s  ts=%s  ess=%s"
                 % (self._f(m.t_rise_s, "s", 3), self._f(m.t_peak_s, "s", 3),
                    self._f(m.overshoot_pct, "%", 2),
                    self._f(m.t_settle_s, "s", 3), self._f(m.ess_deg, "°", 3)))
        if m.notes:
            for n in m.notes:
                self.log("    · %s" % n)
        return m

    def test_settle(self, joint: int, hold_s: float = 1.2) -> Optional[SettleResult]:
        """到位稳定性：不动，就在当前位连续采，看抖不抖。"""
        self.log("\n  ── %s 到位稳定性（静止 %.1fs）──" % (JOINT_NAMES[joint], hold_s))
        s = Sampler(self.arm, joint)
        s.reset()
        s.run_for(hold_s)
        vals = s.vals
        tgt = next((v for v in reversed(vals) if v is not None), 0.0)
        r = analyze_settle(joint, tgt, vals)
        self.log("    落点均值 %.3f°  std %.4f°  峰峰 %.3f°  （%d 点，丢读 %d）"
                 % (r.mean_deg or 0, r.std_deg or 0, r.pkpk_deg or 0,
                    r.samples, r.dropouts))
        return r

    def test_repeat(self, joint: int, amp: float, rounds: int) -> Optional[RepeatResult]:
        """重复定位精度：同一起点来回 N 次，看目标点落点分布。"""
        self.log("\n  ── %s 重复定位精度（%d 轮）──" % (JOINT_NAMES[joint], rounds))
        pose = self.read_pose()
        cur = pose[joint]
        if cur is None:
            self.log("    ⚠ 起点丢读，跳过")
            return None
        tgt = self.pick_target(joint, cur, amp)
        if tgt is None:
            self.log("    ⚠ 余量不足，跳过")
            return None
        self.log("    往返 %.1f° ⇄ %.1f°" % (cur, tgt))

        landings: List[float] = []
        for k in range(rounds):
            if not self.move_one(joint, tgt):
                continue
            s = Sampler(self.arm, joint)
            s.reset()
            s.run_for(3.0, until_settled=True)
            tail = [v for v in s.vals[-12:] if v is not None]
            if tail:
                landings.append(sum(tail) / len(tail))
                self.log("    第 %d 轮落点 %.3f°" % (k + 1, landings[-1]))
            if not self.move_one(joint, cur):
                break
            time.sleep(0.35)

        r = analyze_repeat(joint, tgt, landings)
        self.log("    落点均值 %.3f°  std %.4f°  极差 %.4f°  最大偏差 %.4f°"
                 % (r.mean_deg or 0, r.std_deg or 0, r.range_deg or 0, r.max_err_deg or 0))
        return r

    # ---------- ⑨ 运动通讯性能 ----------
    def test_cmd_latency(self, joint: int, n: int = 60) -> List[Dict[str, Any]]:
        """逐命令测 ACK 往返延迟 —— 看**运动指令**比只读指令贵多少。

        ⚠ 其中 MOVE_JOINT_ONE 发的是**当前位置**（臂不动），但走完整的
          "读全场快照 → 设目标 → ACK" 路径 ⇒ 这才是运动指令的真实开销。
          发当前位置而不是别的角度，是为了**零位移**。
        """
        self.log("\n  ── 各命令 ACK 往返延迟（各 %d 次）──" % n)
        pose = self.read_pose()
        cur = pose[joint]
        if cur is None:
            for _ in range(5):
                time.sleep(0.08)
                cur = self.read_pose()[joint]
                if cur is not None:
                    break
        if cur is None:
            self.log("    ⚠ 当前角度丢读，跳过 MOVE_JOINT_ONE 一项")
            cur = 180.0

        cases = [
            ("PING",            lambda: self.arm.ping()),
            ("GET_FLAGS(0x0A)", lambda: self.arm.get_flags()),
            ("GET_STATUS(0x02)", lambda: self.arm.get_status()),
            ("GET_JOINT_DETAIL", lambda: self.arm.get_joint_detail(joint)),
            ("MOVE_JOINT_ONE*", lambda: self.arm.move_joint_one(joint, cur, v=self.vel, a=self.acc)),
            ("STOP(0x11)",      lambda: self.arm.stop()),
        ]
        out: List[Dict[str, Any]] = []
        for name, fn in cases:
            lat: List[float] = []
            errs: Dict[str, int] = {}
            for _ in range(n):
                t0 = time.perf_counter()
                try:
                    fn()
                except Exception as e:                       # noqa: BLE001
                    errs[type(e).__name__] = errs.get(type(e).__name__, 0) + 1
                    continue
                lat.append((time.perf_counter() - t0) * 1000.0)
                time.sleep(0.004)
            st = _stats(lat)
            st.update({"cmd": name, "n": n, "errors": errs})
            out.append(st)
            self.log("    %-20s p50 %7.3f ms  p90 %7.3f  p99 %7.3f  max %8.3f  n=%d%s"
                     % (name, st.get("p50", -1), st.get("p90", -1), st.get("p99", -1),
                        st.get("max", -1), len(lat),
                        ("  错误 %s" % errs) if errs else ""))
        self.log("    * MOVE_JOINT_ONE 发的是当前位置 ⇒ **臂不动**，但走完整运动指令路径")
        return out

    def test_stream(self, rates: Sequence[float], sec: float,
                    joint: int = 0, amp: float = 3.0,
                    stream: bool = False) -> List[Dict[str, Any]]:
        """STREAM_JOINTS 流控性能。

        stream=False（默认）**不发 STREAM** —— 只做一次 nx 基线读取，
        避免在不知道 Home 逻辑角的情况下误发。
        stream=True 才真发：围绕**当前逻辑角**做 ±amp 的正弦，
        需要先回中位（Home 附近），否则会被单帧 >25° 拒收。
        """
        if not stream:
            self.log("\n  ── STREAM 流控：跳过（未加 --comm-stream）──")
            self.log("     加 --comm-stream 才会真发流帧。注意 STREAM 用的是**逻辑角**")
            self.log("     （相对 Home），不是舵机度 ⇒ 没有标定数据时不要乱发。")
            return []

        self.log("\n  ── STREAM_JOINTS 流控（±%.1f° 正弦，逻辑角）──" % amp)
        out: List[Dict[str, Any]] = []
        for rate in rates:
            self.log("\n    @ %.0f Hz × %.1f s" % (rate, sec))
            base = 0.0                       # 回中位后 Home≈逻辑角 0
            n = int(rate * sec)
            period = 1.0 / rate if rate > 0 else 0.02
            sent = 0
            send_t: List[float] = []
            t_start = time.perf_counter()
            nx_before = self.read_nx()
            next_t = t_start
            for k in range(n):
                ang = base + amp * math.sin(2 * math.pi * (k / rate) / 1.5)
                deg = [base] * NUM_JOINTS
                deg[joint] = ang
                try:
                    self.arm.stream_joints(deg, wait=False)
                    sent += 1
                    send_t.append(time.perf_counter())
                except Exception as e:                       # noqa: BLE001
                    self.log("      发送异常: %s" % e)
                    break
                next_t += period
                slack = next_t - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
            elapsed = time.perf_counter() - t_start
            time.sleep(0.3)
            acks = self.arm.poll_stream_acks()
            nx_after = self.read_nx()

            ok = sum(1 for a in acks if getattr(a, "err", 0) == 0)
            err4 = sum(1 for a in acks if getattr(a, "err", None) == 4)
            r = {
                "rate_target_hz": rate, "sent": sent, "seconds": round(elapsed, 4),
                "rate_actual_hz": round(sent / elapsed, 2) if elapsed > 0 else 0,
                "run_duration_s": round(sec, 3),
                "acks": len(acks), "ack_ok": ok, "ack_err4": err4,
                "ack_loss_pct": round(100.0 * (sent - len(acks)) / sent, 3) if sent else None,
                "nx_before": nx_before, "nx_after": nx_after,
                "board_delta": (None if (nx_before is None or nx_after is None)
                                else nx_after - nx_before),
            }
            if r["board_delta"] is not None and sent:
                r["board_loss_pct"] = round(
                    100.0 * max(0, sent - r["board_delta"]) / sent, 3)
            self.log("      实际 %.1f Hz  发出 %d  收 ACK %d（ok %d / err4 %d）"
                     % (r["rate_actual_hz"], sent, len(acks), ok, err4))
            if r["nx_before"] is not None:
                self.log("      板侧 udp_cmd 差 %s（含其它命令）" % r["board_delta"])
            out.append(r)
        return out

    def read_nx(self) -> Optional[int]:
        """读臂侧 `udp cmd=` 累计值，用于测**真实单向丢包**。

        ⚠ `nx` 全文 1050~1311 B（≈23 页）⇒ 必须用 text_exec（自动翻页），
          用 text_out 只会拿到第一页、正则永远匹配不到。
        """
        import re
        try:
            txt = self.arm.text_exec("nx")
        except Exception:                                    # noqa: BLE001
            return None
        m = re.search(r"udp\s*cmd\s*=\s*(\d+)", txt)
        return int(m.group(1)) if m else None

    @staticmethod
    def _f(v, unit: str, nd: int) -> str:
        return "--" if v is None else ("%." + str(nd) + "f%s") % (v, unit)


def _pct(sorted_vals: Sequence[float], q: float) -> float:
    """线性插值分位数。q ∈ [0,1]。"""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = q * (len(sorted_vals) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(sorted_vals) - 1)
    f = pos - lo
    return sorted_vals[lo] * (1 - f) + sorted_vals[hi] * f


def _stats(vals: Sequence[float]) -> Dict[str, Any]:
    """延迟统计（毫秒）。空输入返回 {}，不编造 0。"""
    if not vals:
        return {}
    s = sorted(vals)
    return {
        "count": len(s),
        "min": round(s[0], 4),
        "p50": round(_pct(s, 0.50), 4),
        "p90": round(_pct(s, 0.90), 4),
        "p99": round(_pct(s, 0.99), 4),
        "max": round(s[-1], 4),
        "mean": round(sum(s) / len(s), 4),
    }




# ============================================================
# 命令行
# ============================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="网口版运动控制性能实测（阶跃 / 到位稳定性 / 重复定位）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("--mode", default="ctrl",
                   choices=["ctrl", "step", "settle", "repeat", "comm", "all"],
                   help="ctrl=控制性能全做(默认)；step/settle/repeat 单项；comm=运动通讯性能；all=全做")
    p.add_argument("--target-ip", default="192.168.7.120", help="机械臂 IP")
    p.add_argument("--port", type=int, default=5000, help="UDP 端口")
    p.add_argument("--joints", default="0,1,2,3,4,5", help="要测的关节（0 基，逗号分隔）")
    p.add_argument("--amp", type=float, default=10.0, help="阶跃幅值（舵机度，默认 10）")
    p.add_argument("--vel", type=float, default=60.0, help="速度（舵机度/s，默认 60）")
    p.add_argument("--acc", type=float, default=0.0, help="加速度（0=固件默认）")
    p.add_argument("--hold-sec", type=float, default=1.2, help="到位稳定性静止时长")
    p.add_argument("--rounds", type=int, default=5, help="重复定位轮数")
    # ---- 运动通讯性能 ----
    p.add_argument("--comm-n", type=int, default=60, help="comm 模式每种命令的往返次数")
    p.add_argument("--comm-stream", action="store_true",
                   help="⚠ comm 模式**真发** STREAM_JOINTS 流帧（需先 --center）。不加则只测各命令 ACK 延迟")
    p.add_argument("--comm-amp", type=float, default=3.0,
                   help="--comm-stream 时的正弦幅值（逻辑角，默认 3°）")
    p.add_argument("--comm-rates", default="50,100,200",
                   help="--comm-stream 的速率阶梯（逗号分隔，Hz）")
    p.add_argument("--comm-sec", type=float, default=3.0, help="--comm-stream 每档时长")
    p.add_argument("--center", action="store_true",
                   help="开始前先回中位 180°×6（慢速）。⚠ 会大幅运动")
    p.add_argument("--center-deg", type=float, default=180.0, help="回中位目标（舵机度）")
    p.add_argument("--dry-run", action="store_true", help="只打印计划，不发运动指令")
    p.add_argument("--no-restore", action="store_true",
                   help="结束时**不回**起点（默认会回）")
    p.add_argument("--json", default=None, help="把原始结果写到 JSON")
    p.add_argument("--selftest", action="store_true", help="离线自检（对假臂，不动真机）")
    p.add_argument("-q", "--quiet", action="store_true")
    return p


def parse_joints(txt: str) -> List[int]:
    out: List[int] = []
    for part in txt.split(","):
        part = part.strip()
        if not part:
            continue
        j = int(part)
        if not 0 <= j < NUM_JOINTS:
            raise SystemExit("关节号越界: %d（0~5）" % j)
        out.append(j)
    return out


def run_bench(args) -> int:
    joints = parse_joints(args.joints)
    bench = MotionBench(args.target_ip, args.port, vel=args.vel, acc=args.acc,
                        verbose=not args.quiet)

    print("=" * 74)
    print(" 运动控制性能实测（网口）")
    print("   目标   : %s" % bench.uri)
    print("   模式   : %s" % args.mode)
    print("   关节   : %s" % ", ".join(JOINT_NAMES[j] for j in joints))
    print("   幅值   : %.1f°    速度: %.1f°/s    加速度: %s"
          % (args.amp, args.vel, "固件默认" if args.acc == 0 else "%.1f" % args.acc))
    print("   回中位 : %s" % ("是（慢速）" if args.center else "否"))
    if args.mode in ("comm", "all"):
        print("   通讯   : 各命令 ACK 延迟 ×%d%s"
              % (args.comm_n,
                 ("  +  STREAM 流控 %s Hz（**会真动臂**）" % args.comm_rates)
                 if args.comm_stream else "  （STREAM 用 MOVE_JOINT_ONE 到当前位置代替，臂不动）"))
    print("=" * 74)

    result: Dict[str, Any] = {
        "meta": {
            "uri": bench.uri, "mode": args.mode, "amp_deg": args.amp,
            "vel_deg_s": args.vel, "acc": args.acc,
            "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "steps": [], "settles": [], "repeats": [],
        "cmd_latency": [], "stream": [],
    }

    try:
        bench.open()
    except Exception as e:                                   # noqa: BLE001
        print("✗ 连接失败: %s" % e)
        return 3

    rc = 0
    try:
        print("\n[1] 预检")
        info = bench.precheck()
        result["meta"]["start_pose"] = info["pose"]
        result["meta"]["limits"] = bench.limits
        result["meta"]["limits_source"] = bench.limits_source

        if args.dry_run:
            print("\n[2] DRY-RUN 计划（不发任何运动指令）")
            pose = info["pose"]
            for j in joints:
                cur = pose[j]
                if cur is None:
                    print("   %s  起点丢读 ⇒ 运行时会重读，读不到则跳过" % JOINT_NAMES[j])
                    continue
                tgt = bench.pick_target(j, cur, args.amp)
                lo, hi = bench.limits[j]
                if tgt is None:
                    print("   %s  当前 %.1f°  限位 %d~%d  余量 下%+.1f/上%+.1f"
                          "  ⇒ ⚠ 余量不足，会跳过"
                          % (JOINT_NAMES[j], cur, lo, hi, cur - lo, hi - cur))
                else:
                    print("   %s  当前 %6.1f°  限位 %d~%d  计划: %6.1f° ⇄ %6.1f°"
                          % (JOINT_NAMES[j], cur, lo, hi, cur, tgt))
            if args.center:
                print("   然后回中位 %.0f°×6（慢速 15°/s）" % args.center_deg)
            print("\n   未发任何指令。去掉 --dry-run 才会真动。")
            return 0

        if args.center:
            print("\n[2] 回中位 %.0f°×6（慢速）" % args.center_deg)
            if bench.move_joints_center(args.center_deg, v=15.0):
                s = Sampler(bench.arm, 0)
                s.reset()
                s.run_for(12.0, until_settled=True)
                print("   到位后姿态: " + "  ".join(
                    "%s %s" % (JOINT_NAMES[i], "--" if v is None else "%.1f" % v)
                    for i, v in enumerate(bench.read_pose())))
            else:
                print("   ⚠ 回中位下发失败，仍然继续（每轴会各自校验限位）")

        # 记下"测试开始前"的姿态 —— 收尾要回到这里（不是上电时的姿态）
        origin = bench.read_pose()
        result["meta"]["origin_pose"] = origin

        seq = 0
        run_step = args.mode in ("ctrl", "step", "all")
        run_settle = args.mode in ("ctrl", "settle", "all")
        run_repeat = args.mode in ("ctrl", "repeat", "all")
        run_comm = args.mode in ("comm", "all")
        if run_step:
            print("\n[%d] 阶跃响应" % (seq + 3))
            for j in joints:
                m = bench.test_step(j, args.amp)
                if m is not None:
                    result["steps"].append(asdict(m))
        if run_settle:
            print("\n[%d] 到位稳定性" % (seq + 4))
            for j in joints:
                r = bench.test_settle(j, args.hold_sec)
                if r is not None:
                    result["settles"].append(asdict(r))
        if run_repeat:
            print("\n[%d] 重复定位精度" % (seq + 5))
            for j in joints:
                r = bench.test_repeat(j, args.amp, args.rounds)
                if r is not None:
                    result["repeats"].append(asdict(r))

        # ---- 运动通讯性能 ----
        if run_comm:
            print("\n[%d] 运动通讯性能" % (seq + 6))
            result["cmd_latency"] = bench.test_cmd_latency(joints[0], args.comm_n)
            rates = [float(x) for x in args.comm_rates.split(",") if x.strip()]
            result["stream"] = bench.test_stream(
                rates, args.comm_sec, joint=joints[0],
                amp=args.comm_amp, stream=args.comm_stream)

        # ---- 收尾：把关节放回测试开始前的位置 ----
        if not args.no_restore:
            print("\n[收尾] 回起点（测试开始前的姿态）")
            moved = 0
            for j in joints:
                want = origin[j] if j < len(origin) else None
                if want is None:
                    print("   %s 起点未记录（丢读），跳过" % JOINT_NAMES[j])
                    continue
                now = bench.read_pose()[j]
                if now is not None and abs(now - want) < 0.2:
                    continue                      # 已经在起点，不用动
                if bench.move_one(j, want):
                    moved += 1
                    print("   %s → %.1f°" % (JOINT_NAMES[j], want))
            if moved:
                time.sleep(0.6)
                print("   收尾后姿态: " + "  ".join(
                    "%s %s" % (JOINT_NAMES[i], "--" if v is None else "%.1f" % v)
                    for i, v in enumerate(bench.read_pose())))
            else:
                print("   无需移动（都在起点）")

    except KeyboardInterrupt:
        print("\n⚠ 收到 Ctrl+C —— 发 STOP 并收尾")
        bench.stop()
        rc = 130
    except Exception as e:                                   # noqa: BLE001
        print("\n✗ 异常: %s" % e)
        bench.stop()
        rc = 1

    result["meta"]["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    print("\n" + "=" * 74)
    print(" 汇总")
    print("=" * 74)
    print_summary(result)

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print("\n原始数据已写: %s" % args.json)

    bench.close()
    return rc


def print_summary(result: Dict[str, Any]) -> None:
    steps = result.get("steps") or []
    if steps:
        print("\n  阶跃响应")
        print("  %-4s %8s %8s %8s %8s %8s %8s %6s"
              % ("轴", "起点°", "目标°", "tr(s)", "tp(s)", "OS(%)", "ts(s)", "ess°"))
        for m in steps:
            print("  %-4s %8.2f %8.2f %8s %8s %8s %8s %6s"
                  % (JOINT_NAMES[m["joint"]], m["start_deg"], m["target_deg"],
                     f3(m["t_rise_s"]), f3(m["t_peak_s"]),
                     f2(m["overshoot_pct"]), f3(m["t_settle_s"]), f3(m["ess_deg"])))
        os_ = [m["overshoot_pct"] for m in steps if m["overshoot_pct"] is not None]
        tr = [m["t_rise_s"] for m in steps if m["t_rise_s"] is not None]
        if os_:
            print("  → 超调量 max %.2f%%  平均 %.2f%%" % (max(os_), sum(os_) / len(os_)))
        if tr:
            print("  → 上升时间 max %.3fs  平均 %.3fs" % (max(tr), sum(tr) / len(tr)))
    settles = result.get("settles") or []
    if settles:
        print("\n  到位稳定性（静止时的抖动）")
        print("  %-4s %10s %10s %10s" % ("轴", "均值°", "std°", "峰峰°"))
        for r in settles:
            print("  %-4s %10.3f %10.4f %10.3f"
                  % (JOINT_NAMES[r["joint"]], r["mean_deg"] or 0,
                     r["std_deg"] or 0, r["pkpk_deg"] or 0))
    reps = result.get("repeats") or []
    if reps:
        print("\n  重复定位精度")
        print("  %-4s %10s %10s %10s %10s %4s"
              % ("轴", "目标°", "均值°", "std°", "极差°", "轮数"))
        for r in reps:
            print("  %-4s %10.3f %10.3f %10.4f %10.4f %4d"
                  % (JOINT_NAMES[r["joint"]], r["target_deg"], r["mean_deg"] or 0,
                     r["std_deg"] or 0, r["range_deg"] or 0, len(r["landings"])))
    lats = result.get("cmd_latency") or []
    if lats:
        print("\n  运动通讯：各命令 ACK 往返延迟（ms）")
        print("  %-20s %8s %8s %8s %8s %6s" % ("命令", "p50", "p90", "p99", "max", "样本"))
        for s in lats:
            if not s.get("count"):
                print("  %-20s %8s   （全部失败 %s）" % (s.get("cmd", "?"), "--", s.get("errors")))
                continue
            print("  %-20s %8.3f %8.3f %8.3f %8.3f %6d"
                  % (s["cmd"], s["p50"], s["p90"], s["p99"], s["max"], s["count"]))
    streams = result.get("stream") or []
    if streams:
        print("\n  运动通讯：STREAM_JOINTS 流控")
        print("  %-10s %10s %8s %8s %10s %12s"
              % ("目标Hz", "实际Hz", "发出", "收ACK", "ACK丢失%", "板侧丢包%"))
        for r in streams:
            print("  %-10.0f %10.2f %8d %8d %10s %12s"
                  % (r["rate_target_hz"], r["rate_actual_hz"], r["sent"], r["acks"],
                     f2(r.get("ack_loss_pct")), f2(r.get("board_loss_pct"))))


def f2(v):
    return "--" if v is None else "%.2f" % v


def f3(v):
    return "--" if v is None else "%.3f" % v


# ============================================================
# 离线自检（用假臂，不动真机）
# ============================================================

def selftest() -> int:
    print("=" * 74)
    print(" motion_perf_bench 离线自检（对假臂 mock，不动真机）")
    print("=" * 74)
    n_ok = n_bad = 0

    def chk(tag: str, cond: bool, detail: str = "") -> None:
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print("  PASS  %s%s" % (tag, ("  " + detail) if detail else ""))
        else:
            n_bad += 1
            print("  FAIL  %s  %s" % (tag, detail))

    # ---- ① 阶跃分析：构造一条已知指标的曲线 ----
    # 一阶惯性 + 10% 超调（二阶欠阻尼），fs=200Hz
    def synth(zeta=0.55, wn=18.0, start=180.0, step=10.0, n=600, fs=200.0):
        out = []
        for k in range(n):
            t = k / fs
            if t <= 0:
                out.append((0.0, start))
                continue
            wd = wn * math.sqrt(1 - zeta ** 2)
            env = math.exp(-zeta * wn * t)
            y = 1 - env * (math.cos(wd * t) + (zeta / math.sqrt(1 - zeta ** 2)) * math.sin(wd * t))
            out.append((t, start + step * y))
        return out

    for zeta, expect_os_lo, expect_os_hi in ((0.9, -0.5, 3.0), (0.5, 10.0, 20.0), (0.3, 30.0, 45.0)):
        m = analyze_step(0, synth(zeta=zeta), 180.0, 190.0)
        chk("阶跃分析 ζ=%.1f 超调量在理论值附近" % zeta,
            m.overshoot_pct is not None and expect_os_lo <= m.overshoot_pct <= expect_os_hi,
            "OS=%.2f%%（期望 %.1f~%.1f%%）" % (m.overshoot_pct or -1, expect_os_lo, expect_os_hi))
        chk("阶跃分析 ζ=%.1f 上升时间合理" % zeta,
            m.t_rise_s is not None and 0.0 < m.t_rise_s < 0.5,
            "tr=%.4fs" % (m.t_rise_s or -1))
        chk("阶跃分析 ζ=%.1f 稳态误差接近 0" % zeta,
            m.ess_deg is not None and abs(m.ess_deg) < 0.05,
            "ess=%.4f°" % (m.ess_deg or 0))

    # ---- ② 分析函数的边界 ----
    m = analyze_step(0, [(0.0, 180.0)], 180.0, 190.0)
    chk("采样点太少时不硬算（notes 说明原因）", not m.ok() and m.notes, str(m.notes))
    m = analyze_step(0, [(i * 0.01, 180.0) for i in range(50)], 180.0, 180.0)
    chk("阶跃幅值为 0 时不硬算", not m.ok() and m.notes, str(m.notes))

    # ---- ③ 稳定性分析 ----
    r = analyze_settle(0, 180.0, [180.0] * 50)
    chk("完全静止 ⇒ std=0 峰峰=0",
        r.std_deg == 0.0 and r.pkpk_deg == 0.0 and r.samples == 50, "")
    r = analyze_settle(0, 180.0, [180.2, None, 179.8, 180.0, None])
    chk("丢弃 None 后仍算对", r.samples == 3 and r.dropouts == 2, "")
    r = analyze_settle(0, 180.0, [None, None])
    chk("全丢读 ⇒ 不返回伪造的 0", r.mean_deg is None and r.samples == 0, "")

    # ---- ④ 重复定位 ----
    r = analyze_repeat(0, 190.0, [190.0, 190.1, 189.9, 190.0])
    chk("重复定位：极差 0.2°", r.range_deg is not None and abs(r.range_deg - 0.2) < 1e-9, "")
    chk("重复定位：最大偏差 0.1°", abs((r.max_err_deg or 0) - 0.1) < 1e-9, "")

    # ---- ⑤ 安全逻辑：目标挑选 ----
    b = MotionBench("127.0.0.1", verbose=False)
    b.limits = list(FALLBACK_LIMITS)
    # J2 当前 76.7，限位 75~284 ⇒ 必须往**上**（余量大的一侧）动
    t = b.pick_target(1, 76.7, 10.0)
    chk("J2 贴下限时往上动（不撞限位）", t is not None and t > 76.7, "target=%s" % t)
    # J3 当前 272.5，限位 76~271 ⇒ 已在限位外；正确行为是**往限位内**动，不继续往外
    t = b.pick_target(2, 272.5, 10.0)
    chk("J3 已越限时往限位**内**动（不继续往外）",
        t is not None and t < 272.5 and b.limits[2][0] <= t <= b.limits[2][1],
        "target=%s（须落在 76~271 且 < 272.5）" % t)
    # 余量充足时正常
    t = b.pick_target(4, 180.0, 10.0)
    chk("J5 余量充足时能出目标", t in (170.0, 190.0), "target=%s" % t)
    # 当前位两侧余量**都**不够时 ⇒ 必须拒绝
    b.limits[1] = (75, 80)          # 窄限位
    t = b.pick_target(1, 77.5, 50.0)
    chk("两侧余量都不足时拒绝（不冒险）", t is None, "target=%s" % t)
    b.limits = list(FALLBACK_LIMITS)

    # ---- ⑥ 限位解析 ----
    import re
    fake_L = """
  | J1 |   63 |  303 |  173 |
  | J2 |   75 |  284 |    0 |
  | J3 |   76 |  271 |    0 |
  | J4 |   71 |  284 |  201 |
  | J5 |   12 |  351 |  180 |
  | J6 |  138 |  266 |  145 |
"""
    rows = re.findall(r"\|\s*J(\d)\s*\|\s*(-?\d+)\s*\|\s*(-?\d+)\s*\|", fake_L)
    chk("`L` 输出解析出 6 行限位", len(rows) == 6, "%d 行" % len(rows))
    chk("J5 限位解析为 12~351",
        len(rows) >= 5 and rows[4][1:] == ("12", "351"),
        str(rows[4] if len(rows) > 4 else None))

    # ---- ⑦ 端到端：对假臂跑一遍（验证管线不崩） ----
    try:
        import threading
        from arm_sdk.mock_server import MockArmServer
        mock = MockArmServer(port=0)
        mock.bind()
        port = mock.sock.getsockname()[1]
        th = threading.Thread(target=mock.serve_forever, daemon=True)
        th.start()
        time.sleep(0.3)

        b2 = MotionBench("127.0.0.1", port, vel=60.0, verbose=False, timeout=1.0)
        b2.open()
        info = b2.precheck()
        chk("对假臂：预检通过（无急停）", info["estop"] is False, "")
        chk("对假臂：解析到 6 条限位", len(b2.limits) == 6, str(b2.limits_source))
        m = b2.test_step(0, 10.0)
        chk("对假臂：阶跃测试跑完不崩", m is not None, "samples=%s" % (m.samples if m else -1))
        r = b2.test_settle(0, 0.4)
        chk("对假臂：稳定性测试跑完不崩", r is not None and r.samples > 0,
            "samples=%s" % (r.samples if r else -1))
        lat = b2.test_cmd_latency(0, 6)
        chk("对假臂：各命令延迟测量跑完不崩", len(lat) >= 5, "%d 种命令" % len(lat))
        mv = next((x for x in lat if x["cmd"].startswith("MOVE_JOINT_ONE")), None)
        chk("对假臂：MOVE_JOINT_ONE 有延迟样本",
            mv is not None and mv.get("count", 0) > 0, str(mv and mv.get("count")))
        st = b2.test_stream([50.0], 0.3, stream=False)
        chk("STREAM 默认不发（stream=False 时返回空）", st == [], str(st))
        b2.close()
        mock.close()
    except Exception as e:                                   # noqa: BLE001
        chk("对假臂跑端到端", False, "%s: %s" % (type(e).__name__, e))

    print("\n" + "=" * 74)
    print("  自检结果: %d 通过 / %d 失败" % (n_ok, n_bad))
    print("=" * 74)
    return 0 if n_bad == 0 else 1


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    return run_bench(args)


if __name__ == "__main__":
    sys.exit(main())
