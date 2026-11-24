# -*- coding: utf-8 -*-
"""
teach.py — 主从臂遥操 **示教**（遥操中录制 / 回放 / 存盘 / 导出）

两种录制方式
------------
1. **PC 侧（本模块，推荐）**
   `TeleopTeachSession` 在遥操循环里逐帧落盘，**没有 80 s / 4000 点上限**
   （那是固件 Flash 录音的限制），存 JSON / JSONL / CSV / NPZ，可直接给
   LeRobot / PyTorch / MATLAB 吃。
2. **固件侧（`FirmwareTeach`）**
   从臂 `teach rec` 跟录（50 Hz，4000 点 ≈ 80 s，存 Flash、**阻塞 1~3 s**、
   有"三种丢录音"的坑）。适合不带 PC 的独立演示。

固件侧的三个"丢录音"陷阱（见 `docs/遥操中录制示教轨迹_2026-09-16.md`）
------------------------------------------------------------------
* `rec clear` / 直接断电 / 再进一次 `teach` 都会把**未存盘**的录音清掉。
* `save` 之前必须确认 `rec list` 里的点数对得上。
* `teach rec` 是**只读跟录**（不碰舵机），`teach`（无参数）会**松力矩**进手拖模式。

单位约定
--------
轨迹里一律存 **逻辑角（rel-home，度）** —— 与 `STREAM_JOINTS` / `#P` 同量纲，
回放到任何一台**已校准过同一物理基准**的臂上都对。要换回舵机度用
`JointCalib.to_servo()`。
"""

from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from .client import ArmClient, ArmCommandError
from .console import ArmSDK
from .protocol import NUM_JOINTS, ErrCode, JointCalib
from .teleop import (FW_MAX_JUMP_DEG, FW_MIN_HZ, TeleopConfig, TeleopSession)

__all__ = [
    "Trajectory", "TeachRecorder", "TeleopTeachSession", "PlaybackSession",
    "PlaybackStats", "FirmwareTeach", "TeachError",
]

LOGICAL = "logical_deg"
SERVO = "servo_deg"
JOINT_NAMES = ["J%d" % i for i in range(1, NUM_JOINTS + 1)]


class TeachError(RuntimeError):
    """示教 / 轨迹相关错误。"""


# ============================================================================
# 轨迹容器
# ============================================================================

@dataclass
class Trajectory:
    """一条关节轨迹。`frames` 是 `(t_seconds, [6 × 逻辑角])`。

    t 从 0 起（相对时间）；绝对开始时刻存 `created`（unix 秒）。
    """

    name: str = "traj"
    hz: float = 50.0
    frames: List[Tuple[float, List[float]]] = field(default_factory=list)
    joint_names: List[str] = field(default_factory=lambda: list(JOINT_NAMES))
    units: str = LOGICAL
    source: str = "unknown"
    created: float = field(default_factory=time.time)
    meta: Dict[str, object] = field(default_factory=dict)

    # ---------------------------------------------------------------- 基本
    def __len__(self) -> int:
        return len(self.frames)

    def __iter__(self):
        return iter(self.frames)

    @property
    def duration(self) -> float:
        return self.frames[-1][0] if self.frames else 0.0

    @property
    def point_count(self) -> int:
        return len(self.frames)

    @property
    def rate(self) -> float:
        d = self.duration
        return (len(self.frames) - 1) / d if d > 0 and len(self.frames) > 1 else self.hz

    def append(self, t: float, q: Sequence[float]) -> None:
        if len(q) != NUM_JOINTS:
            raise TeachError("每帧必须 6 个关节值，收到 %d" % len(q))
        self.frames.append((float(t), [float(v) for v in q]))

    def joints_matrix(self) -> List[List[float]]:
        """转成 `[[J1 全程], [J2 全程], ...]`（画图/导出用）。"""
        m: List[List[float]] = [[] for _ in range(NUM_JOINTS)]
        for _t, q in self.frames:
            for j in range(NUM_JOINTS):
                m[j].append(q[j])
        return m

    def times(self) -> List[float]:
        return [t for t, _q in self.frames]

    def start_pose(self) -> Optional[List[float]]:
        return list(self.frames[0][1]) if self.frames else None

    def end_pose(self) -> Optional[List[float]]:
        return list(self.frames[-1][1]) if self.frames else None

    # ---------------------------------------------------------------- 变换
    def resample(self, hz: float, duration: Optional[float] = None) -> "Trajectory":
        """线性插值重采样到固定 `hz`（回放前统一节拍用）。"""
        if not self.frames:
            return Trajectory(name=self.name, hz=hz, units=self.units,
                              source=self.source, meta=dict(self.meta))
        if hz <= 0:
            raise TeachError("hz 必须 > 0")
        total = self.duration if duration is None else float(duration)
        out = Trajectory(name=self.name + "@%gHz" % hz, hz=hz, units=self.units,
                         source=self.source, created=self.created,
                         meta=dict(self.meta))
        n = max(2, int(round(total * hz)) + 1)
        src = self.frames
        idx = 0
        for k in range(n):
            t = k / hz
            while idx + 2 < len(src) and src[idx + 1][0] < t:
                idx += 1
            if idx + 1 >= len(src):
                out.append(t, src[-1][1])
                continue
            t0, q0 = src[idx]
            t1, q1 = src[idx + 1]
            if t1 <= t0:
                out.append(t, q0)
                continue
            a = min(1.0, max(0.0, (t - t0) / (t1 - t0)))
            out.append(t, [q0[j] + (q1[j] - q0[j]) * a for j in range(NUM_JOINTS)])
        return out

    def smooth(self, window: int = 3) -> "Trajectory":
        """移动平均平滑（`window` 奇数，3~7 常用）。

        边缘 `half` 帧**原样保留** —— 保证首末点不变（回放起停不会多出一个跳变）。
        """
        if window < 3 or not self.frames:
            return self.copy()
        if window % 2 == 0:
            window += 1
        half = window // 2
        qs = self.joints_matrix()
        out = Trajectory(name=self.name + "_smooth", hz=self.hz, units=self.units,
                         source=self.source, created=self.created, meta=dict(self.meta))
        n = len(self.frames)
        for i in range(n):
            if i < half or i >= n - half:
                out.append(self.frames[i][0], self.frames[i][1])
                continue
            lo, hi = i - half, i + half + 1
            q = [sum(qs[j][lo:hi]) / (hi - lo) for j in range(NUM_JOINTS)]
            out.append(self.frames[i][0], q)
        return out

    def slice(self, t0: float = 0.0, t1: Optional[float] = None) -> "Trajectory":
        """按时间裁一段，并把时间轴平移到 0。"""
        t1 = self.duration if t1 is None else t1
        out = Trajectory(name=self.name + "_cut", hz=self.hz, units=self.units,
                         source=self.source, created=self.created, meta=dict(self.meta))
        for t, q in self.frames:
            if t0 <= t <= t1:
                out.append(t - t0, q)
        return out

    def scale_time(self, factor: float) -> "Trajectory":
        """时间轴缩放（<1 加速，>1 减速）。"""
        if factor <= 0:
            raise TeachError("factor 必须 > 0")
        out = Trajectory(name=self.name, hz=self.hz / factor, units=self.units,
                         source=self.source, created=self.created, meta=dict(self.meta))
        for t, q in self.frames:
            out.append(t * factor, q)
        return out

    def copy(self) -> "Trajectory":
        return Trajectory(name=self.name, hz=self.hz,
                          frames=[(t, list(q)) for t, q in self.frames],
                          joint_names=list(self.joint_names), units=self.units,
                          source=self.source, created=self.created,
                          meta=dict(self.meta))

    # ---------------------------------------------------------------- 健康检查
    def max_step(self) -> List[float]:
        """每轴全程最大单帧步长（度）。>25° 的轴回放会被固件整帧拒收。"""
        out = [0.0] * NUM_JOINTS
        for i in range(1, len(self.frames)):
            q0 = self.frames[i - 1][1]
            q1 = self.frames[i][1]
            for j in range(NUM_JOINTS):
                out[j] = max(out[j], abs(q1[j] - q0[j]))
        return out

    def check_limits(self, limits: Sequence[Tuple[float, float]]
                     ) -> List[Tuple[int, float]]:
        """检查是否越出软限位。返回 `[(轴0基, 越界值), ...]`。"""
        bad: List[Tuple[int, float]] = []
        for _t, q in self.frames:
            for j in range(NUM_JOINTS):
                if j >= len(limits):
                    break
                lo, hi = limits[j]
                if q[j] < lo or q[j] > hi:
                    bad.append((j, q[j]))
        return bad

    def validate(self, limits: Optional[Sequence[Tuple[float, float]]] = None
                 ) -> Dict[str, object]:
        """回放前自检：点数 / 时长 / 采样率 / 单帧跳变 / 越界。UI 直接展示。"""
        ms = self.max_step()
        warnings: List[str] = []
        if len(self.frames) < 2:
            warnings.append("点数不足 2 —— 没有可回放的运动")
        if self.rate > 200:
            warnings.append("采样率 %.0f Hz 过高（>200 Hz 无意义，固件只吃 20~50 Hz）"
                            % self.rate)
        if self.rate and self.rate < FW_MIN_HZ:
            warnings.append("采样率 %.2f Hz 过低 —— 回放会断流（>2 Hz，建议 ≥20 Hz）"
                            % self.rate)
        over = [("J%d" % (j + 1), round(ms[j], 1))
                for j in range(NUM_JOINTS) if ms[j] > FW_MAX_JUMP_DEG]
        if over:
            warnings.append("单帧跳变 >25° 的轴 %s —— 回放会被整帧拒收（先 resample "
                            "或 smooth）" % over)
        if limits is not None:
            bad = self.check_limits(limits)
            if bad:
                warnings.append("越出软限位 %d 处，例：J%d=%.1f"
                                % (len(bad), bad[0][0] + 1, bad[0][1]))
        return {"points": len(self.frames), "duration_s": round(self.duration, 3),
                "rate_hz": round(self.rate, 2), "max_step_deg": [round(v, 2) for v in ms],
                "warnings": warnings, "ok": not warnings}

    # ---------------------------------------------------------------- 序列化
    def to_dict(self) -> Dict[str, object]:
        return {"format": "arm_trajectory/1", "name": self.name, "hz": self.hz,
                "units": self.units, "source": self.source, "created": self.created,
                "joint_names": list(self.joint_names),
                "points": len(self.frames),
                "duration_s": round(self.duration, 6),
                "meta": dict(self.meta),
                "frames": [[round(t, 6), [round(v, 6) for v in q]]
                           for t, q in self.frames]}

    @classmethod
    def from_dict(cls, d: Dict[str, object]) -> "Trajectory":
        t = cls(name=str(d.get("name", "traj")), hz=float(d.get("hz", 50.0)),
                units=str(d.get("units", LOGICAL)),
                source=str(d.get("source", "file")),
                created=float(d.get("created", time.time())),
                joint_names=list(d.get("joint_names") or JOINT_NAMES),
                meta=dict(d.get("meta") or {}))
        for row in d.get("frames") or []:
            t.append(float(row[0]), [float(x) for x in row[1]])
        return t

    def save(self, path: str) -> str:
        """按扩展名存：`.json` / `.jsonl` / `.csv` / `.npz`（不写扩展名 = .json）。"""
        ext = os.path.splitext(path)[1].lower()
        if ext == ".jsonl":
            return self.save_jsonl(path)
        if ext == ".csv":
            return self.save_csv(path)
        if ext == ".npz":
            return self.save_npz(path)
        return self.save_json(path)

    @classmethod
    def load(cls, path: str) -> "Trajectory":
        ext = os.path.splitext(path)[1].lower()
        if ext == ".jsonl":
            return cls.load_jsonl(path)
        if ext == ".csv":
            return cls.load_csv(path)
        if ext == ".npz":
            return cls.load_npz(path)
        return cls.load_json(path)

    # ---- JSON（单文件，带元数据；推荐）----
    def save_json(self, path: str) -> str:
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=1)
        return path

    @classmethod
    def load_json(cls, path: str) -> "Trajectory":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    # ---- JSONL（一行一帧，追加友好，超大轨迹不占内存峰值）----
    def save_jsonl(self, path: str) -> str:
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        head = {k: v for k, v in self.to_dict().items() if k != "frames"}
        with open(path, "w", encoding="utf-8") as f:
            f.write(json.dumps({"__header__": head}, ensure_ascii=False) + "\n")
            for t, q in self.frames:
                f.write(json.dumps([round(t, 6), [round(v, 6) for v in q]]) + "\n")
        return path

    @classmethod
    def load_jsonl(cls, path: str) -> "Trajectory":
        t = cls()
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                row = json.loads(line)
                if isinstance(row, dict):
                    hdr = row.get("__header__")
                    if isinstance(hdr, dict):
                        t = cls.from_dict({"frames": [], **hdr})
                    continue
                t.append(float(row[0]), [float(x) for x in row[1]])
        return t

    # ---- CSV（最通用：t,J1..J6，表头一行）----
    def save_csv(self, path: str) -> str:
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s"] + list(self.joint_names))
            for t, q in self.frames:
                w.writerow(["%.6f" % t] + ["%.6f" % v for v in q])
        return path

    @classmethod
    def load_csv(cls, path: str) -> "Trajectory":
        t = cls(name=os.path.basename(path), source="csv")
        with open(path, "r", newline="", encoding="utf-8") as f:
            r = csv.reader(f)
            header = next(r, None)
            if header and header[0].strip().lower() in ("t", "t_s", "time", "time_s"):
                t.joint_names = [h.strip() for h in header[1:]] or list(JOINT_NAMES)
            else:
                if header:
                    t.append(0.0, [float(x) for x in header[:NUM_JOINTS]])
            for row in r:
                if len(row) < NUM_JOINTS + 1:
                    continue
                t.append(float(row[0]), [float(x) for x in row[1:NUM_JOINTS + 1]])
        if len(t) > 1:
            d = t.duration
            t.hz = (len(t) - 1) / d if d > 0 else 50.0
        return t

    # ---- NPZ（给 numpy / LeRobot 用；需 numpy）----
    def save_npz(self, path: str) -> str:
        try:
            import numpy as np
        except ImportError as e:
            raise TeachError("存 .npz 需要 numpy（pip install numpy）") from e
        q = np.asarray([f[1] for f in self.frames], dtype="float32")
        ts = np.asarray([f[0] for f in self.frames], dtype="float64")
        np.savez_compressed(path, t=ts, q=q, hz=self.hz, units=self.units,
                            joint_names=np.asarray(self.joint_names),
                            name=self.name)
        return path

    @classmethod
    def load_npz(cls, path: str) -> "Trajectory":
        try:
            import numpy as np
        except ImportError as e:
            raise TeachError("读 .npz 需要 numpy（pip install numpy）") from e
        z = np.load(path, allow_pickle=True)
        t = cls(name=str(z["name"]) if "name" in z else os.path.basename(path),
                hz=float(z["hz"]) if "hz" in z else 50.0,
                units=str(z["units"]) if "units" in z else LOGICAL,
                source="npz")
        ts, q = z["t"], z["q"]
        for i in range(len(ts)):
            t.append(float(ts[i]), [float(v) for v in q[i]])
        return t

    # ---- 展示 ----
    def summary(self) -> str:
        v = self.validate()
        return ("%s: %d 点 / %.2fs @ %.1f Hz | 单帧最大跳变 %s%s"
                % (self.name, v["points"], v["duration_s"], v["rate_hz"],
                   v["max_step_deg"],
                   "" if v["ok"] else " | ⚠ " + "; ".join(v["warnings"])))

    def __repr__(self) -> str:
        return "Trajectory(%s, %d pts, %.2fs, %s)" % (
            self.name, len(self.frames), self.duration, self.units)


# ============================================================================
# 录制器
# ============================================================================

class TeachRecorder:
    """从遥操帧回调里攒轨迹。线程安全（主循环写、UI 读）。

    最常用：挂在 `TeleopSession(on_frame=rec)` 上，录**从臂被命令到的**轨迹
    （也就是从臂实际要走的轨迹 —— 示教要的正是这个）。

        rec = TeachRecorder(name="demo1", mode="follower")
        tp = TeleopSession(lead, foll, on_frame=rec)
        tp.run(duration=30)
        rec.trajectory().save("demo1.json")

    参数
    ----
    mode : `"follower"` 录发往从臂的逻辑角（默认）；`"leader"` 录主臂逻辑角。
    hz   : 标称采样率（写进轨迹头；真实节拍由调用方决定）。
    max_points : 点数上限（0 = 无限）。到顶后停止接收并记 `dropped`。
    min_interval_s : 最小帧间隔，防重复帧刷爆（0 = 不限）。
    """

    def __init__(self, name: str = "traj", mode: str = "follower", hz: float = 50.0,
                 max_points: int = 0, min_interval_s: float = 0.0,
                 meta: Optional[Dict[str, object]] = None):
        if mode not in ("follower", "leader"):
            raise ValueError("mode 只能是 follower / leader")
        self.name = name
        self.mode = mode
        self.hz = hz
        self.max_points = int(max_points)
        self.min_interval = float(min_interval_s)
        self.meta = dict(meta or {})

        self._lock = threading.RLock()
        self._t0: Optional[float] = None
        self._last_t: Optional[float] = None
        self._frames: List[Tuple[float, List[float]]] = []
        self._over = False
        self.dropped = 0
        self.recording = False

    # ---- 生命周期 ----
    def start(self) -> "TeachRecorder":
        with self._lock:
            self._t0 = None
            self._last_t = None
            self._frames = []
            self._over = False
            self.dropped = 0
            self.recording = True
        return self

    def stop(self) -> "TeachRecorder":
        with self._lock:
            self.recording = False
        return self

    def clear(self) -> None:
        with self._lock:
            self._frames = []
            self._t0 = None
            self._last_t = None
            self._over = False
            self.dropped = 0

    # ---- 采集 ----
    def record(self, q: Sequence[float], t: Optional[float] = None) -> bool:
        """显式记一帧（t 缺省用单调时钟相对 start）。返回是否记下。"""
        now = time.monotonic() if t is None else float(t)
        with self._lock:
            if not self.recording or self._over:
                return False
            if self._t0 is None:
                self._t0 = now
            rel = now - self._t0
            if self._last_t is not None and self.min_interval > 0 and \
                    (rel - self._last_t) < self.min_interval:
                self.dropped += 1
                return False
            if self.max_points and len(self._frames) >= self.max_points:
                self._over = True
                return False
            self._frames.append((rel, [float(v) for v in q]))
            self._last_t = rel
            return True

    def __call__(self, lead_logical: Sequence[float],
                 target_logical: Sequence[float]) -> None:
        """直接当 `TeleopSession(on_frame=...)` 用。"""
        self.record(lead_logical if self.mode == "leader" else target_logical)

    # ---- 取结果 ----
    @property
    def count(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def duration(self) -> float:
        with self._lock:
            return self._frames[-1][0] if self._frames else 0.0

    def trajectory(self) -> Trajectory:
        with self._lock:
            t = Trajectory(name=self.name, hz=self.hz, units=LOGICAL,
                           source="teleop:%s" % self.mode, meta=dict(self.meta))
            t.frames = [(f[0], list(f[1])) for f in self._frames]
            return t

    def save(self, path: str) -> str:
        return self.trajectory().save(path)

    def summary(self) -> str:
        return "录制 %d 点 / %.2fs（丢 %d）" % (self.count, self.duration, self.dropped)


# ============================================================================
# 遥操 + 示教（一把梭）
# ============================================================================

class TeleopTeachSession:
    """一边遥操一边录，**无固件的 80 s 限制**。

        from arm_sdk import open_arm, TeleopTeachSession, Trajectory

        lead = open_arm("net://192.168.7.121:5000")
        foll = open_arm("net://192.168.7.120:5000")
        with TeleopTeachSession(lead, foll, out="demo1.json", hz=50) as ts:
            ts.record(duration=60)                 # 拖 60 秒
        Trajectory.load("demo1.json").validate()

    也可以不立刻录（`arm="r"` 热键那套的 SDK 版）:
        ts.run(duration=60)          # 先练一遍，不录
        ts.record(duration=60)       # 再录
    """

    def __init__(self, leader: ArmClient, follower: ArmClient,
                 out: Optional[str] = None, name: str = "traj",
                 mode: str = "follower", hz: float = 50.0,
                 lead_calib: Optional[JointCalib] = None,
                 foll_calib: Optional[JointCalib] = None,
                 cfg: Optional[TeleopConfig] = None,
                 max_points: int = 0,
                 on_event: Optional[Callable[[str], None]] = None,
                 **teleop_kw):
        self.cfg = cfg or TeleopConfig(hz=hz)
        self.cfg.hz = hz
        self.rec = TeachRecorder(name=name, mode=mode, hz=hz, max_points=max_points)
        self.out = out
        self.tp = TeleopSession(leader, follower, lead_calib=lead_calib,
                                foll_calib=foll_calib, cfg=self.cfg,
                                on_frame=self.rec, on_event=on_event,
                                **teleop_kw)
        self.last_traj: Optional[Trajectory] = None

    # ---- 透传 ----
    @property
    def stats(self):
        return self.tp.stats

    @property
    def leader(self):
        return self.tp.leader

    @property
    def follower(self):
        return self.tp.follower

    def start(self) -> "TeleopTeachSession":
        self.tp.start()
        return self

    def stop(self) -> None:
        self.tp.stop()
        self.rec.stop()

    def __enter__(self) -> "TeleopTeachSession":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # ---- 主流程 ----
    def run(self, duration: Optional[float] = None,
            driver: Optional[Callable[[float], None]] = None, **kw):
        """只遥操不录。`driver(elapsed_s)` 见 `TeleopSession.run`。"""
        return self.tp.run(duration=duration, driver=driver, **kw)

    def record(self, duration: Optional[float] = None,
               out: Optional[str] = None,
               until: Optional[Callable[[], bool]] = None,
               driver: Optional[Callable[[float], None]] = None) -> Trajectory:
        """遥操 + 录制。返回轨迹（同时存到 `out`，缺省用构造时的 `out`）。

        `driver(elapsed_s)` 每帧先调一次 —— 自动化演示/测试时用它驱动"虚拟主臂"，
        真人遥操时留空。
        """
        self.rec.start()
        try:
            self.tp.run(duration=duration, until=until, driver=driver)
        finally:
            self.rec.stop()
        traj = self.rec.trajectory()
        path = out or self.out
        if path:
            traj.save(path)
        self.last_traj = traj
        return traj

    def save(self, path: Optional[str] = None) -> str:
        """把最近一次录制存盘（不重录）。"""
        p = path or self.out
        if not p:
            raise TeachError("没有指定保存路径")
        return (self.last_traj or self.rec.trajectory()).save(p)


# ============================================================================
# 回放
# ============================================================================

@dataclass
class PlaybackStats:
    frames: int = 0
    sent: int = 0
    loops: int = 0
    ack_limit: int = 0
    ack_estop: int = 0
    slew_clamped: int = 0
    behind: int = 0           # 落后计划超 1 帧次数
    t_start: float = 0.0
    t_end: float = 0.0

    @property
    def elapsed(self) -> float:
        if not self.t_start:
            return 0.0
        return (self.t_end or time.monotonic()) - self.t_start

    def summary(self) -> str:
        return ("回放 %d 帧 / %.1fs / %d 圈 | ACK limit=%d estop=%d | "
                "限速 %d 落后 %d"
                % (self.frames, self.elapsed, self.loops, self.ack_limit,
                   self.ack_estop, self.slew_clamped, self.behind))


class PlaybackSession:
    """把一条 `Trajectory` 回放到某台臂上（走 `STREAM_JOINTS`，逻辑角）。

        traj = Trajectory.load("demo1.json")
        with open_arm("net://192.168.7.120:5000") as arm:
            PlaybackSession(arm).play(traj, speed=0.5, loop=3)

    安全
    ----
    * 默认把轨迹按 `cfg.hz` 重采样（原轨迹 50 Hz、回放 50 Hz 时几乎无损）。
    * `slew_deg_per_s` 默认 0 = 不再额外限速（轨迹本身已经是限过速录出来的）；
      但 `max_step_deg` 恒 ≤ 25°（固件门限），超了自动削，**绝不触发自锁拒帧**。
    * 每 `estop_check_s` 查一次状态；E-Stop 立刻停。
    * `dry_run` = 一帧都不发。
    """

    def __init__(self, arm: ArmClient, cfg: Optional[TeleopConfig] = None,
                 slew_deg_per_s: float = 0.0, estop_check_s: float = 0.5,
                 dry_run: bool = False, calib: Optional[JointCalib] = None,
                 approach: bool = True, approach_speed: float = 20.0,
                 approach_tol_deg: float = 2.0, approach_timeout_s: float = 8.0,
                 warmup: int = 3,
                 on_frame: Optional[Callable[[int, List[float]], None]] = None,
                 on_event: Optional[Callable[[str], None]] = None):
        self.arm = arm
        self.cfg = cfg or TeleopConfig(hz=50.0)
        self.slew = float(slew_deg_per_s)
        self.estop_check_s = float(estop_check_s)
        self.dry_run = dry_run
        self.calib = calib or JointCalib()
        self.approach = approach
        self.approach_speed = float(approach_speed)
        self.approach_tol_deg = float(approach_tol_deg)
        self.approach_timeout_s = float(approach_timeout_s)
        self.warmup = int(warmup)
        self.on_frame = on_frame
        self.on_event = on_event
        self.stats = PlaybackStats()
        self._abort = False

    def _event(self, m: str) -> None:
        if self.on_event:
            try:
                self.on_event(m)
            except Exception:
                pass

    def stop_playback(self) -> None:
        self._abort = True

    # ---------------------------------------------------------------- 接近段
    def _approach_start(self, start_logical: Sequence[float]) -> bool:
        """回放前先走到轨迹起点。

        为什么必须做：`STREAM_JOINTS` 单帧单轴跳变 >25° 会被**整帧拒收**，而且被拒时
        基准不更新 ⇒ 如果当前姿态离轨迹起点很远，回放会**每一帧都被拒**（表现为
        "回放跑完了但臂一动不动，ACK limit 一大堆"）。用 `MOVE_JOINTS` 走一段
        S 曲线过去，回放就能正常起步。

        返回 True = 已接近（或无需接近）；False = 接近失败（调用方决定是否继续）。
        """
        if self.dry_run:
            return True
        try:
            st = self.arm.get_status()
        except ArmCommandError as e:
            self._event("⚠ 接近段读取当前姿态失败：%s" % e)
            return False
        cur = st.as_servo_deg()
        if cur is None:
            self._event("⚠ 接近段有轴读失败 J%s —— 跳过接近（回放可能被拒帧）"
                        % st.failed_joints())
            return False
        cur_log = self.calib.to_logical(cur)
        gap = max(abs(cur_log[j] - start_logical[j]) for j in range(NUM_JOINTS))
        if gap <= self.approach_tol_deg:
            self._event("接近段：已在线（最大差 %.2f°）" % gap)
            return True
        target_servo = self.calib.to_servo(start_logical)
        self._event("接近段：当前 %s → 起点 %s（最大差 %.1f°，MOVE_JOINTS v=%.0f）"
                    % ("[" + ", ".join("%.1f" % v for v in cur_log) + "]",
                       "[" + ", ".join("%.1f" % v for v in start_logical) + "]",
                       gap, self.approach_speed))
        r = self.arm.move_joints(target_servo, v=self.approach_speed)
        if not r.ok:
            self._event("⚠ 接近段 MOVE_JOINTS 失败：%s" % r)
            return False
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < self.approach_timeout_s:
            if self._abort:
                return False
            time.sleep(0.05)
            try:
                st = self.arm.get_status(timeout=0.3)
            except ArmCommandError:
                continue
            sv = st.as_servo_deg()
            if sv is None:
                continue
            lg = self.calib.to_logical(sv)
            err = max(abs(lg[j] - start_logical[j]) for j in range(NUM_JOINTS))
            if err <= self.approach_tol_deg:
                self._event("接近段：到位（残差 %.2f°）" % err)
                return True
            last = err
        self._event("⚠ 接近段 %.1fs 未收敛（残差 %s）—— 继续回放，可能被拒帧"
                    % (self.approach_timeout_s,
                       "%.1f°" % last if last is not None else "?"))
        return False

    # ---------------------------------------------------------------- 主流程
    def play(self, traj: Trajectory, speed: float = 1.0, loop: int = 1,
             start_pose_delay_s: float = 0.0,
             duration: Optional[float] = None) -> PlaybackStats:
        """回放。`loop=0` 表示无限循环（Ctrl-C 停）。"""
        if speed <= 0:
            raise TeachError("speed 必须 > 0")
        if len(traj) < 2:
            raise TeachError("轨迹不足 2 点，无法回放")
        hz = float(self.cfg.hz or traj.hz or 50.0)
        if hz < FW_MIN_HZ:
            raise TeachError("回放频率 %.1f Hz 太低（>2 Hz）" % hz)
        rep = traj.resample(hz)
        period = 1.0 / hz
        # 额外限速：显式参数优先，其次 cfg.slew_deg_per_s；都不给 = 不额外限
        max_step = self.slew if self.slew > 0 else float(self.cfg.slew_deg_per_s or 0.0)
        max_step = max_step / hz if max_step > 0 else 0.0
        self._abort = False
        self.stats = PlaybackStats()
        self.stats.t_start = time.monotonic()
        self._event("回放 %s：%d 点 → %d 点 @ %.0f Hz, speed=%.2f, loop=%d"
                    % (traj.name, len(traj), len(rep), hz, speed, loop))
        if start_pose_delay_s > 0:
            time.sleep(start_pose_delay_s)
        # 先接近轨迹起点（否则第一帧就是 >25° 跳变 ⇒ 整段回放全被拒）
        start_logical = rep.frames[0][1]
        if self.approach:
            self._approach_start(start_logical)
        for _i in range(max(0, self.warmup)):
            self._send(start_logical)      # 建立流基准，保证从臂进入流模式
            time.sleep(period)
        try:
            n_loop = 0
            while True:
                t_loop0 = time.monotonic()
                for i, (t, q) in enumerate(rep.frames):
                    if self._abort:
                        raise KeyboardInterrupt
                    target = list(q)
                    if max_step > 0 and i > 0:
                        prev = rep.frames[i - 1][1]
                        for j in range(NUM_JOINTS):
                            d = target[j] - prev[j]
                            if abs(d) > max_step:
                                target[j] = prev[j] + math.copysign(max_step, d)
                                self.stats.slew_clamped += 1
                    # 单帧跳变硬上限（固件 25°，留 20% 余量）
                    if i > 0:
                        prev = rep.frames[i - 1][1]
                        for j in range(NUM_JOINTS):
                            d = target[j] - prev[j]
                            if abs(d) > FW_MAX_JUMP_DEG * 0.8:
                                target[j] = prev[j] + math.copysign(
                                    FW_MAX_JUMP_DEG * 0.8, d)
                                self.stats.slew_clamped += 1
                    self._send(target)
                    self.stats.frames += 1
                    if self.on_frame:
                        try:
                            self.on_frame(i, target)
                        except Exception:
                            pass
                    # 节拍
                    want = t_loop0 + t / speed
                    slp = want - time.monotonic()
                    if slp > 0:
                        time.sleep(min(slp, period))
                    elif slp < -period:
                        self.stats.behind += 1
                    self._maybe_check_estop()
                    if duration is not None and \
                            (time.monotonic() - t_loop0) > duration:
                        break
                n_loop += 1
                self.stats.loops = n_loop
                if loop and n_loop >= loop:
                    break
                if duration is not None and \
                        (time.monotonic() - self.stats.t_start) > duration:
                    break
        except KeyboardInterrupt:
            self._event("回放中止")
        finally:
            self.stats.t_end = time.monotonic()
            if not self.dry_run:
                try:
                    self.arm.stop()
                except Exception:
                    pass
            self._event("回放结束：" + self.stats.summary())
        return self.stats

    def _send(self, target: Sequence[float]) -> None:
        if self.dry_run:
            self.stats.sent += 1
            return
        self.arm.stream_joints(target, wait=False)
        self.stats.sent += 1
        for a in self.arm.poll_stream_acks():
            if a.err == int(ErrCode.LIMIT):
                self.stats.ack_limit += 1
            elif a.err == int(ErrCode.ESTOP):
                self.stats.ack_estop += 1

    def _maybe_check_estop(self) -> None:
        if self.estop_check_s <= 0 or self.dry_run:
            return
        now = time.monotonic()
        if now - getattr(self, "_t_estop", 0.0) < self.estop_check_s:
            return
        self._t_estop = now
        try:
            st = self.arm.get_status(timeout=0.2)
        except ArmCommandError:
            return
        if st.estop:
            self._event("🛑 回放中检测到 E-Stop —— 停")
            self._abort = True


# ============================================================================
# 固件侧示教（串口文本）
# ============================================================================

class FirmwareTeach:
    """从臂固件录音/回放的串口命令封装（`ConsoleClient`）。

        t = FirmwareTeach(foll_console)
        t.enter_rec()          # teach rec —— **只读跟录**，不碰舵机（推荐）
        # ... 拖主臂遥操 ...
        print(t.rec_list())    # rec list 看点数和时长（存盘前必须核对）
        t.save("demo1")        # save demo1 → 写 Flash（阻塞 1~3 s）
        t.play("demo1")        # play demo1 / play demo1 loop

    ⚠ 固件录音限制：50 Hz × 4000 点 ≈ **80 s**；`save` 阻塞 1~3 s 期间 CPU 冻结；
      未存盘的录音会被 `rec clear` / 断电 / 再进 `teach` 清掉。
      超过 80 s 或要无损导出请用 `TeleopTeachSession`（PC 侧）。
    """

    def __init__(self, console: ArmSDK):
        self.c = console

    # ---- 录制 ----
    def enter_rec(self):
        """`teach rec` 跟随录制：只读舵机、不松力矩、不碰舵机（PC 中转时用）。"""
        return self.c.send_command("teach rec", timeout=5.0)

    def enter_teach(self):
        """`teach` 进入手拖示教（**会松力矩**——臂会下沉，先托住）。"""
        return self.c.send_command("teach", timeout=5.0)

    def exit_teach(self):
        """`teach exit` 退出示教并恢复力矩。"""
        return self.c.send_command("teach exit", timeout=5.0)

    def rec_list(self):
        """`rec list` 未存盘录音摘要 + 已存盘数量（存盘前必看）。"""
        return self.c.send_command("rec list")

    def rec_clear(self):
        """`rec clear` 丢弃未存盘录音（不影响已存盘文件）。"""
        return self.c.send_command("rec clear")

    def dump(self):
        """`teach dump` 打印示教/录音内部状态。"""
        return self.c.send_command("teach dump")

    # ---- 存盘 / 文件 ----
    def save(self, name: Optional[str] = None, timeout: float = 20.0):
        """`save [name]` 把未存盘录音写 Flash（阻塞 1~3 s）。名字 ≤11 字符。"""
        return self.c.send_command("save" if not name else "save %s" % name,
                                   timeout=timeout)

    def files(self):
        """`files`（= `list`）列出已存盘录音：序号/名字/样本数/时长。"""
        return self.c.send_command("files")

    def play(self, name: Optional[str] = None, loop: bool = False):
        """`play <name|#> [loop]` 回放已存盘录音（从 Flash 流式读）。"""
        cmd = "play"
        if name is not None:
            cmd += " %s" % name
        if loop:
            cmd += " loop"
        return self.c.send_command(cmd, timeout=10.0)

    def play_temp(self, loop: bool = False):
        """`play`（无参）回放**未存盘**的临时录音。"""
        return self.c.send_command("play loop" if loop else "play", timeout=10.0)

    def delete(self, name: str):
        """`del <name|#>` 删文件（墓碑；空间靠 `compact` 回收）。"""
        return self.c.send_command("del %s" % name)

    def compact(self, timeout: float = 60.0):
        """`compact` 回收墓碑空间（数秒，期间 CPU 冻结可接受）。"""
        return self.c.send_command("compact", timeout=timeout)

    # ---- 示教参数（手拖手感）----
    def set_param(self, key: str, *vals) -> object:
        """`teach <key> [val...]`：g(重力补偿) / o(偏置) / c(阻尼) / d(死区)
        / v(阻尼) / b(死区避开) / cap / f / p(PID) / i / test …"""
        parts = ["teach", str(key)] + [str(v) for v in vals]
        return self.c.send_command(" ".join(parts))

    def status(self):
        """`teach dump` + `V` + PID 缓存，一起看手拖相关状态。"""
        return self.c.send_command("teach dump")


# ============================================================================
# 自检（离线，不需要硬件）
# ============================================================================

def _selftest() -> int:
    import tempfile

    fails: List[str] = []
    TOTAL = [0]

    def chk(name, cond, extra=""):
        TOTAL[0] += 1
        if not cond:
            fails.append("%s %s" % (name, extra))
            print("  FAIL %s %s" % (name, extra))
        else:
            print("  ok   %s" % name)

    def mk(hz=50.0, n=51, amp=10.0):
        t = Trajectory(name="t", hz=hz, source="test")
        for i in range(n):
            ts = i / hz
            t.append(ts, [amp * math.sin(2 * math.pi * 0.5 * ts + j) for j in range(6)])
        return t

    t = mk()
    chk("点数", len(t) == 51)
    chk("时长", abs(t.duration - 1.0) < 1e-6)
    chk("采样率", abs(t.rate - 50.0) < 1e-6)
    chk("joints_matrix 形状", len(t.joints_matrix()) == 6 and len(t.joints_matrix()[0]) == 51)
    chk("start/end pose", t.start_pose() is not None and len(t.end_pose()) == 6)

    # ---- resample ----
    r = t.resample(100.0)
    chk("resample 100Hz 点数", len(r) == 101, str(len(r)))
    chk("resample hz 记录", r.hz == 100.0)
    r2 = t.resample(25.0)
    chk("resample 降采样", len(r2) == 26, str(len(r2)))

    # ---- smooth ----
    s = t.smooth(5)
    chk("smooth 点数不变", len(s) == len(t))
    chk("smooth 首点不变", s.frames[0][1] == t.frames[0][1])
    jt, js = t.max_step()[1], s.max_step()[1]
    chk("smooth 降跳变", js <= jt + 1e-9, "%s -> %s" % (jt, js))

    # ---- slice / scale ----
    c = t.slice(0.2, 0.6)
    chk("slice 时间归零", abs(c.frames[0][0]) < 1e-9)
    chk("slice 时长", abs(c.duration - 0.4) < 1e-6, str(c.duration))
    sc = t.scale_time(2.0)
    chk("scale_time 时长翻倍", abs(sc.duration - 2.0) < 1e-6)

    # ---- validate ----
    v = t.validate()
    chk("validate ok", v["ok"] is True, str(v["warnings"]))
    bad = Trajectory(name="bad", hz=50.0)
    bad.append(0.0, [0] * 6)
    bad.append(0.02, [60, 0, 0, 0, 0, 0])       # 单帧 60° 跳变
    vb = bad.validate()
    chk("validate 抓 25° 跳变", vb["ok"] is False and
        any("25" in w for w in vb["warnings"]), str(vb["warnings"]))
    chk("validate 抓点数不足", any("不足" in w for w in
                                 Trajectory(name="x").validate()["warnings"]))

    # ---- 软限位 ----
    lb = t.check_limits([(-3, 3)] * 6)
    chk("check_limits 有越界", len(lb) > 0)
    chk("check_limits 轴号", all(0 <= j < 6 for j, _ in lb))

    # ---- 序列化往返 ----
    with tempfile.TemporaryDirectory() as d:
        for ext, loader, saver in ((".json", Trajectory.load_json, "save_json"),
                                   (".jsonl", Trajectory.load_jsonl, "save_jsonl"),
                                   (".csv", Trajectory.load_csv, "save_csv")):
            p = os.path.join(d, "traj" + ext)
            getattr(t, saver)(p)
            back = loader(p)
            same = (len(back) == len(t) and
                    abs(back.duration - t.duration) < 1e-4 and
                    all(abs(back.frames[i][1][j] - t.frames[i][1][j]) < 1e-4
                        for i in range(0, len(t), 10) for j in range(6)))
            chk("往返 %s" % ext, same, "%d pts" % len(back))
        # save()/load() 按扩展名自动分派
        p = os.path.join(d, "auto.csv")
        t.save(p)
        chk("save() 自动分派", len(Trajectory.load(p)) == len(t))

    # ---- TeachRecorder ----
    rec = TeachRecorder(name="r", mode="follower", hz=50.0)
    rec.start()
    for i in range(10):
        rec.record([float(i)] * 6, t=0.02 * i)
    chk("recorder 点数", rec.count == 10)
    chk("recorder 时长", abs(rec.duration - 0.18) < 1e-6, str(rec.duration))
    tr = rec.trajectory()
    chk("recorder 产物 units", tr.units == LOGICAL)
    chk("recorder source", tr.source == "teleop:follower")
    # 当 TeleopSession 回调
    rec.start()
    rec([0.0] * 6, [1.0] * 6)
    chk("recorder __call__ follower", rec.trajectory().frames[0][1] == [1.0] * 6)
    rec.start()
    rec2 = TeachRecorder(mode="leader")
    rec2.start()
    rec2([7.0] * 6, [1.0] * 6)
    chk("recorder __call__ leader", rec2.trajectory().frames[0][1] == [7.0] * 6)
    # max_points
    recm = TeachRecorder(max_points=3)
    recm.start()
    for i in range(10):
        recm.record([0.0] * 6, t=0.02 * i)
    chk("max_points 截断", recm.count == 3)

    # ---- PlaybackSession（假臂）----
    class _FakeArm:
        def __init__(self):
            self.sent: List[List[float]] = []
            self.dry_run = False
            self.estop = False
            self.servo = [180.0] * 6
            self.moves: List[List[float]] = []

        def stream_joints(self, q, wait=True, timeout=None):
            self.sent.append(list(q))
            return None

        def poll_stream_acks(self, drain=True):
            return []

        def get_status(self, **kw):
            from .protocol import Status
            s = Status()
            s.joints = list(self.servo)
            s.read_ok = True
            s.estop = self.estop
            return s

        def move_joints(self, q, v=0.0, a=0.0, **kw):
            self.moves.append(list(q))
            self.servo = [float(x) for x in q]

            class R:
                ok = True
            return R()

        def stop(self, **kw):
            return None

    arm = _FakeArm()
    pb = PlaybackSession(arm, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0, warmup=0)
    small = Trajectory(name="p", hz=50.0)
    for i in range(11):
        small.append(i / 50.0, [i * 0.5] * 6)     # 0.5°/帧
    st = pb.play(small, speed=50.0)              # 加速跑完
    chk("回放发出帧数", st.sent == 11, str(st.sent))
    chk("回放 loops=1", st.loops == 1)
    chk("回放末帧值", arm.sent[-1][1] == 5.0, str(arm.sent[-1][1]))

    arm2 = _FakeArm()
    pb2 = PlaybackSession(arm2, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0, warmup=0)
    st2 = pb2.play(small, speed=50.0, loop=2)
    chk("回放 loop=2", st2.loops == 2 and len(arm2.sent) == 22, str(len(arm2.sent)))

    # 单帧大跳变被硬削到 <25°
    arm3 = _FakeArm()
    pb3 = PlaybackSession(arm3, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0, warmup=0)
    jump = Trajectory(name="j", hz=50.0)
    jump.append(0.0, [0.0] * 6)
    jump.append(0.02, [100.0] * 6)
    pb3.play(jump, speed=50.0)
    chk("回放硬削 25° 门限", abs(arm3.sent[-1][0]) < FW_MAX_JUMP_DEG,
        str(arm3.sent[-1][0]))
    chk("回放限速计数", pb3.stats.slew_clamped > 0)

    # ---- 接近段：当前姿态离起点很远 ⇒ 先 MOVE_JOINTS 走过去 ----
    arm4 = _FakeArm()                      # 当前 180（逻辑 0）
    arm4.servo = [180.0] * 6
    pb4 = PlaybackSession(arm4, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0,
                          warmup=0, approach=True, approach_timeout_s=0.5)
    far = Trajectory(name="far", hz=50.0)
    for i in range(11):
        far.append(i / 50.0, [30.0 + i * 0.5] * 6)     # 起点逻辑 +30°
    pb4.play(far, speed=50.0)
    chk("接近段触发了 MOVE_JOINTS", len(arm4.moves) >= 1, str(len(arm4.moves)))
    chk("接近段目标是逻辑 +30 的舵机度",
        arm4.moves and abs(arm4.moves[0][1] - 210.0) < 0.01, str(arm4.moves[:1]))
    # approach=False ⇒ 不接近
    arm5 = _FakeArm()
    pb5 = PlaybackSession(arm5, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0,
                          warmup=0, approach=False)
    pb5.play(far, speed=50.0)
    chk("approach=False 不接近", len(arm5.moves) == 0)

    # ---- 预热帧：保证从臂进入流模式 ----
    arm6 = _FakeArm()
    pb6 = PlaybackSession(arm6, cfg=TeleopConfig(hz=50.0), estop_check_s=0.0,
                          warmup=3, approach=False)
    pb6.play(small, speed=50.0)
    chk("预热帧计入发送", len(arm6.sent) == 14, str(len(arm6.sent)))

    # ---- 失败用例 ----
    try:
        PlaybackSession(_FakeArm()).play(Trajectory(name="empty"))
        chk("空轨迹拒绝", False)
    except TeachError:
        chk("空轨迹拒绝", True)
    try:
        TeachRecorder(mode="nope")
        chk("非法 mode 拒绝", False)
    except ValueError:
        chk("非法 mode 拒绝", True)

    # ---- FirmwareTeach 命令串 ----
    class _FakeConsole:
        def __init__(self):
            self.cmds = []

        def send_command(self, c, timeout=None, echo=True):
            self.cmds.append(c)
            class R:
                ok = True
            return R()

    fc = _FakeConsole()
    ft = FirmwareTeach(fc)
    ft.enter_rec(); ft.rec_list(); ft.save("demo1"); ft.play("demo1", loop=True)
    ft.files(); ft.delete("demo1"); ft.compact(); ft.rec_clear()
    ft.set_param("g", 3, 60)
    want = ["teach rec", "rec list", "save demo1", "play demo1 loop",
            "files", "del demo1", "compact", "rec clear", "teach g 3 60"]
    chk("FirmwareTeach 命令串", fc.cmds == want, str(fc.cmds))

    print("")
    print("  teach 自检：%d 项，%s" % (TOTAL[0],
                                    "全部通过" if not fails else "失败 %d" % len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())

