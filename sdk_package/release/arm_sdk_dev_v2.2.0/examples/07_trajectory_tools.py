# -*- coding: utf-8 -*-
"""
07_trajectory_tools.py — 轨迹检查 / 转换 / 重采样 / 平滑 / 导出

跑法：
    python examples/07_trajectory_tools.py demo.json
"""
import os
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import Trajectory

if len(sys.argv) < 2:
    raise SystemExit(__doc__)

SRC = sys.argv[1]
t = Trajectory.load(SRC)

# ---- 1. 体检（回放前必看）----
v = t.validate(limits=[(-60, 60)] * 6)
print("=== %s ===" % SRC)
print("  %d 点 / %.3fs / %.2f Hz / 单位=%s / 来源=%s"
      % (v["points"], v["duration_s"], v["rate_hz"], t.units, t.source))
print("  每轴单帧最大跳变 %s  (>25° 会被固件整帧拒收)" % v["max_step_deg"])
print("  首帧 %s" % t.start_pose())
print("  末帧 %s" % t.end_pose())
print("  结论：%s" % ("可以做回放" if v["ok"] else "先修：" + "; ".join(v["warnings"])))

# ---- 2. 修（如果体检不过）----
fixed = t
if any(s > 25 for s in v["max_step_deg"]):
    print("\n  → 单帧跳变过大：先 resample 到 50Hz 再 smooth")
    fixed = t.resample(50.0).smooth(5)
    print("    修后最大跳变 %s" % fixed.max_step())

# ---- 3. 时间轴操作 ----
print("\n=== 变换 ===")
print("  原样         %s" % fixed.summary())
print("  减速 2×      %s" % fixed.scale_time(2.0).summary())
print("  裁 1s~3s     %s" % fixed.slice(1.0, 3.0).summary())
print("  量化到 25Hz  %s" % fixed.resample(25.0).summary())

# ---- 4. 导出（各格式的用途）----
print("\n=== 导出 ===")
base = os.path.splitext(SRC)[0]
for ext, why in ((".json", "单文件带元数据，SDK 首选"),
                 (".jsonl", "一行一帧，超大轨迹不占内存峰值"),
                 (".csv", "最通用，Excel/MATLAB 直接开"),
                 (".npz", "numpy / LeRobot 数据集")):
    p = base + "_out" + ext
    try:
        fixed.save(p)
        print("  %-28s %s" % (p, why))
    except Exception as e:                              # noqa: BLE001
        print("  %-28s 跳过（%s）" % (p, e))

# ---- 5. 统计（画图用）----
print("\n=== 每轴统计 ===")
m = fixed.joints_matrix()
for j in range(6):
    if not m[j]:
        continue
    print("  J%d  min %7.2f  max %7.2f  mean %7.2f  (n=%d)"
          % (j + 1, min(m[j]), max(m[j]), sum(m[j]) / len(m[j]), len(m[j])))
