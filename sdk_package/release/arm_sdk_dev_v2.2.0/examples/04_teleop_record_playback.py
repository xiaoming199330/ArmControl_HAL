# -*- coding: utf-8 -*-
"""
04_teleop_record_playback.py — 遥操示教：一边遥操一边录，然后回放

比固件 `teach rec` 好在哪：
  · **没有 80 s / 4000 点上限**（那是固件 Flash 录音的限制）
  · 直接落 JSON/JSONL/CSV/NPZ，能喂 LeRobot / PyTorch / MATLAB
  · 存盘不阻塞（固件 `save` 会冻结 CPU 1~3 s）
  · 不会被 `rec clear` / 断电 / 再进 `teach` 悄悄清掉

跑法：
    python examples/04_teleop_record_playback.py sim:// sim:// demo.json 5
    python examples/04_teleop_record_playback.py net://192.168.7.121:5000 \
           net://192.168.7.120:5000 demo.json 60
"""
import math
import os
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import (JointCalib, PlaybackSession, TeleopConfig, TeleopTeachSession,
                     Trajectory, open_arm)

LEADER = sys.argv[1] if len(sys.argv) > 1 else "sim://"
FOLLOWER = sys.argv[2] if len(sys.argv) > 2 else "sim://"
OUT = sys.argv[3] if len(sys.argv) > 3 else "demo.json"
SEC = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0

cfg = TeleopConfig(hz=50.0, slew_deg_per_s=240.0, sync="hold", verbose=True)
lead = open_arm(LEADER, timeout=0.3)
foll = open_arm(FOLLOWER, timeout=0.3)

# ------------------------------------------------------------------ 录制
with lead, foll:
    ts = TeleopTeachSession(lead, foll, out=OUT, name=os.path.basename(OUT),
                            hz=50.0, cfg=cfg,
                            lead_calib=JointCalib(), foll_calib=JointCalib(),
                            on_event=lambda m: print("  [ev] " + m))
    ts.start()
    if LEADER.startswith("sim"):
        def drive(el):
            lead.move_joints([180 + 15 * math.sin(el * 1.2),
                              180 + 10 * math.sin(el * 1.8),
                              180 + 6 * math.sin(el * 2.4),
                              180, 180, 180], v=100)
        traj = ts.record(duration=SEC, driver=drive)
    else:
        print("拖主臂开始示教（Ctrl-C 停止并保存）")
        try:
            traj = ts.record(duration=None)
        except KeyboardInterrupt:
            ts.stop()
            traj = ts.last_traj or ts.rec.trajectory()
            traj.save(OUT)
    ts.stop()

print("\n=== 录制结果 ===")
print(traj.summary())
print("validate: %s" % traj.validate())

# ------------------------------------------------------------------ 导出
print("\n=== 导出 ===")
for ext in (".json", ".jsonl", ".csv"):
    p = os.path.splitext(OUT)[0] + ext
    traj.save(p)
    print("  %-10s %6d 点  %s" % (ext, len(Trajectory.load(p)), p))
try:
    p = os.path.splitext(OUT)[0] + ".npz"
    traj.save(p)
    print("  %-10s %6d 点  %s  ← 给 numpy/LeRobot" % (".npz", len(traj), p))
except Exception as e:                                  # noqa: BLE001
    print("  .npz 跳过（%s）" % e)

# ------------------------------------------------------------------ 回放
print("\n=== 回放到从臂 ===")
arm = open_arm(FOLLOWER, timeout=0.3)
with arm:
    pb = PlaybackSession(arm, cfg=TeleopConfig(hz=50.0), estop_check_s=0.3,
                         approach=True,               # 先 MOVE_JOINTS 走到轨迹起点
                         on_event=lambda m: print("  [pb] " + m))
    st = pb.play(traj.resample(50.0), speed=1.0, loop=1)
    print("  %s" % st.summary())
    if st.ack_limit:
        print("  ⚠ 有拒帧：见 docs/主从遥操与示教.md 的排查表")
