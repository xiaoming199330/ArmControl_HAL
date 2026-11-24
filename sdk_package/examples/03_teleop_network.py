# -*- coding: utf-8 -*-
"""
03_teleop_network.py — 主从臂遥操（网口路线，推荐）

为什么首选网口：UDP 往返 p50 **1.61 ms**（CAN 2.72 / 串口 4.78），50 Hz 有 15× 余量，
而且**两臂之间一根线都不用接** —— PC 就是那根线。

架构：
    主臂 GET_STATUS(50Hz, 舵机度) → 逻辑角 → gain/反向/软限位 → **限速** → 从臂 STREAM_JOINTS

「限速」是必须的：固件单帧单轴跳变 >25° 会**整帧拒收且不更新基准** ⇒ 会自锁冻结。
TeleopSession 用 slew_deg_per_s(默认 240°/s = 4.8°/帧 @50Hz) 从源头避开。

跑法：
    # 无硬件先试逻辑
    python examples/03_teleop_network.py sim:// sim://

    # 真机（从臂 .120 / 主臂 .121）
    python examples/03_teleop_network.py net://192.168.7.121:5000 net://192.168.7.120:5000 30

前置（真机）：
    1. 两臂都敲 `h` 看 fw-tag；`zi` 确认 Home 零点没被污染
    2. 两臂都做过 `c` 校准（Home 零点对应同一物理姿态）⇒ 逻辑角才可比
    3. 从臂 `sf 1`（默认就是 ON；`sf 0` 会忽略所有流帧）
    4. 从臂 E-Stop 在手上
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import (JointCalib, TeleopAbort, TeleopConfig, TeleopLinkLost,
                     TeleopSession, open_arm)

LEADER = sys.argv[1] if len(sys.argv) > 1 else "sim://"
FOLLOWER = sys.argv[2] if len(sys.argv) > 2 else "sim://"
SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

lead = open_arm(LEADER, timeout=0.3)
foll = open_arm(FOLLOWER, timeout=0.3)

cfg = TeleopConfig(
    hz=50.0,                     # >2Hz 是底线，20~50Hz 推荐
    slew_deg_per_s=240.0,        # 每轴限速：50Hz → 4.8°/帧，离 25° 门限 5× 余量
    sync="hold",                 # hold=按位移增量跟随（最安全）；align=先对齐姿态
    gain=1.0,
    reverse=False,               # 镜像安装的从臂可能要 [0,0,0,0,0,0]→按需给序列
    estop_trips=True,            # 主臂 E-Stop → 立刻停从臂
    hold_on_leader_loss=True,    # 主臂读失败时 hold 上一帧，保住流不超时自锁
    link_loss_timeout_s=1.0,
    verbose=True,
    # soft_limits_deg=[(-20, 20)] * 6,   # 逻辑角软限位（可选）
)

print("leader=%s  follower=%s  %.0f Hz / 限速 %.0f°/s" %
      (LEADER, FOLLOWER, cfg.hz, cfg.slew_deg_per_s))

with lead, foll:
    tp = TeleopSession(lead, foll, cfg=cfg,
                       lead_calib=JointCalib(), foll_calib=JointCalib(),
                       on_event=lambda m: print("  [ev] " + m))

    if LEADER.startswith("sim"):
        # 无硬件：用一段正弦当"虚拟人手"，跑完自动打印统计
        def drive(el):
            import math
            lead.move_joints([180 + 15 * math.sin(el * 1.5),
                              180 + 10 * math.sin(el * 2.0),
                              180 + 8 * math.sin(el * 2.5),
                              180, 180, 180], v=100)

        tp.start()
        try:
            tp.run(duration=SEC, driver=drive)
        except (TeleopLinkLost, TeleopAbort) as e:
            print("停机：%s" % e)
    else:
        # 真机：拖主臂。Ctrl-C 停。
        print("现在拖主臂 → 从臂跟随；Ctrl-C 结束")
        try:
            tp.start()
            tp.run(duration=None)
        except KeyboardInterrupt:
            print("\n用户中断")
        except TeleopLinkLost as e:
            print("✗ 主臂链路丢失：%s" % e)
        except TeleopAbort as e:
            print("🛑 %s" % e)

    print(tp.stats.summary())
    if tp.stats.ack_limit:
        print("⚠ 有 %d 帧被拒(err=4)：检查 sf 门闩 / 起点是否太远 / 限速是否过松"
              % tp.stats.ack_limit)
