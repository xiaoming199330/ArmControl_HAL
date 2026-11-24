# -*- coding: utf-8 -*-
"""
02_offline_sim.py — 无硬件开发（内存假臂）

`sim://` 实现了固件的全部关键语义，包括那些"坑"：
  · 25° 跳变整帧拒收 + 被拒时基准不更新（自锁）
  · 静默钳制（MOVE_JOINTS 目标 / PARAM_WRITE 值）
  · read_ok=0 与 -32768 哨兵
  · PING 第 2 字节是 run_id 而不是错误码
  · CMD>=0x80 故意不回

可以拿它当"上位机联调的靶子"：先在 sim 上把逻辑跑通，再上真机。

跑法：python examples/02_offline_sim.py
"""
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import ErrCode, open_arm

with open_arm("sim://") as arm:
    print("=== 正常路径 ===")
    print("PING  %s" % arm.ping())
    print("状态  %s" % arm.get_status().joints)
    arm.move_joints([180] * 6, v=50)
    print("正解  %s" % arm.fk_query())

    print("\n=== 坑 1：静默钳制（err=0 但值被改了）===")
    r = arm.param_write(4, 1e9)                       # ik.fk_verify_mm ∈ [1,20]
    print("写 1e9 -> %s，回读 %s（不是 1e9！）" % (r, arm.param_read(4)))

    print("\n=== 坑 2：25° 跳变整帧拒收 + 自锁 ===")
    arm.stream_joints([0, 0, 0, 0, 0, 0])
    bad = [0, 0, 0, 0, 0, 60]
    for i in range(3):
        r = arm.stream_joints(bad)
        print("  第 %d 次发 %s -> %s" % (i + 1, bad, r.desc))
    print("  ⇒ 被拒时不更新基准，同一目标会一直拒（这就是'自锁冻结'）")

    print("\n=== 坑 3：读失败不兜底 ===")
    arm.transport.set_joint_fail([2])                 # J3 读失败
    st = arm.get_status()
    print("  joints=%s  failed_joints=%s  as_servo_deg=%s"
          % (st.joints, st.failed_joints(), st.as_servo_deg()))
    print("  ⇒ None 就是 None，绝不填 0 或上次值")
    arm.transport.set_joint_fail([])

    print("\n=== 坑 4：CMD>=0x80 故意不回（防死锁）===")
    r = arm.request(0x80, timeout=0.15)
    print("  发 0x80 -> timeout=%s" % r.timeout)

    print("\n=== 故障注入：E-Stop ===")
    arm.transport.set_estop(True)
    st = arm.get_status()
    r = arm.move_joints([180] * 6)
    print("  estop=%s  MOVE_JOINTS -> %s (err=%s/%s)"
          % (st.estop, r, r.err, ErrCode(r.err).name if r.err in
             [e.value for e in ErrCode] else "?"))
    arm.transport.set_estop(False)

    print("\n=== 统计 ===")
    print("  %s" % arm.describe())
    print("  stream: %s" % arm.transport.stream_stats())
