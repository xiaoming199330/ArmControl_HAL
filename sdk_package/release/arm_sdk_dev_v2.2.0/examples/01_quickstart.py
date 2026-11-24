# -*- coding: utf-8 -*-
"""
01_quickstart.py — 5 分钟上手（网口）

跑法：
    python examples/01_quickstart.py                      # 默认 192.168.7.120:5000
    python examples/01_quickstart.py net://192.168.7.121:5000
    python examples/01_quickstart.py sim://               # 无硬件

⚠ 动之前先看 `armctl preflight` 那套检查：PING / 6 轴全读到 / 没有 E-Stop。
"""
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import ArmCommandError, JointCalib, open_arm

URI = sys.argv[1] if len(sys.argv) > 1 else "net://192.168.7.120:5000"

with open_arm(URI, timeout=0.5) as arm:
    print("链路：%s" % arm.transport.name)

    # ---- 1. 探活 ----
    pi = arm.ping()
    if pi is None:
        raise SystemExit("PING 无响应：①`link net`→`link save`→RESET ②端口 5000 "
                         "③本构建含网口(LINK=net|both) ④IP/网线")
    print("PING proto_ver=%d run_id=%d" % (pi.proto_ver, pi.run_id))

    # ---- 2. 状态（舵机度）----
    st = arm.get_status()
    print("舵机度 %s" % st.joints)          # 某轴 None = 该轴读失败（不兜底）
    print("位姿   X=%.1f Y=%.1f Z=%.1f pitch=%.1f" % (st.x, st.y, st.z, st.pitch))
    print("flags  estop=%s traj=%s corr=%s read_ok=%s"
          % (st.estop, st.traj, st.corr, st.read_ok))
    if st.failed_joints():
        print("⚠ 读失败的轴 J%s —— 先排总线再运动" % st.failed_joints())

    # ---- 3. 逻辑角（STREAM_JOINTS / 遥操用的量纲）----
    calib = JointCalib()                     # 真机应先把 `zi` 读到的 offset 填进来
    sv = st.as_servo_deg()
    if sv:
        print("逻辑角 %s" % [round(v, 2) for v in calib.to_logical(sv)])

    # ---- 4. 正解 ----
    fk = arm.fk_query()
    print("FK     X=%.1f Y=%.1f Z=%.1f pitch=%.1f read_ok=%d"
          % (fk.x, fk.y, fk.z, fk.pitch, fk.read_ok))

    # ---- 5. 点动（**必须用增量**）----
    if sv:
        target = list(sv)
        target[1] += 5.0                     # J2 +5°（舵机度）
        r = arm.move_joints(target, v=10)
        print("点动 %s" % r)
        # ⚠ err=0 只代表"命令收下"：读起点失败时也回 ACK 但轨迹没起步。
        #    判断"动没动"必须再读一次状态。
        import time
        time.sleep(0.8)
        after = arm.get_status().as_servo_deg()
        print("点动后 %s" % (None if after is None
                             else [round(v, 2) for v in after]))

    # ---- 6. 参数（写要回读，因为超范围会**静默钳制**）----
    print("PARAM_COUNT = %d" % arm.param_count())
    ok, back = arm.param_write_verified(34, 2.5)      # corr.threshold
    print("写 34=2.5 → 实际 %.3f (%s)" % (back, "一致" if ok else "被钳制"))

    # ---- 7. 停 ----
    print("%s" % arm.stop())
    print("统计 %s" % arm.describe())
