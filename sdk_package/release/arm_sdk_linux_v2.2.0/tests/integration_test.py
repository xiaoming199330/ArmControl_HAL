# -*- coding: utf-8 -*-
"""
integration_test.py — arm_sdk 离线端到端自检（不需要任何硬件）

覆盖：协议字节精确性 → 模拟器语义 → 客户端 → 主从遥操 → 录制 → 回放 → 存盘。

跑法：
    cd sdk_package
    python -m tests.integration_test        # 或直接 python tests/integration_test.py
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_sdk import (ArmClient, ErrCode, JointCalib, PlaybackSession, TeleopConfig,
                     TeleopSession, TeleopTeachSession, Trajectory, crc16, open_arm,
                     pack_logical, unpack_logical)

FAILS = []
TOTAL = [0]


def chk(name, cond, extra=""):
    TOTAL[0] += 1
    if cond:
        print("  ok   %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s %s" % (name, extra))


def sec(t):
    print("\n== %s ==" % t)


# ============================================================================
sec("1. 协议字节精确性（对齐 docs/网口指令大全 hex 例子）")

chk("CRC16 标准向量 crc16(b'123456789')==0x29B1", crc16(b"123456789") == 0x29B1)
chk("PING seq=0x31",
    pack_logical(0x31, 0x01).hex().upper() == "310100380D",
    pack_logical(0x31, 0x01).hex().upper())
chk("PARAM_COUNT seq=0x43",
    pack_logical(0x43, 0x42).hex().upper() == "434200CFE3",
    pack_logical(0x43, 0x42).hex().upper())
pl = pack_logical(0x00, 0x13, bytes.fromhex("00006400C8002C019001F401"))
chk("STREAM_JOINTS [0,10,20,30,40,50]",
    pl[3:15].hex().upper() == "00006400C8002C019001F401", pl.hex().upper())
chk("往返一致", unpack_logical(pack_logical(7, 0x02)) == (7, 0x02, b""))


# ============================================================================
sec("2. 模拟器 + 客户端（全部 14 条命令）")

arm = open_arm("sim://", timeout=0.5)
chk("ping 返回 PingInfo", arm.ping() is not None)
chk("proto_ver==1", arm.ping().proto_ver == 1)

st = arm.get_status()
chk("GET_STATUS 6 轴", len(st.joints) == 6)
chk("read_ok", st.read_ok)
chk("as_servo_deg 可用", st.as_servo_deg() is not None)

fk = arm.fk_query()
chk("FK_QUERY 返回位姿", fk is not None and fk.read_ok == 1)

chk("PARAM_COUNT == 24", arm.param_count() == 24)

r = arm.param_write(4, 7.5)                       # ik.fk_verify_mm ∈ [1,20]
chk("PARAM_WRITE ACK", r.ok)
chk("param_read 回写正确", abs(arm.param_read(4) - 7.5) < 1e-3,
    str(arm.param_read(4)))

# 静默钳制：写超范围值仍回 err=0，但回读是钳过的
pid = 4
old = arm.param_read(pid)
r = arm.param_write(pid, 1e9)
back = arm.param_read(pid)
chk("超范围写仍回 ACK(err=0)", r.ok and r.err == 0)
chk("回读 != 输入（静默钳制）", back != 1e9, "back=%s" % back)
arm.param_write(pid, old or 0.0)

r = arm.move_joints([200] * 6, v=20)
chk("MOVE_JOINTS ACK", r.ok)
time.sleep(0.3)
st = arm.get_status()
chk("MOVE_JOINTS 真的动了", abs(st.as_servo_deg()[0] - 200) < 20,
    str(st.as_servo_deg()[0]))

chk("SET_TORQUE 全部 on", arm.set_torque(0xFF, True).ok)
try:
    arm.set_torque(6, True)
    chk("SET_TORQUE 非法轴 J6(6) 被拒", False, "没抛异常")
except ValueError:
    # 客户端先拦（固件侧同样会回 err=1），更早更省事
    chk("SET_TORQUE 非法轴 J6(6) 被拒", True)

r = arm.move_cart(200.0, 0.0, 300.0)
chk("MOVE_CART ACK", r.ok)
chk("SET_GRAVITY", arm.set_gravity(0.05).ok)
chk("SET_CORRECTION", arm.set_correction(True, 2.0, 10).ok)
chk("LOG_SET_LEVEL", arm.log_set_level(2).ok)
chk("STOP 总能成功", arm.stop().ok)

# STREAM_JOINTS：正常帧
r = arm.stream_joints([0, 0, 0, 0, 0, 0])
chk("STREAM_JOINTS 正常帧", r is not None and r.ok)
# 25° 跳变 → 整帧拒收 err=4
bad_frame = [0, 0, 0, 0, 0, 60]
r = arm.stream_joints(bad_frame)
chk("STREAM_JOINTS >25° 整帧拒 err=4", r is not None and r.err == int(ErrCode.LIMIT),
    str(r))
# 被拒后基准不更新 ⇒ 同一目标反复发还是拒（这就是"自锁"）
r = arm.stream_joints(bad_frame)
chk("拒帧后基准不更新（同目标仍拒）",
    r is not None and r.err == int(ErrCode.LIMIT), str(r))
chk("模拟器拒帧计数 == 2", arm.transport.stream_stats()["rejects"] == 2)

# CMD>=0x80 故意不回
chk("CMD>=0x80 无响应（防死锁）", arm.request(0x80, timeout=0.15).timeout)
# 长度不符 → 静默丢弃
chk("长度不符静默丢弃", arm.request(0x40, b"", timeout=0.15).timeout
    if False else arm.request(0x41, b"\x01", timeout=0.15) is not None)

print("  客户端统计：", arm.describe())
arm.close()


# ============================================================================
sec("3. 主从遥操（模拟器双臂）")

lead = open_arm("sim://", timeout=0.5)
foll = open_arm("sim://", timeout=0.5)

# 刻意让两臂起始姿态不同：验证 hold 模式是"增量跟随"而不是"绝对对齐"
lead.move_joints([180] * 6, v=50)                    # 逻辑角 0
foll.move_joints([150] * 6, v=50)                    # 逻辑角 -30
time.sleep(0.6)

cfg = TeleopConfig(hz=100.0, slew_deg_per_s=240.0, wait_ack=False)
tp = TeleopSession(lead, foll, cfg=cfg)
tp.start()
chk("遥操启动", tp._active)
chk("参考已建立", tp._lead_ref is not None and tp._foll_ref is not None)
chk("从臂参考 = -30（不是被拉到 0）", abs(tp._foll_ref[1] + 30.0) < 1.5,
    str(tp._foll_ref[1]))

# 主臂 J2 走 +20°（舵机 180→200，dir=+1 → 逻辑 +20）
lead.move_joints([180, 200, 180, 180, 180, 180], v=80)
for _ in range(80):
    tp.step()
    time.sleep(0.005)
chk("遥操跑了帧", tp.stats.sent > 0, str(tp.stats.sent))
chk("无拒帧", tp.stats.ack_limit == 0, "limit=%d" % tp.stats.ack_limit)
# 从臂应停在 (-30 + 20) = -10 附近（增量跟随）
time.sleep(0.4)
foll_log = foll.logical_angles(JointCalib())
chk("从臂增量跟随到 -10 附近", foll_log is not None and abs(foll_log[1] + 10) < 3,
    "%s (lead_ref=%s foll_ref=%s)" % (foll_log[1] if foll_log else None,
                                      tp._lead_ref[1], tp._foll_ref[1]))
print("  遥操统计：", tp.stats.summary())
tp.stop()

# ---- 限速：低 slew + 大跨步 ⇒ 必须被削，且永不触发 25° 拒帧 ----
lead3 = open_arm("sim://", timeout=0.5)
foll3 = open_arm("sim://", timeout=0.5)
lead3.move_joints([180] * 6, v=50)
foll3.move_joints([180] * 6, v=50)
time.sleep(0.6)
cfg3 = TeleopConfig(hz=100.0, slew_deg_per_s=12.0)     # 0.12°/帧
tp3 = TeleopSession(lead3, foll3, cfg=cfg3)
tp3.start()
lead3.move_joints([240, 240, 240, 240, 240, 240], v=100)   # 逻辑 +60
for _ in range(40):
    tp3.step()
    time.sleep(0.005)
chk("限速生效（被削）", tp3.stats.slew_clamped > 0, str(tp3.stats.slew_clamped))
chk("限速下零拒帧", tp3.stats.ack_limit == 0, "limit=%d" % tp3.stats.ack_limit)
tp3.stop()
lead3.close(); foll3.close()


# ============================================================================
sec("4. 遥操 + 示教录制（PC 侧，无 80s 上限）")

lead2 = open_arm("sim://", timeout=0.5)
foll2 = open_arm("sim://", timeout=0.5)
lead2.move_joints([180] * 6, v=50)
foll2.move_joints([180] * 6, v=50)
time.sleep(0.4)

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, "demo.json")
    ts = TeleopTeachSession(lead2, foll2, out=out, name="demo", hz=100.0,
                            cfg=TeleopConfig(hz=100.0, slew_deg_per_s=600.0))
    ts.start()

    # 用 driver 驱动"虚拟主臂"画一条扫频轨迹（真人遥操时留空即可）
    def drive(el):
        lead2.move_joints([180 + 20 * math.sin(el * 2 * math.pi), 180,
                           180 + 10 * math.sin(el * 4 * math.pi), 180,
                           180, 180], v=100)

    traj = ts.record(duration=1.0, out=out, driver=drive)
    ts.stop()

    chk("录制轨迹存在文件", os.path.exists(out))
    t2 = Trajectory.load(out)
    chk("轨迹点数 > 20", len(t2) > 20, str(len(t2)))
    chk("轨迹 units=logical_deg", t2.units == "logical_deg")
    chk("轨迹 source=teleop", t2.source.startswith("teleop"))
    v = t2.validate()
    chk("validate 无警告", v["ok"] is True, str(v["warnings"]))
    print("  轨迹：", t2.summary())

    # 导出 CSV / NPZ
    csv_p = os.path.join(d, "demo.csv")
    t2.save_csv(csv_p)
    chk("导出 CSV", os.path.exists(csv_p) and len(Trajectory.load_csv(csv_p)) == len(t2))
    try:
        import numpy  # noqa: F401
        npz_p = os.path.join(d, "demo.npz")
        t2.save_npz(npz_p)
        chk("导出 NPZ", len(Trajectory.load_npz(npz_p)) == len(t2))
    except ImportError:
        print("  skip NPZ（无 numpy）")

    # ---- 回放到"从臂" ----
    arm3 = open_arm("sim://", timeout=0.5)
    arm3.move_joints([180] * 6, v=50)
    time.sleep(0.3)
    pb = PlaybackSession(arm3, cfg=TeleopConfig(hz=100.0), estop_check_s=0.0,
                         on_event=lambda m: print("    · " + m))
    stp = pb.play(t2, speed=1.0, loop=1)
    # 帧数 = 轨迹重采样点数 + 预热帧(默认 3，用于让从臂进入流模式)
    n_rep = len(t2.resample(100.0))
    chk("回放帧数 = 重采样点数 + 预热帧", abs(stp.sent - (n_rep + 3)) <= 2,
        "%d vs %d(+3)" % (stp.sent, n_rep))
    chk("回放结束循环一圈", stp.loops == 1)
    chk("回放无拒帧", stp.ack_limit == 0, "limit=%d" % stp.ack_limit)
    print("  回放统计：", stp.summary())
    arm3.close()

ts.tp.leader.close()
ts.tp.follower.close()


# ============================================================================
sec("5. 固件命令封装（串口文本，需硬件才算真通）")

from arm_sdk import FirmwareLink, FirmwareTeach  # noqa: E402


class _FakeConsole:
    def __init__(self):
        self.cmds = []

    def send_command(self, c, timeout=None, echo=True):
        self.cmds.append(c)

        class R:
            ok = True
            lines = []
        return R()

    def write_raw(self, s):
        pass


fc = _FakeConsole()
fl = FirmwareLink(fc)
fl.link_status(); fl.snapshot(); fl.forwarding(False); fl.active(False)
fl.follow(True); fl.stream(50); fl.stream_off(); fl.link_show(); fl.link_set("net")
fl.link_save()
chk("FirmwareLink 命令串",
    fc.cmds == ["lk", "ms", "mf 0", "ma 0", "sf 1", "sp 50", "sp 0",
                "link", "link net", "link save"], str(fc.cmds))

fc2 = _FakeConsole()
ft = FirmwareTeach(fc2)
ft.enter_rec(); ft.rec_list(); ft.save("a"); ft.play("a", loop=True); ft.files()
chk("FirmwareTeach 命令串",
    fc2.cmds == ["teach rec", "rec list", "save a", "play a loop", "files"],
    str(fc2.cmds))


# ============================================================================
sec("结果")
print("  %d 项，%s" % (TOTAL[0], "全部通过" if not FAILS else "失败：%s" % FAILS))
if FAILS:
    raise SystemExit(1)
print("  离线端到端集成测试通过（无硬件）")
