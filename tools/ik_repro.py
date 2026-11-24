#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ik_repro.py — 逐行复刻 ArmControl_HAL 固件 `g <x> <y> <z> <pitch>` 的判定链。

复刻对象 (照抄固件常量与公式, 行号对应源码):
  kinematics.h  ROBOT_* 几何 / JOINT_DIR
  main.c:1211   apply_calib_limits()  (含 J3/J5 zero-相对偏移)
  main.c:1285   开机顺序: apply_calib_limits() 在 Kinematics_SetZeroRaw() 之前 ← 时序 bug
  kinematics.c:87   IK_SetJointLimitsRaw()
  kinematics.c:216  ik_analytic()  (肘上/肘下两解 + jlim + FK 回代)
  kinematics.c:349  IK_Solve()  Step1/2/3a/3b
  main.c:3255   cmdGoto(): rad_to_servo_deg -> raw_in_range 检查 -> pitch 校验

用法:
  python tools/ik_repro.py -t 200 150 200 90
  python tools/ik_repro.py -t 200 150 200 90 --zero 0 0 0 0 0 0 --phase2 0
  python tools/ik_repro.py -t 200 150 200 90 --zero 2014 896 3106 2331 2022 1567
"""
import argparse
import math

# ---------------- 几何 (Core/Inc/kinematics.h) ----------------
ROBOT_BASE_H = 65.30
ROBOT_J2_OFF_Y = 31.32
ROBOT_J2_OFF_Z = 54.80
ROBOT_L_UPPER = 116.00
ROBOT_L_FORE = 135.00
ROBOT_L_DISTAL = 167.80
ROBOT_PSI_UPPER0 = 0.243787
ROBOT_PSI_FORE0 = 1.532268
ARM_MAX_REACH = ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL

JOINT_DIR = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
KIN_LEN = [ROBOT_L_UPPER, ROBOT_L_FORE, ROBOT_L_DISTAL]
KIN_BASE = [ROBOT_PSI_UPPER0, ROBOT_PSI_FORE0, ROBOT_PSI_FORE0]

KIN_IK_FAIL_MM = 1.0
KIN_FK_VERIFY_MM = 5.0
DEF_MIN = [0.0, 58.0, 10.0, 83.0, 180.0, 0.0]
DEF_MAX = [360.0, 264.0, 177.0, 289.0, 340.0, 360.0]
J3_OFF_BACK, J3_OFF_FRONT = -1163, 1090
J5_OFF_MIN, J5_OFF_MAX = -1033, 787

D = math.degrees


def wrap_raw(v):
    return v % 4096


def raw_dist_fwd(a, b):
    return (b + 4096 - a) % 4096


def deg_to_servo_raw(deg):
    deg = max(0.0, min(360.0, deg))
    return int(deg / 360.0 * 4095.0 + 0.5)


def servo_raw_to_deg(raw):
    return raw / 4095.0 * 360.0


def raw_in_range(raw, mn, mx):
    span = raw_dist_fwd(mn, mx)
    if span == 0:
        return False
    return raw_dist_fwd(mn, raw) <= span


def planar_fk(ang):
    s = 0.0
    y = z = 0.0
    for i in range(3):
        s += ang[i]
        y += KIN_LEN[i] * math.sin(KIN_BASE[i] + s)
        z += KIN_LEN[i] * math.cos(KIN_BASE[i] + s)
    return y, z


def apply_calib_limits(zero, phase2, zero_first):
    """复刻 main.c:1211。zero_first=True 表示 SERVO_OFFSET 已在 SetZeroRaw 后被更新
    (即修好顺序后的行为); False = 固件现状 (apply 先跑, 用默认 180°)。"""
    off_new = [180.0] * 6
    for j in range(6):
        z = zero[j]
        if z and z < 4096:
            off_new[j] = z / 4095.0 * 360.0
    off_used = off_new if zero_first else [180.0] * 6

    cmax = [0] * 6
    cmin = [deg_to_servo_raw(DEF_MIN[j]) if phase2[j] else 4095 for j in range(6)]
    cmax = [deg_to_servo_raw(DEF_MAX[j]) if phase2[j] else 0 for j in range(6)]

    JMIN = [0] * 6
    JMAX = [0] * 6
    skip = []
    for j in range(6):
        if j == 4:
            continue
        if j == 2:
            if zero[2]:
                mn, mx = wrap_raw(zero[2] + J3_OFF_BACK), wrap_raw(zero[2] + J3_OFF_FRONT)
            else:
                mn, mx = deg_to_servo_raw(77.8), deg_to_servo_raw(275.9)
        else:
            mn, mx = cmin[j], cmax[j]
        span = raw_dist_fwd(mn, mx)
        if span == 0 or span < 20:
            skip.append(j + 1)
            continue
        JMIN[j], JMAX[j] = mn, mx
    if zero[4]:
        JMIN[4], JMAX[4] = wrap_raw(zero[4] + J5_OFF_MIN), wrap_raw(zero[4] + J5_OFF_MAX)
    else:
        JMIN[4], JMAX[4] = deg_to_servo_raw(180.0), deg_to_servo_raw(340.0)

    jmin = [-150.0] * 6
    jmax = [150.0] * 6
    for i in range(6):
        span = raw_dist_fwd(JMIN[i], JMAX[i])
        if span == 0:
            continue
        dmn = servo_raw_to_deg(JMIN[i])
        dmx = dmn + span * 360.0 / 4095.0
        a = JOINT_DIR[i] * (dmn - off_used[i])
        b = JOINT_DIR[i] * (dmx - off_used[i])
        jmin[i], jmax[i] = min(a, b), max(a, b)
    return JMIN, JMAX, jmin, jmax, off_new, skip


def ik_analytic(ty, tz, pitch, jmin, jmax):
    j4y = ty - ROBOT_L_DISTAL * math.sin(pitch)
    j4z = tz - ROBOT_L_DISTAL * math.cos(pitch)
    d = math.hypot(j4y, j4z)
    if d < 1e-3:
        return None, "d<1e-3"
    if d > ROBOT_L_UPPER + ROBOT_L_FORE + 0.01:
        return None, "d=%.2f > 251.00 (超平面臂展)" % d
    if d < abs(ROBOT_L_UPPER - ROBOT_L_FORE):
        return None, "d=%.2f < 19.00" % d
    cosb = max(-1.0, min(1.0, (ROBOT_L_UPPER ** 2 + d * d - ROBOT_L_FORE ** 2)
                         / (2.0 * ROBOT_L_UPPER * d)))
    beta, gamma = math.acos(cosb), math.atan2(j4y, j4z)
    rows = []
    for side in (0, 1):
        psa = gamma + (-beta if side else beta)
        j3y, j3z = ROBOT_L_UPPER * math.sin(psa), ROBOT_L_UPPER * math.cos(psa)
        psb = math.atan2(j4y - j3y, j4z - j3z)
        t2 = psa - ROBOT_PSI_UPPER0
        t3 = (psb - ROBOT_PSI_FORE0) - t2
        t4 = pitch - ROBOT_PSI_FORE0 - (t2 + t3)
        py, pz = planar_fk([t2, t3, t4])
        err = math.hypot(py - ty, pz - tz)
        rej = []
        for nm, idx, val in (("J2", 1, t2), ("J3", 2, t3), ("J4", 3, t4)):
            if not (jmin[idx] <= val <= jmax[idx]):
                rej.append("%s %+.2f deg out [%+.2f,%+.2f]" % (nm, D(val), D(jmin[idx]), D(jmax[idx])))
        if err > KIN_IK_FAIL_MM:
            rej.append("FKerr %.2f>%.1f" % (err, KIN_IK_FAIL_MM))
        rows.append(dict(name="肘上" if side == 0 else "肘下", t2=D(t2), t3=D(t3),
                         t4=D(t4), err=err, rej=rej))
    return rows, None


def run(x, y, z, pitch_deg, zero, phase2, zero_first, label):
    JMIN, JMAX, jmin, jmax, off, skip = apply_calib_limits(zero, phase2, zero_first)
    print("-" * 74)
    print(label)
    print("  SERVO_OFFSET(实际用于 IK 输出)= [%s]" % ", ".join("%7.2f" % v for v in off))
    print("  JOINT_MIN_RAW = [%s]" % ", ".join("%5d" % v for v in JMIN))
    print("  JOINT_MAX_RAW = [%s]" % ", ".join("%5d" % v for v in JMAX))
    if skip:
        print("  (跳过的关节 = RAW 0/0 未校准 -> IK 限位保持默认 +-150: J%s)" % ",J".join(map(str, skip)))
    print("  jlim(deg)  J2=[%8.2f,%8.2f]  J3=[%8.2f,%8.2f]  J4=[%8.2f,%8.2f]"
          % (D(jmin[1]), D(jmax[1]), D(jmin[2]), D(jmax[2]), D(jmin[3]), D(jmax[3])))

    tool_pitch = math.radians(pitch_deg)
    theta1 = math.atan2(y, x)
    rho = math.hypot(x, y)
    ty, tz = rho - ROBOT_J2_OFF_Y, z - (ROBOT_BASE_H + ROBOT_J2_OFF_Z)
    pd = math.hypot(ty, tz)
    print("  J1=%.2f deg   ty=%.2f tz=%.2f  planar_dist=%.2f (<%.2f OK)"
          % (D(theta1), ty, tz, pd, ARM_MAX_REACH))

    res, err = ik_analytic(ty, tz, tool_pitch, jmin, jmax)
    if err:
        print("  解析解返回 0 (%s) -> 走 CCD" % err)
        return
    ok = None
    for r in res:
        tag = "采纳" if not r["rej"] else "拒绝"
        print("   [%s] %s: J2=%+8.2f J3=%+8.2f J4=%+8.2f FKerr=%.4fmm %s"
              % (tag, r["name"], r["t2"], r["t3"], r["t4"], r["err"],
                 ("<-- " + "; ".join(r["rej"])) if r["rej"] else ""))
    ok = [r for r in res if not r["rej"]]
    if not ok:
        print("   ==> 两解全拒 -> 降级 CCD (位置对, pitch 自由)  ← 现场现象")
        return
    b = min(ok, key=lambda r: r["err"])
    print("   ==> 解析解采纳 (%s)" % b["name"])
    jdeg = [D(theta1), b["t2"], b["t3"], b["t4"]]
    deg = [off[j] + JOINT_DIR[j] * jdeg[j] for j in range(4)]
    print("       打印应为: J1=%d J2=%d J3=%d J4=%d"
          % (round(deg[0]), round(deg[1]), round(deg[2]), round(deg[3])))
    bad = []
    for j in range(4):
        raw = deg_to_servo_raw(deg[j])
        if not raw_in_range(raw, JMIN[j], JMAX[j]):
            bad.append("J%d raw=%d not in [%d,%d]" % (j + 1, raw, JMIN[j], JMAX[j]))
    if bad:
        print("       ★ raw 检查失败 -> 固件会打印 'IK: solution out of joint limits!':")
        for s in bad:
            print("           %s" % s)
    else:
        print("       raw 检查通过 -> 应打印 'IK: x=.. -> J1..' 且 pitch 达标")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--target", nargs=4, type=float, action="append", required=True)
    ap.add_argument("--zero", nargs=6, type=int, default=[0, 0, 0, 0, 0, 0])
    ap.add_argument("--phase2", default="0 1 0 1 0 1",
                    help="每个关节是否有有效 Phase2 极值(默认按 L 输出推断: J2/J4/J6 有效)")
    a = ap.parse_args()
    ph = [bool(int(v)) for v in a.phase2.split()]
    for t in a.target:
        print("=" * 74)
        print("目标 g %g %g %g %g" % tuple(t))
        run(t[0], t[1], t[2], t[3], a.zero, ph, False,
            "【固件现状】apply_calib_limits() 先用 SERVO_OFFSET=180 算 jlim (时序 bug)")
        run(t[0], t[1], t[2], t[3], a.zero, ph, True,
            "【修好顺序】jlim 用更新后的 SERVO_OFFSET")


if __name__ == "__main__":
    main()
