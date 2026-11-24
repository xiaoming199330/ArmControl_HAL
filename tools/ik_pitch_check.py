#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_pitch_check.py — 复刻 Core/Src/kinematics.c 的平面解析 IK / FK，
回答：“g x y z pitch 为什么 pitch 达不成、是哪个关节限位挡住了、改成多少才能通”。

与固件同一套几何与判据：常量照抄 Core/Inc/kinematics.h；平面 FK、ik_analytic、
限位判据与 C 代码逐行等价。关节角用“相对中性位”的 t(deg) 表达；
换算成舵机度 = ZERO + DIR*t（ZERO 默认 180 = raw 2048）。

用法:
  python tools/ik_pitch_check.py                      # 复盘现场那 4 个点
  python tools/ik_pitch_check.py --lim 4:83:200       # 按实测限位改某一轴
  python tools/ik_pitch_check.py -t 260 0 300 90      # 单个目标
  python tools/ik_pitch_check.py --pitch-window 260 0 300
  python tools/ik_pitch_check.py --z-window 260 0 90  # 固定 pitch, 扫 z 看能到多高
  python tools/ik_pitch_check.py --j4-fit 260 0 300   # 反推 J4 上限要多少才通
"""
import math
import argparse

# ---- 几何常量 (kinematics.h) ----
BASE_H, J2_OFF_Y, J2_OFF_Z = 65.30, 31.32, 54.80
L_UPPER, L_FORE, L_WRIST, L_TOOL = 116.00, 135.00, 62.50, 105.30
L_DISTAL = L_WRIST + L_TOOL                                   # 167.80
PSI_UPPER0 = math.degrees(math.atan2(28.0, 112.57))           # +13.97
PSI_FORE0 = math.degrees(math.atan2(134.9, 5.2))              # +87.79
PSI_DISTAL0 = PSI_FORE0
MAX_REACH = L_UPPER + L_FORE + L_DISTAL                       # 418.80

ZERO = [180.0] * 6                          # 舵机零位 (servo deg)
DIR = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]       # JOINT_DIR

# 限位 (servo deg, min<max; J3 的包裹弧按等价常规区间近似)
DEFAULT_LIM = [(0, 360), (58, 264), (10, 177), (83, 289), (180, 340), (0, 360)]
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def planar_fk(t2, t3, t4):
    s = 0.0
    y = z = 0.0
    for ln, base, t in zip((L_UPPER, L_FORE, L_DISTAL),
                           (PSI_UPPER0, PSI_FORE0, PSI_DISTAL0), (t2, t3, t4)):
        s += t
        y += ln * math.sin(math.radians(base + s))
        z += ln * math.cos(math.radians(base + s))
    return y, z


def ik_analytic(ty, tz, pitch, lim):
    """返回 (旁解列表, 不可达原因)。旁解: t/servo/err/bad/name"""
    j4y = ty - L_DISTAL * math.sin(math.radians(pitch))
    j4z = tz - L_DISTAL * math.cos(math.radians(pitch))
    d = math.hypot(j4y, j4z)
    if d < 1e-3:
        return [], "d≈0"
    if d > L_UPPER + L_FORE + 0.01:
        return [], f"腕轴目标点超出 2R 臂展 (d={d:.1f} > {L_UPPER+L_FORE:.1f})"
    if d < abs(L_UPPER - L_FORE):
        return [], f"腕轴目标点太近 (d={d:.1f} < {abs(L_UPPER-L_FORE):.1f})"

    beta = math.degrees(math.acos(max(-1.0, min(1.0,
        (L_UPPER ** 2 + d ** 2 - L_FORE ** 2) / (2 * L_UPPER * d)))))
    gamma = math.degrees(math.atan2(j4y, j4z))

    out = []
    for side in (0, 1):
        psa = gamma + (-beta if side else beta)
        j3y, j3z = L_UPPER * math.sin(math.radians(psa)), L_UPPER * math.cos(math.radians(psa))
        psb = math.degrees(math.atan2(j4y - j3y, j4z - j3z))
        t2 = psa - PSI_UPPER0
        t3 = (psb - PSI_FORE0) - t2
        t4 = pitch - PSI_FORE0 - (t2 + t3)
        py, pz = planar_fk(t2, t3, t4)
        err = math.hypot(py - ty, pz - tz)
        servo = [ZERO[i + 1] + DIR[i + 1] * t for i, t in enumerate((t2, t3, t4))]
        bad = [f"{NAMES[i+1]}={servo[i]:.1f} 越界[{lim[i+1][0]:g},{lim[i+1][1]:g}]"
               for i in range(3) if not (lim[i + 1][0] <= servo[i] <= lim[i + 1][1])]
        out.append(dict(name="肘上" if side == 0 else "肘下", t=(t2, t3, t4),
                        servo=servo, err=err, bad=bad))
    return out, ""


def reachable(x, y, z, pitch, lim):
    rho = math.hypot(x, y)
    sols, _ = ik_analytic(rho - J2_OFF_Y, z - (BASE_H + J2_OFF_Z), pitch, lim)
    return any(not s["bad"] for s in sols)


def _runs(vals, step):
    if not vals:
        return ""
    out, a, prev = [], vals[0], vals[0]
    for v in vals[1:]:
        if v != prev + step:
            out.append((a, prev))
            a = v
        prev = v
    out.append((a, prev))
    return "  ".join(f"{p}~{q}deg" if p != q else f"{p}deg" for p, q in out)


def report(x, y, z, pitch, lim):
    rho = math.hypot(x, y)
    ty, tz = rho - J2_OFF_Y, z - (BASE_H + J2_OFF_Z)
    print(f"\n=== g {x} {y} {z} {pitch} ===")
    print(f"  相对 J2 轴的平面目标: 前向 {ty:.1f} mm, 高度 {tz:.1f} mm, 距 J2 轴 {math.hypot(ty,tz):.1f} mm")
    print(f"  工具指向 {pitch}deg ⇒ 腕(J4)轴必须位于 目标 - 167.8mm*u(pitch) = "
          f"({ty - L_DISTAL*math.sin(math.radians(pitch)):.1f}, {tz - L_DISTAL*math.cos(math.radians(pitch)):.1f})")
    sols, why = ik_analytic(ty, tz, pitch, lim)
    if not sols:
        print(f"  ✘ 解析 IK 直接返回 0: {why}")
        print("  ⇒ 落 CCD 位置兜底 → pitch 丢失")
        return
    for s in sols:
        t2, t3, t4 = s["t"]
        v = "接受 ✔ (pitch 可达)" if not s["bad"] else "被拒 ✘ " + "; ".join(s["bad"])
        print(f"  {s['name']}: t=({t2:+7.2f},{t3:+7.2f},{t4:+7.2f})deg → "
              f"J2={s['servo'][0]:6.1f} J3={s['servo'][1]:6.1f} J4={s['servo'][2]:6.1f} "
              f"| FKerr={s['err']:.4f}mm | {v}")
    if not any(not s["bad"] for s in sols):
        print("  ⇒ 两个旁解都被限位拒 ⇒ IK_Solve 降级 CCD(只保位置, pitch 自由)")
        print("     ⇒ FK 位置仍为 0mm, 但 verify.yaw ≠ req ⇒ g 的俯仰守卫取消动作")


def show_lim(lim):
    print("  当前限位(servo deg): " + "  ".join(
        f"{NAMES[i]}[{lim[i][0]:g}~{lim[i][1]:g}]" for i in range(6)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lim", action="append", default=[],
                    help="覆盖限位, 形如 4:83:200 (关节号从 1 开始)")
    ap.add_argument("-t", "--target", nargs=4, type=float, metavar=("X", "Y", "Z", "PITCH"))
    ap.add_argument("--pitch-window", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--z-window", nargs=3, type=float, metavar=("X", "Y", "PITCH"))
    ap.add_argument("--j4-fit", nargs=3, type=float, metavar=("X", "Y", "Z"))
    a = ap.parse_args()

    lim = list(DEFAULT_LIM)
    for spec in a.lim:
        j, mn, mx = spec.split(":")
        lim[int(j) - 1] = (float(mn), float(mx))

    if a.j4_fit:
        x, y, z = (int(v) for v in a.j4_fit)
        print(f"\n--- 反推: 目标 ({x},{y},{z}) 各 pitch 下 J4 至少要放到多大 ---")
        for p in (80, 85, 90, 95, 100, 105, 110):
            rho = math.hypot(x, y)
            sols, _ = ik_analytic(rho - J2_OFF_Y, z - (BASE_H + J2_OFF_Z), p, lim)
            need = [f"{s['name']}需 J3={s['servo'][1]:.0f}, J4={s['servo'][2]:.0f}"
                    for s in sols]
            okp = "可达" if any(not s["bad"] for s in sols) else "不可达"
            print(f"  pitch={p:3d}deg: {okp:6s}  " + " | ".join(need))
        return

    if a.z_window:
        x, y, pitch = a.z_window
        print(f"\n--- 固定 x={int(x)} y={int(y)} pitch={int(pitch)}, 扫 z 看工具能否保持水平 ---")
        show_lim(lim)
        good = [z for z in range(150, 461, 10) if reachable(x, y, z, pitch, lim)]
        print("  pitch 可达的 z: " + (_runs(good, 10) or "无"))
        print("  (腕轴 J4 的高度 = z - 167.8*cos(pitch); pitch=90 时 J4 必须与 TCP 等高)")
        return

    if a.pitch_window:
        x, y, z = a.pitch_window
        print(f"\n--- 目标 ({int(x)},{int(y)},{int(z)}) 可达 pitch 窗口 ---")
        show_lim(lim)
        good = [p for p in range(0, 181, 5) if reachable(x, y, z, p, lim)]
        print("  可达 pitch: " + (_runs(good, 5) or "0~180 全部不可达"))
        return

    cases = [a.target] if a.target else [
        (260, 0, 300, 90), (250, 0, 300, 90), (280, 0, 300, 90), (230, 0, 300, 90)]
    show_lim(lim)
    for c in cases:
        report(int(c[0]), int(c[1]), int(c[2]), int(c[3]), lim)


if __name__ == "__main__":
    main()
