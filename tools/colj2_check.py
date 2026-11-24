#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""colj2 离线验证 —— J2 壳体包线 (gripper vs J2 housing keep-out) 参数标定。

背景 (2026-09-29 现场两次碰撞):
  姿态1: FK x=102 y=5 z=99 pitch=142 (含 C 0 0 8)   -> 夹爪初端撞 J2 顶部外壳
         用户实测: J2 舵机顶部外壳离桌 131.38mm
  姿态2: FK x=92 y=-28 z=3 pitch=171 (含 C 0 0 8)   -> 夹爪初端撞 J2 前部外壳
         用户实测: J1 轴心到 J2 前部外壳水平距离 51.05mm
夹爪 = 等腰三角形箭头, 中位姿态立着指向正前方。

本脚本复刻固件 FK (kinematics.h 当前值) + 拟新增的 colj2 包线检查,
验证: 两个碰撞姿态必须被拦住; 折叠/中位/待命/挥动 及其插补路径不能被新增拦。
"""
import math
import sys

# ---- kinematics.h 当前值 (逐字) ----
BASE_H   = 63.13
J2_OFF_Y = 26.70
J2_OFF_Z = 46.92
L  = [112.45, 134.13, 61.55 + 104.35]      # UPPER, FORE, WRIST+TOOL
B  = [0.243787, 1.532268, 1.532268]        # PSI_UPPER0, PSI_FORE0, DISTAL0
L_TOOL = 104.35
D_J6   = 23.58                              # J5->J6

WOX, WOY, WOZ = 0.0, 0.0, 8.0               # 现场 C 0 0 8
ZF_PREF = 2.0                               # 现场 zf 2

# ---- 拟定 colj2 默认参数 (可在此调参后同步进固件) ----
RHO_MAX = 65.0    # J2 前部外壳 51.05 + 余量
Z_TOP   = 143.5   # J2 顶部外壳 131.38 + 余量
W       = 18.0    # 夹爪横向半宽 (三角形箭头半高, 绕工具轴旋转)
D_SAMP  = (D_J6, 50.0, 80.0, L_TOOL)        # 沿工具轴采样点 (J6/中段/TCP)

TRAJ_COL_R, TRAJ_COL_Z = 25.0, 120.0        # 既有立柱圆柱
HOME_DOCK_TOL = 3.0

POSE_HOME_J  = [0.75, -103.38, 90.20, 70.68, 2.29, -34.29]
POSE_FOLD_J  = [0, 40, -70, 40, 0, 0]
POSE_READY_J = [0.97, -49.05, 27.08, 44.31, 2.81, -39.56]
POSE_WAVE1_J = [-87.47, 14.15, -69.27, 26.20, 2.90, -39.56]
POSE_WAVE2_J = [-89.90, -42.02, -70.86, 22.68, 2.55, -39.56]
POSE_MID_J   = [0, 0, 0, 0, 0, 0]


def fk(j):
    t1 = math.radians(j[0])
    s = 0.0
    py = pz = 0.0
    for i in range(3):
        s += math.radians(j[i + 1])
        py += L[i] * math.sin(B[i] + s)
        pz += L[i] * math.cos(B[i] + s)
    py += J2_OFF_Y
    pz += BASE_H + J2_OFF_Z
    x = py * math.cos(t1)
    y = py * math.sin(t1)
    return x, y, pz, math.degrees(B[1] + s)


def colj2_violation(rho, z, yaw_deg, rho_max=RHO_MAX, z_top=Z_TOP, w=W):
    """拟新增检查: 夹爪采样点 (沿轴 d + 轴向±w/面内垂直±w) 是否进 J2 壳体包线。
    返回 None=安全, 否则返回描述字符串。"""
    yr = math.radians(yaw_deg)
    dx, dz = math.sin(yr), math.cos(yr)          # 工具方向 (臂平面内)
    px, pz_ = math.cos(yr), -math.sin(yr)        # 面内垂直方向
    tcp = (rho, z)
    for d in D_SAMP:
        bx = tcp[0] - (L_TOOL - d) * dx
        bz = tcp[1] - (L_TOOL - d) * dz
        cand = [(bx, bz),
                (bx + w * dx, bz + w * dz), (bx - w * dx, bz - w * dz),
                (bx + w * px, bz + w * pz_), (bx - w * px, bz - w * pz_)]
        for (qx, qz) in cand:
            if 0.0 <= qx < rho_max and qz < z_top:
                return "pt d=%.1f (%.1f,%.1f) 进包线 rho<%.1f z<%.1f" % (
                    d, qx, qz, rho_max, z_top)
    return None


def is_home_dock(j):
    return all(abs(j[k] - POSE_HOME_J[k]) <= HOME_DOCK_TOL for k in range(6))


def path_report(name, a, b, colj2_on=True):
    """复刻 fk_path_z_ok: 24 段插补 + 白名单 + raw 略 + 桌面 + 立柱 + colj2"""
    N = 24
    new_blocks = []
    other_fail = None
    for i in range(N + 1):
        r = i / N
        q = [a[k] + (b[k] - a[k]) * r for k in range(6)]
        if is_home_dock(q):
            continue
        x, y, z, yaw = fk(q)
        rho = math.hypot(x - WOX, y - WOY)
        zb = z - WOZ
        floor = ZF_PREF - WOZ
        if zb < floor:
            other_fail = "i=%d 桌面 z=%.1f<floor %.1f" % (i, zb, floor)
            break
        if abs(rho) < TRAJ_COL_R and zb < TRAJ_COL_Z:
            other_fail = "i=%d 立柱 rho=%.1f z=%.1f" % (i, rho, zb)
            break
        if colj2_on:
            v = colj2_violation(rho, zb, yaw)
            if v:
                new_blocks.append("i=%d %s" % (i, v))
                break
    tag = "colj2=ON " if colj2_on else "colj2=OFF"
    if new_blocks:
        print("  [%s] %-22s ❌ 新增拦截: %s" % (tag, name, new_blocks[0]))
        return False
    if other_fail and not colj2_on:
        print("  [%s] %-22s (既有检查拦: %s — 非本次新增)" % (tag, name, other_fail))
        return True
    print("  [%s] %-22s ✅ 通过" % (tag, name))
    return True


def pose_report(name, j, tcp_override=None, yaw_override=None, expect_block=False):
    if tcp_override:
        x, z, yaw = tcp_override
        rho, zb, yw = x, z - WOZ, yaw
    else:
        x, y, z, yaw = fk(j)
        rho, zb = math.hypot(x - WOX, y - WOY), z - WOZ
        yw = yaw
    dock = " [折叠白名单]" if (not tcp_override and is_home_dock(j)) else ""
    v = colj2_violation(rho, zb, yw)
    blocked = v is not None
    good = (blocked == expect_block)
    print("  %-12s TCP=(%.1f,%.1f) yaw=%.0f%s -> %s%s" % (
        name, rho, zb, yw, dock,
        ("❌ 拦: " + v) if blocked else "✅ 安全",
        "" if good else "  ⚠ 与预期不符!"))
    return good


def main():
    ok = True
    print("=" * 76)
    print("一、两个碰撞姿态必须被拦住 (由 f 回报的 TCP+pitch 直接重构)")
    print("-" * 76)
    ok &= pose_report("碰撞姿态1", None, tcp_override=(102.0, 99.0, 142.0), expect_block=True)
    ok &= pose_report("碰撞姿态2", None, tcp_override=(92.0, 3.0, 171.0), expect_block=True)

    print()
    print("=" * 76)
    print("二、单点安全姿态不能被拦")
    print("-" * 76)
    for name, j in (("折叠HOME", POSE_HOME_J), ("备用FOLD", POSE_FOLD_J),
                    ("中位MID", POSE_MID_J), ("待命READY", POSE_READY_J),
                    ("挥动WAVE1", POSE_WAVE1_J), ("挥动WAVE2", POSE_WAVE2_J)):
        pose_report(name, j)

    print()
    print("=" * 76)
    print("三、插补路径: colj2 OFF(基线) vs ON 对比, 只关心新增拦截")
    print("-" * 76)
    paths = [("折叠->中位", POSE_HOME_J, POSE_MID_J),
             ("中位->折叠", POSE_MID_J, POSE_HOME_J),
             ("折叠->待命", POSE_HOME_J, POSE_READY_J),
             ("待命->中位", POSE_READY_J, POSE_MID_J),
             ("中位->待命", POSE_MID_J, POSE_READY_J),
             ("WAVE1->WAVE2", POSE_WAVE1_J, POSE_WAVE2_J),
             ("FOLD备用->中位", POSE_FOLD_J, POSE_MID_J)]
    for name, a, b in paths:
        path_report(name, a, b, colj2_on=False)
        ok &= path_report(name, a, b, colj2_on=True)

    print()
    print("=" * 76)
    print("结论: %s (RHO_MAX=%.1f Z_TOP=%.1f W=%.1f)" % (
        "✅ 参数可用" if ok else "❌ 需调参", RHO_MAX, Z_TOP, W))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
