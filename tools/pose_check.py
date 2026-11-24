#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位姿常量体检 —— 直接从 Core/Src/main.c 解析 POSE_*_J[6], 用固件同款 FK 公式
算出爪尖位置与工具俯仰, 再对关键项做安全检查。

用途: 手改 POSE_HOME_J / POSE_FOLD_J / POSE_READY_J / POSE_WAVE1_J / POSE_WAVE2_J
之后, 上机前先离线看一眼 —— 不用烧固件、不用动臂。

几何常量与 Core/Inc/kinematics.h 逐字一致; FK 公式复刻 kinematics.c 的
planar_fk() + FK_Solve()。输入是**关节角 (rel-home, 度)**, 与固件同一口径。

用法:
    py -3 tools/pose_check.py                     # 默认读 Core/Src/main.c
    py -3 tools/pose_check.py --src <main.c>
    py -3 tools/pose_check.py --zero 2048         # 指定 calib_zero_raw (默认 2048)
"""
import argparse
import math
import re
import sys

# ---- kinematics.h (逐字) ----
BASE_H = 65.30
J2_OFF_Y = 31.32
J2_OFF_Z = 54.80
L = [116.00, 135.00, 167.80]
B = [0.243787, 1.532268, 1.532268]

# 折叠收纳位旁的安全下限: 模型 z 低于此值就值得人工复核 (mm)
Z_REVIEW_MM = 12.0


def fk(j):
    """关节角(度) -> (x, y, z, pitch_deg, rho)"""
    t1 = math.radians(j[0])
    s = 0.0
    py = pz = 0.0
    for i in range(3):
        s += math.radians(j[i + 1])
        py += L[i] * math.sin(B[i] + s)
        pz += L[i] * math.cos(B[i] + s)
    py += J2_OFF_Y
    pz += BASE_H + J2_OFF_Z
    return py * math.cos(t1), py * math.sin(t1), pz, math.degrees(B[1] + s), math.hypot(py, pz)


def parse(src):
    """从 main.c 抓 static const float POSE_XXX_J[6] = { a, b, ... };"""
    txt = open(src, encoding='utf-8', errors='replace').read()
    pat = re.compile(
        r'static\s+const\s+float\s+(POSE_\w+?_J)\s*\[6\]\s*=\s*\{([^}]*)\}\s*;\s*(/\*.*?\*/)?',
        re.S)
    out = []
    for m in pat.finditer(txt):
        raw = m.group(2).replace('f', '')
        vals = [float(v) for v in re.split(r'[,\s]+', raw.strip()) if v]
        if len(vals) != 6:
            print("  !! %s 解析出 %d 个数, 跳过" % (m.group(1), len(vals)))
            continue
        out.append((m.group(1), vals, (m.group(3) or '').strip()))
    return out


def parse_macro(name, paths=('Core/Src/main.c', 'config/arm_config.h')):
    """按名字取 #define 的浮点值; 返回 (值, 出处)。找不到返回 (None, None)。

    值一律现读, 不在这里硬编码 —— 否则又变成一条会过期的记忆
    (2026-09-16 已踩过: 工具里写死的老限位与实际配置脱节)。"""
    pat = re.compile(r'^\s*#define\s+' + re.escape(name)
                     + r'\s+\(?\s*(-?[0-9.]+)f?', re.M)
    for p in paths:
        try:
            with open(p, encoding='utf-8', errors='replace') as fh:
                s = fh.read()
        except IOError:
            continue
        m = pat.search(s)
        if m:
            return float(m.group(1)), p
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='Core/Src/main.c')
    ap.add_argument('--zero', type=int, default=2048,
                    help='calib_zero_raw (默认 2048); 只影响 J3/J5 的 raw 窗口换算')
    a = ap.parse_args()

    poses = parse(a.src)
    if not poses:
        print("没解析到位姿常量, 检查 --src")
        return 1

    print("位姿常量体检  (%s)" % a.src)
    print("=" * 78)
    print("%-16s %7s %7s %7s %8s %7s   %s"
          % ("常量", "x", "y", "z", "rho", "pitch", "备注"))
    print("-" * 78)
    warn = []
    for name, j, cmt in poses:
        x, y, z, pitch, rho = fk(j)
        note = ""
        if z < Z_REVIEW_MM:
            note = "z 偏低, 复核桌面余量"
            warn.append((name, note))
        print("%-16s %7.1f %7.1f %7.1f %8.1f %7.1f   %s"
              % (name, x, y, z, rho, pitch, note))

    # ---- 桌面下限推算: 改 POSE_HOME_J 的最大连带影响就在这一条 ----
    mg, mg_src = parse_macro('TRAJ_Z_MARGIN_MM')
    tol, _ = parse_macro('HOME_DOCK_TOL_DEG')
    home = [j for n, j, _ in poses if 'HOME' in n]
    print()
    print("桌面下限推算  (traj_z_floor_mm = FK(POSE_HOME_J).z - TRAJ_Z_MARGIN_MM)")
    print("-" * 78)
    if not home:
        print("  没找到 POSE_HOME_J")
    elif mg is None:
        print("  没解析到 TRAJ_Z_MARGIN_MM")
    else:
        _, _, zhome, _, _ = fk(home[0])
        print("  POSE_HOME_J 模型爪尖 z   = %8.2f mm" % zhome)
        print("  TRAJ_Z_MARGIN_MM        = %8.2f mm    (%s)" % (mg, mg_src))
        print("  => traj_z_floor_mm      = %8.2f mm    <== 全局桌面下限" % (zhome - mg))
        print()
        print("  真实含义: 爪尖不得低于【停靠位真实高度 - %.1fmm】。锚点与路径检查" % mg)
        print("  用的是同一套 FK, 模型绝对偏置自动抵消 ⇒ 模型 z 准不准都不影响该")
        print("  下限的安全性; 但**别**拿 f 打印的 z 当桌面间隙读数 (09-11 记模型")
        print("  4mm/实测 10mm, 09-16 记模型 19.5mm/实测 10mm, 两次符号相反)。")
        if tol is not None:
            def tcp(j):
                x, y, z, _, _ = fk(j)
                return (x, y, z)
            base = tcp(home[0])
            worst, which = 0.0, ''
            for i, ax in ((0, 'J1'), (1, 'J2'), (2, 'J3'), (3, 'J4')):
                for s in (1, -1):
                    j = list(home[0])
                    j[i] += s * tol
                    d = math.dist(base, tcp(j))
                    if d > worst:
                        worst, which = d, '%s%+g' % (ax, s * tol)
            print()
            print("  折叠位白名单 (HOME_DOCK_TOL_DEG=%.1f°): 该段内爪尖最多可偏移 %.1f mm"
                  % (tol, worst))
            print("  (%s° 单轴推到边界), 这段路径**跳过**桌面 z 与 raw 限位检查。" % which)

    print()
    print("注释里的旧值 (仅供核对是否已过期)")
    print("-" * 78)
    for name, j, cmt in poses:
        if cmt:
            print("  %-16s %s" % (name, cmt))

    print()
    print("J3 / J5 关节角合法区自检  (zero=%d)" % a.zero)
    print("-" * 78)
    # ⚠ 2026-09-16 政策变更: J3/J5 的内置偏移宏 (J3_LIMIT_OFF_BACK/FRONT、
    #   J5_LIMIT_OFF_MIN/MAX) 连同运行期开关 hm **已从 main.c 删除** ——
    #   六轴现在统一使用 Phase 2 手推实测的 calib_min_raw / calib_max_raw。
    #   那套值存在舵机 Flash 校准区里, **离线读不到** ⇒ 本工具不再能算 J3/J5 窗口。
    #   下面保留旧宏的兼容分支: 在 2026-09-16 之前的旧源码树上跑仍会照旧展开;
    #   在当前树上 WINDOW 为空, 会明确打印"宏已删除"而不是静默给出错误结论。
    def macro(n):
        # ⚠ 只能从【真正参与编译的】Core/Src/main.c 取 —— parse_macro 的默认 paths
        #   会把 config/arm_config.h 当 fallback, 而那是**从不被 include 的死副本**
        #   (2026-09-16 删宏后仍从它读到旧值, 害得本工具判成"旧版宏还在")。
        v, src = parse_macro(n, paths=('Core/Src/main.c',))
        v2, _ = parse_macro(n, paths=('config/arm_config.h',))
        if v is not None and v2 is not None and abs(v - v2) > 0.5:
            print("  ⚠ %s: main.c=%g / config/arm_config.h=%g 不一致 (后者不参与编译)"
                  % (n, v, v2))
            warn.append(("config/arm_config.h", "%s 副本与 main.c 不一致" % n))
        return v

    j3b, j3f = macro('J3_LIMIT_OFF_BACK'), macro('J3_LIMIT_OFF_FRONT')
    j5n, j5x = macro('J5_LIMIT_OFF_MIN'), macro('J5_LIMIT_OFF_MAX')
    off = a.zero * 360.0 / 4095.0
    WINDOW = {}
    if j3b is not None and j3f is not None:
        WINDOW['J3'] = ((a.zero + j3b) * 360.0 / 4095.0 - off,
                        (a.zero + j3f) * 360.0 / 4095.0 - off)
    if j5n is not None and j5x is not None:
        WINDOW['J5'] = ((a.zero + j5n) * 360.0 / 4095.0 - off,
                        (a.zero + j5x) * 360.0 / 4095.0 - off)
    if WINDOW:
        print("  (检测到旧版内置偏移宏 ⇒ 这棵源码树是 2026-09-16 之前的固件)")
        for jn, (lo, hi) in WINDOW.items():
            print("  %s 固件窗口 rel-home [%+.2f, %+.2f]" % (jn, lo, hi))
        for name, j, _ in poses:
            bad = []
            for idx, jn in enumerate(('J1', 'J2', 'J3', 'J4', 'J5', 'J6')):
                if jn in WINDOW:
                    lo, hi = WINDOW[jn]
                    if not (lo <= j[idx] <= hi):
                        bad.append("%s=%+.2f 越界 [%.2f, %.2f]" % (jn, j[idx], lo, hi))
            print("  %-16s %s" % (name, "  ".join(bad) if bad else "OK"))
            if bad:
                warn.append((name, "关节角越界"))
    else:
        print("  J3/J5 内置偏移宏已删除 (2026-09-16 起六轴统一走 Phase2 手推实测值)")
    print("  (六轴行程均为 Flash 里的 Phase2 实测值, 离线算不出 ⇒ 本工具不做关节角越界判定)")

    print()
    if warn:
        print("需要人工复核:")
        for n, w in warn:
            print("  - %-16s %s" % (n, w))
    else:
        print("全部位姿通过离线自检。")
    print()
    print("注意: 这只做几何/关节角检查。真机还会过 raw 限位 + 立柱圆柱 + "
          "安全规划, 结论以上机的 `P <n>` 打印为准。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
