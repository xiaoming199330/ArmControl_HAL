#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""0924BF 限位钳制数值验证 (2026-11-03 J3 冒烟事故复盘).

用纯 Python 复刻 main.c 的 raw_dist_fwd / raw_in_range / clamp_raw 三版实现,
喂入真机限位与现场角度, 检查:
  1) 0924BD(单向) 是否会把"略超 max"夹到对面 min
  2) 0924BE(双向) 是否只解决夹错侧, 但仍把目标钉在挡块上
  3) 0924BF(双向 + 防顶死余量) 是否让目标落在离挡块 >= MARGIN 的安全带内
  4) "目标合法但贴近边界" 这种不触发 clamp 的情况是否也被覆盖  <-- 关键盲区
  5) 未校准 (mn==mx==0) 不再把目标全部打成 raw 0
"""
M = 4096
MARGIN = 28          # STREAM_CLAMP_MARGIN_RAW, ≈2.5°


def fwd(a, b):
    return (b + M - a) % M


def in_range(r, mn, mx):
    span = fwd(mn, mx)
    if span == 0:
        return False
    return fwd(mn, r) <= span


def clamp_bd(r, mn, mx):                 # 09-02 ~ 0924BD: 只比正向距离
    if in_range(r, mn, mx):
        return r
    return mn if fwd(r, mn) <= fwd(r, mx) else mx


def clamp_be(r, mn, mx):                 # 0924BE: 双向圆周最短
    if in_range(r, mn, mx):
        return r
    dmc = min(fwd(r, mn), fwd(mn, r))
    dxc = min(fwd(r, mx), fwd(mx, r))
    return mn if dmc <= dxc else mx


def clamp_bf_clamped_only(r, mn, mx):    # 0924BF 初版: 仅"被夹住"才内缩
    if mn == 0 and mx == 0:
        return r                          # 未校准保护
    c = clamp_be(r, mn, mx)
    if c == r:
        return r
    return (mx + M - MARGIN) % M if c == mx else (mn + MARGIN) % M


def clamp_bf_band(r, mn, mx):            # 0924BF 终版: 边界安全带(无论是否被夹)
    if mn == 0 and mx == 0:
        return r
    c = clamp_be(r, mn, mx)
    span = fwd(mn, mx)
    if span <= 2 * MARGIN:               # 行程过窄: 不推, 避免推穿到对面
        return c
    d_mn = fwd(mn, c)                   # c 距下边界(沿合法弧)
    d_mx = fwd(c, mx)                   # c 距上边界(沿合法弧)
    if d_mn < MARGIN:
        return (mn + MARGIN) % M
    if d_mx < MARGIN:
        return (mx + M - MARGIN) % M
    return c


def d2r(d):
    return int(round(d * 4095.0 / 360.0)) % M


def r2d(r):
    return r * 360.0 / 4095.0


def dist_to_stop(r, mn, mx):
    """目标距最近机械挡块的 raw 距离; 0 = 正在顶死挡块"""
    if in_range(r, mn, mx):
        return min(fwd(mn, r), fwd(r, mx))
    return 0


cases = [
    ("正常范围内",            200.0, 76.0, 271.0),
    ("主臂顶死超限 0.4°(现场)", 271.4, 76.0, 271.0),
    ("略超上限 2°",           273.0, 76.0, 271.0),
    ("低于下限 1°",            75.0, 76.0, 271.0),
    ("合法但贴上限 0.4°",     270.6, 76.0, 271.0),   # 不触发 clamp 的盲区
    ("合法但贴下限 0.4°",      76.4, 76.0, 271.0),
    ("J5 窗口 341° 越界",     341.0, 180.0, 340.0),  # 0924N 旧案
    ("跨 0 弧: 3500~500",      None, 3500, 500),     # 用 raw 直接给
]

print("=" * 96)
print("MARGIN = %d raw (%.2f deg), 4096 raw/圈" % (MARGIN, MARGIN * 360.0 / 4095.0))
print("=" * 96)
hdr = "%-26s %8s | %8s %8s %8s %8s | %8s" % (
    "case", "want", "BD(旧)", "BE", "BF初版", "BF终版", "终版距挡块")
print(hdr)
print("-" * 96)

for name, deg, mn_deg, mx_deg in cases:
    if deg is None:
        mn, mx, want = mn_deg, mx_deg, 4000        # 补集里的目标
        label = "%s raw=%d" % (name, want)
    else:
        mn, mx, want = d2r(mn_deg), d2r(mx_deg), d2r(deg)
        label = "%s %.1fdeg" % (name, deg)
    a = clamp_bd(want, mn, mx)
    b = clamp_be(want, mn, mx)
    c = clamp_bf_clamped_only(want, mn, mx)
    d = clamp_bf_band(want, mn, mx)
    gap = dist_to_stop(d, mn, mx)
    print("%-26s %8d | %8.1f %8.1f %8.1f %8.1f | %6d raw" % (
        label, want, r2d(a), r2d(b), r2d(c), r2d(d), gap))

print("-" * 96)
print("未校准 mn=mx=0:")
for fn, tag in ((clamp_bd, "BD"), (clamp_be, "BE"),
                (clamp_bf_clamped_only, "BF初版"), (clamp_bf_band, "BF终版")):
    print("  %-8s want=%d(200deg) -> %d (%.1fdeg)" %
          (tag, d2r(200), fn(d2r(200), 0, 0), r2d(fn(d2r(200), 0, 0))))

print("=" * 96)
print("判定: BF终版 每例都必须 >= %d raw 远离挡块, 且落在合法弧内" % MARGIN)
ok = True
for name, deg, mn_deg, mx_deg in cases:
    if deg is None:
        mn, mx, want = mn_deg, mx_deg, 4000
    else:
        mn, mx, want = d2r(mn_deg), d2r(mx_deg), d2r(deg)
    if mn == 0 and mx == 0:
        continue
    d = clamp_bf_band(want, mn, mx)
    gap = dist_to_stop(d, mn, mx)
    good = in_range(d, mn, mx) and gap >= MARGIN
    if not good:
        ok = False
        print("  FAIL: %s -> raw=%d gap=%d in_range=%s" %
              (name, d, gap, in_range(d, mn, mx)))
print("RESULT:", "PASS" if ok else "FAIL")
