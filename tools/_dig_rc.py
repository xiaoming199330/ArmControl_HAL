#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_dig_rc.py — 对 rc_monitor 的 jsonl 做一次深挖（一次性分析, 用完可删）"""
import json
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PATH = sys.argv[1] if len(sys.argv) > 1 else \
    r"C:\Users\Administrator\Desktop\ArmControl_HAL（1）\outputs\rc_monitor_20260928_122112\log.jsonl"

recs = []
with open(PATH, encoding="utf-8") as f:
    for ln in f:
        try:
            recs.append(json.loads(ln))
        except Exception:
            pass

udp = [r for r in recs if r.get("src") == "udp"]
ok = [r for r in udp if not r.get("miss") and r.get("j") and all(v is not None for v in r["j"])]
bad = [r for r in udp if r.get("miss")]
print("样本: ok=%d miss=%d 总=%.1fs" % (len(ok), len(bad), (udp[-1]["t"] - udp[0]["t"]) / 1000 if udp else 0))

# read_ok 统计
n_ro = sum(1 for r in ok if r.get("read_ok"))
print("read_ok=True: %d/%d (%.1f%%)" % (n_ro, len(ok), 100.0 * n_ro / max(1, len(ok))))

# ---------- 1) pitch=0 伪影 ----------
p0 = [r for r in ok if r.get("pitch") == 0.0]
print("\n[1] pitch==0.0 样本: %d (%.1f%%)" % (len(p0), 100.0 * len(p0) / max(1, len(ok))))
# pitch=0 附近 J4 值
if p0:
    for r in p0[:6]:
        i = ok.index(r)
        ctx = ok[max(0, i - 2):i + 3]
        print("  t=%.2f J=%s pitch∈%s" % (r["t"] / 1000, ["%.0f" % v for v in r["j"]],
              [x.get("pitch") for x in ctx]))
    # pitch=0 前后的样本 pitch（看它是瞬态还是持续）
    idxs = [ok.index(r) for r in p0[:200]]
    import statistics
    prevs = [ok[i - 1]["pitch"] for i in idxs if i > 0]
    nxts = [ok[i + 1]["pitch"] for i in idxs if i + 1 < len(ok)]
    print("  pitch=0 的前一样本 pitch 中位=%.1f, 后一样本中位=%.1f" % (
        statistics.median(prevs) if prevs else -1, statistics.median(nxts) if nxts else -1))

# ---------- 2) J3 贴上限时段（>=269°） ----------
print("\n[2] J3 贴上限(>269)连续区段")
runs = []
cur = None
for r in ok:
    v = r["j"][2]
    if v > 269.0:
        if cur is None:
            cur = [r["t"], r["t"], v]
        else:
            cur[1] = r["t"]
            cur[2] = max(cur[2], v)
    else:
        if cur is not None:
            runs.append(cur)
            cur = None
if cur:
    runs.append(cur)
print("  共 %d 段" % len(runs))
for a, b, mx in runs:
    if b - a > 300:
        # 区段内 J3 是否冻结
        seg = [r for r in ok if a <= r["t"] <= b]
        j3r = max(x["j"][2] for x in seg) - min(x["j"][2] for x in seg)
        j2r = max(x["j"][1] for x in seg) - min(x["j"][1] for x in seg)
        other = max(max(abs(x["j"][k] - seg[0]["j"][k]) for x in seg) for k in (0, 3, 4, 5))
        print("  %.1f~%.1fs (%.1fs) J3max=%.1f | 区段内 J3摆动=%.2f° J2摆动=%.1f° 其他轴最大位移=%.1f°"
              % (a / 1000, b / 1000, (b - a) / 1000, mx, j3r, j2r, other))

# ---------- 3) J4 微段（单拍抽动） ----------
print("\n[3] J4 单拍跳变 (>0.8° 且下一拍就停)")
jumps = []
for a, b in zip(ok, ok[1:]):
    dt = (b["t"] - a["t"]) / 1000
    if 0 < dt <= 0.06:
        d4 = b["j"][3] - a["j"][3]
        others = max(abs(b["j"][k] - a["j"][k]) for k in (0, 1, 2, 4, 5))
        if abs(d4) > 0.8 and others < 0.5:
            jumps.append((b["t"], d4))
print("  共 %d 次" % len(jumps))
for t, d in jumps[:15]:
    print("  t=%.2fs  ΔJ4=%+.1f°" % (t / 1000, d))

# ---------- 4) J5 段停滞细节（60~68s） ----------
print("\n[4] J5 速度细节 (60~68s)")
seg = [r for r in ok if 59.0 <= r["t"] / 1000 <= 68.5]
prev = None
slow_run = 0
slow_total = 0
n = 0
for r in seg:
    if prev is not None:
        dt = (r["t"] - prev["t"]) / 1000
        if 0 < dt <= 0.1:
            v = abs(r["j"][4] - prev["j"][4]) / dt
            n += 1
            if v < 2.0:
                slow_run += 1
                slow_total += 1
            else:
                slow_run = 0
    prev = r
print("  拍数=%d, J5 速度<2°/s 的拍=%d (%.0f%%)" % (n, slow_total, 100.0 * slow_total / max(1, n)))

# ---------- 5) J6 夹爪段 ----------
print("\n[5] J6 运动段 (|ΔJ6|>3° 的连续区段)")
runs6 = []
cur = None
prev = None
for r in ok:
    if prev is not None:
        d = abs(r["j"][5] - prev["j"][5])
        if d > 0.05:
            if cur is None:
                cur = [prev["t"], r["t"], prev["j"][5], r["j"][5]]
            else:
                cur[1] = r["t"]
                cur[3] = r["j"][5]
        else:
            if cur is not None and r["t"] - cur[1] > 150:
                runs6.append(cur)
                cur = None
    prev = r
if cur:
    runs6.append(cur)
for a, b, s, e in runs6:
    dur = (b - a) / 1000
    travel = abs(e - s)
    if travel > 3:
        print("  %.2f~%.2fs (%.2fs) 行程 %.1f° 平均速率 %.0f°/s" % (
            a / 1000, b / 1000, dur, travel, travel / max(0.01, dur)))

# ---------- 6) 过滤 pitch=0 后的真实漂移（X 段） ----------
print("\n[6] 真实 pitch 漂移 (过滤 pitch==0, X 位移>100mm 的段)")
# 用运动段检测（简化: 每 2s 窗口）
win = 2000
for i in range(0, len(ok) - 1, win // 40):
    t0 = ok[i]["t"]
    grp = [r for r in ok if t0 <= r["t"] < t0 + win and r.get("pitch") not in (None, 0.0)]
    if len(grp) < 30:
        continue
    dx = max(r["x"] for r in grp) - min(r["x"] for r in grp)
    if dx < 100:
        continue
    dp = max(r["pitch"] for r in grp) - min(r["pitch"] for r in grp)
    j4r = max(r["j"][3] for r in grp) - min(r["j"][3] for r in grp)
    flag = " ←漂移大" if dp > 5 else ""
    print("  %.0f~%.0fs ΔX=%.0fmm pitch极差=%.1f° J4极差=%.1f°%s" % (
        t0 / 1000, (t0 + win) / 1000, dx, dp, j4r, flag))
