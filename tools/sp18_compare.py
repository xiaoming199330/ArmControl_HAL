#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SP18加速度对比：不同加速度下 P0回竖直 的加减速段规划特征"""
import re, serial, time

ser = serial.Serial("COM8", 115200, timeout=0.4)


def send(cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\r\n".encode("ascii"))
    ser.flush()
    time.sleep(wait)


def cap_plan(cmd, duration):
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii") + b"\r\n")
    ser.flush()
    plans = []
    t0 = time.time()
    while time.time() - t0 < duration:
        while ser.in_waiting:
            d = ser.read(ser.in_waiting).decode("ascii", errors="replace")
            for ln in d.splitlines():
                if ln.startswith("PLAN"):
                    plans.append(ln)
        time.sleep(0.01)
    return plans


def analyze(plans, joint_idx, start_val, end_val):
    vals = []
    for p in plans:
        pt = p.split()
        if len(pt) < 9:
            continue
        try:
            vals.append(int(pt[2 + joint_idx]))
        except ValueError:
            pass
    if len(vals) < 3:
        return None
    # 总步数
    total = len(vals)
    # 位移方向
    d = 1 if end_val >= start_val else -1
    # 加速度段:从起始到位移超过总位移的某比例? 简单:算出每步相对起始
    base = vals[0]
    # 各段特征
    steps = [abs(vals[i+1] - vals[i]) for i in range(len(vals)-1)]
    max_step = max(steps)
    # 加速度段大约=非匀速的首尾段,粗略:位移达到90%前的步数
    drange = max(vals) - min(vals)
    # 用"达到95%总位移"的时间步数估算减速段长度
    full_lo, full_hi = min(vals), max(vals)
    span = full_hi - full_lo
    reach95 = 0
    for i, v in enumerate(vals):
        frac = (v - full_lo) / span if span > 0 else 1
        if frac >= 0.95:
            reach95 = i
            break
    decel_frac = (total - reach95) / total if total else 0  # 最后5%位移占的比例
    return {"total": total, "max_step": max_step,
            "decel_frac": round(decel_frac, 2), "range_deg": span}


# 开启诊断
send("d t 1", 0.3)

# 先固定速度,只改加速度SP18
send("SP 17 300")
send("g 150 0 100 0", 2); time.sleep(4)  # 到前伸

sp18_vals = [1500, 800, 400, 200, 100]
print("SP18 加速度对比 — P0回竖直 加减速段规划特征")
print("=" * 70)
for sp18 in sp18_vals:
    send(f"SP 18 {sp18}")
    send("g 150 0 100 0", 1); time.sleep(4)  # 确保在前伸位
    plans = cap_plan("P 0", 10)
    print(f"\nSP18={sp18}: {len(plans)}条PLAN")
    for j, jidx, jn in [(0, 0, "J1"), (1, 1, "J2"), (2, 2, "J3")]:
        r = analyze(plans, jidx, 150, 182)
        if r:
            print(f"  {jn}: 总步数{r['total']} 最大step{r['max_step']}° 位移{r['range_deg']}° "
                  f"末端减速段占{r['decel_frac']}")
    time.sleep(1)

send("d t 0")
ser.close()
print("\n完成。观察: 总步数越多=运动越慢越平缓; 减速段占比越高=末端越柔和")
