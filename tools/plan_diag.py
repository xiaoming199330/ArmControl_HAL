#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固化 PLAN 诊断采集：测 g100前伸 + P0回竖直 的规划轨迹平滑度"""
import re, serial, time

ser = serial.Serial("COM8", 115200, timeout=0.4)

# 开启诊断
ser.reset_input_buffer()
ser.write(b"d t 1\r\n")
ser.flush()
time.sleep(0.4)
buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
print("诊断开启:", buf.decode("ascii", errors="replace").strip()[-30:])


def cap_plan(cmd, duration, name):
    """发cmd, 抓PLAN行, 返回 {joint: [(tick, angle)]}"""
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
        time.sleep(0.02)
    print(f"=== {name}: {cmd} — {len(plans)} 条PLAN ===")
    return plans


# 先确保一致起点: 运动到 g100
print("先到 g100...")
ser.reset_input_buffer()
ser.write(b"g 150 0 100 0\r\n")
ser.flush()
time.sleep(6)

# 测回竖直 P0
plans_p0 = cap_plan("P 0", 12, "P0回竖直")

# 测前伸 g100
ser.write(b"P 0\r\n"); ser.flush(); time.sleep(6)
plans_g = cap_plan("g 150 0 100 0", 12, "G100前伸")

ser.close()


def analyze_plan(plans, joint_idx):
    """分析某关节(0-based)的规划步进"""
    vals = []
    for p in plans:
        parts = p.split()
        if len(parts) < 9:
            continue
        try:
            tick = int(parts[1])
            ang = int(parts[2 + joint_idx])
            vals.append((tick, ang))
        except ValueError:
            pass
    if len(vals) < 3:
        return None
    steps = [abs(vals[i+1][1] - vals[i][1]) for i in range(len(vals)-1)]
    jumps = sum(1 for s in steps if s > 2)
    return {"n": len(vals), "jumps": jumps, "max_step": max(steps) if steps else 0,
            "mean_step": sum(steps)/len(steps) if steps else 0}


print("\n===== 规划轨迹平滑度 (PLAN = 固件规划目标, 不受总线污染) =====")
for name, plans in [("P0回竖直", plans_p0), ("G100前伸", plans_g)]:
    print(f"-- {name} ({len(plans)}条) --")
    for j, jname in [(0, "J1"), (1, "J2"), (2, "J3")]:
        r = analyze_plan(plans, j)
        if r:
            flag = "平滑✅" if r["jumps"] == 0 else f"跳台阶❌({r['jumps']}次)"
            print(f"  {jname}: 步数{r['n']} 平均步进{r['mean_step']:.1f}° 最大{r['max_step']:.0f}° {flag}")
        else:
            print(f"  {jname}: 数据不足")
