#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""降速前后 回竖直到位减速段 平滑度对比测试"""
import re, serial, time, sys

ser = serial.Serial("COM8", 115200, timeout=0.4)


def read_angle(ser, idx):
    for _ in range(3):
        ser.reset_input_buffer()
        ser.write(f"p {idx}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(0.15)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
        txt = buf.decode("ascii", errors="replace")
        m = re.search(r"angle=(-?[0-9.]+)", txt)
        if m:
            v = float(m.group(1))
            if v > 0:
                return v
    return None


def send(cmd, wait=0.2):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\r\n".encode("ascii"))
    ser.flush()
    time.sleep(wait)


def track_return(joints, duration, name):
    """P0 回竖直,跟踪joints全程,返回轨迹"""
    send("P 0")
    hist = {j: [] for j in joints}
    t0 = time.time()
    while time.time() - t0 < duration:
        t = time.time() - t0
        if int(t * 4) >= len(hist[joints[0]]):
            for j in joints:
                v = read_angle(ser, j)
                if v is not None:
                    hist[j].append((round(t, 2), v))
        time.sleep(0.03)
    return hist


def analyze(hist, joints):
    """分析: 相邻步进>3°的跳动次数, 最大步进, 平滑度"""
    out = {}
    for j in joints:
        vals = hist.get(j, [])
        if len(vals) < 3:
            out[j] = {"samples": len(vals), "jumps": 0, "max_step": 0, "smooth": True}
            continue
        steps = [abs(vals[i+1][1] - vals[i][1]) for i in range(len(vals)-1)]
        jumps = sum(1 for s in steps if s > 3.0)
        max_step = max(steps) if steps else 0
        smooth = jumps == 0
        out[j] = {"samples": len(vals), "jumps": jumps, "max_step": max_step, "smooth": smooth}
    return out


# 先回到 HOME 确保一致起点
send("P 0", 3)
time.sleep(3)
send("g 150 0 100 0", 3)
time.sleep(3)

# 阶段A: 原始速度回竖直
print("=== 阶段A: 原始速度 回竖直(P0) ===")
send("SP 17 300")
send("SP 18 1500")
send("g 150 0 100 0", 2)
time.sleep(3)
histA = track_return([2, 3], 12, "A")
repA = analyze(histA, [2, 3])
for j, r in repA.items():
    print(f"  J{j}: 采样{r['samples']}点, 跳动(>3°){r['jumps']}次, 最大步进{r['max_step']:.1f}°, {'✅平滑' if r['smooth'] else '❌有跳台阶'}")

# 阶段B: 降速回竖直
print("\n=== 阶段B: 降速(SP17=120, SP18=500) 回竖直(P0) ===")
send("g 150 0 100 0", 2)  # 先回前伸
time.sleep(3)
send("SP 17 120")
send("SP 18 500")
send("g 150 0 100 0", 2)
time.sleep(3)
histB = track_return([2, 3], 15, "B")
repB = analyze(histB, [2, 3])
for j, r in repB.items():
    print(f"  J{j}: 采样{r['samples']}点, 跳动(>3°){r['jumps']}次, 最大步进{r['max_step']:.1f}°, {'✅平滑' if r['smooth'] else '❌有跳台阶'}")

print("\n=== 对比结论 ===")
for j in [2, 3]:
    a, b = repA.get(j, {}), repB.get(j, {})
    print(f"  J{j}: 原速跳动{a.get('jumps','?')}次/最大{a.get('max_step','?')}° -> 降速跳动{b.get('jumps','?')}次/最大{b.get('max_step','?')}°")

ser.close()
print("\n完成")
