#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0回竖直到位后 J3 抖动 vs 修正日志 聚焦观测"""
import re, serial, time

ser = serial.Serial("COM8", 115200, timeout=0.4)

ser.reset_input_buffer()
ser.write(b"P 0\r\n")
ser.flush()

def read_angle(ser, idx):
    for _ in range(3):
        ser.reset_input_buffer()
        ser.write(f"p {idx}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(0.3)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
        txt = buf.decode("ascii", errors="replace")
        m = re.search(r"angle=(-?[0-9.]+)", txt)
        if m:
            v = float(m.group(1))
            if v > 0:
                return v
    return None

logs = []
j3 = []
t0 = time.time()
print("P0 -> 竖直到位, 观测J3(45s)...")
while time.time() - t0 < 45:
    t = time.time() - t0
    # 抓主动日志
    while ser.in_waiting:
        d = ser.read(ser.in_waiting).decode("ascii", errors="replace")
        for ln in d.splitlines():
            if any(k in ln.lower() for k in
                   ["corr", "ok", "done", "stop", "overshoot",
                    "stall", "read failed", "abort"]):
                logs.append((round(t, 1), ln.strip()))
    # 每0.5s采样J3
    if int(t * 10) % 5 == 0:
        v = read_angle(ser, 2)
        if v is not None:
            j3.append((round(t, 1), v))
    time.sleep(0.01)

print("\nJ3 45s轨迹(每2个点):")
for t, v in j3[::2]:
    print(f"  t={t:4.1f}s J3={v:.1f}")
print("\n修正/状态日志(前40):")
for t, ln in logs[:40]:
    print(f"  t={t:5.1f}s {ln}")
ser.close()
