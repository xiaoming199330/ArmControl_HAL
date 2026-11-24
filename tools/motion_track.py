#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动全程高保真轨迹采样 - 抓加减速段抖动"""
import re, serial, time

ser = serial.Serial("COM8", 115200, timeout=0.4)


def read_angle(ser, idx):
    for _ in range(3):
        ser.reset_input_buffer()
        ser.write(f"p {idx}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(0.18)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
        txt = buf.decode("ascii", errors="replace")
        m = re.search(r"angle=(-?[0-9.]+)", txt)
        if m:
            v = float(m.group(1))
            if v > 0:
                return v
    return None


def track(cmd, duration, joints, name):
    """执行cmd并跟踪joints，间隔尽量小"""
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii") + b"\r\n")
    ser.flush()
    hist = {j: [] for j in joints}
    logs = []
    t0 = time.time()
    print(f"=== {name}: {cmd} ===")
    while time.time() - t0 < duration:
        t = time.time() - t0
        # 抓日志
        while ser.in_waiting:
            d = ser.read(ser.in_waiting).decode("ascii", errors="replace")
            for ln in d.splitlines():
                if any(k in ln.lower() for k in
                       ["hold", "resume", "corr", "abort", "lag"]):
                    logs.append((round(t, 1), ln.strip()))
        # 采样(0.25s间隔)
        if int(t * 4) >= len(hist[joints[0]]):
            for j in joints:
                v = read_angle(ser, j)
                if v is not None:
                    hist[j].append((round(t, 2), v))
        time.sleep(0.03)
    # 打印轨迹(平滑度检查:相邻步进)
    for j in joints:
        print(f"  J{j}轨迹(t:angle 步进):")
        prev = None
        for t, v in hist[j]:
            step = f"{v-prev:+.1f}" if prev is not None else "  "
            print(f"    t={t:4.2f} {v:6.1f}° step{step}")
            prev = v
    print(f"  {name} hold/corr日志:")
    for t, ln in logs[:15]:
        print(f"    t={t:4.1f}s {ln}")
    print()
    return hist


# 从当前可能位置开始，先 g100 再 P0，复现加减速
try:
    track("g 150 0 100 0", 10, [2, 3], "G100(加速段)")
    track("P 0", 10, [2, 3], "P0(减速段)")
finally:
    ser.close()
