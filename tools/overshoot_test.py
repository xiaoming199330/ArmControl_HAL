#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回竖直到位过冲实测：从下垂位 g -> P0, 密集读J2看是否冲过头"""
import re, serial, time

ser = serial.Serial("COM8", 115200, timeout=0.4)


def send(cmd, wait=0.3):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\r\n".encode("ascii"))
    ser.flush()
    time.sleep(wait)


def read_angle(ser, idx, sleep_t):
    for _ in range(3):
        ser.reset_input_buffer()
        ser.write(f"p {idx}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(sleep_t)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
        txt = buf.decode("ascii", errors="replace")
        m = re.search(r"angle=(-?[0-9.]+)", txt)
        if m:
            v = float(m.group(1))
            if v > 0:
                return v
    return None


# 测试下端到P0回竖直的J2全程
def test_one(z, sleep_t, name):
    send(f"g 150 0 {z} 0", 2)
    time.sleep(4)  # 到位
    # 记录下垂位当前J2
    j2_start = read_angle(ser, 1, sleep_t)
    # 触发回竖直
    ser.reset_input_buffer()
    ser.write(b"P 0\r\n")
    ser.flush()
    # 密集采样J2全程
    t0 = time.time()
    hist = []
    while time.time() - t0 < 12:
        t = time.time() - t0
        v = read_angle(ser, 1, sleep_t)
        if v is not None:
            hist.append((round(t, 2), v))
        time.sleep(0.05)
    # 分析: 找到最小值(回竖直J2应降到150附近), 看是否跌破150(过冲)
    vals = [v for _, v in hist]
    min_v = min(vals) if vals else None
    final_v = vals[-1] if vals else None
    min_t = hist[vals.index(min_v)][0] if vals else None
    print(f"-- {name} (z={z}), J2起始{j2_start} --")
    print(f"   采样{len(hist)}点, J2最小{min_v}(t={min_t}s), J2最终{final_v}")
    if min_v is not None:
        over = 150.0 - min_v
        print(f"   过冲量(跌破150) = {over:.1f}° {'⚠️明显过冲' if over>2 else '✅无明显过冲'}")
    # 打印关键段
    for t, v in hist[::3]:
        print(f"     t={t:5.2f} J2={v}")
    print()


print("===== 回竖直到位 过冲实测 (J2) =====")
# 不同下垂程度: z越大越下垂(离地面越近? 实际z=末端高度, z越小越靠近地面=臂越下垂前伸)
# 之前 g 150 0 100 是低位下垂, 用 z=50(z更低=更下垂) vs z=150 vs z=100 对比
test_one(120, 0.18, "下垂z=120")
test_one(60, 0.18, "下垂z=60(更低)")
ser.close()
print("完成")
