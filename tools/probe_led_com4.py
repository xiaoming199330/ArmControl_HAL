# -*- coding: utf-8 -*-
"""COM4 探测: fw-tag + i 上力矩响应, 判断报警灯常亮属于哪一层"""
import serial, time, sys

PORT = "COM4"
ser = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.3)
ser.reset_input_buffer()

def send(cmd, wait=1.0):
    ser.write((cmd + "\r").encode())
    t0 = time.time()
    buf = b""
    while time.time() - t0 < wait:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
    return buf.decode("utf-8", "replace")

print("===== h (fw-tag) =====")
h = send("h", 2.0)
for line in h.splitlines():
    l = line.strip()
    if l:
        print(l)
        if "ARM-" in l or "fw" in l.lower() or "build" in l.lower():
            pass

print("\n===== i (上力矩) =====")
print(send("i", 1.5))

print("\n===== S 后再 i 复现报警灯状态 (不做, 只查 g_soft_stop 无法直接查) =====")
ser.close()
