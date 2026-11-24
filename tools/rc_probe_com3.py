# -*- coding: utf-8 -*-
"""COM3 自主探查: rc 链路状态两轮观察 + Mode2/Mode3 对照"""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"

def open_port():
    return serial.Serial(PORT, 115200, timeout=0.1)

def drain(ser, t=0.5):
    end = time.time() + t
    while time.time() < end:
        if ser.read(4096):
            end = time.time() + t

def send(ser, cmd, wait=2.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    t0 = time.time()
    buf = b""
    while time.time() - t0 < wait:
        c = ser.read(4096)
        if c:
            buf += c
            # 收到提示符后再等 0.3s 收尾
            if b"\n> " in buf[-8:] or buf.rstrip().endswith(b">"):
                t0_end = time.time() + 0.3
                while time.time() < t0_end:
                    c2 = ser.read(4096)
                    if c2:
                        buf += c2
                        t0_end = time.time() + 0.3
                break
    return buf.decode("utf-8", "replace")

ser = open_port()
drain(ser, 1.0)

print("########## [1] rc diag (首轮, Mode3) ##########")
print(send(ser, "rc diag", 3.0))

print("########## [2] 等 6s 让 F3 组合拳跑几轮, 再看 rc ##########")
time.sleep(6)
print(send(ser, "rc", 2.0))

print("########## [3] rc diag 第二轮 (看响应是否演变) ##########")
print(send(ser, "rc diag", 3.0))

print("########## [4] 再等 8s, rc 看统计 ##########")
time.sleep(8)
print(send(ser, "rc", 2.0))

print("########## [5] rc raw (最近原始帧缓存) ##########")
print(send(ser, "rc raw", 2.0))

print("########## [6] Mode2 对照: rc cpha 0 + diag ##########")
print(send(ser, "rc cpha 0", 2.0))
print(send(ser, "rc diag", 3.0))

print("########## [7] 恢复 Mode3 ##########")
print(send(ser, "rc cpha 1", 2.0))

ser.close()
print("DONE")
