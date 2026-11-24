# -*- coding: utf-8 -*-
"""probe2_new_board.py — 第二轮: 节拍抖动/BusA 六轴/运动墙钟耗时/nt 时钟体检"""
import re, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"

def open_port():
    return serial.Serial(PORT, 115200, timeout=0.05, write_timeout=2)

def read_for(ser, seconds):
    lines = []
    t_end = time.time() + seconds
    frag = b""
    while time.time() < t_end:
        chunk = ser.read(512)
        if not chunk:
            continue
        frag += chunk
        while b"\n" in frag:
            ln, frag = frag.split(b"\n", 1)
            lines.append((time.time(), ln.decode("utf-8", "replace").rstrip("\r")))
    return lines

def full(ser, cmd, seconds=3.0):
    ser.reset_input_buffer(); time.sleep(0.15); ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    return read_for(ser, seconds)

ser = open_port()
print("== probe2 @%s ==" % PORT)

# ---- 1) tm 200 逐行到达间隔 (10s) ----
full(ser, "tm 200", 0.8)
lines = read_for(ser, 10.0)
tm_t = [t for t, ln in lines if ln.startswith("TM ")]
if len(tm_t) >= 5:
    gaps = [tm_t[i+1] - tm_t[i] for i in range(len(tm_t)-1)]
    avg = sum(gaps)/len(gaps)
    mx, mn = max(gaps), min(gaps)
    print("[1] tm 200: %d 行, 平均间隔 %.1fms (期望200ms), min=%.1f max=%.1f" %
          (len(tm_t), avg*1000, mn*1000, mx*1000))
    print("    %s" % ("间隔均匀 -> SysTick 真慢(时钟配置问题)" if mx < avg*1.6
                     else "间隔忽快忽慢 -> 打印被缓冲/中断风暴, 不是单纯时钟慢"))
full(ser, "tm 0", 0.6)

# ---- 2) fi diag: 六轴力矩回读 (Bus A J1-J3 死活) ----
print("[2] fi diag (六轴力矩回读, Bus A 死活):")
lines = full(ser, "fi diag", 8.0)
for t, ln in lines:
    s = ln.strip()
    if s and ("力矩" in s or "主臂" in s or "J" in s or "统计" in s or "OFF" in s):
        print("    " + s[:120])

# ---- 3) J4 运动墙钟耗时 (v=25deg/s, +20deg, 预期 ~1.15s) ----
lines = full(ser, "f", 1.5)
txt = "".join(ln for t, ln in lines)
m = re.search(r"servo\s*:\s*J4=([\d.]+)", txt)
if m:
    j4 = float(m.group(1))
    tgt = j4 + 20.0
    if tgt > 284: tgt = j4 - 20.0
    exp = abs(tgt - j4)/25.0 + 0.35
    print("[3] J4=%.1f -> %.1f @25deg/s, 预期 %.2fs" % (j4, tgt, exp))
    ser.reset_input_buffer(); time.sleep(0.15); ser.reset_input_buffer()
    t0 = time.time()
    ser.write(("a 3 %.0f 25\r" % tgt).encode())
    reached = None
    t_end = t0 + 10.0
    frag = b""
    while time.time() < t_end:
        c = ser.read(512)
        if not c: continue
        frag += c
        while b"\n" in frag:
            ln, frag = frag.split(b"\n", 1)
            s = ln.decode("utf-8", "replace")
            if "Move reached" in s or "settle timeout" in s:
                reached = time.time() - t0
                t_end = time.time() + 0.3
    print("    实测 Move reached: %s" % ("%.2fs" % reached if reached else "未在10s内完成!"))
    if reached:
        r = reached/exp
        print("    实测/预期 = %.2f %s" % (r, "OK" if r < 1.35 else "*** 运动被拉慢 x%.2f ***" % r))
    full(ser, "a 3 %.0f 25" % j4, 4.0)   # 回原位
else:
    print("[3] J4 读不到, 跳过")

# ---- 4) nt 时钟体检 ----
print("[4] nt 时钟体检 (REF_CLK/TIM5/DWT, 约5~15s):")
lines = full(ser, "nt", 30.0)
for t, ln in lines:
    s = ln.strip()
    if s and re.search(r"(MHz|Hz|自检|结论|时钟|edge|沿|周期)", s):
        print("    " + s[:150])

ser.close()
print("== probe2 完成 ==")
