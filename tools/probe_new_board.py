# -*- coding: utf-8 -*-
"""probe_new_board.py — 新板子功能探查 (2026-11-19)
四项检测:
  A. 控制台响应延迟 (h 回显耗时)
  B. SysTick 节拍 vs PC 墙钟 (tm 100 连续 6s 数行数, 偏差>15% = 时钟异常)
  C. 已知速度运动实测耗时 (a J +20deg @v=25, 预期 ~1.1s; 超时=轨迹时钟慢)
     + 运动中连续采样 J1, 统计"停顿段"量化一顿一顿
  D. 舵机总线压测 (d p: 每 ID 30 ping, OK/NO-RESP/BAD-FRAME)
用法: PYTHONUTF8=1 python tools/probe_new_board.py COM4
"""
import re, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

def open_port():
    try:
        return serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=2)
    except Exception as e:
        print("!! 打开 %s 失败: %s" % (PORT, e))
        print("   如果是'拒绝访问': 先关掉你正占着 COM 口的串口终端工具再跑。")
        sys.exit(1)

def drain(ser, quiet=0.3):
    ser.reset_input_buffer()
    time.sleep(quiet)
    ser.reset_input_buffer()

def send(ser, cmd, wait=0.1):
    ser.write((cmd + "\r").encode())
    time.sleep(wait)

def read_for(ser, seconds):
    """读 seconds 秒, 返回 (完整文本, 每行带到达时间戳的列表)"""
    buf = b""
    lines = []          # (t_recv, line)
    t_end = time.time() + seconds
    frag = b""
    while time.time() < t_end:
        chunk = ser.read(512)
        if chunk:
            buf += chunk
            frag += chunk
            while b"\n" in frag:
                ln, frag = frag.split(b"\n", 1)
                lines.append((time.time(), ln.decode("utf-8", "replace").rstrip("\r")))
    if frag:
        lines.append((time.time(), frag.decode("utf-8", "replace").rstrip("\r")))
    return buf.decode("utf-8", "replace"), lines

def full(ser, cmd, seconds=3.0):
    """发命令并收 seconds 秒完整回显"""
    drain(ser, 0.15)
    ser.write((cmd + "\r").encode())
    txt, lines = read_for(ser, seconds)
    return txt, lines

def main():
    ser = open_port()
    print("== 端口 %s @%d 已打开 ==" % (PORT, BAUD))

    # ---- 唤醒提示符 ----
    send(ser, "", wait=0.5)
    txt, _ = read_for(ser, 0.5)
    print("[0] 提示符探测: %r" % txt[-40:])

    # ---- A: h 延迟 + fw-tag ----
    t0 = time.time()
    drain(ser, 0.15)
    ser.write(b"h\r")
    txtA, linesA = read_for(ser, 4.0)
    dt_h = time.time() - t0
    tag = re.findall(r"fw-tag\s+(\S+)", txtA)
    firstline = None
    for t, ln in linesA:
        if "fw-tag" in ln or "ArmControl" in ln or "===" in ln:
            firstline = t - t0
            break
    print("[A] h 总耗时 %.2fs | fw-tag=%s | 首响应 %.3fs" %
          (dt_h, tag[0] if tag else "???", firstline if firstline is not None else -1))
    print("    (老板子基线: h 5406B 应 <1s 内刷完)")

    # ---- B: SysTick 节拍 vs 墙钟 ----
    txtB, _ = full(ser, "tm 100", 1.0)
    txtB2, linesB = read_for(ser, 6.0)
    fullB = txtB + txtB2
    tm_lines = [t for t, ln in linesB if ln.startswith("TM ")]
    n = len(tm_lines)
    rate_ok = ""
    if n >= 5 and len(tm_lines) >= 2:
        span = tm_lines[-1] - tm_lines[0]
        period = span / (n - 1) * 1000.0
        dev = (period - 100.0) / 100.0 * 100.0
        rate_ok = "周期=%.1fms (期望100ms) 偏差=%+.1f%% %s" % (
            period, dev, "OK" if abs(dev) < 15 else "*** 异常: SysTick/时钟不准! ***")
    print("[B] tm 100 持续6s 收到 %d 行 | %s" % (n, rate_ok))
    full(ser, "tm 0", 0.6)

    # ---- C: 已知速度运动实测 (J1 +20deg @ v=25deg/s) ----
    txtF, _ = full(ser, "f", 1.0)
    m = re.search(r"servo\s*:\s*J1=([\d.]+)", txtF)
    if not m:
        print("[C] 读不到 J1 当前角度, 跳过运动测试 (f 回显: %r)" % txtF[:200])
    else:
        j1 = float(m.group(1))
        tgt = j1 + 20.0
        if tgt > 300: tgt = j1 - 20.0
        dur_expected = abs(tgt - j1) / 25.0 + 0.35   # 匀速+加减速段估算
        print("[C] J1=%.1f -> %.1f @v=25deg/s (预期 %.2fs)" % (j1, tgt, dur_expected))
        drain(ser, 0.15)
        t0 = time.time()
        ser.write(("a 0 %.0f 25\r" % tgt).encode())
        samples = []      # (t, J1)
        reached_at = None
        t_end = t0 + 8.0
        frag = b""
        while time.time() < t_end:
            chunk = ser.read(512)
            if not chunk:
                continue
            frag += chunk
            while b"\n" in frag:
                ln, frag = frag.split(b"\n", 1)
                s = ln.decode("utf-8", "replace").strip()
                mm = re.search(r"servo\s*:\s*J1=([\d.]+)", s)
                if mm:
                    samples.append((time.time() - t0, float(mm.group(1))))
                if "Move reached" in s or "Move cancelled" in s or "settle" in s.lower():
                    reached_at = time.time() - t0
                    t_end = time.time() + 0.5   # 多收 0.5s 就结束
        dt_move = reached_at if reached_at else -1
        print("[C] Move reached 耗时: %s | 采样点 %d 个" %
              ("%.2fs" % dt_move if reached_at else "超时未到(>8s)!", len(samples)))
        ratio = (dt_move / dur_expected) if dt_move and dt_move > 0 else 0
        if ratio:
            verdict = "OK" if ratio < 1.35 else ("偏慢 x%.2f (轨迹时钟可能半速?)" % ratio)
            print("[C] 实测/预期 = %.2f  %s" % (ratio, verdict))
        # 一顿一顿量化: 相邻采样差为 0 的连续段
        stalls = 0
        prev = None
        zero_run = 0
        deltas = []
        for t, v in samples:
            if prev is not None:
                d = v - prev
                deltas.append((t, d))
                if abs(d) < 0.05 and v != samples[0][1]:
                    zero_run += 1
                else:
                    if zero_run >= 2:
                        stalls += 1
                    zero_run = 0
            prev = v
        if zero_run >= 2: stalls += 1
        print("[C] 运动中位置停顿段(连续>=3个采样无位移): %d 处  %s" %
              (stalls, "(>2 处即明显一顿一顿)" if stalls > 2 else "(少量=正常, 总线偶发丢写)"))
        # 回原位
        full(ser, "a 0 %.0f 25" % j1, 3.0)

    # ---- D: 舵机总线压测 ----
    print("[D] 跑 d p (每 ID 30 ping, 约 10~30s, 丢读严重时更久)...")
    txtD, _ = full(ser, "d p", 60.0)
    for ln in txtD.splitlines():
        if "OK=" in ln or "stress" in ln or "===" in ln:
            print("    " + ln.strip())

    ser.close()
    print("== 探查完成 ==")

if __name__ == "__main__":
    main()
