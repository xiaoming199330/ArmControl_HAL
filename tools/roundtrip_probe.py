# -*- coding: utf-8 -*-
"""往返测试 + 版本行为探测 + J2过冲测量
执行 g 150 0 100 <-> P 0 往返，抓:
1. Curve complete -> Move reached 时间间隔(判断固件版本行为: 新版应保持低速慢爬秒级延迟)
2. J2 到位后是否跌破150(过冲量)
"""
import serial, time, re, sys, os

PORT = 'COM8'
BAUD = 115200

LOG = []
T = []          # (t, j2) 时间序列
CURVE_T = None  # curve complete 时刻
REACHED_T = None
CURVE_REACHED_DELTAS = []

def log(msg):
    LOG.append(msg)
    print(msg, flush=True)

def read_j2(ser):
    for _ in range(2):
        ser.reset_input_buffer()
        ser.write(b'p 1\r\n'); ser.flush()
        time.sleep(0.22)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b''
        m = re.search(r'angle=(-?[0-9.]+)', buf.decode('ascii', errors='replace'))
        if m:
            v = float(m.group(1))
            if v > 0: return v
    return None

def read_all(ser):
    """读6关节, 返回 dict"""
    res = {}
    for idx in range(6):
        for _ in range(2):
            ser.reset_input_buffer()
            ser.write(f'p {idx}\r\n'.encode()); ser.flush()
            time.sleep(0.15)
            buf = ser.read(ser.in_waiting) if ser.in_waiting else b''
            m = re.search(r'angle=(-?[0-9.]+)', buf.decode('ascii', errors='replace'))
            if m:
                v = float(m.group(1))
                if v > 0:
                    res[idx+1] = v
                    break
        time.sleep(0.05)
    return res

def wait_for_lines(ser, keywords, timeout_s, stop_keywords=None):
    """等待抓取关键词行, 返回匹配行列表和持续时长"""
    end = time.time() + timeout_s
    buf = b''
    matches = []
    t0 = time.time()
    while time.time() < end:
        while ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buf += data
        txt = buf.decode('ascii', errors='replace')
        # 按行处理已到达的
        lines = txt.split('\r\n')
        buf = b''
        for ln in lines:
            if ln.strip():
                matches.append((time.time()-t0, ln.strip()))
        # 检查
        full = '\n'.join(l for _, l in matches)
        if stop_keywords and all(k in full for k in stop_keywords):
            return matches, time.time()-t0
        time.sleep(0.005)
    return matches, time.time()-t0

def run_cycle(ser, cycle_idx):
    """执行 g->P0 往返一个cycle, 返回过冲数据"""
    global CURVE_T, REACHED_T
    log(f"\n===== 往返 {cycle_idx}: g 150 0 100 → P 0 =====")
    # 先确保在竖直附近便于观察: 如果当前不在前伸位, 先去g
    ser.reset_input_buffer()
    ser.write(b'g 150 0 100 0\r\n'); ser.flush()
    # 等待 g 到位
    m1, d1 = wait_for_lines(ser, [], 20, stop_keywords=['Move reached'])
    for t, ln in m1:
        if 'Curve complete' in ln: CURVE_T = t
        if 'Move reached' in ln: REACHED_T = t
    if CURVE_T is not None and REACHED_T is not None:
        CURVE_REACHED_DELTAS.append(REACHED_T - CURVE_T)
    log(f"  g100 完成 (curve→reached {REACHED_T-CURVE_T if (CURVE_T and REACHED_T) else '?'} s)")
    time.sleep(1.5)

    # 记录前伸位各关节
    pos1 = read_all(ser)
    log(f"  前伸位: " + ' '.join(f'J{j}={pos1.get(j,"-")}' for j in range(1,7)))

    # 采样P0回竖直的J2(高频)
    log("  [P 0] 回竖直, 高频采样 J2 ...")
    ser.reset_input_buffer()
    ser.write(b'P 0\r\n'); ser.flush()
    t0 = time.time()
    j2_hist = []
    curve_t = None
    while time.time() - t0 < 25:
        t = time.time() - t0
        # 抓日志
        while ser.in_waiting:
            d = ser.read(ser.in_waiting).decode('ascii', errors='replace')
            for ln in d.splitlines():
                if 'Curve complete' in ln: curve_t = t
                if 'Move reached' in ln:
                    log(f"    [t={t:.1f}s] Move reached")
                    # 到位后再持续采样几秒看稳态/过冲恢复
                if 'approach slow' in ln:
                    log(f"    [t={t:.1f}s] {ln.strip()}")
        v = read_j2(ser)
        if v is not None:
            j2_hist.append((round(t, 2), v))
        if curve_t and t - curve_t > 8:
            break
        time.sleep(0.02)
    return j2_hist

def analyze_j2(j2_hist, target=150.0):
    """分析J2回竖直: 找最小谷值(过冲)与稳定值"""
    if len(j2_hist) < 10:
        return None
    vals = [v for _, v in j2_hist]
    # 运动阶段(前段,从起始值下降到目标附近)
    start = vals[0]
    tail = vals[-8:]
    stable = sum(tail)/len(tail)
    # 最小值(过冲谷值)
    vmin = min(vals)
    overshoot = (target - stable)  # 低于目标的值=负
    max_dip = target - vmin       # 最大下冲量
    return {
        'start': start, 'min': vmin, 'max_dip': max_dip,
        'stable': stable, 'err_stable': stable - target,
    }

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.4)
    results = []
    try:
        # 开启诊断(观测approach slow)
        ser.reset_input_buffer()
        ser.write(b'd t 1\r\n'); ser.flush()
        time.sleep(0.5)
        ser.read(ser.in_waiting) if ser.in_waiting else None
        log("已开启 d t 1 诊断")

        for c in range(1, 4):  # 3轮
            j2 = run_cycle(ser, c)
            if j2:
                ana = analyze_j2(j2)
                results.append((c, ana, j2))
                if ana:
                    log(f"  J2: start={ana['start']:.1f}° min={ana['min']:.1f}° "
                        f" 最大下冲={ana['max_dip']:.1f}° 稳定={ana['stable']:.1f}° "
                        f"(目标150, 稳态err={ana['err_stable']:+.1f}°)")
        # 关闭诊断
        ser.reset_input_buffer()
        ser.write(b'd t 0\r\n'); ser.flush()
        time.sleep(0.3)
        ser.read(ser.in_waiting) if ser.in_waiting else None
    finally:
        try: ser.close()
        except: pass

    print("\n===== 汇总 =====")
    print(f"Curve complete → Move reached 间隔: {[round(d,2) for d in CURVE_REACHED_DELTAS]} s "
          f"(新版应>1s慢爬, 旧版应<0.5s)")
    for c, ana, _ in results:
        if ana:
            print(f"  轮{c}: J2 最大下冲(过冲量)={ana['max_dip']:.1f}°  稳态err={ana['err_stable']:+.1f}°")

if __name__ == '__main__':
    main()