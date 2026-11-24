# -*- coding: utf-8 -*-
"""整臂6关节 到位后可靠稳定性测量
对 P0(竖直) 和 g150,0,100(前伸) 两个位姿，到位后逐个关节连续读取，
统计每关节的波动幅度(std/峰峰)，找出真正在摆的关节。
全程用慢速可靠读取(发p+等待+重试)，排除运动中总线污染。
"""
import serial, time, re

PORT = 'COM8'
BAUD = 115200

def read_joint(ser, idx, attempt=4):
    for _ in range(attempt):
        ser.reset_input_buffer()
        ser.write(f'p {idx}\r\n'.encode()); ser.flush()
        time.sleep(0.2)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b''
        m = re.search(r'angle=(-?[0-9.]+)', buf.decode('ascii', errors='replace'))
        if m:
            v = float(m.group(1))
            if 0 < v < 300: return v
    return None

def wait_reached(ser, timeout_s=25):
    end = time.time() + timeout_s
    while time.time() < end:
        while ser.in_waiting:
            d = ser.read(ser.in_waiting).decode('ascii', errors='replace')
            if 'Move reached' in d or 'Move completed' in d:
                return True
        time.sleep(0.05)
    return False

def measure(ser, label, target_deg, rounds=12):
    """在当前位置读6关节 rounds 轮, 统计每关节波动"""
    print(f'\n===== 测量位: {label} (目标 {target_deg}) =====')
    data = {j: [] for j in range(1, 7)}
    for r in range(rounds):
        for idx in range(6):
            v = read_joint(ser, idx)
            if v is not None:
                data[idx+1].append(v)
            time.sleep(0.1)
    print('  J  采样  mean    峰峰   std   波动判定')
    for j in range(1, 7):
        v = data[j]
        if len(v) < rounds * 0.5:
            print(f'  J{j}  {len(v):<4}  数据不足')
            continue
        mean = sum(v)/len(v)
        pk = max(v) - min(v)
        std = (sum((x-mean)**2 for x in v)/len(v))**0.5
        if std < 0.25:
            verdict = '稳定'
        elif std < 1.0:
            verdict = '轻微波动'
        else:
            verdict = '明显摆动'
        print(f'  J{j}  {len(v):<4}  {mean:6.2f}  {pk:6.2f}  {std:6.3f}  {verdict}')
    return data

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.4)
    try:
        # 先到竖直 P0
        print('发 P 0 ...')
        ser.reset_input_buffer()
        ser.write(b'P 0\r\n'); ser.flush()
        if wait_reached(ser):
            time.sleep(1.0)
            d1 = measure(ser, 'P0 竖直', [150]*6)
        else:
            print('P0 未确认到位')

        # 到前伸 g100
        time.sleep(0.5)
        print('\n发 g 150 0 100 ...')
        ser.reset_input_buffer()
        ser.write(b'g 150 0 100 0\r\n'); ser.flush()
        if wait_reached(ser, 15):
            time.sleep(1.0)
            d2 = measure(ser, 'g100 前伸', [150,169,182,179,213,5])
        else:
            print('g100 未确认到位')
    finally:
        ser.close()

if __name__ == '__main__':
    main()