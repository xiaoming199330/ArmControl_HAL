# -*- coding: utf-8 -*-
"""多轮往返 + 每轮到位后测6关节稳定性
P 0 <-> g 150 0 100 往返 N 轮，每轮到位(Move reached)后对6关节各读8次，
记录每关节每轮的峰峰/标准差，最后统计每关节在 N 轮中的最大峰峰/平均std，
找出反复出现的真实抖动关节。
"""
import serial, time, re

PORT = 'COM8'
BAUD = 115200
ROUNDS = 6
SAMPLES = 10  # 每轮每关节读几次

def read_joint(ser, idx, attempt=3):
    for _ in range(attempt):
        ser.reset_input_buffer()
        ser.write(f'p {idx}\r\n'.encode()); ser.flush()
        time.sleep(0.16)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b''
        m = re.search(r'angle=(-?[0-9.]+)', buf.decode('ascii', errors='replace'))
        if m:
            v = float(m.group(1))
            if 0 < v < 300: return v
    return None

def wait_reached(ser, timeout_s=28):
    end = time.time() + timeout_s
    while time.time() < end:
        while ser.in_waiting:
            d = ser.read(ser.in_waiting).decode('ascii', errors='replace')
            if 'Move reached' in d or 'Move completed' in d:
                return True
        time.sleep(0.04)
    return False

def move_and_wait(ser, cmd, timeout=28):
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode()); ser.flush()
    return wait_reached(ser, timeout)

def measure_all(ser):
    """到位后读6关节 SAMPLES 轮, 返回 {j: [values]}"""
    data = {j: [] for j in range(1, 7)}
    for _ in range(SAMPLES):
        for idx in range(6):
            v = read_joint(ser, idx)
            if v is not None:
                data[idx+1].append(v)
    return data

def summarize(round_results):
    """round_results = [{j: [vals]}, ...] 每轮每关节的采样列表"""
    print('\n========== 多轮汇总（找出反复抖动关节） ==========')
    print(f'共 {ROUNDS} 轮，每轮 {SAMPLES} 次采样')
    print('  J  平均std   最大峰峰   出现波动轮次')
    for j in range(1, 7):
        # 每轮该关节的 std 和 峰峰
        r_std, r_pk = [], []
        for rd in round_results:
            v = rd[j]
            if len(v) < 3: continue
            mean = sum(v)/len(v)
            r_std.append((sum((x-mean)**2 for x in v)/len(v))**0.5)
            r_pk.append(max(v) - min(v))
        if not r_std:
            print(f'  J{j}  数据不足')
            continue
        avg_std = sum(r_std)/len(r_std)
        max_pk = max(r_pk)
        # 出现 std>0.3 的轮次比例
        turbulent = sum(1 for s in r_std if s > 0.3)
        flag = ''
        if avg_std > 0.5: flag = '  <== 明显'
        elif avg_std > 0.3: flag = '  <== 注意'
        print(f'  J{j}  {avg_std:7.3f}  {max_pk:7.2f}    {turbulent}/{len(r_std)}轮{flag}')

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.4)
    all_rounds = []  # 每轮返回的 measure_all 结果
    try:
        for r in range(1, ROUNDS+1):
            print(f'\n===== 往返 {r}/{ROUNDS} =====')
            # 奇数轮: P0 -> g100; 偶数轮: g100 -> P0 (交替覆盖两个方向)
            if r % 2 == 1:
                # 先到 g100 (从当前位出发)
                if not move_and_wait(ser, 'g 150 0 100 0', 25):
                    print('  g100 未到位'); continue
                time.sleep(0.8)
                print(f'  [{r}] g100 到位, 测6关节...')
                all_rounds.append(measure_all(ser))
                # 到 P0
                time.sleep(0.3)
                if not move_and_wait(ser, 'P 0', 28):
                    print('  P0 未到位'); continue
                time.sleep(0.8)
                print(f'  [{r}] P0 到位, 测6关节...')
                all_rounds.append(measure_all(ser))
            else:
                # 当前应在竖直, 先 P0(保持) 再 g100
                if not move_and_wait(ser, 'P 0', 20):
                    print('  P0 未到位'); continue
                time.sleep(0.8)
                print(f'  [{r}] P0 到位, 测6关节...')
                all_rounds.append(measure_all(ser))
                time.sleep(0.3)
                if not move_and_wait(ser, 'g 150 0 100 0', 25):
                    print('  g100 未到位'); continue
                time.sleep(0.8)
                print(f'  [{r}] g100 到位, 测6关节...')
                all_rounds.append(measure_all(ser))
        # 汇总
        summarize(all_rounds)
        # 打印每轮每个关节的峰峰,便于看规律
        print('\n每轮每关节峰峰值(°):')
        for i, rd in enumerate(all_rounds):
            pk = [f'J{j}={max(rd[j])-min(rd[j]):.2f}' if rd[j] else f'J{j}=--' for j in range(1,7)]
            print(f'  轮{i+1}: ' + ' '.join(pk))
    finally:
        try: ser.close()
        except: pass

if __name__ == '__main__':
    main()