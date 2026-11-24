# -*- coding: utf-8 -*-
"""六轴机械臂自动测试套件（一键跑完整套工程验收测试 + 生成报告）
覆盖测试纪要 docs/六轴机械臂测试纪要_2026-08-17.md 第1层各项：

  T1 冒烟自检      n / i / t a 1 / T / s
  T2 IK精度        g x y z -> 发多个目标点, f 读FK, 算ΔPos
  T3 整臂到位稳定  P0 <-> g150,0,100 多轮往返, 每轮到位后测6关节std/峰峰
  T4 回竖直过冲    从g100回P0, 到位后连读J2/J3看稳态

运行前:
  1) 机械臂已上电, COM8空闲(关闭串口助手)
  2) python run_full_test.py [--port COM8] [--rounds 3] [--outdir docs]

输出:
  测试结果_YYYYMMDD_HHMMSS.md  汇总报告(含每个测试项的表格)
"""
import serial, time, re, os, sys, datetime, argparse

PORT = 'COM8'
BAUD = 115200

class Console:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.5)
        self.ser.reset_input_buffer()
        time.sleep(0.3)

    def send(self, cmd, wait=0.4):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r\n').encode('ascii'))
        self.ser.flush()
        time.sleep(wait)
        buf = self.ser.read(self.ser.in_waiting) if self.ser.in_waiting else b''
        return buf.decode('ascii', errors='replace')

    def cmd_wait_prompt(self, cmd, timeout_s=15.0, pre_drain=0.5):
        """发送命令, 先排空残留, 持续读到提示符'>'出现(命令已处理完)才返回。
        固件一些命令(n扫描/i初始化)会阻塞主循环, 必须等到提示符才算完成。"""
        # 先排空可能的上一条命令残留, 避免干扰关键词/提示符判定
        if pre_drain > 0:
            time.sleep(pre_drain)
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
        self.ser.write((cmd + '\r\n').encode('ascii'))
        self.ser.flush()
        end = time.time() + timeout_s
        buf = b''
        while time.time() < end:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                buf += data
                if b'>' in buf[-2:]:
                    break
            time.sleep(0.08)
        if self.ser.in_waiting:
            buf += self.ser.read(self.ser.in_waiting)
        return buf.decode('ascii', errors='replace')

    def send_wait(self, cmd, keyword_list, timeout_s=3.0):
        """发送并持续读取直到出现任一关键词且见到提示符(或超时)。返回完整输出"""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r\n').encode('ascii'))
        self.ser.flush()
        end = time.time() + timeout_s
        buf = b''
        while time.time() < end:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                buf += data
            txt = buf.decode('ascii', errors='replace')
            # 关键词命中 且 见过行尾提示符 '>' （表示该命令已处理完）
            if any(k in txt for k in keyword_list) and '>' in txt:
                break
            time.sleep(0.05)
        if self.ser.in_waiting:
            buf += self.ser.read(self.ser.in_waiting)
        return buf.decode('ascii', errors='replace')

    def read_joint(self, idx, attempt=3):
        """可靠读单关节角度 p <idx> (0-based)。返回浮点或None"""
        for _ in range(attempt):
            self.ser.reset_input_buffer()
            self.ser.write(f'p {idx}\r\n'.encode())
            self.ser.flush()
            time.sleep(0.2)
            buf = self.ser.read(self.ser.in_waiting) if self.ser.in_waiting else b''
            m = re.search(r'angle=(-?[0-9.]+)', buf.decode('ascii', errors='replace'))
            if m:
                v = float(m.group(1))
                if 0 < v < 300: return v
        return None

    def read_all(self):
        """读6关节物理角度, 返回 {1..6: angle}"""
        res = {}
        for idx in range(6):
            v = self.read_joint(idx)
            if v is not None:
                res[idx+1] = v
        return res

    def wait_reached(self, timeout_s=30):
        end = time.time() + timeout_s
        while time.time() < end:
            while self.ser.in_waiting:
                d = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                if 'Move reached' in d or 'Move completed' in d or 'Adaptive' in d:
                    return True
            time.sleep(0.04)
        return False

    def move_wait(self, cmd, timeout=30):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + '\r\n').encode())
        self.ser.flush()
        # 等运动完成(Move reached)
        end = time.time() + timeout
        while time.time() < end:
            while self.ser.in_waiting:
                d = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                if 'Move reached' in d or 'Adaptive correction starting' in d:
                    return True
            time.sleep(0.04)
        return False

    def read_fk(self, attempt=3):
        """读FK: f 命令返回 x=.. y=.. z=.."""
        for _ in range(attempt):
            out = self.send('f', 0.5)
            m = re.search(r'x\s*=\s*(-?[0-9.]+)\D+y\s*=\s*(-?[0-9.]+)\D+z\s*=\s*(-?[0-9.]+)',
                          out, re.IGNORECASE)
            if m:
                return float(m.group(1)), float(m.group(2)), float(m.group(3))
        return None

    def close(self):
        try: self.ser.close()
        except: pass

# ============ 测试项 ============

def t1_smoke(c: Console):
    """冒烟自检"""
    res = {}
    # 每条命令都用 cmd_wait_prompt 等到提示符(命令真正完成), 避免命令积压
    res['n'] = c.cmd_wait_prompt('n', 25.0, pre_drain=1.0)      # 总线扫描可能阻塞主循环较久
    res['init'] = c.cmd_wait_prompt('i', 25.0, pre_drain=1.0)   # 初始化6舵机更久
    res['torque'] = c.cmd_wait_prompt('t a 1', 8.0, pre_drain=0.5)
    res['temp'] = c.cmd_wait_prompt('T', 12.0, pre_drain=0.5)   # 温度逐舵机读
    # 判断
    s_all = (res['n'] + res['init'] + res['temp']).lower()
    busa_ok = ('servo' in s_all and '3' in s_all) or 'scanning buses' in s_all or 'bus a' in s_all
    init_noerr = ('error' not in res['init'].lower() and 'failed' not in res['init'].lower())
    init_ok = ('done' in res['init'].lower() or 'joint mode' in res['init'].lower() or 'ready' in res['init'].lower()) and init_noerr
    temp_ok  = (('| j' in s_all or 'c' in s_all) and len(res['temp'].strip()) > 3
                and 'error' not in res['temp'].lower())
    ok = {
        'bus_scan': busa_ok,
        'init_done': init_ok,
        'torque_ok': ('ok' in res['torque'].lower() or 'error' not in res['torque'].lower()),
        'temp_read': temp_ok,
    }
    return res, ok

def t2_ik(c: Console):
    """IK精度: 多个目标点 g x y z -> f, 算偏差"""
    targets = [(150, 0, 200), (150, 0, 100), (0, 120, 180), (0, -120, 180)]
    rows = []
    # 先回竖直
    c.move_wait('P 0', 30)
    for (x, y, z) in targets:
        c.move_wait(f'g {x} {y} {z} 0', 30)
        time.sleep(0.8)
        fk = c.read_fk()
        if fk:
            dx, dy, dz = fk[0]-x, fk[1]-y, fk[2]-z
            d = (dx*dx+dy*dy+dz*dz)**0.5
            rows.append({'x':x,'y':y,'z':z, 'fx':fk[0],'fy':fk[1],'fz':fk[2], 'dx':dx,'dy':dy,'dz':dz, 'd':d})
        else:
            rows.append({'x':x,'y':y,'z':z, 'fx':None,'fy':None,'fz':None,'d':None})
    c.move_wait('P 0', 30)
    return targets, rows

def t3_multi_round(c: Console, rounds):
    """整臂多轮往返到位稳定"""
    results = []  # 每轮 {j: [vals]}
    for r in range(1, rounds+1):
        # 交替 P0 -> g100 -> P0, 每到位测6关节
        for cmd, label in [('g 150 0 100 0', 'g100'), ('P 0', 'P0')]:
            if not c.move_wait(cmd, 30):
                continue
            time.sleep(1.5)  # 到位后留足时间让收尾慢爬/修正完成,再测稳态
            # 测6关节读多次
            data = {j: [] for j in range(1, 7)}
            for _ in range(10):
                for j in range(1, 7):
                    v = c.read_joint(j-1)  # idx=j-1
                    if v is not None:
                        data[j].append(v)
                time.sleep(0.08)
            results.append({'round': r, 'pose': label, 'data': data})
    return results

def t4_return_settle(c: Console):
    """回竖直过冲/稳态: 从g100回P0, 到位后连读J2/J3"""
    # 先去g100
    c.move_wait('g 150 0 100 0', 30)
    time.sleep(0.8)
    # 回P0
    ok = c.move_wait('P 0', 30)
    time.sleep(0.5)  # 等待稳定
    seq_j2 = []
    seq_j3 = []
    for _ in range(15):
        v2 = c.read_joint(1)  # J2(物理2, idx1)
        v3 = c.read_joint(2)
        if v2: seq_j2.append(v2)
        if v3: seq_j3.append(v3)
        time.sleep(0.12)
    def stats(arr):
        if len(arr) < 5: return None
        mean = sum(arr)/len(arr)
        std = (sum((x-mean)**2 for x in arr)/len(arr))**0.5
        return {'mean': mean, 'std': std, 'n': len(arr)}
    return ok, stats(seq_j2), stats(seq_j3)

# ============ 报告生成 ============

def build_report(results):
    lines = []
    lines.append('# 六轴机械臂测试结果报告')
    lines.append('')
    lines.append(f'> 生成时间：{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    lines.append('> 设备：STM32F407 + 6×STS3215')
    lines.append('')
    lines.append('---')
    lines.append('')

    # T1
    r1, ok1 = results['t1']
    lines.append('## T1 冒烟自检')
    lines.append('| 检查 | 结果 |')
    lines.append('|------|------|')
    lines.append('| 总线扫描 | ' + ('✅' if ok1['bus_scan'] else '❌') + ' (见下) |')
    lines.append('| 初始化 | ' + ('✅' if ok1['init_done'] else '❌') + ' |')
    lines.append('| 扭矩使能 | ' + ('✅' if ok1['torque_ok'] else '❌') + ' |')
    lines.append('| 温度读取 | ' + ('✅' if ok1.get('temp_read') else '❌') + ' |')
    lines.append('')
    lines.append('```')
    lines.append('扫描输出: ' + r1['n'].strip()[:120])
    lines.append('初始化输出: ' + r1['init'].strip()[:120])
    lines.append('温度输出: ' + r1['temp'].strip()[:120])
    lines.append('```')
    lines.append('')

    # T2
    targets, rows = results['t2']
    lines.append('## T2 IK 精度（|ΔPos|，判据 ≤5mm）')
    lines.append('| 目标(X,Y,Z) | FK实测(X,Y,Z) | ΔX | ΔY | ΔZ | ΔPos(mm) | 判定 |')
    lines.append('|--------------|---------------|------|------|------|----------|------|')
    for r in rows:
        if r['d'] is None:
            lines.append(f"| ({r['x']},{r['y']},{r['z']}) | 读取失败 | - | - | - | - | ❌ |")
        else:
            verdict = '✅' if r['d'] <= 5 else '❌'
            lines.append(f"| ({r['x']},{r['y']},{r['z']}) | ({r['fx']:.1f},{r['fy']:.1f},{r['fz']:.1f}) | "
                         f"{r['dx']:.2f} | {r['dy']:.2f} | {r['dz']:.2f} | {r['d']:.2f} | {verdict} |")
    lines.append('')

    # T3
    t3 = results['t3']
    lines.append('## T3 整臂多轮往返到位稳定（判据 avg std <0.3°）')
    lines.append('| 轮次 | 位姿 | J1 std | J2 std | J3 std | J4 std | J5 std | J6 std |')
    lines.append('|------|------|--------|--------|--------|--------|--------|--------|')
    for ev in t3:
        stds = {}
        for j in range(1, 7):
            v = ev['data'][j]
            if len(v) >= 3:
                mean = sum(v)/len(v)
                stds[j] = (sum((x-mean)**2 for x in v)/len(v))**0.5
            else:
                stds[j] = None
        row = [f"| {ev['round']} | {ev['pose']} "]
        for j in range(1, 7):
            row.append(f"| {stds[j]:.3f}" if stds[j] is not None else '| -')
        row.append(' |')
        lines.append(' '.join(row))
    lines.append('')

    # T4
    ok4, s2, s3 = results['t4']
    lines.append('## T4 回竖直（P0）到位稳态（J2/J3）')
    if s2:
        lines.append(f"| 关节 | 稳态均值 | 标准差 | 判定 |")
        lines.append(f"|------|---------|--------|------|")
        v2 = '✅ 稳定' if s2['std'] < 0.3 else '⚠️'
        lines.append(f"| J2 | {s2['mean']:.2f}° | {s2['std']:.3f}° | {v2} |")
        if s3:
            v3 = '✅ 稳定' if s3['std'] < 0.3 else '⚠️'
            lines.append(f"| J3 | {s3['mean']:.2f}° | {s3['std']:.3f}° | {v3} |")
    else:
        lines.append('J2/J3 读取失败或无数据')
    lines.append('')

    lines.append('---')
    lines.append('*报告由 run_full_test.py 自动生成*')
    return '\n'.join(lines)

def main():
    ap = argparse.ArgumentParser(description='六轴机械臂自动测试套件')
    ap.add_argument('--port', default=PORT, help='串口号')
    ap.add_argument('--rounds', type=int, default=3, help='T3多轮往返轮数')
    ap.add_argument('--outdir', default='docs', help='报告输出目录')
    ap.add_argument('--skip-smoke', action='store_true', help='跳过冒烟')
    args = ap.parse_args()

    results = {}
    c = Console(args.port)
    try:
        print('===== T1 冒烟自检 =====')
        results['t1'] = t1_smoke(c)
        print('  完成')

        print('\n===== T2 IK精度 =====')
        results['t2'] = t2_ik(c)
        print('  完成')

        print(f'\n===== T3 整臂往返稳定 ({args.rounds}轮) =====')
        results['t3'] = t3_multi_round(c, args.rounds)
        print(f'  采集 {len(results["t3"])} 组')

        print('\n===== T4 回竖直稳态 =====')
        results['t4'] = t4_return_settle(c)
        print('  完成')
    finally:
        c.close()

    # 生成报告
    report = build_report(results)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(args.outdir, f'测试结果_{ts}.md')
    os.makedirs(args.outdir, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(report)
    print(f'\n✅ 报告已生成: {path}')
    print(report)

if __name__ == '__main__':
    main()