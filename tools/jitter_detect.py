#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jitter_detect.py - 高频抖动检测工具

用户反馈 J1/J2 到位后小幅抖动。慢速(300ms)采样可能捕捉不到，
本工具用尽可能快的速度连续采样指定关节，捕捉真实抖动幅度/频率。

用法：
  python jitter_detect.py --port COM8 --joints 1 2 --seconds 30

输出：每关节的时间-角度序列 CSV + 抖动统计(峰峰值/标准差/主摆幅)
"""

import argparse
import csv
import sys
import time

import serial


def parse_args():
    ap = argparse.ArgumentParser(description="高频抖动检测")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--joints", nargs="+", type=int, default=[1, 2],
                    help="要检测的物理关节(1~6)，固件p命令 index=joint-1")
    ap.add_argument("--seconds", type=float, default=30,
                    help="采样时长(秒)")
    ap.add_argument("--out", default="jitter_result")
    return ap.parse_args()


class ArmConsole:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     timeout=0.2, write_timeout=1.0)
        except serial.SerialException as e:
            print(f"[错误] 串口 {port} 被占用或无响应: {e}")
            raise
        self.ser.reset_input_buffer()

    def read_one(self, phys_joint):
        """读取单个物理关节角度。index=phys-1。返回角度或None。"""
        idx = phys_joint - 1
        self.ser.reset_input_buffer()
        self.ser.write(f"p {idx}\r\n".encode("ascii"))
        self.ser.flush()
        # 同步读取，等一小段后批量读
        time.sleep(0.015)
        if not self.ser.in_waiting:
            return None
        data = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
        a = data.rfind("angle=")
        if a < 0:
            return None
        try:
            return float(data[a + 6:].split(" deg", 1)[0].strip())
        except ValueError:
            return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

def main():
    args = parse_args()
    console = ArmConsole(args.port, args.baud)
    # 采样前先运动到中位确保有信号? 不——用户自己控制到目标位,
    # 本工具只做纯观测。采样开始前清一次缓冲。
    console.ser.reset_input_buffer()

    # 数据: per joint list of (t, angle)
    data = {j: [] for j in args.joints}
    t0 = time.monotonic()
    print(f"[开始] 采样 {args.joints} 关节, {args.seconds}s, 每轮循环读取...")
    last_print = 0
    success = {j: 0 for j in args.joints}
    fail = {j: 0 for j in args.joints}

    while True:
        now = time.monotonic()
        if now - t0 > args.seconds:
            break
        # 一轮读所有目标关节
        for j in args.joints:
            ang = console.read_one(j)
            t = time.monotonic() - t0
            if ang is not None:
                data[j].append((t, ang))
                success[j] += 1
            else:
                fail[j] += 1
        # 进度
        if int(now - t0) != last_print:
            last_print = int(now - t0)
            s = f"[{last_print:>3}s] " + "  ".join(
                f"J{j}:{data[j][-1][1] if data[j] else '-'}度" for j in args.joints)
            print(s)

    console.close()

    # 输出
    import datetime
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"{args.out}_{ts}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["time_s"] + [f"J{j}_deg" for j in args.joints])
        # 对齐时间戳(以最长序列为准)
        max_n = max(len(data[j]) for j in args.joints)
        for k in range(max_n):
            row = []
            # 用第一个关节的时间
            t_val = data[args.joints[0]][k][0] if k < len(data[args.joints[0]]) else ""
            row.append(round(t_val, 3) if t_val != "" else "")
            for j in args.joints:
                if k < len(data[j]):
                    row.append(round(data[j][k][1], 2))
                else:
                    row.append("")
            w.writerow(row)
    print(f"[CSV] {csv_path}")

    # 统计
    print("\n===== 抖动统计(稳态:去掉前20%) =====")
    for j in args.joints:
        pts = data[j]
        if len(pts) < 5:
            print(f"J{j}: 数据太少({len(pts)}), 跳过")
            continue
        # 去掉前20%(运动段)
        cut = int(len(pts) * 0.2)
        pts = pts[cut:]
        vals = [a for _, a in pts]
        mean = sum(vals) / len(vals)
        pkpk = max(vals) - min(vals)
        # 标准差
        var = sum((a - mean) ** 2 for a in vals) / len(vals)
        std = var ** 0.5
        # 相邻变化
        diffs = [abs(vals[i + 1] - vals[i]) for i in range(len(vals) - 1)]
        max_step = max(diffs) if diffs else 0
        # 采样率
        if len(pts) > 1:
            dt = pts[-1][0] - pts[-0][0] if (pts[-1][0] != pts[0][0]) else 0
        print(f"  J{j}: 采样{len(pts)}点 mean={mean:.2f}° 峰峰值={pkpk:.2f}° "
              f"标准差={std:.3f}° 最大相邻变化={max_step:.3f}° 成功{success[j]}次/失败{fail[j]}次")
        print(f"       {'✅ 平稳' if pkpk < 1.0 else '⚠️ 有明显抖动(>1°)'} "
              f"({'显著振荡' if pkpk > 3.0 else '轻微波动' if pkpk > 1.0 else '稳定'})")
    print("\n完成。")


if __name__ == "__main__":
    sys.exit(main())
