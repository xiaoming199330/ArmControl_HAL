#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
d_gain_sweep.py - X-Gain(P/I/D) 自动扫参降抖动工具

用户反馈 J1/J2 到位后小幅抖动(P/D 振荡)。本工具对指定关节自动扫描
D-Gain(P-Gain/I-Gain也可)从低到高，每档测一段时间抖动(峰峰值/标准差)，
输出对比表，直接标出最稳定的一档。

用法：
  python d_gain_sweep.py --port COM8 --joints 1 2 \
        --gain D --values 12 16 20 24 28 32 --seconds 8

说明：
  - 在机械臂当前位置直接测抖动(不运动)，因抖动是PID振荡，与位置强相关
    但就近测试足以比较不同Gain的稳定性
  - 每档用 D J val 命令设置后测 N 秒
"""

import argparse
import csv
import datetime
import sys
import time

import serial


def parse_args():
    ap = argparse.ArgumentParser(description="X-Gain 自动扫参降抖动")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--joints", nargs="+", type=int, default=[1, 2],
                    help="目标物理关节 1~6")
    ap.add_argument("--gain", default="D", choices=["P", "I", "D"],
                    help="扫的参数类型 (P/I/D)")
    ap.add_argument("--values", nargs="+", type=int, required=True,
                    help="要扫描的增益值，如 12 16 20 24")
    ap.add_argument("--seconds", type=float, default=8,
                    help="每档测抖动时长(秒)")
    ap.add_argument("--set-cmd", default=None,
                    help="自定义设置命令模板，如 '{gain} {j} {v}'（默认 D J v）")
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

    def send(self, cmd, wait=0.3):
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\r\n".encode("ascii"))
        self.ser.flush()
        time.sleep(wait)

    def read_one(self, phys_joint):
        """读单个物理关节角度。index=phys-1。返回角度或None。"""
        idx = phys_joint - 1
        self.ser.reset_input_buffer()
        self.ser.write(f"p {idx}\r\n".encode("ascii"))
        self.ser.flush()
        time.sleep(0.02)
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


def measure(console, joints, seconds):
    """测 joints 每个关节 seconds 秒，返回 {joint: (mean, pkpk, std, samples)}"""
    data = {j: [] for j in joints}
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        for j in joints:
            ang = console.read_one(j)
            if ang is not None:
                data[j].append(ang)
    res = {}
    for j in joints:
        vals = data[j]
        if len(vals) < 5:
            res[j] = (None, None, None, len(vals))
            continue
        mean = sum(vals) / len(vals)
        pkpk = max(vals) - min(vals)
        std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        res[j] = (mean, pkpk, std, len(vals))
    return res


def main():
    args = parse_args()
    console = ArmConsole(args.port, args.baud)

    # 先读取当前各关节增益做基线
    print(f"[基线] 读取当前各关节 {args.gain}-Gain ...")
    base = {}
    for j in args.joints:
        console.send(f"{args.gain} {j}", 0.3)
        base[j] = None  # 不解析，仅示意

    # 对每个关节：逐档扫
    all_rows = []
    best = {j: None for j in args.joints}  # j -> (val, std)
    for j in args.joints:
        print(f"\n===== 关节 J{j} 扫参 {args.gain}-Gain =====")
        for v in args.values:
            # 设置该关节增益
            console.send(f"{args.gain} {j} {v}", 0.4)
            print(f"  设置 J{j} {args.gain}={v} ...")
            # 等一点让舵机响应
            time.sleep(0.5)
            # 测抖动
            res = measure(console, [j], args.seconds)
            mean, pkpk, std, ns = res[j]
            if mean is None:
                print(f"    {args.gain}={v}: 数据不足({ns}点)")
                all_rows.append((j, v, None, None, None, ns))
                continue
            verdict = "✅最稳" if (best[j] is None or std < best[j][1]) else ""
            if best[j] is None or std < best[j][1]:
                best[j] = (v, std)
            print(f"    {args.gain}={v}: mean={mean:.1f}° 峰峰={pkpk:.2f}° "
                  f"std={std:.3f}° ({ns}点) {verdict}")
            all_rows.append((j, v, mean, pkpk, std, ns))
            time.sleep(0.3)

    console.close()

    # 输出对比表
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"dgain_sweep_{ts}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["joint", f"{args.gain}_gain", "mean_deg", "pkpk_deg", "std_deg", "samples"])
        for row in all_rows:
            w.writerow(row)
    print(f"\n[CSV] {csv_path}")

    print("\n===== 最优结果 =====")
    for j in args.joints:
        b = best[j]
        if b:
            print(f"  J{j}: 最优 {args.gain}={b[0]} (std={b[1]:.3f}°)")
        else:
            print(f"  J{j}: 无结果")
    print("\n请查看 CSV 对比，选出各关节最稳的增益值。")
    print(f"确认后用命令保存: N J P I D  或分别 D J / I J / K J (J为物理编号)")


if __name__ == "__main__":
    sys.exit(main())
