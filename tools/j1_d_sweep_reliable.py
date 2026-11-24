#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
j1_d_sweep_reliable.py - 针对单个关节(J1)在前伸位用可靠慢速读取扫D-Gain

目的：找到能把 J1 波动压到最小的 D-Gain。
方法修正（关键）：不用高频多关节轮询（会造成读数错乱假抖动），
改为对单个关节发 `p J` + sleep 0.4s + 批量读 + 重试失败读数，
可靠地测量真实角度波动。

用法：
  python j1_d_sweep_reliable.py --port COM8 --joint 1 --dvalues 12 16 20 24 --points 30
"""

import argparse
import re
import sys
import time

import serial


def parse_args():
    ap = argparse.ArgumentParser(description="单关节可靠D-Gain扫参(前伸位)")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--joint", type=int, default=1, help="物理关节号 1~6")
    ap.add_argument("--dvalues", nargs="+", type=int, default=[12, 16, 20, 24],
                    help="要扫的 D 值")
    ap.add_argument("--points", type=int, default=30,
                    help="每档采样点数")
    ap.add_argument("--sleep", type=float, default=0.4,
                    help="发 p 后等待秒数(可靠读取)")
    return ap.parse_args()


def read_angle(ser, idx, sleep_s):
    """可靠读取单个关节角度。返回角度或 None。"""
    for attempt in range(3):
        ser.reset_input_buffer()
        ser.write(f"p {idx}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(sleep_s)
        buf = ser.read(ser.in_waiting) if ser.in_waiting else b""
        txt = buf.decode("ascii", errors="replace")
        m = re.search(r"angle=(-?[0-9.]+)", txt)
        if m:
            v = float(m.group(1))
            if v > 0:
                return v
        time.sleep(0.2)
    return None


def measure(ser, joint, points, sleep_s):
    """测关节 points 个点，返回 (angle_list, success_count)。"""
    idx = joint - 1
    vals = []
    for _ in range(points):
        v = read_angle(ser, idx, sleep_s)
        if v is not None:
            vals.append(v)
        time.sleep(0.1)
    return vals


def main():
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"[错误] 串口 {args.port} 被占用或无响应: {e}")
        print("请先关闭串口助手再运行。")
        return 1
    ser.reset_input_buffer()

    idx = args.joint - 1
    print(f"单关节 J{args.joint} D-Gain 可靠扫参")
    print(f"方法: p {idx} + sleep {args.sleep}s + 重试, 每档 {args.points} 点")
    print("-" * 60)

    results = []  # (d, mean, pkpk, std, ok_count)
    for d in args.dvalues:
        # 设 D
        ser.reset_input_buffer()
        ser.write(f"D {args.joint} {d}\r\n".encode("ascii"))
        ser.flush()
        time.sleep(0.6)
        # 清掉回显
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        time.sleep(0.5)  # 等舵机响应新 D
        # 测
        vals = measure(ser, args.joint, args.points, args.sleep)
        if len(vals) < 5:
            print(f"  D={d}: 读不到数据({len(vals)}点)")
            results.append((d, None, None, None, len(vals)))
            continue
        mean = sum(vals) / len(vals)
        pkpk = max(vals) - min(vals)
        std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        results.append((d, mean, pkpk, std, len(vals)))
        print(f"  D={d}: mean={mean:.2f}° 峰峰={pkpk:.2f}° std={std:.3f}° ({len(vals)}点)")

    ser.close()

    # 找最优
    print("-" * 60)
    best = None
    for d, mean, pkpk, std, n in results:
        if std is not None:
            if best is None or std < best[1]:
                best = (d, std)
    if best:
        print(f"J{args.joint} 最优 D={best[0]} (std={best[1]:.3f}°)")
    print(f"请确认后用 F 保存 (保存所有关节当前 PID)")


if __name__ == "__main__":
    sys.exit(main())
