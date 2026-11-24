#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_move_test.py - 自动化 P0 <-> g150,0,100 往复运动观测

用途：自动循环执行 P 0 -> g 150 0 100 -> P 0 ... 若干轮，用可靠方式
观测关键关节(J2为主)的运动过程与到位稳定性，输出每轮的运动时间曲线
和到位抖动特征。供调参闭环使用。

用法：
  python auto_move_test.py --port COM8 --rounds 3 --track 2 3 --settle 8
"""
import argparse
import csv
import re
import sys
import time

import serial


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rounds", type=int, default=3, help="往复轮数")
    ap.add_argument("--track", nargs="+", type=int, default=[2, 3],
                    help="要跟踪的物理关节")
    ap.add_argument("--settle", type=float, default=8.0,
                    help="到位后稳定观察秒数")
    ap.add_argument("--out", default="auto_move")
    return ap.parse_args()


class Arm:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.4)
        except serial.SerialException as e:
            raise SystemExit(f"[错误] {port} 占用或无响应: {e}")
        self.reset()

    def reset(self):
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.reset()
        self.ser.write(f"{cmd}\r\n".encode("ascii"))
        self.ser.flush()

    def read_angle(self, phys_joint):
        idx = phys_joint - 1
        for _ in range(3):
            self.ser.reset_input_buffer()
            self.ser.write(f"p {idx}\r\n".encode("ascii"))
            self.ser.flush()
            time.sleep(0.3)
            buf = self.ser.read(self.ser.in_waiting) if self.ser.in_waiting else b""
            m = re.search(r"angle=(-?[0-9.]+)", buf.decode("ascii", errors="replace"))
            if m:
                v = float(m.group(1))
                if v > 0:
                    return v
            time.sleep(0.15)
        return None

    def read_many(self, joints):
        return {j: self.read_angle(j) for j in joints}

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def move_and_track(arm, joints, cmd, duration, label, settle):
    """发运动命令，跟踪关节过程(duration秒)，然后到位后稳定观察(settle秒)。"""
    rec = {"label": label, "t": [], "ang": {j: [] for j in joints}, "logs": []}
    arm.send(cmd)
    t0 = time.time()
    # 运动过程采样(每0.5秒一轮,读所有track关节)
    while time.time() - t0 < duration:
        t = time.time() - t0
        angs = arm.read_many(joints)
        for j in joints:
            if angs.get(j) is not None:
                rec["t"].append(round(t, 2))
                rec["ang"][j].append(angs[j])
        # 抓日志(hold/corr)
        if arm.ser.in_waiting:
            d = arm.ser.read(arm.ser.in_waiting).decode("ascii", errors="replace")
            for ln in d.splitlines():
                if any(k in ln.lower() for k in ["hold", "resume", "corr", "abort"]):
                    rec["logs"].append((round(t, 1), ln.strip()))
        time.sleep(0.4)
    # 到位后稳定观察
    st = {j: [] for j in joints}
    s0 = time.time()
    while time.time() - s0 < settle:
        angs = arm.read_many(joints)
        for j in joints:
            if angs.get(j) is not None:
                st[j].append(angs[j])
        time.sleep(0.4)
    rec["settle"] = st
    return rec


def summarize(rec, joints):
    lines = [f"  [{rec['label']}] 运动采样{len(rec['t'])}点, 日志{len(rec['logs'])}条"]
    for ln in rec["logs"][:8]:
        lines.append(f"      {ln[1]}")
    for j in joints:
        vals = rec["settle"].get(j, [])
        if vals:
            mean = sum(vals) / len(vals)
            pk = max(vals) - min(vals)
            std = (sum((x - mean) ** 2 for x in vals) / len(vals)) ** 0.5
            lines.append(f"      J{j} 到位稳: mean={mean:.2f}° 峰峰={pk:.2f}° std={std:.3f}° ({len(vals)}点)")
        else:
            lines.append(f"      J{j} 到位无采样")
    return "\n".join(lines)


def main():
    args = parse_args()
    arm = Arm(args.port, args.baud)
    joints = args.track
    all_recs = []
    print(f"自动往复测试: {args.rounds}轮, P0 <-> g150,0,100, 跟踪J{joints}")
    for r in range(args.rounds):
        print(f"\n===== 轮{r+1}/{args.rounds} =====")
        # P 0 (回到HOME竖直)
        rec1 = move_and_track(arm, joints, "P 0", 12, f"P0_#{r+1}", args.settle)
        all_recs.append(rec1)
        print(summarize(rec1, joints))
        # g 150 0 100 (低位前伸)
        rec2 = move_and_track(arm, joints, "g 150 0 100 0", 12, f"g100_#{r+1}", args.settle)
        all_recs.append(rec2)
        print(summarize(rec2, joints))
    arm.close()

    # 汇总到CSV
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = f"{args.out}_{ts}.csv"
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["phase", "t", "joint", "angle"])
        for rec in all_recs:
            for i, t in enumerate(rec["t"]):
                for j in joints:
                    if i < len(rec["ang"][j]):
                        w.writerow([rec["label"], t, j, round(rec["ang"][j][i], 2)])
    print(f"\n[CSV] {path}")
    print("完成。")


if __name__ == "__main__":
    sys.exit(main())
