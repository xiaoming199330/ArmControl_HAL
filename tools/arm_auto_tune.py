#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_auto_tune.py - 六轴机械臂整臂联动 自适应修正监控 + PID 自动调参工具

目标：让 6 个舵机同时运动到目标位姿后，能平稳、准确地到位，并尽量减少
     固件自适应修正的轮数。本工具监控固件现有的
     「m命令6关节联动 → S曲线 → 多关节自适应修正(J2/J3/J4)」流程。

它做什么：
  1. 对 6 个舵机做一次 m 命令联动运动（可点对点或重复多次）
  2. 同时开启固件 pv 周期反馈，连续采集 6 个舵机的实际角度 → 画出完整
     运动+修正过程曲线（可看运动是否平稳/有无振荡/超调）
  3. 解析固件自适应修正日志：
       - J%。corr #n  : 第 n 轮修正（actual/tgt/err/gain/off）
       - OS/STALL     : 过冲/停滞（表明参数不佳）
       - auto-tune    : 固件自动调整的 P/I/D 记录
       - OK / TIMEOUT : 到位判定
       - DONE / STOP  : 完成/超时及总轮数
  4. 输出统计报表：每个舵机的修正次数、最终到位准确度、PID 被自动调节了
     多少次/从多少调到多少
  5. 自动调参模式（--sweep）：尝试多组 PID 参数，每组重复跑同一套动作，
     输出对比表，找出「修正次数最少、到位最准、最平稳」的一组

用法示例：
  # 单次监控：6关节从[150x6]联动到 m 165 161 170 182 155 200
  python arm_auto_tune.py --port COM8 \
        --start 150 150 150 150 150 150 \
        --target 165 161 170 182 155 200 --v 200

  # 自动扫参：对比 (P32/D12/I4) 和 (P48/D12/I4) 两套 PID
  python arm_auto_tune.py --port COM8 --sweep \
        --sets "32,4,12|48,4,12" --repeat 4 \
        --start 150 150 150 150 150 150 \
        --target 165 161 170 182 155 200

  # 全部使能扭矩后，使用重力补偿
  curl ... # 略

依赖：pyserial, matplotlib（隔离 venv）
固件关节编号：m 命令用物理顺序 J1..J6；修正日志 J1..J6 也是物理编号。
"""

import argparse
import csv
import datetime
import math
import os
import re
import sys
import time

import serial

# ============ 解析反馈行日志 ============
RE_FEEDBACK = re.compile(r"\s*Feedback:\s*(?:J([1-6])=(-?[0-9.]+)deg\s*)+")
RE_CORR = re.compile(
    r"J(\d)\s+corr #(\d+): actual=(-?\d+) tgt=(-?\d+) -> (-?\d+) "
    r"\(err=(-?\d+), gain=(\d+)% off=(-?\d+)\)")
RE_OK = re.compile(r"J(\d) OK: actual=(-?\d+) tgt=(-?\d+) \(err=(-?\d+)\)")
RE_TIMEOUT = re.compile(r"J(\d) TIMEOUT: actual=(-?\d+) tgt=(-?\d+) \(err=(-?\d+)\)")
RE_OVERSHOOT = re.compile(r"J(\d) OVERSHOOT: gain->(\d+)% \(osc=(\d+)\)")
RE_STALL = re.compile(r"J(\d) STALL: gain->(\d+)% \(stall=(\d+)\)")
RE_AUTOTUNE = re.compile(
    r"J(\d) auto-tune: ([PID])_Gain->(\d+) \((damp oscillation|boost torque|steady error)\)")
RE_DONE = re.compile(r"Adaptive correction DONE: all within \+/-?(\d+) deg \((\d+) rounds\)")
RE_STOP = re.compile(r"Adaptive correction STOP: max (\d+) rounds reached")
RE_READFAIL = re.compile(r"J(\d) read failed, skip")

# ============ 参数 ============
def parse_args():
    ap = argparse.ArgumentParser(description="整臂联动 自适应修正监控 + PID 自动调参")
    ap.add_argument("--port", default="COM8", help="串口 (默认 COM8)")
    ap.add_argument("--baud", type=int, default=115200)

    ap.add_argument("--start", nargs=6, type=float,
                    default=[150, 150, 150, 150, 150, 150],
                    help="起始 6 关节角度 J1..J6 (度)")
    ap.add_argument("--target", nargs=6, type=float, required=True,
                    help="目标 6 关节角度 J1..J6 (度)")
    ap.add_argument("--v", type=float, default=200,
                    help="运动速度 deg/s (默认 200)")

    ap.add_argument("--sweep", action="store_true",
                    help="自动扫参模式")
    ap.add_argument("--sets", type=str, default=None,
                    help="扫参 PID 组，竖线分隔，每组 'P,I,D'，如 32,4,12|48,4,12|64,4,20")
    ap.add_argument("--repeat", type=int, default=3,
                    help="每组重复次数（取平均），默认3")
    ap.add_argument("--joints-pid", type=str, default="2,3,4",
                    help="参与扫参的物理关节（默认 J2,J3,J4）")

    ap.add_argument("--feedback-ms", type=int, default=200,
                    help="pv 周期反馈间隔 ms (默认200)，越小越平滑但串口占用高")
    ap.add_argument("--timeout", type=float, default=30,
                    help="单次运动+修正总超时秒数 (默认30)")
    ap.add_argument("--sample-label", type=str, default=None,
                    help="本次采样名（输出文件前缀）")
    ap.add_argument("--out-dir", type=str, default="tune_results",
                    help="输出目录")
    ap.add_argument("--no-gravity", action="store_true",
                    help="不发送 G 命令（默认发 G 0.04 开启J2重力前馈）")
    ap.add_argument("--no-correction", action="store_true",
                    help="不开启自动修正（默认发 A e 1 开启，用于统计修正轮数）")
    ap.add_argument("--corr-threshold", type=float, default=None,
                    help="修正阈值度 (A t val)，默认不改")
    ap.add_argument("--corr-rounds", type=int, default=None,
                    help="修正最大轮数 (A r val)，默认不改")
    return ap.parse_args()


# ============ 串口封装 ============
class ArmConsole:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     timeout=0.3, write_timeout=1.0)
        except serial.SerialException as e:
            if "Access denied" in str(e) or "拒绝访问" in str(e) or "could not open" in str(e).lower():
                raise RuntimeError(
                    f"串口 {port} 被占用：请先关闭串口助手再运行。")
            raise
        self.reset()

    def reset(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, cmd, wait=0.2):
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()
        time.sleep(wait)

    def wait_prompt(self, timeout=3.0):
        """读到提示符 > 前的所有输出（用于命令确认）。"""
        end = time.time() + timeout
        s = ""
        while time.time() < end:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
                s += chunk
                if "> " in s:
                    break
        return s

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def read_all_joints(self, settle_ms=40):
        """逐个发送 p 0..p 5 读取 6 个舵机实际角度（固件 p 命令用 0-based
        index: p 0=J1, p 1=J2, ..., p 5=J6）。返回 {joint(1~6): angle}，
        读取失败的关节不放入结果。单点读比 pv 连续读可靠得多。"""
        result = {}
        for idx in range(6):
            # 发一个 p 命令，等固件同步回传后批量读取整块
            self.ser.write(f"p {idx}\r\n".encode("ascii"))
            self.ser.flush()
            time.sleep(0.03)  # 等固件回传（p 是同步返回，30ms 足够）
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
                # 取最后一段含 angle= 的内容
                a_pos = data.rfind("angle=")
                if a_pos >= 0:
                    num_str = data[a_pos + 6:].split(" deg", 1)[0].strip()
                    try:
                        result[idx + 1] = float(num_str)
                    except ValueError:
                        pass
            time.sleep(settle_ms / 1000.0)  # 每读间隔，降低总线负荷
        return result


# ============ 日志解析 ============
def parse_line(line, corrected_events, ok_events, timeout_events,
               autotune_events, done_flag, stop_flag, readfail_events):
    """解析一行，把识别到的事件追加到对应列表。返回该行提取到的反馈角度 dict 或 None。"""
    fb = {}
    # Feedback 行（固件输出带前导空格，用 search 匹配行内任意位置）
    m = RE_FEEDBACK.search(line)
    if m:
        for mm in re.finditer(r"J([1-6])=(-?[0-9.]+)deg", line):
            fb[int(mm.group(1))] = float(mm.group(2))
        return fb or None

    # 修正事件
    m = RE_CORR.search(line)
    if m:
        ev = {
            "joint": int(m.group(1)), "round": int(m.group(2)),
            "actual": int(m.group(3)), "tgt": int(m.group(4)),
            "cmd": int(m.group(5)), "err": int(m.group(6)),
            "gain": int(m.group(7)), "off": int(m.group(8)),
        }
        ev["kind"] = "corr"
        corrected_events.append(ev)
        return None
    m = RE_OK.search(line)
    if m:
        ok_events.append({"joint": int(m.group(1)),
                          "actual": int(m.group(2)), "tgt": int(m.group(3)),
                          "err": int(m.group(4)), "kind": "ok"})
        return None
    m = RE_TIMEOUT.search(line)
    if m:
        timeout_events.append({"joint": int(m.group(1)),
                               "actual": int(m.group(2)), "tgt": int(m.group(3)),
                               "err": int(m.group(4)), "kind": "timeout"})
        return None
    m = RE_AUTOTUNE.search(line)
    if m:
        autotune_events.append({"joint": int(m.group(1)),
                                "gain": m.group(2), "val": int(m.group(3)),
                                "reason": m.group(4), "kind": "autotune"})
        return None
    m = RE_DONE.search(line)
    if m:
        done_flag.append({"thr": int(m.group(1)), "rounds": int(m.group(2)),
                          "kind": "done"})
        return None
    m = RE_STOP.search(line)
    if m:
        stop_flag.append({"max_rounds": int(m.group(1)), "kind": "stop"})
        return None
    m = RE_READFAIL.search(line)
    if m:
        readfail_events.append({"joint": int(m.group(1)), "kind": "readfail"})
        return None
    return None


# ============ 核心采集 ============
def run_once(console, args, targets, label, pid_str, gravity, timeout_s,
             out_dir, feedback_ms):
    """执行一次完整：m运动 + 逐个p J轮询采集(可靠) + 解析修正日志。
    返回结果dict。"""
    # 采集缓冲
    series = []          # (t, {joint: angle})
    corrected_events = []
    ok_events = []
    timeout_events = []
    autotune_events = []
    done_flag = []
    stop_flag = []
    readfail_events = []
    raw_lines = []
    move_started = False  # 是否检测到 m 轨迹启动

    # 发送 m 运动（启动计时）
    start_cmd = ("m " + " ".join(f"{a:.1f}" for a in targets) +
                 f" {args.v:.1f}")
    t0 = time.monotonic()
    console.send(start_cmd, 0.3)
    # 读一下 m 的回显，确认是否 "Move started" 或被 abort
    echo = console.wait_prompt(2.0)
    raw_lines.append(echo)
    if "Move started" in echo:
        move_started = True
    elif "Move aborted" in echo:
        move_started = False
        print("    [警告] m 轨迹被中止（Move aborted: 读起始位置失败）")
    else:
        print(f"    [提示] 未识别到 m 回显标志，继续采集")

    # 主采集循环：每个循环读一轮 6 关节（逐个 p J），并解析修正日志
    loop_count = 0
    while True:
        now = time.monotonic()
        if now - t0 > timeout_s:
            print(f"    [超时] {timeout_s}s 到达，停止采集")
            break

        # 读这个时刻的 6 关节位置（逐个 p，可靠）
        ang = console.read_all_joints(settle_ms=10)
        if ang:
            series.append((now - t0, ang))

        # 读修正日志（非阻塞，取串口剩余）
        if console.ser.in_waiting:
            data = console.ser.read(console.ser.in_waiting).decode("ascii", errors="replace")
            for ln in data.splitlines():
                before_n = len(corrected_events) + len(ok_events) + \
                           len(timeout_events) + len(autotune_events)
                parse_line(ln, corrected_events, ok_events, timeout_events,
                           autotune_events, done_flag, stop_flag, readfail_events)
                if len(corrected_events) + len(ok_events) + len(timeout_events) + \
                   len(autotune_events) > before_n:
                    raw_lines.append(ln)

        loop_count += 1
        # 结束条件：修正完成/停止 且已采集一定时间
        if (done_flag or stop_flag) and (now - t0 > 2.0):
            # 再等一轮让最后数据落袋
            break
        # 每轮之间给固件处理时间
        time.sleep(0.05)

    # 组装结果
    res = {
        "label": label, "pid_str": pid_str, "targets": targets,
        "t_series": [x[0] for x in series],
        "angle_series": [x[1] for x in series],
        "corrected": corrected_events, "ok": ok_events,
        "timeout": timeout_events, "autotune": autotune_events,
        "done": done_flag, "stop": stop_flag, "readfail": readfail_events,
        "raw_lines": raw_lines, "dur": time.monotonic() - t0,
        "move_started": move_started,
    }
    return res


# ============ 统计 ============
def build_report(res, args):
    targets = res["targets"]
    rep = {}
    rep["label"] = res["label"]
    rep["pid_str"] = res["pid_str"]
    rep["dur"] = round(res["dur"], 2)
    rep["done"] = [d for d in res["done"]]
    rep["stop"] = [d for d in res["stop"]]

    # 每关节：修正次数、最后actual、最后err、到位状态、auto-tune
    joint_stat = {}
    for j in range(1, 7):
        joint_stat[j] = {
            "corr_count": 0, "last_actual": None, "last_err": None,
            "final_cmd": None, "status": "none",
            "autotune": [], "max_gain_err": None, "overshoot_count": 0,
            "stall_count": 0, "readfail": False, "feedback_count": 0,
        }
    # 修正轮次
    for ev in res["corrected"]:
        j = ev["joint"]
        joint_stat[j]["corr_count"] += 1
        joint_stat[j]["last_actual"] = ev["actual"]
        joint_stat[j]["last_err"] = ev["err"]
        joint_stat[j]["final_cmd"] = ev["cmd"]
        joint_stat[j]["status"] = "corrected"
    # OK / timeout
    for ev in res["ok"]:
        j = ev["joint"]
        joint_stat[j]["status"] = "ok"
        joint_stat[j]["last_err"] = ev["err"]
    for ev in res["timeout"]:
        j = ev["joint"]
        joint_stat[j]["status"] = "timeout"
        joint_stat[j]["last_err"] = ev["err"]
    # auto-tune
    for ev in res["autotune"]:
        j = ev["joint"]
        joint_stat[j]["autotune"].append(ev)

    # 从 pv 角度序列推导：每关节最后非空角度作为实际到位位置，
    # 用于在无修正日志时仍能评估到位精度。无数据则标记 readfail。
    last_ang = {j: None for j in range(1, 7)}
    fb_count = {j: 0 for j in range(1, 7)}
    for fb in res["angle_series"]:
        for j in range(1, 7):
            if j in fb and fb[j] is not None:
                last_ang[j] = fb[j]
                fb_count[j] += 1
    for j in range(1, 7):
        joint_stat[j]["feedback_count"] = fb_count[j]
        if joint_stat[j]["last_actual"] is None and last_ang[j] is not None:
            joint_stat[j]["last_actual"] = round(last_ang[j], 1)
            joint_stat[j]["last_err"] = round(targets[j-1] - last_ang[j], 1)
            joint_stat[j]["status"] = "feedback"
        if fb_count[j] == 0:
            joint_stat[j]["readfail"] = True

    rep["joints"] = joint_stat
    return rep


def fmt_report(rep):
    lines = []
    lines.append(f"\n===== 采样 {rep['label']} | PID[{rep['pid_str']}] | 耗时 {rep['dur']}s =====")
    status = rep["done"] and "DONE" or (rep["stop"] and "STOP(超时)" or "未完成")
    if rep["done"]:
        lines.append(f"  修正完成: {rep['done'][0]['rounds']} 轮, 阈值±{rep['done'][0]['thr']}°")
    if rep["stop"]:
        lines.append(f"  修正超时停止: 达最大 {rep['stop'][0]['max_rounds']} 轮")
    if len(rep["done"]) == 0 and len(rep["stop"]) == 0:
        lines.append("  [警告] 未检测到修正完成/停止 → 修正可能未触发（常见：舵机位置读取失败使 m 轨迹中止）")
    lines.append("  J  修正轮  实际到位  误差  有反馈  状态   auto-tune")
    for j in range(1, 7):
        st = rep["joints"][j]
        if st["status"] == "none" and st["last_actual"] is None and st["feedback_count"] == 0:
            continue
        fb_note = f"{st['feedback_count']}次" if st["feedback_count"] else ("ERR" if st["readfail"] else "0")
        if st["readfail"] and st["feedback_count"] == 0:
            continue  # 完全读不到的关节单列说明
        lines.append(
            f"  J{j}  {st['corr_count']:>3}   {str(st['last_actual']):>8}  "
            f"{str(st['last_err']):>7}  {fb_note:>6}  {st['status']:>7}   {len(st['autotune'])}")
    # readfail 汇总
    rf = [j for j in range(1, 7) if rep["joints"][j]["readfail"] and rep["joints"][j]["feedback_count"] == 0]
    if rf:
        lines.append(f"  [总线告警] 以下关节位置读取完全失败(读到0次): " + ", ".join(f"J{j}" for j in rf) +
                     " → 检查对应舵机ID/总线/USB是否正常")
    if any(re["autotune"] for re in rep["joints"].values()):
        lines.append("  --- PID 自动调节过程 ---")
        for j in range(1, 7):
            for ev in rep["joints"][j]["autotune"]:
                lines.append(f"    J{j}: {ev['gain']}_Gain->{ev['val']} ({ev['reason']})")
    return "\n".join(lines)


def save_csv(res, out_path):
    targets = res["targets"]
    with open(out_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["time_s"] + [f"J{i}_deg" for i in range(1, 7)] +
                   [f"J{i}_tgt" for i in range(1, 7)])
        for t, fb in zip(res["t_series"], res["angle_series"]):
            row = [round(t, 3)] + [fb.get(i, "") for i in range(1, 7)] + \
                  [round(targets[i-1], 1) for i in range(1, 7)]
            w.writerow(row)


def plot_results(res_map, out_path):
    """画多个采样（同一次扫参的多组）的6关节曲线对比。res_map: {pid_label: res}"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(18, 8))
    axes = axes.flatten()
    colors = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#e377c2"]
    for j in range(1, 7):
        ax = axes[j-1]
        for k, (pid_label, res) in enumerate(res_map.items()):
            t = res["t_series"]
            ang = [fb.get(j) for fb in res["angle_series"]]
            ax.plot(t, ang, "-", linewidth=1.5, color=colors[k % len(colors)],
                    label=pid_label)
        # 目标线
        tgt = [r["targets"][j-1] for r in res_map.values()][0]
        ax.axhline(tgt, color="gray", ls="--", alpha=0.6, label=f"tgt {tgt:.0f}°")
        ax.set_title(f"J{j}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("deg")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("6-Axis Joint Positions during motion + adaptive correction")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(out_path, dpi=120)
    plt.close()


def main():
    args = parse_args()
    if args.sweep and not args.sets:
        print("[错误] --sweep 需配合 --sets 'P,I,D|P,I,D'")
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    try:
        console = ArmConsole(args.port, args.baud)
    except RuntimeError as e:
        print(f"[错误] {e}")
        return 1

    # 初始化
    try:
        print("[i] 初始化 ...")
        console.send("i", 1.0)
        console.wait_prompt(3)
        print("[t] 使能扭矩 ...")
        console.send("t a 1", 0.8)
        console.wait_prompt(2)
        if not args.no_gravity:
            print("[G] 开启 J2 重力前馈 0.04 ...")
            console.send("G 0.04", 0.5)
            console.wait_prompt(2)
        # 自动修正（默认开启，用于统计修正轮数）
        if args.no_correction:
            print("[修正] 保持关闭 (A e 0)")
            console.send("A e 0", 0.5)
            console.wait_prompt(2)
        else:
            print("[修正] 开启自适应 (A e 1)")
            console.send("A e 1", 0.5)
            console.wait_prompt(2)
        if args.corr_threshold is not None:
            console.send(f"A t {args.corr_threshold:.0f}", 0.4)
            console.wait_prompt(2)
        if args.corr_rounds is not None:
            console.send(f"A r {args.corr_rounds}", 0.4)
            console.wait_prompt(2)
    except Exception as e:
        print(f"[错误] 初始化阶段失败: {e}")
        console.close()
        return 1

    # 先运动到起始位
    start_cmd = ("m " + " ".join(f"{a:.1f}" for a in args.start) + " 200.0")
    print("[起始] 回到初始位 ...")
    console.send(start_cmd, 0.5)
    time.sleep(3)  # 等到位

    # ---- 收集多组结果 ----
    res_map = {}   # pid_label -> res
    report_lines = []
    if args.sweep:
        pid_sets = [s.split(",") for s in args.sets.split("|")]
        pid_sets = [(int(p), int(i), int(d)) for p, i, d in pid_sets]
        joints_pid = [int(x) for x in args.joints_pid.split(",")]
        for (P, I, D) in pid_sets:
            label = f"P{P}_I{I}_D{D}"
            print(f"\n===== 扫参 {label} =====")
            # 设 PID（N 用物理编号）
            for j in joints_pid:
                console.send(f"N {j} {P} {I} {D}", 0.8)
                console.wait_prompt(2)
            # 重复多次取平均（这里只跑 repeat 次，report 取最后一次，另有趋势由曲线呈现）
            last_res = None
            for r in range(max(1, args.repeat)):
                print(f"  运行 {r+1}/{args.repeat} ...")
                res = run_once(console, args, args.target, f"{label}_#{r+1}",
                              label, args.no_gravity, args.timeout, args.out_dir, args.feedback_ms)
                # 每轮单独存 CSV
                csv_path = os.path.join(args.out_dir, f"{ts}_{label}_run{r+1}.csv")
                save_csv(res, csv_path)
                rep = build_report(res, args)
                report_lines.append(fmt_report(rep))
                report_lines.append(f"    数据: {csv_path}")
                last_res = res
            res_map[label] = last_res
    else:
        label = args.sample_label or "once"
        print(f"\n===== 单次监控 {label} =====")
        res = run_once(console, args, args.target, label, "current",
                      args.no_gravity, args.timeout, args.out_dir, args.feedback_ms)
        csv_path = os.path.join(args.out_dir, f"{ts}_{label}.csv")
        save_csv(res, csv_path)
        rep = build_report(res, args)
        report_lines.append(fmt_report(rep))
        report_lines.append(f"    数据: {csv_path}")
        res_map[label] = res

    console.close()

    # 输出报告
    report_path = os.path.join(args.out_dir, f"{ts}_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report_lines))
    print("\n".join(report_lines))

    # 画对比图（多组或单组）
    plot_path = os.path.join(args.out_dir, f"{ts}_joint_curves.png")
    try:
        plot_results(res_map, plot_path)
        print(f"\n[图] {plot_path}")
    except Exception as e:
        print(f"[图] 失败: {e}")

    # 扫参总结表
    if args.sweep:
        print("\n===== 扫参对比（修正轮数/状态）=====")
        for label, res in res_map.items():
            rep = build_report(res, args)
            nc = sum(rep["joints"][j]["corr_count"] for j in range(1, 7))
            status = "DONE" if rep["done"] else ("STOP" if rep["stop"] else "?")
            print(f"  {label:>12}: 修正轮总和={nc}, 状态={status}, 耗时={rep['dur']}s")
        print(f"\n[报告] {report_path}")

    print("\n完成。请把 CSV/PNG/report 发给我分析，或告诉我需要调整哪组 PID。")


if __name__ == "__main__":
    sys.exit(main())
