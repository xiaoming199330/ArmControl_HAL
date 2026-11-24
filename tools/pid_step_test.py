#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_step_test.py - STS3215 PID 阶跃响应自动测试脚本

功能：
  1. 通过 USART6 控制台（115200）与 STM32 通信
  2. 对指定关节执行 `a J deg` 阶跃运动
  3. 以固定周期轮询 `p J` 采集实际角度，记录时间-角度序列
  4. 自动分析：超调量(OS)、上升时间(tr)、峰值时间(tp)、
     稳定时间(ts, 进入±2%窗口)、稳态误差(ess)
  5. 输出 CSV + 阶跃响应曲线图(PNG)，方便观察不同 PID 的效果

依赖：pyserial, matplotlib（已装入隔离 venv）
      C:/Users/Administrator/.workbuddy/binaries/python/envs/default/Scripts/python.exe

用法：
  python pid_step_test.py --port COM8 --joint 2 \
        --start 165 --end 200 --p 32 --i 4 --d 12 \
        --save 1 --samples 200

常用命令速查（手动）：
  i          # 初始化（关节模式+写PID+扭矩OFF）
  t a 1      # 使能全部扭矩
  N J P I D  # 设置关节J的 PID（N 2 32 4 12）
  a J deg    # 单关节阶跃
  p J        # 读单关节实际角度
  F          # 保存PID到Flash（下次上电自动生效）
"""

import argparse
import csv
import math
import sys
import time

import serial


def parse_args():
    ap = argparse.ArgumentParser(description="STS3215 PID 阶跃响应自动测试")
    ap.add_argument("--port", default="COM8", help="串口号，如 COM8")
    ap.add_argument("--baud", type=int, default=115200, help="波特率")
    ap.add_argument("--joint", type=int, default=2,
                    help="被测物理关节号 1~6（J1=1, J2=2, ..., J6=6）")
    ap.add_argument("--start", type=float, default=165, help="阶跃起始角度(度)")
    ap.add_argument("--end", type=float, default=200, help="阶跃目标角度(度)")
    ap.add_argument("--p", type=int, default=None, help="P-Gain(不指定则不改)")
    ap.add_argument("--i", type=int, default=None, help="I-Gain(不指定则不改)")
    ap.add_argument("--d", type=int, default=None, help="D-Gain(不指定则不改)")
    ap.add_argument("--samples", type=int, default=200,
                    help="采样点数（每个间隔20ms，200点约4秒）")
    ap.add_argument("--interval_s", type=float, default=0.02,
                    help="采样间隔(秒)，默认0.02s=20ms")
    ap.add_argument("--save", type=int, default=1, help="1=保存PID到Flash，0=不保存")
    ap.add_argument("--out", default="pid_step", help="输出文件名前缀")
    return ap.parse_args()


class PortBusyError(Exception):
    """串口被其他程序占用"""
    pass


class ArmConsole:
    """与 STM32 USART6 控制台通信的封装"""

    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     timeout=0.3, write_timeout=1.0)
        except serial.SerialException as e:
            if "Access denied" in str(e) or "拒绝访问" in str(e) or \
               "PermissionError" in str(e) or "could not open" in str(e).lower():
                raise PortBusyError(
                    f"串口 {port} 被占用：请先关闭串口助手（XCOM/SSCOM/SSCOM32/串口调试工具等），"
                    f"再重新运行本脚本。同一个串口只能被一个程序打开。")
            raise
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, cmd):
        """发送一行命令（自动加 \\r\\n）"""
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def read_until_prompt(self, timeout_s=2.0):
        """读到 '\\r\\n> ' 提示符前的所有输出"""
        end = time.time() + timeout_s
        buf = b""
        while time.time() < end:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
                if b"> " in buf:
                    break
        return buf.decode("ascii", errors="replace")

    def cmd(self, command, delay_s=0.3):
        """发送命令并读回（返回输出文本）"""
        self.ser.reset_input_buffer()
        self.send(command)
        out = self.read_until_prompt()
        time.sleep(delay_s)
        return out

    def read_single_angle(self, joint_phys):
        """读取物理关节 joint_phys(1~6) 的实际角度。
        发送 'p <idx>'（idx=joint_phys-1），解析 'angle=<d> deg'。"""
        idx = joint_phys - 1
        self.ser.reset_input_buffer()
        self.send(f"p {idx}")
        end = time.time() + 1.0
        line = b""
        while time.time() < end:
            c = self.ser.read(1)
            if not c:
                continue
            line += c
            if b"\n" in line:
                break
        txt = line.decode("ascii", errors="replace")
        # 解析 angle=<d> deg
        a_pos = txt.find("angle=")
        if a_pos < 0:
            return None
        rest = txt[a_pos + 6:]
        num_str = rest.split(" deg", 1)[0].strip()
        try:
            return float(num_str)
        except ValueError:
            return None

    @staticmethod
    def parse_feedback_line(line):
        """从 'Feedback: J1=155.3deg J2=162.5deg ...' 解析6关节角度。
        返回 dict {joint_index(0~5): angle} 或 None（本行非反馈）。"""
        s = line.strip()
        if not s.startswith("Feedback:"):
            return None
        result = {}
        # 用正则找 J<k>=<num>deg
        import re
        for m in re.finditer(r"J([1-6])=(-?[0-9.]+)deg", s):
            idx = int(m.group(1)) - 1
            try:
                result[idx] = float(m.group(2))
            except ValueError:
                pass
        return result

    def start_pv(self, interval_ms):
        """开启固件周期反馈（pv <ms>）。"""
        self.ser.reset_input_buffer()
        self.send(f"pv {interval_ms}")
        time.sleep(0.3)
        self.ser.reset_input_buffer()  # 清掉启动行的回显

    def stop_pv(self):
        self.send("pv 0")
        time.sleep(0.3)

    def close(self):
        self.ser.close()


def analyze(name, t, y, y_target):
    """分析阶跃响应特性"""
    y0 = y[0]
    dy = y_target - y0
    if abs(dy) < 1e-9:
        return {"note": "零阶跃，跳过分析"}

    # 稳态（取后 10% 平均）
    n = len(y)
    y_ss = sum(y[int(0.9 * n):]) / max(1, n - int(0.9 * n))
    ess = y_target - y_ss          # 稳态误差
    band = 0.02 * abs(dy)          # ±2% 窗口

    # 峰值 & 超调
    peak_idx = max(range(n), key=lambda i: abs(y[i] - y0))
    peak = y[peak_idx]
    os_v = (peak - y_target) / abs(dy) * 100.0  # 超调量%%
    sgn = 1 if dy > 0 else -1
    pid = [i for i in range(n) if sgn * (y[i] - y0) > 0.9 * abs(dy)]
    tr = t[pid[0]] - t[0] if pid else None          # 10%? 简化：首次达90%
    # 更标准的上升时间：首次达 10% 到 90%
    y10 = y0 + 0.1 * dy
    y90 = y0 + 0.9 * dy
    idx10 = next((i for i in range(n) if sgn * (y[i] - y10) >= 0), None)
    idx90 = next((i for i in range(n) if sgn * (y[i] - y90) >= 0), None)
    tr = (t[idx90] - t[idx10]) if (idx10 is not None and idx90 is not None) else None

    # 峰值时间
    if sgn * (peak - y_target) > 0:
        tp = t[peak_idx]
    else:
        tp = None

    # 稳定时间(进入±2%并保持)
    ts = None
    for i in range(n):
        if all(abs(y[k] - y_target) <= band for k in range(i, n)):
            ts = t[i]
            break

    return {
        "y0": round(y0, 2), "y_target": y_target,
        "y_steady": round(y_ss, 2), "ess": round(ess, 2),
        "peak": round(peak, 2), "overshoot_pct": round(os_v, 2),
        "rise_time_s": round(tr, 3) if tr is not None else None,
        "peak_time_s": round(tp, 3) if tp is not None else None,
        "settle_time_s": round(ts, 3) if ts is not None else None,
    }


def main():
    args = parse_args()
    try:
        console = ArmConsole(args.port, args.baud)
    except PortBusyError as e:
        print(f"[错误] {e}")
        return 1
    except Exception as e:
        print(f"[错误] 无法打开串口 {args.port}: {e}")
        return 1
    print(f"[连接] {args.port} @ {args.baud} OK")
    try:
        # 0) 初始化
        print("[初始化] ...")
        console.cmd("i", 1.0)
        # 1) 使能扭矩
        print("[使能扭矩] t a 1 ...")
        console.cmd("t a 1", 0.6)

        # 固件编号说明：
        #   - 控制命令 a/p/t 用「0-based index」：a 0=J1, a 1=J2, a 2=J3 ...
        #   - PID 命令 N/K/I/D 用「物理编号」：N 1=J1, N 2=J2 ...
        #   - pv 输出 Feedback: J1..J6 为物理编号
        phys = args.joint          # 物理关节 1~6
        idx = phys - 1             # 固件 a/p 命令的 index

        # 2) 设置 PID（N/K 用物理编号）
        pid_note = "保持当前 PID"
        if args.p is not None and args.i is not None and args.d is not None:
            console.cmd(f"N {phys} {args.p} {args.i} {args.d}", 0.6)
            pid_note = f"P={args.p} I={args.i} D={args.d}"
            print(f"[PID] J{phys}: {pid_note}")
        elif args.p is not None:
            console.cmd(f"K {phys} {args.p}", 0.4)
            pid_note = f"P={args.p}"
            print(f"[PID] J{phys}: {pid_note}")

        # 3) 先运动到起始角度，等待到位（a 用 index）
        print(f"[准备] 运动到起始 {args.start}° ...")
        console.cmd(f"a {idx} {args.start}", 1.0)
        time.sleep(1.5)  # 等稳定
        y0 = console.read_single_angle(phys)
        if y0 is None:
            print("[警告] 读不到起始角度，仍继续")
        else:
            print(f"[起始] J{phys} 当前={y0}°")

        # 4) 开启固件周期反馈 pv（流式采集，无需逐个发命令）
        #    回传格式：Feedback: J1=..deg J2=..deg ...
        pv_ms = max(10, int(args.interval_s * 1000))
        print(f"[采集] 开启 pv {pv_ms}ms 周期反馈 ...")
        console.start_pv(pv_ms)

        # 5) 发送阶跃目标（t=0）
        print(f"[阶跃] 发送 {args.start}° -> {args.end}° ({args.samples} 点, dt={args.interval_s}s)")
        t_mark = time.monotonic()
        console.send(f"a {idx} {args.end}")
        t0 = time.monotonic()

        # 流式读取采样：向空闲方向对齐采样间隔
        t = []
        y = []
        next_posix = t0
        last_ok = 0
        while True:
            now = time.monotonic()
            if now >= next_posix:
                # 读到一条完整 feedback 行
                line = console.ser.readline().decode("ascii", errors="replace")
                if line:
                    fb = console.parse_feedback_line(line)
                    # fb 的 key 是 index(0~5)，物理关节 phys 对应 index=idx
                    if fb and idx in fb:
                        t.append(now - t0)
                        y.append(fb[idx])
                        last_ok += 1
                        if last_ok % 50 == 0:
                            print(f"    ... 已采 {last_ok} 点 t={now-t0:.2f}s")
                next_posix += args.interval_s
            # 结束条件：采样够数 或 超时（防止流丢失卡死）
            if last_ok >= args.samples:
                break
            if now - t0 > (args.samples * args.interval_s) + 5.0:
                print(f"    ... 超时，已采 {last_ok} 点")
                break
            time.sleep(0.001)

        console.stop_pv()
        if last_ok < 5:
            print("[错误] 有效采样点太少，无法分析")
            return
        yf = [v for v in y if not math.isnan(v)]
        tf = [t[i] for i in range(len(y)) if not math.isnan(y[i])]
        res = analyze("", tf, yf, args.end)

        # 6) 写 CSV
        csv_path = f"{args.out}_J{args.joint}.csv"
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["time_s", f"J{args.joint}_deg"])
            for a, b in zip(t, y):
                w.writerow([round(a, 4), (round(b, 2) if not math.isnan(b) else "")])
        print(f"[CSV] {csv_path}")

        # 7) 绘图
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10, 5))
            plt.plot(tf, yf, "-", linewidth=1.6, label=f"J{args.joint} actual")
            plt.axhline(args.start, color="gray", ls="--", alpha=0.6, label=f"start {args.start}°")
            plt.axhline(args.end, color="red", ls="--", alpha=0.6, label=f"target {args.end}°")
            plt.xlabel("Time (s)")
            plt.ylabel("Angle (deg)")
            plt.title(f"PID Step Response  J{args.joint}  ({pid_note})")
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            png_path = f"{args.out}_J{args.joint}.png"
            plt.savefig(png_path, dpi=120)
            print(f"[图] {png_path}")
        except Exception as e:
            print(f"[图] 生成失败: {e}")

        # 8) 打印分析结果
        print("\n===== 阶跃响应分析 =====")
        print(f"  起始角度     y0     = {res['y0']}°")
        print(f"  目标角度           = {res['y_target']}°")
        print(f"  稳态角度     y_ss  = {res['y_steady']}°")
        print(f"  稳态误差     ess   = {res['ess']}°")
        print(f"  峰值               = {res['peak']}°")
        print(f"  超调量       OS    = {res['overshoot_pct']}%")
        print(f"  上升时间     tr    = {res['rise_time_s']} s")
        print(f"  峰值时间     tp    = {res['peak_time_s']} s")
        print(f"  稳定时间     ts    = {res['settle_time_s']} s")
        print("==============================\n")

        # 9) 保存 PID 到 Flash（可选）
        if args.save:
            print("[保存] 写Flash ...")
            out = console.cmd("F", 1.0)
            if "saved" in out.lower():
                print("[保存] OK，下次上电自动生效")
            else:
                print("[保存] 输出:", out.strip().replace("\n", " "))
    finally:
        console.close()
        print("[关闭] 串口已关闭")


if __name__ == "__main__":
    sys.exit(main())
