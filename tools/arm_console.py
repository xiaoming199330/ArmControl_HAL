#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_console.py — 直接对臂的调试串口敲命令并取回打印（不用人手敲）

为什么值得单独有个工具:
  臂侧唯一的观测面就是这几行串口打印(`cn` / `ct` / `cp` / `can` …), 而 CAN 联调
  的判据恰恰是**两侧对拍**。人手敲一遍、跑一次 PC 脚本、再敲一遍, 中间很容易
  "对不上时间点"(比如把上一轮的计数当成这一轮的)。把它脚本化之后,
  「取臂侧计数 → PC 侧发帧 → 再取臂侧计数」可以一次跑完, 顺序严格可控。

用法:
  python tools/arm_console.py --port COM11 --cmd cn
  python tools/arm_console.py --port COM11 --cmd "ct 200"        # 发 200 帧自检
  python tools/arm_console.py --port COM11 --cmd cn --cmd ct     # 连敲多条
  python tools/arm_console.py --list                             # 列出候选串口

⚠ 控制台是「多字符命令 + 单键」混排: 只发完整命令 + \\r, 绝不发裸的单字符,
   否则会掉进单键模式(数字键会写 RAM 零点, 误按 s 就固化坏零点 —— 见工程红线)。
"""
import argparse
import sys
import time

# 已知的两个控制台口(见工作区记忆): 两臂 = COM11 / COM22 (CH340 @115200)
KNOWN_PORTS = ("COM11", "COM22", "COM8")


def read_for(ser, seconds, quiet=False):
    """持续读 seconds 秒(非阻塞分段收), 返回 bytes。"""
    out = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        chunk = ser.read(4096)
        if chunk:
            out += chunk
    if not quiet:
        sys.stdout.write(out.decode("utf-8", "replace"))
        sys.stdout.flush()
    return bytes(out)


def send(port, cmds, baud=115200, first_wait=0.3, per_cmd=1.6, quiet=False,
         settle=0.2):
    """往 port 依次发 cmds(每条自己带 \\r), 返回拼接后的全部回显。"""
    import serial
    ser = serial.Serial(port, baud, timeout=0.05)
    try:
        time.sleep(first_wait)
        ser.reset_input_buffer()
        acc = bytearray()
        for c in cmds:
            cmd = c if c.endswith("\r") else c + "\r"
            if not quiet:
                print("\n$ %s" % cmd.strip())
            ser.write(cmd.encode("ascii", "replace"))
            acc += read_for(ser, per_cmd, quiet)
            time.sleep(settle)
        return bytes(acc)
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="臂调试串口命令执行器")
    ap.add_argument("--port", help="串口, 如 COM11")
    ap.add_argument("--cmd", action="append", default=[],
                    help="要敲的命令(可重复, 如 --cmd cn --cmd ct)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--per-cmd", type=float, default=1.6,
                    help="每条命令等多久收打印, 秒")
    ap.add_argument("--list", action="store_true", help="列出串口")
    ap.add_argument("--quiet", action="store_true", help="不打印回显, 只返回")
    args = ap.parse_args()

    if args.list:
        import serial.tools.list_ports as lp
        for p in lp.comports():
            print("  %-8s %s" % (p.device, p.description))
        return 0

    if not args.port:
        print("✗ 需要 --port (或用 --list 看有哪些口)"); return 2
    if not args.cmd:
        print("✗ 需要 --cmd"); return 2

    try:
        send(args.port, args.cmd, args.baud, per_cmd=args.per_cmd,
             quiet=args.quiet)
    except Exception as e:                  # noqa: BLE001
        print("✗ 打开/通信失败: %s" % e)
        print("  口在、0 字节输出 = 板子没上电或线没接(不是波特率问题)。")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
