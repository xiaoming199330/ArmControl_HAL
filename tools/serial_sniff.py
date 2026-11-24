#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""serial_sniff.py — 串口只读嗅探器（回答"数据到底从哪个口出来"）

为什么需要它
    主从链路排查里最费时间的困惑是："我明明看见主臂在吐 #P，转发脚本却一行没收到"。
    真因几乎总是**端口选错**：
      - 主臂控制台挂在板载 USB 串口（CH340/CP210x），你却在读外接 USB-TTL；
      - 两个 USB-TTL 接反了（读的是从臂口）；
      - 或者是 RX 线压根没接到板子的控制台 TX。
    本工具把候选端口**同时只读监听**几秒，直接给每个口下判决，不用再靠猜。

用法
    py tools/serial_sniff.py --list
    py tools/serial_sniff.py COM11 COM12 --secs 6
    py tools/serial_sniff.py COM8 COM11 COM12 COM22 --secs 6 --lines 5

行为与安全
    - **一个字节都不写**：纯监听，不会碰舵机，不会打断固件状态机。
    - 不启用 RTS/CTS、DSR/DTR 流控（避免拉错握手线）。
      ⚠ 但 Windows 打开串口时内核仍可能瞬时动一下 DTR —— 若你的板子用 DTR 做复位，
        嗅探会顺带触发一次复位，请知悉（本项目的控制台不用 DTR 复位）。
    - 端口被别的程序占用会明确报出来 —— 这本身就是线索：主臂控制台常被串口助手占着，
      那种情况下"脚本读不到"是必然的。

判决判据
    打不开              → 有别的程序在用（串口助手 / 另一个转发脚本 / 上次没退干净）
    0 字节              → 这个口不是你要的控制台（RX 没接 / 接反 / 换错 UART）
    #P 合法行 > 0       → ✅ 这是【主臂位置流】口，直接拿它当 --master
    有 #P 前缀但字段数不对 → 波特率对不上，或半行截断（核对是否 115200）
    有字节但没有 #P     → 可能是【从臂】口，或主臂还没敲 `sp <Hz>`
"""

import argparse
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("缺少 pyserial。安装：\n    py -m pip install pyserial\n")
    sys.exit(2)

FIELDS = 6


def classify(raw: bytes) -> str:
    """与 pc_relay.py 同一套判据：pos / bad / other。"""
    s = raw.strip()
    if not s.startswith(b"#P"):
        return "other"
    if len(s.split()) != FIELDS + 1:
        return "bad"
    return "pos"


def cmd_list() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("没有发现任何串口。检查 USB-TTL 是否插好、驱动是否装了。")
        return 0
    print("可用串口：")
    for p in ports:
        print("  %-10s %s" % (p.device, p.description or ""))
    print("\n把候选口都丢给嗅探器：py tools/serial_sniff.py <口1> <口2> --secs 6")
    return 0


class Sniffer(threading.Thread):
    """只读读线程。"""

    def __init__(self, port: str, baud: int, keep_lines: int):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.keep_lines = keep_lines
        self.ser = None
        self.open_err = None
        self.io_err = None
        self.nbytes = 0
        self.kinds = {"pos": 0, "bad": 0, "other": 0}
        self.samples = []
        self._tail = bytearray()      # 未成行的残留字节
        self._halt = threading.Event()

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1,
                                     rtscts=False, dsrdtr=False)
            return True
        except Exception as e:
            self.open_err = str(e).strip()
            return False

    def run(self):
        while not self._halt.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                self.io_err = str(e).strip()
                return
            if not chunk:
                continue
            self.nbytes += len(chunk)
            self._tail += chunk
            while True:
                i = self._tail.find(b"\n")
                if i < 0:
                    break
                head = bytes(self._tail[:i])
                del self._tail[:i + 1]
                self._consume(head)

    def _consume(self, line: bytes):
        if not line.strip():
            return
        k = classify(line)
        self.kinds[k] += 1
        if len(self.samples) < self.keep_lines:
            self.samples.append((k, line))

    def stop(self):
        # 注意：不能叫 self._stop —— threading.Thread 内部就有 _stop()，
        # 用同名属性会把它覆盖掉，join() 时会 TypeError: 'Event' object is not callable
        self._halt.set()

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


def verdict(s: Sniffer) -> str:
    if s.open_err:
        return ("❌ 打不开：%s\n"
                "     → 有别的程序占着这个口（串口助手 / 另一个转发脚本 / 上次没退干净）。\n"
                "       先把它关掉 —— 也正因如此，这个口不可能是你脚本正在读的那个。" % s.open_err)
    if s.nbytes == 0:
        return ("❗ 一个字节都没收到\n"
                "     → 这个口不是主臂控制台：RX 线没接 / RX-TX 接反 / 插到了另一个 UART。\n"
                "       把本机全部串口都嗅一遍，找「#P 合法行 > 0」的那个。")
    if s.kinds["pos"]:
        return "✅ 这是【主臂位置流】口 —— 直接拿它当 --master"
    if s.kinds["bad"]:
        return ("⚠ 有 #P 前缀但字段数不对 → 多半是波特率不匹配（应为 115200），"
                "或串口缓冲把行截断了")
    return "○ 有输出但没有 #P 位置流 → 可能是【从臂】口，或主臂还没敲 `sp <Hz>`"


def main() -> int:
    ap = argparse.ArgumentParser(
        description="串口只读嗅探器：找出 #P 位置流到底在哪个串口",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("ports", nargs="*", help="要嗅探的串口，如 COM11 COM12（可给多个，同时监听）")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200，与固件控制台一致）")
    ap.add_argument("--secs", type=float, default=5.0, help="每个口监听时长（默认 5 s）")
    ap.add_argument("--lines", type=int, default=6, help="每个口最多回显几行样例（默认 6）")
    ap.add_argument("--hex", action="store_true", help="样例按十六进制打印（怀疑乱码/波特率错时用）")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    args = ap.parse_args()

    if args.list:
        return cmd_list()
    if not args.ports:
        ap.error("至少给一个串口，例如：py tools/serial_sniff.py COM11 COM12 --secs 6")

    print("串口只读嗅探：本工具**不会向任何端口写一个字节**。")
    print("监听 %.1f s，波特率 %d。" % (args.secs, args.baud))
    print("前提：主臂此刻正在吐位置流（敲过 `sp <Hz>`），否则看不到 #P。")
    print("-" * 62)

    snifs = []
    for p in args.ports:
        s = Sniffer(p, args.baud, args.lines)
        s.open()
        snifs.append(s)

    live = [s for s in snifs if s.open_err is None]
    for s in live:
        s.start()

    t0 = time.time()
    try:
        while True:
            left = args.secs - (time.time() - t0)
            if left <= 0:
                break
            sys.stdout.write("\r  监听中… 还剩 %4.1f s " % left)
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    sys.stdout.write("\r" + " " * 30 + "\r")

    for s in live:
        s.stop()
    for s in live:
        s.join(timeout=0.5)
    for s in snifs:
        s.close()

    print()
    for s in snifs:
        print("[%s] @ %d" % (s.port, s.baud))
        if s.open_err:
            print("    " + verdict(s))
            print()
            continue
        print("    字节 %d | 行 %d（#P合法 %d / #P坏行 %d / 其它 %d）"
              % (s.nbytes, sum(s.kinds.values()), s.kinds["pos"], s.kinds["bad"], s.kinds["other"]))
        if s.io_err:
            print("    ⚠ 读取中断：%s" % s.io_err)
        if s.samples:
            print("    样例：")
            for k, raw in s.samples:
                show = raw.hex(" ") if args.hex else raw.decode("utf-8", "replace")
                print("      [%-5s] %s" % (k, show[:78]))
        elif s._tail:
            show = bytes(s._tail).hex(" ") if args.hex else bytes(s._tail).decode("utf-8", "replace")
            print("    只收到未成行的残留：%s" % show[:78])
        print("    " + verdict(s))
        print()

    print("-" * 62)
    print("下一步：把打出「✅ 主臂位置流」的那个口填给 pc_relay.py 的 --master，")
    print("        另一个（有输出但无 #P，或就是接从臂的）填 --slave。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
