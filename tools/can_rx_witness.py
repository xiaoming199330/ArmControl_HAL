#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_rx_witness.py — 一条命令回答: **卡发的帧到底进没进 MCU?**

■ 为什么需要它（2026-09-18 实测得到的教训）

`can_pin_hunt.py` 用 500 帧/秒灌帧, 然后把 `cx`/`cw`/`ps` 塞在灌帧窗口里敲。
现场结果: **`h` 在灌帧前是好的(认到 fw-tag), 灌帧一开始三条命令全哑, 灌完再也没回来**。

"灌帧即哑、且灌完不恢复"只有一个自洽的机理:
    CAN RX 中断在中枢里被反复重进(RX FIFO 没被放掉) ⇒ CPU 永久卡在 ISR
    ⇒ 主循环再不执行 ⇒ 控制台永久静默(本工程**没有看门狗**, 不会自恢复)。
而这个机理成立的前提是 —— **帧真的到了 MCU 的 CAN 脚上**。

于是出现一个**二选一、结论完全相反**的局面, 而高速灌帧把它们混在一起分不开:
    帧到了   ⇒ 灌帧把它压死  ⇒ 物理层其实是通的(推翻 09-17 的"静默"结论)
    帧没到   ⇒ 灌帧与它无关  ⇒ 它的死是掉电/别的原因, 物理层仍然不通

■ 本工具怎么把它们拆开

不追求"把卡开到最大", 而是**可控速率 + 同步读臂侧计数器**, 三件事同时做:

    1) 先测**基线稳定性**: `--no-flood` 只轮询 `cn`, 完全不发帧。
       → 若这段就哑了, 说明"板子自己会死", 后面的一切读数都不成立(先查电源)。
    2) 再用**很低的速率**灌帧(默认 5 帧/秒), 同时每个 `--poll` 秒读一次 `cn`,
       把 6 个计数器做成时间序列:
           test rx / peer rx / 合法好帧 / rec  → 涨了 ⇒ 帧确实进了 MCU
           MSR.RX                              → 灌帧期间变 0 ⇒ 线上真的有电平
           tec                                 → 涨了 ⇒ 是臂自己在发且没人应答
    3) 一旦控制台**哑掉就立刻停手**, 并报出"死在什么速率、已经灌了多少帧"
       —— 这个阈值本身就是关键证据(和"到没到"互相印证)。

■ 判读表

    ⚠ **入队 0 帧 = 本次读数无效**, 直接作废重跑 —— 绝不能当成"链路不通"。
      (09-18 现场就是因为灌帧线程漏 import, "入队 0 帧"被误读成"物理层不通"。)
    基线就哑                      ⇒ 板子/电源问题, 与 CAN 无关, 先解决它
    灌帧中计数器不涨 & MSR.RX 恒 1 ⇒ 帧没进 MCU(物理层不通 / 卡在另一个口)
    灌帧中计数器涨  或 MSR.RX 变 0 ⇒ ★帧进了 MCU, 断点不在物理层, 去查过滤器/协议
    涨一点就哑                    ⇒ ★同上 + 固件 RX ISR 有 latch-up(记录阈值, 再谈修)

■ ⚠ "刚起来就静默"还要再分一层: 是**某条命令**杀的, 还是**板子自己**撑不住?

2026-09-18 现场连续三次"板子刚起来就静默", 其中两次紧跟在 `h` 之后、`cn` 之前 ——
光看这个序列会误判成"`cn` 把板子搞死了"。所以必须做**同长度对照**:

    只发 `h`(只读帮助, 不改任何状态) 也活不满  ⇒ 板子自己活不过几秒 ⇒ **供电/硬件**, 与 cn 无关
    只发 `h` 能活满, 而 `cn` 一发就死        ⇒ 才是 `cn` 杀的 ⇒ 查 `cn` 读的哪个寄存器块
                                               (重点: 读**没开时钟**的 CAN2 寄存器 = BusFault)

■ 用法

    # ① 对照甲: 全程只发 h, 看板子自己能活多久(60 秒)
    python tools/can_rx_witness.py --port COM11 --no-flood --poll-cmd h --secs 60

    # ② 对照乙: 同样的时长, 只发 cn —— 两者一比就知道"是不是 cn 杀的"
    python tools/can_rx_witness.py --port COM11 --no-flood --poll-cmd cn --secs 60

    # ③ 温和灌帧: 5 帧/秒, 45 秒, 每 5 秒读一次 cn
    python tools/can_rx_witness.py --port COM11 --gap-ms 200 --secs 45 --poll 5

    # ④ 找出"多少速率会哑": 逐级降 gap(2000→1000→500→200→50→2 ms)
    python tools/can_rx_witness.py --port COM11 --ramp --stage-secs 20

⚠ 安全: 只发 `cn`(纯读寄存器 + 打印, 从不改状态、从不碰力矩)。
  开跑前先用 `h` 认固件指纹 `fw-tag ARM-CAN-<版本>`(复用 `arm_preflight.FW_TAG`,
  现役 `ARM-CAN-0918G`; 直接调用即自动跟随, 不要在这里抄死版本号), 认不到就一条都不发。
"""
import argparse
import os
import re
import sys
import threading
import time

# ⚠ 2026-09-18 现场踩到(代价: 一轮 45 s 的白工 + 一次几乎误判):
#   本文件原来**没有 import can** —— 灌帧线程里 `can.Message(...)` 每轮抛 NameError,
#   被 `except Exception` 静默吞掉, 跑满 21 次后线程自杀 ⇒ 表里"累计入队 0 帧",
#   而结论段照样打印"帧没进 MCU ⇒ 物理层不通, 或卡接在另一个口" ——
#   **把"卡根本没发"伪装成了"链路不通"**(红线 22 "工具会撒谎" 的同款)。
#   ⇒ 两道药: ① 这里补 import(用 try/except 包住, 因为 --no-flood 不需要 python-can);
#             ② 灌帧异常前 3 条必须打出来; ③ 结束时若"入队 0 帧"直接判本次读数无效。
try:
    import can
except Exception:                                   # noqa: BLE001
    can = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# cn 输出里的 6 个关键量。⚠ 一律用**纯 ASCII 锚点**做正则:
# 现场串口助手的编码五花八门(用户贴回来的中文全是 `���`), 靠中文匹配会失手。
RE_TX_RX = re.compile(r"test tx\s*:\s*(\d+)[^\n]*?test rx\s*:\s*(\d+)")
RE_PEER = re.compile(r"peer rx\s*:\s*(\d+)")
RE_OKF = re.compile(r":\s*(\d+)\s*\S*\s*\(CRC")          # 合法好帧: N 帧 (CRC ...
RE_ERR = re.compile(r"tec=(\d+)\s+rec=(\d+)\s+lec=(\d+)")
RE_BUSRX = re.compile(r"MSR\.RX=(\d)")
RE_HW = re.compile(r"hw init\s*:\s*(\w+)")
RE_IF = re.compile(r"(CAN[12])\s+(P[AB]\d+)\(RX\)/(P[AB]\d+)")

FIELDS = ("t", "flood", "hw", "testrx", "peer", "okf", "tec", "rec", "lec", "rx", "note")


def parse_cn(txt):
    """从 `cn` 的回显里抽出关键量。缺失的置 None(而不是 0 —— 0 是有意义的读数)。"""
    d = {k: None for k in ("hw", "testtx", "testrx", "peer", "okf",
                           "tec", "rec", "lec", "rx", "iface")}
    m = RE_HW.search(txt)
    if m:
        d["hw"] = m.group(1)
    m = RE_TX_RX.search(txt)
    if m:
        d["testtx"], d["testrx"] = int(m.group(1)), int(m.group(2))
    m = RE_PEER.search(txt)
    if m:
        d["peer"] = int(m.group(1))
    m = RE_OKF.search(txt)
    if m:
        d["okf"] = int(m.group(1))
    m = RE_ERR.search(txt)
    if m:
        d["tec"], d["rec"], d["lec"] = (int(x) for x in m.groups())
    m = RE_BUSRX.search(txt)
    if m:
        d["rx"] = int(m.group(1))
    m = RE_IF.search(txt)
    if m:
        d["iface"] = "%s %s/%s" % m.groups()
    return d


def ask(ser, cmd, secs):
    """敲一条命令并读 secs 秒。⚠ 只发 \\r(发 \\r\\n 会多一个 \\n 掉进交互模式)。"""
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    t0 = time.time()
    buf = b""
    while time.time() - t0 < secs:
        buf += sread(ser, 600)
    return buf.decode("utf-8", "replace").replace("\r", "")


class Port:
    """包一层串口, 让"口被拔掉/重枚举"不再是致命异常, 而是**有记录的事件**。

    ⚠ 2026-09-18 现场: 观测途中板子/USB 重枚举, pyserial 抛

        serial.serialutil.SerialException:
          GetOverlappedResult failed (PermissionError(13, '拒绝访问。'))

    当时脚本**直接崩掉**, 于是"口是什么时候掉的"这个**关键时间点**、
    以及它前后的读数全部丢失 —— 而那恰恰是判断"供电 vs 固件"最要紧的证据。
    (整板掉电时 CH340 会一起掉 ⇒ COM 号可能变; 而"口还在"≠"还是那块板"。)

    所以这里改成: 出错 → 记一笔(带相对时间) → 每秒重开, 最多 30 次 → 继续跑。
    重开成功也记一笔。最后 `events` 就是一份"掉线/恢复"的时间线。
    """

    def __init__(self, dev, baud=115200, timeout=0.25):
        self.dev, self.baud, self.timeout = dev, baud, timeout
        self.t0 = time.time()
        self.events = []
        self.ser = None
        self._open()

    def _open(self):
        import serial
        self.ser = serial.Serial(self.dev, self.baud, timeout=self.timeout)
        return self.ser

    @property
    def port(self):
        return self.dev

    def read(self, n):
        for attempt in range(1, 31):
            try:
                return self.ser.read(n)
            except Exception as e:                            # noqa: BLE001
                ev = "t=%.1f 口读失败: %s" % (time.time() - self.t0, str(e)[:80])
                self.events.append(ev)
                print("  ⚠ " + ev)
                print("     ⇒ 口被拔掉/重枚举(整板掉电时 CH340 会一起掉, COM 号可能变)。"
                      "1 s 后重开...")
                try:
                    self.ser.close()
                except Exception:                             # noqa: BLE001
                    pass
                time.sleep(1.0)
                try:
                    self._open()
                    self.events.append("t=%.1f 口重开成功(第 %d 次)"
                                       % (time.time() - self.t0, attempt))
                    print("  ✓ 口重开成功(第 %d 次)" % attempt)
                    return b""
                except Exception as e2:                       # noqa: BLE001
                    print("     重开失败: %s" % str(e2)[:80])
        raise RuntimeError("串口 %s 重开 30 次都失败" % self.dev)

    def write(self, b):
        self.ser.write(b)

    def reset_input_buffer(self):
        try:
            self.ser.reset_input_buffer()
        except Exception:                                     # noqa: BLE001
            pass

    def close(self):
        try:
            self.ser.close()
        except Exception:                                     # noqa: BLE001
            pass


def sread(ser, n):
    """薄壳: 真正的重试/重开逻辑在 Port.read 里(也可直接传普通 Serial)。"""
    return ser.read(n)


def fmt(v):
    return "--" if v is None else str(v)


def main():
    ap = argparse.ArgumentParser(description="低速率灌帧 + 臂侧 cn 计数器时间序列对拍")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--id", default="0x7E0")
    ap.add_argument("--gap-ms", type=float, default=200.0, help="默认 200ms = 5 帧/秒")
    ap.add_argument("--secs", type=float, default=45.0, help="整个观测时长")
    ap.add_argument("--poll", type=float, default=5.0, help="每隔几秒读一次")
    ap.add_argument("--poll-cmd", default="cn",
                    help="轮询用哪条命令: cn(默认, 读计数器) / h(只读帮助, 测'不发命令稳不稳')")
    ap.add_argument("--await-boot", action="store_true",
                    help="先等板子上电(只发 h, 不需要指纹), 从上电那一刻开始计时 —— 抓'开机几秒就死'")
    ap.add_argument("--await-secs", type=float, default=180.0, help="--await-boot 最多等多久")
    ap.add_argument("--no-flood", action="store_true", help="完全不发帧, 只测基线稳定性")
    ap.add_argument("--ramp", action="store_true",
                    help="逐级降 gap(2000/1000/500/200/50/2 ms)找'多少速率会哑'")
    ap.add_argument("--stage-secs", type=float, default=20.0, help="--ramp 每级时长")
    ap.add_argument("--force", action="store_true", help="跳过固件指纹前置检查")
    args = ap.parse_args()

    import serial
    from arm_preflight import (check as preflight, assert_safe_cmd,
                               PreflightError, ST_VERIFIED)

    can_id = int(args.id, 0)

    # ── 串口 ─────────────────────────────────────────────
    try:
        ser = Port(args.port, args.baud, 0.25)
    except Exception as e:                                   # noqa: BLE001
        print("✗ 打开臂串口 %s 失败: %s" % (args.port, e))
        print("  (PermissionError/拒绝访问 = 口被别的程序占着, 先关掉那个串口工具)")
        return 1

    if args.await_boot:
        # ── 等上电模式: 从上电那一刻开始计时 ────────────────────────
        # 现场问题是"板子刚起来就静默", 而板子上电前 preflight 必然失败 ⇒ 不能用它做门。
        # 这里**只发 `h`**(所有固件版本都安全: case 'h' → cmdHelp, 从不改状态),
        # 所以就不需要指纹, 也不需要 --force。
        print("  等待板子上电(最多 %.0f s) —— **等待期间只发 `h`**, 不发任何其他命令。" % args.await_secs)
        print("  (上电后按 `--poll-cmd %s` 轮询)" % args.poll_cmd)
        print("  ⇒ 现在请给板子断电、等 2 秒、再上电 / 按 RESET。")
        tw = time.time()
        booted = None
        seen = b""
        while time.time() - tw < args.await_secs:
            seen += sread(ser, 4096)        # 先被动收, 抓开机横幅(Init TIM2... / Init done.)
            ser.write(b"h\r")
            t0 = time.time()
            buf = b""
            while time.time() - t0 < 0.6:
                buf += sread(ser, 4096)
            if buf.strip():
                booted = time.time() - tw
                break
            time.sleep(0.4)
        if booted is None:
            print("  ✗ %.0f s 内没等到任何应答 ⇒ 板子没上电 / 没接在这只口上。" % args.await_secs)
            ser.close()
            return 4
        print("  ✔ 上电后 %.1f s 首次应答(t_wait=%.1f s)。" % (0.0, booted))
        if seen.strip():
            print("  --- 上电前后被动收到的自发打印(应为启动横幅) ---")
            for l in seen.decode("utf-8", "replace").replace("\r", "").split("\n"):
                if l.strip():
                    print("    | " + l.rstrip())
        print("  --- 首次应答内容(前 6 行) ---")
        for l in buf.decode("utf-8", "replace").replace("\r", "").split("\n")[:6]:
            if l.strip():
                print("    | " + l.rstrip())
        fw_state = "await"
        t_boot = time.time()
    else:
        try:
            fw_state = preflight(ser, force=args.force)
        except PreflightError as e:
            print(str(e))
            print("=" * 78)
            ser.close()
            return 3

    # ── 卡(只有要发帧才开) ────────────────────────────────
    bus = None
    stop = threading.Event()
    stat = {"n": 0, "err": 0}
    if not args.no_flood:
        try:
            from can_flood import open_bus_with_retry
            bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
        except Exception as e:                               # noqa: BLE001
            print("✗ 开卡失败: %s" % e)
            ser.close()
            return 2
        print("  卡 %s@%d  id=%s" % (args.channel, args.bitrate, args.id))

    print("  臂 %s@%d   fw=%s" % (args.port, args.baud, fw_state))
    print("=" * 78)

    # ── 基线: 不发帧, 先确认"板子自己会不会死" ────────────────
    # ⚠ 2026-09-18 现场: 这块板出现过三次"刚起来就静默", 其中两次紧跟在 `h` 之后、
    #   `cn` 之前 ⇒ 必须能分开"是 cn 这条命令杀它"还是"它自己活不过几秒"。
    #   办法就是 --poll-cmd h: 全程只发 `h`(只读帮助), 看它活不活得久。
    print("[0] 基线: 读一次 `%s`, 建立初值" % args.poll_cmd)
    base_txt = ask(ser, args.poll_cmd, 3.0)
    base = parse_cn(base_txt)
    base_ok = bool(base_txt.strip())
    if not base_ok:
        if not args.no_flood:
            # 要灌帧就必须先拿到计数器初值, 否则"涨没涨"无从判断。
            print("    ✗ `%s` 一个字都没回 ⇒ **板子现在就是死的**。" % args.poll_cmd)
            print("      先断电重上电/按 RESET, 看到启动打印再跑本脚本。")
            if bus:
                bus.shutdown()
            ser.close()
            return 4
        # --no-flood 时基线失败也有价值: 说明"开机后连第一条命令都回不了",
        # 直接进轮询表, 让时间序列自己把死亡时刻标出来。
        print("    ⚠ `%s` 没回 ⇒ 板子开机后连第一条命令都没答上, 直接进轮询观察。" % args.poll_cmd)
    if base_ok:
        print("    接口=%s hw=%s  初值: test rx=%s peer=%s 好帧=%s tec=%s rec=%s lec=%s RX=%s"
              % (fmt(base["iface"]), fmt(base["hw"]), fmt(base["testrx"]),
                 fmt(base["peer"]), fmt(base["okf"]), fmt(base["tec"]),
                 fmt(base["rec"]), fmt(base["lec"]), fmt(base["rx"])))

    # ── 灌帧速率表 ────────────────────────────────────────
    if args.no_flood:
        stages = [(None, args.secs)]
    elif args.ramp:
        ramp = [2000.0, 1000.0, 500.0, 200.0, 50.0, 2.0]
        stages = [(g, args.stage_secs) for g in ramp if g >= args.gap_ms] or \
                 [(args.gap_ms, args.secs)]
    else:
        stages = [(args.gap_ms, args.secs)]

    def flood(gap_ms):
        shown = 0
        while not stop.is_set():
            try:
                if can is None:
                    raise RuntimeError("python-can 不可用(未安装/导入失败)")
                bus.send(can.Message(arbitration_id=can_id, data=bytes(8),
                                     is_extended_id=False))
                stat["n"] += 1
            except Exception as e:                       # noqa: BLE001
                stat["err"] += 1
                # ⚠ 绝不能静默吞异常 —— 吞掉 NameError 的后果就是"入队 0 帧"被当成"链路不通"。
                if shown < 3:
                    shown += 1
                    print("  ✗ 灌帧异常 #%d: [%s] %s"
                          % (stat["err"], type(e).__name__, str(e)[:130]))
                if stat["err"] > 20:
                    print("  ✗ 灌帧异常已超 20 次, 停止灌帧(发送端有问题, 本次读数将判为无效)")
                    break
            time.sleep(gap_ms / 1000.0)

    print()
    print("  %-6s %-8s %-6s %-7s %-6s %-6s %-5s %-5s %-4s %-4s %s"
          % ("t(s)", "灌帧数", "hw", "test rx", "peer", "好帧", "tec", "rec", "lec", "RX", "备注"))
    print("  " + "-" * 76)

    # ⚠ await 模式下 t=0 定在"上电后首次应答"那一刻, 这样表里的 t 就是**上电后的秒数**,
    #   直接就是"开机后多久死的"。
    t_start = t_boot if args.await_boot else time.time()
    last = dict(base)
    died_at = None
    rows = []

    for gi, (gap, dur) in enumerate(stages):
        th = None
        if gap is not None:
            th = threading.Thread(target=flood, args=(gap,), daemon=True)
            th.start()
            print("  ── 阶段 %d: gap=%.0f ms (约 %.0f 帧/秒), 持续 %.0f s ──"
                  % (gi + 1, gap, 1000.0 / max(gap, 0.001), dur))
        else:
            print("  ── 阶段 %d: **不灌帧**, 持续 %.0f s ──" % (gi + 1, dur))

        t_stage = time.time()
        while time.time() - t_stage < dur:
            time.sleep(min(args.poll, 1.0))
            txt = ask(ser, args.poll_cmd, 2.5)
            d = parse_cn(txt)
            t_rel = time.time() - t_start
            # 死活判据 = **有没有回任何字节**。
            # ⚠ 不能用 `d["hw"] is None` —— `--poll-cmd h` 的回复里本来就没有 hw 字段,
            #   那样会把"只发 h 的活板子"全判成死的。
            if not txt.strip():
                died_at = (t_rel, stat["n"], gap)
                rows.append((t_rel, stat["n"], "--", "--", "--", "--",
                             "--", "--", "--", "--", "★哑了"))
                print("  %-6.1f %-8d %-6s %-7s %-6s %-6s %-5s %-5s %-4s %-4s %s"
                      % (t_rel, stat["n"], "--", "--", "--", "--", "--",
                         "--", "--", "--", "★★ 控制台哑了, 立刻停手"))
                break
            note = ""
            for k, name in (("testrx", "test rx"), ("peer", "peer"),
                            ("okf", "好帧"), ("rec", "rec")):
                if d[k] is not None and last[k] is not None and d[k] > last[k]:
                    note += "+%s " % name
            if d["rx"] == 0 and (last["rx"] != 0):
                note += "RX被拉低 "
            print("  %-6.1f %-8d %-6s %-7s %-6s %-6s %-5s %-5s %-4s %-4s %s"
                  % (t_rel, stat["n"], fmt(d["hw"]), fmt(d["testrx"]), fmt(d["peer"]),
                     fmt(d["okf"]), fmt(d["tec"]), fmt(d["rec"]), fmt(d["lec"]),
                     fmt(d["rx"]), note))
            rows.append((t_rel, stat["n"], d["hw"], d["testrx"], d["peer"], d["okf"],
                         d["tec"], d["rec"], d["lec"], d["rx"], note))
            last = d

        stop.set()
        if th:
            th.join(timeout=1.5)
        if died_at:
            break
        if gap is not None:
            stop = threading.Event()          # 下一阶段重新起
            print("  ── 阶段 %d 结束: 累计入队 %d 帧, 控制台仍活 ──" % (gi + 1, stat["n"]))

    # ── 收尾 ─────────────────────────────────────────────
    print()
    post = None
    if not died_at and args.poll_cmd == "cn":
        post = parse_cn(ask(ser, "cn", 3.0))
    print("=" * 78)
    if bus:
        bus.shutdown()
    evs = list(getattr(ser, "events", []) or [])
    ser.close()

    # 口掉线/重开的时间线 —— 本身就指向"供电/连接", 必须在结论之前摆出来。
    if evs:
        print("  ⚠ 观测期间串口掉线/重开事件 %d 次(口被拔掉/重枚举 ⇒ 整板掉电连 CH340 一起掉):"
              % len(evs))
        for e in evs:
            print("      " + e)
        print("  ⇒ 这**本身就是证据**: 指向供电/连接, 不是脚本问题。")
        print()

    if died_at:
        t_rel, n, gap = died_at
        print("结论: **板子在 t=%.0fs 哑掉** —— 当时已灌 %d 帧%s。"
              % (t_rel, n, ("(gap=%.0f ms ≈ %.0f 帧/秒)" % (gap, 1000.0 / max(gap, 0.001)))
                 if gap else "(不灌帧阶段)"))
        if gap is not None and n == 0:
            print()
            print("=" * 78)
            print("✗ **本次读数无效**: 卡一帧都没发出去(入队 0 帧, 异常 %d 次)。" % stat["err"])
            print("  ⇒ 板子哑了这件事**不能归因于灌帧** —— 线上一帧都没有。")
            print("  先修发送端(端口/卡占用/USB 口), 再重跑。")
            return 2
        if gap is None:
            print("  ⇒ **完全不发帧也死了** ⇒ 与 CAN 无关。")
            if args.poll_cmd == "h":
                print("    全程**只发 `h`**(只读帮助, 不改任何状态) 都能死 ⇒ 不是哪条命令杀它,")
                print("    是**板子/供电自己撑不住**。先查 12V/5V、共地、插头松动、USB 线接触,")
                print("    以及舵机保持力矩时的电流是否把电源拉垮(断电后能撑一会儿 = 典型欠压)。")
                print("    ⚠ 在解决这个之前, CAN 侧的结论全都不成立。")
            else:
                print("    但**还不能**据此说'`cn` 是凶手' —— 要跟 `--poll-cmd h` 的同一时长对照:")
                print("      python tools/can_rx_witness.py --port %s --no-flood --poll-cmd h --secs %.0f"
                      % (args.port, args.secs))
                print("    只发 h 也活不满 ⇒ 板子自己活不过几秒(供电问题), 与 cn 无关;")
                print("    只发 h 能活满   ⇒ 才是 `cn` 这条命令把它杀了, 去查 cn 读的哪个寄存器块。")
        else:
            print("  ⇒ 灌帧把它压死了 ⇒ **帧确实进了 MCU**(否则它根本感知不到灌帧)。")
            print("    下一步: 固件侧查 CAN RX 中断有没有正确放掉 RX FIFO(HAL_CAN_ReleaseRxFifo),")
            print("            ISR 里不要做重活; 有条件的话加独立看门狗兜底。")
        print("  ⚠ 板子现在需要断电重上电才能继续。")
        return 5

    print("结论: 全程活到结束(累计入队 %d 帧, 异常 %d 次)。" % (stat["n"], stat["err"]))
    if not args.no_flood and stat["n"] == 0:
        print()
        print("=" * 78)
        print("✗ **本次读数无效**: 卡一帧都没发出去(入队 0 帧, 异常 %d 次)。" % stat["err"])
        print("  ⚠ 此时**绝不能**判成'帧没进 MCU / 物理层不通' ——")
        print("     那是把'卡没发'伪装成'链路不通'(红线 22 工具会撒谎)。")
        print("  先修发送端: python-can 是否可用 / 卡是否被 PCAN-View·CanAst 占用 / 换 USB 口,")
        print("  再重跑本脚本。判'链路通不通'的前提是**卡确实往线上发了帧**。")
        return 2
    if args.poll_cmd != "cn":
        print("  (本次只发 `%s`, 没读计数器 —— 它证明的是'这种流量不致命'。)"
              % args.poll_cmd)
        return 0
    if post is None:
        return 0

    moved = []
    for k, name in (("testrx", "test rx"), ("peer", "peer rx"),
                    ("okf", "合法好帧"), ("rec", "rec")):
        if post[k] is not None and base[k] is not None and post[k] > base[k]:
            moved.append("%s %d→%d" % (name, base[k], post[k]))
    rx_low = post["rx"] == 0
    print("  基线 → 收尾: %s" % ("; ".join(moved) if moved else "**四个收帧计数器全都没动**"))
    print("  MSR.RX 收尾 = %s" % fmt(post["rx"]))
    if moved or rx_low:
        print("  ★★ 有收帧证据(计数器增长" + ("/RX 被拉低" if rx_low else "") +
              ") ⇒ **帧真的进了 MCU** ⇒ 断点不在物理层。")
    else:
        print("  ⇒ 帧**没进 MCU**: 计数器不动、RX 也没被拉低。")
        print("     与 09-17 的结论一致 —— 物理层不通, 或卡接在板子的另一个 CAN 口上。")
        print("     下一步: `can_pin_hunt.py --port %s` 看 ps 的 ★ 落在哪只脚;" % args.port)
        print("             若 ★ 在 PB12/PB13 ⇒ 加 --cx 2 再验(运行时可切, 不用重烧)。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
