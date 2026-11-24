#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_path_probe.py — 「只看一根线上的电平」的 CAN 通路判定器

为什么还要有这个东西（2026-09-17 晚的教训）
-------------------------------------------
前面所有判据都带"帧"的语义: 帧计数要对拍、ACK 要看 TEC、状态帧要看 0x101。
这些判据**全都被波特率、验收过滤器、节点号污染** —— 一旦不通, 你分不清是
"线没接"还是"帧被丢了"。而且实测里 PCAN 卡自己的状态读数完全不敏感
(故意把它设成 250k 与臂的 500k 冲突, 它照样报 OK)。

本工具换一个维度取证: **电平**。
  · CAN 总线空闲 = 隐性 = CAN_H≈CAN_L
  · 任何一个节点在发 = 显性位 = CAN_H 被拉高 / CAN_L 被拉低
  · MCU 能直接读到 CAN_RX 引脚的电平(bxCAN 的 MSR.RX, 硬件内部采样)
电平**不认波特率、不认过滤器、不认节点号** —— 只要对端的差分信号真的
沿着线走过来, CAN_RX 就会跳。这是把"物理层"单独拎出来考的唯一干净办法。

三个实验(全部零改动, 不用重烧固件):
  E1 臂自己发→总线:  cz        (发 1 帧, 期间 µs 级采样 MSR.RX)
                               lo>0 ⇒ 模块把 MCU 的 TXD 送上了总线
  E2 控制组:           cy <win> (谁都不发)
                               应当"恒定隐性" —— 证明 cy 本身可信(不是永远报有活动)
  E3 卡在发:           cy <win> (同时 PCAN 全力发 0x7E0)

判读(见末尾自动输出)
  E3 出现 低/跳变 ⇒ 卡的差分信号**确实到了臂的收发器** ⇒ 物理层 OK
                    ⇒ 不通的原因在帧层(过滤器/节点号/计数器/协议), 与接线无关
  E3 恒定隐性     ⇒ 卡的信号**根本没到臂** ⇒ 物理层确凿不通
                    (此时可以理直气壮地否掉"物理层没问题"这个前提)

用法:
  python tools/can_path_probe.py --port COM11 --channel PCAN_USBBUS1
  python tools/can_path_probe.py --port COM11 --interface pcan --win 2000 --send-sec 4
"""
import argparse
import os
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_preflight import assert_safe_cmd, PreflightError, ST_VERIFIED  # noqa: E402

# 固件指纹状态; 只有 ST_VERIFIED 时 ask() 才放行全部命令(见 arm_preflight)
_FW_STATE = None

ID_TEST = 0x7E0
MAGIC0, MAGIC1 = 0xC5, 0x3A

RE_CZ = re.compile(r"入邮箱\s*=\s*(\w+)\s+高\s*(\d+)\s+低\s*(\d+)\s+跳变\s*(\d+)"
                   r"\s+tec=(\d+)\s+lec=(\d+)")
RE_CY = re.compile(r"高\s*(\d+)\s*次\s*低\s*(\d+)\s*次\s*跳变\s*(\d+)\s*次")


def ask(ser, cmd, secs):
    """发一条命令, 阻塞收满 secs 秒。"""
    assert_safe_cmd(cmd, _FW_STATE)       # 指纹未认到时拦住危险首字母(见 arm_preflight)
    out = bytearray()
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode("ascii"))   # ⚠ 只发 \r
    end = time.monotonic() + secs
    while time.monotonic() < end:
        out += ser.read(4096)
    return out.decode("utf-8", "replace")


def parse_cz(text):
    m = RE_CZ.search(text)
    if not m:
        return None
    return {"tx": m.group(1), "hi": int(m.group(2)), "lo": int(m.group(3)),
            "edges": int(m.group(4)), "tec": int(m.group(5)), "lec": int(m.group(6))}


def parse_cy(text):
    m = RE_CY.search(text)
    if not m:
        return None
    return {"hi": int(m.group(1)), "lo": int(m.group(2)), "edges": int(m.group(3))}


class Burst:
    """后台持续发帧, 让总线在整个观测窗口里都处于'有帧在跑'。"""

    def __init__(self, bus, can_mod, gap_ms=2.0):
        self.bus, self.can, self.gap = bus, can_mod, gap_ms / 1000.0
        self.sent = 0
        self.err = None
        self._stop = threading.Event()
        self._th = None

    def start(self):
        def run():
            i = 0
            while not self._stop.is_set():
                data = bytes([MAGIC0, MAGIC1, i & 0xFF, (i >> 8) & 0xFF, 0, 0, 0, 0])
                try:
                    self.bus.send(self.can.Message(arbitration_id=ID_TEST, data=data,
                                                   is_extended_id=False))
                    self.sent += 1
                except Exception as e:          # noqa: BLE001
                    self.err = str(e)
                    break
                i += 1
                time.sleep(self.gap)
        self._th = threading.Thread(target=run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        if self._th:
            self._th.join(timeout=1.5)


def main():
    ap = argparse.ArgumentParser(description="CAN 通路电平判定器(与波特率/过滤器无关)")
    ap.add_argument("--port", required=True, help="臂控制台串口, 如 COM11")
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--win", type=int, default=2000, help="cy 观测窗口 ms")
    ap.add_argument("--send-sec", type=float, default=None,
                    help="卡持续发送时长(默认 = cy 窗口 + 1.2s)")
    ap.add_argument("--gap-ms", type=float, default=2.0, help="卡发帧间隔 ms")
    ap.add_argument("--skip-cz", action="store_true")
    ap.add_argument("--cx", type=int, default=0,
                    help="开测前先切 CAN 接口: 1=CAN1(PB8/PB9) 2=CAN2(PB12/PB13) 0=不动")
    ap.add_argument("--force", action="store_true",
                    help="跳过固件指纹前置检查(危险: 老固件上敲 cx 会松全部力矩)")
    args = ap.parse_args()
    send_sec = args.send_sec if args.send_sec else args.win / 1000.0 + 1.2

    import serial
    import can
    from arm_preflight import check as preflight, assert_safe_cmd, PreflightError

    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except Exception as e:                      # noqa: BLE001
        print("✗ 打开臂控制台失败: %s" % e)
        print("  (控制台要按两次回车才回显; 若一直无输出, 查臂是否上电 / 串口是否被占用)")
        return 2
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"\r")                            # ⚠ 只发 \r, 别发 \r\n
    time.sleep(0.8)
    if not ser.read(2000):
        print("⚠ 控制台敲回车没有任何回显 —— 继续, 但结果可能不可信")

    # ── 固件指纹前置检查 (2026-09-18) ──────────────────────────────
    # 本脚本要敲 cx / cz / cy, 全是 c 开头。在老固件上它们不会报"未知命令",
    # 而是掉进首字母兜底 `case 'c'` → **校准向导 → 松全部力矩**。
    global _FW_STATE
    try:
        fw_state = preflight(ser, force=args.force)
    except PreflightError as e:                 # noqa: BLE001
        print(str(e))
        print("=" * 68)
        ser.close()
        return 3
    _FW_STATE = fw_state

    try:
        from can_host import open_bus_with_retry
        bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
    except Exception as e:                      # noqa: BLE001
        print("✗ 打开 CAN 卡失败: %s" % e)
        print("  → python tools/can_host.py --probe")
        ser.close(); return 2

    print("=" * 72)
    print("  can_path_probe — 只看电平, 与波特率/过滤器/节点号无关")
    print("  %s(臂)  <-->  %s:%s @%d" % (args.port, args.interface, args.channel,
                                         args.bitrate))
    print("=" * 72)

    # ---------------- E0: 确认/切换 CAN 接口 (2026-09-18) ----------------
    # 下面所有结论都是"针对**当前这一组 CAN 脚**"的, 所以先把接口问清楚。
    # --cx 1|2 可以直接切到另一颗控制器(F407: CAN1=PB8/PB9, CAN2=PB12/PB13),
    # 不用重烧固件 —— 这是验"卡是不是插在板子另一个 CAN 口上"的唯一办法。
    if args.cx:
        print("\n[E0] cx %d — 把臂的 CAN 接口切到 CAN%d" % (args.cx, args.cx))
        for l in ask(ser, "cx %d" % args.cx, 2.5):
            print("     " + l)
    for l in ask(ser, "cx", 1.5):
        print("  " + l)

    # ---------------- E1: 臂自己的发→总线通路 ----------------
    cz = None
    if not args.skip_cz:
        print("\n[E1] cz — 臂发 1 帧, 期间 µs 级采样 MSR.RX")
        cz = parse_cz(ask(ser, "cz", 2.0))
        if cz is None:
            print("     ✗ 没解析到 cz 输出 (固件里没有这条命令? 还是控制台掉进单键模式?)")
        else:
            print("     入邮箱=%s  高=%d 低=%d 跳变=%d  tec=%d lec=%d"
                  % (cz["tx"], cz["hi"], cz["lo"], cz["edges"], cz["tec"], cz["lec"]))
            if cz["lo"] > 0:
                print("     ✅ 发送期间总线出现显性 ⇒ 模块确实把 MCU 的 TXD 送上了总线")
                print("        (⇒ 模块供电、TXD 接线、电平跳线都不是嫌疑了)")
            else:
                print("     ❌ 全程隐性: MCU 拉了 CAN1_TX, 但总线上什么都没发生")
                print("        ⇒ 模块没在驱动差分对。查: 模块供电 / TXD 是否真接到 CAN1_TX")
                print("          / TXD 那根与 RXD 是否接反(cp 分不出这两种!)")
        # 让 TEC 回到干净状态再往下走
        ask(ser, "ct 40", 2.5)

    # ---------------- E2: 控制组(谁都不发) ----------------
    print("\n[E2] 控制组: 谁都不发, cy %d ms" % args.win)
    print("     (这一步是为了证明 cy 本身可信 —— 它应该在总线空闲时报'恒定隐性')")
    cy_idle = parse_cy(ask(ser, "cy %d" % args.win, args.win / 1000.0 + 1.5))
    if cy_idle is None:
        print("     ✗ 没解析到 cy 输出")
    else:
        print("     高=%d 低=%d 跳变=%d" % (cy_idle["hi"], cy_idle["lo"], cy_idle["edges"]))
        if cy_idle["lo"] == 0:
            print("     ✅ 恒定隐性 —— 符合预期 ⇒ cy 可信(不是永远报有活动)")
            if cy_idle["hi"] == 0:
                print("        ⚠ 高/低都是 0? 说明窗口根本没跑完, 这条不作数")
        else:
            print("     ⚠ 没人发却有显性 ⇒ 要么臂自己在发(主臂 mf 1 / 转发开着),")
            print("        要么总线上有别的节点。这份 E2 基线不干净, E3 要打折看。")

    # ---------------- E3: 卡在发的时候臂看到什么 ----------------
    print("\n[E3] 卡全力发帧(%s 间隔 %gms, 共 %.1fs), 同时臂 cy %d ms"
          % ("0x%03X" % ID_TEST, args.gap_ms, send_sec, args.win))
    burst = Burst(bus, can, gap_ms=args.gap_ms)
    burst.start()
    time.sleep(0.5)
    cy_busy = parse_cy(ask(ser, "cy %d" % args.win, args.win / 1000.0 + 1.5))
    burst.stop()
    print("     卡已入队 %d 帧%s" % (burst.sent, ("  发送报错: " + burst.err) if burst.err else ""))
    if cy_busy is None:
        print("     ✗ 没解析到 cy 输出")
    else:
        print("     高=%d 低=%d 跳变=%d" % (cy_busy["hi"], cy_busy["lo"], cy_busy["edges"]))

    # ---------------- 判读 ----------------
    print("\n" + "-" * 72)
    if cy_busy is None or cy_idle is None:
        print("  证据不足(命令没回显) ⇒ 先解决控制台回显问题")
    elif cy_busy["edges"] > 0 or cy_busy["lo"] > 0:
        print("  ✅ 卡在发的时候, 臂的 CAN_RX 引脚**动了** (低=%d 跳变=%d)"
              % (cy_busy["lo"], cy_busy["edges"]))
        print("     ⇒ 卡的差分信号**确实走到了臂的收发器** ⇒ **物理层是通的**")
        print("     ⇒ 那不通就只剩帧层: 波特率 / 验收过滤器 / 节点号 / 计数点")
        print("       (下一步: 跑 tools/can_link_pair.py, 看两侧帧计数对拍)")
    else:
        print("  ❌ 卡发了 %d 帧, 臂的 CAN_RX 引脚**一动不动**(恒定隐性)"
              % burst.sent)
        print("     ⇒ 卡的差分信号**没有到达臂的收发器** ⇒ 物理层确实不通")
        print("       注意: 这个判据与波特率无关(电平不认波特率), 所以排除了'速率不匹配'")
        print("       也排除了'过滤器/节点号' —— 那些都还在帧层, 管不到引脚电平")
        print("     按可能性查:")
        print("       ⭐ 卡的 CAN_H/CAN_L 与臂收发器的 CANH/CANL 不是同一对线")
        print("          (最常见: 接到收发器模块的 TXD/RXD 那侧引脚上了)")
        print("       ⭐ 卡那端 H/L 接反(自制 DB9 转接把 2/7 搞反) —— 反了电平不会跳")
        print("       ⭐ 没共地: 量卡 GND ↔ 臂 GND 必须 ≈0V")
        print("       ⭐ 收发器模块的 CANH/CANL 那两根是否**真接到端子上**(虚接/只插了一半)")
        print("       · 终端电阻缺失通常不会让电平完全不跳, 所以它不是本症状的主因")
    print("  ⚠ 别再拿'PCAN 卡报不报总线错'当判据 —— 实测它不敏感。")
    bus.shutdown(); ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
