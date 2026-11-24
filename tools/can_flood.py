#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_flood.py — 往总线狂灌帧(默认 0x7E0, 数据全 0 = 显性位最多), 做"电平级"取证。

为什么数据用全 0:
  CAN 的显性位 = 0。帧里 0 越多, 总线被拉低的时间越长。
  对端只要**电气上接在同一对线上**, 它的 CAN_RX 引脚就必然出现长时间低电平 ——
  这个判据跟波特率、验收过滤器、节点号、协议**全都无关**。

⚠ 2026-09-17 教训: 本脚本原来**不检查开卡是否成功**, 也不报告卡的错误状态。
  PCAN 通道会**间歇性**报 "The value of a handle ... is invalid"
  (被 PCAN-View/CanAst 占用、或上一次 bus-off 后句柄没释放)。
  那种情况下脚本直接 traceback 退出, 而对面 `cy` 照样读到"高 N 次 低 0 次" ——
  **看起来像"链路不通", 实际是"根本没在发"**。同一晚已经因此误判过一次。
  现在: 开卡重试 + 发送期间读卡状态 + 结束时报 bus-off/错误计数。

用法:
  python tools/can_flood.py --channel PCAN_USBBUS1 --secs 8 --gap-ms 2
  然后立刻在另一头敲 cy / cn 观察。
"""
import argparse
import sys
import time


def open_bus_with_retry(interface, channel, bitrate, tries=5, verbose=True):
    """PCAN 会间歇性打不开(占用/句柄残留)。重试 + 明确报错, 不再静默失败。"""
    import can
    last = None
    for i in range(tries):
        try:
            return can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        except Exception as e:                     # noqa: BLE001
            last = e
            if verbose:
                print("[flood] 开卡第 %d/%d 次失败: %s" % (i + 1, tries, str(e)[:90]))
            time.sleep(1.0)
    raise RuntimeError("开卡失败 %d 次, 最后一次: %s\n"
                       "  → 关掉 PCAN-View / CanAst.exe, 或把 USB-CAN 拔插一次"
                       % (tries, last))


def read_card_status(bus):
    """尽力读 PCAN 卡的错误状态。返回 (ok_bitmask, 文本)。
    ⚠ PCAN 的状态**不敏感**: 实测把卡设成与对端冲突的波特率照样返回 OK。
      所以它只作参考, 判据永远是"两侧帧计数对拍"。"""
    try:
        st = bus.status() if hasattr(bus, "status") else None
        if st is None:
            return None, "n/a"
        raw = getattr(st, "value", None)
        if raw is None:
            return None, str(st)
        # PCAN_ERROR_OK=0 / BUSOFF=0x01 / BUSWARNING=0x08 / BUSPASSIVE=0x20 ...
        names = []
        if raw == 0:
            names.append("OK(无错)")
        if raw & 0x01:
            names.append("BUS-OFF(无人应答,已离线)")
        if raw & 0x08:
            names.append("BUS-WARNING")
        if raw & 0x20:
            names.append("BUS-PASSIVE(错误太多)")
        return raw, ("%s [0x%08X]" % ("|".join(names) or "?", raw))
    except Exception as e:                          # noqa: BLE001
        return None, "读状态失败: %s" % str(e)[:60]


def main():
    ap = argparse.ArgumentParser(description="CAN 灌帧器(电平级取证用)")
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--id", default="0x7E0")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--gap-ms", type=float, default=2.0)
    ap.add_argument("--data", default="0000000000000000",
                    help="8 字节十六进制, 默认全 0(显性位最多)")
    args = ap.parse_args()

    import can
    can_id = int(args.id, 16) if args.id.lower().startswith("0x") else int(args.id)
    payload = bytes.fromhex(args.data)[:8]
    if len(payload) < 1:
        payload = bytes([0] * 8)

    bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
    _, st_before = read_card_status(bus)
    print("[flood] %s@%d  id=0x%03X data=%s  %gs"
          % (args.channel, args.bitrate, can_id, payload.hex(), args.secs))
    print("[flood] 开卡成功 ✔   发送前卡状态: %s" % st_before)

    t0 = time.monotonic()
    n = 0
    send_err = 0
    last_report = t0
    while time.monotonic() - t0 < args.secs:
        try:
            bus.send(can.Message(arbitration_id=can_id, data=payload,
                                 is_extended_id=False))
            n += 1
        except Exception as e:                    # noqa: BLE001
            send_err += 1
            if send_err <= 3:
                print("[flood] 发送异常: %s" % str(e)[:80])
            if send_err > 20:
                break
        time.sleep(args.gap_ms / 1000.0)
        now = time.monotonic()
        if now - last_report >= 3.0:              # 每 3 秒报一次卡状态
            _, st = read_card_status(bus)
            print("[flood]   t+%.1fs 已发 %d 帧   卡状态: %s"
                  % (now - t0, n, st))
            last_report = now

    dt = time.monotonic() - t0
    _, st_after = read_card_status(bus)
    print("[flood] 入队 %d 帧 / %.1fs (约 %.0f 帧/秒)  发送异常 %d 次"
          % (n, dt, n / max(dt, 1e-6), send_err))
    print("[flood] 结束后卡状态: %s" % st_after)
    if n == 0:
        print("[flood] ✗✗ 一帧都没发出去 —— 这次 cy 的读数**无效**, 别拿它下结论!")
    else:
        print("[flood] ⇒ 现在去看臂侧 cy 的读数(注意: 必须在本脚本**还在跑**的时候读)")
    bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
