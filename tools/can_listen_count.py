#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_listen_count.py — 在 CAN 卡上"只监听并计数"，按 ID 分类打印，最后给总结。

为什么要有它（2026-09-18 现场教训）：
  排查"臂到底有没有在发帧 / 卡到底有没有在收帧"时，最怕的是**工具自己撒谎**：
  `can_rx_witness.py` 曾因漏 `import can` 导致"入队 0 帧"被当成"链路不通"；
  PCAN 的 `bus.send()` 又是**异步入队**，返回成功 ≠ 对端收到。
  ⇒ 所以需要一台**纯被动、无发送、口径单一**的仪器：它只回答一个问题 ——
     "在这 N 秒里，卡在总线上真正收到过哪些帧？"

关键点：PCAN 卡收到合法帧时会**自动 ACK**，所以一个纯监听的本工具同时也在
  给对端"喂 ACK"。若臂侧 `ct` 报 tec≈0，说明就是这里在 ACK ⇒ 臂→卡方向通。

判读（配合臂侧命令）：
  本工具收到 0 帧 + 臂侧 `ct 50` 报 tec 涨到 128+  ⇒ 臂发的帧没到卡（臂→卡 断）
  本工具收到 ~50 帧 + 臂侧 `ct 50` 报 tec≈0        ⇒ ★臂→卡 方向通
  本工具收到 0 帧 + 臂侧 `ct 50` 报 tec≈0          ⇒ 有别人在 ACK（还有第三个节点）

⚠⚠ --passive：仪器会改变被测对象时，必须先关掉自己的影响（2026-09-18 教训）
  默认模式（ACTIVE）会 ACK，而 **ACK 会改臂的错误计数**：臂若正卡在 error-passive
  自锁（tec≥128、状态帧停发），本工具一开卡就等于替它把 TEC 一帧一帧降下来
  （实测 TEC>127 时"成功发一帧 ⇒ TEC-1"，从 134 恢复到 1 Hz 要 7 帧 ≈ 35 s）。
  ⇒ 想回答"**没人在 ACK 时，臂到底还在不在发**"，必须用 `--passive`；
     用默认模式只会得到"臂在发"的**假阳性**（是被你自己救活的）。
  `--passive` 下实测：臂仍每 5 s 发一帧探路帧 ⇒ 探路机制真的在跑（0918G 验收就用它）。

  ⚠ 别用 `can.Bus(..., listen_only=True)`：python-can 的 **PCAN 后端会静默忽略**这个
    kwarg（它认的是 `state=can.BusState.PASSIVE`），于是你以为在旁听、其实一直在 ACK。
    测法：PASSIVE 下 20 s 应收 4 帧(5 s 间隔)；若收到 ~18 帧(1 Hz) 就说明根本没被动，
    是卡在 ACK 把臂拉回了 error-active。

用法:
  python tools/can_listen_count.py --secs 120
  python tools/can_listen_count.py --interface pcan --channel PCAN_USBBUS1 --secs 60
  python tools/can_listen_count.py --secs 20 --passive        # 真旁听, 不改臂的 TEC
"""

import argparse
import sys
import time

DEFAULT_IFACE = "pcan"
DEFAULT_CHANNEL = "PCAN_USBBUS1"

ID_NAMES = {
    0x7E0: "自检帧 (ct / can_flood)",
    0x101: "命令 -> 臂 (node=1)",
    0x181: "臂 -> 回包 (node=1)",
    0x281: "臂 -> 事件/状态帧 (node=1)",
}


def look(iface, channel, bitrate, secs, verbose, passive=False):
    try:
        import can
    except ImportError:
        print("✗ 没装 python-can。用隔离环境：")
        print("  %USERPROFILE%\\.workbuddy\\binaries\\python\\envs\\default\\Scripts\\python.exe")
        return 2

    try:
        if passive:
            # ⚠ 必须用 state=PASSIVE —— python-can 的 PCAN 后端**静默忽略**
            #   listen_only=True 这个 kwarg, 用它会以为在旁听、其实一直在 ACK。
            bus = can.Bus(interface=iface, channel=channel, bitrate=bitrate,
                          state=can.BusState.PASSIVE)
        else:
            bus = can.Bus(interface=iface, channel=channel, bitrate=bitrate)
    except Exception as e:                                    # noqa: BLE001
        print("✗ 开卡失败 (%s:%s @%d): %s" % (iface, channel, bitrate, e))
        print("  排查: 卡插着吗? 通道名对吗(PCAN_USBBUS1)? 被别的程序占了吗?")
        return 3

    print("✔ 开卡 %s:%s @%d bit/s   (python-can %s)" % (iface, channel, bitrate, can.__version__))
    if passive:
        print("  模式: **PASSIVE(真旁听, 不 ACK)** —— 不会改臂的 TEC, 不会把 error-passive 的臂救活。")
    else:
        print("  模式: ACTIVE(会 ACK) —— ACK 会替臂把 TEC 降下来, 别用它判「臂在不在发」。")
        print("  只监听 %d 秒，**不发送任何帧**。PCAN 收到合法帧会自动 ACK。"
              "现在去臂侧敲命令。" % secs)
    print("-" * 70)

    n_by_id = {}
    t0 = time.monotonic()
    last_report = t0
    while time.monotonic() - t0 < secs:
        m = bus.recv(timeout=0.2)
        if m is None:
            # 每 10 秒报一次"还活着、还没收到东西"，避免看起来像卡住了
            if time.monotonic() - last_report >= 10.0:
                last_report = time.monotonic()
                print("  t=%4.0fs  累计 %d 帧 (仍在听)" %
                      (time.monotonic() - t0, sum(n_by_id.values())))
            continue
        last_report = time.monotonic()
        aid = m.arbitration_id
        n_by_id[aid] = n_by_id.get(aid, 0) + 1
        if verbose:
            print("  t=%6.2fs  RX id=0x%03X dlc=%d  %s  %s" % (
                time.monotonic() - t0, aid, m.dlc, bytes(m.data).hex(),
                ID_NAMES.get(aid, "")))

    bus.shutdown()
    total = sum(n_by_id.values())
    print("-" * 70)
    print("监听 %.0f 秒结束，共收到 %d 帧：" % (secs, total))
    if total == 0:
        print("  (一帧都没收到)")
        print()
        if passive:
            print("判读(PASSIVE): 卡侧 0 帧 ⇒ ★臂在那 %.0f s 里真的没往总线上放任何东西。" % secs)
            print("      查臂侧 `cn`: tec≥128 且 hb 没被设成 0 ⇒ 臂把自己闸住了；")
            print("      tec≈0 ⇒ 臂根本没在发(协议层没跑到 / hb=0 / 命令没到)。")
        else:
            print("判读: 卡侧 0 帧。若臂侧 `ct 50` 的 tec 涨到 128+ ⇒ 臂发的帧没到卡；")
            print("      若臂侧 `ct 50` 的 tec≈0 却说发了 ⇒ 有别的节点在 ACK（不对）。")
        return 4
    for aid in sorted(n_by_id):
        print("  id=0x%03X  %6d 帧   %s" % (aid, n_by_id[aid], ID_NAMES.get(aid, "")))
    print()
    if passive:
        rate = total / secs if secs > 0 else 0.0
        print("判读(PASSIVE, 纯旁听): 臂确实在总线上放帧。")
        if 0.15 < rate < 0.25:
            print("      节奏 ~1 帧/5 s ⇒ ★这是 error-passive 下的**探路帧**(0918G 起) ——")
            print("        臂被闸住了但没死，卡一回来(能 ACK)它就会自己爬出来。")
        elif rate >= 0.5:
            print("      节奏 ~1 Hz ⇒ 臂处于 error-active，周期状态帧正常运行。")
    else:
        print("判读: 出现 0x181/0x281 ⇒ ★臂在回包/上报 ⇒ 双向都通。")
        print("      只有 0x7E0 ⇒ 臂在发自检帧但没回协议包(多半是命令没被正确解析)。")
    return 0


def main():
    ap = argparse.ArgumentParser(description="CAN 卡只监听并计数(不发送)")
    ap.add_argument("--interface", default=DEFAULT_IFACE, help="python-can 后端, 默认 pcan")
    ap.add_argument("--channel", default=DEFAULT_CHANNEL, help="通道, 默认 PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--secs", type=float, default=60.0, help="监听时长(秒)")
    ap.add_argument("--verbose", action="store_true", help="逐帧打印")
    ap.add_argument("--passive", action="store_true",
                    help="★真旁听: 不 ACK, 不改臂的 TEC。判「没人在 ACK 时臂还在不在发」必须加它, "
                         "否则默认模式自己的 ACK 会把 error-passive 的臂救活 ⇒ 假阳性")
    a = ap.parse_args()
    sys.exit(look(a.interface, a.channel, a.bitrate, a.secs, a.verbose, a.passive))


if __name__ == "__main__":
    main()
