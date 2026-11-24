#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_link_pair.py — 「两侧同时活着」的 CAN 物理层判定器（一条命令出结论）

为什么必须单独有这个东西（2026-09-17 现场踩到的坑）:
  CAN 的应答(ACK)是**帧级**动作 —— 只有在对面那块卡**已经被打开(初始化)**时,
  它的收发器才接在总线上、才会应答。所以:
    · 卡的程序没在跑 → 臂发帧必然 `lec=3`(无人应答)。这是正常现象, 跟接线对错**无关**。
    · 只有让两边**同时活着**, lec/tec 与帧计数才有判据意义。
  而这个"同时"人手很难对准(取臂侧计数、跑 PC 脚本、再取计数, 中间极易错拍)。
  本脚本把它们绑在一起, 顺序严格可控。

另一条已经证伪的判据（别再用了）:
  PCAN 卡自己的 `CAN_GetStatus` 在"必然没有对端"的条件下(位速率故意设成 250k
  而臂是 500k)照样返回 0x00000000 正常 ⇒ **这个读数不敏感, 不能当判据**。
  同理 `bus.send()` 不抛异常只代表"已入卡队列", 不代表有人应答。
  ⇒ 唯一可信的是**两侧帧计数对拍**, 用不过协议层的 0x7E0 自检帧。

用法:
  python tools/can_link_pair.py --port COM11 --channel PCAN_USBBUS1 --bitrate 500000
  python tools/can_link_pair.py --port COM11 --interface pcan --n 200 --arm-n 200
"""
import argparse
import re
import sys
import threading
import time

ID_TEST = 0x7E0
MAGIC0, MAGIC1 = 0xC5, 0x3A

# 臂侧 cn 打印里的字段
RE_TX_RX = re.compile(r"test tx\s*:\s*(\d+)\s*帧入邮箱\s*test rx\s*:\s*(\d+)\s*帧收到")
RE_ERR = re.compile(r"err\s*:\s*tec=(\d+)\s*rec=(\d+)\s*lec=(\d+)")
RE_BITRATE = re.compile(r"bitrate\s*:\s*(\d+)\s*kbit/s")
RE_NODE = re.compile(r"node=(\d+)")
RE_BUSPIN = re.compile(r"MSR\.RX=(\d)")
RE_HW = re.compile(r"hw init\s*:\s*(\w+)")


def parse_cn(text):
    d = {"tx": None, "rx": None, "tec": None, "rec": None, "lec": None}
    m = RE_TX_RX.search(text)
    if m:
        d["tx"], d["rx"] = int(m.group(1)), int(m.group(2))
    m = RE_ERR.search(text)
    if m:
        d["tec"], d["rec"], d["lec"] = (int(x) for x in m.groups())
    m = RE_BITRATE.search(text)
    if m:
        d["kbps"] = int(m.group(1))
    m = RE_NODE.search(text)
    if m:
        d["node"] = int(m.group(1))
    m = RE_BUSPIN.search(text)
    if m:
        d["buspin"] = int(m.group(1))
    m = RE_HW.search(text)
    if m:
        d["hw"] = m.group(1)
    return d


def main():
    ap = argparse.ArgumentParser(description="CAN 物理层判定器(两侧同时活着)")
    ap.add_argument("--port", required=True, help="臂控制台串口, 如 COM11")
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--n", type=int, default=200, help="PC 侧发多少帧(0x7E0)")
    ap.add_argument("--arm-n", type=int, default=200, help="臂侧发多少帧")
    ap.add_argument("--gap-ms", type=float, default=5.0)
    ap.add_argument("--listen", type=float, default=4.0, help="反向监听窗口, 秒")
    args = ap.parse_args()

    import serial
    import can

    # ---------- 1. 两边同时打开 ----------
    try:
        ser = serial.Serial(args.port, 115200, timeout=0.05)
    except Exception as e:                      # noqa: BLE001
        print("✗ 打开臂控制台失败: %s" % e); return 2
    time.sleep(0.3); ser.reset_input_buffer()
    try:
        from can_host import open_bus_with_retry
        bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
    except Exception as e:                      # noqa: BLE001
        print("✗ 打开 CAN 卡失败: %s" % e)
        print("  → 跑 python tools/can_host.py --probe 看 channel_condition")
        print("    (2 = 已被占用: 关掉 PCAN-View / CanAst.exe, 或把 PCAN-USB 拔插一次)")
        ser.close(); return 2

    def ask(cmd, secs):
        out = bytearray()
        ser.reset_input_buffer()
        ser.write((cmd + "\r").encode("ascii"))
        end = time.monotonic() + secs
        while time.monotonic() < end:
            out += ser.read(4096)
        return out.decode("utf-8", "replace")

    print("=" * 70)
    print("  can_link_pair — 两侧同时活着, 判 CAN 物理层")
    print("  %s(臂控制台)  <-->  %s:%s @%d" %
          (args.port, args.interface, args.channel, args.bitrate))
    print("=" * 70)
    print("✓ 两边都打开了 (这一步本身就把'对面不活着'排除掉了)")

    # ---------- 2. 基线 ----------
    base = parse_cn(ask("cn", 2.0))
    print("\n[臂侧 cn 基线] %s" % base)
    if base.get("hw") != "ok":
        print("  ⚠ 臂侧 hw init != ok ⇒ 先让臂的 CAN 起来再谈链路")
    if base.get("kbps") and base["kbps"] * 1000 != args.bitrate:
        print("  ⚠⚠ 位速率不一致! 臂 %d kbit/s, 卡 %d kbit/s ⇒ 这就是不通的原因"
              % (base["kbps"], args.bitrate // 1000))
        bus.shutdown(); ser.close(); return 1
    rx0 = base["rx"] or 0
    tx0 = base["tx"] or 0

    # ---------- 2b. 应答(TEC)增量探测 —— 整套里最灵敏的一条 ----------
    # 它不依赖对面"收不收得到", 只依赖对面"应不应答"。应答是帧级动作,
    # 与对面程序的过滤器无关 ⇒ 只要对面的卡是**打开着的**且位速率一致,
    # 臂发 10 帧的 TEC 就应该**降**(成功发送每帧 -1), 而不是**涨**(无人应答每帧 +8)。
    # 注意: 这块板 AutoBusOff 开着 ⇒ TEC 涨到 >255 会 bus-off 并自动清零,
    #       所以要看"涨得快不快", 不能只看最终值。
    print("\n[0/2] 应答探测: 让臂发 10 帧, 看它自己的 TEC 往哪边走 ...")
    ask("ct 10", 2.5)
    ack_st = parse_cn(ask("cn", 2.0))
    t0, t1 = base.get("tec") or 0, ack_st.get("tec") or 0
    l0, l1 = base.get("lec"), ack_st.get("lec")
    print("    tec: %s → %s (Δ=%+d)   lec: %s → %s   自检帧已收=%s"
          % (t0, t1, t1 - t0, l0, l1, ack_st.get("rx")))
    # ⚠ 这一步**只在基线干净(tec=0 且 lec=0)时**才有判据力。
    #   基线不干净时: AutoBusOff 开着 ⇒ TEC 撞到 255 就 bus-off 并自动清零,
    #   "发完反而变小"完全可能是 bus-off 清零, 不能反推成"被应答了"。
    # 判据(利用"每帧一次 ACK 错 = TEC +8"这个精确特征, 基线脏也能用):
    #   ✅ 被应答  : lec 回到 0(成功收发会把 LEC 清 0) 且 TEC 不涨(每帧 -1 或不动作)
    #   ❌ 无人应答: lec=3 且 TEC 恰好涨了 ≈8×N —— 这是"N 帧各错一次"的指纹
    #   ⚠ 其余情况(尤其 TEC 撞顶被 AutoBusOff 清零)一律不下结论
    n10 = 10
    delta = t1 - t0
    if l1 == 0 and delta <= 0:
        ack_ok = True
    elif l1 == 3 and delta >= n10 * 8 * 0.8:
        ack_ok = False
    else:
        ack_ok = None
    if ack_ok is True:
        print("    ✅ 帧**被应答了**(LEC 回到 0 且 TEC 没涨)")
        print("       ⇒ 卡的收发器确实挂在这对线上 ⇒ 问题在卡侧接收/过滤, 不是接线")
    elif ack_ok is False:
        print("    ❌ 帧**无人应答**: TEC 恰好涨了 %+d = %d 帧 × 8 ⇒ 每帧一次 ACK 错" % (delta, n10))
        print("       ⇒ 卡虽然开着、速率也一致, 它的信号**没到臂的收发器** ⇒ 物理层不通")
    else:
        print("    ⚠ 指纹不明确(tec %d→%d, lec %s→%s) ⇒ 本步**不作判据**"
              % (t0, t1, l0, l1))
        print("      (AutoBusOff 会把撞顶的 TEC 清零, '发完变小'不等于'被应答')")
        print("      要让它干净: 断电重启臂板, 然后单独重跑本命令。")
    # 收尾: 再发一小把, 让 TEC 要么被 ACK 抹平、要么撞到 bus-off 被 AutoBusOff 清零,
    # 免得后面几步都带着 error-passive 的状态观察(两种结局都会回到 0)。
    ask("ct 40", 2.5)

    # ---------- 3. 卡 -> 臂 ----------
    print("\n[1/2] 卡 → 臂: 发 %d 帧 0x%03X ..." % (args.n, ID_TEST))
    sent = 0
    for i in range(args.n):
        data = bytes([MAGIC0, MAGIC1, i & 0xFF, (i >> 8) & 0xFF, 0, 0, 0, 0])
        try:
            bus.send(can.Message(arbitration_id=ID_TEST, data=data,
                                 is_extended_id=False))
            sent += 1
        except Exception as e:                  # noqa: BLE001
            print("   第 %d 帧失败: %s" % (i + 1, e)); break
        time.sleep(args.gap_ms / 1000.0)
    time.sleep(0.3)
    after = parse_cn(ask("cn", 2.0))
    rx1 = after["rx"] or 0
    d_fwd = rx1 - rx0
    print("    卡已入队 %d 帧; 臂侧 '自检帧已收': %d → %d  (Δ=%+d)"
          % (sent, rx0, rx1, d_fwd))

    # ---------- 4. 臂 -> 卡 ----------
    print("\n[2/2] 臂 → 卡: 让臂发 %d 帧, 同时卡在听 ..." % args.arm_n)
    got = {"n": 0, "ids": {}}
    stop = threading.Event()

    def listener():
        while not stop.is_set():
            m = bus.recv(timeout=0.05)
            if m is None:
                continue
            got["n"] += 1
            got["ids"][m.arbitration_id] = got["ids"].get(m.arbitration_id, 0) + 1

    th = threading.Thread(target=listener, daemon=True)
    th.start()
    ser.write(("ct %d\r" % args.arm_n).encode("ascii"))
    # 命令回显里也带 "自检帧 已收", 一并抓下来
    out = bytearray()
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        out += ser.read(4096)
    stop.set(); th.join(timeout=1.0)
    txt = out.decode("utf-8", "replace")
    print("    卡侧收到 %d 帧 %s" % (got["n"], got["ids"] or ""))
    # ct 自己的那行(格式与 cn 不同, 单独抓一份)
    m = re.search(r"入发送邮箱\s*(\d+)/(\d+)\s*帧\s*tec=(\d+)\s*rec=(\d+)\s*lec=(\d+)", txt)
    if m:
        print("    臂侧 ct: 入邮箱 %s/%s 帧   tec=%s rec=%s lec=%s" % m.groups())
    # 再用 cn 读一次权威的计数与错误状态(读 cn 本身不发帧)
    st2 = parse_cn(ask("cn", 2.0))
    print("    [臂侧 cn 复读] %s" % st2)
    d_bwd = (st2["tx"] or 0) - (after["tx"] or 0)
    print("    臂侧 '自检帧已发'(入邮箱) 增量 = %+d" % d_bwd)

    # ---------- 5. 判读 ----------
    print("\n" + "-" * 70)
    fwd_ok = d_fwd >= max(1, sent * 0.8)
    bwd_ok = got["n"] >= max(1, int(args.arm_n * 0.8))
    print("  卡→臂: %s  (臂侧收到 %d / 发出 %d)"
          % ("通 ✅" if fwd_ok else "不通 ❌", d_fwd, sent))
    print("  臂→卡: %s  (卡侧收到 %d / 臂发出约 %d)"
          % ("通 ✅" if bwd_ok else "不通 ❌", got["n"], args.arm_n))
    if ack_ok is None:
        print("  应答(TEC 探测): 基线不干净, 未作判据")
    else:
        print("  应答(TEC 探测): %s" % ("对面在应答 ✅" if ack_ok else "对面不应答 ❌"))
    if ack_ok is False:
        print("  ⭐ 结论: 臂发出的帧**没有任何节点应答**, 而卡此刻是活着的、速率也一致")
        print("     ⇒ 卡与臂**没有真正接在同一对差分线上**。查:")
        print("       H/L 是否接到收发器的 CANH/CANL(不是接到 MCU 的 CAN_TX/CAN_RX 上)")
        print("       H/L 在卡那端是否接反(自制 DB9 转接最容易把 2/7 搞反)")
        print("       收发器模块的 TX(输入) 那根是否真的接在 MCU 的 CAN1_TX 上")
        print("       (cp 只验了 RX 那根; TX 那根没接 = 收发器永远不驱动显性 = 两边全静默)")
    if fwd_ok and bwd_ok:
        print("\n  ⇒ 物理层**双向都通**。剩下的问题在协议/判据侧, 跑 --smoke 往下走。")
    elif fwd_ok != bwd_ok:
        print("\n  ⇒ 只有单向通 —— 差分总线理论上不会这样, 优先怀疑:")
        print("     ① H/L 接成了「H→L、L→H」(反了: 一侧能收另一侧收不到的表现很像)")
        print("     ② 只有一根线实际接通(另一根虚接/断)")
        print("     ③ 收发器单向损坏")
    else:
        print("\n  ⇒ 双向都不通。按这个顺序量(每步都能证伪):")
        print("     ① 全部断电, 万用表量 H–L: ≈60Ω(两端各 120Ω) 或 ≈120Ω(仅一端);")
        print("        几 kΩ/开路 = **一个终端电阻都没有** ⇒ 先补上再测")
        print("     ② 卡侧 DB9 的 2–7 脚单独量: ≈120Ω = 卡内终端已激活; 开路 = 没有")
        print("     ③ 带电量: 空闲 H≈L≈2.5V; 有帧时 H↑≈3.5V / L↓≈1.5V。")
        print("        若 H/L 读数与上面反了 ⇒ 两根线接反")
        print("     ④ 共地: 量卡的 GND 与臂板 GND, 必须 ≈0V")
        print("     ⑤ 收发器供电与跳线(先 3.3V, 不通再试 5V)")
        print("     ⚠ 别再用「卡报不报总线错」判断 —— 实测它不敏感。")
    bus.shutdown(); ser.close()
    return 0 if (fwd_ok and bwd_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
