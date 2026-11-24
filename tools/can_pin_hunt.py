#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_pin_hunt.py — 一条命令回答: **USB-CAN 卡的数据到底落在 MCU 的哪只脚上?**

背景(2026-09-18):
  用户问"板子有没有解析 CAN2 过来的数据, CAN2 是 PB13/PB14"。
  工程事实: CAN2 在**业务代码/.ioc/gpio.c 里一处都没有** —— 只存在于 CMSIS 寄存器定义。
  所以"固件解析 CAN2"这件事不存在。但用户的问题本身指向一个**真实可能**:
  板子上(或外接)还有第二个 CAN 口, 卡接在了那个口上, 而固件只讲 CAN1(PB8/PB9)。
  两种情况的现场症状**完全一样**: 帧发得出去(TEC+8/lec=3)、没人应答、RX 脚一动不动。

本脚本把三件事放进**同一个时间窗口**:
  1) 卡那一端持续灌帧(0x7E0, 数据全 0 = 显性位最多) —— 开卡重试 + 报入队帧数,
     入队 0 帧就直接判定"本次读数无效", 不让"卡没发"伪装成"链路不通"
  2) 灌帧期间在臂控制台上敲 cx / cw / ps
       cx  —— 打印**当前正在用哪颗控制器、哪一对脚**(2026-09-18 起接口运行时可切)
       cw  —— 外环: TX -> 收发器 -> 总线 -> 收发器 -> RX
       ps  —— 引脚嗅探: 11 只候选脚, 看哪只出现活动信号

★ 2026-09-18 起固件支持 `cx 1|2` **运行期**切 CAN1/CAN2, 所以本工具也加了
  `--cx 1|2`: 灌帧开始后先切过去, 后面 cw/cz/ct 的结果就是"新那一组脚"的。
  验"卡是不是插在板子的另一个 CAN 口上"再也不用重烧固件。

用法:
  python tools/can_pin_hunt.py --port COM11 --channel PCAN_USBBUS1
  python tools/can_pin_hunt.py --port COM11 --cx 2          # 先切到 CAN2 再取证
  python tools/can_pin_hunt.py --port COM11 --cmds cx,cp,cw,cz,ct

⚠ 前置检查(2026-09-18): 开跑前先敲 `h` 认固件指纹 `fw-tag ARM-CAN-<版本>`
  (期望值 = `arm_preflight.FW_TAG`, 现役 `ARM-CAN-0918G`)。
  认不到就**直接退出, 一条 c 开头的命令都不发** —— 因为在老固件上 `cx` 不会报
  "未知命令", 而是掉进首字母兜底 `case 'c'` → **校准向导 → 松全部力矩**。
  确知风险仍要硬跑: 加 --force(仍有第二道闸拦住危险首字母)。
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description="灌帧 + 臂侧 cx/cw/ps 联动取证")
    ap.add_argument("--port", default="COM11", help="臂控制台串口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--id", default="0x7E0")
    ap.add_argument("--secs", type=float, default=24.0, help="灌帧总时长")
    ap.add_argument("--gap-ms", type=float, default=2.0)
    ap.add_argument("--cx", type=int, default=0,
                    help="灌帧开始后先切 CAN 接口: 1=CAN1 2=CAN2 0=不动")
    ap.add_argument("--cmds", default="cx,cw,ps",
                    help="灌帧期间要敲的臂侧命令(逗号分隔), 默认 cx,cw,ps")
    ap.add_argument("--cmd1", default=None, help="[兼容] 等价于 --cmds 的第一条")
    ap.add_argument("--cmd2", default=None, help="[兼容] 等价于 --cmds 的第二条")
    ap.add_argument("--force", action="store_true",
                    help="跳过固件指纹前置检查(危险: 老固件上敲 cx 会松全部力矩)")
    args = ap.parse_args()

    if args.cmd1 or args.cmd2:                        # 兼容旧调用
        args.cmds = ",".join([c for c in (args.cmd1, args.cmd2) if c])

    import can
    import serial
    from can_flood import open_bus_with_retry, read_card_status
    from arm_preflight import (check as preflight, assert_safe_cmd,
                               PreflightError, ST_VERIFIED, NEW_ONLY_CMDS)

    can_id = int(args.id, 16) if args.id.lower().startswith("0x") else int(args.id)
    print("=" * 68)
    print("  can_pin_hunt — 卡的数据落在哪只 MCU 脚上")
    print("=" * 68)

    try:
        bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
    except Exception as e:                            # noqa: BLE001
        print("✗ 开卡失败: %s" % e)
        print("  → 关掉 PCAN-View / CanAst.exe, 或把卡拔插一次再试")
        return 2
    _, st0 = read_card_status(bus)
    print("  卡 %s@%d   id=0x%03X(数据全 0 = 显性位最多)   臂 %s@%d"
          % (args.channel, args.bitrate, can_id, args.port, args.baud))
    print("  开卡成功 ✔   发送前卡状态: %s" % st0)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:                            # noqa: BLE001
        print("✗ 打开臂串口 %s 失败: %s" % (args.port, e))
        print("  (PermissionError/拒绝访问 = 口被别的程序占着, 先关掉那个串口工具)")
        bus.shutdown()
        return 2
    time.sleep(0.3)
    ser.reset_input_buffer()
    for _ in range(3):                                # 唤醒控制台
        ser.write(b"\r")                              # ⚠ 只发 \r, 别发 \r\n
        time.sleep(0.35)
    ser.read(400)

    # ── 固件指纹前置检查 (2026-09-18) ──────────────────────────────
    # 老固件上敲 cx/cw/ps 不会报"未知命令", 而是掉进首字母兜底 case:
    # cx → case 'c' → 校准向导 → **松全部力矩**。这块板重烧要手动拉 BOOT0,
    # 所以宁可不跑, 也不能在不确定固件版本的板子上敲 c 开头的命令。
    try:
        fw_state = preflight(ser, force=args.force)
    except PreflightError as e:                       # noqa: BLE001
        print(str(e))
        print("=" * 68)
        ser.close()
        bus.shutdown()
        return 3

    stop = threading.Event()
    stat = {"n": 0, "err": 0}

    def flood():
        while not stop.is_set():
            try:
                bus.send(can.Message(arbitration_id=can_id, data=bytes(8),
                                     is_extended_id=False))
                stat["n"] += 1
            except Exception:                          # noqa: BLE001
                stat["err"] += 1
                if stat["err"] > 20:
                    break
            time.sleep(args.gap_ms / 1000.0)

    th = threading.Thread(target=flood, daemon=True)
    th.start()
    time.sleep(2.0)
    if stat["n"] == 0:
        print("✗ 灌帧**一帧都没发出去** ⇒ 下面的读数全部无效, 先解决开卡/占用问题。")
        stop.set()
        th.join(timeout=1.0)
        ser.close()
        bus.shutdown()
        return 2
    print("  灌帧已启动(头 2 秒入队 %d 帧) —— 下面开始读臂侧\n" % stat["n"])

    # --force 也要拦住"只存在于本轮固件"的命令: 老固件上它们零收益, 却会按
    # 首字母兜底成 `case 'c'` → 校准向导 → 松全部力矩。半路抛异常不如提前说清。
    if fw_state != ST_VERIFIED:
        want = ([("cx %d" % args.cx)] if args.cx else []) + \
               [c for c in args.cmds.split(",") if c.strip()]
        blocked = [c for c in want if c.strip().split()[0] in NEW_ONLY_CMDS]
        if blocked:
            print("✗ 固件指纹未认到, 以下命令只存在于本轮固件, 拒绝发送: %s"
                  % ", ".join(blocked))
            print("  想只在老固件上取证: 去掉 --cx, 并改 --cmds cw,ps (老固件里有这两条)。")
            print("=" * 68)
            stop.set()
            th.join(timeout=1.0)
            ser.close()
            bus.shutdown()
            return 3

    def ask(cmd, secs):
        assert_safe_cmd(cmd, fw_state)     # --force 时仍拦住危险首字母
        ser.reset_input_buffer()
        ser.write((cmd + "\r").encode())   # ⚠ 只发 \r(发 \r\n 会多一个 \n 掉进交互模式)
        t0 = time.time()
        buf = b""
        while time.time() - t0 < secs:
            buf += ser.read(600)
        txt = buf.decode("utf-8", "replace").replace("\r", "")
        return [l.rstrip() for l in txt.split("\n") if l.strip()]

    if args.cx:
        print("########## 臂侧 `cx %d` (切 CAN 接口, 灌帧进行中) ##########" % args.cx)
        for l in ask("cx %d" % args.cx, 3.0):
            print("  " + l)
        print()

    for cmd in [c.strip() for c in args.cmds.split(",") if c.strip()]:
        secs = 5.0 if cmd.split()[0] in ("ps",) else 3.0
        print("########## 臂侧 `%s` (灌帧进行中) ##########" % cmd)
        lines = ask(cmd, secs)
        if not lines:
            print("  (没有输出 —— 板子可能没上电/串口被占; '口在、无输出' 就是没上电)")
        for l in lines:
            print("  " + l)
        print()

    stop.set()
    th.join(timeout=1.5)
    _, st1 = read_card_status(bus)
    print("=" * 68)
    print("  灌帧合计入队 %d 帧, 发送异常 %d 次, 结束后卡状态: %s"
          % (stat["n"], stat["err"], st1))
    print("  ⚠ 入队 0 帧 ⇒ 本次读数无效, 不许据此下结论")
    print("-" * 68)
    print("  怎么读上面的输出:")
    print("   cx: 先看'当前正在用'是 CAN1(PB8/PB9) 还是 CAN2(PB12/PB13)")
    print("   cw: 显性->00000000 且 隐性->11111111")
    print("        ⇒ 外环通: MCU 与**当前这组脚上的**收发器都好 ⇒ 断点在「卡 ↔ 模块 CANH/CANL」")
    print("      显性->11111111 (仍隐性高)")
    print("        ⇒ 外环断: 当前这组脚上的收发器没工作 ⇒ 收发器很可能**不在这组脚上**")
    print("   ps: 盯 PB8 / PB9 / PB12 / PB13 四行")
    print("       ★(边沿≥8) 出现在 PB8/PB9   ⇒ 卡在 CAN1 口上, 查收发器与线")
    print("       ★ 出现在 PB12/PB13        ⇒ 卡在板子的 CAN2 口上")
    print("                                    ⇒ **不用重烧**, 加 --cx 2 再跑一次这个脚本")
    print("       11 行全无 ★               ⇒ 卡的信号没进到 MCU 任何一只 CAN 脚上")
    print("     (PB14/PB15 在 F407 上没有 CAN 复用功能, 出现 ★ 说明那两根线串了别的信号)")
    print("=" * 68)

    ser.close()
    bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
