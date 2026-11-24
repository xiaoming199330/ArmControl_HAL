#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""can_forge.py — 生成 PCAN-View "New Transmit Message" 里可直接手敲的帧字节。

为什么需要它: 协议每条命令带 CRC16-CCITT(0x1021, init 0xFFFF), 手算不现实;
而字节差一位臂就当 CRC 错拒收(返回码 2)。本工具与 tools/can_host.py 共用同一套
编码函数(payload_move_joints/crc16/build_frames), 保证与真实上位机逐字节一致。

用法(在工程根目录):
  python tools/can_forge.py --ping
  python tools/can_forge.py --stop
  python tools/can_forge.py --move 182.3 76.8 270.7 250.3 182.5 147.1 --speed 10
  python tools/can_forge.py --cart 140.5 1.5 17.4 145.7
  可选: --node 1  --seq 5  (seq 每发一条新命令 +1)
  ★ --send: 算完直接经 USB-CAN 卡发出去(不用再手敲 PCAN-View)。
    ⚠ 发送前必须关闭 PCAN-View, 否则通道被占用打不开。
    默认 PEAK PCAN: --interface pcan --channel PCAN_USBBUS1 --bitrate 500000
    其他卡: --interface slcan --channel COM<x> 等, 与 can_host.py 参数一致。

输出每帧的 ID/Len/Data, 在 PCAN-View 里 Transmit -> New:
  ID 填 hex(默认 101), Length 按表填, Data 逐字节敲进去;
  Message Type 什么都不勾(标准帧), Cycle Time=0(发一次)。
⚠ 一条命令的多帧要**按顺序依次发**(点一条发一条)。
"""
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_host as ch  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="生成 PCAN-View 手发帧字节(与 can_host.py 同一编码)")
    ap.add_argument("--node", type=int, default=1, help="节点号, 默认 1 (ID=0x100+node)")
    ap.add_argument("--seq", type=str, default="1", help="起始序号, 每条命令 +1 (支持 0x 十六进制)")
    ap.add_argument("--ping", action="store_true", help="PING 探活(同时解锁臂的状态帧)")
    ap.add_argument("--stop", action="store_true", help="STOP 停轨迹")
    ap.add_argument("--move", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                    help="关节运动, 舵机度 0~300")
    ap.add_argument("--cart", nargs=4, type=float, metavar=("X", "Y", "Z", "PITCH"),
                    help="笛卡尔运动 mm/mm/mm/deg")
    ap.add_argument("--speed", type=float, default=0.0, help="deg/s, 0=臂内默认")
    ap.add_argument("--accel", type=float, default=0.0, help="deg/s^2, 0=默认")
    ap.add_argument("--send", action="store_true",
                    help="算完直接发送(须先关 PCAN-View; 默认 PEAK PCAN)")
    ap.add_argument("--interface", default="pcan", help="python-can 接口名, 默认 pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1", help="通道, 如 PCAN_USBBUS1 或 COM8")
    ap.add_argument("--bitrate", type=int, default=500000)
    a = ap.parse_args()
    a.seq = int(str(a.seq), 0)   # 支持 0x21 / 33 两种写法

    cid = 0x100 + a.node
    seq = a.seq
    cmds = []
    if a.ping:
        cmds.append(("PING 探活", ch.CMD_PING, b""))
    if a.stop:
        cmds.append(("STOP 停轨迹", ch.CMD_STOP, b""))
    if a.move:
        cmds.append(("MOVE_JOINTS", ch.CMD_MOVE_JOINTS,
                     ch.payload_move_joints(a.move, a.speed, a.accel)))
    if a.cart:
        cmds.append(("MOVE_CART", ch.CMD_MOVE_CART,
                     ch.payload_move_cart(*a.cart, a.speed, a.accel)))
    if not cmds:
        ap.error("没有命令: 用 --ping / --stop / --move ... / --cart ... 至少给一个")

    for label, cmd, payload in cmds:
        print("== %s  (cmd=0x%02X, seq=0x%02X, 负载 %d B) ==" % (label, cmd, seq, len(payload)))
        for i, f in enumerate(ch.build_frames(seq, cmd, payload)):
            print("  帧%d: ID=%03X(hex)  Len=%d  Data=%s"
                  % (i + 1, cid, len(f), " ".join("%02X" % b for b in f)))
        seq += 1
    if a.send:
        import can
        import time
        print()
        print(">>> 直发模式: %s @ %s, %d kbps (PCAN-View 必须已关闭)" % (a.interface, a.channel, a.bitrate // 1000))
        try:
            bus = ch.open_bus_with_retry(a.interface, a.channel, a.bitrate)
        except Exception as e:
            print("!! 打不开 CAN 卡: %s" % e)
            print("   最常见原因: PCAN-View 还开着占用通道 → 关掉它再跑一次")
            sys.exit(1)
        try:
            for label, cmd, payload in cmds:
                print("== 发送 %s (cmd=0x%02X, seq=0x%02X) ==" % (label, cmd, seq))
                for i, f in enumerate(ch.build_frames(seq, cmd, payload)):
                    bus.send(can.Message(arbitration_id=cid, data=f, is_extended_id=False))
                    print("  已发 帧%d: ID=%03X  %s" % (i + 1, cid, " ".join("%02X" % b for b in f)))
                    time.sleep(0.005)   # 帧间小间隔, 保证多帧按序进臂
                seq += 1
            print("✅ 全部帧已发出。观察臂是否动作; Ctrl+C 退出前可再跑 --stop 急停。")
        except KeyboardInterrupt:
            print("(中断)")
        finally:
            bus.shutdown()
        return

    print()
    print("PCAN-View: Transmit -> New Transmit Message; Message Type 全不勾;")
    print("Cycle Time=0; 一条命令的多帧按顺序逐条发送。响应 ID=0x%X, 状态帧 ID=0x%X。"
          % (0x180 + a.node, 0x280 + a.node))


if __name__ == "__main__":
    main()
