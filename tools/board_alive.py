#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
board_alive.py — 一条命令回答"这块板现在到底处于什么状态"

为什么需要它:
  排查 CAN 的时候，最容易在错的假设上花一整轮 —— 比如"板子在跑只是命令没实现"。
  而实际现场反复出现的是另外两种:**板子根本没上电**(CH340 照样枚举, 所以"口在"骗人)
  和 **固件挂住了**(本工程没有看门狗 ⇒ 挂住就是永久静默, 不会自己复位)。
  这两者的现象完全一样: 0 字节输出。必须用别的判据把它们和"在跑"区分开。

判据(按顺序, 前面的过了就不做后面的):
  1. 敲 `h`(所有版本都安全的只读命令), 看回显
  2. 若 0 字节 —— 试 ROM bootloader(57600 8E1)。stm32flash 只读 version 不写, 安全。
     在 bootloader = BOOT0 还挂着高(或没拉回来); 也不在 = MCU 侧真的没在跑。

五种结论:
  NEW_FW      认到 `fw-tag <...>`  ⇒ 本轮固件在跑, 可以测 CAN
  OLD_FW      有控制器横幅但没指纹 ⇒ 旧固件在跑(可跑 cw/ps/cy/ct, **绝不要敲 cx**)
  IN_WIZARD   回显里有 "Joint Calibration" ⇒ 卡在校准向导(敲 q 退出, **千万别敲 s**)
  SILENT      0 字节 ⇒ 没上电 / 挂死 / 串口线断(下面再用 bootloader 细分)
  PORT_BUSY   口打不开 ⇒ 串口助手占着口, 先关掉它

用法:
  python tools/board_alive.py --port COM11
  python tools/board_alive.py --port COM11 --no-bootloader   # 跳过 bootloader 探测, 快一点
  python tools/board_alive.py --list
"""
import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

BAUD = 115200
STM32FLASH = r"C:\STM32Tools\stm32flash.exe"

# 与 arm_preflight 保持同一张指纹表, 别在这里另立一份。
# ⚠ 2026-09-20 修: 原来只 import 单个 FW_TAG —— arm_preflight 改成 FW_TAGS 元组后
#   这边 import 直接失败、静默回落到 "fw-tag ARM-CAN-"，于是**认不出 NET 版固件**，
#   把 0920A/B 全判成 OLD_FW（还顺带建议用户去烧 CAN 版，方向完全反了）。
#   ⇒ 用元组 + 前缀兜底；兜底值同时含 NET 前缀。
try:
    from arm_preflight import FW_TAGS as _FW_TAGS           # noqa: E402
except Exception:                                            # noqa: BLE001
    try:
        from arm_preflight import FW_TAG as _one            # noqa: E402
        _FW_TAGS = (_one,)
    except Exception:                                        # noqa: BLE001
        _FW_TAGS = ("fw-tag ARM-CAN-", "fw-tag ARM-NET-")

OLD_BANNER = "6-Axis Arm Controller"
WIZARD_MARK = "Joint Calibration"


def probe_console(port, baud=BAUD, tries=3, per_try=1.8):
    """开口气 + 敲 h, 返回 (原始回显, 错误信息)。"""
    try:
        import serial
    except ImportError as e:
        return "", "没有 pyserial: %s" % e

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except PermissionError:
        return "", "PORT_BUSY"
    except Exception as e:                            # noqa: BLE001
        return "", "打开 %s 失败: %s" % (port, e)

    try:
        time.sleep(0.4)
        ser.reset_input_buffer()
        ser.write(b"\r")                              # 唤醒(控制台对空行不回显)
        time.sleep(0.4)
        ser.read(2000)
        buf = bytearray()
        for i in range(tries):
            ser.reset_input_buffer()
            ser.write(b"h\r")
            end = time.monotonic() + per_try
            while time.monotonic() < end:
                buf += ser.read(4096)
            if buf:                                   # 有反应就不必再试
                break
        return bytes(buf), ""
    finally:
        ser.close()


def probe_bootloader(port):
    """试 ROM bootloader(57600 8E1)。只读 version, 不写任何东西。

    返回 (在bootloader?, 原文)。
    """
    if not os.path.exists(STM32FLASH):
        return None, "(没找到 %s, 跳过)" % STM32FLASH
    try:
        p = subprocess.run([STM32FLASH, port], capture_output=True, text=True,
                           timeout=25)
        out = (p.stdout or "") + (p.stderr or "")
    except Exception as e:                            # noqa: BLE001
        return None, "(bootloader 探测失败: %s)" % e
    in_bl = ("Device ID" in out) or ("Version      :" in out)
    return in_bl, out.strip()


def classify(text):
    """text 可以是 bytes 或 str —— 统一成 str 再比对。

    ⚠ 别用 bytes 去 in 一个 str: 那是 `TypeError`, 而它会被误当成"脚本坏了",
      看不出"板子其实是静默的"。这个坑本轮真踩了一次。
    """
    if isinstance(text, (bytes, bytearray)):
        text = bytes(text).decode("utf-8", "replace")
    if any(t in text for t in _FW_TAGS):
        return "NEW_FW"
    if WIZARD_MARK in text:
        return "IN_WIZARD"
    if OLD_BANNER in text:
        return "OLD_FW"
    return "SILENT"


def main():
    ap = argparse.ArgumentParser(description="一块板现在是什么状态(在跑/旧版/挂住/没上电)")
    ap.add_argument("--port", help="串口, 如 COM11")
    ap.add_argument("--baud", type=int, default=BAUD,
                    help="控制台波特率, 固定 115200。⚠ 不要为了'排除时钟问题'去扫其它档 —— "
                         "错误波特率下发的字节会变成随机命令注入解析器")
    ap.add_argument("--no-bootloader", action="store_true",
                    help="跳过 ROM bootloader 探测(快 ~3s)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        import serial.tools.list_ports as lp
        for p in lp.comports():
            print("  %-8s %s" % (p.device, p.description))
        return 0

    if not args.port:
        print("✗ 需要 --port (或用 --list 看有哪些口)"); return 2

    print("=" * 68)
    print("  board_alive — %s @%d 8N1" % (args.port, args.baud))
    print("=" * 68)

    text, err = probe_console(args.port, args.baud)
    if err == "PORT_BUSY":
        print("  ✗ 口被占用(PermissionError) ⇒ 先关掉串口助手/其它调试窗口再跑。")
        print("=" * 68)
        return 3
    if err:
        print("  ✗ %s" % err)
        print("  ⚠ 口在、0 字节输出 = 板子没上电或串口线没接(不是波特率问题)。")
        print("=" * 68)
        return 2

    verdict = classify(text)
    icon = {"NEW_FW": "✔", "OLD_FW": "△", "IN_WIZARD": "⚠", "SILENT": "✗"}[verdict]
    print("  结论: %s %s" % (icon, verdict))

    if verdict == "NEW_FW":
        line = ""
        for l in text.decode("utf-8", "replace").replace("\r", "").split("\n"):
            if any(t in l for t in _FW_TAGS):
                line = l.strip(); break
        print("  指纹: %s" % line)
        if "ARM-NET" in line:
            print("  ⇒ NET 版固件在跑。网口排查三步:")
            print("     串口敲 `nx`(PHY/链路/计数器) → `np`(引脚级取证) → `nx scan`(重扫)")
            print("     PC 侧: ping 192.168.7.120, 再 python tools/arm_net_host.py --ping")
        else:
            print("  ⇒ CAN 版固件在跑。可以开始测 CAN:")
            print("     python tools/can_pin_hunt.py --port %s" % args.port)
    elif verdict == "OLD_FW":
        print("  △ 有控制器横幅但没有指纹行。")
        print("    ⚠ 先排除**本工具判据过时**: 若刚烧的是 NET 版或更新的 CAN 版,")
        print("      请确认 tools/arm_preflight.py 的 FW_TAGS 里有你烧的那版指纹。")
        print("    确认是真旧固件的话: 可以跑 cw/ps/cy/cz/ct(它们在那版里真实存在),")
        print("    **绝不要敲 cx** —— 它会掉进首字母兜底 case 'c' 校准向导并松全部力矩。")
        print("  ⇒ 烧对应版本: python tools/flash_arm.py --link can|net --port %s" % args.port)
    elif verdict == "IN_WIZARD":
        print("  ⚠⚠ 板子**正卡在校准向导里**。请在串口终端敲 `q` 退出。")
        print("     ⚠⚠ **千万别敲 `s`** —— 那会把当前姿态固化成零点(见工程红线 4)。")
    else:
        print("  ✗ 敲 h 三次, 一个字都没回。")
        if args.no_bootloader:
            print("     (已跳过 bootloader 探测)")
        else:
            in_bl, raw = probe_bootloader(args.port)
            if in_bl:
                print("  ⚠ 但它**应答了 ROM bootloader**(57600 8E1) ⇒ BOOT0 还挂着高,")
                print("     应用没在跑。把 BOOT0 拉回低再按一下 RESET。")
            elif in_bl is False:
                print("  ⇒ 也不在 ROM bootloader 里。两种可能:")
                print("       ① **MCU 没上电**(CH340 靠 USB 供电照样枚举 ⇒ '口在'不代表板子有电)")
                print("       ② **固件挂住了** —— 本工程没有看门狗, 挂住就是永久静默")
                print("     两种都只能现场处理: 确认电源 → 按 RESET / 断电重上电。")
                print("     ⚠ 复位后应看到启动打印: Init TIM2... / Init UART5 (Bus B)... / Init done.")
    print("=" * 68)
    return 0 if verdict in ("NEW_FW", "OLD_FW") else 1


if __name__ == "__main__":
    sys.exit(main())
