#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一条命令完成「烧录 + 烧后自检」，把 2026-09-18 现场踩的三个坑都堵在脚本里。

现场事实（这脚本为什么长这样）
--------------------------------------------------------------------------
① **烧错了固件**：手敲 stm32flash 时给了 `build/ArmControl_HAL.bin`。
   那是**默认构建**（`LINK=uart`，09-17 17:43），里面**没有** `cx` / `ps` /
   `cw` / `fw-tag` —— 烧上去之后敲 `cx` 照样掉进 `case 'c'` 校准向导、松力矩。
   CAN 版在 `build_can/`，导出版在 `outputs/can_api/`。
   ⇒ 本脚本烧之前先 grep 固件指纹，和 `arm_preflight.FW_TAG` 不一致就**拒绝烧**。

② **认错串口**：Windows 的 COM 号随插拔/换 USB 口会变（本机从 COM11/COM22 变成
   COM8/COM11）。凭记忆认口 = 赌，烧错板子比烧错文件更难发现。
   ⇒ 本脚本每次都把全部串口列出来，并把实际要用的那个标在最上面。

③ **擦除没做完就退出**（`Caught signal 0`）：那是 stm32flash **自己的**信号
   处理器打的（它把信号号按 `%lu` 打出来），即**进程被中断**，不是 STM32 回的错。
   常见于「擦除握手超时」或手动 Ctrl+C。板子不会被烧坏（只要 BOOT0 还高着，
   重新烧一次即可）。
   ⇒ 脚本识别到就自动打印下一步阶梯（`-o` 只擦 / `-e N` 指定页数 / `-b` 换速率）。

用法
--------------------------------------------------------------------------
  python tools/flash_arm.py                      # 从臂 CAN 版 -> COM8
  python tools/flash_arm.py --link net --console-port COM11   # 从臂 NET 版（网口）
  python tools/flash_arm.py --role master        # 主臂 CAN 版
  python tools/flash_arm.py --port COM11 --baud 115200
  python tools/flash_arm.py --erase-first        # 先 `-o` 单独擦一遍再烧
  python tools/flash_arm.py --no-flash           # 只做烧后自检（不烧）
  python tools/flash_arm.py --bin build/ArmControl_HAL.bin   # 会被指纹闸门拒掉

现场步骤（脚本会再提醒一次）
--------------------------------------------------------------------------
  ① BOOT0 拉高  →  ② 按一下 RESET  →  ③ 跑本脚本  →
  ④ BOOT0 拉回低  →  ⑤ 再按 RESET（本脚本用 -g 已经让板子跑起来了，
     但 BOOT0 还高的话下次上电仍会进 bootloader，所以这一步不能省）
"""

import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from arm_preflight import FW_TAG, FW_TAGS, PreflightError, check as preflight  # noqa: E402

STM32FLASH = os.environ.get("STM32FLASH", "C:/STM32Tools/stm32flash.exe")
CONSOLE_BAUD = 115200

BINS = {
    "slave":  os.path.join(ROOT, "outputs", "can_api", "02_slave_CANslave.bin"),
    "master": os.path.join(ROOT, "outputs", "can_api", "01_master_CANslave.bin"),
}
# LINK=net（网口 UDP 承载）的导出版。2026-09-20 新增。
#   ⚠ net 构建与 CAN **互斥**：本板 CAN2 在 PB12/PB13，而 RMII 发送脚取
#     PG11/PG13/PG14（PB11 被 USART3_RX 占、PB12/PB13 是 CAN2 ⇒ B 组出局）；
#      PA1 = ETH_REF_CLK = UART4_RX ⇒ 主从链路也一并停用（见 docs 红线 3）。
NET_BINS = {
    "slave":  os.path.join(ROOT, "outputs", "net_api", "02_slave_NET.bin"),
    "master": os.path.join(ROOT, "outputs", "net_api", "01_master_NET.bin"),
}
# LINK=both（**单固件, 运行期选 CAN / 网口**）的导出版。2026-09-22 新增。
#   ⚠ 与 net/can 两份单链路固件**不是**并列关系: both 是超集, 开机默认
#     **两条都不开**（Flash 扇区 11 区 C 里没记录）⇒ 烧完必须
#     `link can|net` -> `link save` -> RESET 才通。
#     详见 docs/链路开关_link命令_2026-09-22.md。
BOTH_BINS = {
    "slave":  os.path.join(ROOT, "outputs", "both_api", "02_slave_BOTH.bin"),
    "master": os.path.join(ROOT, "outputs", "both_api", "01_master_BOTH.bin"),
}
# 顺便把构建目录里那份也列出来，方便发现「导出件与 build 目录不一致」
# ⚠ 2026-09-18: 现有样机的 CAN1 收发器接在 PA11/PA12 ⇒ 构建目录改用 *_pa 那份,
#   否则 compare_build 会拿 PB 版去比, 每次都报"不一致"却说不清为什么。
BUILD_BINS = {
    "slave":  os.path.join(ROOT, "build_can_pa", "ArmControl_HAL.bin"),
    "master": os.path.join(ROOT, "build_master_can_pa", "ArmControl_Master.bin"),
}
# net 构建目录（LINK=net → build_net/）
BUILD_BINS_NET = {
    "slave":  os.path.join(ROOT, "build_net", "ArmControl_HAL.bin"),
    "master": os.path.join(ROOT, "build_master_net", "ArmControl_Master.bin"),
}
# both 构建目录（LINK=both → build_both/）
BUILD_BINS_BOTH = {
    "slave":  os.path.join(ROOT, "build_both", "ArmControl_HAL.bin"),
    "master": os.path.join(ROOT, "build_master_both", "ArmControl_Master.bin"),
}
# 引脚组闸门用的特征串（`h` 的指纹行里带 `pins=CAN1 <desc>`，见 cmdHelp）。
EXPECT_PIN_MARK = "PA11(RX)/PA12(TX)"
WRONG_PIN_MARK = "PB8(RX)/PB9(TX)"


def _banner(msg):
    print("=" * 70)
    print("  " + msg)
    print("=" * 70)


def list_ports():
    try:
        import serial.tools.list_ports as lp
    except Exception:                                     # noqa: BLE001
        return []
    return [(p.device, p.description) for p in lp.comports()]


def find_tag(path):
    """返回 bin 里的固件指纹原文（形如 `fw-tag ARM-CAN-0918G`），没有则 None。

    `cprint` 的格式串会直接进 `.bin`，所以这是比「看构建目录时间戳」硬得多的
    判据 —— 时间戳只说明文件什么时候写的，不说明里面是哪一代代码。
    """
    try:
        data = open(path, "rb").read()
    except OSError as e:
        print("✗ 读不了 %s: %s" % (path, e))
        return None
    m = re.search(rb"fw-tag ARM-(?:CAN|NET|BOTH)-[0-9A-Za-z._-]+", data)
    return m.group(0).decode("ascii", "replace") if m else None


def gate_bin(path, allow_unknown, link="can"):
    """烧之前的第一道闸：这份 bin 是不是「工具链认识的那一代」。"""
    if not os.path.isfile(path):
        print("✗ 找不到固件文件: %s" % path)
        return False
    size = os.path.getsize(path)
    tag = find_tag(path)
    print("  固件: %s" % path)
    print("  大小: %d B" % size)
    if tag is None:
        print("  指纹: **没有** fw-tag ⇒ 这份是 09-18 之前那一代（无 cx/ps/cw）")
        if allow_unknown:
            print("  ⚠ --allow-unknown: 放行。但烧上去之后 cx/ps 大概率不存在。")
            return True
        print()
        print("  ✗ **拒绝烧录**。你现在多半给到了 `build/ArmControl_HAL.bin` ——")
        print("    那是**默认构建**（LINK=uart），不是 CAN 版，而且早于 09-18。")
        print("    CAN 从站在 `outputs/can_api/02_slave_CANslave.bin`（导出版）")
        print("    或 `build_can/ArmControl_HAL.bin`（构建目录）。")
        print("    （确知风险仍要烧: 加 --allow-unknown）")
        return False
    print("  指纹: %s" % tag)

    # ── LINK=net 分支（2026-09-20）───────────────────────────────
    # net 构建里**没有** CAN 配置区 / CAN 引脚组，下面那两道闸检查的字符串
    # 本来就不存在 ⇒ 必须在这里分流，否则会把好好的 net 固件判成"认不出引脚组"。
    if link == "net":
        if not tag.startswith("fw-tag ARM-NET-"):
            print("  ⚠ 你要烧的是 net 版（LINK=net），但这份 bin 的指纹不是 ARM-NET 系列。")
            print("    net 版产物: %s" % NET_BINS.get("slave"))
            print("    （用 CAN 版占用 PB13/PA1 的固件去跑网口是跑不起来的）")
            if not allow_unknown:
                return False
            print("  ⚠ --allow-unknown: 照烧。")
        else:
            print("  ✔ 指纹是 ARM-NET 系列（LINK=net）")
        print("  ⚠ LINK=net 的三条硬约束（必读）:")
        print("     · **CAN 从站停用** —— LINK=net 是独立构建，CAN 外设未使能")
        print("     · **主从链路(UART4)停用** —— PA1 = ETH_REF_CLK")
        print("     · **控制台仍在 USART6(PC6/PC7) @115200** —— 它和烧录口不是同一个串口,")
        print("       所以烧完要在**控制台口**上看回显, 不是 --port 那个口")
        print("     · 网口参数: 静态 IP 192.168.7.120:5000, PHY = LAN8720A(RMII), MAC 02:4E:45:54:00:01")
        print("     · 烧完**不需要** `cx 2` —— net 构建根本没启用 CAN, 而且 `cx`/`ps`/`can`")
        print("       等命令已被固件**显式拦掉**(它们会把 PB12/PB13 重配成 AF9_CAN2 = 掐断网口)")
        return True

    # ── LINK=both 分支（2026-09-22）──────────────────────────────
    # both 是 **CAN 与 网口 的超集**：两条链路的外设都编进来了，开机由
    # Flash 里的偏好决定初始化哪一条（默认两条都不开）。所以它既不能按
    # net 的闸门判（会要求 ARM-NET 前缀），也不能按 CAN 的闸门判。
    if link == "both":
        if not tag.startswith("fw-tag ARM-BOTH-"):
            print("  ⚠ 你要烧的是 both 版（LINK=both），但这份 bin 的指纹不是 ARM-BOTH 系列。")
            print("    both 版产物: %s" % BOTH_BINS.get("slave"))
            print("    （烧成单链路固件也能用，但就没有 `link` 这条运行期切换命令了）")
            if not allow_unknown:
                return False
            print("  ⚠ --allow-unknown: 照烧。")
        else:
            print("  ✔ 指纹是 ARM-BOTH 系列（LINK=both，串口常通 + CAN/网口二选一）")
        print("  ⚠⚠ LINK=both 的**第一条硬约束（必读）**:")
        print("     · 开机默认**两条链路都不开**（Flash 扇区 11 区 C 里没记录）")
        print("       ⇒ 烧完**必须先** `link can` 或 `link net` -> `link save` -> RESET，")
        print("         否则网口 ping 不通 / CAN 不应答，现象和\"烧坏了\"一模一样。")
        print("     · 切换链路**只能重启生效**（引脚 AF 是启动时配的，不做热切换）")
        print("     · 串口 USART6(PC6/PC7) @115200 **永远常通**，是切换唯一可靠通道")
        print("     · 详见 docs/链路开关_link命令_2026-09-22.md")
        return True

    if not tag.startswith("fw-tag ARM-CAN-"):
        print("  ⚠ 指纹与工具链预期不符（CAN 版应为 %s 之一: %s）" % ("/", "/".join(FW_TAGS)))
        print("    ⇒ 要么这份是新一代固件（那请同步更新 arm_preflight.FW_TAGS），")
        print("       要么它跟 cx/ps 那套工具对不上。确认无误再加 --allow-unknown。")
        if not allow_unknown:
            return False
    else:
        print("  ✔ 指纹是 ARM-CAN 系列（与 arm_preflight.FW_TAGS 一致）")

    # ── 第三道闸: 引脚组 ─────────────────────────────────────────
    # 2026-09-18 现场: 板子的 CAN1 收发器实际接在 PA11(RX)/PA12(TX), 固件却一直
    # 驱动 PB8/PB9 ⇒ 表现是"卡在发、arm 侧 MSR.RX 恒 1、收帧计数器全 0",
    # **和"线没接/接错端子"一模一样**, 白烧了几小时。
    # ⇒ 这块板(现有样机)必须烧 `CANPIN=pa`; 烧成 pb 会原样复现那个假故障。
    # ⚠ 0918H 起开机默认接口已改成 CAN2 ⇒ 本闸门重要性下降（CAN1 的引脚组只在
    #   `cx 1` 之后才生效），但保留作防呆：真烧错了敲 `cx pa|pb` 即可，不必重烧。
    data = open(path, "rb").read()

    # ── 第二道闸: 开机默认接口（2026-09-18 晚, 0918H 新增）──────
    # 0918H 起编译期默认接口 = CAN2(PB12/PB13) = 本板卡的实际接线 ⇒ **烧完不用 `cx 2`**。
    # ≤0918G 默认是 CAN1，而烧录走整片擦除（配置区被抹）⇒ 每次烧完都回落到 CAN1，
    # 必须先 `cx 2`；漏了这一步的表现是"指纹对、但上位机全部超时"，极易误判成烧坏了（红线 37）。
    # ⚠ 本条只提示、**不拒绝**：旧版固件也能用，只是多一步。
    if b"defif=CAN2" in data:
        print("  ✔ 默认接口: CAN2(PB12/PB13) —— 烧完**不需要** `cx 2`")
    else:
        print("  ⚠ 默认接口: 非 CAN2（≤0918G）—— 烧完**必须** `cx 2`，否则上位机全部超时（红线 37）")

    if EXPECT_PIN_MARK.encode() in data:
        print("  ✔ 引脚组: CAN1 %s（与板子实际接法一致）" % EXPECT_PIN_MARK)
    elif WRONG_PIN_MARK.encode() in data:
        print()
        print("  ✗ **引脚组不对**: 这份固件把 CAN1 驱动在 %s 上，" % WRONG_PIN_MARK)
        print("    但本板收发器接的是 %s。" % EXPECT_PIN_MARK)
        print("    ⇒ 烧上去会**原样复现假故障**: 卡在发、臂侧 MSR.RX 恒 1、收帧计数器全 0,")
        print("       看上去和'线没接'完全一样（2026-09-18 就为这个白烧了几小时）。")
        print("    正确固件: make LINK=can CANPIN=pa [ROLE=master] → build*_pa/")
        print("    ⚠ 从 0918E 起引脚组运行期可切 —— 万一真烧错了, 上电敲 `cx pa` 即可,")
        print("       不用再拉 BOOT0 重烧。但**默认值**还是烧对更省事。")
        print("    （确知要烧 pb 版: 加 --allow-unknown）")
        if not allow_unknown:
            return False
    else:
        print("  ⚠ 这份 bin 里既没有 %s 也没有 %s —— 认不出引脚组," % (EXPECT_PIN_MARK, WRONG_PIN_MARK))
        print("    可能太老。确认无误加 --allow-unknown。")
        if not allow_unknown:
            return False
    return True


def compare_build(path, role, link="can"):
    """导出版 vs 构建目录那份，不一致就提醒（防「我以为导出了」）。"""
    b = (BUILD_BINS_NET if link == "net" else
         BUILD_BINS_BOTH if link == "both" else BUILD_BINS).get(role)
    if not b or not os.path.isfile(b):
        return
    import hashlib

    def md5(p):
        return hashlib.md5(open(p, "rb").read()).hexdigest()
    h1, h2 = md5(path), md5(b)
    if h1 != h2:
        print("  ⚠ 导出版 md5=%s ≠ build 目录 md5=%s" % (h1[:12], h2[:12]))
        print("    (build 目录那份是 %s) —— 确认你要烧的是哪一份。" % b)
    else:
        print("  ✔ 与 build 目录那份 md5 一致 (%s)" % h1[:12])


def run_flash(bin_path, port, baud, erase_first, timeout=240):
    """调 stm32flash。返回 (ok, 输出文本)。"""
    def _run(args, label):
        _banner(label)
        print("  " + " ".join('"%s"' % a if " " in a else a for a in args))
        print("-" * 70)
        try:
            p = subprocess.run(args, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, timeout=timeout)
        except FileNotFoundError:
            print("✗ 找不到 stm32flash: %s（用 --stm32flash 或环境变量 STM32FLASH 指定）"
                  % STM32FLASH)
            return None
        except subprocess.TimeoutExpired:
            print("✗ 超过 %ds 还没结束 —— 串口可能被别的程序占着（串口助手/终端）"
                  % timeout)
            return None
        txt = p.stdout.decode("utf-8", "replace")
        print(txt.rstrip())
        return txt

    if erase_first:
        out = _run([STM32FLASH, "-o", port], "① 先单独擦一遍（-o erase only）")
        if out is None:
            return False, ""
        if "Caught signal" in out or "Failed" in out:
            print("  ⚠ 单独擦除就没过 —— 先解决链路/供电/BOOT0，再谈烧录。")

    args = [STM32FLASH, "-w", bin_path, "-v", "-g", "0x08000000"]
    if baud:
        args += ["-b", str(baud)]
    args.append(port)
    out = _run(args, "② 写入 + 校验 + 启动")
    if out is None:
        return False, ""

    bad = [k for k in ("Caught signal", "Failed to init device", "Failed to erase",
                       "Mass erase failed", "Failed to write memory",
                       "Failed to verify") if k in out]
    if bad:
        print()
        print("=" * 70)
        print("  ✗ 烧录没完成: %s" % ", ".join(bad))
        print("-" * 70)
        if "Caught signal" in out:
            print("  `Caught signal 0` = stm32flash 自己的信号处理器报「进程被中断」，")
            print("  不是 STM32 回的错；最常见是**擦除握手超时**。板子没坏 ——")
            print("  只要 BOOT0 还高着，重烧一次就行。按这个阶梯试：")
            print("    1) 再跑一遍本脚本（擦除时序是边缘性的，重跑常能过）")
            print("    2) 加 --erase-first（先 -o 单独擦，把擦除与写入分开）")
            print("    3) 加 -b 换个速率: --baud 115200 / --baud 9600")
            print("    4) 确认 BOOT0 在**整个**擦写过程中稳定为高（别用悬空的线夹着），")
            print("       擦除期间别碰 RESET；USB 口换直连、线换短的")
            print("    5) 机械臂板用**独立电源**供电，USB 只做串口（擦除电流大，")
            print("       CH340 的 3V3 带不动整臂时会在擦除瞬间掉压）")
        if "Failed to init device" in out:
            print("  连不上 ROM bootloader ⇒ 三查: BOOT0 真的高了吗 / 按过 RESET 了吗 /")
            print("  这个 COM 是不是你要烧的那块板的（列出来的口自己认一遍）")
        print("=" * 70)
        return False, out
    print()
    print("  ✔ stm32flash 报成功（写入 + 校验都过）")
    return True, out


def post_check(port, wait=3.0):
    """烧后自检：端口切回 115200 8N1，敲 `h` 认指纹。"""
    _banner("③ 烧后自检（切回 %d 8N1，敲 h 认指纹）" % CONSOLE_BAUD)
    try:
        import serial
    except Exception:                                     # noqa: BLE001
        print("  ⚠ 没有 pyserial，跳过自检（用隔离环境的 python 跑本脚本）")
        return True
    print("  ⚠ 记得把 BOOT0 拉回低 + 按一下 RESET，应用才会常驻运行")
    time.sleep(wait)
    try:
        ser = serial.Serial(port, CONSOLE_BAUD, timeout=0.2)
    except Exception as e:                                # noqa: BLE001
        print("  ⚠ 打不开 %s 做自检: %s（串口助手占了？）" % (port, e))
        return True
    try:
        time.sleep(0.4)
        ser.reset_input_buffer()
        preflight(ser)
        print()
        print("  ✔✔ 自检通过：板子跑的就是这一代固件（有 fw-tag）")
        print("  ⇒ 下一步: python tools/can_pin_hunt.py --port %s" % port)
        return True
    except PreflightError as e:
        print(str(e))
        print()
        print("  ✗ 自检没过 —— 别接着敲 cx/ps。先看上面 h 的原文：")
        print("    · 一个字都没回 ⇒ 板子没上电 / 串口被占 / BOOT0 还高着（还在 bootloader）")
        print("    · 是老固件的 help ⇒ 没烧进去（或烧错了文件/板子）")
        return False
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="烧录 6 轴臂固件 + 烧后自检（带三道闸）")
    ap.add_argument("--role", choices=("slave", "master"), default="slave",
                    help="从臂(默认) / 主臂")
    ap.add_argument("--link", choices=("can", "net", "both"), default="can",
                    help="链路形态: can(默认, CAN 从站) / net(网口 UDP 承载)")
    ap.add_argument("--bin", default=None,
                    help="直接指定 bin（默认按 --link 取 outputs/{can,net}_api/ 的导出版）")
    ap.add_argument("--port", default="COM8", help="烧录口（默认 COM8；每次都会把全部口列出来）")
    ap.add_argument("--console-port", default=None,
                    help="控制台口（烧后自检用）。默认与 --port 相同；"
                         "net 构建下控制台在 USART6, 与烧录口不同, 要显式给（如 COM11）")
    ap.add_argument("--baud", type=int, default=0,
                    help="stm32flash 的 -b（不给就用工具默认 57600；慢了可 115200）")
    ap.add_argument("--erase-first", action="store_true", help="先 `-o` 单独擦一遍再烧")
    ap.add_argument("--no-flash", action="store_true", help="不烧，只做烧后自检")
    ap.add_argument("--no-check", action="store_true", help="不做事后自检")
    ap.add_argument("--allow-unknown", action="store_true",
                    help="指纹对不上也照烧（危险，默认拒烧）")
    ap.add_argument("--stm32flash", default=None, help="stm32flash.exe 路径")
    args = ap.parse_args()

    global STM32FLASH
    if args.stm32flash:
        STM32FLASH = args.stm32flash

    bins = (NET_BINS if args.link == "net" else
            BOTH_BINS if args.link == "both" else BINS)
    bin_path = args.bin or bins[args.role]

    _banner("烧录前的三道确认（文件指纹 / 串口 / BOOT0）")
    print("  角色: %s    链路: %s    烧录口: %s    控制台口: %s    stm32flash: %s"
          % (args.role, args.link, args.port,
             args.console_port or ("(同烧录口)" if args.link in ("can", "both") else "(未指定)"),
             STM32FLASH))
    ports = list_ports()
    if ports:
        print("  当前机器上的串口:")
        for dev, desc in ports:
            mark = "  <== 本脚本要用这个" if dev.upper() == args.port.upper() else ""
            print("      %-8s %s%s" % (dev, desc, mark))
        if not any(d.upper() == args.port.upper() for d, _ in ports):
            print("  ⚠ 列表里**没有** %s —— 先用 --port 指定一个存在的口。" % args.port)
            return 2
    else:
        print("  ⚠ 没列出任何串口（pyserial 缺失？）")
    print()

    if not gate_bin(bin_path, args.allow_unknown, args.link):
        return 3
    compare_build(bin_path, args.role, args.link)
    print()
    print("  BOOT0: 拉高 → 按一下 RESET → （现在）→ 烧完拉回低 → 再按 RESET")
    print("  ⚠ 若你没做这一步, stm32flash 会报 `Failed to init device`")
    print()

    if args.no_flash:
        print("  (--no-flash: 跳过烧录)")
    else:
        ok, _ = run_flash(bin_path, args.port, args.baud, args.erase_first)
        if not ok:
            return 4

    if args.no_check:
        return 0

    # ── 烧后自检要用**控制台口**，不是烧录口 ──────────────────────
    # 控制台 = USART6(PC6/PC7)。CAN 版现场这两个口常常是同一个
    # (CH340 既接了 bootloader 又接了控制台), 但 net 构建下必须分开给,
    # 否则自检会对着一个"永远不说话的 bootloader 口"敲 h, 误报"没烧进去"。
    cport = args.console_port
    if cport is None:
        if args.link == "net":
            print()
            print("  ⚠ net 构建: 控制台在 USART6, 与烧录口 --port %s 不是同一个串口。" % args.port)
            print("    ⇒ 跳过烧后自检。想自检请加 --console-port <控制台口>（如 COM11），")
            print("       或直接在控制台口敲 `h`，看首行是不是 `fw-tag ARM-NET-…`。")
            return 0
        cport = args.port
        if args.link == "both":
            print("  ℹ both 构建: 默认按「控制台与烧录口同一个串口」做烧后自检（--port %s）。" % args.port)
            print("    若你的控制台在另一个口（USART6 / COM11），请加 --console-port <口>。")
            print("    ⚠ 烧后**别急着测网口/CAN** —— 开机默认两条都不开，先 `link can|net`+`save`+RESET。")
    return 0 if post_check(cport) else 1


if __name__ == "__main__":
    sys.exit(main())
