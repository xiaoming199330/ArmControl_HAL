# -*- coding: utf-8 -*-
"""
06_firmware_console.py — 串口文本控制台：链路开关 / 固件直连主从 / 固件录音

串口（USART6 PC6/PC7 @115200）是**永远常通**的通道：
CAN 与网口物理互斥（PB13 是 ETH_TXD1 又是 CAN2_TX），切链路只能靠串口。

跑法：
    python examples/06_firmware_console.py COM11 h s zi L link
    python examples/06_firmware_console.py /dev/ttyUSB0 h
    python examples/06_firmware_console.py COM11 link net      # 改偏好
    python examples/06_firmware_console.py COM11 link save     # 落盘（之后要 RESET）
"""
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import ArmSDK, FirmwareLink, FirmwareTeach, SerialTransport

if len(sys.argv) < 2:
    raise SystemExit(__doc__)

PORT = sys.argv[1]
CMDS = sys.argv[2:] or ["h"]

with ArmSDK(SerialTransport(PORT, 115200), response_timeout=20.0) as c:
    L, T = FirmwareLink(c), FirmwareTeach(c)

    for cmd in CMDS:
        r = c.send_command(cmd, timeout=20.0)
        print("\n>>> %s\n%s" % (cmd, r.text()))
        if r.clamps():
            print("  ⚠ 被限位钳制：%s" % r.clamps())
        if not r.ok:
            print("  ✗ kind=%s" % r.kind)

    # ---------------------------------------------------------------- 常用片段
    #
    # 【链路开关】工程红线 69：运行期不热切换，改偏好 → 落盘 → RESET
    #     print(L.link_show().text())      # 本次生效 / 下次开机 / 是否已落盘
    #     L.link_set("net")                # 只改 RAM
    #     L.link_save()                    # 写 Flash 扇区11 区C
    #     # 然后按板子上的 RESET（不要断电：CCM RAM 跨复位保留）
    #     # 复位后：`h` 看首行 active=net
    #
    # 【固件直连主从（UART4 PA0/PA1）】
    #     print(L.link_status().text())    # lk：字节/帧/CRC 错/uart 硬件错
    #     L.forwarding(True)               # mf 1
    #     L.active(False)                  # ma 0 手拖主臂
    #     F = FirmwareLink(从臂控制台)
    #     F.follow(True)                   # sf 1
    #     # 判"线到底通不通"必须看 lk 的字节数，不能看命令有没有回错：
    #     L.loopback(200)                  # ml 200（舵机断电也能跑）
    #     L.gpio_loopback()                # mg（需 PA0<->PA1 跳线）
    #     L.pin_probe()                    # mp（不需跳线）
    #
    # 【固件录音（从臂，5~80s）】
    #     T.enter_rec()                    # teach rec：只读跟录，不碰舵机（★PC 中转时用）
    #     # …拖主臂…
    #     print(T.rec_list().text())       # 存盘前必须核对点数
    #     T.save("demo1")                  # 阻塞 1~3s
    #     T.play("demo1")                  # 或 T.play("demo1", loop=True)
    #     print(T.files().text())          # 列表；del / compact 管理空间
    #     # ⚠ 三种丢录音方式：rec clear / 断电 / 再进一次 teach
    #
    # 【零点检查（★排障第一件事）】
    #     print(c.send_command("zi").text())
    #     # 健康值：未校准全 0；已校准接近 2048×6
    #     # 若被示教模式误触 '0' 污染 ⇒ `zr save` 复位（并重算 IK 限位）
    #
    # 【运行时限位】
    #     print(c.send_command("L").text())
    #     # J1 63~303 / J2 75~284 / J3 76~271 / J4 71~284 / J5 12~351 / J6 138~266 (舵机度)
