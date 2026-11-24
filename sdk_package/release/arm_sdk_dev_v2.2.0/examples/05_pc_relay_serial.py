# -*- coding: utf-8 -*-
"""
05_pc_relay_serial.py — 官网式「PC 中转」主从遥操（串口版）

什么时候用它：固件直连链路（PA0/PA1）焊点不良、两臂之间那根线不通，但手上有两根
USB 串口。此时 **两臂之间一根线都不接**，PC 用两根串口当中转：

    主臂 `sp 30` ──控制台吐 `#P a1..a6`──▶ PC ──原样转发──▶ 从臂（走 0x13 同通路）

⚠ 三个前置（PcRelay.auto_gate=True 时会自动做）：
    1. 主臂 `mf 0`  —— 静默固件直连链路；否则两路各读一遍舵机，总线负载翻倍
    2. 从臂 `sf 1`  —— 允许跟随
    3. 从臂 `ec 0`  —— 关回显；否则逐字符回显白占发送线、挤掉收帧时间

串口号会变，先列口：
    Python:  python -c "import serial.tools.list_ports as p; print([x.device for x in p.comports()])"
    Linux:   ls /dev/ttyUSB* /dev/ttyACM*
    Windows: 设备管理器 → 端口(COM 和 LPT)

跑法：
    python examples/05_pc_relay_serial.py COM9 COM11 30 20      # 主臂口 从臂口 频率 秒
    python examples/05_pc_relay_serial.py /dev/ttyUSB0 /dev/ttyUSB1 30 20
"""
import sys

sys.path.insert(0, __file__.rsplit("examples", 1)[0])

from arm_sdk import ArmSDK, FirmwareLink, FirmwareTeach, PcRelay, SerialTransport

if len(sys.argv) < 3:
    raise SystemExit(__doc__)

LPORT, FPORT = sys.argv[1], sys.argv[2]
HZ = int(sys.argv[3]) if len(sys.argv) > 3 else 30
SEC = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

lead = ArmSDK(SerialTransport(LPORT, 115200), response_timeout=2.0)
foll = ArmSDK(SerialTransport(FPORT, 115200), response_timeout=2.0)

with lead, foll:
    # ---- 0. 先认板子（fw-tag 在 h 的首行）----
    for name, c in (("主臂", lead), ("从臂", foll)):
        r = c.send_command("h")
        first = (r.lines or ["(无响应)"])[0]
        print("%s %s -> %s" % (name, LPORT if name == "主臂" else FPORT, first.strip()))

    L, F = FirmwareLink(lead), FirmwareLink(foll)
    T = FirmwareTeach(foll)

    # ---- 1. 链路诊断（命令不回错 ≠ 线是通的，必须看字节数）----
    print("\n--- 主臂链路诊断（lk）---")
    print(L.link_status().text())
    print("--- 从臂链路诊断（lk）---")
    print(F.link_status().text())

    # ---- 2. 两臂姿态对拍（ms）----
    print("--- 校准一致性对拍 ---")
    print("主臂 ms:\n%s" % L.snapshot().text())
    print("从臂 ms:\n%s" % F.snapshot().text())

    # ---- 3. 关回显，省发送线 ----
    foll.send_command("ec 0")
    print("从臂 echo OFF")

    # ---- 4. 中转 ----
    rel = PcRelay(lead, foll, hz=HZ, slew_deg_per_s=240.0, auto_gate=True,
                  on_event=lambda m: print("  [relay] " + m))
    print("\n开始 PC 中转：主臂 %s (sp %d) → 从臂 %s，%.0fs；Ctrl-C 停"
          % (LPORT, HZ, FPORT, SEC))
    rel.start()
    try:
        rel.run(duration=SEC)
    except KeyboardInterrupt:
        rel.stop()
    print(rel.stats.summary())

    # ---- 5. 回读收帧质量（★ 关键：ok/reject/bad 三列）----
    print("\n--- 从臂收帧质量（lk 的 PC relay 段）---")
    print(F.link_status().text())
    print("提示：reject 持续增长 = 整帧被拒（多半是限速不够或 #P 行被截断）；")
    print("      bad 增长 = 行本身解析失败（串口丢字节/两行粘连）。")
    print('      录下来的轨迹无长度上限（与固件 `teach rec` 的 80s 不同）。')
    print("      `teach rec` 也在这里可用：T.enter_rec() → 拖 → T.rec_list() → T.save('demo')")
