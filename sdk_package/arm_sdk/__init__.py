# -*- coding: utf-8 -*-
"""
arm_sdk — 六轴机械臂（STM32F407 + STS3215×6）上位机 SDK

面向 Linux（Windows 亦可），纯 Python 标准库即可跑（串口才需要 pyserial）。

三条链路
--------
1. **网口 UDP（首选）**：实测 p50 往返 **1.61 ms**、p99 3.40 ms，三链路里最快；
   一个 UDP 数据报 = 一个逻辑包，无帧头、无分包、无重传。
2. 串口 USART6（115200）：文本控制台 + 二进制帧（AA 55）共口，永远可用（含 `link` 开关）。
3. CAN 500 kbps：差分总线，物理层最稳，但有分包/乱序，主机侧要重组。

快速上手
--------
    from arm_sdk import open_arm, TeleopSession, TeachRecorder

    arm = open_arm("net://192.168.7.120:5000")
    print(arm.ping())                     # PingInfo(proto_ver=1, run_id=...)
    st = arm.get_status()
    print(st.joints)                      # 舵机度；None = 该轴读失败（不兜底）
    arm.move_joints([180]*6, v=20)        # ⚠ 舵机度，不是逻辑角
    arm.close()

    # 无硬件也能开发
    sim = open_arm("sim://")

主从臂遥操 / 遥操示教
---------------------
    from arm_sdk import TeleopSession, TeleopTeachSession
    lead = open_arm("net://192.168.7.121:5000")     # 主臂
    foll = open_arm("net://192.168.7.120:5000")     # 从臂
    with TeleopSession(lead, foll, hz=50) as tp:
        tp.run(duration=30)                          # 拖主臂 → 从臂跟随

    with TeleopTeachSession(lead, foll, out="demo1.json", hz=50) as ts:
        ts.record(duration=30)                       # 一边遥操一边录（无 80s 上限）
    from arm_sdk import Trajectory, PlaybackSession
    Trajectory.load("demo1.json").save_csv("demo1.csv")   # 导出给 LeRobot 等
"""

from __future__ import annotations

__version__ = "2.0.0"
__all__ = [
    # 协议
    "crc16", "pack_logical", "unpack_logical", "ProtocolError",
    "UdpCarrier", "SerialCarrier", "CanCarrier", "CanReassembler", "FrameStreamParser",
    "payload_move_joints", "payload_stream_joints", "payload_move_cart",
    "decode_status", "decode_fk", "Status", "FkResult", "PingInfo",
    "JointCalib", "JOINT_DIR", "DEFAULT_SERVO_OFFSET", "PARAM_TABLE", "ErrCode",
    "wrap180", "round_half_away",
    # 传输 / 客户端
    "Transport", "NetTransport", "SerialTransport", "make_transport", "RxEvent",
    "ArmClient", "Reply", "ArmCommandError", "open_arm",
    # 主从 / 示教
    "TeleopSession", "TeleopConfig", "TeleopStats",
    "TeleopError", "TeleopLinkLost", "TeleopAbort",
    "FirmwareLink", "PcRelay", "PcRelayStats",
    "TeleopTeachSession", "Trajectory", "PlaybackSession", "PlaybackStats",
    "TeachRecorder", "FirmwareTeach", "TeachError",
    "LOGICAL", "SERVO", "JOINT_NAMES",
    # 控制台（串口文本）
    "ConsoleClient", "Response",
    # 运动学
    "kinematics",
]

from .protocol import (CanCarrier, CanReassembler, DEFAULT_SERVO_OFFSET, ErrCode,
                       FkResult, FrameStreamParser, JOINT_DIR, PARAM_TABLE, PingInfo,
                       ProtocolError, SerialCarrier, Status, UdpCarrier, JointCalib,
                       crc16, decode_fk, decode_status, pack_logical,
                       payload_move_cart, payload_move_joints, payload_stream_joints,
                       round_half_away, unpack_logical, wrap180)
from .transport import NetTransport, RxEvent, SerialTransport, Transport, make_transport
from .client import ArmClient, ArmCommandError, Reply, open_arm

# 主从遥操 / 示教（模块内部才 import 重依赖，这里保持轻量）
from .teleop import (PcRelay, PcRelayStats, FirmwareLink, TeleopAbort, TeleopConfig,
                     TeleopError, TeleopLinkLost, TeleopSession, TeleopStats,
                     FW_MAX_JUMP_DEG, FW_STREAM_TIMEOUT_MS)
from .teach import (LOGICAL, SERVO, JOINT_NAMES, FirmwareTeach, PlaybackSession,
                    PlaybackStats, TeachError, TeachRecorder, TeleopTeachSession,
                    Trajectory)

# 控制台（串口文本）—— 需要 .kinematics，随包提供
from .console import ArmSDK as ConsoleClient
from .console import Response

# 运动学模块（可 `from arm_sdk import kinematics as K`）
from . import kinematics  # noqa: E402
