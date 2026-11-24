# -*- coding: utf-8 -*-
"""
protocol.py — 六轴机械臂「逻辑包」协议层（纯标准库，无 HAL / 无串口依赖）

一个逻辑包在固件里只有一种定义：

    SEQ(1) | CMD(1) | LEN(1) | PAYLOAD(LEN) | CRC16-LE(2)

不同物理承载只是「怎么把这一串字节送上线」不同：

    承载      前缀         分包        备注
    --------  -----------  ----------  ------------------------------------------
    UDP 网口  无           不需要      一个数据报 = 一个逻辑包（**最快**，p50 1.6ms）
    串口      AA 55       不需要      与控制台文本共口；二进制帧以 AA 55 起头
    CAN 总线  无           需要        8 字节/帧，首帧 d[0]=0x80|帧数

命令码、负载、错误码、CRC 算法在三种承载上**逐字节相同** —— 所以本模块只有一份
`CMD_*` / `ErrCode` / `build_payload_*` / `decode_*`，由各承载复用。

⚠ 三个最容易错的点（都已在代码里处理，移植到别的语言时务必照做）：
  1.  CRC-16/CCITT-FALSE：poly 0x1021、init 0xFFFF、**非反射**（MSB-first）、无最终异或。
      校验向量：crc16(b"123456789") == 0x29B1
  2.  定点编码一律「四舍五入、远离零」——固件用 `(int)(v*10 + (v>=0?0.5:-0.5))`。
      Python 内建 round() 是**银行家舍入**（round(0.5)==0），会和固件差 1 个 LSB。
      本模块的 `round_half_away()` 与之对齐。
  3.  `GET_STATUS` 与 `FK_QUERY` 的负载**顺序相反、偏移 20 含义不同**（位域 vs 裸值），
      绝对**不能**共用同一个解析函数 —— 见 `decode_status` / `decode_fk`。

⚠ 两套角度量纲（本协议最容易踩的坑）：
      舵机度 servo deg : 0~360 全圈；GET_STATUS / FK_QUERY / MOVE_JOINTS / MOVE_CART 用
      逻辑角 logical    : 相对本臂 Home 零点的带符号角；**STREAM_JOINTS 用**
    换算见 §9 的 JointCalib / servo_to_logical / logical_to_servo。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Sequence, Tuple

# ============================================================================
# 1. 协议常量
# ============================================================================

APROTO_HEAD0 = 0xAA
APROTO_HEAD1 = 0x55
APROTO_HEAD = bytes([APROTO_HEAD0, APROTO_HEAD1])

MAX_PAYLOAD = 64                     # APROTO_MAX_PAYLOAD
MAX_FRAME = 5 + MAX_PAYLOAD + 2      # = 71，含前缀前的逻辑包最大 69？见下
MAX_PACKET = 3 + MAX_PAYLOAD + 2     # = 69 逻辑包（SEQ+CMD+LEN+PAYLOAD+CRC）
MAX_DATAGRAM = 3 + MAX_PAYLOAD + 2   # UDP 负载上限（不含前缀）

NUM_JOINTS = 6

# ---- 命令码（请求） ----
CMD_PING = 0x01
CMD_GET_STATUS = 0x02
CMD_LOG_SET_LEVEL = 0x03
CMD_MOVE_JOINTS = 0x10
CMD_STOP = 0x11
CMD_SET_TORQUE = 0x12
CMD_STREAM_JOINTS = 0x13
CMD_MOVE_CART = 0x20
CMD_FK_QUERY = 0x21
CMD_SET_GRAVITY = 0x30
CMD_SET_CORRECTION = 0x31
CMD_PARAM_READ = 0x40
CMD_PARAM_WRITE = 0x41
CMD_PARAM_COUNT = 0x42

# ---- 2026-09-23 扩展：让网口覆盖文本控制台能力 ----
# 设计基准 docs/网口指令扩展设计_2026-09-23.md
# 四条铁律：
#   ① 命令码必须 < 0x80（固件把 cmd>=0x80 当回包丢弃）
#   ② 单帧 payload ≤ 64 B ⇒ 大数组（示教轨迹）走【分页】
#   ③ 危险命令必须带 MAGIC，无连接通道上防误触发
#   ④ 响应码 = 请求码 | 0x80（查询类）；控制类仍回 ACK/NACK
CMD_GET_JOINT_DETAIL = 0x04
CMD_GET_LIMITS = 0x05
CMD_GET_INFO = 0x07
CMD_GET_FLAGS = 0x0A
# ⭐ 文本命令透传：一条命令把整个控制台（124 条）搬到网口。
# 为什么不是逐条加 opcode —— 逐条永远追不上"以后新增的控制台命令"，而且
# 两套语义会慢慢漂移。这条走的是固件里**同一条** ProcessCommand。
# 固件侧安全闸门自己会挡掉 11 条抢 ETH 引脚的命令（继承 ProcessCommand）
# 与交互式的 `c`（会卡死主循环）。
CMD_TEXT_EXEC = 0x0B
CMD_TEXT_FETCH = 0x0C
CMD_MOVE_JOINT_ONE = 0x14
CMD_SET_BUS_MODE = 0x17
CMD_SERVO_INIT = 0x18
CMD_IK_QUERY = 0x22
CMD_GET_GRAVITY = 0x32
CMD_GET_CORRECTION = 0x33
CMD_GET_PID = 0x36
CMD_PARAM_SAVE = 0x43
CMD_PARAM_LIST = 0x44
CMD_TEACH_LIST = 0x50
CMD_TEACH_READ = 0x51
CMD_TEACH_DELETE = 0x53
CMD_TEACH_COMPACT = 0x54
CMD_TEACH_TOTAL = 0x55
CMD_CENTER_CALIB = 0x70
CMD_CALIB_ENTER = 0x71
CMD_ZERO_RESET = 0x72
CMD_SYS_RESET = 0x73

# 危险命令的确认字。必须显式构造才命中 —— 这挡住的是"扫描器误打"
# 和"串包"，不是挡住人。见固件 APROTO_MAGIC。
MAGIC = 0x5A5A5A5A

# ---- 响应码 ----
RSP_ACK = 0x7F
RSP_NACK = 0x7E
RSP_PING = 0x81
RSP_STATUS = 0x82
RSP_FK = 0xA1
RSP_PARAM_READ = 0xC0
RSP_PARAM_COUNT = 0xC2
RSP_JOINT_DETAIL = 0x84
RSP_LIMITS = 0x85
RSP_INFO = 0x87
RSP_FLAGS = 0x8A
RSP_TEXT_OUT = 0x8B      # TEXT_EXEC 的应答（6B 分页头 + 正文）
RSP_TEXT_FETCH = 0x8C    # TEXT_FETCH 的应答（与上面同格式）
RSP_IK_QUERY = 0xA2
RSP_GRAVITY = 0xB2
RSP_CORRECTION = 0xB3
RSP_PID = 0xB6
RSP_PARAM_LIST = 0xC4
RSP_TEACH_LIST = 0xD0
RSP_TEACH_READ = 0xD1
RSP_TEACH_TOTAL = 0xD5

# ---- 文本透传尺寸常量。**必须与固件一致** --------------------------------
# 固件出处: Core/Src/arm_control_protocol.c(APROTO_TEXT_CHUNK) +
#           Core/Src/main.c(ARM_TEXT_BUF_SZ)。改了固件这里要一起改。
TEXT_BUF_SZ = 2048        # ARM_TEXT_BUF_SZ：单次捕获上限
TEXT_EXEC_MAX = 58        # 一条 TEXT_EXEC 能带的命令字符数（64 − 4B magic − 1B NUL − 1B 余量）
TEXT_CHUNK = 58           # 每页正文上限（64 − 6B 分页头）
TEXT_FLAG_TRUNC = 0x01    # 输出被 TEXT_BUF_SZ 截断（半截，别当完整）
TEXT_FLAG_GATE = 0x02     # 被固件安全闸门拒绝：正文是**原因说明**，不是命令输出

# 查询类：响应码 = 请求码 | 0x80
QUERY_RSP = {
    CMD_PING: RSP_PING, CMD_GET_STATUS: RSP_STATUS, CMD_FK_QUERY: RSP_FK,
    CMD_PARAM_READ: RSP_PARAM_READ, CMD_PARAM_COUNT: RSP_PARAM_COUNT,
    CMD_GET_JOINT_DETAIL: RSP_JOINT_DETAIL, CMD_GET_LIMITS: RSP_LIMITS,
    CMD_GET_INFO: RSP_INFO, CMD_GET_FLAGS: RSP_FLAGS, CMD_IK_QUERY: RSP_IK_QUERY,
    CMD_TEXT_EXEC: RSP_TEXT_OUT, CMD_TEXT_FETCH: RSP_TEXT_FETCH,
    CMD_GET_GRAVITY: RSP_GRAVITY, CMD_GET_CORRECTION: RSP_CORRECTION,
    CMD_GET_PID: RSP_PID, CMD_PARAM_LIST: RSP_PARAM_LIST,
    CMD_TEACH_LIST: RSP_TEACH_LIST, CMD_TEACH_READ: RSP_TEACH_READ,
    CMD_TEACH_TOTAL: RSP_TEACH_TOTAL,
}
CMD_NAME = {
    CMD_PING: "PING", CMD_GET_STATUS: "GET_STATUS", CMD_LOG_SET_LEVEL: "LOG_SET_LEVEL",
    CMD_MOVE_JOINTS: "MOVE_JOINTS", CMD_STOP: "STOP", CMD_SET_TORQUE: "SET_TORQUE",
    CMD_STREAM_JOINTS: "STREAM_JOINTS", CMD_MOVE_CART: "MOVE_CART",
    CMD_FK_QUERY: "FK_QUERY", CMD_SET_GRAVITY: "SET_GRAVITY",
    CMD_SET_CORRECTION: "SET_CORRECTION", CMD_PARAM_READ: "PARAM_READ",
    CMD_PARAM_WRITE: "PARAM_WRITE", CMD_PARAM_COUNT: "PARAM_COUNT",
    CMD_GET_JOINT_DETAIL: "GET_JOINT_DETAIL", CMD_GET_LIMITS: "GET_LIMITS",
    CMD_GET_INFO: "GET_INFO", CMD_GET_FLAGS: "GET_FLAGS",
    CMD_TEXT_EXEC: "TEXT_EXEC", CMD_TEXT_FETCH: "TEXT_FETCH",
    CMD_MOVE_JOINT_ONE: "MOVE_JOINT_ONE", CMD_SET_BUS_MODE: "SET_BUS_MODE",
    CMD_SERVO_INIT: "SERVO_INIT", CMD_IK_QUERY: "IK_QUERY",
    CMD_GET_GRAVITY: "GET_GRAVITY", CMD_GET_CORRECTION: "GET_CORRECTION",
    CMD_GET_PID: "GET_PID", CMD_PARAM_SAVE: "PARAM_SAVE",
    CMD_PARAM_LIST: "PARAM_LIST", CMD_TEACH_LIST: "TEACH_LIST",
    CMD_TEACH_READ: "TEACH_READ", CMD_TEACH_DELETE: "TEACH_DELETE",
    CMD_TEACH_COMPACT: "TEACH_COMPACT", CMD_TEACH_TOTAL: "TEACH_TOTAL",
    CMD_CENTER_CALIB: "CENTER_CALIB", CMD_CALIB_ENTER: "CALIB_ENTER",
    CMD_ZERO_RESET: "ZERO_RESET", CMD_SYS_RESET: "SYS_RESET",
    RSP_ACK: "ACK", RSP_NACK: "NACK", RSP_PING: "RSP_PING", RSP_STATUS: "RSP_STATUS",
    RSP_FK: "RSP_FK", RSP_PARAM_READ: "RSP_PARAM_READ", RSP_PARAM_COUNT: "RSP_PARAM_COUNT",
    RSP_JOINT_DETAIL: "RSP_JOINT_DETAIL", RSP_LIMITS: "RSP_LIMITS",
    RSP_INFO: "RSP_INFO", RSP_FLAGS: "RSP_FLAGS", RSP_IK_QUERY: "RSP_IK_QUERY",
    RSP_TEXT_OUT: "RSP_TEXT_OUT", RSP_TEXT_FETCH: "RSP_TEXT_FETCH",
    RSP_GRAVITY: "RSP_GRAVITY", RSP_CORRECTION: "RSP_CORRECTION",
    RSP_PID: "RSP_PID", RSP_PARAM_LIST: "RSP_PARAM_LIST",
    RSP_TEACH_LIST: "RSP_TEACH_LIST", RSP_TEACH_READ: "RSP_TEACH_READ",
    RSP_TEACH_TOTAL: "RSP_TEACH_TOTAL",
}


def cmd_name(cmd: int) -> str:
    return CMD_NAME.get(cmd, "0x%02X" % (cmd & 0xFF))


class ErrCode(IntEnum):
    """NACK 负载第 2 字节。"""
    OK = 0
    PARAM = 1        # 负载长度不足 / 未知 CMD / 非法参数 ID / 非法关节号
    CRC = 2          # 帧 CRC 校验失败（唯一"坏包也会回 NACK"的情况）
    UNREACH = 3      # IK 不收敛（仅 MOVE_CART）
    LIMIT = 4        # 超限 / 不安全（MOVE_CART 标称限位；STREAM 跳变>25°/撞桌/总线写失败）
    ESTOP = 5        # 急停激活
    FK_VERIFY = 6    # FK 回代验证超差（仅 MOVE_CART）
    READ = 7         # 舵机总线读失败（协议层实际不会以 NACK 出现）
    MAGIC = 8        # 危险命令 magic 不符（防误触发/串包）
    BUSY = 9         # 轨迹/示教/回放进行中，或命令被安全闸门拒绝
    NOTCAL = 10      # 未做 Home 零点校准，无法换算


ERR_DESC = {
    0: "成功",
    1: "参数非法（长度/范围/ID/未知命令）",
    2: "帧 CRC 校验失败",
    3: "IK 不可达 / 不收敛",
    4: "超限/不安全（跳变>25°、撞桌面/立柱、总线写失败）",
    5: "急停激活，禁止运动",
    6: "IK 解 FK 回代验证超差",
    # ⚠ 同一个码 7 被多条命令复用、语义不同：
    #   GET_STATUS/GET_JOINT_DETAIL/GET_LIMITS(运行时) → 舵机总线读失败（查线缆/共地/ID）
    #   GET_LIMITS/GET_PID 等参数类                → 读 **Flash 参数区(扇区 11)** 失败
    #     （典型原因：从未 `zr`/写过 PID ⇒ PID_FlashLoad 非 OK，**不是线缆问题**）
    #   ⇒ 这里只给中性描述，按命令的细化解释见 host_integration/上位机对接指南.md §5
    7: "读失败（舵机总线或 Flash 参数区，按命令区分）",
    8: "危险命令 magic 不符（必须显式构造 0x5A5A5A5A）",
    9: "忙/被安全闸门拒绝（轨迹或示教进行中；或该命令从网口下发不安全）",
    10: "该关节未做 Home 零点校准（先 `zi` 检查、`zr` 修复）",
}

# ---- 流水线状态（STREAM_JOINTS 的 err=4 需要按固件返回码细分） ----
STREAM_RC_OK = 0
STREAM_RC_LIMIT = 1        # 跳变>25° / 撞桌 / 撞立柱 / 总线写失败
STREAM_RC_ESTOP = 2
STREAM_RC_MASTER_ROLE = 3  # 本臂是主臂 ⇒ 拒绝直写（语义不准，实为主从角色问题）

# ---- 状态哨兵 ----
RAW_READ_FAIL = -32768

# ---- 参数表（PARAM_READ / PARAM_WRITE） ----
AP_PARAM_COUNT = 24          # 现役项数（还有 46 是表长哨兵，不连续是正常的）

# id -> (name, unit, vmin, vmax, default)
PARAM_TABLE = {
    1:  ("ik.converge_mm",      "mm",      0.1,   5.0,   0.5),
    2:  ("ik.max_iter",         "iter",    10,    500,   100),
    3:  ("ik.fail_mm",          "mm",      0.5,   10.0,  1.0),
    4:  ("ik.fk_verify_mm",     "mm",      1.0,   20.0,  5.0),
    16: ("sc.tj",               "s",       0.02,  0.50,  0.15),
    17: ("sc.def_vmax",         "deg/s",   5,     300,   25),
    18: ("sc.def_accel",        "deg/s2",  10,    600,   80),
    19: ("sc.v_min",            "deg/s",   1,     50,    5),
    20: ("sc.v_max",            "deg/s",   50,    300,   300),
    21: ("sc.accel_factor",     "ratio",   1.0,   4.0,   2.0),
    32: ("gc.k",                "deg/mm",  0.0,   0.5,   0.0),
    33: ("corr.enabled",        "0/1",     0,     1,     1),
    34: ("corr.threshold",      "deg",     0.5,   10.0,  2.0),
    35: ("corr.max_rounds",     "rounds",  1,     20,    10),
    36: ("corr.gain_j1",        "ratio",   0.0,   0.85,  0.2),
    37: ("corr.gain_j2",        "ratio",   0.0,   0.85,  0.5),
    38: ("corr.gain_j3",        "ratio",   0.0,   0.85,  0.3),
    39: ("corr.gain_j4",        "ratio",   0.0,   0.85,  0.2),
    40: ("corr.gain_j5",        "ratio",   0.0,   0.85,  0.2),
    41: ("corr.gain_j6",        "ratio",   0.0,   0.85,  0.2),
    42: ("traj.approach_speed", "1=4.39d/s", 0,   4095,  30),
    43: ("traj.approach_frac",  "ratio",   0.0,   0.25,  0.05),
    44: ("servo.accel",         "-",       0,     254,   10),
    45: ("traj.handoff_deg",    "deg",     0.0,   15.0,  0.0),
}


# ============================================================================
# 2. CRC-16/CCITT-FALSE
# ============================================================================

def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, **非反射**, 无最终异或。

    校验范围 = SEQ, CMD, LEN, PAYLOAD（即从逻辑包偏移 0 起，共 3+LEN 字节）。
    已知向量：crc16(b"123456789") == 0x29B1
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# ============================================================================
# 3. 逻辑包（三承载共用的那一段）
# ============================================================================

def pack_logical(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    """组逻辑包：SEQ CMD LEN PAYLOAD CRC_LO CRC_HI（**不含**任何前缀）。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 超长 %d > %d" % (len(payload), MAX_PAYLOAD))
    body = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + bytes(payload)
    c = crc16(body)
    return body + bytes([c & 0xFF, (c >> 8) & 0xFF])


class ProtocolError(ValueError):
    """逻辑包长度或 CRC 不符。"""


def unpack_logical(data: bytes) -> Tuple[int, int, bytes]:
    """解逻辑包（不含前缀）。返回 (seq, cmd, payload)。

    ⚠ 长度必须**恰好** 3+LEN+2：不等则抛 ProtocolError。
       固件对长度不符的包**静默丢弃**（连 NACK 都没有），只表现为超时。
    """
    data = bytes(data)
    if len(data) < 5:
        raise ProtocolError("包太短: %d" % len(data))
    seq, cmd, ln = data[0], data[1], data[2]
    if ln > MAX_PAYLOAD:
        raise ProtocolError("LEN=%d 超上限 %d" % (ln, MAX_PAYLOAD))
    if len(data) != 3 + ln + 2:
        raise ProtocolError("长度不符: 实收 %d, LEN=%d 要求 %d" % (len(data), ln, 3 + ln + 2))
    got = data[3 + ln] | (data[3 + ln + 1] << 8)
    want = crc16(data[:3 + ln])
    if got != want:
        raise ProtocolError("CRC 错: got 0x%04X want 0x%04X" % (got, want))
    return seq, cmd, data[3:3 + ln]


# ============================================================================
# 4. 定点编码（**四舍五入远离零**，与固件一致）
# ============================================================================

def round_half_away(v: float) -> int:
    """四舍五入、远离零。对齐固件 `(int)(v + (v>=0?0.5:-0.5))`。

    与 Python 的 round() 不同：round(0.5)==0 / round(2.5)==2（银行家舍入）——
    用内建 round 会和固件差 1 个 LSB。
    """
    return int(v + 0.5) if v >= 0 else -int(-v + 0.5)


def _i16(v: int) -> bytes:
    return struct.pack("<h", max(-32768, min(32767, int(v))))


def _u16(v: int) -> bytes:
    return struct.pack("<H", max(0, min(65535, int(v))))


def _i32(v: int) -> bytes:
    return struct.pack("<i", max(-2147483648, min(2147483647, int(v))))


def rd_i16(b: bytes, off: int) -> int:
    return struct.unpack_from("<h", b, off)[0]


def rd_u16(b: bytes, off: int) -> int:
    return struct.unpack_from("<H", b, off)[0]


def rd_i32(b: bytes, off: int) -> int:
    return struct.unpack_from("<i", b, off)[0]


# ============================================================================
# 5. 请求负载构造
# ============================================================================

def payload_move_joints(servo_deg: Sequence[float], v: float = 0.0, a: float = 0.0) -> bytes:
    """0x10 MOVE_JOINTS：6×i16 舵机度×10 + u16 速度×10 + u16 加速度×10  = 16 B。

    ⚠ 角度是**舵机度 0~360**（不是逻辑角）。v/a 给 0 = 用臂内默认。
    """
    if len(servo_deg) != NUM_JOINTS:
        raise ValueError("需要 6 个舵机度")
    p = b"".join(_i16(round_half_away(d * 10)) for d in servo_deg)
    return p + _u16(round_half_away(max(0.0, v) * 10)) + _u16(round_half_away(max(0.0, a) * 10))


def payload_stream_joints(logical_deg: Sequence[float]) -> bytes:
    """0x13 STREAM_JOINTS：6×i16 **逻辑角**×10 = 12 B。

    ⚠ 逻辑角 = 相对本臂 Home 零点、带符号；**不是**舵机度。
    """
    if len(logical_deg) != NUM_JOINTS:
        raise ValueError("需要 6 个逻辑角")
    return b"".join(_i16(round_half_away(d * 10)) for d in logical_deg)


def payload_move_cart(x: float, y: float, z: float, pitch: float = 0.0,
                      v: float = 0.0, a: float = 0.0) -> bytes:
    """0x20 MOVE_CART：i16 x,y,z(mm×10) + i16 pitch(deg×10) + u16 v×10 + u16 a×10 = 12 B。"""
    return (_i16(round_half_away(x * 10)) + _i16(round_half_away(y * 10))
            + _i16(round_half_away(z * 10)) + _i16(round_half_away(pitch * 10))
            + _u16(round_half_away(max(0.0, v) * 10)) + _u16(round_half_away(max(0.0, a) * 10)))


def payload_set_torque(joint: int, enable: bool) -> bytes:
    """0x12 SET_TORQUE：u8 joint(0~5 或 0xFF=全部) + u8 enable。"""
    if joint != 0xFF and not (0 <= joint <= 5):
        raise ValueError("joint 取 0~5 或 0xFF(全部)；6 是非法值")
    return bytes([joint & 0xFF, 1 if enable else 0])


def payload_set_gravity(k_deg_per_mm: float) -> bytes:
    """0x30 SET_GRAVITY：i16 K(deg/mm ×1000)。⚠ 本项目 gc.k 现为占位，不影响普通运动。"""
    return _i16(round_half_away(k_deg_per_mm * 1000))


def payload_set_correction(enable: bool, threshold_deg: float, max_rounds: int) -> bytes:
    """0x31 SET_CORRECTION：u8 en + u16 thr(deg×100) + u8 rounds。三参数一次写全。"""
    return (bytes([1 if enable else 0])
            + _u16(round_half_away(threshold_deg * 100))
            + bytes([int(max_rounds) & 0xFF]))


def payload_param_read(param_id: int) -> bytes:
    """0x40 PARAM_READ：u8 id。"""
    return bytes([param_id & 0xFF])


def payload_param_write(param_id: int, value: float) -> bytes:
    """0x41 PARAM_WRITE：u8 id + i32 value×1000。"""
    return bytes([param_id & 0xFF]) + _i32(round_half_away(value * 1000))


def payload_log_set_level(level: int) -> bytes:
    """0x03 LOG_SET_LEVEL：u8 level 0~4（>4 按 4 处理）。"""
    return bytes([max(0, min(4, int(level))) & 0xFF])


# ============================================================================
# 6. 响应负载解码
# ============================================================================

@dataclass
class PingInfo:
    """0x81 响应负载（2 字节）。"""
    proto_ver: int = 0
    run_id: int = 0

    def __str__(self) -> str:
        return "proto_ver=%d run_id=%d" % (self.proto_ver, self.run_id)


@dataclass
class Status:
    """0x82 GET_STATUS 响应负载（21 字节）。

    joints : 六轴**舵机度**；某轴读失败 = None（原始哨兵 -32768）
    x/y/z  : mm（read_ok=0 时固件提前返回，这里是 0，**不是**有效位姿）
    pitch  : deg；flags 见 estop/traj/corr/read_ok
    """
    joints: List[Optional[float]] = field(default_factory=list)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    flags: int = 0
    estop: bool = False
    traj: bool = False
    corr: bool = False
    read_ok: bool = False

    def failed_joints(self) -> List[int]:
        """返回读失败轴的 1 基编号，如 [3]。"""
        return [i + 1 for i, v in enumerate(self.joints) if v is None]

    def as_servo_deg(self) -> Optional[List[float]]:
        """读全部成功才返回 6 个舵机度，否则 None（**不做兜底**）。"""
        if not self.read_ok or any(v is None for v in self.joints):
            return None
        return [float(v) for v in self.joints]

    def pose(self):
        """read_ok 且读数齐全才返回 (x, y, z, pitch)，否则 None。"""
        if not self.read_ok or self.joints and any(v is None for v in self.joints):
            return None
        return (self.x, self.y, self.z, self.pitch)


@dataclass
class FkResult:
    """0xA1 FK_QUERY 响应负载（21 字节）。

    ⚠ 与 Status **顺序相反**（先位姿后关节），且偏移 20 是**裸 read_ok(0/1)**，不是位域。
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    joints: List[Optional[float]] = field(default_factory=list)
    read_ok: int = 0

    def as_servo_deg(self) -> Optional[List[float]]:
        if self.read_ok != 1 or any(v is None for v in self.joints):
            return None
        return [float(v) for v in self.joints]


def decode_status(pl: bytes) -> Status:
    """解 0x82 负载（21 B）。顺序：6×i16 关节(舵机度) → i16 X/Y/Z/Pitch → u8 flags 位域。"""
    if len(pl) < 21:
        raise ProtocolError("GET_STATUS 负载应 21 字节, 实收 %d" % len(pl))
    st = Status()
    for j in range(NUM_JOINTS):
        v = rd_i16(pl, j * 2)
        st.joints.append(None if v == RAW_READ_FAIL else v / 10.0)
    st.x = rd_i16(pl, 12) / 10.0
    st.y = rd_i16(pl, 14) / 10.0
    st.z = rd_i16(pl, 16) / 10.0
    st.pitch = rd_i16(pl, 18) / 10.0
    st.flags = pl[20]
    st.estop = bool(st.flags & 0x01)
    st.traj = bool(st.flags & 0x02)
    st.corr = bool(st.flags & 0x04)
    st.read_ok = bool(st.flags & 0x08)
    return st


def decode_fk(pl: bytes) -> FkResult:
    """解 0xA1 负载（21 B）。顺序：i16 X/Y/Z/Pitch → 6×i16 关节(舵机度) → u8 read_ok(裸值)。"""
    if len(pl) < 21:
        raise ProtocolError("FK_QUERY 负载应 21 字节, 实收 %d" % len(pl))
    r = FkResult()
    r.x = rd_i16(pl, 0) / 10.0
    r.y = rd_i16(pl, 2) / 10.0
    r.z = rd_i16(pl, 4) / 10.0
    r.pitch = rd_i16(pl, 6) / 10.0
    for j in range(NUM_JOINTS):
        v = rd_i16(pl, 8 + j * 2)
        r.joints.append(None if v == RAW_READ_FAIL else v / 10.0)
    r.read_ok = pl[20]
    return r


def decode_param_read(pl: bytes) -> Tuple[int, float]:
    """解 0xC0 负载（5 B）。返回 (id, value)。"""
    if len(pl) < 5:
        raise ProtocolError("PARAM_READ 负载应 5 字节, 实收 %d" % len(pl))
    return pl[0], rd_i32(pl, 1) / 1000.0


def decode_param_count(pl: bytes) -> int:
    if len(pl) < 1:
        raise ProtocolError("PARAM_COUNT 负载应 1 字节")
    return pl[0]


# ============================================================================
# 7. 角度体系（两套量纲之间的换算）
# ============================================================================

DEFAULT_SERVO_OFFSET = (180.0, 180.0, 180.0, 180.0, 180.0, 180.0)
JOINT_DIR = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0)     # 仅 J1 反向


def wrap180(a: float) -> float:
    """把角度归一化到 (-180, 180]。"""
    a = (a + 180.0) % 360.0
    if a <= 0:
        a += 360.0
    return a - 180.0


@dataclass
class JointCalib:
    """单台臂的关节标定：逻辑角 ⇄ 舵机度。

    offset : 每个关节"运动学零点(官方 Home)"对应的**舵机度**，来自 `c` 校准 Phase 1
             （出厂默认 180.0；用 `zi` 文本命令可读出本机值）
    dir    : JOINT_DIR，仅 J1 反向（默认 [-1,1,1,1,1,1]）

    换算（与 kinematics.c / main.c 一致）：
        舵机度 = (offset[j] + dir[j] * 逻辑角) mod 360
        逻辑角 = dir[j] * (舵机度 - offset[j])，归一到 (-180, 180]
    """
    offset: List[float] = field(default_factory=lambda: list(DEFAULT_SERVO_OFFSET))
    dir: List[float] = field(default_factory=lambda: list(JOINT_DIR))
    source: str = "default(180.0, 未校准)"

    def __post_init__(self):
        if len(self.offset) != NUM_JOINTS or len(self.dir) != NUM_JOINTS:
            raise ValueError("offset/dir 必须各 6 个")

    def to_logical(self, servo_deg: Sequence[float]) -> List[float]:
        """舵机度 → 逻辑角（STREAM_JOINTS 用）。"""
        return [wrap180(self.dir[j] * (servo_deg[j] - self.offset[j]))
                for j in range(NUM_JOINTS)]

    def to_servo(self, logical_deg: Sequence[float]) -> List[float]:
        """逻辑角 → 舵机度（MOVE_JOINTS 用）。"""
        return [(self.offset[j] + self.dir[j] * logical_deg[j]) % 360.0
                for j in range(NUM_JOINTS)]

    @property
    def is_default(self) -> bool:
        return self.source.startswith("default")

    def describe(self) -> str:
        return "offset=%s dir=%s (%s)" % (
            "[" + ", ".join("%.1f" % v for v in self.offset) + "]",
            "[" + ",".join("%+d" % d for d in self.dir) + "]",
            self.source)


def parse_servo_offset_from_zi(text: str) -> Optional[List[float]]:
    """从串口 `zi` 的输出里抽出 6 个零点 raw，转成舵机度。

    `zi` 打印形如：
        [ZERO] J1=2048 J2=2048 ...   （或 Home 零点已记录: 2014 896 3106 2331 2022 1567）
    这里接受 6 个 0~4095 的整数（可带逗号/空格分隔），按 raw→舵机度换算。
    """
    import re
    nums = [int(x) for x in re.findall(r"\b(\d{2,4})\b", text)
            if 0 <= int(x) <= 4095]
    # 取最后 6 个（前面可能有 J1/J2 之类的噪声数字）
    cand = nums[-6:]
    if len(cand) != 6:
        return None
    return [round(v / 4095.0 * 360.0, 4) for v in cand]


# ============================================================================
# 8. 承载（Carrier）
# ============================================================================

class Carrier:
    """承载抽象：把逻辑包变成线上字节，以及把线上字节还原成逻辑包。"""

    name = "abstract"

    def build(self, seq: int, cmd: int, payload: bytes = b"") -> bytes:
        raise NotImplementedError

    def parse(self, data: bytes) -> Tuple[int, int, bytes]:
        raise NotImplementedError


class UdpCarrier(Carrier):
    """UDP 网口承载：**没有帧头**，一个数据报 = 一个逻辑包。

    ⚠ 目的地必须 `ip:5000`；报文长度必须恰好 5+LEN，否则固件静默丢弃。
    ⚠ 回包发往**请求的源 IP:源端口**，所以本端不必绑固定端口。
    """
    name = "udp"

    def build(self, seq: int, cmd: int, payload: bytes = b"") -> bytes:
        return pack_logical(seq, cmd, payload)

    def parse(self, data: bytes) -> Tuple[int, int, bytes]:
        return unpack_logical(data)


class SerialCarrier(Carrier):
    """串口承载（二进制）：逻辑包前面加 `AA 55`。

    ⚠ 与控制台**文本**共口：0xAA 开头的字节被固件协议状态机吃掉。
       所以**不要**发以 0xAA 开头的文本命令。
    """
    name = "serial"
    HEAD = APROTO_HEAD

    def build(self, seq: int, cmd: int, payload: bytes = b"") -> bytes:
        return self.HEAD + pack_logical(seq, cmd, payload)

    def parse(self, data: bytes) -> Tuple[int, int, bytes]:
        data = bytes(data)
        if data[:2] != self.HEAD:
            raise ProtocolError("缺 AA 55 帧头")
        return unpack_logical(data[2:])


# ============================================================================
# 9. CAN 承载（分包 / 重组）
# ============================================================================

def can_plan_frames(payload_len: int) -> Tuple[int, int]:
    """返回 (总帧数, 首帧携带的 (PAYLOAD+CRC) 字节数)。与固件 can_send_packed 同公式。"""
    tail = payload_len + 2                     # PAYLOAD + CRC
    first_n = min(tail, 4)
    rest = max(tail - 4, 0)
    return 1 + (rest + 6) // 7, first_n


def can_build_frames(seq: int, cmd: int, payload: bytes = b"") -> List[bytes]:
    """把一条逻辑包切成 CAN 数据场列表（DLC 可变，不补 8）。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 超长")
    logical = pack_logical(seq, cmd, payload)
    frames, first_n = can_plan_frames(len(payload))
    out = [bytes([0x80 | frames, logical[0], logical[1], logical[2]]) + logical[3:3 + first_n]]
    off = 3 + first_n
    for f in range(1, frames):
        chunk = logical[off:off + 7]
        out.append(bytes([f]) + chunk)
        off += len(chunk)
    return out


class CanReassembler:
    """把 CAN 帧还原成逻辑包。

    ⚠ bxCAN 只有 3 个发送邮箱，**续帧会乱序到达**（硬件行为，不是丢帧）。
      实测到达顺序可能是 0,3,1,2。所以这里**按帧号缓冲**，收齐再拼；严格递增
      的老式判据会误判成"整包乱序"。
    """

    def __init__(self, strict: bool = False):
        self.strict = strict
        self.reset()

    def reset(self):
        self.active = False
        self.frames = 0
        self.next = 0
        self.head = b""
        self.hdr = b""
        self.parts = {}

    def feed(self, data: bytes):
        """喂一帧，收齐则返回 (seq, cmd, payload, crc_ok)，否则 None。"""
        data = bytes(data)
        if not data:
            return None
        if data[0] & 0x80:                                  # 首帧
            if len(data) < 4:
                return None
            frames = data[0] & 0x0F
            ln = data[3]
            if frames == 0 or ln > MAX_PAYLOAD:
                self.reset()
                return None
            want, first_n = can_plan_frames(ln)
            if frames != want:
                self.reset()
                return None
            self.active, self.frames, self.next = True, frames, 1
            self.hdr = bytes([data[1], data[2], data[3]])
            self.head = data[4:4 + first_n]
            self.parts = {}
        elif self.active and 1 <= data[0] < self.frames:
            if self.strict and data[0] != self.next:
                self.reset()
                return None
            self.parts.setdefault(data[0], data[1:])
            self.next = max(self.next, data[0] + 1)
        else:
            self.reset()
            return None

        if self.active and len(self.parts) >= self.frames - 1:
            body = self.head + b"".join(self.parts[i] for i in range(1, self.frames))
            seq, cmd, ln = self.hdr[0], self.hdr[1], self.hdr[2]
            if len(body) >= ln + 2:
                got = body[ln] | (body[ln + 1] << 8)
                ok = (got == crc16(self.hdr + body[:ln]))
                self.reset()
                return (seq, cmd, body[:ln], ok)
        return None


class CanCarrier(Carrier):
    """CAN 承载的门面（build 返回帧列表；parse 需外部维护 CanReassembler）。"""
    name = "can"

    def __init__(self):
        self.reasm = CanReassembler()

    def build(self, seq: int, cmd: int, payload: bytes = b"") -> List[bytes]:  # type: ignore[override]
        return can_build_frames(seq, cmd, payload)

    def parse(self, data: bytes):                                             # type: ignore[override]
        return self.reasm.feed(data)


# ============================================================================
# 10. 串口混合字节流解析（文本 + AA55 二进制）
# ============================================================================

class FrameStreamParser:
    """从混合字节流里挑出二进制帧；非 0xAA 的字节原样交还文本通道。

    与固件 `Aproto_ParserFeed` 对称。喂单个字节，返回：
        None                  —— 已吃进（在帧中）
        ("text", byte)        —— 这个字节属于文本
        ("frame", (seq,cmd,payload))
        ("crc_err", (seq,cmd,payload))
        ("drop", None)        —— 帧头后的杂散字节，丢弃
    """

    def __init__(self, interbyte_timeout_s: float = 0.2):
        self._buf = bytearray()
        self._timeout = interbyte_timeout_s
        self._last = 0.0

    def feed(self, byte: int, now: Optional[float] = None):
        import time as _t
        now = _t.monotonic() if now is None else now
        if self._buf and (now - self._last) > self._timeout:
            self._buf.clear()
        b = self._buf
        b.append(byte & 0xFF)
        n = len(b)
        if n == 1:
            if b[0] != APROTO_HEAD0:
                b.clear()
                return ("text", byte & 0xFF)
            self._last = now
            return None
        if n == 2:
            if b[1] == APROTO_HEAD1:
                self._last = now
                return None
            if b[1] == APROTO_HEAD0:
                del b[0]
                return None
            b.clear()
            return ("drop", None)
        self._last = now
        if n <= 4:
            return None
        if n == 5:
            if b[4] > MAX_PAYLOAD:
                b.clear()
                return ("drop", None)
            return None
        need = 5 + b[4] + 2
        if n < need:
            return None
        seq, cmd, ln = b[2], b[3], b[4]
        payload = bytes(b[5:5 + ln])
        got = b[5 + ln] | (b[5 + ln + 1] << 8)
        want = crc16(bytes(b[2:5 + ln]))
        b.clear()
        if got != want:
            return ("crc_err", (seq, cmd, payload))
        return ("frame", (seq, cmd, payload))


# ============================================================================
# 11. 离线自检
# ============================================================================

# ============================================================================
# 10. 2026-09-23 扩展：网口全能力命令
#     设计基准 docs/网口指令扩展设计_2026-09-23.md
#
# 为什么单独成节：这批命令的定位与 §1~§9 那批「运动学核心」不同 ——
# 它们大多是**只读诊断**与**存储管理**，目标是让网口覆盖文本控制台
# 能做的事（温度/负载/限位/示教文件读取等），上位机做 UI 时才用得上。
#
# ⚠ 分页命令（TEACH_READ / PARAM_LIST）判结束要用 total 或
#   「返回条数 < 请求条数」。**不要**用「收到空响应」判 —— 越界时固件
#   回的是 n=0 的正常响应，不是错误。
# ============================================================================

DETAIL_SIZE = 10          # 每轴 10 B
TEACH_SAMPLE_SIZE = 12    # 每样本 6×u16 raw
TEACH_READ_MAX = 5        # 单帧最多 5 个样本（5×12 = 60 ≤ 64）
PARAM_LIST_MAX = 6        # 单帧最多 6 条元数据（2 + 6×10 = 62 ≤ 64）
TEMP_LOST = 0xFF          # 温度丢读哨兵（不是 0 度）
HWERR_LOST = 0xFF         # 硬件错误读失败哨兵（不是 0 = 正常）
BUILD_NAME = {0: "net", 1: "can", 2: "both", 3: "uart"}


# ---- 请求负载 ----

def payload_move_joint_one(joint: int, servo_deg: float,
                           v: float = 0.0, a: float = 0.0) -> bytes:
    """0x14 MOVE_JOINT_ONE：u8 joint(0~5) + i16 deg×10 + u16 v×10 + u16 a×10 = 7 B。

    ⚠ 角度是**舵机度**，与 MOVE_JOINTS 同量纲（不是 STREAM_JOINTS 的逻辑角）。
    ⚠ 固件会先读全场快照、只改这一轴目标 —— 任一轴丢读则整条命令回 ERR_READ
      （拿不准位置时不动，比动错强）。
    """
    if not (0 <= joint <= 5):
        raise ValueError("joint 取 0~5")
    return (bytes([joint & 0xFF]) + _i16(round_half_away(servo_deg * 10))
            + _u16(round_half_away(max(0.0, v) * 10))
            + _u16(round_half_away(max(0.0, a) * 10)))


def payload_get_joint_detail(joint: int = 0xFF) -> bytes:
    """0x04 GET_JOINT_DETAIL：u8 joint（0~5，或 0xFF = 六轴）。"""
    if joint != 0xFF and not (0 <= joint <= 5):
        raise ValueError("joint 取 0~5 或 0xFF(全部)")
    return bytes([joint & 0xFF])


def payload_ik_query(x: float, y: float, z: float, pitch: float = 0.0) -> bytes:
    """0x22 IK_QUERY：i16 x,y,z(mm×10) + i16 pitch(deg×10) = 8 B。

    只解不算：固件做 IK 解算 + 关节限位检查 + FK 回代验证，但**不启动运动**。
    用于「先问能不能到，再决定动不动」—— 不可达/超限位/FK 超差分别回
    ERR_UNREACH(3) / ERR_LIMIT(4) / ERR_FK_VERIFY(6)。
    """
    return (_i16(round_half_away(x * 10)) + _i16(round_half_away(y * 10))
            + _i16(round_half_away(z * 10)) + _i16(round_half_away(pitch * 10)))


def payload_get_pid(joint: int) -> bytes:
    """0x36 GET_PID：u8 joint（0~5）。"""
    if not (0 <= joint <= 5):
        raise ValueError("joint 取 0~5")
    return bytes([joint & 0xFF])


def payload_set_bus_mode(mode: int) -> bytes:
    """0x17 SET_BUS_MODE：u8 mode（0=正常 A:AABBB / 1=全A / 2=全B）。

    ⚠ 会改变 SetPositionsSync 的分组方式，调试用，别在生产路径里随手调。
    """
    if mode not in (0, 1, 2):
        raise ValueError("mode 取 0/1/2")
    return bytes([mode])


def payload_servo_init(joint: int = 0xFF) -> bytes:
    """0x18 SERVO_INIT：u8 joint（0~5 或 0xFF=全部）。

    ⚠ 会重写该轴 PID/限位/中位并 RESET，副作用与文本命令 `i` 相同。
    """
    if joint != 0xFF and not (0 <= joint <= 5):
        raise ValueError("joint 取 0~5 或 0xFF(全部)")
    return bytes([joint & 0xFF])


def payload_param_list(offset: int = 0) -> bytes:
    """0x44 PARAM_LIST：u8 offset（从第几条开始）。只给元数据，值用 PARAM_READ 取。"""
    return bytes([offset & 0xFF])


def payload_teach_list(idx: int) -> bytes:
    """0x50 TEACH_LIST：u8 idx（第几个文件，0 基）。"""
    return bytes([idx & 0xFF])


def payload_teach_read(idx: int, off: int, n: int = TEACH_READ_MAX) -> bytes:
    """0x51 TEACH_READ：u8 idx + u16 off + u8 n = 4 B。

    ⚠ n 会被固件钳到 5（5×12 B = 60 ≤ 64 B 上限）。
    """
    if n < 1:
        raise ValueError("n >= 1")
    return bytes([idx & 0xFF]) + _u16(off & 0xFFFF) + bytes([min(n, TEACH_READ_MAX)])


def payload_teach_delete(idx: int) -> bytes:
    """0x53 TEACH_DELETE：u8 idx（写 tombstone，空间用 TEACH_COMPACT 回收）。"""
    return bytes([idx & 0xFF])


def payload_magic() -> bytes:
    """危险命令的 4 字节 MAGIC 负载。"""
    return _i32(MAGIC)


def payload_text_exec(text: str) -> bytes:
    """0x0B TEXT_EXEC：u32 magic + 文本命令 + NUL。

    这是"所有指令都支持网口下发"的**那一把钥匙**：负载里就是一行 ASCII，
    固件走 COM11 上完全相同的 ProcessCommand，输出改道到内存再回传。

    ⚠ 最长 58 个字符（TEXT_EXEC_MAX）。超长固件会钳断 —— 与其静默执行半条
       命令（可能是完全不同的命令！），不如在这里就报错。
    ⚠ 结尾的 CR/LF 会被剥掉；ProcessCommand 只吃一行内容。
    """
    if not isinstance(text, str):
        raise TypeError("text 必须是 str")
    body = text.replace("\r", " ").replace("\n", " ").strip()
    if not body:
        raise ValueError("命令不能为空")
    if len(body) > TEXT_EXEC_MAX:
        raise ValueError("命令最长 %d 字符，实际 %d：%r"
                         % (TEXT_EXEC_MAX, len(body), body))
    raw = body.encode("ascii")     # 固件控制台是 ASCII；非 ASCII 直接报错
    return _i32(MAGIC) + raw + b"\x00"


def payload_text_fetch(off: int, n: int = TEXT_CHUNK) -> bytes:
    """0x0C TEXT_FETCH：u16 off + u8 n = 3 B。取上一条 TEXT_EXEC 输出的后续页。"""
    if n < 1:
        raise ValueError("n >= 1")
    if not (0 <= off <= 0xFFFF):
        raise ValueError("off 取 0~65535")
    return _u16(off) + bytes([min(n, TEXT_CHUNK)])


def payload_zero_reset(save: bool = False) -> bytes:
    """0x72 ZERO_RESET：u8 flags（bit0=1 同时落盘）+ u32 magic。

    ⚠ 文本 `zr` **没有单轴变体**（一次复位全部六轴），所以这里也没有关节号 ——
       不要按"每关节重置"来用。
    ⚠ 会把 Home 零点重置回出厂中位（SERVO_OFFSET=180），量纲基准随之改变。
       save=False 仅本次运行有效；save=True 等价 `zr save`（写 Flash 固化）。
    """
    return bytes([1 if save else 0]) + _i32(MAGIC)


# ---- 响应解码 ----

@dataclass
class JointDetail:
    """单轴遥测。ok=False 时除 temp/hw_err（哨兵）外字段均无意义。"""
    joint: int
    pos_raw: int
    speed: int
    load: int
    voltage: float
    temp: int
    hw_err: int
    ok: bool

    @property
    def temp_lost(self) -> bool:
        """True = 温度这一拍丢读。总线是【逐字段瞬时丢读】的（约 20%），
        不代表故障 —— 下一拍可能就有了。"""
        return self.temp == TEMP_LOST

    @property
    def hw_err_lost(self) -> bool:
        """True = 硬件错误寄存器读失败（≠ 硬件真的报错）。"""
        return self.hw_err == HWERR_LOST


def decode_joint_detail(pl: bytes) -> List[JointDetail]:
    """解码 0x84：u8 n + u8 joint0 + n×10 B。

    每轴 10 B 布局：u16 pos_raw, i16 speed, i16 load, u8 volt(0.1V),
    u8 temp, u8 hw_err, u8 ok。单轴查询与六轴查询**同一份解包代码**。
    """
    if len(pl) < 2:
        raise ProtocolError("JOINT_DETAIL 负载过短: %d B" % len(pl))
    n = pl[0]
    j0 = pl[1]
    need = 2 + n * DETAIL_SIZE
    if len(pl) < need:
        raise ProtocolError("JOINT_DETAIL 需要 %d B，实际 %d B" % (need, len(pl)))
    out = []
    for k in range(n):
        o = 2 + k * DETAIL_SIZE
        out.append(JointDetail(
            joint=(j0 + k) if n > 1 else j0,
            pos_raw=rd_u16(pl, o),
            speed=rd_i16(pl, o + 2),
            load=rd_i16(pl, o + 4),
            voltage=pl[o + 6] / 10.0,
            temp=pl[o + 7],
            hw_err=pl[o + 8],
            ok=pl[o + 9] != 0,
        ))
    return out


@dataclass
class ArmLimits:
    min_raw: List[int]
    pos_raw: List[int]
    max_raw: List[int]
    zero_raw: List[int]

    @property
    def calibrated(self) -> List[bool]:
        """逐轴：zero_raw != 0 即做过 Home 零点校准（0 = 固件回落 180°/raw 2048）。"""
        return [z != 0 for z in self.zero_raw]


def decode_limits(pl: bytes) -> ArmLimits:
    """解码 0x85：6 × 8 B（min, pos, max, zero，均 u16 raw）= 48 B。

    ⚠ 这 4 组 raw 来自 Flash 校准记录；**运行时限位**（舵机度）不在这条命令里。
    """
    if len(pl) < 48:
        raise ProtocolError("LIMITS 需要 48 B，实际 %d B" % len(pl))
    mn, ps, mx, zr = [], [], [], []
    for k in range(6):
        o = k * 8
        mn.append(rd_u16(pl, o))
        ps.append(rd_u16(pl, o + 2))
        mx.append(rd_u16(pl, o + 4))
        zr.append(rd_u16(pl, o + 6))
    return ArmLimits(mn, ps, mx, zr)


@dataclass
class ArmInfo:
    fw_tag: str
    build: int
    link_mode: int
    n_params: int

    @property
    def build_name(self) -> str:
        return BUILD_NAME.get(self.build, "?")


def decode_info(pl: bytes) -> ArmInfo:
    """解码 0x87：u8 tag_len + char[16] + u8 build + u8 link + u8 n_params = 20 B。

    link_mode 是**启动冻结值**（LinkCfg_ActiveMode），不是 Flash 里的偏好 ——
    两者可能在「改了偏好但还没 RESET」时分叉。
    """
    if len(pl) < 20:
        raise ProtocolError("INFO 需要 20 B，实际 %d B" % len(pl))
    tl = min(pl[0], 16)
    tag = pl[1:1 + tl].decode("ascii", "replace")
    return ArmInfo(tag, pl[17], pl[18], pl[19])


@dataclass
class ArmFlags:
    """运行状态位图。只读几个变量，**不碰舵机总线** ⇒ 可以 10~50Hz 轮询。"""
    raw: int
    run_id: int

    @property
    def estop(self) -> bool:      return bool(self.raw & 0x01)
    @property
    def traj(self) -> bool:       return bool(self.raw & 0x02)
    @property
    def stream(self) -> bool:     return bool(self.raw & 0x04)
    @property
    def corr(self) -> bool:       return bool(self.raw & 0x08)
    @property
    def teach_rec(self) -> bool:  return bool(self.raw & 0x10)
    @property
    def teach_play(self) -> bool: return bool(self.raw & 0x20)


def decode_flags(pl: bytes) -> ArmFlags:
    """解码 0x8A：u8 flags + u8 run_id = 2 B。"""
    if len(pl) < 2:
        raise ProtocolError("FLAGS 需要 2 B，实际 %d B" % len(pl))
    return ArmFlags(pl[0], pl[1])


def decode_ik_query(pl: bytes) -> List[float]:
    """解码 0xA2：6 × i16 舵机度×10 = 12 B。"""
    if len(pl) < 12:
        raise ProtocolError("IK_QUERY 需要 12 B，实际 %d B" % len(pl))
    return [rd_i16(pl, k * 2) / 10.0 for k in range(6)]


def decode_gravity(pl: bytes) -> float:
    """解码 0xB2：i16 K（deg/mm ×1000）= 2 B。"""
    if len(pl) < 2:
        raise ProtocolError("GRAVITY 需要 2 B，实际 %d B" % len(pl))
    return rd_i16(pl, 0) / 1000.0


@dataclass
class CorrectionCfg:
    enabled: bool
    threshold_deg: float
    max_rounds: int
    pending: bool


def decode_correction(pl: bytes) -> CorrectionCfg:
    """解码 0xB3：u8 en + u16 thr×100 + u8 rounds + u8 pending = 5 B。"""
    if len(pl) < 5:
        raise ProtocolError("CORRECTION 需要 5 B，实际 %d B" % len(pl))
    return CorrectionCfg(pl[0] != 0, rd_u16(pl, 1) / 100.0, pl[3], pl[4] != 0)


@dataclass
class PidTriple:
    p: int
    i: int
    d: int
    from_eeprom: bool


def decode_pid(pl: bytes) -> PidTriple:
    """解码 0xB6：u8 p + u8 i + u8 d + u8 from_eeprom = 4 B。

    ⚠ 固件底层 SERVO_ReadPidEeprom 的形参顺序是 (p, d, i)，API 层已换算成
      协议顺序 p/i/d —— 到这里就是 p/i/d，不要再换一次（换两次等于没换）。
    from_eeprom=False 表示读取失败，p/i/d 不可信。
    """
    if len(pl) < 4:
        raise ProtocolError("PID 需要 4 B，实际 %d B" % len(pl))
    return PidTriple(pl[0], pl[1], pl[2], pl[3] != 0)


@dataclass
class ParamMeta:
    id: int
    type: int
    vmin: float
    vmax: float


def decode_param_list(pl: bytes) -> Tuple[int, List[ParamMeta]]:
    """解码 0xC4：u8 total + u8 n + n × 10 B = 最多 62 B。

    返回 (total, 本页条目)。翻页终止条件：(offset + n) >= total 或 n == 0。
    """
    if len(pl) < 2:
        raise ProtocolError("PARAM_LIST 负载过短: %d B" % len(pl))
    total = pl[0]
    n = pl[1]
    need = 2 + n * 10
    if len(pl) < need:
        raise ProtocolError("PARAM_LIST 需要 %d B，实际 %d B" % (need, len(pl)))
    out = []
    for k in range(n):
        o = 2 + k * 10
        out.append(ParamMeta(pl[o], pl[o + 1],
                             rd_i32(pl, o + 2) / 1000.0,
                             rd_i32(pl, o + 6) / 1000.0))
    return total, out


@dataclass
class TeachFile:
    idx: int
    name: str
    samples: int
    valid: bool


def decode_teach_list(pl: bytes) -> TeachFile:
    """解码 0xD0：u8 total + u8 idx + u8 valid + char[12] + u32 samples = 19 B。"""
    if len(pl) < 19:
        raise ProtocolError("TEACH_LIST 需要 19 B，实际 %d B" % len(pl))
    name = pl[3:15].split(b"\x00", 1)[0].decode("ascii", "replace")
    return TeachFile(pl[1], name, int.from_bytes(pl[15:19], "little"), pl[2] != 0)


def decode_teach_read(pl: bytes) -> List[List[int]]:
    """解码 0xD1：u8 n + n × 12 B（6×u16 raw）。

    返回 n 个样本，每个 6 个 raw 值。
    ⚠ n=0 表示 off 越界 —— 那是**正常读完**，不是错误。
    """
    if len(pl) < 1:
        raise ProtocolError("TEACH_READ 负载为空")
    n = pl[0]
    need = 1 + n * TEACH_SAMPLE_SIZE
    if len(pl) < need:
        raise ProtocolError("TEACH_READ 需要 %d B，实际 %d B" % (need, len(pl)))
    out = []
    for k in range(n):
        o = 1 + k * TEACH_SAMPLE_SIZE
        out.append([rd_u16(pl, o + j * 2) for j in range(6)])
    return out


@dataclass
class TeachTotals:
    n_files: int
    capacity: int
    total_samples: int
    free_bytes: int


def decode_teach_total(pl: bytes) -> TeachTotals:
    """解码 0xD5：u8 nfiles + u8 capacity + u32 total + u32 free = 10 B。"""
    if len(pl) < 10:
        raise ProtocolError("TEACH_TOTAL 需要 10 B，实际 %d B" % len(pl))
    return TeachTotals(pl[0], pl[1],
                       int.from_bytes(pl[2:6], "little"),
                       int.from_bytes(pl[6:10], "little"))


@dataclass
class TextOut:
    """0x8B / 0x8C：文本透传的一页输出。

    负载：u16 total, u8 flags, u16 off, u8 n, char text[n]
    ⚠ `raw` 是**字节**，不是 str —— 跨页拼接必须按字节拼（否则一页正好切在
      多字节字符中间时，两页各自 decode 会拼出一个坏字符）。要文本用 `.text`。
    """
    raw: bytes
    total: int          # 固件缓冲里的**总**字节数（不是本页长度）
    off: int            # 本页在总输出里的字节偏移
    truncated: bool     # flags bit0：缓冲满 2048 B，后面还有内容被丢了
    gated: bool         # flags bit1：命令被安全闸门拒绝，raw 是**原因说明**

    @property
    def text(self) -> str:
        """按 UTF-8 解码（errors='replace'）。

        固件控制台里的中文提示是 UTF-8 字节（源码就是 UTF-8）；也允许出现
        非法/半截序列 —— 用 replace 永不抛异常。要严格解码就自己用 self.raw。
        """
        return self.raw.decode("utf-8", "replace")

    @property
    def done(self) -> bool:
        """本页是否已经是最后一页。"""
        return self.off + len(self.raw) >= self.total

    def __str__(self) -> str:      # 便于 REPL/日志里直接看
        return self.text


def decode_text_out(pl: bytes) -> TextOut:
    """解码 0x8B / 0x8C：u16 total, u8 flags, u16 off, u8 n, text[n]。"""
    if len(pl) < 6:
        raise ProtocolError("TEXT_OUT 负载过短: %d B" % len(pl))
    total = rd_u16(pl, 0)
    flags = pl[2]
    off = rd_u16(pl, 3)
    n = pl[5]
    if len(pl) < 6 + n:
        raise ProtocolError("TEXT_OUT 需要 %d B，实际 %d B" % (6 + n, len(pl)))
    return TextOut(bytes(pl[6:6 + n]), total, off,
                   bool(flags & TEXT_FLAG_TRUNC),
                   bool(flags & TEXT_FLAG_GATE))


def _selftest() -> int:
    fails = 0

    def chk(name, cond, extra=""):
        nonlocal fails
        print("  %s %s%s" % ("PASS" if cond else "FAIL", name, ("  " + extra) if extra else ""))
        if not cond:
            fails += 1

    print("== CRC-16/CCITT-FALSE 向量 ==")
    chk("crc16('123456789') == 0x29B1", crc16(b"123456789") == 0x29B1,
        "实际 0x%04X" % crc16(b"123456789"))

    print("== 四舍五入远离零（与固件一致） ==")
    chk("round_half_away(0.5)==1", round_half_away(0.5) == 1)
    chk("round_half_away(2.5)==3", round_half_away(2.5) == 3)
    chk("round_half_away(-0.5)==-1", round_half_away(-0.5) == -1)
    chk("round_half_away(182.3*10)==1823", round_half_away(182.3 * 10) == 1823)

    print("== 逻辑包 往返 ==")
    for seq, cmd, payload in ((0x00, CMD_PING, b""),
                              (0x01, CMD_GET_STATUS, b""),
                              (0x02, CMD_MOVE_JOINTS, bytes(range(16))),
                              (0x03, CMD_STREAM_JOINTS, bytes(12))):
        pkt = pack_logical(seq, cmd, payload)
        s, c, p = unpack_logical(pkt)
        chk("seq=%d cmd=0x%02X len=%d" % (seq, cmd, len(payload)),
            s == seq and c == cmd and p == payload)

    print("== 网口承载无帧头（SEQ 在偏移 0） ==")
    udp = UdpCarrier()
    d = udp.build(0x31, CMD_PING)
    chk("udp 报文首字节是 SEQ", d[0] == 0x31, "hex=%s" % d.hex(" "))
    chk("udp 报文长度 = 5", len(d) == 5)

    print("== 串口承载有 AA55 帧头 ==")
    ser = SerialCarrier()
    s = ser.build(0x07, CMD_PING)
    chk("serial 首两字节 AA 55", s[:2] == b"\xAA\x55", "hex=%s" % s.hex(" "))
    chk("serial 往返", ser.parse(s) == (0x07, CMD_PING, b""))

    print("== 网口报文与文档示例逐字节一致 ==")
    # 文档 §5.1: 请求 SEQ=0x31 CMD=0x01 LEN=0 -> 31 01 00 38 0D
    chk("PING seq=0x31 => 31 01 00 38 0D", udp.build(0x31, CMD_PING).hex(" ").upper()
        == "31 01 00 38 0D",
        "实际 %s" % udp.build(0x31, CMD_PING).hex(" ").upper())
    # 文档 §5.14: PARAM_COUNT seq=0x43 -> 43 42 00 CF E3
    chk("PARAM_COUNT seq=0x43 => 43 42 00 CF E3", udp.build(0x43, CMD_PARAM_COUNT).hex(" ").upper()
        == "43 42 00 CF E3",
        "实际 %s" % udp.build(0x43, CMD_PARAM_COUNT).hex(" ").upper())

    print("== 负载构造：MOVE_JOINTS 文档示例 ==")
    # 文档 §5.4: 180.0×6, v=10, a=20 -> 08 07 ×6 + 64 00 + C8 00
    pl = payload_move_joints([180.0] * 6, 10.0, 20.0)
    chk("MOVE_JOINTS 负载 = 16 B", len(pl) == 16, "实际 %d" % len(pl))
    chk("J1..J6 = 08 07 ×6", pl[:12].hex(" ").upper() == " ".join(["08 07"] * 6),
        "实际 %s" % pl[:12].hex(" ").upper())
    chk("v=0x0064 a=0x00C8", pl[12:].hex(" ").upper() == "64 00 C8 00",
        "实际 %s" % pl[12:].hex(" ").upper())

    print("== 负载构造：STREAM_JOINTS 文档示例 ==")
    # 文档 §5.7: 逻辑角 0,10,20,30,40,50 -> 00 00 64 00 C8 00 2C 01 90 01 F4 01
    pl = payload_stream_joints([0, 10, 20, 30, 40, 50])
    chk("STREAM_JOINTS = 00 00 64 00 C8 00 2C 01 90 01 F4 01",
        pl.hex(" ").upper() == "00 00 64 00 C8 00 2C 01 90 01 F4 01",
        "实际 %s" % pl.hex(" ").upper())

    print("== GET_STATUS / FK_QUERY 顺序相反、偏移 20 含义不同 ==")
    # 造一个 21B 负载：6 关节 180.0 等；位姿 150.0/-3.0/420.0/88.0；flags 位域 bit3
    body = b"".join(_i16(1800) for _ in range(6)) + _i16(1500) + _i16(-30) + _i16(4200) + _i16(880)
    st = decode_status(body + bytes([0x0A]))
    chk("status 先关节: joints[0]==180.0", st.joints[0] == 180.0)
    chk("status x=150.0 y=-3.0 z=420.0 pitch=88.0",
        (st.x, st.y, st.z, st.pitch) == (150.0, -3.0, 420.0, 88.0))
    chk("status flags 0x0A => estop=0 traj=1 corr=0 read_ok=1",
        (st.estop, st.traj, st.corr, st.read_ok) == (False, True, False, True))
    fkbody = _i16(1500) + _i16(-30) + _i16(4200) + _i16(880) + b"".join(_i16(1800) for _ in range(6))
    fk = decode_fk(fkbody + bytes([1]))
    chk("fk 先位姿: x=150.0", fk.x == 150.0)
    chk("fk joints[0]==180.0 read_ok==1", fk.joints[0] == 180.0 and fk.read_ok == 1)

    print("== 读失败哨兵 -32768 ==")
    body = _i16(1800) + _i16(1205) + _i16(-32768) + _i16(2000) + _i16(900) + _i16(3599)
    body += bytes(8) + bytes([0x00])
    st2 = decode_status(body)
    chk("J3 读失败 => None, read_ok=0",
        st2.joints[2] is None and st2.read_ok is False)
    chk("失败轴列表 == [3]", st2.failed_joints() == [3])
    chk("as_servo_deg() 返回 None（不兜底）", st2.as_servo_deg() is None)

    print("== CAN 分包 / 重组 ==")
    chk("LEN=0 -> 1 帧", can_plan_frames(0)[0] == 1)
    chk("LEN=21 -> 4 帧", can_plan_frames(21)[0] == 4)
    chk("LEN=64 -> 10 帧", can_plan_frames(64)[0] == 10)
    for seq, cmd, pl in ((0x11, CMD_MOVE_JOINTS, bytes(16)),
                         (0x12, CMD_GET_STATUS, b""),
                         (0x13, CMD_STREAM_JOINTS, bytes(12))):
        frames = can_build_frames(seq, cmd, pl)
        r = CanReassembler()
        got = None
        for f in frames:
            got = r.feed(f) or got
        chk("CAN 往返 seq=%d cmd=0x%02X (%d 帧)" % (seq, cmd, len(frames)),
            got is not None and got[0] == seq and got[1] == cmd and got[2] == pl and got[3] is True)
    # 乱序到达（1 和 2 对调）仍能重组
    frames = can_build_frames(0x14, CMD_MOVE_JOINTS, bytes(16))
    r = CanReassembler()
    got = r.feed(frames[0]) or r.feed(frames[2]) or r.feed(frames[1])
    chk("CAN 乱序 0,2,1 仍重组成功", got is not None and got[3] is True)

    print("== 串口混合流解析（AA55 + 文本） ==")
    fp = FrameStreamParser()
    texts = []
    got = None
    for byte in b"s\r\n" + SerialCarrier().build(0x22, CMD_PING):
        r = fp.feed(byte)
        if r is None:
            continue
        tag, val = r
        if tag == "text":
            texts.append(val)
        elif tag == "frame":
            got = val
    chk("文本字节被交还", bytes(texts) == b"s\r\n")
    chk("AA55 帧被解出", got == (0x22, CMD_PING, b""))

    print("== 角度体系：舵机度 ⇄ 逻辑角 ==")
    calib = JointCalib(offset=[180.0] * 6, dir=list(JOINT_DIR), source="test")
    lg = calib.to_logical([180.0] * 6)
    chk("中位 180° -> 逻辑角全 0", all(abs(v) < 1e-9 for v in lg), "实际 %s" % lg)
    chk("J1 dir=-1: servo 190 -> logical -10", abs(calib.to_logical([190, 180, 180, 180, 180, 180])[0] + 10) < 1e-9)
    chk("往返 舵机度->逻辑->舵机度", all(
        abs(a - b) < 1e-6 for a, b in
        zip(calib.to_servo(calib.to_logical([10, 200, 45, 300, 359, 1])), [10, 200, 45, 300, 359, 1])))
    chk("wrap180(190) == -170", abs(wrap180(190) + 170) < 1e-9)
    chk("wrap180(-190) == 170", abs(wrap180(-190) - 170) < 1e-9)
    chk("wrap180(180) == 180", abs(wrap180(180) - 180) < 1e-9)

    print("== 错误码表 ==")
    chk("ESTOP == 5", ErrCode.ESTOP == 5)
    chk("FK_VERIFY == 6", ErrCode.FK_VERIFY == 6)
    chk("参数表 24 项", len(PARAM_TABLE) == AP_PARAM_COUNT,
        "实际 %d" % len(PARAM_TABLE))

    print("\n" + "=" * 70)
    print("%s (%d 项失败)" % ("全部通过 ✅" if fails == 0 else "有失败项 ❌", fails))
    print("=" * 70)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
