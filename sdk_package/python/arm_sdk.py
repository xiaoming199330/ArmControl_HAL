# -*- coding: utf-8 -*-
"""
arm_sdk.py — 六轴机械臂上位机 SDK（Python，可运行）

设计目标
--------
1. **业务层不可能漏掉行尾**：`send_command()` 内部强制追加 `\\r\\n`。对外不提供
   裸写接口给业务层（`write_raw` 明确标注为高级/危险用途）。
2. **响应按"提示符"分帧**，不按字节数、不按超时猜。
   固件每条命令处理完打印 `> `（无换行），所以 `\\r\\n> ` 就是天然的结束标记。
3. **异步行分流**：`TM `(遥测) / `  Feedback:` / `  Pose:` / `[RUN]`(日志)
   是设备自己随时吐的，不属于任何命令的响应 —— 必须分流，否则会污染解析。
4. **"读失败"如实上报**：任何关节读失败 → `None`，**绝不**用上一次的值或
   目标值填充。

用法（同步阻塞式，适合 GUI 后台线程或脚本）
-------------------------------------------
    from arm_sdk import ArmSDK

    arm = ArmSDK("COM11")            # 或 ArmSDK.simulated() 无硬件演示
    arm.open()
    print(arm.status())              # 6 轴状态表(结构体)
    print(arm.pose())                # 末端位姿
    arm.move_joints([180, 180, 180, 180, 180, 180], speed=30)
    arm.close()

线程模型
--------
- 一个读线程持续读串口，按状态机把字节流拆成"文本行 / 二进制帧 / 提示符"。
- `send_command()` 由调用线程阻塞等待，内部用 `threading.Condition` 唤醒。
- **同一时刻只允许一条命令在途**（内部有锁）；这是刻意的，因为固件是
  单命令行的、回包没有请求 ID。并发调用会被串行化。
"""

from __future__ import annotations

import math
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

# 同目录的 FK/IK 参考实现
try:
    from arm_fk import (Pose, deg_to_servo_raw, fk_from_servo_deg, rad2deg,
                        servo_raw_to_deg)
except ImportError:      # 允许作为包的一部分被导入
    from .arm_fk import (Pose, deg_to_servo_raw, fk_from_servo_deg, rad2deg,
                         servo_raw_to_deg)

PROMPT = b"\r\n> "        # 命令结束标记（固件每条命令后打印 "> "）
NUM_JOINTS = 6

# ============================================================================
# 1. 二进制协议（CRC / 组帧 / 解帧）
# ============================================================================

APROTO_HEAD0 = 0xAA
APROTO_HEAD1 = 0x55
APROTO_MAX_PAYLOAD = 64

CMD_PING = 0x01
CMD_GET_STATUS = 0x02
CMD_LOG_SET_LEVEL = 0x03
CMD_MOVE_JOINTS = 0x10
CMD_STOP = 0x11
CMD_SET_TORQUE = 0x12
CMD_STREAM_JOINTS = 0x13          # 主从臂专用，本版上位机不用
CMD_MOVE_CART = 0x20
CMD_FK_QUERY = 0x21
CMD_SET_GRAVITY = 0x30
CMD_SET_CORRECTION = 0x31
CMD_PARAM_READ = 0x40
CMD_PARAM_WRITE = 0x41
CMD_PARAM_COUNT = 0x42
RSP_ACK = 0x7F
RSP_NACK = 0x7E

ERR_NAME = {
    0: "OK",
    1: "PARAM",
    2: "CRC",
    3: "UNREACH",
    4: "LIMIT",
    5: "ESTOP",
    6: "FK_VERIFY",
    7: "READ",
}
ERR_DESC = {
    0: "成功",
    1: "参数非法（长度/范围/ID）",
    2: "帧 CRC 校验失败",
    3: "IK 不可达 / 不收敛",
    4: "解算结果超关节限位",
    5: "急停激活，禁止运动",
    6: "IK 解 FK 回代验证超差",
    # 码 7 是多命令复用的：舵机总线读失败 或 读 Flash 参数区失败
    7: "读失败（舵机总线或 Flash 参数区，按命令区分）",
}

# 关节读取失败的哨兵值
RAW_READ_FAIL = -32768


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, **非反射**（MSB-first）。

    计算范围 = SEQ, CMD, LEN, PAYLOAD（即从帧偏移 2 开始，共 3+LEN 字节）。
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    """组帧：AA 55 SEQ CMD LEN PAYLOAD CRC_LO CRC_HI。"""
    if len(payload) > APROTO_MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {APROTO_MAX_PAYLOAD}")
    body = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + payload
    crc = crc16_ccitt_false(body)
    return bytes([APROTO_HEAD0, APROTO_HEAD1]) + body + bytes([crc & 0xFF, crc >> 8])


class FrameParser:
    """二进制帧解析状态机（与固件 Aproto_ParserFeed 对称）。

    同时负责**从混合字节流里分流**：喂进来自控制台的每个字节，
    若返回 None 表示"这是文本字节"；若返回 (seq,cmd,payload) 表示收全一帧。
    """

    def __init__(self, interbyte_timeout_s: float = 0.2):
        self._buf = bytearray()
        self._timeout = interbyte_timeout_s
        self._last = 0.0

    def feed(self, byte: int, now: Optional[float] = None):
        now = time.monotonic() if now is None else now
        # 帧间超时 -> 复位（防半截帧卡死）
        if self._buf and (now - self._last) > self._timeout:
            self._buf.clear()

        b = self._buf
        b.append(byte)
        n = len(b)

        if n == 1:
            if b[0] != APROTO_HEAD0:
                b.clear()
                return ("text", byte)      # 文本字节，交还文本通道
            self._last = now
            return None

        if n == 2:
            if b[1] == APROTO_HEAD1:
                self._last = now
                return None
            if b[1] == APROTO_HEAD0:
                del b[0]                       # AA AA -> 保留后一个
                return None
            b.clear()
            return ("drop", None)              # AA 后的杂散字节，丢弃

        self._last = now
        # n >= 3：已有 SEQ CMD LEN
        if n == 3:
            return None
        if n == 4:
            return None
        if n == 5:
            if b[4] > APROTO_MAX_PAYLOAD:
                b.clear()
                return ("drop", None)
            return None
        need = 5 + b[4] + 2
        if n < need:
            return None

        seq, cmd, length = b[2], b[3], b[4]
        payload = bytes(b[5:5 + length])
        got = b[5 + length] | (b[5 + length + 1] << 8)
        want = crc16_ccitt_false(bytes(b[2:5 + length]))
        b.clear()
        if got != want:
            return ("crc_err", (seq, cmd, payload))
        return ("frame", (seq, cmd, payload))


# ============================================================================
# 2. 响应对象与关键词判定
# ============================================================================

SUCCESS_KW = ("OK", "Saved", "Stopped", "complete", "Complete", "set to", "saved",
              "Done", "done", "started", "accepted")
USAGE_KW = ("Usage:", "usage:")
# ⚠ 2026-09-11: 补入固件真实错误串。此前 "Unknown command 'x'. Type 'h' for help."
#   不命中任何关键词, 被末尾 "有输出即成功" 的兜底判成 ok=True —— 命令根本没执行,
#   UI 却显示成功。下面这些串固件都会原样打印, 必须显式判失败。
FAIL_KW = ("FAIL", "FAILED", "failed", "reject", "Rejected", "NACK",
           "unreachable", "out of range", "IK failed", "cancelled", "Not ",
           "Unknown command",      # 固件 main.c:5719 未知命令
           "Unknown pose",         # cmdPose: 非法预设位编号
           "must be",              # "Joint must be 0-5" / "Segment must be 1~8"
           "Cannot ",              # "Cannot move." / "Cannot enter teach mode."
           "Error:", "Invalid", "invalid",
           "not calibrated", "uncalibrated")
SAFETY_KW = ("path check fail", "path unsafe", "read FAILED", "READ FAIL",
             "UNSTABLE", "Another loop is active", "no reply", "offline",
             "out of joint limits", "NOT met", "too large")
ESTOP_KW = ("E-STOP",)

# 钳制回显行: clamp_angle() 在把角度改掉时打印
#   "  J3: 320 -> 300 deg (raw 4096 -> 4095 limit)"
# 命令本身是"成功"的, 但输入被静默改动 —— UI 必须标黄提示, 不能当正常成功。
CLAMP_RE = re.compile(
    r"J(\d+):\s*(-?[\d.]+)\s*->\s*(-?[\d.]+)\s*deg\s*\(raw\s*(\d+)\s*->\s*(\d+)\s*limit\)")
# 限位设置回显: "  J1 limits set: 80 ~ 220 deg (raw 910 ~ 2503)"
LIMITS_SET_RE = re.compile(
    r"J(\d+)\s+limits set:\s*(-?[\d.]+)\s*~\s*(-?[\d.]+)\s*deg\s*"
    r"\(raw\s*(\d+)\s*~\s*(\d+)\)")


@dataclass
class Response:
    """一条命令的响应。

    raw      : 原始文本（未过滤，用于排错显示）
    lines    : 过滤后的有效行（去回显 / 去 `> ` / 去异步行）
    async_lines: 响应期间收到的异步行（TM/Feedback/Pose/RUN）
    ok       : 是否成功（关键词判定）
    kind     : "ok" | "usage" | "fail" | "safety" | "estop" | "timeout" | "empty"
    """
    command: str
    raw: str = ""
    lines: List[str] = field(default_factory=list)
    async_lines: List[str] = field(default_factory=list)
    ok: bool = False
    kind: str = "empty"

    def text(self) -> str:
        return "\n".join(self.lines)

    def clamps(self) -> List[Dict[str, object]]:
        """本响应里被**限位钳制**的轴（见 `parse_clamp_lines`）。UI 要标黄提示。"""
        return parse_clamp_lines(self.lines)

    def __str__(self) -> str:
        return f"Response({self.command!r}, ok={self.ok}, kind={self.kind}, {len(self.lines)} lines)"


def classify(command: str, lines: Sequence[str]) -> Tuple[bool, str]:
    """按关键词判定结果。顺序：急停 > 安全拦截 > 失败 > 用法错误 > 成功。"""
    blob = "\n".join(lines)
    if any(k in blob for k in ESTOP_KW):
        return False, "estop"
    if any(k in blob for k in SAFETY_KW):
        return False, "safety"
    if any(k in blob for k in USAGE_KW):
        return False, "usage"
    if any(k in blob for k in FAIL_KW):
        return False, "fail"
    if any(k in blob for k in SUCCESS_KW):
        return True, "ok"
    # 有些只读命令（p/s/L）没有关键词，但有输出 => 视为成功
    if lines:
        return True, "ok"
    return False, "empty"


def parse_clamp_lines(lines: Sequence[str]) -> List[Dict[str, object]]:
    """抽出响应里**被静默钳制**的轴（`clamp_angle()` 的回显行）。

    返回 `[{"joint": 2, "joint_label": "J3", "from_deg": 320.0,
            "to_deg": 300.0, "from_raw": 4096, "to_raw": 4095}, ...]`

    `joint` 是 **0 基**（与 `p J` / `L J` 一致）；`joint_label` 是固件显示的 `J3`。
    命令本身算"成功"（角度确实下去了），但**输入被改了** —— UI 必须标黄并明说
    "已被限位钳制"，否则用户看到的就是"我发了 320，臂停在 300"却不明白为什么。
    """
    out: List[Dict[str, object]] = []
    for ln in lines:
        m = CLAMP_RE.search(ln)
        if m:
            j1 = int(m.group(1))
            out.append({"joint": j1 - 1, "joint_label": "J%d" % j1,
                        "from_deg": float(m.group(2)), "to_deg": float(m.group(3)),
                        "from_raw": int(m.group(4)), "to_raw": int(m.group(5))})
    return out


def parse_limits_set(line: str) -> Optional[Dict[str, object]]:
    """解析 `L J min max` 的回显：`J1 limits set: 80 ~ 220 deg (raw 910 ~ 2503)`。"""
    m = LIMITS_SET_RE.search(line)
    if not m:
        return None
    return {"min_deg": float(m.group(2)), "max_deg": float(m.group(3)),
            "min_raw": int(m.group(4)), "max_raw": int(m.group(5))}


# 异步行前缀：这些是设备主动推的，不属于命令响应
ASYNC_PREFIXES = ("TM ", "  Feedback:", "  Pose:", "[RUN]", "  [RUN]")
# 需要从响应里剔除的前缀（提示/日志）
DROP_PREFIXES = ("[RUN]", "  [RUN]", "TM ")


def is_async_line(line: str) -> bool:
    return any(line.startswith(p) for p in ASYNC_PREFIXES)


# ============================================================================
# 3. 传输层
# ============================================================================

class Transport:
    """传输抽象。真实串口与模拟器都实现这个接口。"""

    def open(self) -> None: ...
    def close(self) -> None: ...
    def write(self, data: bytes) -> None: ...
    def read(self, n: int = 256) -> bytes: raise NotImplementedError
    @property
    def is_open(self) -> bool: return True


class SerialTransport(Transport):
    """pyserial 封装。pyserial 缺失时给出明确安装提示。"""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None

    def open(self) -> None:
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise RuntimeError(
                "缺少 pyserial。请先安装：pip install pyserial"
            ) from e
        # ⚠ 参数固定：115200 8N1、无流控、无 DTR/RTS 握手
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        # 打开后清空收发缓冲（不然可能读到开机横幅的半截 / 上次残留）
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def write(self, data: bytes) -> None:
        if self._ser is None:
            raise RuntimeError("串口未打开")
        self._ser.write(data)

    def read(self, n: int = 256) -> bytes:
        if self._ser is None:
            return b""
        try:
            return self._ser.read(n)
        except Exception:
            return b""

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open


# ============================================================================
# 4. 主 SDK 类
# ============================================================================

class ArmSDK:
    """六轴机械臂上位机 SDK（同步 API，内部自带读线程）。

    生命周期：
        arm = ArmSDK("COM11"); arm.open(); ...; arm.close()
    或上下文管理：
        with ArmSDK("COM11") as arm: ...
    无硬件演示：
        with ArmSDK.simulated() as arm: ...
    """

    def __init__(self, transport: Transport, response_timeout: float = 3.0,
                 long_op_timeout: float = 60.0):
        self._t = transport
        self._resp_timeout = response_timeout
        self._long_timeout = long_op_timeout

        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self._reader: Optional[threading.Thread] = None
        self._running = False

        self._parser = FrameParser()
        self._text_buf = bytearray()      # 当前行累积
        self._echo_expect = b""           # 本次命令的期望回显（用于剔除）
        self._resp_lines: List[str] = []  # 当前响应累积
        self._async_lines: List[str] = []
        self._pending = False
        self._done = threading.Event()

        self._frames: Dict[int, Tuple[int, bytes]] = {}   # seq -> (cmd,payload)
        self._seq = 0

        # 异步回调
        self.on_telemetry: Optional[Callable[[Dict[str, List[Optional[float]]]], None]] = None
        self.on_log: Optional[Callable[[str], None]] = None
        self.on_async: Optional[Callable[[str], None]] = None
        self.on_raw: Optional[Callable[[str], None]] = None

        # 最近一次读取到的状态（供 UI 用；**不可**用失败值覆盖）
        self.last_status: Optional["JointStatus"] = None
        self.last_pose: Optional[Pose] = None

    # ---------------------------------------------------------------- 构造
    @classmethod
    def simulated(cls, **kw) -> "ArmSDK":
        """无硬件演示/离线开发用。模拟器为**近似**实现，仅供 UI 联调。"""
        return cls(SimTransport(), **kw)

    # ---------------------------------------------------------------- 连接
    def open(self) -> None:
        self._t.open()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, name="arm-sdk-reader",
                                       daemon=True)
        self._reader.start()
        # 吃一次开机横幅/残留提示符，避免污染第一条命令
        time.sleep(0.15)
        with self._lock:
            self._text_buf.clear()
            self._resp_lines.clear()

    def close(self) -> None:
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        self._t.close()

    def __enter__(self) -> "ArmSDK":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._t.is_open

    # ---------------------------------------------------------- 读线程
    def _read_loop(self) -> None:
        while self._running:
            data = self._t.read(256)
            if not data:
                continue
            if self.on_raw:
                try:
                    self.on_raw(data.decode("utf-8", "replace"))
                except Exception:
                    pass
            for byte in data:
                kind = self._parser.feed(byte)
                if kind is None:
                    continue
                tag, val = kind
                if tag == "text":
                    self._on_text_byte(val)
                elif tag == "frame":
                    seq, cmd, payload = val
                    with self._lock:
                        self._frames[seq] = (cmd, payload)
                        self._cond.notify_all()
                elif tag == "crc_err":
                    seq, cmd, _ = val
                    with self._lock:
                        # 对端报 CRC 错：单独标记，避免与正常响应混淆
                        self._frames[("crc", seq)] = (cmd, b"")
                        self._cond.notify_all()

    def _on_text_byte(self, byte: int) -> None:
        if byte == 0x0D or byte == 0x0A:
            line = self._text_buf.decode("utf-8", "replace")
            self._text_buf.clear()
            if line:
                self._on_line(line)
            return
        self._text_buf.append(byte)
        # 提示符 "> " 是命令结束标记：固件打印 "> " 时**前面已经有 \r\n**，
        # 所以这里不会和 "-> " 之类的普通文本混淆（那些出现在行内）。
        if len(self._text_buf) >= 2 and self._text_buf[-2:] == b"> ":
            prefix = self._text_buf[:-2].decode("utf-8", "replace")
            self._text_buf.clear()
            if prefix:
                self._on_line(prefix)
            self._on_prompt()

    def _on_line(self, line: str) -> None:
        if is_async_line(line):
            with self._lock:
                self._async_lines.append(line)
            self._handle_async(line)
            return
        with self._lock:
            self._resp_lines.append(line)

    def _handle_async(self, line: str) -> None:
        try:
            if line.startswith("TM ") and self.on_telemetry:
                self.on_telemetry(parse_tm_line(line))
            elif line.startswith("[RUN]") and self.on_log:
                self.on_log(line)
            elif self.on_async:
                self.on_async(line)
        except Exception:
            pass

    def _on_prompt(self) -> None:
        with self._lock:
            if self._pending:
                self._done.set()
                self._cond.notify_all()

    # ---------------------------------------------------------- 同步等待
    def _wait(self, timeout: float) -> bool:
        return self._done.wait(timeout)

    # ---------------------------------------------------------- 发送
    def send_command(self, command: str, timeout: Optional[float] = None,
                     echo: bool = True) -> Response:
        """发送一条**文本命令**并等待其响应。

        ⚠ 行尾由本函数保证：内部追加 `\\r\\n`。业务层**永远不要**自己加行尾，
           也**不要**传入已经带行尾的字符串（会被拒绝以防双行尾误执行）。

        参数
        ----
        command : 不含行尾的命令本体，例如 "p 0"、"m 180 180 180 180 180 180 30"
        timeout : 秒。`Y`（烧 EEPROM）、`play`、`compact`、`c` 等长操作要放大。
        echo    : True 时把命令回显一并剔除（固件逐字符回显）。
        """
        cmd = command.strip()
        if not cmd:
            raise ValueError("空命令")
        # 先查行尾（必须在 strip 之前，否则 '\n' 已被吃掉）
        if command.rstrip(" \t").endswith(("\r", "\n")):
            raise ValueError(
                "send_command() 会自动追加行尾，请勿再传入 '\\r'/\\n'（防双行尾）")

        with self._lock:
            self._resp_lines.clear()
            self._async_lines.clear()
            self._frames.clear()
            self._done.clear()
            self._pending = True

        # 强制 CRLF
        data = (cmd + "\r\n").encode("ascii", "replace")
        self._t.write(data)

        got = self._wait(timeout if timeout is not None else self._resp_timeout)

        with self._lock:
            self._pending = False
            raw_lines = list(self._resp_lines)
            async_lines = list(self._async_lines)

        lines = self._filter_lines(raw_lines, cmd, echo)
        ok, kind = classify(cmd, lines)
        if not got:
            kind, ok = "timeout", False
        return Response(command=cmd, raw="\n".join(raw_lines), lines=lines,
                        async_lines=async_lines, ok=ok, kind=kind)

    @staticmethod
    def _filter_lines(raw_lines: Sequence[str], cmd: str, echo: bool) -> List[str]:
        out: List[str] = []
        for ln in raw_lines:
            if any(ln.startswith(p) for p in DROP_PREFIXES):
                continue
            if echo and ln.strip() == cmd.strip():
                continue                       # 回显行
            if ln.strip() in (">", ">"):
                continue                       # 提示符残留
            if not ln.strip():
                continue
            out.append(ln)
        return out

    def write_raw(self, text: str) -> None:
        """⚠ 高级/危险：不追加行尾、不等响应。仅用于交互式模式（如 `c` 校准）。

        普通业务**请用 send_command()**。用错这里 = "下了命令没反应" 的经典原因。
        """
        self._t.write(text.encode("ascii", "replace"))

    # ---------------------------------------------------------- 二进制通道
    def next_seq(self) -> int:
        with self._lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def send_binary(self, cmd: int, payload: bytes = b"",
                    timeout: Optional[float] = None) -> Tuple[Optional[int], Optional[bytes], bool]:
        """发送一个二进制帧并等响应（按 SEQ 配对）。

        返回 (rsp_cmd, rsp_payload, crc_ok)。
        超时返回 (None, None, False)。crc_ok=False 表示对端报 CRC 错。
        """
        seq = self.next_seq()
        frame = build_frame(seq, cmd, payload)
        with self._lock:
            self._frames.clear()
        self._t.write(frame)
        deadline = time.monotonic() + (timeout or self._resp_timeout)
        while time.monotonic() < deadline:
            with self._lock:
                if ("crc", seq) in self._frames:
                    self._frames.pop(("crc", seq), None)
                    return None, None, False
                hit = self._frames.pop(seq, None)
                if hit is not None:
                    r_cmd, r_payload = hit
                    return r_cmd, r_payload, True
            time.sleep(0.002)
        return None, None, False

    # ==================================================================
    # 高级 API —— 常用命令
    # ==================================================================

    # ---- 基础 ----
    def help(self) -> Response:
        return self.send_command("h")

    def scan(self) -> "ScanResult":
        return parse_scan(self.send_command("n"))

    def status(self) -> "JointStatus":
        """`s` —— 6 轴状态表。读失败的轴 pos=None（原始值 0）。"""
        st = parse_status(self.send_command("s"))
        self.last_status = st
        return st

    def joint(self, j: int) -> "JointRead":
        """`p J`（J=0~5）—— 单轴 raw / 舵机度 / rel-home 关节角。"""
        if not 0 <= j <= 5:
            raise ValueError("单轴命令的关节号是 0~5（J1=0）")
        return parse_joint(self.send_command(f"p {j}"), j)

    def pose(self) -> Optional[Pose]:
        """`f` —— 末端位姿（FK）。

        ⚠ 固件在 6 轴任一读失败时**不打印任何东西**（静默），所以返回 None
          就等于"读取失败"，UI 必须如实显示错误，**不要**沿用旧值。
        """
        r = self.send_command("f")
        p = parse_fk(r.lines)
        if p is not None:
            self.last_pose = p
        return p

    def limits(self) -> "LimitsInfo":
        """`L` —— 显示 6 轴限位（**舵机度整数**）。

        ⚠ 限制：固件没有"直接回读 raw 限位"的命令。`L` 打印的是整数舵机度，
          跨 0/360 的包裹区间会显示成 MIN>MAX。本地 IK 预校验因此只能是
          **近似**；权威判定始终在固件侧（被拒会回显 out of joint limits）。
        """
        return parse_limits(self.send_command("L"))

    def set_limit(self, j: int, min_deg: float, max_deg: float) -> Response:
        """`L J min max`（J=0~5，舵机度）。⚠ 会同时改写 JOINT_MIN_RAW/MAX_RAW。"""
        if not 0 <= j <= 5:
            raise ValueError("单轴命令的关节号是 0~5（J1=0）")
        return self.send_command(f"L {j} {min_deg:g} {max_deg:g}")

    # ---- 运动 ----
    def move_joints(self, servo_deg: Sequence[float], speed: Optional[float] = None,
                    long_op: bool = False) -> Response:
        """`m a1..a6 [v]` —— 6 关节同时运动。**输入是舵机度，不是关节角！**

        ⚠ 两个**静默**陷阱（固件都不报错，只是把值改掉）：
          1. 每轴被硬钳到 `0~300`，超 300 会**静默变成 300**。这里先抛
             `ValueError` 拦下；要 300~360 的行程请改 `move_angle()`（单轴）
             或 `bin_move_joints()`。
          2. `speed` 省略时固件用 `ARM_SC_DEF_VMAX`＝**25** deg/s
             （固件帮助文本写 "default=30"，是错的）；给值会被钳到 5~300。
        """
        if len(servo_deg) != 6:
            raise ValueError("需要 6 个舵机度")
        bad = [v for v in servo_deg if v < 0 or v > 300]
        if bad:
            raise ValueError(
                f"`m` 的舵机度硬钳制是 0~300，越界值 {bad} 会被固件静默改成 300。"
                "超 300 请用 move_angle() / bin_move_joints()")
        cmd = "m " + " ".join(f"{v:g}" for v in servo_deg)
        if speed is not None:
            cmd += f" {speed:g}"
        return self.send_command(cmd, timeout=self._long_timeout if long_op else None)

    def move_angle(self, j: int, servo_deg: float, long_op: bool = True) -> Response:
        """`a J deg`（J=0~5）—— 单轴运动到舵机度。

        与 `move_joints` 的区别：**没有 0~300 的硬钳制**，上限到 360
        （`deg_to_servo_raw` 把 >360 折成 360）。仍会被 `clamp_angle()` 按该轴
        raw 限位钳制，被钳时会回显 `J%d: x -> y deg` —— 请解析这行并在 UI 上
        标黄，不要当成正常成功。
        """
        if not 0 <= j <= 5:
            raise ValueError("单轴命令的关节号是 0~5（J1=0）")
        return self.send_command(f"a {j} {servo_deg:g}",
                                 timeout=self._long_timeout if long_op else None)

    def save_waypoint(self, n: int, servo_deg: Sequence[float]) -> Response:
        """`q n j1..j6` —— 存路点（`n`=1~8）。**输入是舵机度，不是关节角！**

        与 `m` 不同，这里**没有 0~300 硬钳制**，按 raw 限位钳制即可。
        存的点被 `QL`/`QLC` 消费；存盘需另发 `save`（见示教文档）。
        """
        if not 1 <= n <= 8:
            raise ValueError("路点号 n 是 1~8")
        if len(servo_deg) != 6:
            raise ValueError("需要 6 个舵机度")
        return self.send_command("q " + str(n) + " " +
                                 " ".join(f"{v:g}" for v in servo_deg))

    def point_loop(self, a: Sequence[float], b: Sequence[float],
                   rounds: int = 0, speed: Optional[float] = None,
                   long_op: bool = True) -> Response:
        """`B a1..a6 b1..b6 rounds [v]` —— A/B 两点往返。**舵机度**。

        `rounds` = 轮数（**1 轮 = A→B→A**，起手先到 A），`0` = 无限（用 `S` 停）。
        固件把 `rounds` 钳到 ≤1000、`speed` 钳到 5~300（缺省 30）。
        返回后必须发 `stop()` 才能解除"另一个循环在跑"的拒命令状态。
        """
        if len(a) != 6 or len(b) != 6:
            raise ValueError("需要 6+6 个舵机度")
        cmd = "B " + " ".join(f"{v:g}" for v in a) + " " + \
              " ".join(f"{v:g}" for v in b) + f" {int(rounds)}"
        if speed is not None:
            cmd += f" {speed:g}"
        return self.send_command(cmd, timeout=self._long_timeout if long_op else None)

    def multi_point_loop(self, n: int, rounds: int = 0, speed: Optional[float] = None,
                         long_op: bool = True) -> Response:
        """`QL n r [v]` —— 多点循环（走 `q` 存的前 `n` 个点，每点到点测量）。

        ⚠ **`Q` 必须大写**（小写 `ql` 会被当成路点存储，回 Usage）。
        到点判定含"连续稳定 500ms"，读不到位置时**暂停计时而不清零**。
        `rounds` 0 = 无限，用 `S` 停。
        """
        if not 1 <= n <= 8:
            raise ValueError("点数 n 是 1~8")
        cmd = f"QL {n} {int(rounds)}"
        if speed is not None:
            cmd += f" {speed:g}"
        return self.send_command(cmd, timeout=self._long_timeout if long_op else None)

    def move_cart(self, x: float, y: float, z: float, pitch_deg: float = 0.0,
                  long_op: bool = True) -> Response:
        """`g x y z [pitch]` —— IK 笛卡尔运动，单位 mm / 度。

        固件会依次检查：IK 可达 → raw 限位 → FK 回代(5mm) → 工具俯仰(10°)
        → 安全路径规划（桌面/立柱）。任一不过就**拒绝并回显原因**。
        本地可用 `arm_fk.ik_checked()` 预判，减少无效下发。
        """
        return self.send_command(f"g {x:g} {y:g} {z:g} {pitch_deg:g}",
                                 timeout=self._long_timeout if long_op else None)

    def preset(self, pose_id: int, long_op: bool = True) -> Response:
        """`P 0..4` —— 0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2。"""
        if not 0 <= pose_id <= 4:
            raise ValueError("pose_id 需为 0~4")
        return self.send_command(f"P {pose_id}", timeout=self._long_timeout if long_op else None)

    def stop(self) -> Response:
        """`S` —— 一键停止（轨迹/QL/QLC/W/示教回放）。**安全命令，任何时候都能发。**"""
        return self.send_command("S")

    def torque(self, j: int, on: bool) -> Response:
        """`t J 0|1`（J=0~5）。"""
        if not 0 <= j <= 5:
            raise ValueError("单轴命令的关节号是 0~5（J1=0）")
        return self.send_command(f"t {j} {1 if on else 0}")

    def home(self, j: Optional[int] = None) -> Response:
        """`z J` / `z a` —— 单轴/全部回中位（用校准零点）。"""
        return self.send_command("z a" if j is None else f"z {j}",
                                 timeout=self._long_timeout)

    # ---- PID（三步链） ----
    def pid_all(self) -> Dict[str, List[int]]:
        """`K`/`I`/`D` 各打一次，得到 6 轴的缓存 PID。"""
        out = {"p": [], "i": [], "d": []}
        for key, letter in (("p", "K"), ("i", "I"), ("d", "D")):
            r = self.send_command(letter)
            m = re.search(r":\s*(.*)", r.text())
            if m:
                out[key] = [int(x) for x in re.findall(r"J\d+=(\d+)", m.group(1))]
        return out

    def pid_one(self, j: int) -> Dict[str, int]:
        """`K J`/`I J`/`D J` —— 单轴缓存 PID。J = 1~6（注意不是 0~5）。"""
        self._check_pid_joint(j)
        res = {}
        for key, letter in (("p", "K"), ("i", "I"), ("d", "D")):
            m = re.search(rf"J{j} {key.upper()}-Gain = (\d+)", self.send_command(f"{letter} {j}").text())
            res[key] = int(m.group(1)) if m else -1
        return res

    def pid_cache_vs_eeprom(self, j: int) -> "PidCompare":
        """`N J` —— 对比"STM32 缓存 PID" 与 "舵机 EEPROM PID"。J = 1~6。

        ⚠ 解析陷阱：EEPROM 那一行的字段顺序是 **P D I**，不是 P I D。
        """
        self._check_pid_joint(j)
        return parse_pid_compare(self.send_command(f"N {j}"), j)

    def set_pid(self, j: int, p: Optional[int] = None, i: Optional[int] = None,
                d: Optional[int] = None) -> Response:
        """设单轴 PID。**仅改 STM32 缓存**，必须再 `burn_pid()` 才进舵机 EEPROM。

        - 一次只给一个值时用 `K/I/D J val`
        - 三个都给时用 `N J P I D`（更原子，一次命令）
        """
        self._check_pid_joint(j)
        vals = [v for v in (p, i, d) if v is not None]
        for v in vals:
            if not 0 <= v <= 254:
                raise ValueError("PID 取值 0~254")
        if p is not None and i is not None and d is not None:
            return self.send_command(f"N {j} {p} {i} {d}")
        parts = []
        if p is not None:
            parts.append(f"K {j} {p}")
        if i is not None:
            parts.append(f"I {j} {i}")
        if d is not None:
            parts.append(f"D {j} {d}")
        last = Response(command="")
        for c in parts:
            last = self.send_command(c)
        return last

    def burn_pid(self, j: Optional[int] = None, timeout: float = 20.0) -> Response:
        """`Y J`（J=1~6）或 `Y 0`（全部）—— 把缓存 PID 烧入舵机 EEPROM。

        ⚠⚠ **该关节会失力约 0.6 秒** —— 调用方必须先弹窗要求用户托住机械臂！
            本方法不会替你确认，请在 UI 层做二次确认。
        """
        if j is not None:
            if not 1 <= j <= 6:
                raise ValueError("PID 命令的关节号是 1~6")
            return self.send_command(f"Y {j}", timeout=timeout)
        return self.send_command("Y 0", timeout=timeout)

    def restore_default_pid(self) -> Response:
        """`X` —— 恢复固件默认 PID（只写 RAM，不动两处持久化）。"""
        return self.send_command("X")

    def save_flash(self, timeout: float = 20.0) -> Response:
        """`F` —— 把 PID/修正/速度/参数/限位/零点写入 Flash（掉电保留）。"""
        return self.send_command("F", timeout=timeout)

    @staticmethod
    def _check_pid_joint(j: int) -> None:
        if not 1 <= j <= 6:
            raise ValueError("PID 命令的关节号是 1~6（J1=1）—— 与单轴命令的 0~5 不同！")

    # ---- 参数表 ----
    def param_list(self) -> List["ArmParam"]:
        """`LP` —— 算法参数表。"""
        return parse_params(self.send_command("LP"))

    def set_param(self, pid: int, value: float) -> Response:
        """`SP id value` —— **仅改 RAM，需 `save_flash()`**。"""
        return self.send_command(f"SP {pid} {value:g}")

    # ---- 遥测 ----
    def set_telemetry(self, period_ms: int = 200,
                      fields: str = "pti") -> Response:
        """`tm <ms> [fields]` —— 遥测。

        fields: p=角度 t=温度 i=电流 v=电压 s=速度 l=负载 a/*=全部
        ⚠ 运动中建议 >=100ms（后台按 20ms 轮转 1 轴，约 120ms 刷完 6 轴）。
        """
        if period_ms <= 0:
            return self.send_command("tm 0")
        return self.send_command(f"tm {period_ms} {fields}")

    def set_angle_stream(self, period_ms: int = 200) -> Response:
        """`pv <ms>` —— 周期打印 6 轴角度（走遥测缓存，不占总线）。"""
        return self.send_command(f"pv {period_ms}")

    def set_pose_stream(self, period_ms: int = 200) -> Response:
        """`pf <ms>` —— 周期打印末端位姿。"""
        return self.send_command(f"pf {period_ms}")

    # ---- 二进制协议快捷方法 ----
    def ping(self) -> Optional[Tuple[int, int]]:
        """PING -> (proto_ver, run_id)。"""
        cmd, pl, ok = self.send_binary(CMD_PING)
        if not ok or cmd is None or len(pl) < 2:
            return None
        return pl[0], pl[1]

    def bin_get_status(self) -> Optional["BinStatus"]:
        """GET_STATUS(0x02) -> 结构化状态。高频刷新（比 `s` 省带宽）。"""
        cmd, pl, ok = self.send_binary(CMD_GET_STATUS)
        if not ok or cmd is None or len(pl) < 21:
            return None
        return parse_bin_status(pl)

    def bin_fk(self) -> Optional[Pose]:
        """FK_QUERY(0x21) -> Pose。read_ok=0 时返回 None（不做兜底）。"""
        cmd, pl, ok = self.send_binary(CMD_FK_QUERY)
        if not ok or cmd is None or len(pl) < 21:
            return None
        if pl[20] != 1:
            return None
        x, y, z, p = (i16(pl, 0) / 10.0, i16(pl, 2) / 10.0,
                      i16(pl, 4) / 10.0, i16(pl, 6) / 10.0)
        return Pose(x, y, z, math.radians(p))

    def bin_move_joints(self, servo_deg: Sequence[float],
                        speed_dps: float = 0.0, accel_dpss: float = 0.0) -> Tuple[bool, str]:
        """MOVE_JOINTS(0x10)。负载 = 6×i16 舵机度×10 + u16 速度×10 + u16 加速度×10。"""
        if len(servo_deg) != 6:
            raise ValueError("需要 6 个舵机度")
        pl = bytearray()
        for v in servo_deg:
            pl += i16_bytes(int(round(v * 10)))
        pl += u16_bytes(int(round(speed_dps * 10)))
        pl += u16_bytes(int(round(accel_dpss * 10)))
        cmd, rp, ok = self.send_binary(CMD_MOVE_JOINTS, bytes(pl))
        if not ok:
            return False, "timeout/CRC"
        if cmd == RSP_ACK:
            return True, "OK"
        if cmd == RSP_NACK and len(rp) >= 2:
            return False, ERR_DESC.get(rp[1], f"err={rp[1]}")
        return False, f"unexpected rsp 0x{cmd:02X}"

    def bin_move_cart(self, x: float, y: float, z: float, pitch_deg: float = 0.0,
                      speed_dps: float = 0.0, accel_dpss: float = 0.0) -> Tuple[bool, str]:
        """MOVE_CART(0x20)。负载 = 4×i16(×10) + u16 速度×10 + u16 加速度×10。"""
        pl = bytearray()
        for v in (x, y, z, pitch_deg):
            pl += i16_bytes(int(round(v * 10)))
        pl += u16_bytes(int(round(speed_dps * 10)))
        pl += u16_bytes(int(round(accel_dpss * 10)))
        cmd, rp, ok = self.send_binary(CMD_MOVE_CART, bytes(pl))
        if not ok:
            return False, "timeout/CRC"
        if cmd == RSP_ACK:
            return True, "OK"
        if cmd == RSP_NACK and len(rp) >= 2:
            return False, ERR_DESC.get(rp[1], f"err={rp[1]}")
        return False, f"unexpected rsp 0x{cmd:02X}"

    def bin_stop(self) -> bool:
        cmd, _, ok = self.send_binary(CMD_STOP)
        return ok and cmd == RSP_ACK

    def bin_param_read(self, pid: int) -> Optional[float]:
        """PARAM_READ(0x40) -> 参数值（定点 ×1000）。"""
        cmd, pl, ok = self.send_binary(CMD_PARAM_READ, bytes([pid & 0xFF]))
        if not ok or cmd is None or len(pl) < 5:
            return None
        return i32(pl, 1) / 1000.0

    def bin_param_write(self, pid: int, value: float) -> Tuple[bool, str]:
        """PARAM_WRITE(0x41)。⚠ 等价 `SP`：仅改 RAM，需 `F` 存盘。"""
        pl = bytes([pid & 0xFF]) + i32_bytes(int(round(value * 1000)))
        cmd, rp, ok = self.send_binary(CMD_PARAM_WRITE, pl)
        if not ok:
            return False, "timeout/CRC"
        if cmd == RSP_ACK:
            return True, "OK"
        if cmd == RSP_NACK and len(rp) >= 2:
            return False, ERR_DESC.get(rp[1], f"err={rp[1]}")
        return False, f"unexpected rsp 0x{cmd:02X}"

    # ---- 危险操作（必须二次确认后调用） ----
    def init_servos(self, j: Optional[int] = None, timeout: float = 30.0) -> Response:
        """⚠⚠ `i` / `i J` —— 初始化舵机（Torque OFF→RESET→模式）。

        **会重写 PID/限位/中位**。UI 必须二次确认。"""
        return self.send_command("i" if j is None else f"i {j}", timeout=timeout)

    def center_all(self, timeout: float = 30.0) -> Response:
        """⚠ `M a` —— 把当前位置设为中位（raw 2048），全部轴。"""
        return self.send_command("M a", timeout=timeout)

    def broadcast_reboot(self, timeout: float = 30.0) -> Response:
        """⚠ `R 1` —— 广播 reboot 两条总线上所有舵机。"""
        return self.send_command("R 1", timeout=timeout)


# ============================================================================
# 5. 解析器（**全部按固件真实 printf 格式写**）
# ============================================================================

def i16(b: bytes, off: int) -> int:
    v = b[off] | (b[off + 1] << 8)
    return v - 0x10000 if v & 0x8000 else v


def u16(b: bytes, off: int) -> int:
    return b[off] | (b[off + 1] << 8)


def i32(b: bytes, off: int) -> int:
    v = b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)
    return v - 0x100000000 if v & 0x80000000 else v


def i16_bytes(v: int) -> bytes:
    return (v & 0xFFFF).to_bytes(2, "little")


def u16_bytes(v: int) -> bytes:
    return (v & 0xFFFF).to_bytes(2, "little")


def i32_bytes(v: int) -> bytes:
    return (v & 0xFFFFFFFF).to_bytes(4, "little")


@dataclass
class JointStatus:
    """`s` 表格解析结果。pos=None 表示该轴读取失败（固件打 0.0）。"""
    pos: List[Optional[float]] = field(default_factory=list)     # 舵机度
    moving: List[Optional[bool]] = field(default_factory=list)   # None = Err
    temp: List[int] = field(default_factory=list)
    raw_text: str = ""

    def ok_mask(self) -> List[bool]:
        return [p is not None for p in self.pos]


_STATUS_ROW = re.compile(
    r"\|\s*J(\d)\s*\|\s*(\d+)\s*\|\s*([\d.]+)\s*\|\s*(\S+)\s*\|\s*(\d+)C\s*\|")


def parse_status(r: Response) -> JointStatus:
    """解析 `s` 的 ASCII 表格：

        | J1 | 1  | 123.4 |   Yes    |  35C |

    ⚠ 固件对读失败的轴打印 `0.0`（`pos<0 -> deg_int=0`），**无法与真实的 0.0°
       区分**。要精确判断读成功与否，请用 `p J` 或二进制 GET_STATUS（有明确的
       `-32768` 哨兵 / bit3 标志）。
    """
    st = JointStatus(raw_text=r.text())
    for ln in r.lines:
        m = _STATUS_ROW.search(ln)
        if m:
            j = int(m.group(1))
            pos = float(m.group(3))
            mov_s = m.group(4).strip()
            mov = None if mov_s.startswith("Err") else mov_s.startswith("Yes")
            temp = int(m.group(5))
            while len(st.pos) < j:
                st.pos.append(None)
                st.moving.append(None)
                st.temp.append(0)
            st.pos[j - 1] = pos
            st.moving[j - 1] = mov
            st.temp[j - 1] = temp
    return st


@dataclass
class JointRead:
    """`p J` 结果。格式：  J1: raw=2048 servo=180.04 deg  rel-home=+0.04 deg"""
    index: int = 0
    raw: Optional[int] = None
    servo_deg: Optional[float] = None
    joint_deg: Optional[float] = None
    raw_text: str = ""


_JOINT_RE = re.compile(r"raw=(-?\d+)\s+servo=(-?[\d.]+)\s+deg\s+rel-home=([+-]?[\d.]+)\s+deg")


def parse_joint(r: Response, index: int = 0) -> JointRead:
    out = JointRead(index=index, raw_text=r.text())
    m = _JOINT_RE.search(r.raw or "")
    if m:
        out.raw = int(m.group(1))
        out.servo_deg = float(m.group(2))
        out.joint_deg = float(m.group(3))
    return out


_FK_RE = re.compile(r"FK:\s*x=(-?\d+)\s+y=(-?\d+)\s+z=(-?\d+)\s+mm\s+pitch=(-?\d+)\s+deg")


def parse_fk(lines: Sequence[str]) -> Optional[Pose]:
    """解析 `f`：
        FK: x=131 y=0 z=3 mm  pitch=150 deg
        servo  : J1=...
        relhome: J1=...

    ⚠ 返回 None = **读失败**（固件读失败时不打印任何行）。**不要用旧值兜底。**
    """
    for ln in lines:
        m = _FK_RE.search(ln)
        if m:
            return Pose(float(m.group(1)), float(m.group(2)), float(m.group(3)),
                        math.radians(float(m.group(4))))
    return None


@dataclass
class LimitsInfo:
    """`L` 表格： | J1 |  MIN |  MAX |  CUR |  （单位：整数舵机度）

    ⚠ MIN>MAX 是合法的（跨 0/360 的包裹区间），不要当成错误数据丢弃。
    """
    min_deg: List[Optional[float]] = field(default_factory=list)
    max_deg: List[Optional[float]] = field(default_factory=list)
    cur_deg: List[Optional[float]] = field(default_factory=list)
    raw_text: str = ""

    def approx_raw(self) -> Tuple[List[int], List[int]]:
        """由整数舵机度**近似**反算 raw 限位（误差 <= ~11 raw ≈ 1°）。

        仅用于本地 IK 预校验。**权威判定在固件侧。**
        """
        mn = [deg_to_servo_raw(v) if v is not None else 0 for v in self.min_deg]
        mx = [deg_to_servo_raw(v) if v is not None else 0 for v in self.max_deg]
        return mn, mx


_LIM_ROW = re.compile(r"\|\s*J(\d)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|")


def parse_limits(r: Response) -> LimitsInfo:
    info = LimitsInfo(raw_text=r.text())
    for ln in r.lines:
        m = _LIM_ROW.search(ln)
        if m:
            j = int(m.group(1))
            while len(info.min_deg) < j:
                info.min_deg.append(None)
                info.max_deg.append(None)
                info.cur_deg.append(None)
            info.min_deg[j - 1] = float(m.group(2))
            info.max_deg[j - 1] = float(m.group(3))
            info.cur_deg[j - 1] = float(m.group(4))
    return info


@dataclass
class PidCompare:
    """`N J` 结果。

        J3 cache PID = P:16 I:4 D:12
        J3 servo EEPROM = P:16 D:12 I:4  (一致)

    ⚠ EEPROM 行的字段顺序是 **P D I**（固件 printf 就是如此），解析时勿混。
    """
    joint: int = 1
    cache: Optional[Tuple[int, int, int]] = None      # (P, I, D)
    eeprom: Optional[Tuple[int, int, int]] = None     # (P, I, D) —— 已重排
    match: Optional[bool] = None
    eeprom_read_failed: bool = False
    raw_text: str = ""


_PID_CACHE = re.compile(r"J(\d) cache PID = P:(\d+) I:(\d+) D:(\d+)")
_PID_EE = re.compile(r"J(\d) servo EEPROM = P:(\d+) D:(\d+) I:(\d+)\s*(\(.*?\))?")


def parse_pid_compare(r: Response, joint: int = 1) -> PidCompare:
    out = PidCompare(joint=joint, raw_text=r.text())
    blob = r.text()
    m = _PID_CACHE.search(blob)
    if m:
        out.cache = (int(m.group(2)), int(m.group(3)), int(m.group(4)))   # P,I,D
    m = _PID_EE.search(blob)
    if m:
        # 固件输出顺序 P D I -> 归一化成 (P, I, D)
        p, d, i = int(m.group(2)), int(m.group(3)), int(m.group(4))
        out.eeprom = (p, i, d)
        tag = (m.group(5) or "")
        out.match = "一致" in tag and "不一致" not in tag
    elif "EEPROM read FAILED" in blob:
        out.eeprom_read_failed = True
    return out


@dataclass
class ArmParam:
    pid: int = 0
    name: str = ""
    value: float = 0.0
    vmin: float = 0.0
    vmax: float = 0.0
    unit: str = ""


_PARAM_RE = re.compile(
    r"#\s*(\d+)\s+(\S+)\s+([+-]?\d+\.\d+)\s*\[\s*([+-]?\d+\.\d+)\s*~\s*([+-]?\d+\.\d+)\]\s*(\S*)")


def parse_params(r: Response) -> List[ArmParam]:
    """解析 `LP` 输出：
        # 1 ik.converge_mm   +0.500 [ +0.100 ~ +5.000] mm
    """
    out: List[ArmParam] = []
    for ln in r.lines:
        m = _PARAM_RE.search(ln)
        if m:
            out.append(ArmParam(int(m.group(1)), m.group(2), float(m.group(3)),
                                float(m.group(4)), float(m.group(5)), m.group(6) or ""))
    return out


@dataclass
class ScanResult:
    online_ids: List[int] = field(default_factory=list)
    raw_text: str = ""


_SCAN_RE = re.compile(r"ID\s*[=:]\s*(\d+)|Found.*ID\s*(\d+)|^\s*\|\s*(\d+)\s*\|")


def parse_scan(r: Response) -> ScanResult:
    out = ScanResult(raw_text=r.text())
    for ln in r.lines:
        for m in _SCAN_RE.finditer(ln):
            v = next((g for g in m.groups() if g), None)
            if v:
                out.online_ids.append(int(v))
    return out


# ---- 遥测行（异步） ----

def parse_tm_line(line: str) -> Dict[str, List[Optional[float]]]:
    """解析一行 `TM <字段>:` 遥测。

        TM p:  180.0 180.0 ... 180.0
        TM t:   35  35 ...  35   (C)
        TM i:   12  12 ...  12   (mA)
        TM v:  12.0 ...          (V)
        TM s:  ...
        TM l:  ...

    缺失值以 `--` 出现 -> 对应元素为 None。
    返回 {"p": [...], "t": [...], ...}（单字段一行）。
    """
    key_map = {"p": "p", "t": "t", "i": "i", "v": "v", "s": "s", "l": "l"}
    if not line.startswith("TM "):
        return {}
    body = line[3:]
    if ":" not in body:
        return {}
    key, rest = body.split(":", 1)
    key = key.strip()
    if key not in key_map:
        return {}
    # 去掉尾部单位标记 (C) / (mA) / (V)
    rest = re.sub(r"\((?:C|mA|V)\)\s*$", "", rest.strip())
    vals: List[Optional[float]] = []
    for tok in rest.split():
        if tok == "--":
            vals.append(None)
        else:
            try:
                vals.append(float(tok))
            except ValueError:
                vals.append(None)
    return {key: vals}


# ---- 二进制状态 ----

@dataclass
class BinStatus:
    """GET_STATUS(0x02) 响应 0x82（21 字节）。"""
    joint_deg: List[Optional[float]] = field(default_factory=list)   # 舵机度，失败=None
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    pitch_deg: Optional[float] = None
    estop: bool = False
    traj_active: bool = False
    correction: bool = False
    read_ok: bool = False

    def pose(self) -> Optional[Pose]:
        """read_ok=False 或任一轴失败时返回 None —— **不做兜底**。"""
        if not self.read_ok or self.x is None or self.y is None or self.z is None:
            return None
        if any(v is None for v in self.joint_deg):
            return None
        return Pose(self.x, self.y, self.z, math.radians(self.pitch_deg or 0.0))


def parse_bin_status(pl: bytes) -> BinStatus:
    st = BinStatus()
    for j in range(6):
        v = i16(pl, j * 2)
        st.joint_deg.append(None if v == RAW_READ_FAIL else v / 10.0)
    st.x = i16(pl, 12) / 10.0
    st.y = i16(pl, 14) / 10.0
    st.z = i16(pl, 16) / 10.0
    st.pitch_deg = i16(pl, 18) / 10.0
    flags = pl[20]
    st.estop = bool(flags & 0x01)
    st.traj_active = bool(flags & 0x02)
    st.correction = bool(flags & 0x04)
    st.read_ok = bool(flags & 0x08)
    return st


# ============================================================================
# 6. 无硬件模拟器（**近似**实现，仅供离线开发 UI 用）
# ============================================================================

class SimTransport(Transport):
    """内存模拟器：实现常用命令的**近似**响应，让上位机可以离线开发。

    ⚠ 这不是仿真器：没有动力学、没有总线时序、没有真实限位。
      它只保证"命令 -> 响应格式"与固件一致，用来调 UI 解析逻辑。
      最终必须在真机上验证。
    """

    DEF_PID = ([16, 32, 16, 16, 16, 16], [4] * 6, [12] * 6)

    def __init__(self):
        self._out = bytearray()
        self._lock = threading.Lock()
        self._opened = False
        self._servo = [180.0] * 6              # 舵机度
        self._torque = [1] * 6
        self._pid_p, self._pid_i, self._pid_d = [list(x) for x in self.DEF_PID]
        self._limits = [(0, 360)] * 6
        self._params: Dict[int, Tuple[str, float, float, float, str]] = {
            1: ("ik.converge_mm", 0.5, 0.1, 5.0, "mm"),
            2: ("ik.max_iter", 100.0, 10.0, 500.0, ""),
            3: ("ik.fail_mm", 1.0, 0.5, 10.0, "mm"),
            4: ("ik.fk_verify_mm", 5.0, 1.0, 20.0, "mm"),
            16: ("sc.tj", 0.15, 0.02, 0.50, "s"),
            17: ("sc.def_vmax", 25.0, 5.0, 300.0, "deg/s"),
            18: ("sc.def_accel", 80.0, 10.0, 600.0, "deg/s2"),
            19: ("sc.v_min", 5.0, 1.0, 50.0, "deg/s"),
            20: ("sc.v_max", 300.0, 50.0, 300.0, "deg/s"),
            21: ("sc.accel_factor", 2.0, 1.0, 4.0, ""),
            32: ("gc.k", 0.0, 0.0, 0.5, "deg/mm"),
            33: ("corr.enabled", 1.0, 0.0, 1.0, ""),
            34: ("corr.threshold", 2.0, 0.5, 10.0, "deg"),
            35: ("corr.max_rounds", 10.0, 1.0, 20.0, ""),
            42: ("traj.approach_speed", 30.0, 0.0, 4095.0, ""),
            43: ("traj.approach_frac", 0.05, 0.0, 0.25, ""),
            44: ("servo.accel", 10.0, 0.0, 254.0, ""),
            45: ("traj.handoff_deg", 0.0, 0.0, 15.0, "deg"),
        }
        self._tm_period = 0
        self._tm_next = 0.0
        self._estop = False
        self._waypoints: Dict[int, List[float]] = {}   # q 存的路点（1~8，舵机度）
        self._loop_active = False                      # B / QL 是否在跑
        self._parser = FrameParser()
        self._seq = 0

    # --- Transport 接口 ---
    def open(self) -> None:
        self._opened = True
        self._out += b"\r\n  ===== ArmControl_HAL (SIMULATED) =====\r\n"
        self._out += b"  Binary proto v1 ready (0xAA 0x55 frame). 'LP' lists params.\r\n> "

    def close(self) -> None:
        self._opened = False

    @property
    def is_open(self) -> bool:
        return self._opened

    def write(self, data: bytes) -> None:
        if not self._opened:
            raise RuntimeError("not open")
        with self._lock:
            for b in data:
                res = self._parser.feed(b)
                if res is None:
                    continue
                tag, val = res
                if tag == "text":
                    self._feed_text(val)
                elif tag == "frame":
                    self._out += self._handle_frame(*val)
                elif tag == "crc_err":
                    seq, cmd, _ = val
                    self._out += build_frame(seq, RSP_NACK, bytes([cmd, 2]))

    def read(self, n: int = 256) -> bytes:
        time.sleep(0.01)
        with self._lock:
            # 模拟 tm 周期性输出
            now = time.monotonic()
            if self._tm_period and now >= self._tm_next:
                self._tm_next = now + self._tm_period / 1000.0
                for k, unit, fmt in (("p", "", "%5.1f"), ("t", "  (C)", "%4d"),
                                     ("i", "  (mA)", "%4d")):
                    if k == "t":
                        self._out += ("TM t:" + "".join(f" {35:4d}" for _ in range(6))
                                      + "   (C)\r\n").encode()
                    elif k == "i":
                        self._out += ("TM i:" + "".join(f" {12:4d}" for _ in range(6))
                                      + "   (mA)\r\n").encode()
                    else:
                        self._out += ("TM p:" + "".join(f" {v:5.1f}" for v in self._servo)
                                      + "\r\n").encode()
            chunk = bytes(self._out[:n])
            del self._out[:n]
            return chunk

    # --- 文本命令处理 ---
    def _feed_text(self, byte: int) -> None:
        if not hasattr(self, "_line"):
            self._line = bytearray()
        if byte in (0x0D, 0x0A):
            line = bytes(self._line).decode("ascii", "replace")
            self._line.clear()
            if line:
                self._exec(line)
            return
        self._line.append(byte)
        self._out += bytes([byte])          # 回显

    def _exec(self, line: str) -> None:
        toks = line.split()
        cmd = toks[0]
        self._out += b"\r\n"
        try:
            self._exec_one(cmd, toks)
        except Exception as e:              # 模拟器绝不崩，方便调 UI
            self._out += f"  [SIM] internal error: {e}\r\n".encode()
        self._out += b"> "

    def _exec_one(self, cmd: str, toks: List[str]) -> None:
        P = lambda s: setattr(self, "_out", self._out + s.encode())

        # 固件: 有循环在跑时, 其它运动命令会被拒 (除非先发 S)
        if self._loop_active and cmd in ("m", "a", "g", "P", "W", "B", "QL", "BG"):
            P("  Another loop is active. Use S first.\r\n")
            return

        if cmd == "h":
            P("  ===== Command Help =====\r\n"
              "  | [LP]       List all algo params      |\r\n"
              "  | [SP id v]  Set param (see LP ids)    |\r\n")
        elif cmd == "s":
            P("\r\n  +----+----+-------+--------+------+\r\n")
            P("  |Jnt | ID | Angle | Moving | Temp |\r\n")
            P("  +----+----+-------+--------+------+\r\n")
            for j in range(6):
                P(f"  | J{j+1} | {j+1}  | {int(self._servo[j]):3d}.0 |   No     |  35C |\r\n")
            P("  +----+----+-------+--------+------+\r\n")
        elif cmd == "p" and len(toks) >= 2:
            j = int(float(toks[1]))
            raw = deg_to_servo_raw(self._servo[j])
            rel = -1.0 * (self._servo[j] - 180.0) if j == 0 else (self._servo[j] - 180.0)
            P(f"  J{j+1}: raw={raw} servo={self._servo[j]:.2f} deg  rel-home={rel:+.2f} deg\r\n")
        elif cmd in ("pv", "pf") and len(toks) >= 2:
            ms = int(toks[1])
            if ms <= 0:
                self._out += b"  Real-time angle feedback stopped.\r\n"
                self._tm_period = 0
            else:
                self._tm_period = max(ms, 10) / 1000.0
                self._tm_next = time.monotonic()
                P(f"  Real-time angle feedback: every {max(ms,10)} ms. Use pv 0 to stop.\r\n")
        elif cmd == "f":
            p = fk_from_servo_deg(self._servo)
            P(f"  FK: x={int(p.x)} y={int(p.y)} z={int(p.z)} mm  pitch={int(round(p.pitch_deg))} deg\r\n")
            P("  servo  : " + " ".join(f"J{j+1}={self._servo[j]:.2f}" for j in range(6)) + " deg\r\n")
            P("  relhome: " + " ".join(
                f"J{j+1}={(self._servo[j]-180.0)*((-1) if j==0 else 1):+.2f}" for j in range(6)) + " deg\r\n")
        elif cmd == "L" and len(toks) < 4:
            P("\r\n  Joint Safety Limits:\r\n")
            P("  +----+------+------+------+\r\n  | J  | MIN  | MAX  | CUR  |\r\n  +----+------+------+------+\r\n")
            for j in range(6):
                P(f"  | J{j+1} | {int(self._limits[j][0]):4d} | {int(self._limits[j][1]):4d} "
                  f"| {int(self._servo[j]):4d} |\r\n")
            P("  +----+------+------+------+\r\n")
            P("  Usage: L <J> <min> <max>  (e.g. L 0 80 220)\r\n")
        elif cmd == "L" and len(toks) >= 4:
            j = int(float(toks[1]))
            mn, mx = float(toks[2]), float(toks[3])
            self._limits[j] = (mn, mx)
            P(f"  J{j+1} limits set: {int(mn)} ~ {int(mx)} deg "
              f"(raw {deg_to_servo_raw(mn)} ~ {deg_to_servo_raw(mx)})\r\n")
        elif cmd in ("K", "I", "D"):
            arr = {"K": self._pid_p, "I": self._pid_i, "D": self._pid_d}[cmd]
            letter = cmd
            if len(toks) == 1:
                P(f"  {letter}-Gain: " + " ".join(f"J{j+1}={arr[j]}" for j in range(6)) + "\r\n")
            elif len(toks) == 2:
                j = int(float(toks[1]))
                P(f"  J{j} {letter}-Gain = {arr[j-1]}\r\n")
            else:
                j, v = int(float(toks[1])), int(float(toks[2]))
                arr[j - 1] = v
                P(f"  J{j} {letter}-Gain = {v}\r\n")
                P(f"  [cache only] send 'Y {j}' to burn into servo EEPROM\r\n")
        elif cmd == "N" and len(toks) == 5:
            j, p, i, d = (int(float(toks[k])) for k in range(1, 5))
            self._pid_p[j - 1], self._pid_i[j - 1], self._pid_d[j - 1] = p, i, d
            P(f"  J{j} PID = P:{p} I:{i} D:{d}\r\n")
            P(f"  [cache only] send 'Y {j}' to burn into servo EEPROM\r\n")
        elif cmd == "N" and len(toks) == 2:
            j = int(float(toks[1]))
            k = j - 1
            P(f"  J{j} cache PID = P:{self._pid_p[k]} I:{self._pid_i[k]} D:{self._pid_d[k]}\r\n")
            P(f"  J{j} servo EEPROM = P:{self._pid_p[k]} D:{self._pid_d[k]} "
              f"I:{self._pid_i[k]}  (一致)\r\n")
        elif cmd == "Y":
            j = int(float(toks[1]))
            P(f"  Burned PID to servo EEPROM: {'all' if j == 0 else 'J'+str(j)}\r\n")
            P("  (servo lost torque ~0.6s during write)\r\n")
        elif cmd == "X":
            self._pid_p, self._pid_i, self._pid_d = [list(x) for x in self.DEF_PID]
            P("  Default PID restored (RAM only).\r\n")
        elif cmd == "F":
            P("  Saved to Flash (sector 11). Power-cycle safe.\r\n")
        elif cmd == "LP":
            P(f"  === Algorithm Params (arm_params, {len(self._params)} entries) ===\r\n")
            P("  id  name             value       range             unit\r\n")
            for k in sorted(self._params):
                name, v, vmin, vmax, unit = self._params[k]
                P(f"  #{k:2d} {name:<16s} {v:+.3f} [{vmin:+.3f} ~ {vmax:+.3f}] {unit}\r\n")
            P("  Set: SP <id> <value>\r\n")
        elif cmd == "SP" and len(toks) >= 3:
            pid, v = int(float(toks[1])), float(toks[2])
            if pid in self._params:
                name, _, vmin, vmax, unit = self._params[pid]
                v = min(max(v, vmin), vmax)
                self._params[pid] = (name, v, vmin, vmax, unit)
                P(f"  Param #{pid} {name} = {v:+.3f} {unit}\r\n")
                P("  [RAM only] send 'F' to save to Flash\r\n")
            else:
                P(f"  Unknown param id {pid} (LP lists ids)\r\n")
        elif cmd == "m" and len(toks) >= 7:
            # 固件: 数值先被**静默**硬钳到 0~300, 再走 cmdMove
            for j in range(6):
                v = float(toks[1 + j])
                self._servo[j] = 0.0 if v < 0 else (300.0 if v > 300 else v)
            v_max = float(toks[7]) if len(toks) >= 8 else 25.0   # ARM_SC_DEF_VMAX
            v_max = min(max(v_max, 5.0), 300.0)
            P("  Move started (6 joints).\r\n  Move reached: actual feedback settled.\r\n")
            P(f"  [SIM] target=" + " ".join(f"{v:.2f}" for v in self._servo)
              + f"  v_max={v_max:g} deg/s\r\n")
        elif cmd == "a" and len(toks) >= 3:
            j = int(float(toks[1]))
            if not 0 <= j <= 5:
                P("  Joint must be 0-5\r\n")
            else:
                deg = float(toks[2])
                raw = deg_to_servo_raw(deg)          # 内部: <0 -> 0, >360 -> 360
                folded = servo_raw_to_deg(raw)
                mn, mx = self._limits[j]
                target = folded
                if not (mn <= folded <= mx):         # 与 clamp_angle 同形回显
                    target = mn if folded < mn else mx
                    P(f"  J{j+1}: {int(deg)} -> {int(target)} deg "
                      f"(raw {raw} -> {deg_to_servo_raw(target)} limit)\r\n")
                self._servo[j] = target
                P(f"  J{j+1} -> {target:.2f} deg: OK\r\n")
        elif cmd == "q" and len(toks) >= 8:
            idx = int(float(toks[1]))
            if not 1 <= idx <= 8:
                P(f"  Point index must be 1~8\r\n")
            else:
                pts = []
                for j in range(6):
                    deg = float(toks[2 + j])
                    mn, mx = self._limits[j]
                    pts.append(min(max(deg, mn), mx))
                self._waypoints[idx] = pts
                P(f"  Point {idx} saved: " + " ".join(str(int(v)) for v in pts) + "\r\n")
        elif cmd == "q" and len(toks) < 8:
            P("  Usage: q <1~8> <j1> <j2> <j3> <j4> <j5> <j6>\r\n")
        elif cmd == "B" and len(toks) >= 14:
            a = [min(max(float(toks[1 + j]), 0.0), 360.0) for j in range(6)]
            b = [min(max(float(toks[7 + j]), 0.0), 360.0) for j in range(6)]
            rounds = int(float(toks[13]))
            speed = float(toks[14]) if len(toks) >= 15 else 30.0
            speed = min(max(speed, 5.0), 300.0)
            self._loop_active = True
            P("  Point loop: A->B->A, rounds=%s, speed=%d deg/s\r\n"
              % ("infinite" if rounds == 0 else "finite", int(speed)))
        elif cmd == "QL" and len(toks) >= 3:
            n, rounds = int(float(toks[1])), int(float(toks[2]))
            speed = float(toks[3]) if len(toks) >= 4 else 25.0
            if not 1 <= n <= 8:
                P("  Multi-point count must be 1~8\r\n")
            elif n > len(self._waypoints):
                P(f"  Only {len(self._waypoints)} points saved; use 'q' first.\r\n")
            else:
                self._loop_active = True
                P("  Multi-point loop: %d points, rounds=%s, speed=%d deg/s\r\n"
                  % (n, "infinite" if rounds == 0 else "finite", int(speed)))
        elif cmd == "S":
            self._loop_active = False
            P("  Stopped.\r\n")
        elif cmd == "g" and len(toks) >= 4:
            x, y, z = (float(toks[k]) for k in range(1, 4))
            pitch = float(toks[4]) if len(toks) >= 5 else 0.0
            try:
                from arm_fk import ik_checked
                mn, mx = [0] * 6, [4095] * 6
                cr = ik_checked(x, y, z, pitch, min_raw=mn, max_raw=mx)
            except Exception:
                cr = None
            if cr is None or not cr.ok:
                P(f"  IK: unreachable! (x={int(x)} y={int(y)} z={int(z)})\r\n")
            else:
                sv = cr.servo_deg
                P(f"  IK: x={int(x)} y={int(y)} z={int(z)} -> "
                  + " ".join(f"J{j+1}={int(sv[j])}" for j in range(6)) + "\r\n")
                P(f"  Verify FK: x={int(x)} y={int(y)} z={int(z)} (err: 0 mm)\r\n")
                P(f"  Pitch: req={int(pitch)} got={int(pitch)} (err +0 deg)\r\n")
                for j in range(6):
                    self._servo[j] = sv[j]
                P("  Move started.\r\n")
        elif cmd == "P" and len(toks) >= 2:
            P(f"  Pose {int(float(toks[1]))} (SIM): 180 180 180 180 180 180\r\n")
            P("  Move started.\r\n")
        elif cmd == "S":
            self._loop_active = False
            P("  All motion stopped.\r\n")
        elif cmd == "t" and len(toks) >= 3:
            on = int(float(toks[2]))
            P(f"  J{int(float(toks[1]))+1} Torque {'ON' if on else 'OFF'}: OK\r\n")
        elif cmd == "z":
            P("  Homing... done\r\n")
        elif cmd.startswith("tm"):
            if len(toks) >= 2 and int(toks[1]) == 0:
                self._tm_period = 0
                P("  Telemetry stopped.\r\n")
            else:
                self._tm_period = max(int(toks[1]) if len(toks) >= 2 else 200, 10) / 1000.0
                self._tm_next = time.monotonic()
                P("  Telemetry started.\r\n")
        elif cmd == "n":
            P("  Scanning Bus A... 3 servo(s) found\r\n")
            P("  Scanning Bus B... 3 servo(s) found\r\n")
            P("  Total: 6 / 6 servos online\r\n")
        elif cmd == "i":
            P("  Init all servos... done\r\n  Done. All servos set to joint mode, torque ON (holding).\r\n")
        elif cmd == "M":
            P("  All servos centered (raw 2048 = middle).\r\n")
        elif cmd == "LL":
            P("  RunLog level = 2 (0=off 1=err 2=warn 3=info 4=debug)\r\n")
        else:
            P(f"  Unknown command: {cmd} (try 'h')\r\n")

    # --- 二进制帧处理 ---
    def _handle_frame(self, seq: int, cmd: int, payload: bytes) -> bytes:
        if cmd == CMD_PING:
            return build_frame(seq, 0x81, bytes([1, 7]))
        if cmd == CMD_GET_STATUS:
            pl = bytearray()
            for v in self._servo:
                pl += i16_bytes(int(round(v * 10)))
            p = fk_from_servo_deg(self._servo)
            pl += i16_bytes(int(round(p.x * 10)))
            pl += i16_bytes(int(round(p.y * 10)))
            pl += i16_bytes(int(round(p.z * 10)))
            pl += i16_bytes(int(round(p.pitch_deg * 10)))
            pl.append(0x08)                      # bit3 = read ok
            return build_frame(seq, 0x82, bytes(pl))
        if cmd == CMD_FK_QUERY:
            pl = bytearray()
            p = fk_from_servo_deg(self._servo)
            pl += i16_bytes(int(round(p.x * 10)))
            pl += i16_bytes(int(round(p.y * 10)))
            pl += i16_bytes(int(round(p.z * 10)))
            pl += i16_bytes(int(round(p.pitch_deg * 10)))
            for v in self._servo:
                pl += i16_bytes(int(round(v * 10)))
            pl.append(1)
            return build_frame(seq, 0xA1, bytes(pl))
        if cmd == CMD_MOVE_JOINTS and len(payload) == 16:
            for j in range(6):
                self._servo[j] = i16(payload, j * 2) / 10.0
            return build_frame(seq, RSP_ACK, bytes([cmd, 0]))
        if cmd == CMD_MOVE_CART and len(payload) == 12:
            return build_frame(seq, RSP_ACK, bytes([cmd, 0]))
        if cmd == CMD_STOP:
            return build_frame(seq, RSP_ACK, bytes([cmd, 0]))
        if cmd == CMD_PARAM_COUNT:
            return build_frame(seq, 0xC2, bytes([len(self._params)]))
        if cmd == CMD_PARAM_READ and len(payload) >= 1:
            pid = payload[0]
            v = self._params.get(pid, ("", 0.0, 0, 0, ""))[1]
            return build_frame(seq, 0xC0, bytes([pid]) + i32_bytes(int(round(v * 1000))))
        if cmd == CMD_PARAM_WRITE and len(payload) >= 5:
            pid = payload[0]
            v = i32(payload, 1) / 1000.0
            if pid in self._params:
                name, _, vmin, vmax, unit = self._params[pid]
                self._params[pid] = (name, min(max(v, vmin), vmax), vmin, vmax, unit)
                return build_frame(seq, RSP_ACK, bytes([cmd, 0]))
            return build_frame(seq, RSP_NACK, bytes([cmd, 1]))
        return build_frame(seq, RSP_NACK, bytes([cmd, 1]))
