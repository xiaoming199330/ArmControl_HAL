# -*- coding: utf-8 -*-
"""
transport.py — 传输层：把逻辑包送到线上 / 从线上收回逻辑包

三种承载各有一个 Transport：

    NetTransport("192.168.7.120", 5000)   UDP，**首选**（实测 p50 1.61 ms，三链路最快）
    SerialTransport("/dev/ttyUSB0")       串口二进制（AA 55 帧）+ 文本控制台透传
    SimArmTransport()                     纯内存假臂（离线开发 / 自检，不需要硬件）

统一接口（`Transport`）：

    open() / close() / is_open
    send(seq, cmd, payload)               发一个逻辑包
    recv_event(timeout) -> RxEvent        收一个事件（packet / bad / text / timeout）

为什么用「事件」而不是直接返回包：串口上同时有**二进制帧**和**设备日志文本**，
把"坏包""超时""设备日志"分开上报，上层才能既不丢信息又不误判。
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from .protocol import (Carrier, FrameStreamParser, ProtocolError, SerialCarrier,
                       UdpCarrier, pack_logical)


@dataclass
class RxEvent:
    """一次接收事件。"""
    kind: str                              # "packet" | "bad" | "text" | "timeout"
    seq: int = -1
    cmd: int = -1
    payload: bytes = b""
    error: str = ""
    line: str = ""                         # kind=="text" 时的文本行
    raw: bytes = b""

    def __str__(self) -> str:
        if self.kind == "packet":
            return "packet(seq=%d cmd=0x%02X len=%d)" % (self.seq, self.cmd, len(self.payload))
        if self.kind == "bad":
            return "bad(%s)" % self.error
        if self.kind == "text":
            return "text(%r)" % self.line
        return "timeout"


# ============================================================================
# 基类
# ============================================================================

class Transport:
    carrier: Carrier = UdpCarrier()
    name = "abstract"

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    @property
    def is_open(self) -> bool:
        return False

    def send(self, seq: int, cmd: int, payload: bytes = b"") -> None:
        raise NotImplementedError

    def recv_event(self, timeout: float = 0.2) -> RxEvent:
        raise NotImplementedError


# ============================================================================
# UDP 网口（首选）
# ============================================================================

class NetTransport(Transport):
    """UDP 承载。固件监听 `ip:port`（默认 192.168.7.120:5000）。

    ⚠ 三个现场要点：
      · 回包发往**请求的源 IP:源端口** —— 所以本端不绑 5000，用临时端口即可。
      · 目的端口不是 5000 ⇒ 固件直接丢弃，什么都不回（只表现为超时）。
      · UDP 校验和固件写 0（IPv4 允许），抓包看到 0 是正常的。
    """

    carrier = UdpCarrier()
    name = "net"

    def __init__(self, ip: str = "192.168.7.120", port: int = 5000,
                 local_ip: str = "", timeout: float = 0.3):
        self.ip = ip
        self.port = int(port)
        self.local_ip = local_ip
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None

    def open(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.local_ip, 0))          # 临时源端口；回包按源端口回来
        s.settimeout(self.timeout)
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def send(self, seq: int, cmd: int, payload: bytes = b"") -> None:
        if self._sock is None:
            raise RuntimeError("NetTransport 未打开")
        self._sock.sendto(pack_logical(seq, cmd, payload), (self.ip, self.port))

    def recv_event(self, timeout: float = 0.2) -> RxEvent:
        if self._sock is None:
            return RxEvent("timeout")
        self._sock.settimeout(max(0.0, timeout))
        try:
            data, _addr = self._sock.recvfrom(2048)
        except socket.timeout:
            return RxEvent("timeout")
        except OSError as e:
            return RxEvent("bad", error="socket: %s" % e)
        try:
            seq, cmd, pl = self.carrier.parse(data)
            return RxEvent("packet", seq=seq, cmd=cmd, payload=pl, raw=data)
        except ProtocolError as e:
            return RxEvent("bad", error=str(e), raw=data)

    def drain(self, max_n: int = 64) -> int:
        """清空接收缓冲（发新命令前用，避免迟到回包串话）。返回丢掉的包数。"""
        if self._sock is None:
            return 0
        n = 0
        self._sock.settimeout(0.0)
        try:
            for _ in range(max_n):
                try:
                    self._sock.recvfrom(2048)
                    n += 1
                except (socket.timeout, BlockingIOError, OSError):
                    break
        finally:
            pass
        return n


# ============================================================================
# 串口（二进制 AA55 帧 + 文本日志）
# ============================================================================

class SerialTransport(Transport):
    """串口承载（pyserial）。同一端口上既有二进制帧也有控制台文本。

    ⚠ 115200 8N1、无流控、打开后清缓冲。
    ⚠ **不要**用本类发文本命令（那会与二进制帧混用）；文本控制台请用
       `console.ConsoleClient`（内部自带按提示符分帧的状态机）。
       本类只做二进制收发 + 把设备日志文本"顺带"上报，供排错。
    """

    carrier = SerialCarrier()
    name = "serial"

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._ser = None
        self._parser = FrameStreamParser()
        self._text_buf = bytearray()
        self._text_q = []

    def open(self) -> None:
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise RuntimeError("缺少 pyserial。请先安装：pip install pyserial") from e
        self._ser = serial.Serial(
            port=self.port, baudrate=self.baudrate, bytesize=8, parity="N", stopbits=1,
            timeout=0.02, write_timeout=1.0, xonxoff=False, rtscts=False, dsrdtr=False)
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

    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def send(self, seq: int, cmd: int, payload: bytes = b"") -> None:
        if self._ser is None:
            raise RuntimeError("SerialTransport 未打开")
        self._ser.write(self.carrier.build(seq, cmd, payload))

    def recv_event(self, timeout: float = 0.2) -> RxEvent:
        if self._ser is None:
            return RxEvent("timeout")
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            if self._text_q:
                return RxEvent("text", line=self._text_q.pop(0))
            remain = deadline - time.monotonic()
            if remain <= 0:
                return RxEvent("timeout")
            try:
                chunk = self._ser.read(256)
            except Exception:
                chunk = b""
            if not chunk:
                continue
            for byte in chunk:
                res = self._parser.feed(byte)
                if res is None:
                    continue
                tag, val = res
                if tag == "frame":
                    seq, cmd, pl = val
                    return RxEvent("packet", seq=seq, cmd=cmd, payload=pl)
                if tag == "crc_err":
                    seq, cmd, pl = val
                    return RxEvent("bad", seq=seq, cmd=cmd, payload=pl, error="CRC 错")
                if tag == "text":
                    b = val
                    if b in (0x0D, 0x0A):
                        if self._text_buf:
                            self._text_q.append(self._text_buf.decode("utf-8", "replace"))
                            self._text_buf.clear()
                    else:
                        self._text_buf.append(b)
                        if len(self._text_buf) > 512:
                            self._text_buf.clear()


# ============================================================================
# 选择器
# ============================================================================

def make_transport(kind: str, **kw) -> Transport:
    """按名字建 Transport。

    kind: "net" | "udp" | "serial" | "sim"
    """
    k = (kind or "").lower()
    if k in ("net", "udp"):
        return NetTransport(kw.get("ip", "192.168.7.120"), int(kw.get("port", 5000)),
                            kw.get("local_ip", ""), float(kw.get("timeout", 0.3)))
    if k == "serial":
        return SerialTransport(kw["port"], int(kw.get("baudrate", 115200)))
    if k in ("sim", "simulated", "mock"):
        from .sim import SimArmTransport
        return SimArmTransport(**kw)
    raise ValueError("未知 transport: %r（可选 net/udp/serial/sim）" % kind)


__all__ = ["RxEvent", "Transport", "NetTransport", "SerialTransport", "make_transport"]
