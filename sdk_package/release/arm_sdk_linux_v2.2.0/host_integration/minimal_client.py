#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
minimal_client.py — 网口协议**最小客户端**（只用 Python 标准库）

三个用途：

1. **给上位机工程师当参照物** —— 这个文件里**没有 import arm_sdk**，
   协议是照着文档从零手写的（CRC16、组包、解包、文本分页全在里面）。
   C# / C / C++ / Java / Go 的实现只要和它对得上，就对得上固件。

2. **现场排查工具** —— 拿一台裸 Python 的电脑就能验一条命令：
       python minimal_client.py --ip 192.168.7.120 --smoke
       python minimal_client.py --ip 192.168.7.120 --text "s"
       python minimal_client.py --ip 192.168.7.120 --text "h" --text "zi"

3. **无真机也能自检** —— 先起假臂，再对着它跑：
       armctl mock --port 5000            # 终端 1
       python minimal_client.py --ip 127.0.0.1 --smoke      # 终端 2  → 应 5/5 通过

   也可以完全离线验协议本身（不碰网络）：
       python minimal_client.py --selftest

⚠ 这个文件故意**不依赖 arm_sdk** —— 别为了"少写几行"改成 import。
   它的价值就在于"独立实现"，这样两家实现不一致时才能发现。
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# ============================================================ 协议常量
PORT = 5000
MAX_PAYLOAD = 64
MAGIC = 0x5A5A5A5A
RAW_READ_FAIL = -32768            # GET_STATUS 读失败哨兵（不是 0！）
TEMP_LOST = 0xFF
HWERR_LOST = 0xFF
TEXT_BUF_SZ = 2048
TEXT_EXEC_MAX = 58
TEXT_CHUNK = 58
TEXT_FLAG_TRUNC = 0x01
TEXT_FLAG_GATE = 0x02

# 命令码（38 条）
PING, GET_STATUS, LOG_SET_LEVEL = 0x01, 0x02, 0x03
GET_JOINT_DETAIL, GET_LIMITS, GET_INFO, GET_FLAGS = 0x04, 0x05, 0x07, 0x0A
TEXT_EXEC, TEXT_FETCH = 0x0B, 0x0C
MOVE_JOINTS, STOP, SET_TORQUE, STREAM_JOINTS = 0x10, 0x11, 0x12, 0x13
MOVE_JOINT_ONE, SET_BUS_MODE, SERVO_INIT = 0x14, 0x17, 0x18
MOVE_CART, FK_QUERY, IK_QUERY = 0x20, 0x21, 0x22
SET_GRAVITY, SET_CORRECTION, GET_GRAVITY, GET_CORRECTION = 0x30, 0x31, 0x32, 0x33
GET_PID = 0x36
PARAM_READ, PARAM_WRITE, PARAM_COUNT, PARAM_SAVE, PARAM_LIST = 0x40, 0x41, 0x42, 0x43, 0x44
TEACH_LIST, TEACH_READ, TEACH_DELETE, TEACH_COMPACT, TEACH_TOTAL = 0x50, 0x51, 0x53, 0x54, 0x55
CENTER_CALIB, CALIB_ENTER, ZERO_RESET, SYS_RESET = 0x70, 0x71, 0x72, 0x73

CMD_NAME = {
    PING: "PING", GET_STATUS: "GET_STATUS", LOG_SET_LEVEL: "LOG_SET_LEVEL",
    GET_JOINT_DETAIL: "GET_JOINT_DETAIL", GET_LIMITS: "GET_LIMITS",
    GET_INFO: "GET_INFO", GET_FLAGS: "GET_FLAGS",
    TEXT_EXEC: "TEXT_EXEC", TEXT_FETCH: "TEXT_FETCH",
    MOVE_JOINTS: "MOVE_JOINTS", STOP: "STOP", SET_TORQUE: "SET_TORQUE",
    STREAM_JOINTS: "STREAM_JOINTS", MOVE_JOINT_ONE: "MOVE_JOINT_ONE",
    SET_BUS_MODE: "SET_BUS_MODE", SERVO_INIT: "SERVO_INIT",
    MOVE_CART: "MOVE_CART", FK_QUERY: "FK_QUERY", IK_QUERY: "IK_QUERY",
    SET_GRAVITY: "SET_GRAVITY", SET_CORRECTION: "SET_CORRECTION",
    GET_GRAVITY: "GET_GRAVITY", GET_CORRECTION: "GET_CORRECTION",
    GET_PID: "GET_PID",
    PARAM_READ: "PARAM_READ", PARAM_WRITE: "PARAM_WRITE",
    PARAM_COUNT: "PARAM_COUNT", PARAM_SAVE: "PARAM_SAVE", PARAM_LIST: "PARAM_LIST",
    TEACH_LIST: "TEACH_LIST", TEACH_READ: "TEACH_READ", TEACH_DELETE: "TEACH_DELETE",
    TEACH_COMPACT: "TEACH_COMPACT", TEACH_TOTAL: "TEACH_TOTAL",
    CENTER_CALIB: "CENTER_CALIB", CALIB_ENTER: "CALIB_ENTER",
    ZERO_RESET: "ZERO_RESET", SYS_RESET: "SYS_RESET",
}
RSP_NAME = {
    0x7F: "ACK", 0x7E: "NACK",
    0x81: "RSP_PING", 0x82: "RSP_STATUS", 0x84: "RSP_JOINT_DETAIL",
    0x85: "RSP_LIMITS", 0x87: "RSP_INFO", 0x8A: "RSP_FLAGS",
    0x8B: "RSP_TEXT_OUT", 0x8C: "RSP_TEXT_FETCH",
    0xA1: "RSP_FK", 0xA2: "RSP_IK", 0xB2: "RSP_GRAVITY",
    0xB3: "RSP_CORRECTION", 0xB6: "RSP_PID",
    0xC0: "RSP_PARAM_READ", 0xC2: "RSP_PARAM_COUNT", 0xC4: "RSP_PARAM_LIST",
    0xD0: "RSP_TEACH_LIST", 0xD1: "RSP_TEACH_READ", 0xD5: "RSP_TEACH_TOTAL",
}

ERR_NAME = {0: "OK", 1: "PARAM", 2: "CRC", 3: "UNREACH", 4: "LIMIT", 5: "ESTOP",
            6: "FK_VERIFY", 7: "READ", 8: "MAGIC", 9: "BUSY", 10: "NOTCAL"}
ERR_HINT = {
    1: "参数非法（量纲用错 / 越界 / 枚举无此值）",
    2: "CRC 或长度错",
    3: "IK 无解 / 目标不可达",
    4: "越限（含单帧跳变 >25° 被整帧拒收）",
    5: "急停生效，现在不能动",
    6: "上报位姿与正解不符，安全拒动",
    7: "舵机读失败（有轴丢读）",
    8: "危险命令缺/错 MAGIC",
    9: "轨迹/示教/回放进行中，此刻不受理",
    10: "未校准",
}


# ============================================================ 协议实现
def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF，非反射，无最终异或。

    自检：crc16(b"123456789") == 0x29B1  ← 写任何语言都先过这一条
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def pack(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    """组逻辑包：SEQ(1) CMD(1) LEN(1) PAYLOAD(LEN) CRC_LO CRC_HI。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload 超长 %d > %d" % (len(payload), MAX_PAYLOAD))
    body = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + bytes(payload)
    c = crc16(body)
    return body + bytes([c & 0xFF, (c >> 8) & 0xFF])


def unpack(data: bytes):
    """解逻辑包 → (seq, cmd, payload)。长度或 CRC 不符抛 ValueError。"""
    d = bytes(data)
    if len(d) < 5:
        raise ValueError("包太短 %d" % len(d))
    ln = d[2]
    if ln > MAX_PAYLOAD:
        raise ValueError("LEN=%d 超上限" % ln)
    if len(d) != 3 + ln + 2:
        raise ValueError("长度不符：实收 %d，LEN=%d 要求 %d" % (len(d), ln, 3 + ln + 2))
    got = d[3 + ln] | (d[3 + ln + 1] << 8)
    want = crc16(d[:3 + ln])
    if got != want:
        raise ValueError("CRC 错 got 0x%04X want 0x%04X" % (got, want))
    return d[0], d[1], d[3:3 + ln]


def enc(v: float) -> int:
    """实际值 → i16（四舍五入**远离零**，与固件一致）。"""
    return int(v + 0.5) if v >= 0 else int(v - 0.5)


def encu(v: float) -> int:
    """实际值 → u16，负数先钳到 0（速度/加速度的固件语义）。"""
    v = 0.0 if v < 0 else v
    return int(v + 0.5)


def rd_i16(b: bytes, o: int) -> int:
    return struct.unpack_from("<h", b, o)[0]


def rd_u16(b: bytes, o: int) -> int:
    return struct.unpack_from("<H", b, o)[0]


# ------------------------------------------------------------ 载荷构造
def pl_move_joints(servo_deg6, v=0, a=0) -> bytes:
    """绝对角，**舵机度**（不是逻辑角）。

    ⚠ v/a 是 **u16 且乘 10**：传 100 → 线上写 1000。0 = 用臂内默认。
      写成 i16 或不乘 10 会让臂"几乎不动"或"满速冲"，而且**不报错**。
    """
    return b"".join(struct.pack("<h", enc(x * 10.0)) for x in servo_deg6) \
        + struct.pack("<HH", encu(v * 10.0), encu(a * 10.0))


def pl_move_cart(x, y, z, pitch=0.0, v=0.0, a=0.0) -> bytes:
    """笛卡尔 mm/deg。⚠ 会归零 J5/J6；坐标原点是夹爪中心 TCP。"""
    return struct.pack("<hhhh", enc(x * 10.0), enc(y * 10.0), enc(z * 10.0),
                       enc(pitch * 10.0)) + struct.pack("<HH", encu(v * 10.0),
                                                        encu(a * 10.0))


def pl_stream_joints(delta_deg6) -> bytes:
    """流控帧，**逻辑角增量**（度）。⚠ 单帧单轴 >25° 会被整帧拒收。"""
    return b"".join(struct.pack("<h", enc(x * 10.0)) for x in delta_deg6)


def pl_set_torque(joint: int, enable: bool) -> bytes:
    return bytes([joint & 0xFF, 1 if enable else 0])


def pl_text_exec(line: str) -> bytes:
    """TEXT_EXEC：u32 magic + **ASCII** 正文 + **NUL(0x00)**。

    ⚠ 三个容易错的点：
      1. 末尾必须有 NUL —— 固件按 C 字符串读，少了会读到上一次的残留
      2. 正文必须 **ASCII**（固件 encode("ascii")，非 ASCII 直接报错）
      3. 上限 58 是**字符数**；超长要报错，**不能静默截断**
         （截断后可能变成一条完全不同的命令，比如 `sf 1` → `sf`）
    结尾的 CR/LF 会被剥掉 —— ProcessCommand 只吃一行内容。
    """
    body = line.replace("\r", " ").replace("\n", " ").strip()
    if not body:
        raise ValueError("命令不能为空")
    if len(body) > TEXT_EXEC_MAX:
        raise ValueError("命令最长 %d 字符，实际 %d" % (TEXT_EXEC_MAX, len(body)))
    return struct.pack("<I", MAGIC) + body.encode("ascii") + b"\x00"


def pl_text_fetch(off: int, n: int = TEXT_CHUNK) -> bytes:
    return struct.pack("<HB", off, n)


def magic_prefix() -> bytes:
    """危险命令（CENTER_CALIB/ZERO_RESET/SYS_RESET/PARAM_SAVE…）的前 4 字节。"""
    return struct.pack("<I", MAGIC)


# ------------------------------------------------------------ 客户端
class Reply(object):
    __slots__ = ("ok", "timeout", "seq", "cmd", "payload", "err", "ms")

    def __init__(self):
        self.ok = False
        self.timeout = False
        self.seq = 0
        self.cmd = 0
        self.payload = b""
        self.err = None
        self.ms = 0.0

    def __repr__(self):
        if self.timeout:
            return "<超时>"
        if self.err is not None:
            return "<NACK %s: %s>" % (ERR_NAME.get(self.err, self.err),
                                      ERR_HINT.get(self.err, ""))
        return "<%s %dB %.1fms>" % (RSP_NAME.get(self.cmd, hex(self.cmd)),
                                    len(self.payload), self.ms)


class ArmLink(object):
    """UDP 客户端。**请求串行化**（UDP 乱序 + 同端口并发会串包）。"""

    def __init__(self, ip: str, port: int = PORT, timeout: float = 0.5):
        self.addr = (ip, port)
        self.timeout = timeout
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)
        self._seq = 0

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def request(self, cmd: int, payload: bytes = b"", timeout: float = None) -> Reply:
        r = Reply()
        self._seq = (self._seq + 1) & 0xFF
        r.seq = self._seq
        r.cmd = cmd
        t0 = time.monotonic()
        self._sock.sendto(pack(self._seq, cmd, payload), self.addr)
        deadline = t0 + (timeout if timeout is not None else self.timeout)
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                r.timeout = True
                return r
            self._sock.settimeout(left)
            try:
                data, _ = self._sock.recvfrom(2048)
            except (socket.timeout, ConnectionResetError):
                # ⚠ Windows：UDP 收到 ICMP port-unreachable 时 recvfrom 抛
                #   ConnectionResetError(10054) 而不是超时 —— 必须容忍
                continue
            try:
                seq, rcmd, pl = unpack(data)
            except ValueError:
                continue                          # 坏包 / 迟到包，丢掉继续等
            if seq != self._seq:
                continue
            r.ms = (time.monotonic() - t0) * 1000.0
            r.cmd = rcmd
            r.payload = pl
            if rcmd == 0x7E:                      # NACK
                r.err = pl[1] if len(pl) >= 2 else 1
            else:
                r.ok = True
            return r

    def send_only(self, cmd: int, payload: bytes = b"") -> None:
        """只发不等 —— 50 Hz STREAM_JOINTS 必须用这个。"""
        self._seq = (self._seq + 1) & 0xFF
        self._sock.sendto(pack(self._seq, cmd, payload), self.addr)

    # ------------------------------------------------------ 便捷方法
    def ping(self):
        r = self.request(PING)
        if not r.ok or len(r.payload) < 2:
            return None
        return {"proto_ver": r.payload[0], "run_id": r.payload[1], "ms": r.ms}

    def get_status(self):
        r = self.request(GET_STATUS)
        if not r.ok or len(r.payload) < 21:
            return None
        pl = r.payload
        joints = []
        for j in range(6):
            v = rd_i16(pl, j * 2)
            joints.append(None if v == RAW_READ_FAIL else v / 10.0)
        flags = pl[20]
        return {
            "servo_deg": joints,
            "failed_joints": [i + 1 for i, v in enumerate(joints) if v is None],
            "x": rd_i16(pl, 12) / 10.0, "y": rd_i16(pl, 14) / 10.0,
            "z": rd_i16(pl, 16) / 10.0, "pitch": rd_i16(pl, 18) / 10.0,
            "estop": bool(flags & 0x01), "traj": bool(flags & 0x02),
            "corr": bool(flags & 0x04), "read_ok": bool(flags & 0x08),
            "flags": flags,
        }

    def get_joint_detail(self, joint=-1):
        """joint=-1 → 全部 6 轴。每轴 10B：pos_raw(u16) speed(i16) load(i16)
        volt(u8,.1V) temp(u8) hw_err(u8) ok(u8)。"""
        r = self.request(GET_JOINT_DETAIL, bytes([0xFF if joint < 0 else joint]))
        if not r.ok or len(r.payload) < 2:
            return None
        n, j0 = r.payload[0], r.payload[1]
        out = []
        for k in range(n):
            o = 2 + k * 10
            if o + 10 > len(r.payload):
                break
            pl = r.payload
            out.append({
                "joint": (j0 + k) if n > 1 else j0,
                "pos_raw": rd_u16(pl, o),
                "speed": rd_i16(pl, o + 2),
                "load": rd_i16(pl, o + 4),
                "voltage": pl[o + 6] / 10.0,
                "temp": pl[o + 7], "temp_lost": pl[o + 7] == TEMP_LOST,
                "hw_err": pl[o + 8], "hw_err_lost": pl[o + 8] == HWERR_LOST,
                "ok": pl[o + 9] != 0,
            })
        return out

    def get_info(self):
        """u8 tag_len + char[16] + u8 build + u8 link + u8 n_params = 20 B。"""
        r = self.request(GET_INFO)
        if not r.ok or len(r.payload) < 20:
            return None
        pl = r.payload
        tl = min(pl[0], 16)
        return {"fw_tag": pl[1:1 + tl].decode("ascii", "replace"),
                "build": pl[17], "link_mode": pl[18], "n_params": pl[19]}

    def move_joints(self, servo_deg6, v=0, a=0):
        return self.request(MOVE_JOINTS, pl_move_joints(servo_deg6, v, a))

    def stop(self):
        return self.request(STOP)

    def set_torque(self, joint=0xFF, enable=True):
        return self.request(SET_TORQUE, pl_set_torque(joint, enable))

    def stream_joints(self, delta_deg6):
        self.send_only(STREAM_JOINTS, pl_stream_joints(delta_deg6))

    # ------------------------------------------------------ 文本透传
    def text(self, line: str, timeout: float = None):
        """下发任意控制台命令，**自动分页**取全回显。

        返回 (正文, truncated, gated)。
        ⚠ 分页按**字节**拼、最后统一 UTF-8 解码 —— 中文一个字 3 字节，
          页边界会把它切开，逐页解码会出乱码。
        """
        r = self.request(TEXT_EXEC, pl_text_exec(line), timeout)
        if not r.ok:
            raise IOError("TEXT_EXEC 失败：%r" % r)
        if len(r.payload) < 6:
            return "", False, False
        total = rd_u16(r.payload, 0)
        flags = r.payload[2]
        off = rd_u16(r.payload, 3)
        n = r.payload[5]
        buf = bytearray(r.payload[6:6 + n])
        off += n
        truncated = bool(flags & TEXT_FLAG_TRUNC)
        gated = bool(flags & TEXT_FLAG_GATE)
        guard = TEXT_BUF_SZ // TEXT_CHUNK + 2
        while off < total and len(buf) <= TEXT_BUF_SZ + 64 and guard > 0:
            guard -= 1
            r2 = self.request(TEXT_FETCH, pl_text_fetch(off), timeout)
            if not r2.ok or len(r2.payload) < 6:
                break
            total2 = rd_u16(r2.payload, 0)
            flags2 = r2.payload[2]
            off2 = rd_u16(r2.payload, 3)
            n2 = r2.payload[5]
            if n2 == 0:
                break
            buf += r2.payload[6:6 + n2]
            truncated |= bool(flags2 & TEXT_FLAG_TRUNC)
            off = off2 + n2
            total = total2
        return buf.decode("utf-8", "replace"), truncated, gated


# ============================================================ 自检 / 冒烟
def selftest() -> int:
    """完全离线（不碰网络）验协议实现本身。"""
    fails = []

    def chk(name, cond, extra=""):
        print("  %s %s%s" % ("✓" if cond else "✗", name, ("  " + extra) if extra else ""))
        if not cond:
            fails.append(name)

    print("=== 协议自检（离线）===")
    chk("CRC16(\"123456789\") == 0x29B1",
        crc16(b"123456789") == 0x29B1, "实测 0x%04X" % crc16(b"123456789"))
    chk("CRC16(空) == 0xFFFF", crc16(b"") == 0xFFFF)

    # 组包/解包往返
    pkt = pack(0x12, 0x40, b"\x07")
    chk("PACK 布局 SEQ/CMD/LEN/…/CRC", pkt == bytes([0x12, 0x40, 0x01, 0x07,
                                                     crc16(pkt[:4]) & 0xFF,
                                                     crc16(pkt[:4]) >> 8]),
        pkt.hex())
    chk("PACK 长度 == 3+LEN+2", len(pkt) == 6)
    chk("UNPACK 往返", unpack(pkt) == (0x12, 0x40, b"\x07"))
    try:
        unpack(pkt[:-1]); chk("长度不符要报错", False)
    except ValueError:
        chk("长度不符要报错", True)
    bad = bytearray(pkt); bad[-1] ^= 0xFF
    try:
        unpack(bytes(bad)); chk("CRC 错要报错", False)
    except ValueError:
        chk("CRC 错要报错", True)

    # 载荷构造逐字节
    chk("MOVE_JOINTS 180° → 1800(0x0708)，v=20 → 200(u16×10)",
        pl_move_joints([180.0] * 6, 20, 0) == b"\x08\x07" * 6 + b"\xc8\x00\x00\x00",
        pl_move_joints([180.0] * 6, 20, 0).hex())
    chk("MOVE_CART 12B v/a ×10",
        # 布局 = i16 x, i16 y, i16 z, i16 pitch, u16 v×10, u16 a×10
        # 100mm→0x03e8 / 0mm→0 / 200mm→0x07d0 / pitch 0 / v 20→200(0x00c8) / a 0
        pl_move_cart(100, 0, 200, 0, 20, 0) == bytes.fromhex("e8030000d0070000c8000000"),
        pl_move_cart(100, 0, 200, 0, 20, 0).hex())
    chk("负角四舍五入远离零：-0.05° → -1",
        pl_stream_joints([-0.05] * 6) == struct.pack("<h", -1) * 6,
        pl_stream_joints([-0.05] * 6).hex())
    chk("SET_TORQUE 全部开 = FF 01", pl_set_torque(0xFF, True) == b"\xff\x01")
    chk("TEXT_EXEC 魔数小端", pl_text_exec("s")[:4] == b"\x5a\x5a\x5a\x5a")
    chk("TEXT_EXEC 正文 + NUL 结尾", pl_text_exec("s")[4:] == b"s\x00",
        pl_text_exec("s").hex())
    chk("TEXT_FETCH off/n", pl_text_fetch(58) == struct.pack("<HB", 58, 58))
    try:
        pl_text_exec("x" * 59); chk("文本超 58 字符要拦住", False)
    except ValueError:
        chk("文本超 58 字符要拦住", True)
    try:
        pl_text_exec("s 中文"); chk("非 ASCII 要拦住", False)
    except UnicodeEncodeError:
        chk("非 ASCII 要拦住", True)

    print("")
    print("  minimal_client 自检：%d 项，%s"
          % (16, "全部通过" if not fails else "失败 %d" % len(fails)))
    return 1 if fails else 0


def smoke(ip: str, port: int) -> int:
    """对着目标（真机或假臂）跑 5 步冒烟。"""
    fails = 0
    print("=== 冒烟测试 → %s:%d ===" % (ip, port))
    with ArmLink(ip, port, timeout=0.8) as arm:
        pi = arm.ping()
        print("  [1] PING           %s" % ("✗ 无回包" if pi is None
              else "✓ proto_ver=%d run_id=%d  %.1f ms" % (pi["proto_ver"], pi["run_id"], pi["ms"])))
        if pi is None:
            print("      查：①固件是 NET/BOTH 吗 ②`link` 偏好是 net 吗（改完要 save+RESET）")
            print("          ③端口必须 5000 ④防火墙放行 UDP ⑤IP/网线")
            return 1

        st = arm.get_status()
        if st is None:
            print("  [2] GET_STATUS     ✗ 失败")
            fails += 1
        else:
            txt = ", ".join("--" if v is None else "%.1f" % v for v in st["servo_deg"])
            print("  [2] GET_STATUS     ✓ 舵机度 [%s]" % txt)
            print("      flags=0x%02X read_ok=%s estop=%s traj=%s corr=%s"
                  % (st["flags"], st["read_ok"], st["estop"], st["traj"], st["corr"]))
            if st["failed_joints"]:
                print("      ⚠ 读失败的轴 J%s —— 不要当 0 用，先排总线" % st["failed_joints"])
                fails += 1

        jd = arm.get_joint_detail(-1)
        if not jd:
            print("  [3] JOINT_DETAIL   ✗")
            fails += 1
        else:
            d = jd[0]
            print("  [3] JOINT_DETAIL   ✓ %d 轴 | J1 raw=%d pos=%.1f° temp=%s volt=%.1fV"
                  % (len(jd), d["pos_raw"], d["pos_raw"] * 360.0 / 4095.0,
                     "--" if d["temp_lost"] else "%d℃" % d["temp"], d["voltage"]))

        info = arm.get_info()
        print("  [4] GET_INFO       %s" % ("✗" if info is None else
              "✓ fw=%s build=%d link=%d 参数 %d 项"
              % (info["fw_tag"], info["build"], info["link_mode"], info["n_params"])))

        try:
            out, trunc, gated = arm.text("h")
            first = out.strip().splitlines()[0][:60] if out.strip() else ""
            print("  [5] TEXT_EXEC \"h\"  ✓ %d 字节 trunc=%s gated=%s" %
                  (len(out.encode("utf-8")), trunc, gated))
            print("      首行: %s" % first)
        except IOError as e:
            print("  [5] TEXT_EXEC \"h\"  ✗ %s" % e)
            print("      老固件可能没有 TEXT_EXEC(0x0B)，用串口直接敲 `h` 代替")
            fails += 1

        try:
            out, trunc, gated = arm.text("cx")      # 抢 ETH 引脚的命令，应被拦下
            print("  [6] 抢引脚拦截 \"cx\"  首行: %s"
                  % (out.strip().splitlines() or [""])[0][:60])
            print("      （这类命令不是闸门拒绝，是固件主动拦住并给原因）")
        except IOError as e:
            print("  [6] 抢引脚拦截 \"cx\"  ✗ %s" % e)
            fails += 1
            print("      （旧固件无 TEXT_EXEC 时这条会失败，属正常）")

    print("")
    print("  %s" % ("✅ 全部通过" if fails == 0 else "❌ %d 项失败" % fails))
    return 1 if fails else 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="minimal_client.py",
        description="网口协议最小客户端（纯标准库，独立实现，可当多语言实现的参照物）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：\n"
               "  python minimal_client.py --selftest                    # 离线验协议\n"
               "  armctl mock --port 5000                               # 终端1：起假臂\n"
               "  python minimal_client.py --ip 127.0.0.1 --smoke        # 终端2\n"
               "  python minimal_client.py --ip 192.168.7.120 --text s --text zi\n")
    p.add_argument("--ip", default="192.168.7.120", help="臂的 IP（默认 192.168.7.120）")
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--text", action="append", default=[],
                   help="下发一条控制台命令（可重复），如 --text s --text zi")
    p.add_argument("--smoke", action="store_true", help="跑 6 步冒烟测试")
    p.add_argument("--selftest", action="store_true", help="离线验协议实现（不碰网络）")
    p.add_argument("--timeout", type=float, default=0.8)
    a = p.parse_args(argv)

    if a.selftest:
        return selftest()
    if a.smoke:
        return smoke(a.ip, a.port)
    if not a.text:
        p.print_help()
        return 0

    rc = 0
    with ArmLink(a.ip, a.port, a.timeout) as arm:
        for line in a.text:
            try:
                out, trunc, gated = arm.text(line)
                print(">>> %s" % line)
                print(out.rstrip() or "(无输出)")
                if gated:
                    print("  ⚠ 被固件安全闸门拒绝（正文就是原因）")
                if trunc:
                    print("  ⚠ 输出超过 %d B 被截断，只看到前一段" % TEXT_BUF_SZ)
            except IOError as e:
                print(">>> %s\n  ✗ %s" % (line, e))
                rc = 2
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
