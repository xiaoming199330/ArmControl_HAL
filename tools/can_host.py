#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_host.py — 六轴机械臂 CAN 从站的上位机参考实现（协议规范见
docs/CAN协议规范_2026-09-17.md）

设计原则同 tools/pc_relay.py: **没有硬件也能验证协议层**。
  python tools/can_host.py --selftest      # 离线: CRC/分包/重组 + 假从站跑完冒烟流程, 不需 CAN 卡
  python tools/can_host.py --list          # 列出本机可用的 CAN 通道
  python tools/can_host.py --linktest 200  # ★物理链路自检: 发 0x7E0 自检帧 + 读 CAN 卡总线状态
                                           #   判据是"两侧帧计数对拍", **不是**卡的状态读数
                                           #   (实测: 无对端/位速率不匹配时卡照样报 OK)
  python tools/can_host.py --smoke         # ★一键冒烟: PING→读写→状态帧(全程只读不碰)
  python tools/can_host.py --smoke --allow-move    # 加上运动往返(只动 J1/J6 ±3°)
  python tools/can_host.py --ping          # 探活
  python tools/can_host.py --status        # 读六轴角度 + 末端位姿
  python tools/can_host.py --monitor 10    # 监听 10s: 解周期状态帧与事件帧
  python tools/can_host.py --move 90 90 90 90 90 90      # 关节运动(舵机度)
  python tools/can_host.py --cart 180 0 200 90           # 笛卡尔运动(mm, pitch)
  python tools/can_host.py --stop                        # 停轨迹

依赖: pip install python-can
  若用创芯/SLCAN 等 USB 卡: --interface slcan --channel COM8 --bitrate 500000
  若 Linux socketcan:      --interface socketcan --channel can0
"""

import argparse
import sys
import time

# ============================================================
# 协议常量（与 Core/Inc/arm_control_protocol.h / arm_proto_can.h 一一对应）
# ============================================================
ID_CMD_BASE = 0x100
ID_RSP_BASE = 0x180
ID_EVT_BASE = 0x280
ID_TEST     = 0x7E0

EVT_TYPE_STATUS = 0x53   # 'S'
EVT_TYPE_EVENT  = 0x45   # 'E'

EVENT_NAMES = {
    1: "ESTOP         E-Stop 触发",
    2: "TRAJ_DONE     轨迹结束",
    3: "STREAM_EXIT   流模式退出",
    4: "TEACH_FULL    示教缓冲写满",
    5: "CFG_SAVED     CAN 配置已落盘",
}

ERR_NAMES = {
    0: "OK", 1: "PARAM", 2: "CRC", 3: "UNREACH",
    4: "LIMIT", 5: "ESTOP", 6: "FK_VERIFY", 7: "READ",
}

CMD_PING          = 0x01
CMD_GET_STATUS    = 0x02
CMD_LOG_SET_LEVEL = 0x03
CMD_MOVE_JOINTS   = 0x10
CMD_STOP          = 0x11
CMD_SET_TORQUE    = 0x12
CMD_STREAM_JOINTS = 0x13
CMD_MOVE_CART     = 0x20
CMD_FK_QUERY      = 0x21
CMD_SET_GRAVITY   = 0x30
CMD_SET_CORRECTION= 0x31
CMD_PARAM_READ    = 0x40
CMD_PARAM_WRITE   = 0x41
CMD_PARAM_COUNT   = 0x42

RSP_ACK  = 0x7F
RSP_NACK = 0x7E

MAX_PAYLOAD = 64

# 报文构造函数。离线仿真(_FakeArmBus)时被替换掉, 这样 --selftest
# **不需要 python-can 也能跑通整条流程**(否则"验证协议层的工具本身先跪在装包上")。
_MSG_FACTORY = None


def _make_msg(arbitration_id, data):
    if _MSG_FACTORY is not None:
        return _MSG_FACTORY(arbitration_id, data)
    import can
    return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)


# ============================================================
# CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, 非反射)
# 校验值: crc16(b"123456789") == 0x29B1
# ============================================================
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# ============================================================
# 分包 / 重组
# ============================================================
def plan_frames(payload_len: int):
    """返回 (总帧数, 首帧在 d[4..7] 里装的字节数)。与固件 can_send_packed 同一公式。"""
    tail_total = payload_len + 2                 # PAYLOAD + CRC
    first_n = min(tail_total, 4)
    rest = max(tail_total - 4, 0)
    frames = 1 + (rest + 6) // 7
    return frames, first_n


def build_frames(seq: int, cmd: int, payload: bytes) -> list:
    """把一条逻辑帧切成 CAN 数据场列表(DLC 可变, 不补 8)。"""
    assert len(payload) <= MAX_PAYLOAD, "payload 超长"
    crc = crc16(bytes([seq, cmd, len(payload)]) + payload)
    logical = bytes([seq, cmd, len(payload)]) + payload + bytes([crc & 0xFF, crc >> 8])
    frames, first_n = plan_frames(len(payload))

    out = [bytes([0x80 | frames, seq, cmd, len(payload)]) + logical[3:3 + first_n]]
    off = 3 + first_n
    for f in range(1, frames):
        chunk = logical[off:off + 7]
        out.append(bytes([f]) + chunk)
        off += len(chunk)
    return out


class Reassembler:
    """收 CAN 帧 -> 还原逻辑帧。只处理命令/响应两段 ID。

    ⚠ 2026-09-18 现场踩到的大坑: **续帧会乱序到达, 而且这是硬件行为不是丢帧**。
      bxCAN 只有 **3 个发送邮箱**。一条 4 帧的回包发出去时:
        F1→MB0, F2→MB1, F3→MB2 挂起, 而 F4 在主循环里一等到邮箱空就抢走 **MB0**。
      硬件按"ID 相同则邮箱号小者先发"调度 ⇒ 实际总线顺序是 **F1, F4, F2, F3**。
      实测(抓原始帧, 载荷字节可反推顺序):
        84 11 82 15 | 01 07 00 03      <- idx0 (首帧, 共 4 帧, LEN=0x15)
        03 | b1 05 08 f5 c9            <- idx3  ← 第 4 帧先到!
        01 | 93 0a c7 09 21 07 bf      <- idx1
        02 | 05 7d 05 0f 00 ae 00      <- idx2
      后果: 旧版 `data[0] == self.next` 的严格递增判定会把整条流判成乱序而丢弃,
      症状是"**1 帧的 PING 通, 3~4 帧的 GET_STATUS/FK 永远超时**" ——
      极容易误判成"协议/物理层有问题"。
      ⇒ 所以这里**按帧号缓冲**(允许乱序), 收齐后再拼; 顺序由发送侧另行保证。
    """

    # 顺序取证统计(2026-09-18 用来验证固件的 `frameseq` 修复有没有真生效)
    STATS = {"packets": 0, "ordered": 0, "reordered": 0, "strict_reject": 0}

    def __init__(self, node: int, strict: bool = False, trace=None):
        """strict=True ⇒ 要求续帧**严格按序到达**, 模拟"没打过补丁的老上位机"。

        ⚠ 验证固件帧序修复**必须**用 strict=True。宽容模式(默认)即使固件没修
          也会 PASS —— 它存在的意义就是让老固件也能用, 因此测不出修复。
        """
        self.node = node
        self.strict = strict
        self.trace = trace        # callable(order: list[int]) -> None, 打到达顺序
        self.reset()

    def reset(self):
        self.active = False
        self.frames = 0
        self.next = 0            # "期望的下一个序号"(strict 模式下是硬判据)
        self.head = b""          # 首帧携带的 (PAYLOAD+CRC) 开头
        self.hdr = b""           # 首帧的 SEQ/CMD/LEN 三字节
        self.parts = {}          # 帧号 -> 该帧携带的字节(乱序安全)
        self.order = []          # 本包实际到达顺序(0=首帧, 1..n-1=续帧号)
        self.tick = 0.0

    def feed(self, can_id: int, data: bytes, now: float = None):
        """喂一帧。返回 (seq, cmd, payload, crc_ok) 或 None。"""
        now = time.monotonic() if now is None else now
        # 重组超时(200ms)复位
        if self.active and now - self.tick > 0.200:
            self.reset()
        if len(data) < 1:
            return None

        if data[0] & 0x80:                        # 首帧
            if len(data) < 4:
                return None
            frames = data[0] & 0x0F
            ln = data[3]
            if frames == 0 or ln > MAX_PAYLOAD:
                self.reset()
                return None
            want, first_n = plan_frames(ln)
            if frames != want:                    # 与 LEN 不自洽 ⇒ 拒
                self.reset()
                return None
            self.active = True
            self.frames = frames
            self.next = 1
            self.hdr = bytes([data[1], data[2], data[3]])
            self.head = data[4:4 + first_n]
            self.parts = {}
            self.order = [0]
            self.tick = now
        elif self.active and 1 <= data[0] < self.frames:   # 续帧(允许乱序)
            if self.strict and data[0] != self.next:
                # 老上位机就是这个行为: 乱序即判整包作废。
                # 0918F 之前多帧命令必超时, 原因就在这一支。
                self.STATS["strict_reject"] += 1
                if self.trace:
                    self.trace(self.order + [data[0]])
                self.reset()
                return None
            if data[0] not in self.parts:         # 重复帧只收一次
                self.parts[data[0]] = data[1:]
                self.order.append(data[0])
            self.next = max(self.next, data[0] + 1)
            self.tick = now
        else:
            self.reset()
            return None

        # 收齐(首帧 + 全部续帧)才拼 —— 与到达顺序无关
        if self.active and len(self.parts) >= self.frames - 1:
            body = self.head + b"".join(self.parts[i] for i in range(1, self.frames))
            seq, cmd, ln = self.hdr[0], self.hdr[1], self.hdr[2]
            need = ln + 2                          # body 里应含 PAYLOAD+CRC
            if len(body) >= need:
                got = body[ln] | (body[ln + 1] << 8)
                ok = (got == crc16(self.hdr + body[:ln]))
                self.STATS["packets"] += 1
                if self.order == list(range(self.frames)):
                    self.STATS["ordered"] += 1
                else:
                    self.STATS["reordered"] += 1
                if self.trace:
                    self.trace(self.order)
                self.reset()
                return (seq, cmd, body[:ln], ok)
        return None


# ============================================================
# 负载编解码小工具(定点)
# ============================================================
def i16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=True)


def u16(v: int) -> bytes:
    return int(v).to_bytes(2, "little", signed=False)


def i32(v: int) -> bytes:
    return int(v).to_bytes(4, "little", signed=True)


def rd_i16(b: bytes, off: int) -> int:
    return int.from_bytes(b[off:off + 2], "little", signed=True)


def rd_u16(b: bytes, off: int) -> int:
    return int.from_bytes(b[off:off + 2], "little", signed=False)


def rd_i32(b: bytes, off: int) -> int:
    return int.from_bytes(b[off:off + 4], "little", signed=True)


def payload_move_joints(deg, v=0.0, a=0.0) -> bytes:
    p = b"".join(i16(round(d * 10)) for d in deg)
    return p + u16(round(v * 10)) + u16(round(a * 10))


def payload_move_cart(x, y, z, pitch, v=0.0, a=0.0) -> bytes:
    return (i16(round(x * 10)) + i16(round(y * 10)) + i16(round(z * 10)) +
            i16(round(pitch * 10)) + u16(round(v * 10)) + u16(round(a * 10)))


def payload_stream_joints(logical_deg) -> bytes:
    return b"".join(i16(round(d * 10)) for d in logical_deg)


def decode_status_payload(p: bytes) -> dict:
    """0x82 GET_STATUS 的 21 字节负载"""
    d = {"joints": [], "x": 0.0, "y": 0.0, "z": 0.0, "pitch": 0.0, "flags": 0}
    for j in range(6):
        v = rd_i16(p, j * 2)
        d["joints"].append(None if v == -32768 else v / 10.0)
    d["x"], d["y"], d["z"] = rd_i16(p, 12) / 10.0, rd_i16(p, 14) / 10.0, rd_i16(p, 16) / 10.0
    d["pitch"] = rd_i16(p, 18) / 10.0
    f = p[20]
    d["flags"] = f
    d["estop"], d["traj"], d["corr"], d["read_ok"] = bool(f & 1), bool(f & 2), bool(f & 4), bool(f & 8)
    return d


def decode_fk_payload(p: bytes) -> dict:
    """0xA1 FK_QUERY 的 21 字节负载。关节同样是舵机度, -32768 = 该轴读失败。"""
    joints = []
    for j in range(6):
        v = rd_i16(p, 8 + 2 * j)
        joints.append(None if v == -32768 else v / 10.0)
    return {
        "x": rd_i16(p, 0) / 10.0, "y": rd_i16(p, 2) / 10.0,
        "z": rd_i16(p, 4) / 10.0, "pitch": rd_i16(p, 6) / 10.0,
        "joints": joints,
        "read_ok": p[20],
    }


def decode_evt_frame(data: bytes) -> str:
    """0x280+node 上的 8 字节状态/事件帧 -> 人可读"""
    if len(data) < 4:
        return None
    node = data[1]
    if data[0] == EVT_TYPE_STATUS:
        f = data[2]
        bits = []
        for bit, name in ((1, "ESTOP"), (2, "TRAJ"), (4, "STREAM"), (8, "CORR"),
                          (0x10, "REC"), (0x20, "PLAY"), (0x40, "HW"), (0x80, "RDY")):
            if f & bit:
                bits.append(name)
        return ("[状态 node=%d] %-34s last_err=%-9s 命令数=%d 运行=%ds"
                % (node, "+".join(bits) or "-",
                   ERR_NAMES.get(data[3], data[3]),
                   data[4] | (data[5] << 8), data[6] | (data[7] << 8)))
    if data[0] == EVT_TYPE_EVENT:
        code = EVENT_NAMES.get(data[2], "未知事件 %d" % data[2])
        return "[事件 node=%d] %-28s param=%d t=%dms" % (node, code, data[3],
                                                        int.from_bytes(data[4:8], "little"))
    return None


# ============================================================
# 离线假从站 —— 没有 CAN 卡时, 验证"冒烟流程本身"是否走得通
# 它只扮演判据用到的那几个角色: 收命令→回响应; 收到过帧→开始发周期状态帧;
# MOVE_JOINTS→ACK, 稍后补一个 TRAJ_DONE 事件帧。
# ============================================================
class _Msg:
    __slots__ = ("arbitration_id", "data")

    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)


class _FakeArmBus:
    def __init__(self, node=1):
        self.node = node
        self.q = []
        self.unlocked = False
        self.traj_at = None
        self.last_st = 0.0
        self.sent = []
        self.joints = [150.0, 120.0, 150.0, 200.0, 200.0, 150.0]   # 舵机度
        self.seq_echo = 0

    # ---- 从站侧: 收命令 ----
    def send(self, msg):
        d = bytes(msg.data)
        self.sent.append((msg.arbitration_id, d))
        if msg.arbitration_id != ID_CMD_BASE + self.node or not d:
            return
        if not (d[0] & 0x80):      # 只处理首帧
            return
        self.unlocked = True       # 收到过发往本节点的帧 ⇒ 解锁状态帧
        seq, cmd = d[1], d[2]
        self.seq_echo = seq
        rcmd, pl = self._resp(cmd)
        # ⚠ 响应也要按分包规则切成多帧(GET_STATUS 是 21B 负载 ⇒ 4 帧),
        #   只回首帧的话主机的重组器永远等不齐 —— 这正是要测的东西。
        for f in build_frames(seq, rcmd, pl):
            self.q.append(_Msg(ID_RSP_BASE + self.node, f))

    def _resp(self, cmd):
        """返回 (响应码, 负载)。注意响应码 ≠ 请求码(0x01->0x81)。"""
        if cmd == CMD_PING:
            return 0x81, bytes([0x01, 0x2A])
        if cmd == CMD_GET_STATUS:
            p = b"".join(i16(round(v * 10)) for v in self.joints)
            return 0x82, p + i16(1800) + i16(0) + i16(2500) + i16(900) + bytes([0x08])
        if cmd == CMD_FK_QUERY:
            p = b"".join(i16(round(v * 10)) for v in self.joints)
            return 0xA1, (i16(1800) + i16(0) + i16(2500) + i16(900) + p + bytes([1]))
        if cmd == CMD_PARAM_COUNT:
            return 0xC2, bytes([24])
        if cmd == CMD_PARAM_READ:
            return 0xC0, bytes([0x08]) + i32(90000)
        if cmd == CMD_MOVE_JOINTS:
            self.traj_at = time.monotonic() + 0.05
            return RSP_ACK, bytes([cmd, 0x00])
        if cmd == CMD_STOP:
            return RSP_ACK, bytes([cmd, 0x00])
        return RSP_NACK, bytes([cmd, 0x01])

    # ---- 主机侧: 收帧 ----
    def recv(self, timeout=0.0):
        now = time.monotonic()
        if self.traj_at is not None and now >= self.traj_at:
            self.traj_at = None
            return _Msg(ID_EVT_BASE + self.node,
                        bytes([EVT_TYPE_EVENT, self.node, 2, 1]) + i32(1234))
        if self.unlocked and now - self.last_st >= 0.05:
            self.last_st = now
            return _Msg(ID_EVT_BASE + self.node,
                        bytes([EVT_TYPE_STATUS, self.node, 0x0E, 0x00]) +
                        u16(3) + u16(42))
        if self.q:
            return self.q.pop(0)
        return None

    def shutdown(self):
        pass



def selftest() -> int:
    fail = 0

    def chk(name, cond, extra=""):
        nonlocal fail
        print("  %s %s%s" % ("PASS" if cond else "FAIL", name, ("  " + extra) if extra else ""))
        if not cond:
            fail += 1

    print("== 1) CRC-16/CCITT-FALSE 标准校验值 ==")
    chk("crc16('123456789') == 0x29B1", crc16(b"123456789") == 0x29B1,
        "实际 0x%04X" % crc16(b"123456789"))

    print("== 2) 分包帧数(LEN -> 帧数), 与固件公式一致 ==")
    for ln, want in ((0, 1), (1, 1), (2, 1), (5, 2), (12, 3), (16, 3), (21, 4), (64, 10)):
        got, _ = plan_frames(ln)
        chk("LEN=%-3d -> %d 帧" % (ln, want), got == want, "实际 %d" % got)

    print("== 3) 往返: 造帧 -> 拆帧 -> 重组, 内容(含 CRC)必须完全一致 ==")
    for seq, cmd, ln in ((0x00, CMD_PING, 0), (0x01, CMD_GET_STATUS, 0),
                         (0x02, CMD_MOVE_JOINTS, 16), (0x03, CMD_MOVE_CART, 12),
                         (0x04, CMD_STREAM_JOINTS, 12), (0x05, 0x82, 21),
                         (0x06, RSP_ACK, 2), (0x07, 0xC0, 5)):
        payload = bytes((i * 7 + 1) & 0xFF for i in range(ln))
        frames = build_frames(seq, cmd, payload)
        r = Reassembler(node=1)
        got = None
        for f in frames:
            got = r.feed(0x100 + 1, f) or got
        chk("seq=%d cmd=0x%02X len=%d (%d 帧)" % (seq, cmd, ln, len(frames)),
            got is not None and got[0] == seq and got[1] == cmd and
            got[2] == payload and got[3] is True,
            "" if got else "重组失败")

    print("== 4) 首帧/续帧 d[0] 取值 ==")
    f = build_frames(0x11, CMD_MOVE_JOINTS, bytes(16))
    chk("MOVE_JOINTS 首帧 d[0]=0x83", f[0][0] == 0x83, "实际 0x%02X" % f[0][0])
    chk("续帧序号 1,2", f[1][0] == 1 and f[2][0] == 2)
    chk("首帧 DLC=8, 末帧 DLC=8", len(f[0]) == 8 and len(f[-1]) == 8)
    f2 = build_frames(0x12, CMD_GET_STATUS, b"")
    chk("GET_STATUS 请求 DLC=6 单帧", len(f2) == 1 and len(f2[0]) == 6,
        "实际 %d 帧/DLC=%s" % (len(f2), [len(x) for x in f2]))

    print("== 5) 注入错误: 篡改一字节后 CRC 必须报错 ==")
    f3 = build_frames(0x20, CMD_MOVE_CART, bytes(12))
    bad = bytearray(f3[1]); bad[3] ^= 0xFF
    r = Reassembler(node=1)
    out = None
    for fr in [f3[0], bytes(bad), f3[2]]:
        out = r.feed(0x100 + 1, fr) or out
    chk("篡改后 crc_ok == False", out is not None and out[3] is False)

    print("== 6) 乱序/丢帧后必须放弃整包 ==")
    f4 = build_frames(0x30, CMD_MOVE_JOINTS, bytes(16))
    r = Reassembler(node=1)
    a = r.feed(0x100 + 1, f4[0])
    b = r.feed(0x100 + 1, f4[2])   # 跳过 f4[1]
    chk("丢续帧 -> 不产出逻辑帧", a is None and b is None)

    print("== 7) 状态帧/事件帧解码 ==")
    st = bytes([EVT_TYPE_STATUS, 1, 0x42, 0x00, 0x34, 0x12, 0x10, 0x00])
    print("     " + decode_evt_frame(st))
    ev = bytes([EVT_TYPE_EVENT, 1, 2, 1, 0xE8, 0x03, 0x00, 0x00])
    print("     " + decode_evt_frame(ev))
    chk("状态帧识别 HW+TRAJ", "HW" in decode_evt_frame(st) and "TRAJ" in decode_evt_frame(st))
    chk("事件帧识别 TRAJ_DONE", "TRAJ_DONE" in decode_evt_frame(ev))

    print("== 8) 冒烟测试流程(离线假从站, 不需要 CAN 卡 / 不需要 python-can) ==")
    global _MSG_FACTORY
    _MSG_FACTORY = lambda cid, data: _Msg(cid, data)      # 绕开 python-can 的 Message
    try:
        fake = _FakeArmBus(node=1)
        rc = smoke(ArmCanHost(fake, node=1, verbose=False), allow_move=True, monitor_s=0.3)
        chk("冒烟流程全绿(含运动往返 + TRAJ_DONE)", rc == 0)
        chk("MOVE_JOINTS 真的上总线了", any(
            len(m[1]) >= 3 and (m[1][0] & 0x80) and m[1][2] == CMD_MOVE_JOINTS
            for m in fake.sent))
    finally:
        _MSG_FACTORY = None

    print("\n%s (%d 项失败)" % ("全部通过 ✅" if fail == 0 else "有失败项 ❌", fail))
    return 1 if fail else 0


# ============================================================
# 在线部分
# ============================================================
class ArmCanHost:
    def __init__(self, bus, node=1, verbose=True, strict=False, trace=None):
        self.bus = bus
        self.node = node
        self.seq = 0
        self.verbose = verbose
        self.reasm = Reassembler(node, strict=strict, trace=trace)
        self.cmd_id = ID_CMD_BASE + node
        self.rsp_id = ID_RSP_BASE + node

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send(self, cmd, payload=b""):
        """发一条命令, 返回 seq。"""
        seq = self._next_seq()
        for f in build_frames(seq, cmd, payload):
            self.bus.send(_make_msg(self.cmd_id, f))
        return seq

    def wait_response(self, seq, timeout=0.3):
        """等 seq 匹配的响应/事件帧。返回 (cmd, payload) 或 None。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            cid, data = msg.arbitration_id, bytes(msg.data)
            if cid == ID_EVT_BASE + self.node:
                txt = decode_evt_frame(data)
                if txt and self.verbose:
                    print("  " + txt)
                continue
            if cid != self.rsp_id:
                continue
            out = self.reasm.feed(cid, data)
            if out:
                rseq, rcmd, payload, ok = out
                if not ok:
                    print("  ⚠ CRC 错 (cmd=0x%02X)" % rcmd)
                    continue
                if rseq == seq:
                    return rcmd, payload
        return None

    def request(self, cmd, payload=b"", timeout=0.3, label=""):
        """发命令 + 等响应 + 打印结果。返回 (rcmd, payload)。"""
        seq = self.send(cmd, payload)
        res = self.wait_response(seq, timeout)
        if res is None:
            print("  ✗ %s 超时(0x%02X), 检查: 波特率一致? 发的是标准帧到 0x%03X? "
                  "总线 120Ω 与共地?" % (label or "命令", cmd, self.cmd_id))
            return None
        rcmd, pl = res
        if rcmd in (RSP_ACK, RSP_NACK):
            err = pl[1] if len(pl) >= 2 else 0xFF
            ok = (rcmd == RSP_ACK)
            print("  %s %s (cmd=0x%02X ret=%s)" %
                  ("✓" if ok else "✗", label or "命令", pl[0] if pl else 0,
                   ERR_NAMES.get(err, err)))
        return rcmd, pl

    def request_quiet(self, cmd, payload=b"", timeout=0.5):
        """同 request, 但返回纯结果码: (rcmd, payload) 或 None。"""
        seq = self.send(cmd, payload)
        return self.wait_response(seq, timeout)

    def wait_event(self, want_code, timeout=25.0):
        """等某类异步事件帧(见 EVENT_NAMES)。返回 param 或 None。
        期间照常把响应喂给重组器, 避免静默丢帧。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                continue
            cid, data = msg.arbitration_id, bytes(msg.data)
            if cid == ID_EVT_BASE + self.node:
                txt = decode_evt_frame(data)
                if txt and self.verbose:
                    print("  " + txt)
                if (data and data[0] == EVT_TYPE_EVENT and len(data) >= 4
                        and data[2] == want_code):
                    return data[3]
            elif cid == self.rsp_id:
                self.reasm.feed(cid, data)      # 只维持重组器状态
        return None

    def count_status_frames(self, seconds):
        """在 given 秒内数周期状态帧, 返回 (状态帧数, 事件帧数)。"""
        n_st, n_ev = 0, 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = self.bus.recv(timeout=max(0.0, end - time.monotonic()))
            if msg is None:
                continue
            if msg.arbitration_id != ID_EVT_BASE + self.node:
                continue
            data = bytes(msg.data)
            if not data:
                continue
            if data[0] == EVT_TYPE_STATUS:
                n_st += 1
                if self.verbose:
                    print("  " + decode_evt_frame(data))
            elif data[0] == EVT_TYPE_EVENT:
                n_ev += 1
                if self.verbose:
                    print("  " + decode_evt_frame(data))
        return n_st, n_ev


def open_bus_with_retry(interface, channel, bitrate, tries=3, delay=2.0):
    """打开 CAN 卡, 带重试。

    为什么需要重试(2026-09-17 实测): PCAN-USB 会**间歇性**报
      「A PCAN Channel has not been initialized yet or the initialization
        process has failed」
    而 `--probe` 里同一时刻可能显示 `channel_condition=2`(= PCAN_CHANNEL_OCCUPIED
    「已被占用」)。这**不是卡坏了**, 而是有别的程序(PCAN-View / CanAst.exe /
    上一次异常退出的残留句柄)占着它。重试几次仍不行 ⇒ 拔插一次 PCAN-USB。
    """
    import can
    last = None
    for i in range(max(1, tries)):
        try:
            return can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        except Exception as e:              # noqa: BLE001
            last = e
            time.sleep(delay)
    raise last


def list_channels(interface):
    import can
    try:
        print("可用通道 (%s):" % interface)
        for ch in sorted(can.detect_available_configs(interface),
                         key=lambda c: str(c)):
            print("  ", ch)
    except Exception as e:            # noqa: BLE001
        print("  列举失败: %s" % e)


# ============================================================
# --probe: 认卡 + 环境体检
#
# 现场最容易卡住的一步不是协议, 而是"我这块 USB-CAN 到底该填哪个 --interface"。
# 填错的症状是"打开失败", 和"线没接"长得一样, 于是白查半天。
# 这个命令把三件事一次说清: ①python-can 有没有 ②厂商 DLL 在不在、**位数对不对**
# ③每种接口实际枚举/打开到什么。最后直接给出该敲的命令。
#
# ⚠ 位数是第一杀手: 32 位厂商 DLL 在 64 位 Python 里 ctypes 加载必然失败
#   ("不是有效的 Win32 应用程序"), 这跟代码写得对不对无关。
# ============================================================
CANDIDATE_DLLS = ("PCANBasic.dll", "PCANBasic64.dll", "lyscan.dll", "ControlCAN.dll",
                  "ECanVci.dll", "USBCAN.dll", "zlgcan.dll", "libusb-1.0.dll")


def _pe_machine(path):
    """返回 (位数文本, 大小)。不是 PE 就返回 (None, None)。"""
    try:
        with open(path, "rb") as f:
            d = f.read(0x400)
            if d[:2] != b"MZ":
                return None, None
            pe = int.from_bytes(d[0x3C:0x40], "little")
            m = int.from_bytes(d[pe + 4:pe + 6], "little")
        import os
        sz = os.path.getsize(path)
        return {0x14C: "x86(32位)", 0x8664: "x64(64位)"}.get(m, "0x%X" % m), sz
    except OSError:
        return None, None


def probe():
    import os
    import platform

    is64 = platform.architecture()[0] == "64bit"
    print("=" * 70)
    print("  CAN 环境体检   Python %s / %s"
          % (sys.version.split()[0], "64 位" if is64 else "32 位"))
    print("=" * 70)

    print("\n[1] python-can")
    try:
        import can
        print("    ✓ python-can %s" % can.__version__)
        have_can = True
    except ImportError:
        print("    ✗ 未安装 → pip install python-can")
        have_can = False

    print("\n[2] 厂商 DLL（★位数必须与 Python 一致，否则 ctypes 必然加载失败）")
    seen = []
    dirs = [os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32"),
            os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "SysWOW64"),
            r"C:\Program Files\PEAK-System\PEAK-Drivers 4",
            r"C:\Program Files (x86)\CanAst",
            r"C:\Program Files (x86)\LysUsbCanDriver",
            r"C:\Program Files\ZLG", r"C:\ZLG"]
    for name in CANDIDATE_DLLS:
        for d in dirs:
            p = os.path.join(d, name)
            if os.path.exists(p):
                bits, sz = _pe_machine(p)
                mark = ""
                if bits and bits.startswith("x86") and is64:
                    mark = "  ⚠ 32 位, 本 Python 加载不了"
                elif bits and bits.startswith("x64") and not is64:
                    mark = "  ⚠ 64 位, 本 Python 加载不了"
                print("    %-62s %-10s%s" % (p, bits or "?", mark))
                seen.append(name)
    if not seen:
        print("    （常见目录里没找到已知的 CAN DLL）")

    if not have_can:
        print("\n装完 python-can 再跑一次本命令。")
        return 0

    import can
    print("\n[3] 各接口枚举到的硬件")
    for iface in ("pcan", "gs_usb", "slcan", "canalystii", "robotell", "usb2can", "serial"):
        try:
            cfgs = can.detect_available_configs(iface)
            if cfgs:
                for c in cfgs:
                    print("    %-12s %s" % (iface, c))
            else:
                print("    %-12s （无）" % iface)
        except Exception as e:                      # noqa: BLE001
            print("    %-12s 列举失败: %s" % (iface, str(e)[:60]))

    print("\n[4] 实际打开试探（能打开 = 卡在线）")
    trials = [("pcan", "PCAN_USBBUS1"),
              ("pcan", "PCAN_USBBUS2"),
              ("gs_usb", "0"),
              ("slcan", None)]
    if not trials:
        pass
    for iface, ch in trials:
        if ch is None:
            continue
        try:
            bus = can.Bus(interface=iface, channel=ch, bitrate=500000)
            bus.shutdown()
            print("    ✓ %s:%s  打开成功  ⇒ 这就是你该用的接口" % (iface, ch))
        except Exception as e:                      # noqa: BLE001
            print("    ✗ %s:%s  %s" % (iface, ch, str(e)[:70]))

    print("\n[5] 结论 / 该敲什么")
    print("""
    · PEAK PCAN 卡在 →  python tools/can_host.py --interface pcan --channel PCAN_USBBUS1 --smoke
    · 卡是 SLCAN/LAWICEL 类(枚举为串口) →
          python tools/can_host.py --interface slcan --channel COM<x> --smoke
    · 卡只有 32 位厂商 DLL(如 lyscan.dll) → 64 位 Python 用不了, 三选一:
          ① 换回 PCAN 卡(驱动已装, 最省事)
          ② 装一个 32 位 Python 再跑(本工具的编解码部分与位数无关, 只需换个解释器)
          ③ 用厂商 GUI(CanAst.exe) 手动收发, 只做物理层验证
    · 接口开着不报错但一帧都收不到 → 不是接口问题, 走测试单阶段 3~4 的排查清单
    """)
    return 0


# ============================================================
# 一键冒烟测试 —— 一条命令跑完全流程, 输出 PASS/FAIL 表
#
# 安全约定: **默认只读不碰**。要发运动命令必须显式加 --allow-move,
# 且用"当前读数 ± 小角度"而不是写死的目标姿态 (写死姿态在不同摆位下
# 可能撞桌面; 见 docs/工程红线速查 §6 与 robot-arm-safe-trajectory)。
# 只动 J1(底座) 和 J6(末端自转) —— 这两轴在任意姿态下都不会撞台面。
# ============================================================
SMOKE_MOVE_AXES = (0, 5)        # 0-based: J1 / J6
SMOKE_MOVE_DEG = 3.0


def order_verify(host, n=20, gap_ms=0):
    """★ 验证固件的帧序修复(`frameseq`) —— 用**严格按序**重组器连打多帧命令。

    ■ 为什么非要有这一步
      `can_host.py` 的默认重组器已经被改成**容忍乱序**(目的: 让没重烧的老固件
      也能用)。于是 `--smoke` 全 PASS **证明不了固件修好了帧序** —— 宽容的上位机
      配没修的固件, 一样全过。要验证固件, 必须把重组器切回 **strict**(等同老上位机),
      只有固件真的按序发送才能全过。

    ■ 判据
      · strict_reject == 0 且 reordered == 0 ⇒ 帧序真的修好了, **任意**上位机可用 ✅
      · strict_reject  > 0                   ⇒ 帧序没修好, 只有打补丁的上位机能用 ⚠
    """
    for name in ("packets", "ordered", "reordered", "strict_reject"):
        Reassembler.STATS[name] = 0
    host.reasm.strict = True
    orders = []
    host.reasm.trace = orders.append

    print("  ── 帧序专项验证 ──")
    print("  重组器已切到 **strict**: 续帧号必须严格递增(等同未打补丁的老上位机)")
    print("  连打 %d 条多帧命令 GET_STATUS —— 期望到达顺序恒为 0,1,2,3" % n)

    ok = 0
    for _ in range(n):
        res = host.request_quiet(CMD_GET_STATUS, b"", timeout=0.4)
        if res and res[0] == 0x82:
            ok += 1
        if gap_ms:
            time.sleep(gap_ms / 1000.0)

    st = Reassembler.STATS
    miss = n - ok
    print("  命令成功 %d/%d   完整重组包 %d 个" % (ok, n, st["packets"]))
    if orders:
        head = orders[:6]
        print("  实测到达顺序(前 %d 包): %s"
              % (len(head), "  ".join(",".join(str(x) for x in o) for o in head)))
        bad = [o for o in orders if o != list(range(len(o)))]
        if bad:
            print("  ⚠ 乱序包样例: %s"
                  % "  ".join(",".join(str(x) for x in o) for o in bad[:4]))
    print("  统计: 完整包=%d  按序=%d  乱序=%d  strict拒收=%d"
          % (st["packets"], st["ordered"], st["reordered"], st["strict_reject"]))

    print("  " + "-" * 62)
    if st["strict_reject"] == 0 and st["reordered"] == 0 and miss == 0:
        print("  ✅ 帧序通过: 严格按序的上位机 0 次拒收、%d/%d 条多帧命令全成功" % (ok, n))
        print("     ⇒ 固件已按序发送, 客户自己写的上位机不必容忍乱序。")
        return 0
    if st["strict_reject"] == 0 and st["reordered"] == 0:
        print("  ⚠ 帧序本身没问题(0 乱序 0 拒收), 但有 %d 条命令没回 —— 另查丢帧/超时" % miss)
        return 1
    print("  ✗ 帧序仍未通过: strict 拒收 %d 次、乱序 %d 包" % (st["strict_reject"], st["reordered"]))
    print("     ⇒ 固件没带上 frameseq 修复(或烧的是旧版)。当前只有宽容的上位机能用。")
    return 1


def smoke(host, allow_move=False, monitor_s=3.0):
    results = []

    def step(name, ok, detail=""):
        results.append((name, ok, detail))
        print("  %s %-24s %s" % ("PASS" if ok else "FAIL", name, detail))
        return ok

    print("=" * 66)
    print("  一键冒烟测试   node=%d   运动测试: %s"
          % (host.node, "开(仅 J1/J6 ±%.0f°)" % SMOKE_MOVE_DEG if allow_move
             else "关 —— 全程只读, 机械臂不会动"))
    print("=" * 66)

    # --- 1) PING: 链路是否通 ---
    print("\n[1/6] PING 探活")
    res = host.request(CMD_PING, b"", timeout=0.5, label="PING")
    if not (res and res[0] == 0x81):
        print("""
  PING 不通 —— 后面全部无需再试。按顺序排查:
    ⓐ 机械臂控制台敲 `cn`  → hw init 必须 ok; 不是就敲 `e`(内部回环自检) 排除 MCU 侧
    ⓑ 机械臂控制台敲 `cp`  → RX 脚加下拉仍读 11111111 = 收发器上电且线接上了
    ⓒ 机械臂控制台敲 `can` → 看 node/baud 是否与本脚本 --node/--bitrate 一致
    ⓓ 波特率不一致是第一位原因(默认 500k); 第二是 ID 段 0x100+node
    ⓔ 只有一帧都不回才怀疑硬件; 若 `cn` 里 rec>0 说明收到了, 是过滤/波特率问题""")
        step("PING", False, "超时")
        _smoke_summary(results)
        return 1
    step("PING", True, "proto_ver=%d run_id=%d" % (res[1][0], res[1][1]) if len(res[1]) >= 2 else "")

    # --- 2) GET_STATUS: 六轴读数 ---
    print("\n[2/6] GET_STATUS 读六轴")
    res = host.request(CMD_GET_STATUS, b"", label="GET_STATUS")
    if not (res and res[0] == 0x82):
        step("GET_STATUS", False, "无响应或响应码不对")
        _smoke_summary(results)
        return 1
    st = decode_status_payload(res[1])
    bad = [i + 1 for i, v in enumerate(st["joints"]) if v is None]
    print("    关节(舵机度): " + "  ".join(
        "J%d=%s" % (i + 1, "FAIL" if v is None else "%.1f" % v)
        for i, v in enumerate(st["joints"])))
    print("    末端: X=%.1f Y=%.1f Z=%.1f pitch=%.1f  estop=%s traj=%s"
          % (st["x"], st["y"], st["z"], st["pitch"], st["estop"], st["traj"]))
    step("GET_STATUS", not bad and not st["estop"],
         "读失败的轴: J%s" % bad if bad else ("E-STOP 按下!" if st["estop"] else "六轴全部读到"))

    # --- 3) PARAM_COUNT: 参数表完整 (红线: 项数固定 24) ---
    print("\n[3/6] PARAM_COUNT 参数表自检")
    res = host.request_quiet(CMD_PARAM_COUNT)
    ok = bool(res and res[0] == 0xC2 and res[1][0] == 24)
    step("PARAM_COUNT == 24", ok,
         "实际 %s" % (res[1][0] if res and res[0] == 0xC2 else "无响应"))

    # --- 4) FK_QUERY: 正解链路 ---
    print("\n[4/6] FK_QUERY 正解")
    res = host.request_quiet(CMD_FK_QUERY)
    if res and res[0] == 0xA1:
        d = decode_fk_payload(res[1])
        print("    X=%.1f Y=%.1f Z=%.1f pitch=%.1f  read_ok=%s"
              % (d["x"], d["y"], d["z"], d["pitch"], d["read_ok"]))
        step("FK_QUERY", bool(d["read_ok"]), "read_ok=%s" % d["read_ok"])
    else:
        step("FK_QUERY", False, "无响应")

    # --- 5) 周期状态帧: 主动上报通路 ---
    print("\n[5/6] 周期状态帧 (%.1fs 窗口)" % monitor_s)
    n_st, _ = host.count_status_frames(monitor_s)
    step("收到周期状态帧", n_st > 0,
         "%d 帧 (hb 默认 1000ms ⇒ 期望 ~%.0f 帧)" % (n_st, max(monitor_s, 1.0)))

    # --- 6) 运动: 只在 --allow-move 时做 ---
    print("\n[6/6] 运动 + TRAJ_DONE 事件")
    if not allow_move:
        print("    跳过(未加 --allow-move)。要测运动: --smoke --allow-move")
        step("运动往返", True, "跳过")
    elif st["estop"]:
        step("运动往返", False, "E-STOP 按下, 拒绝运动")
    else:
        cur = list(st["joints"])
        tgt = list(cur)
        for ax in SMOKE_MOVE_AXES:
            tgt[ax] = round(cur[ax] + SMOKE_MOVE_DEG, 1)
        print("    当前 J1=%.1f J6=%.1f → 目标 J1=%.1f J6=%.1f (其余轴保持)"
              % (cur[0], cur[5], tgt[0], tgt[5]))
        res = host.request(CMD_MOVE_JOINTS, payload_move_joints(tgt),
                           label="MOVE_JOINTS(+%.0f°)" % SMOKE_MOVE_DEG)
        ack = bool(res and res[0] == RSP_ACK)
        done = wait_traj_done(host) if ack else None
        step("MOVE_JOINTS ACK", ack)
        step("TRAJ_DONE 事件", done is not None,
             "等到了" if done is not None else
             "超时(>25s) —— 若臂是【主臂】固件, 先敲 `ma 1` 放行(主臂默认只读, "
             "会 ACK 但不动作); 否则查是否等错事件 / 轨迹没跑完 / 事件被丢")
        if ack:
            print("    回原位…")
            host.request_quiet(CMD_MOVE_JOINTS, payload_move_joints(cur))
            wait_traj_done(host, timeout=25.0)

    return _smoke_summary(results)


def wait_traj_done(host, timeout=25.0):
    return host.wait_event(2, timeout)      # 2 = TRAJ_DONE


def _smoke_summary(results):
    n_fail = sum(1 for _, ok, _ in results if not ok)
    print("\n" + "=" * 66)
    for name, ok, detail in results:
        print("  %-4s %-24s %s" % ("PASS" if ok else "FAIL", name, detail))
    print("=" * 66)
    print("  %s (%d 项失败)" % ("全部通过 ✅" if n_fail == 0 else "有失败项 ❌", n_fail))
    if n_fail == 0:
        print("  → 下一步: 控制台敲 `can` 或 `cn`, 与上面的计数对照。")
    return 1 if n_fail else 0


# ============================================================
# --linktest: 物理链路自检（0x7E0 帧计数 + CAN 卡自己的总线状态）
#
# 为什么单列一步: `--ping` 超时只能证明"臂没回我", 它把三件事混在一起 ——
#   (a) 线上根本没有任何信号 (断线 / 收发器没上电 / 没共地)
#   (b) 电平对了但位速率或极性不对 (CANH-CANL 接反 / 两侧波特率不一致)
#   (c) 帧其实到了, 是被过滤器 / 节点号 / 判据挡掉 (纯软件问题)
# 这一条把 (a)(b) 与 (c) 一刀切开, 靠的是 CAN 的一个硬事实:
#
#   ★ 应答(ACK) 是**帧级**协议动作, 由 CAN 协议引擎完成,
#     **与接收过滤器无关** —— 一个把所有帧都过滤掉的节点, 照样会对
#     每一帧正确应答。所以: 只要总线上有另一个"活着且位速率一致"的节点,
#     本卡就不会累积总线错误; 反过来, 本卡如果迅速报 BUSLIGHT/BUSHEAVY/
#     BUSOFF, 那就是**没有节点应答** ⇒ 物理层真的没通 (或对面没上电)。
#
# 而 0x7E0 自检帧(见 arm_can_driver.c 的 ct)是最钝的探针: 不过协议层、
# 不进重组器、只做一次 ++, 所以它涨不涨**只反映链路**, 不反映任何判据逻辑。
#
# 现场用法(两半都对上才算通):
#   PC:  python tools/can_host.py --interface pcan --channel PCAN_USBBUS1 --linktest 200
#   臂:  敲 `ct` 看 "本板 自检帧 已收" 增加了多少 (增加≈200 ⇒ 通)
#   臂:  敲 `ct 200` 发帧, 上面这个 --linktest 会把它收到的帧逐条打出来 (反向验)
# ============================================================
_PCAN_USBBUS1 = 0x51          # PCAN_USBBUS1 的句柄值

_PCAN_STATUS_BITS = (
    (0x00000001, "XMTFULL   发送队列满"),
    (0x00000002, "OVERRUN   硬件接收溢出"),
    (0x00000004, "BUSLIGHT  ⚠ 总线告警: 错误计数 ≥128 —— 多半是没人在应答"),
    (0x00000008, "BUSHEAVY  ⚠⚠ 总线重错: 错误计数 >255"),
    (0x00000010, "BUSOFF    ⚠⚠⚠ 已 bus-off, 卡被硬件摘下线"),
    (0x00000040, "QOVERRUN  接收队列溢出(读得太慢)"),
    (0x00000080, "QXMTFULL  发送队列满"),
    (0x00000200, "NODRIVER  驱动没装"),
    (0x00000400, "HWINUSE   卡被别的程序占用"),
    (0x00040000, "ILLHW     无效硬件句柄"),
    (0x40000000, "INITIALIZE 初始化失败"),
)
_PCAN_OK_BITS = 0x00000020    # PCAN_ERROR_QRCVEMPTY = 接收队列空, 属于正常态


def pcan_status():
    """直接向 PCANBasic 问总线状态(不经过 python-can)。
    返回 (status_int, [说明...]) ；问不到(非 PCAN 卡 / DLL 不在)返回 None。"""
    try:
        import ctypes
        dll = ctypes.WinDLL("PCANBasic.dll")
    except Exception:                       # noqa: BLE001
        return None
    try:
        dll.CAN_GetStatus.argtypes = [ctypes.c_ushort,
                                      ctypes.POINTER(ctypes.c_uint32)]
        dll.CAN_GetStatus.restype = ctypes.c_uint32
        dll.CAN_GetErrorText.argtypes = [ctypes.c_uint32, ctypes.c_ushort,
                                         ctypes.c_char_p]
        dll.CAN_GetErrorText.restype = ctypes.c_uint32
        st = ctypes.c_uint32(0)
        if dll.CAN_GetStatus(ctypes.c_ushort(_PCAN_USBBUS1),
                             ctypes.byref(st)) != 0:
            return None
        buf = ctypes.create_string_buffer(256)
        dll.CAN_GetErrorText(st.value, ctypes.c_ushort(0x09), buf)
        text = buf.value.decode("ascii", "replace").strip()
    except Exception:                       # noqa: BLE001
        return None
    out = []
    if text:
        out.append(text)
    for bit, desc in _PCAN_STATUS_BITS:
        if st.value & bit:
            out.append(desc)
    return st.value, out


def _pcan_line(tag):
    r = pcan_status()
    if r is None:
        return
    val, notes = r
    if val & ~_PCAN_OK_BITS:
        print("  %-14s 0x%08X  %s" % (tag, val, " | ".join(notes)))
    else:
        print("  %-14s 0x%08X  正常(无总线错误)" % (tag, val))


def _bus_state_line(bus, tag):
    st = getattr(bus, "state", None)
    if st is not None:
        print("  %-14s python-can: %s" % (tag, st))


def linktest(host, bus, n=200, listen_s=2.0, gap_ms=10.0, bitrate=0, verbose=True):
    """0x7E0 帧计数 + 卡侧总线状态。返回 0=看起来通, 1=物理层有问题。"""
    print("\n" + "=" * 66)
    print("  linktest — 物理链路自检 (自检帧 ID 0x%03X, 发 %d 帧)"
          % (ID_TEST, n))
    print("=" * 66)
    print("  比特率 %d  节点 %d  (命令 0x%03X / 响应 0x%03X)"
          % (bitrate, host.node,
             ID_CMD_BASE + host.node, ID_RSP_BASE + host.node))
    _pcan_line("卡状态[发前]")
    _bus_state_line(bus, "卡状态[发前]")

    # 清掉残留接收队列, 免得把上一次的帧当成这一次的证据
    for _ in range(200):
        if bus.recv(timeout=0.0) is None:
            break

    m0 = pcan_status()
    tx = 0
    for i in range(n):
        data = bytes([0xC5, 0x3A, i & 0xFF, (i >> 8) & 0xFF, 0, 0, 0, 0])
        try:
            bus.send(_make_msg(ID_TEST, data))
            tx += 1
        except Exception as e:              # noqa: BLE001
            print("  ✗ 第 %d 帧发送失败: %s" % (i + 1, e))
            break
        if gap_ms:
            time.sleep(gap_ms / 1000.0)
    print("  已入卡 %d/%d 帧" % (tx, n))
    m1 = pcan_status()
    _pcan_line("卡状态[发后]")
    _bus_state_line(bus, "卡状态[发后]")

    # 反向: 听有没有人发帧给我们 (臂敲 `ct 200` 时这里应该刷出 200 帧)
    got = {}
    samples = []
    end = time.monotonic() + max(listen_s, 0.0)
    while time.monotonic() < end:
        msg = bus.recv(timeout=max(0.0, end - time.monotonic()))
        if msg is None:
            continue
        cid = msg.arbitration_id
        got[cid] = got.get(cid, 0) + 1
        if len(samples) < 8:
            samples.append((cid, bytes(msg.data)))
    print("  反向收到 %d 帧" % sum(got.values()))
    for cid, cnt in sorted(got.items()):
        print("    0x%03X x %d" % (cid, cnt))
    for cid, d in samples:
        print("    样例 0x%03X  %s" % (cid, d.hex(" ")))

    # ---- 判读 ----
    bus_err = bool(m1 and (m1[0] & ~_PCAN_OK_BITS))
    print("-" * 66)
    if bus_err:
        print("  ⚠ 卡在发送后报了总线错误(或者说读到的状态不正常)")
        print("     ⇒ 有可能是没有节点应答; 但**先别下结论**, 见下面「判据」一节")
    else:
        print("  卡状态干净。⚠ 但这**不能**证明线是通的 —— PCAN-USB 在我们实测过的")
        print("    「必然没有对端」条件下(位速率故意设成与臂不匹配)同样报 0x00000000,")
        print("    所以这个读数**不敏感, 不作为判据**。")
    print()
    print("  ── 唯一可信的判据: 两侧帧计数对拍(与卡的状态读数无关) ──")
    print("    本命令已经把 %d 帧 0x%03X 发到总线上了。现在去臂侧:" % (tx, ID_TEST))
    print("      ① 敲 `ct`   → 看 '本板 自检帧 已收' 是不是 ≈ %d" % tx)
    print("           涨了   ⇒ 卡→臂 方向通")
    print("           不涨   ⇒ 卡→臂 方向不通(线/极性/速率/终端/共地 之一)")
    print("      ② 敲 `ct 200` 发 200 帧, 同时本命令正开着监听窗口")
    print("           ⇒ 上面『反向收到』应该刷出 ~200 帧 ⇒ 臂→卡 方向通")
    print("    ③ 臂侧 `ct` / `cn` 自己的读数**极其灵敏**, 是更好的传感器:")
    print("         `ct` 发完若打印 `lec=3` 且 `tec` 快速涨 ⇒ 臂的帧**没有任何节点应答**")
    print("         `cn` 的 `bus pin: MSR.RX` = 1 隐性/0 显性 ⇒ 收发器在不在输出")
    print("       注意: 只有对面那块卡**已经被打开**(初始化)时才会应答;")
    print("             程序没跑 / 卡没插稳时, 臂看到 lec=3 是正常现象, 说明不了接线对错。")
    if sum(got.values()) == 0:
        print()
        print("  ⚠ 本窗口内反向一帧都没收到。若臂侧此刻没在发, 这是正常的;")
        print("    请配合臂侧 `ct 200` 重跑本命令再判定。")
    return 1 if bus_err else 0


def main():
    ap = argparse.ArgumentParser(description="六轴机械臂 CAN 从站上位机 (参考实现)")
    ap.add_argument("--interface", default="socketcan",
                    help="socketcan / slcan / pcan / gs_usb / canalystii ... 默认 socketcan")
    ap.add_argument("--backend", choices=("python", "usbcan"), default="python",
                    help="python=走 python-can; usbcan=厂商私有 DLL (ZLG VCI 兼容, "
                         "如 lyscan.dll, ⚠ 需 32 位 Python)")
    ap.add_argument("--dll", help="--backend usbcan 时厂商 DLL 路径（默认自动搜 lyscan.dll）")
    ap.add_argument("--devtype", type=int, default=4,
                    help="--backend usbcan 的 DevType（ZLG 原厂 3=USBCAN-I, 4=USBCAN-II; "
                         "各家不同, 用 can_usbcan.py --scan 问）")
    ap.add_argument("--channel", default="can0", help="通道名, 如 can0 或 COM8")
    ap.add_argument("--bitrate", type=int, default=500000, help="波特率, 默认 500000")
    ap.add_argument("--node", type=int, default=1, help="节点号(对应臂侧 can node), 默认 1")
    ap.add_argument("--selftest", action="store_true", help="离线自检, 不需要 CAN 卡")
    ap.add_argument("--probe", action="store_true",
                    help="★认卡+环境体检: python-can / 厂商DLL位数 / 各接口枚举 / 该敲哪条命令")
    ap.add_argument("--list", action="store_true", help="列出可用 CAN 通道")
    ap.add_argument("--ping", action="store_true", help="PING 探活")
    ap.add_argument("--status", action="store_true", help="GET_STATUS 读六轴与位姿")
    ap.add_argument("--fk", action="store_true", help="FK_QUERY 读末端位姿")
    ap.add_argument("--monitor", type=float, default=0.0, help="监听状态/事件帧 N 秒")
    ap.add_argument("--no-ping", action="store_true",
                    help="--monitor 前不发 PING。臂侧只在收到过发往本节点的帧之后才发状态帧"
                         "(没对端时主动发会让 bxCAN 的发送错误计数 TEC 每帧 +8, 十几秒就 bus-off),"
                         "所以默认先 PING 一次把状态帧打开")
    ap.add_argument("--move", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                    help="关节运动(舵机度 0~300)")
    ap.add_argument("--cart", nargs=4, type=float, metavar=("X", "Y", "Z", "PITCH"),
                    help="笛卡尔运动(mm, mm, mm, deg)")
    ap.add_argument("--speed", type=float, default=0.0, help="速度 deg/s, 0=用臂内默认")
    ap.add_argument("--accel", type=float, default=0.0, help="加速度 deg/s^2, 0=默认")
    ap.add_argument("--stop", action="store_true", help="STOP 停轨迹")
    ap.add_argument("--smoke", action="store_true",
                    help="★一键冒烟测试: PING→读写→状态帧→(可选)运动往返, 输出 PASS/FAIL 表")
    ap.add_argument("--allow-move", action="store_true",
                    help="允许 --smoke 发运动命令(只动 J1/J6 ±3°, 用当前读数当基准)。"
                         "不加则冒烟测试全程只读不碰")
    ap.add_argument("--linktest", type=int, nargs="?", const=200, metavar="N",
                    default=0,
                    help="★物理链路自检: 发 N 帧到 0x7E0 (默认 200) 并读 CAN 卡的总线"
                         "状态 + 反向监听。判据是臂侧 `ct` 的帧计数对拍; PCAN 卡自己的"
                         "状态读数不敏感(实测无对端时也报 OK), 只作参考")
    ap.add_argument("--link-listen", type=float, default=2.0,
                    help="--linktest 之后监听几秒 (接臂侧 `ct 200` 的反向帧), 默认 2")
    ap.add_argument("--quiet-evt", action="store_true",
                    help="不逐帧打印状态/事件帧(只计数)")
    ap.add_argument("--param-read", type=int, metavar="ID", help="读参数表某项")
    ap.add_argument("--param-count", action="store_true", help="读参数表项数")
    ap.add_argument("--order-verify", type=int, nargs="?", const=20, metavar="N",
                    help="★验证固件帧序: 用**严格按序**重组器(等同老上位机)连打 N 条多帧"
                         "命令(默认 20)。用于确认 frameseq 修复真生效 —— 注意 --smoke "
                         "证明不了这一点, 因为默认重组器已容忍乱序")
    ap.add_argument("--strict-order", action="store_true",
                    help="全局把重组器切成严格按序(模拟未打补丁的上位机)")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.probe:
        return probe()
    if args.list:
        list_channels(args.interface)
        return 0

    if args.backend == "usbcan":
        print("打开 厂商DLL后端 @%d bit/s, node=%d (cmd=0x%03X rsp=0x%03X evt=0x%03X)"
              % (args.bitrate, args.node,
                 ID_CMD_BASE + args.node, ID_RSP_BASE + args.node, ID_EVT_BASE + args.node))
    else:
        print("打开 %s:%s @%d bit/s, node=%d (cmd=0x%03X rsp=0x%03X evt=0x%03X)"
              % (args.interface, args.channel, args.bitrate, args.node,
                 ID_CMD_BASE + args.node, ID_RSP_BASE + args.node, ID_EVT_BASE + args.node))

    if args.backend == "usbcan":
        # 厂商私有 DLL (ZLG VCI 兼容) 后端。⚠ lyscan.dll 是 32 位 ⇒ 需 32 位 Python。
        # 借它时也要注入报文工厂 —— 该后端不依赖 python-can。
        global _MSG_FACTORY
        try:
            from can_usbcan import UsbCanBus, _Msg as _UcMsg
        except ImportError:
            print("✗ 找不到 tools/can_usbcan.py（请与 can_host.py 同目录）")
            return 2
        _MSG_FACTORY = lambda cid, data: _UcMsg(cid, data)
        try:
            bus = UsbCanBus(dll_path=args.dll, devtype=args.devtype,
                            kbps=args.bitrate // 1000)
            print("  实到 %s, DevType=%d" % (bus.path, args.devtype))
        except Exception as e:            # noqa: BLE001
            print("✗ 打开失败: %s" % e)
            print("  → 先跑 python tools/can_usbcan.py --scan 认 DevType")
            return 2
    else:
        try:
            import can
        except ImportError:
            print("✗ 当前 Python 里没有 python-can（离线自检/认卡不需要它，在线收发才需要）")
            print("  → 用装好依赖的解释器: "
                  r'"C:\Users\Administrator\.workbuddy\binaries\python\envs\default\Scripts\python.exe"')
            print("  → 或者先跑 python tools/can_host.py --probe 看该用哪个接口")
            return 2
        try:
            bus = open_bus_with_retry(args.interface, args.channel, args.bitrate)
        except Exception as e:            # noqa: BLE001
            print("✗ 打开 CAN 通道失败: %s" % e)
            print("  → python tools/can_host.py --probe    # 体检: 卡认到没 / 该填哪个接口")
            print("  → 若 --probe 里 channel_condition=2  ⇒ 卡被别的程序占着"
                  "(PCAN-View / CanAst.exe), 关掉它, 或把卡拔插一次")
            print("  → 厂商私有 DLL 的卡: --backend usbcan --dll <路径> --devtype <N>")
            return 2

    host = ArmCanHost(bus, args.node, strict=args.strict_order)
    host.verbose = not args.quiet_evt
    try:
        if args.order_verify:
            return order_verify(host, n=args.order_verify)
        if args.linktest:
            return linktest(host, bus, n=args.linktest, listen_s=args.link_listen,
                            bitrate=args.bitrate)
        if args.smoke:
            return smoke(host, args.allow_move)
        if args.ping:
            res = host.request(CMD_PING, b"", label="PING")
            if res and res[0] == 0x81 and len(res[1]) >= 2:
                print("    proto_ver=%d  run_id=%d" % (res[1][0], res[1][1]))
        if args.status:
            res = host.request(CMD_GET_STATUS, b"", label="GET_STATUS")
            if res and res[0] == 0x82:
                d = decode_status_payload(res[1])
                print("    关节(舵机度): " + "  ".join(
                    "J%d=%s" % (i + 1, "FAIL" if v is None else "%.1f" % v)
                    for i, v in enumerate(d["joints"])))
                print("    末端: X=%.1f Y=%.1f Z=%.1f pitch=%.1f mm/deg  read_ok=%s"
                      % (d["x"], d["y"], d["z"], d["pitch"], d["read_ok"]))
                print("    标志: estop=%s traj=%s corr=%s"
                      % (d["estop"], d["traj"], d["corr"]))
        if args.fk:
            res = host.request(CMD_FK_QUERY, b"", label="FK_QUERY")
            if res and res[0] == 0xA1:
                d = decode_fk_payload(res[1])
                print("    X=%.1f Y=%.1f Z=%.1f pitch=%.1f read_ok=%d"
                      % (d["x"], d["y"], d["z"], d["pitch"], d["read_ok"]))
        if args.param_count:
            res = host.request(CMD_PARAM_COUNT, b"", label="PARAM_COUNT")
            if res and res[0] == 0xC2:
                print("    参数表项数 = %d" % res[1][0])
        if args.param_read is not None:
            res = host.request(CMD_PARAM_READ, bytes([args.param_read]), label="PARAM_READ")
            if res and res[0] == 0xC0:
                print("    #%d = %.3f" % (res[1][0], rd_i32(res[1], 1) / 1000.0))
        if args.move:
            print("  MOVE_JOINTS " + " ".join("%.1f" % v for v in args.move) +
                  " (舵机度, 会被限位静默钳制)")
            host.request(CMD_MOVE_JOINTS,
                         payload_move_joints(args.move, args.speed, args.accel),
                         label="MOVE_JOINTS")
            print("    ACK 只代表「已启动」; 等下面的 TRAJ_DONE 事件才算走完")
        if args.cart:
            x, y, z, pitch = args.cart
            print("  MOVE_CART X=%.1f Y=%.1f Z=%.1f pitch=%.1f" % (x, y, z, pitch))
            host.request(CMD_MOVE_CART,
                         payload_move_cart(x, y, z, pitch, args.speed, args.accel),
                         label="MOVE_CART")
        if args.stop:
            host.request(CMD_STOP, b"", label="STOP")
        if args.monitor > 0:
            if not args.no_ping:
                # 臂侧"不向空气发帧": 没收到过发往本节点的帧时状态帧是停发的,
                # 所以先探一下活, 否则监听窗口里会一片安静。
                host.request(CMD_PING, b"", label="PING (解锁状态帧)")
            print("监听 %.1fs (Ctrl+C 结束)…" % args.monitor)
            end = time.monotonic() + args.monitor
            n = 0
            while time.monotonic() < end:
                msg = bus.recv(timeout=max(0.0, end - time.monotonic()))
                if msg is None:
                    continue
                if msg.arbitration_id == ID_EVT_BASE + args.node:
                    txt = decode_evt_frame(bytes(msg.data))
                    if txt:
                        n += 1
                        print("  " + txt)
            if n == 0:
                print("  共 0 帧。没有周期状态帧? 检查: 臂侧 `can` 的 hb 是否被设成 0; "
                      "或臂侧报 '状态帧停发中'(说明 PING 没到) / tec>=128。")
                print("  ⚠ 本进程的卡一直是 ACTIVE(会 ACK), 所以 tec>=128 的自锁在这里会自愈 —— "
                      "若怀疑是自锁, 改用 `tools/can_listen_count.py --passive`(不 ACK) 重测。")
            else:
                rate = n / max(args.monitor, 1e-6)
                print("  共 %d 帧 (%.2f 帧/s)。" % (n, rate))
                if 0.15 < rate < 0.25:
                    print("  ⚠ 节奏 ~5 s 一帧 ⇒ 臂处于 error-passive, 发的是 5 s 探路帧(0918G 起);"
                          " 恢复要 (TEC-127)×5 s, 稍等再测。")
                elif rate >= 0.5:
                    print("  ✔ 节奏 ~1 Hz ⇒ 周期状态帧正常。")
    except KeyboardInterrupt:
        print("\n中断退出")
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
