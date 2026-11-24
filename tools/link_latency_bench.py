#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""link_latency_bench.py — 串口 / 网口 / CAN 三链路「下发-回传」响应时间基准测试。

目的
    对同一条命令（默认 GET_STATUS）在三种物理链路上各打 N 次，
    测出「发出请求 -> 收到匹配 SEQ 的完整响应」的往返时延 RTT，
    统计 min/p50/p90/p99/max/mean/std，并排对比谁快、快多少。

公平性（这是本脚本的全部价值所在）
    三条链路**共用 can_host.py 的同一套编解码**：
      * 串口语义包 / 网口语义包 / CAN 逻辑包 —— CRC、字节序、字段全同；
      * 网口 = [SEQ|CMD|LEN|PAYLOAD|CRC16-LE]（UDP 天然分帧，无帧头）；
      * 串口 = [0xAA 0x55] + 语义包（靠帧头与文本控制台分流）；
      * CAN  = 按 plan_frames 切成多帧，且**续帧会乱序到达**（bxCAN 3 邮箱），
               必须靠 Reassembler 重组 —— 这本身就是 CAN 的真实代价。
    ⇒ 测出来的差值只来自「链路 + 承载协议」，不来自「我们发的内容不同」。

⚠⚠ 硬件互斥（必须先读）
    **CAN2 = PB12/PB13，而 RMII 的 ETH_RXD1/TXD1 也是 PB12/PB13** ——
    同一对引脚。所以：
      * 烧 `LINK=net` 固件 ⇒ 网口可用，**CAN 必超时**（内核都没跑）；
      * 烧 `LINK=can` 固件 ⇒ CAN 可用，**网口必超时**。
    ⇒ 想对比三条链路，必须**分两轮烧不同固件**，用 `--out` 分别存结果，
      最后 `--report` 合并出总表。串口两轮都在（COM11）。

⚠ 安全
    * 默认 `--cmd status` —— **只读，臂完全不动**。
    * `move_joints` / `stream_joints` 走控制通路，**必须显式 `--allow-move`**；
      默认把目标设成「当前角」= 原地不动，只为测控制类的真实开销。
    * 想让臂**一直在动**同时测 RTT ⇒ 加 `--motion`（见下）。

⚠⚠ 两套角度量纲（本脚本最容易搞错的地方，改代码前必读）
    GET_STATUS(0x82) / FK_QUERY(0xA1) / MOVE_JOINTS(0x10)  →  舵机度 0~360
    STREAM_JOINTS(0x13)                                    →  逻辑角 (-180,180]
        舵机度 = (SERVO_OFFSET[j] + JOINT_DIR[j] × 逻辑角) mod 360
        逻辑角 = JOINT_DIR[j] × (舵机度 − SERVO_OFFSET[j])，归一 (-180,180]
        JOINT_DIR = {-1,+1,+1,+1,+1,+1}（kinematics.h:93）
        SERVO_OFFSET = {180×6} 出厂默认，**但会被 Home 校零改写**！
    ⇒ **把 GET_STATUS 的角度直接喂给 STREAM_JOINTS 是错的**（整体偏 SERVO_OFFSET，
      厂值下 J2~J6 会偏 180°，臂会甩一大下）。脚本按 `--stream-offset` 换算，
      默认 180×6；Home 零点重标定过就必须传真实值。
      （依据：docs/网口指令大全_2026-09-21.md §9）

🎯 运动模式 `--motion`：让臂「一直在动」的同时测 RTT
    背景：真实遥操作时固件主循环同时在跑 S 曲线插补 / 流模式直写 + 舵机总线
    同步写，控制类命令的 RTT 会明显比空闲时差。要量这个就必须让臂持续运动。
    做法：读一次起始位姿 base，之后每帧目标 = base 在**单轴**上 ±amp 正弦摆动。
      * 有界 —— 永不超出 base±amp ⇒ 不会跑去撞桌面/基座
      * 连续 —— 正弦连续变化 ⇒ 臂一直在动，不会走走停停
      * 单轴 —— 默认 **J5**（只改夹爪朝向，不改变末端 XYZ）⇒ 风险最低
      * 绝不用 Z 做摆动（唯一能把末端砸向桌面的方向）
    流模式 + 运动前会做**回读自检**（探 3° 后回读）：摆动轴必须真的动了、
    其它轴不许被拖走 —— 不过就中止。这是红线 67 家族的「回读守卫」。

用法（工程根目录）
    # 第 1 轮：烧 LINK=net 固件
    python tools/link_latency_bench.py --link serial,net --n 500
    # 第 2 轮：烧 LINK=can 固件
    python tools/link_latency_bench.py --link serial,can --n 500
    # 合并出总表
    python tools/link_latency_bench.py --report
    # 让臂一直动 + 测 RTT（控制类，务必先读 --motion 的安全说明）
    python tools/link_latency_bench.py --link serial,net --cmd move_joints --motion --allow-move --n 300
    # 离线自检（不需要板子 / CAN 卡 / 串口）
    python tools/link_latency_bench.py --selftest

常用选项
    --cmd status|fk|ping|param_count|move_joints|stream_joints
    --n 500 --warmup 10 --gap 0 --timeout 0.3
    --motion --motion-axis 5 --motion-amp 5 --motion-period 2 --no-motion-restore
    --stream-offset 180,180,180,180,180,180   Home 校零过就必须传真实值
    --serial-port COM11 --baud 115200
    --udp-ip 192.168.7.120 --udp-port 5000
    --can-interface pcan --can-channel PCAN_USBBUS1 --can-bitrate 500000 --can-node 1
    --out outputs/latency_bench              结果 JSON 目录
    --csv outputs/latency_bench/samples.csv  可选：原始样本导出
    --quiet-log                              开测前把固件日志压到 0（只报错）
"""
import argparse
import glob
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_host as ch  # noqa: E402  复用编解码 —— 保证三条链路字节级可比


# ============================================================
# 命令表
# ============================================================
#  name -> (请求码, 期望响应码, 需要构造 payload?)  None = 运行时按当前角生成
CMD_SPEC = {
    "ping":          (ch.CMD_PING,          0x81,            False),
    "status":        (ch.CMD_GET_STATUS,    0x82,            False),
    "fk":            (ch.CMD_FK_QUERY,      0xA1,            False),
    "param_count":   (ch.CMD_PARAM_COUNT,   0xC2,            False),
    "move_joints":   (ch.CMD_MOVE_JOINTS,   ch.RSP_ACK,      True),
    "stream_joints": (ch.CMD_STREAM_JOINTS, ch.RSP_ACK,      True),
}
NEEDS_MOVE = ("move_joints", "stream_joints")

# ⭐ 命令分类：是否触碰硬件（读舵机总线）。
#   ⚠⚠ 这是本工具最容易踩的判读陷阱 —— 实测（2026-09-22 / ARM-NET-0921A / 网口忙等）：
#        ping        0.29 ms  ┐ 纯协议 + 内存
#        param_count 0.37 ms  ┘
#        fk          6.72 ms  ┐ 含「读六轴舵机实际位置」≈6.1 ms
#        status      6.53 ms  ┘
#   ⇒ 用 fk/status 比「链路快慢」会被那 6.1 ms 完全淹没：
#       网口 6.41ms vs 串口 11.14ms ⇒ 只差 1.7 倍（假象）
#       网口 0.29ms vs 串口  3.21ms ⇒ 差 11.3 倍（真值）
#   ⇒ 比「链路本身」必须用 ping；比「端到端控制体验」才用 status/move。
HW_TOUCHING = ("status", "fk")
HW_COST_MS = 6.1        # 读六轴舵机的实测固定开销（仅用于提示，不参与统计）

# 响应码 -> 人类可读
RSP_NAME = {
    0x7F: "ACK", 0x7E: "NACK", 0x81: "PING", 0x82: "STATUS",
    0xA1: "FK", 0xC0: "PARAM_READ", 0xC2: "PARAM_COUNT",
}

# ⭐ NACK 的 PAYLOAD = {orig_cmd, err}（arm_control_protocol.h:72），
#    err 就是 arm_control_api.h:22 那 8 个值 ⇒ 收到 NACK 必须把 err 解出来，
#    否则只能看到"坏响应 1240 次"，完全不知道该查什么。
API_ERR_NAME = {
    0: "OK",
    1: "PARAM(参数非法:长度/范围/ID)",
    2: "CRC(请求帧 CRC 校验失败 ⇒ 传输层丢字节)",
    3: "UNREACH(IK 不收敛/不可达)",
    4: "LIMIT(超限位/桌面/立柱/单帧跳变被拒)",
    5: "ESTOP(急停激活)",
    6: "FK_VERIFY(IK 解 FK 回代超差)",
    7: "READ(读失败: 舵机总线或 Flash 参数区)",
}


def decode_nack(pl):
    """从 NACK 负载解出 (orig_cmd, err) 与可读原因。"""
    if not pl or len(pl) < 2:
        return None, None, "NACK 负载不完整(%s)" % (list(pl) if pl else "空")
    orig, err = pl[0], pl[1]
    return orig, err, "cmd=0x%02X err=%d %s" % (orig, err, API_ERR_NAME.get(err, "?"))


# ============================================================
# 统计
# ============================================================
def percentile(sorted_vals, p):
    """线性插值百分位（与 numpy 默认 'linear' 一致，避免额外依赖）。"""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def summarize(lats):
    """lats: 成功样本的毫秒列表 -> dict。"""
    if not lats:
        return None
    s = sorted(lats)
    n = len(s)
    mean = sum(s) / n
    var = sum((x - mean) ** 2 for x in s) / n if n > 1 else 0.0
    return {
        "n": n,
        "min": s[0],
        "p50": percentile(s, 50),
        "p90": percentile(s, 90),
        "p99": percentile(s, 99),
        "max": s[-1],
        "mean": mean,
        "std": math.sqrt(var),
    }


# ============================================================
# 传输层：三种链路统一接口
#   open() / close() / request_once(cmd, payload, timeout) -> (rcmd, payload_bytes)
#   失败一律抛 TimeoutError；坏包计入 self.bad 并继续等（不重发！）
#   ⚠ 绝不重发：重发会把"超时等待"算进 RTT，统计就废了。
# ============================================================
class Transport:
    name = "?"
    def __init__(self):
        self.bad = 0          # 坏包/CRC错/序号不符次数

    def open(self):  pass
    def close(self): pass
    def request_once(self, cmd, payload, timeout): raise NotImplementedError


# ---------- 串口 (USART6 控制台口) ----------
# 串口语义包帧头（固件 arm_control_protocol.h 的 APROTO_HEAD0/1）。
# can_host.py 只管 CAN（靠仲裁 ID 分段，不需要帧内头），所以这里自己定义。
# 0xAA 0x55 是"文本控制台不会输出"的字节对 ⇒ 与文本日志天然分流。
HEAD0, HEAD1 = 0xAA, 0x55


def build_serial_frame(seq, cmd, payload):
    body = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + payload
    crc = ch.crc16(body)
    return bytes([HEAD0, HEAD1]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_stream(buf):
    """从字节流里找第一帧。返回 (drop_len, frame or None)。

    drop_len = 可以从缓冲头部安全丢弃的字节数（文本日志、噪声、假同步头）。
    ⚠ 控制台串口上**文本命令与二进制帧共存**（固件 ProtoSer_Feed 返回 0 表示
      "文本字节，交还控制台"），所以必须能跳过任意文本再同步。
    """
    i, n = 0, len(buf)
    while i + 5 <= n:
        if buf[i] == HEAD0 and buf[i + 1] == HEAD1:
            ln = buf[i + 4]
            total = 5 + ln + 2
            if i + total > n:
                return i, None                     # 帧不完整，保留等更多字节
            body = buf[i + 2:i + 5 + ln]
            crc_got = buf[i + 5 + ln] | (buf[i + 6 + ln] << 8)
            if ch.crc16(body) == crc_got:
                return i + total, (buf[i + 2], buf[i + 3], bytes(buf[i + 5:i + 5 + ln]))
            i += 1                                 # 假同步头，继续找
            continue
        i += 1
    return i, None


class SerialTransport(Transport):
    def __init__(self, port, baud, name=None):
        super().__init__()
        self.port, self.baud = port, baud
        self.name = name or ("串口 " + port)
        self.ser = None
        self.seq = 0

    def open(self):
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.001)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser:
            try: self.ser.close()
            except Exception: pass

    def request_once(self, cmd, payload, timeout):
        self.seq = (self.seq + 1) & 0xFF
        seq = self.seq
        self.ser.reset_input_buffer()
        self.ser.write(build_serial_frame(seq, cmd, payload))
        self.ser.flush()

        buf = bytearray()
        deadline = time.perf_counter() + timeout
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                raise TimeoutError("serial timeout")
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                buf += chunk
            drop, frame = parse_stream(buf)
            if drop:
                del buf[:drop]
            if frame:
                rseq, rcmd, pl = frame
                if rseq != seq:
                    self.bad += 1                  # 别人的帧/残留，丢掉继续等
                else:
                    return rcmd, pl
                continue
            if not chunk:
                time.sleep(0.0005)                 # 别把 CPU 打满


# ---------- 网口 (UDP) ----------
class NetTransport(Transport):
    def __init__(self, ip, port, name=None):
        super().__init__()
        self.addr = (ip, port)
        self.name = name or ("网口 UDP %s:%d" % (ip, port))
        self.sock = None
        self.seq = 0

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # ⭐ 不绑固定端口：固件"回包到请求源端口"，OS 自动分配即可
        #   （绑死同端口在本机自测时会自冲突，见 arm_net_host.py 注释）

    def close(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass

    @staticmethod
    def build_packet(seq, cmd, payload):
        out = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + payload
        crc = ch.crc16(out)
        return out + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    @staticmethod
    def parse_packet(data):
        if len(data) < 5:
            raise ValueError("too short")
        seq, cmd, ln = data[0], data[1], data[2]
        if 5 + ln != len(data):
            raise ValueError("LEN mismatch")
        crc_got = data[3 + ln] | (data[3 + ln + 1] << 8)
        if crc_got != ch.crc16(data[:3 + ln]):
            raise ValueError("CRC")
        return seq, cmd, data[3:3 + ln]

    def request_once(self, cmd, payload, timeout):
        self.seq = (self.seq + 1) & 0xFF
        seq = self.seq
        self.sock.sendto(self.build_packet(seq, cmd, payload), self.addr)
        deadline = time.perf_counter() + timeout
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                raise TimeoutError("udp timeout")
            self.sock.settimeout(left)
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                raise TimeoutError("udp timeout")
            try:
                rseq, rcmd, pl = self.parse_packet(data)
            except ValueError:
                self.bad += 1
                continue
            if rseq != seq:
                continue
            return rcmd, pl


# ---------- CAN ----------
class CanTransport(Transport):
    def __init__(self, interface, channel, bitrate, node=1, name=None):
        super().__init__()
        self.interface, self.channel, self.bitrate, self.node = interface, channel, bitrate, node
        self.name = name or ("CAN %s@%dk" % (channel, bitrate // 1000))
        self.bus = None
        self.reasm = None
        self.seq = 0
        self.cmd_id = ch.ID_CMD_BASE + node
        self.rsp_id = ch.ID_RSP_BASE + node

    def open(self):
        self.bus = ch.open_bus_with_retry(self.interface, self.channel, self.bitrate)
        self.reasm = ch.Reassembler(self.node)

    def close(self):
        if self.bus:
            try: self.bus.shutdown()
            except Exception: pass

    def request_once(self, cmd, payload, timeout):
        self.seq = (self.seq + 1) & 0xFF
        seq = self.seq
        for f in ch.build_frames(seq, cmd, payload):
            self.bus.send(ch._make_msg(self.cmd_id, f))
        deadline = time.perf_counter() + timeout
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                raise TimeoutError("can timeout")
            msg = self.bus.recv(timeout=left)
            if msg is None:
                continue
            cid = msg.arbitration_id
            if cid != self.rsp_id:
                continue                                # 事件帧/状态帧，本工具不统计
            out = self.reasm.feed(cid, bytes(msg.data))
            if not out:
                continue
            rseq, rcmd, pl, ok = out
            if not ok:
                self.bad += 1
                continue
            if rseq != seq:
                continue
            return rcmd, pl


# ============================================================
# 基准测试主体
# ============================================================
class BenchResult:
    def __init__(self, name):
        self.name = name
        self.lats = []
        self.n_fail = 0        # 超时
        self.n_badrsp = 0      # 响应码不符
        self.n_badpkt = 0      # CRC/序号坏包（传输层计）
        self.first_err = None
        self.elapsed = 0.0
        self.bucket_stats = None      # 长时间模式的按时间分桶统计
        self.err_hist = {}            # NACK 原因直方图 {"err=4 LIMIT...": 次数}

    @property
    def n_ok(self):
        return len(self.lats)

    @property
    def n_total(self):
        return self.n_ok + self.n_fail + self.n_badrsp

    def summary(self):
        s = summarize(self.lats)
        d = {"name": self.name, "n_ok": self.n_ok, "n_total": self.n_total,
             "fail_timeout": self.n_fail, "fail_badrsp": self.n_badrsp,
             "bad_pkt": self.n_badpkt, "elapsed_s": self.elapsed}
        if s:
            d.update(s)
            d["rate"] = self.n_ok / self.elapsed if self.elapsed > 0 else 0.0
        if self.first_err:
            d["first_err"] = self.first_err
        if self.err_hist:
            d["nack_reasons"] = dict(self.err_hist)
        if self.bucket_stats:
            # ⭐ 长时间模式：把每桶统计一起落盘，供事后看「有没有越跑越差」
            #    （min/p90 也要落，否则事后无法用 print_buckets 还原完整分桶表）
            d["buckets"] = [{"t0": b["t0"], "t1": b["t1"], "n": b["n"],
                             "min": b["min"], "p50": b["p50"], "p90": b["p90"],
                             "p99": b["p99"], "max": b["max"]}
                            for b in self.bucket_stats]
        return d


# ============================================================
# 运动模式：让臂【一直在动】的同时测 RTT
# ============================================================
MOTION_AXIS_DEFAULT   = 5        # J5：只改夹爪朝向，不改变末端 XYZ ⇒ 最安全
MOTION_AMP_DEFAULT    = 5.0      # 度（±）
MOTION_PERIOD_DEFAULT = 2.0      # 秒
MOTION_MAX_AMP        = 15.0     # 「小幅摆动」的推荐上限（不传 --motion-amp 时的安全参考）
MOTION_AMP_HARD_MAX   = 90.0     # ⭐ 硬上限：单轴摆动幅度**任何情况下**都不许超过这个
                                 #    要更大必须改代码 —— 不给「先跑跑看」的机会
DURATION_DEFAULT      = 600.0    # ⭐ 长时间模式默认时长：10 分钟
BUCKET_DEFAULT        = 60.0     # 分桶统计粒度（秒）⇒ 看 10 分钟内「有没有越跑越差」

# ⭐ 各关节真实运行限位（舵机度），来自控制台 `L` 命令实测（2026-09-22）。
#    摆幅会被自动夹到这个区间内侧（留 SAFE_MARGIN_DEG 余量），所以「大跨度」也撞不到限位。
DEFAULT_JOINT_LIMITS  = ((63.0, 303.0), (75.0, 284.0), (76.0, 271.0),
                         (71.0, 284.0), (12.0, 351.0), (138.0, 266.0))
SAFE_MARGIN_DEG       = 5.0      # 摆到离限位这么近就停，不顶死

SERVO_OFFSET_FACTORY  = 180.0                                  # kinematics.c:119
JOINT_DIR_FACTORY     = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0)        # kinematics.h:93

PROBE_AMP_DEG         = 3.0      # 流模式自检的试探幅度
PROBE_MOVE_TOL        = 1.0      # 摆动轴至少动这么多度 ⇒ 流帧真的写到了舵机
PROBE_CROSS_TOL       = 6.0      # 其它轴被动超过这么多度 ⇒ 量纲/限位把别处拖走了
STREAM_MAX_JUMP_HINT  = 25.0     # 固件 main.c:1809 STREAM_MAX_JUMP_DEG（只用于提示文案）


def norm180(a):
    """归一化到 (-180, 180]。"""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def _off6(offset):
    """SERVO_OFFSET 允许写成标量（统一值）或 6 个值。"""
    if isinstance(offset, (int, float)):
        return [float(offset)] * 6
    return [float(v) for v in offset]


def servo_to_logical(servo, offset=SERVO_OFFSET_FACTORY, dirs=JOINT_DIR_FACTORY):
    """舵机度 -> 逻辑角。逻辑角 = JOINT_DIR × (舵机度 − SERVO_OFFSET)，归一 (-180,180]。"""
    off = _off6(offset)
    return [norm180(dirs[j] * (float(servo[j]) - off[j])) for j in range(6)]


def logical_to_servo(logic, offset=SERVO_OFFSET_FACTORY, dirs=JOINT_DIR_FACTORY):
    """逻辑角 -> 舵机度。舵机度 = (SERVO_OFFSET + JOINT_DIR × 逻辑角) mod 360。"""
    off = _off6(offset)
    return [(off[j] + dirs[j] * float(logic[j])) % 360.0 for j in range(6)]


def parse6(text, what, cast=float):
    v = [cast(x) for x in str(text).replace("，", ",").split(",") if x.strip() != ""]
    if len(v) != 6:
        raise SystemExit("✗ %s 需要 6 个值（逗号分隔），收到 %d 个: %r" % (what, len(v), text))
    return v


def _series(vals):
    return " ".join("%7.1f" % v for v in vals)


class MotionPlan:
    """围绕起始位姿做**多轴正弦摆动** ⇒ 臂持续运动，但永远不超出 base±amp。

    摆动定义在**舵机度**上；逻辑角由它换算 ⇒ 两个量纲都只动这几轴。
    多个轴用**不同相位**（每轴错开 90°）⇒ 不会同时到极值，运动更像真实作业。
    """

    def __init__(self, base_servo, cmd_name, axes, amps, period,
                 offset, dirs, limits=None, v=0.0, accel=0.0):
        self.base_servo = [float(x) for x in base_servo]
        self.cmd = cmd_name
        self.axes = [int(x) for x in (axes if isinstance(axes, (list, tuple)) else [axes])]
        if not isinstance(amps, (list, tuple)):
            amps = [float(amps)] * len(self.axes)
        self.amps = [abs(float(a)) for a in amps]
        if len(self.amps) != len(self.axes):
            raise SystemExit("✗ --motion-amp 的个数(%d) 要和 --motion-axis 的轴数(%d) 一致"
                             % (len(self.amps), len(self.axes)))
        self.period = max(0.2, float(period))
        self.offset = _off6(offset)
        self.dirs = tuple(dirs)
        self.limits = limits or DEFAULT_JOINT_LIMITS
        self.v, self.accel = float(v), float(accel)
        self.base_logic = servo_to_logical(self.base_servo, self.offset, self.dirs)
        self.clamped = []            # [(轴, 想要, 实际)] —— 被限位夹过的记录
        for k, ax in enumerate(self.axes):
            lo, hi = self.limits[ax - 1]
            room = min(self.base_servo[ax - 1] - (lo + SAFE_MARGIN_DEG),
                       (hi - SAFE_MARGIN_DEG) - self.base_servo[ax - 1])
            if room < self.amps[k]:
                self.clamped.append((ax, self.amps[k], max(0.0, room)))
                self.amps[k] = max(0.0, room)

    def amp_for(self, axis):
        return self.amps[self.axes.index(int(axis))] if int(axis) in self.axes else 0.0

    def servo_target(self, t):
        """t=0 时**所有轴都严格等于 base**（见 _env），随后各轴按各自相位展开。

        ⚠⚠ 为什么必须这样：固件对每个流帧检查「单轴变化 > STREAM_MAX_JUMP_DEG(25°)
        就 NACK err=4」，而且被拒时 `stream_prev` **不更新** ⇒ 一帧超限会连锁拒绝。
        早期版本给每轴加了 90° 相位偏移，于是 t=0 时 J6 直接 = base+amp
        （±30° 那一组正好 30° > 25°）⇒ **第一帧就被拒**，实测 11.4% 的帧被 NACK。
        修法：乘以一个从 0 平滑升到 1 的启动包络 ⇒ 起步时目标 = base，无跳变。
        """
        env = self._env(t)
        s = list(self.base_servo)
        for k, ax in enumerate(self.axes):
            s[ax - 1] += self.amps[k] * env * math.sin(
                2.0 * math.pi * t / self.period + k * math.pi / 2.0)
        return s

    def _env(self, t):
        """启动包络：一个周期内 0 → 1 平滑升起（正弦半波），之后恒为 1。

        作用有两个：① 起步无跳变（躲开固件的跳变门限）；② 起停更柔和。
        代价：第一个周期内的摆幅是渐变的（帧均值统计不受影响）。
        """
        if t >= self.period:
            return 1.0
        return 0.5 * (1.0 - math.cos(math.pi * max(0.0, t) / self.period))

    def max_step_deg(self, dt):
        """估算「相邻两帧之间」单轴最大变化量（度）—— 用来对照固件的跳变门限。

        取包络上升段的保守上界：amp × (2π/T + π/T) × dt。
        """
        if dt <= 0 or not self.amps:
            return 0.0
        return max(a * (2.5 * math.pi / self.period) * dt for a in self.amps)

    def payload(self, t):
        st = self.servo_target(t)
        if self.cmd == "move_joints":
            return ch.payload_move_joints(st, self.v, self.accel)     # 舵机度，同量纲
        return ch.payload_stream_joints(                                     # 逻辑角！
            servo_to_logical(st, self.offset, self.dirs))

    @staticmethod
    def tag_of(axes, amps, period):
        return "motion-%s-%sdeg-T%gs" % (
            "+".join("J%d" % a for a in axes),
            "+".join("%g" % a for a in amps), period)

    def tag(self):
        return MotionPlan.tag_of(self.axes, self.amps, self.period)


def read_pose_servo(transport):
    """读六关节当前角（**舵机度**，与 GET_STATUS/FK 同量纲）。任一轴 -32768 ⇒ 抛错。"""
    rcmd, pl = transport.request_once(ch.CMD_GET_STATUS, b"", 1.0)
    if rcmd != 0x82:
        raise RuntimeError("GET_STATUS 返回 0x%02X（期望 0x82）" % rcmd)
    cur = ch.decode_status_payload(pl)["joints"]
    if any(v is None for v in cur):
        raise RuntimeError("有轴读失败（-32768），拒绝构造目标")
    return [float(v) for v in cur]


def read_pose_retry(transport, tries=4, gap=0.25, what="位姿"):
    """带重试的位姿读取。

    ⚠ 动机：本机 J1 存在**间歇性读失败**（控制台 `s` 的 Moving 列会亮 `Err`）。
    单次失败就让 10 分钟的长跑白跑是不可接受的 ⇒ 重试几次。
    但**绝不静默降级**：全部重试都失败仍然抛错。
    """
    last = None
    for k in range(max(1, tries)):
        try:
            return read_pose_servo(transport)
        except Exception as e:                                # noqa: BLE001
            last = e
            if k < tries - 1:
                print("  （读%s第 %d 次失败: %s ⇒ 重试）" % (what, k + 1, e))
                time.sleep(gap)
    raise RuntimeError("读%s连续 %d 次失败，最后一次: %s" % (what, tries, last))


def motion_probe_stream(transport, plan, quiet):
    """流模式运动前的**回读自检**（红线 67 家族：绝不做「我发的是对的」这种假设）。

    做法：目标只从 base 挪 PROBE_AMP_DEG（只动摆动轴），连发 4 帧，等 0.6 s，
    再回读实际位姿，判定两件事：
      * 摆动轴真的动了 (>= PROBE_MOVE_TOL)  ⇒ 流帧确实写到了舵机
      * 其它轴被动 < PROBE_CROSS_TOL        ⇒ 量纲/限位没有把别处拖走
    任一条不成立 ⇒ 返回 (False, 原因, 残差)，调用方**必须中止**，不许先跑跑看。
    """
    axis = plan.axes[0] - 1
    probe_amp = min(PROBE_AMP_DEG, max(0.5, plan.amps[0]))
    probe_servo = list(plan.base_servo)
    probe_servo[axis] += probe_amp
    pl = ch.payload_stream_joints(servo_to_logical(probe_servo, plan.offset, plan.dirs))

    print("  [自检] 流模式试探 +%g° (J%d)，发 4 帧后回读…" % (probe_amp, plan.axes[0]))
    try:
        for _ in range(4):
            transport.request_once(ch.CMD_STREAM_JOINTS, pl, 0.5)
            time.sleep(0.05)
    except TimeoutError:
        return False, "试探帧无 ACK（流模式命令没被受理）", None
    time.sleep(0.6)
    try:
        after = read_pose_retry(transport, what="自检回读")
    except Exception as e:                                    # noqa: BLE001
        return False, "回读失败: %s" % e, None

    delta = [after[j] - plan.base_servo[j] for j in range(6)]
    moved_axis = abs(delta[axis])
    cross = max([abs(delta[j]) for j in range(6) if j != axis] or [0.0])
    print("  [自检] 回读 Δ: %s" % _series(delta))
    if moved_axis < PROBE_MOVE_TOL:
        return False, ("摆动轴没动（Δ=%.2f°，< %.1f°）—— 流帧没写到舵机。"
                       "先查：日志有没有 Stream reject（跳变/姿态被拒）、"
                       "或 `sf 0` 把跟随关了" % (moved_axis, PROBE_MOVE_TOL)), delta
    if cross > PROBE_CROSS_TOL:
        return False, ("其它轴被动 %.2f°（> %.1f°）—— 量纲或限位钳位在拖别的轴。"
                       "首要怀疑：SERVO_OFFSET 不是出厂 180（Home 校零改过）"
                       "⇒ 用 --stream-offset 传真实值；"
                       "次要：红线 68（JOINT_MIN_RAW/MAX_RAW 未装载 ⇒ 六轴被推向 raw 0）"
                       % (cross, PROBE_CROSS_TOL)), delta
    return True, "摆动轴动 %.2f°，其它轴最大被动 %.2f°" % (moved_axis, cross), delta


def restore_pose(transport, base_servo, v=0.0, accel=0.0, wait=6.0):
    """结束后把臂送回起始位姿。走 MOVE_JOINTS（舵机度，同量纲，限位在固件侧）。"""
    print("\n  回位中…（MOVE_JOINTS 回起始位姿）")
    try:
        transport.request_once(ch.CMD_MOVE_JOINTS,
                               ch.payload_move_joints(base_servo, v, accel), 1.0)
    except TimeoutError:
        print("  ⚠ 回位命令无 ACK —— 请手动确认臂的位置")
        return False
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < wait:
        time.sleep(0.3)
        try:
            rcmd, pl = transport.request_once(ch.CMD_GET_STATUS, b"", 1.0)
        except TimeoutError:
            continue
        if rcmd != 0x82:
            continue
        if not (ch.decode_status_payload(pl)["flags"] & 2):        # traj_active 清 0
            print("  ✓ 已回到起始位姿（耗时 %.1fs）" % (time.perf_counter() - t0))
            return True
    print("  ⚠ 回位未在 %.0fs 内确认结束，请目视确认" % wait)
    return False


def bucketize(lats, ts, bucket):
    """把长跑的样本按**时间桶**切开 ⇒ 看「有没有越跑越差」。

    返回 [{"t0","t1","n", ...分位数...}]。ts 与 lats 一一对应（相对起始时刻的秒）。
    """
    if not lats or not ts:
        return []
    out = []
    lo = 0.0
    while lo < ts[-1]:
        hi = lo + bucket
        sub = [l for l, t in zip(lats, ts) if lo <= t < hi]
        if sub:
            d = summarize(sub)
            d["t0"], d["t1"] = lo, hi
            out.append(d)
        lo = hi
    return out


def print_buckets(res):
    """长时间运行的稳定性表：每个时间桶一行。"""
    bs = res.bucket_stats
    hdr = ["桶(分钟)", "样本", "min", "p50", "p90", "p99", "max"]
    w = [12, 8, 8, 9, 9, 9, 10]
    rule = sum(w) + 2
    print("")
    print("=" * rule)
    print("  ⏱ %s 长时间稳定性（每桶 %g 秒）" % (res.name, bs[0]["t1"] - bs[0]["t0"]))
    print("=" * rule)
    print("  " + "".join(str(c).ljust(x) for c, x in zip(hdr, w)))
    print("  " + "-" * (rule - 2))
    for d in bs:
        print("  " + "".join(str(c).ljust(x) for c, x in zip([
            "%.1f~%.1f" % (d["t0"] / 60.0, d["t1"] / 60.0), d["n"],
            "%.2f" % d["min"], "%.2f" % d["p50"], "%.2f" % d["p90"],
            "%.2f" % d["p99"], "%.2f" % d["max"]], w)))
    print("=" * rule)
    solid, runts = _solid_buckets(bs)
    if runts:
        print("  ⚠ 忽略 %d 个「残桶」(样本数 < 正常桶的一半)：%s"
              % (len(runts), ", ".join("%.0f~%.0fmin(n=%d)"
                                       % (d["t0"] / 60.0, d["t1"] / 60.0, d["n"]) for d in runts)))
        print("     ⭐ 时长不整除分桶时，最后一个桶常常只剩 1~2 个样本。拿它当「末桶」算趋势")
        print("        会给出完全荒谬的结论 —— 实测 CAN 10min 那轮末桶 n=1，于是报告显示")
        print("        p99 从 8.76『降到』2.78 ms (-68%)，纯属假象。趋势统计必须剔残桶。")
    p50s = [d["p50"] for d in solid]
    p99s = [d["p99"] for d in solid]
    print("  首桶→末桶(不计残桶): p50 %.2f → %.2f ms (%+.0f%%)   p99 %.2f → %.2f ms (%+.0f%%)"
          % (p50s[0], p50s[-1], (p50s[-1] / p50s[0] - 1) * 100 if p50s[0] else 0.0,
             p99s[0], p99s[-1], (p99s[-1] / p99s[0] - 1) * 100 if p99s[0] else 0.0))
    print("  全程 p50 波动范围: %.2f ~ %.2f ms (极差 %.2f ms)；p99 最大 %.2f ms"
          % (min(p50s), max(p50s), max(p50s) - min(p50s), max(p99s)))
    if p50s[-1] > p50s[0] * 1.3:
        print("  ⚠ 末桶 p50 比首桶高 30%+ ⇒ 有**热漂移/总线退化**的嫌疑，值得单独查")
    else:
        print("  ✓ 全程稳定（末桶 p50 未超首桶 1.3 倍）")


def _solid_buckets(bs, frac=0.5):
    """剔除「残桶」：样本数 < 中位数的一半。

    时长不整除分桶粒度时（600s ÷ 60s 恰好整除，但收到 601s 的样本
    就会挤出第 11 个桶，里面只有 1~2 个样本），末桶完全不能代表"稳态"。
    ⭐ 用残桶做首→末趋势会得出方向都反了的结论，所以必须剔除并显式告知。
    """
    if len(bs) < 3:
        return list(bs), []
    ns = sorted(d["n"] for d in bs)
    med = ns[len(ns) // 2]
    thr = max(1, int(med * frac))
    solid = [d for d in bs if d["n"] >= thr]
    runts = [d for d in bs if d["n"] < thr]
    return (solid, runts) if solid else (list(bs), [])


def print_bucket_summary(name, buckets):
    """把（已落盘 JSON 里的）分桶统计压成一行稳定性摘要，供 --report 合并时看。
    ⭐ 这是"10 分钟长跑到底稳不稳"的唯一判据：只看总表 p50 会把中途劣化平均掉。"""
    if not buckets or len(buckets) < 2:
        return
    solid, runts = _solid_buckets(buckets)
    p50s = [b["p50"] for b in solid]
    p99s = [b["p99"] for b in solid]
    worst = max(solid, key=lambda b: b["p99"])
    print("  %-34s 桶数=%2d%s  p50 %.2f→%.2f ms (极差 %.2f)   p99 最大 %.2f @ %.0f~%.0f min"
          % (name, len(solid), ("(+%d残)" % len(runts)) if runts else "",
             p50s[0], p50s[-1], max(p50s) - min(p50s),
             worst["p99"], worst["t0"] / 60.0, worst["t1"] / 60.0))
    if p50s[0] > 0 and p50s[-1] > p50s[0] * 1.3:
        print("       ⚠ 末桶 p50 比首桶高 30%+ ⇒ 有**热漂移 / 总线退化**的嫌疑，值得单独查")
    elif p50s[0] > 0:
        print("       ✓ 末桶 p50 未超首桶 1.3 倍 ⇒ 全程稳定")


def audit_pose(transport, plan, cross_tol=None):
    """运行中的位姿巡检：读回实际位姿，核对「还在 base±amp 里、别处没被拖走」。

    返回 (ok, 摘要文字, Δ列表)。读失败只算「警告」(ok=True)，因为
    单轴舵机偶发读失败是已知硬件现象，不该让 10 分钟的长跑白跑；
    但**姿态越界**（可能撞限位/被别处拖走）是硬错，必须中止。
    """
    cross_tol = PROBE_CROSS_TOL if cross_tol is None else cross_tol
    try:
        cur = read_pose_retry(transport, tries=2, gap=0.15, what="巡检位姿")
    except Exception as e:                                   # noqa: BLE001
        return True, "读失败(警告,不中止): %s" % e, []
    delta = [cur[j] - plan.base_servo[j] for j in range(6)]
    over_move, worst_cross = 0.0, 0.0
    for j in range(6):
        ax = j + 1
        if ax in plan.axes:
            allow = plan.amp_for(ax) + 8.0        # 跟随滞后余量
            over_move = max(over_move, abs(delta[j]) - allow)
        else:
            worst_cross = max(worst_cross, abs(delta[j]))
    if over_move > 0.0:
        return False, "摆动轴超出 base±amp %.1f°" % over_move, delta
    if worst_cross > cross_tol:
        return False, "非摆动轴被动 %.1f°（> %.1f°）" % (worst_cross, cross_tol), delta
    return True, "非摆动轴最大被动 %.1f°" % worst_cross, delta


def resolve_payload(transport, cmd_name, allow_move, quiet, offset, dirs):
    """为需要目标角的命令准备 payload（目标 = 当前角 ⇒ 原地不动）。

    ⚠ 量纲：MOVE_JOINTS 与 GET_STATUS 同为**舵机度** ⇒ 可直接回填；
            STREAM_JOINTS 要**逻辑角** ⇒ 必须先换算（2026-09-22 修正：原版直接
            回填，厂值下 J2~J6 会偏 180°，臂会甩一大下）。
    """
    req, exp, need_payload = CMD_SPEC[cmd_name]
    if not need_payload:
        return b""

    # 读当前角（用它自己的链路读，避免混用）
    try:
        cur = read_pose_retry(transport, what="当前角")
    except TimeoutError:
        raise RuntimeError("读当前角失败（超时），拒绝构造 %s 目标" % cmd_name)
    tgt = [round(v, 1) for v in cur]
    if cmd_name == "move_joints":
        print("  目标角 = 当前角（原地，舵机度）: %s" % _series(tgt))
        return ch.payload_move_joints(tgt, 0.0, 0.0)
    logic = [round(v, 1) for v in servo_to_logical(tgt, offset, dirs)]
    print("  目标角 = 当前角（原地）: 舵机度 %s" % _series(tgt))
    print("                           逻辑角 %s   ← STREAM_JOINTS 吃这套" % _series(logic))
    return ch.payload_stream_joints(logic)


def run_one(transport, cmd_name, n, warmup, gap, timeout, allow_move, quiet,
            motion=None, offset=None, dirs=None, duration=None,
            bucket=BUCKET_DEFAULT, limits=None):
    """motion: None = 目标恒为当前角（臂不动）；dict = 正弦摆动（臂一直在动）。
    duration: 秒。给了就按时长跑（n 只作安全上限），并在运行中**分桶统计 + 位姿巡检**。"""
    req, exp, _ = CMD_SPEC[cmd_name]
    res = BenchResult(transport.name)
    offset = _off6(offset) if offset else [SERVO_OFFSET_FACTORY] * 6
    dirs = tuple(dirs) if dirs else JOINT_DIR_FACTORY

    print("\n=== %s ===" % transport.name)
    try:
        transport.open()
    except Exception as e:                              # noqa: BLE001
        print("  ✗ 打开失败: %s" % e)
        res.first_err = "open: %s" % e
        res.elapsed = 0.0
        return res

    plan = None
    payload = None
    t_phase0 = None
    try:
        if quiet:
            try:
                transport.request_once(ch.CMD_LOG_SET_LEVEL, bytes([0]), 1.0)
                print("  已把固件日志压到 0（只报错）。恢复：控制台敲 `log` 或重发 LOG_SET_LEVEL")
            except Exception:                            # noqa: BLE001
                print("  （日志静音命令没成功，忽略）")

        if motion is not None:
            # ---- 运动模式：读起始位姿 -> 建立摆动 -> （流模式）回读自检 ----
            try:
                base = read_pose_retry(transport, what="起始位姿")
            except Exception as e:                       # noqa: BLE001
                print("  ✗ 读起始位姿失败: %s ⇒ 拒绝运动" % e)
                res.first_err = "read base: %s" % e
                res.elapsed = 0.0
                return res
            print("  起始位姿(舵机度): %s" % _series(base))
            plan = MotionPlan(base, cmd_name, motion["axes"], motion["amps"],
                              motion["period"], offset, dirs, limits=limits)
            print("  摆动: J%s ±%s°  周期 %.1fs  -> %s" % (
                "+".join(str(a) for a in plan.axes),
                "+".join("%.1f" % a for a in plan.amps), plan.period,
                "目标=舵机度" if cmd_name == "move_joints" else "目标=逻辑角(已换算)"))
            for ax, want, got in plan.clamped:
                print("  ⚠ J%d 幅度被**限位夹小**: 想要 ±%.1f° → 实际 ±%.1f°"
                      "（该轴限位 %g~%g，留 %g° 余量）"
                      % (ax, want, got, plan.limits[ax - 1][0], plan.limits[ax - 1][1],
                         SAFE_MARGIN_DEG))
            if all(a < 0.5 for a in plan.amps):
                print("  ✗ 所有摆动轴的可动余量都 < 0.5°（已经贴限位了）⇒ 拒绝运动")
                res.first_err = "no room to move"
                res.elapsed = 0.0
                return res
            print("  摆幅跨度: 单程约 %.1f°，一个周期走 %.1f°"
                  % (2 * sum(plan.amps), 4 * sum(plan.amps)))
            if cmd_name == "stream_joints" and motion.get("probe", True):
                ok, msg, _ = motion_probe_stream(transport, plan, quiet)
                if not ok:
                    print("  ✗ 流模式运动自检【不通过】: %s" % msg)
                    print("    ⇒ 已中止，一帧摆动都没发。先把上面那条修好再测。")
                    res.first_err = "stream motion probe: %s" % msg
                    res.elapsed = 0.0
                    return res
                print("  ✓ 流模式自检通过: %s" % msg)
                try:
                    restore_pose(transport, plan.base_servo, wait=3.0)
                except Exception:                        # noqa: BLE001
                    pass
        else:
            payload = resolve_payload(transport, cmd_name, allow_move, quiet, offset, dirs)

        t_phase0 = time.perf_counter()
        t_first = None      # 第一个【有效】样本的发出时刻（不含预热）
        t_last = None       # 最后一个【有效】样本的收到时刻
        ts = []             # 每个有效样本完成时刻（相对 t_phase0）—— 供分桶
        bucket_marks = []   # [(桶结束时刻, 累计有效样本数)]
        n_ext = 0

        if duration:
            deadline = t_phase0 + float(duration)
            print("  ⏱ 时长模式: 跑满 %.0f 秒（%.1f 分钟），每 %.0fs 分一桶统计；"
                  "位姿巡检也在桶边界做" % (duration, duration / 60.0, bucket))
        else:
            deadline = None
            total = warmup + n

        i = 0
        next_bucket = bucket
        next_progress = 30.0
        abort_msg = None
        while True:
            if deadline is None and i >= total:
                break
            now = time.perf_counter()
            if deadline is not None and now >= deadline:
                break
            warm = i < warmup
            # 运动模式：每帧新目标（正弦摆动）；否则恒为同一个 payload
            pl_now = payload if plan is None else plan.payload(now - t_phase0)
            t0 = time.perf_counter()
            try:
                rcmd, pl = transport.request_once(req, pl_now, timeout)
            except TimeoutError:
                if not warm:
                    res.n_fail += 1
                if res.first_err is None:
                    res.first_err = "timeout"
                if not warm and res.n_fail <= 3:
                    print("  #%-4d TIMEOUT" % i)
                i += 1
                continue
            dt = (time.perf_counter() - t0) * 1000.0
            if rcmd != exp:
                if not warm:
                    res.n_badrsp += 1
                # ⭐ NACK 必须解 err，否则只知道"被拒"却不知道"为什么被拒"
                if rcmd == 0x7E:
                    _orig, _err, _why = decode_nack(pl)
                    key = "err=%s" % (_why if _err is not None else _why)
                    res.err_hist[key] = res.err_hist.get(key, 0) + 1
                    reason = _why
                else:
                    reason = "响应 0x%02X" % rcmd
                if res.first_err is None:
                    res.first_err = "rsp=0x%02X (期望 0x%02X)" % (rcmd, exp)
                if not warm and res.n_badrsp <= 3:
                    print("  #%-4d %s" % (i, reason))
                i += 1
                continue
            i += 1
            if warm:
                continue
            if t_first is None:
                t_first = t0
            res.lats.append(dt)
            ts.append(time.perf_counter() - t_phase0)
            if gap > 0:
                time.sleep(gap)
            t_last = time.perf_counter()
            n_ext += 1

            el = t_last - t_phase0
            # ---- 分桶 + 运行中位姿巡检 ----
            if plan is not None and el >= next_bucket:
                bucket_marks.append((round(el, 3), n_ext))
                if motion.get("audit", True):
                    ok, msg, d = audit_pose(transport, plan)
                    tagb = "[桶 %d @%.0fs]" % (len(bucket_marks), el)
                    if ok:
                        print("  %s ✓ 巡检通过 (%s)" % (tagb, msg))
                    else:
                        print("  %s ⚠ 巡检异常: %s" % (tagb, msg))
                        print("       Δ = %s" % _series(d))
                        if "读失败" not in msg:
                            abort_msg = "pose audit: %s" % msg
                            break
                next_bucket += bucket
            if plan is None and el >= next_progress:
                print("  %.0fs  已采 %d 个  p50=%.2fms"
                      % (el, n_ext, percentile(sorted(res.lats), 50)))
                next_progress += 30.0
        if abort_msg:
            res.first_err = abort_msg
            print("  ✗ 因巡检异常中止: %s" % abort_msg)

        # 速率只统计【有效样本的时间跨度】，不含预热与连接/静音开销
        res.elapsed = (t_last - t_first) if (t_first is not None and t_last is not None) else 0.0
        res.n_badpkt = transport.bad
        res.bucket_stats = bucketize(res.lats, ts, bucket)
        if plan is not None and motion is not None and motion.get("restore", True):
            try:
                restore_pose(transport, plan.base_servo)
            except Exception as e:                       # noqa: BLE001
                print("  ⚠ 回位异常: %s（请目视确认臂的位置）" % e)
    finally:
        transport.close()

    s = res.summary()
    if s.get("n"):
        print("  ✓ ok=%d  超时=%d  坏响应=%d  min/p50/p90/p99/max = %.2f/%.2f/%.2f/%.2f/%.2f ms"
              % (s["n_ok"], s["fail_timeout"], s["fail_badrsp"],
                 s["min"], s["p50"], s["p90"], s["p99"], s["max"]))
    else:
        print("  ✗ 无有效样本（ok=0/%d, 超时=%d）%s"
              % (n, s["fail_timeout"], ("  首错: " + s["first_err"]) if s.get("first_err") else ""))
    if getattr(res, "bucket_stats", None) and len(res.bucket_stats) > 1:
        print_buckets(res)
    if plan is not None and res.n_ok > 1:
        _dt = res.elapsed / max(1, res.n_ok)
        _step = plan.max_step_deg(_dt)
        print("  相邻帧目标最大变化 ≈ %.2f°（实测帧间隔 %.1fms；固件单帧门限 %.0f°）"
              % (_step, _dt * 1000.0, STREAM_MAX_JUMP_HINT))
        if _step > STREAM_MAX_JUMP_HINT * 0.5:
            print("  ⚠ 已经用到门限的一半以上 ⇒ 目标是【跳变】而不是跟随；"
                  "把周期调长或摆幅调小更稳")
    if res.err_hist:
        tot = sum(res.err_hist.values())
        print("")
        print("  ⚠ %s 被拒原因直方图: %d 次 / 共 %d 次请求 = %.1f%% 没被受理"
              % (res.name, tot, max(1, res.n_total), 100.0 * tot / max(1, res.n_total)))
        for k, v in sorted(res.err_hist.items(), key=lambda kv: -kv[1]):
            print("      %-58s %6d 次" % (k, v))
        print("      ⇒ err=4(LIMIT) 且伴随大跨度摆动 ⇒ 多半是【单帧跳变 > %g°】被拒"
              % STREAM_MAX_JUMP_HINT)
        print("        （固件 STREAM_MAX_JUMP_DEG；帧被拒后 stream_prev 不更新会连锁放大）")
        print("      ⇒ err=2(CRC) ⇒ 是**传输层丢字节**，查线/终端电阻/干扰，不是姿态问题")
    return res


# ============================================================
# 报告
# ============================================================
def _fit(c, w):
    """把单元格塞进 w 列：超长就截断补 '~'，否则右补空格。
    ⚠ 不截断的话，像 motion-J1_J2_J3_J4_J5_J6-20_20_20_20_70_33deg-T12s
      这种长场景标签（49 字符）会把整行推出分隔线，表格彻底错位。"""
    s = str(c)
    if len(s) > w:
        s = s[:max(0, w - 1)] + "~"
    return s.ljust(w)


def fmt_row(cells, widths):
    return "  " + "".join(_fit(c, w) for c, w in zip(cells, widths))


def print_table(summaries, cmd_name=None):
    """summaries: list of dict（来自 BenchResult.summary()）。
    cmd_name: 本次命令名，用于给出「这组数能不能用来比链路」的判读警告。"""
    hdr = ["链路", "样本", "成功", "超时", "坏包", "min", "p50", "p90", "p99", "max", "mean", "std", "次/秒", "场景"]
    widths = [30, 8, 8, 8, 8, 8, 8, 8, 8, 9, 8, 8, 9, 26]
    rule = sum(widths) + 2
    print("")
    print("=" * rule)
    print("  链路响应时间对比（单位 ms，越小越快）")
    print("=" * rule)
    print(fmt_row(hdr, widths))
    print("  " + "-" * (rule - 2))
    for s in summaries:
        if s.get("p50") is None:
            print(fmt_row([s["name"], s.get("n_total", 0), 0, s.get("fail_timeout", 0),
                           s.get("bad_pkt", 0), "-", "-", "-", "-", "-", "-", "-",
                           "(无数据: %s)" % s.get("first_err", "?"),
                           s.get("scenario", "-")], widths))
            continue
        print(fmt_row([
            s["name"], s["n_total"], s["n_ok"], s["fail_timeout"], s["bad_pkt"],
            "%.2f" % s["min"], "%.2f" % s["p50"], "%.2f" % s["p90"],
            "%.2f" % s["p99"], "%.2f" % s["max"], "%.2f" % s["mean"],
            "%.2f" % s["std"], "%.0f" % s.get("rate", 0),
            s.get("scenario", "-"),
        ], widths))
    print("=" * rule)

    # ⭐ 场景标签（含摆幅/周期）常常有 40+ 字符，超出列宽 26 会被截成 "…~"：
    #    把完整值列在表下，否则事后根本看不出这一轮是什么摆幅/周期。
    scen_full = sorted({str(s.get("scenario", "-")) for s in summaries})
    trunc = [x for x in scen_full if len(x) > widths[-1]]
    if trunc:
        print("  场景标签完整值（表格列宽 %d，超出部分被截断）:" % widths[-1])
        for x in trunc:
            print("    %s" % x)

    scens = {s.get("scenario") for s in summaries if s.get("scenario")}
    # ⭐ 只有「运动 / 静止」这种【类别差异】才谈得上"不可直接比"；
    #    同属运动、只是摆幅/周期不同的多组（--report 合并多轮时很常见）
    #    是同一类负载，说成"混了静止 vs 运动"会把用户带偏（2026-09-22 修）。
    kinds = {("motion" if str(s.get("scenario", "")).startswith("motion") else "static")
             for s in summaries if s.get("scenario")}
    if len(kinds) > 1 and len(scens) > 1:
        print("  ⚠ 本表混了【不同场景】(静止 vs 运动)：%s" % ", ".join(sorted(scens)))
        print("    两者不可直接比 —— 场景不同，负载不同。")
        print("    ⭐ 实测（2026-09-22, J5 ±5°/T2s, 各 150 样本）：")
        print("       stream_joints 运动 vs 静止 = 网口 1.92 vs 1.75 ms (+10%)、串口 4.78 vs 4.78 ms (0%)")
        print("       ⇒ 流模式的 ACK 与插补解耦，臂在动几乎不加时 ⇒ 20~50Hz 遥操作可行。")
        print("       ⇒ 真正吃运动开销的是 move_joints（要等轨迹受理+上力矩, ~91ms）。")
        print("    想看单一场景：--report --cmd <cmd> --report-scenario <关键字>")
    elif len(scens) > 1:
        print("  ℹ 本表含 %d 组【同一类】场景（都是运动，只是摆幅/周期不同）：%s"
              % (len(scens), ", ".join(sorted(scens))))
        print("    同属运动 ⇒ 可以横向比链路；但它们不是同一次运行，")
        print("    要严格比链路请用同一轮（同摆幅/周期）的两条链路。")

    live = [s for s in summaries if s.get("p50") is not None]
    if len(live) >= 2:
        fast = min(live, key=lambda s: s["p50"])
        slow = max(live, key=lambda s: s["p50"])
        print("  ⚡ 最快: %s  (p50 %.2f ms)" % (fast["name"], fast["p50"]))
        if slow["p50"] > 0 and slow is not fast:
            print("  🐢 最慢: %s  (p50 %.2f ms, 是快者的 %.1f 倍)"
                  % (slow["name"], slow["p50"], slow["p50"] / fast["p50"]))
        print("  判读提示:")
        print("    * p50 看「常态」, p99/max 看「最坏情况」 —— 实时控制要盯 p99 和 max, 不是 avg。")
        print("    * 单路测到的差值只来自链路 + 承载协议(含 CAN 的分包+乱序重组)。")
        print("    * 若某链路 超时>0, 先确认固件是否匹配: 网口要 LINK=net, CAN 要 LINK=can")
        print("      (CAN2=PB12/PB13 与 RMII 的 ETH_RXD1/TXD1 是同一对脚 ⇒ 两者互斥)。")
    elif len(live) == 1:
        print("  (只有一个链路有数据; 另一条多半是固件不匹配 —— 网口需 LINK=net, CAN 需 LINK=can)")

    # ⭐ 命令分类判读（本工具最容易踩的坑：把「读硬件耗时」当成「链路慢」）
    if cmd_name in HW_TOUCHING:
        print("")
        print("⚠⚠ 判读警告: 本次 --cmd %s 属【读硬件】类。" % cmd_name)
        print("     它的 RTT 里约有 %.1f ms 是固件【读六轴舵机实际位置】的耗时 ——" % HW_COST_MS)
        print("     这笔钱三条链路都要付，与链路无关 ⇒ 拿它比「哪条链路快」会被压平：")
        _lv = [s for s in summaries if s.get("p50") is not None]
        if len(_lv) >= 2:
            _f = min(_lv, key=lambda s: s["p50"])
            _s = max(_lv, key=lambda s: s["p50"])
            print("       本次实测 %s %.2fms vs %s %.2fms ⇒ 倍数 %.1f"
                  "（每条都含 ~%.1fms 与链路无关的固定开销 ⇒ 倍数被压缩）"
                  % (_f["name"], _f["p50"], _s["name"], _s["p50"],
                     _s["p50"] / _f["p50"] if _f["p50"] else 0.0, HW_COST_MS))
        print("     ⇒ 要比【链路本身】：--cmd ping（纯链路, 不读硬件）")
        print("     ⇒ 要看【真实控制响应】：就是这组数，继续用。")
    elif cmd_name in ("ping", "param_count"):
        print("")
        print("  ✓ 本次 --cmd %s 属【纯链路】类（不读硬件）—— 这组数才反映链路本身。" % cmd_name)
        print("    真实下发控制命令时还要再加约 %.1f ms 的固件读硬件时间。" % HW_COST_MS)
    elif cmd_name in NEEDS_MOVE:
        print("")
        print("  ⚠ 本次 --cmd %s 走【控制通路】—— 固件要 读起点 + 规划 S 曲线 + 上力矩。" % cmd_name)
        _mv = [s for s in summaries if s.get("p50") is not None]
        if _mv:
            print("     本次实测: " + "  |  ".join(
                "%s p50=%.1f p99=%.1f ms" % (s["name"], s["p50"], s.get("p99") or 0.0)
                for s in _mv))
        print("     ⚠ 绝对值随【臂姿态】变化（2026-09-22 实测 91 → 157 ms），**别跨轮比绝对值**；")
        print("       同轮内链路之间几乎相等 ⇒ 『链路对 move_joints 无影响』成立。")
        print("     ⚠ 别把 p99/max 当成 p50 记 —— 本工具早期笔记一度把 153 ms 误记成 p50。")
        print("     ⇒ 这个数 = 「下发→确认」延迟，**不是**「臂动作完成」的时间。")
        print("     ⇒ ⭐ 遥操作(20~50Hz) 绝不能用 move_joints，必须用 stream_joints。")


def save_json(out_dir, tag, cmd_name, summaries, samples=None, scenario=None):
    os.makedirs(out_dir, exist_ok=True)
    slug = "".join(c if (c.isalnum() or c in "-_") else "_" for c in (scenario or "static"))
    path = os.path.join(out_dir, "%s_%s_%s.json" % (cmd_name, slug, tag))
    doc = {"tag": tag, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "cmd": cmd_name,
           "scenario": scenario or "static", "results": summaries}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    print("\n  结果已存: %s" % path)

    if samples:
        csv_path = os.path.join(out_dir, "samples_%s_%s.csv" % (slug, tag))
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("link,index,rtt_ms\n")
            for name, lats in samples.items():
                for i, v in enumerate(lats):
                    f.write("%s,%d,%.4f\n" % (name, i, v))
        print("  原始样本 CSV: %s" % csv_path)


def do_report(out_dir, cmd_name, scenario=None):
    """合并同一 `--cmd` 的历史结果。scenario 为子串过滤（如 motion / static）。"""
    files = sorted(glob.glob(os.path.join(out_dir, "%s_*.json" % cmd_name)),
                   key=lambda p: os.path.getmtime(p))
    if not files:
        print("✗ %s 里没有 %s_*.json（先跑一轮测试）" % (out_dir, cmd_name))
        return 2
    merged = {}
    src = {}
    skipped = 0
    for p in files:
        with open(p, encoding="utf-8") as f:
            doc = json.load(f)
        scen = doc.get("scenario", "static")
        if scenario and scenario.lower() not in scen.lower():
            skipped += 1
            continue
        for s in doc.get("results", []):
            s = dict(s)
            s.setdefault("scenario", scen)
            # ⚠ 必须按 (链路, 场景) 做 key：只按链路名会让"后跑的那一轮"覆盖掉
            #   另一个场景的行 ⇒ 静止轮 / 运动轮互相覆盖（2026-09-22 修）。
            key = (s["name"], s.get("scenario", "static"))
            merged[key] = s                # 同 (链路, 场景) ⇒ 后（较新）者覆盖
            src[key] = os.path.basename(p)
    print("  合并 %d 个结果文件, 命令 = %s%s"
          % (len(files) - skipped, cmd_name,
             ("  (按场景过滤 %r, 跳过 %d 个)" % (scenario, skipped)) if scenario else ""))
    for k, v in src.items():
        print("    %s <- %s" % (k, v))
    if not merged:
        print("✗ 过滤后没有可合并的结果（换一个 --report-scenario 关键字）")
        return 2
    print_table(list(merged.values()), cmd_name=cmd_name)

    # ⭐ 长时间跑过的结果，顺手把分桶稳定性也打出来：
    #   总表 p50 会把中途劣化平均掉，只有分桶才看得出"越跑越差"。
    have_b = [s for s in merged.values()
              if s.get("buckets") and len(s["buckets"]) >= 2]
    if have_b:
        print("")
        print("⏱ 长时间稳定性（原始分桶粒度 %g 秒；只有 --duration 长跑才有）" % BUCKET_DEFAULT)
        for s in have_b:
            print_bucket_summary("%s [%s]" % (s["name"], s.get("scenario", "-")), s["buckets"])
    return 0


# ============================================================
# 离线自检（不需要板子 / CAN 卡）
# ============================================================
class _FakeSerialPort:
    """假串口：把"发出去的帧"变成"固件会回的字节流"，前后还夹一段文本日志，
    专门用来验证 parse_stream 能跳过文本、并正确处理不完整帧。"""
    def __init__(self, arm):
        self.arm = arm
        self.rx = bytearray()       # 待被上位机读走
        self.tx = bytearray()

    def reset_input_buffer(self):
        self.rx.clear()

    def write(self, b):
        self.tx += b
        # 解析出请求（跳过帧头），生成响应
        drop, frame = parse_stream(bytearray(self.tx))
        if frame:
            del self.tx[:drop]
            seq, cmd, pl = frame
            rcmd, rpl = self.arm(seq, cmd, pl)
            # 模拟真实控制台：响应前后夹文本
            self.rx += b"\r\n  MoveJoints tgt=180 ...\r\n"
            self.rx += build_serial_frame(seq, rcmd, rpl)
            self.rx += b"  \xe6\x9f\x90\xe6\x8f\xa1\xe6\x97\xa5\xe5\xbf\x97\r\n"

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def close(self):
        pass


def _fake_arm_resp(seq, cmd, pl):
    """与 _FakeArmBus._resp 同语义的串口侧假臂。"""
    if cmd == ch.CMD_LOG_SET_LEVEL:
        return ch.RSP_ACK, bytes([cmd, 0])
    if cmd == ch.CMD_PING:
        return 0x81, bytes([0x01, 0x2A])
    if cmd == ch.CMD_GET_STATUS:
        joints = [150.0, 120.0, 150.0, 200.0, 200.0, 150.0]
        p = b"".join(ch.i16(round(v * 10)) for v in joints)
        return 0x82, p + ch.i16(1800) + ch.i16(0) + ch.i16(2500) + ch.i16(900) + bytes([0x08])
    if cmd == ch.CMD_FK_QUERY:
        joints = [150.0, 120.0, 150.0, 200.0, 200.0, 150.0]
        p = b"".join(ch.i16(round(v * 10)) for v in joints)
        return 0xA1, ch.i16(1800) + ch.i16(0) + ch.i16(2500) + ch.i16(900) + p + bytes([1])
    if cmd == ch.CMD_PARAM_COUNT:
        return 0xC2, bytes([24])
    if cmd in (ch.CMD_MOVE_JOINTS, ch.CMD_STREAM_JOINTS):
        return ch.RSP_ACK, bytes([cmd, 0x00])
    return ch.RSP_NACK, bytes([cmd, 0x01])


def selftest():
    ok, fail = [0], [0]

    def chk(name, cond, extra=""):
        if cond:
            ok[0] += 1
            print("  PASS  %s" % name)
        else:
            fail[0] += 1
            print("  FAIL  %s %s" % (name, extra))

    print("离线自检：验证统计层 + 三条链路的编解码与端到端路径（假总线 / 假串口带文本噪声 / 假 UDP）。")
    print("⚠ 假总线没有物理传输时间 ⇒ 下表里的绝对毫秒数【没有物理意义】。")
    print("  自检只看 PASS/FAIL（路径通不通、解析对不对）；真实延迟必须接实物测。\n")

    print("=== 1. 统计函数 ===")
    s = summarize([1.0, 2.0, 3.0, 4.0, 5.0])
    chk("min/max/mean", s["min"] == 1.0 and s["max"] == 5.0 and s["mean"] == 3.0, str(s))
    chk("p50 插值=3.0", abs(s["p50"] - 3.0) < 1e-9, str(s["p50"]))
    chk("p90 插值=4.6", abs(s["p90"] - 4.6) < 1e-9, str(s["p90"]))
    chk("空样本 -> None", summarize([]) is None)
    chk("单样本 p99 不炸", summarize([7.0])["p99"] == 7.0)

    print("=== 2. 串口帧 编解码 ===")
    f = build_serial_frame(0x25, ch.CMD_GET_STATUS, b"\x01\x02")
    chk("帧头 AA 55", f[0] == 0xAA and f[1] == 0x55, f.hex())
    chk("帧长 = 5+LEN+2", len(f) == 5 + 2 + 2, str(len(f)))
    # 前面塞文本噪声 + 一个假同步头 + 一个 CRC 错帧，验证能跳过
    noise = b"  log line\r\n\xaa\x55\x00\x00\x00\x00\x00\r\n" + b"\xaa\x55\x99\x02\x00\x00\x00"
    drop, fr = parse_stream(bytearray(noise + f))
    chk("跳过文本/假头/坏帧 找到真帧",
        fr is not None and fr[0] == 0x25 and fr[1] == ch.CMD_GET_STATUS and fr[2] == b"\x01\x02",
        str(fr))
    chk("丢弃长度正确不可越界", drop == len(noise) + len(f), "drop=%d" % drop)
    drop2, fr2 = parse_stream(bytearray(f[:4]))
    chk("半帧返回 None 且不丢弃已确认前缀", fr2 is None and drop2 <= 1, "drop=%d" % drop2)
    # 与网口/语义包的关系：去掉帧头后应逐字节等于网口报文
    net = NetTransport.build_packet(0x25, ch.CMD_GET_STATUS, b"\x01\x02")
    chk("串口帧去掉 AA55 == 网口报文", f[2:] == net, "%s vs %s" % (f[2:].hex(), net.hex()))

    print("=== 3. 网口报文 编解码 ===")
    seq, cmd, pl = NetTransport.parse_packet(net)
    chk("parse 往返", seq == 0x25 and cmd == ch.CMD_GET_STATUS and pl == b"\x01\x02")
    bad = bytearray(net); bad[-1] ^= 0xFF
    try:
        NetTransport.parse_packet(bytes(bad)); chk("CRC 错必须抛 ValueError", False)
    except ValueError:
        chk("CRC 错必须抛 ValueError", True)

    print("=== 4. CAN 分包/重组（复用 can_host） ===")
    frames, first_n = ch.plan_frames(21)
    chk("GET_STATUS 21B 负载 -> 多帧", frames >= 2, "frames=%d" % frames)
    rr = ch.Reassembler(1)
    got = None
    for i, fr_data in enumerate(ch.build_frames(0x11, 0x82, bytes(range(21)))):
        out = rr.feed(ch.ID_RSP_BASE + 1, fr_data)
        if out: got = out
    chk("重组还原 21B", got is not None and got[0] == 0x11 and got[3] and len(got[2]) == 21, str(got))
    # 乱序（F1,F4,F2,F3 —— 实测硬件顺序）也必须能还原
    rr2 = ch.Reassembler(1)
    fs = ch.build_frames(0x12, 0x82, bytes(range(21)))
    order = [0] + list(range(1, len(fs)))[::-1] if len(fs) >= 4 else list(range(len(fs)))
    got2 = None
    for i in order:
        out = rr2.feed(ch.ID_RSP_BASE + 1, fs[i])
        if out: got2 = out
    chk("乱序重组也还原", got2 is not None and got2[0] == 0x12 and got2[3], str(order))

    print("=== 5. 串口链路 端到端（假串口 + 文本噪声） ===")
    t = SerialTransport("FAKE", 115200, name="串口(假)")
    t.open = lambda: None
    t.ser = _FakeSerialPort(_fake_arm_resp)
    r = run_one(t, "status", n=30, warmup=5, gap=0, timeout=0.3, allow_move=False, quiet=True)
    chk("串口 30/30 成功", r.n_ok == 30 and r.n_fail == 0, "ok=%d fail=%d" % (r.n_ok, r.n_fail))

    print("=== 6. 网口链路 端到端（本地假臂 UDP） ===")
    import threading
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.settimeout(0.2)
    srv.bind(("127.0.0.1", 5977))
    stop = threading.Event()

    def fake_udp():
        while not stop.is_set():
            try:
                data, addr = srv.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                sq, cm, pld = NetTransport.parse_packet(data)
            except ValueError:
                continue
            rc, rp = _fake_arm_resp(sq, cm, pld)
            srv.sendto(NetTransport.build_packet(sq, rc, rp), addr)

    th = threading.Thread(target=fake_udp, daemon=True)
    th.start()
    t2 = NetTransport("127.0.0.1", 5977, name="网口(假)")
    r2 = run_one(t2, "status", n=30, warmup=5, gap=0, timeout=0.5, allow_move=False, quiet=False)
    chk("网口 30/30 成功", r2.n_ok == 30 and r2.n_fail == 0, "ok=%d fail=%d" % (r2.n_ok, r2.n_fail))
    stop.set(); srv.close()

    print("=== 7. CAN 链路 端到端（_FakeArmBus） ===")
    bus = ch._FakeArmBus(node=1)
    t3 = CanTransport("fake", "fake", 500000, node=1, name="CAN(假)")
    t3.open = lambda: None
    t3.bus = bus
    t3.reasm = ch.Reassembler(1)
    r3 = run_one(t3, "status", n=30, warmup=5, gap=0, timeout=0.3, allow_move=False, quiet=False)
    chk("CAN 30/30 成功", r3.n_ok == 30 and r3.n_fail == 0, "ok=%d fail=%d" % (r3.n_ok, r3.n_fail))

    print("=== 8. 角度量纲换算 与 运动规划（★本轮重点） ===")
    # 指令大全 §9：两套量纲。
    srv = [180.0, 200.0, 40.0, 90.0, 270.0, 0.0]
    lg = servo_to_logical(srv)
    chk("servo->logical: J1 反向(180->0)", abs(lg[0]) < 1e-6, "%.4f" % lg[0])
    chk("servo->logical: J5 270 -> +90", abs(lg[4] - 90.0) < 1e-6, "%.4f" % lg[4])
    chk("servo->logical: 跨 0/360 归一 (-180,180]", abs(lg[5]) == 180.0, "%.4f" % lg[5])
    back = logical_to_servo(lg)
    chk("logical->servo 往返一致",
        all(abs(back[i] - srv[i]) < 1e-6 for i in range(6)),
        "%s vs %s" % ([round(v, 1) for v in back], srv))
    # ★ 关键回归：旧版把 GET_STATUS(舵机度) 直接当逻辑角喂 STREAM -> 偏一整个 SERVO_OFFSET
    cur = [180.0, 200.0, 200.0, 200.0, 200.0, 200.0]
    naive = logical_to_servo(cur)                      # 旧版做法
    fixed = logical_to_servo(servo_to_logical(cur))    # 修正后
    d_naive = max(abs(naive[j] - cur[j]) for j in range(6))
    d_fixed = max(abs(fixed[j] - cur[j]) for j in range(6))
    chk("旧版直接把舵机度喂 STREAM 会偏 ~180°", d_naive > 90.0, "max偏差 %.1f°" % d_naive)
    chk("修正后确实「原地不动」", d_fixed < 1e-6, "max偏差 %.4f°" % d_fixed)

    # 摆动：有界 / 连续 / 只动一轴 / payload 长度对
    base = [30.0, 120.0, 75.0, 190.0, 250.0, 310.0]
    AMP = 7.0
    for cmd_name in ("move_joints", "stream_joints"):
        pln = MotionPlan(base, cmd_name, 5, AMP, 2.0,
                         [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
        mx, oth = 0.0, 0.0
        raw = set()
        for k in range(400):
            t = 4.0 * k / 399.0                        # 覆盖两个整周期
            st = pln.servo_target(t)
            mx = max(mx, abs(st[4] - base[4]))
            oth = max(oth, max(abs(st[j] - base[j]) for j in (0, 1, 2, 3, 5)))
            raw.add(pln.payload(t))
        chk("%s 摆动有界(<=amp)" % cmd_name, mx <= AMP + 1e-9, "max=%.3f" % mx)
        chk("%s 摆动确实在动(>=0.9*amp)" % cmd_name, mx >= AMP * 0.9, "max=%.3f" % mx)
        chk("%s 只动摆动轴, 其它轴恒定" % cmd_name, oth < 1e-9, "others=%.3f" % oth)
        chk("%s 每帧 payload 都在变" % cmd_name, len(raw) > 100, "不同 payload=%d" % len(raw))
        chk("%s payload 长度" % cmd_name,
            len(pln.payload(0.3)) == (16 if cmd_name == "move_joints" else 12),
            str(len(pln.payload(0.3))))
    chk("摆动幅度硬上限 >= 60°（大跨度要够用）", MOTION_AMP_HARD_MAX >= 60.0,
        "hard max=%g" % MOTION_AMP_HARD_MAX)
    chk("提示阈是安全的小幅值", MOTION_MAX_AMP >= MOTION_AMP_DEFAULT,
        "warn=%g" % MOTION_MAX_AMP)

    # 大跨度摆幅会被**真实限位**自动夹小（撞不到限位）
    pln_c = MotionPlan([200.0, 200.0, 200.0, 200.0, 20.0, 200.0], "stream_joints",
                       [5], [90.0], 6.0, [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
    chk("大跨度摆幅被限位夹小(J5 base=20, 限位 12~351)",
        0 < pln_c.amps[0] <= 90.0 and pln_c.clamped != [],
        "amp=%g clamped=%s" % (pln_c.amps[0], pln_c.clamped))
    chk("夹小后仍不会越限",
        pln_c.servo_target(0) and all(
            pln_c.limits[4][0] <= v <= pln_c.limits[4][1]
            for v in (pln_c.servo_target(0)[4], pln_c.servo_target(pln_c.period / 4)[4])),
        "端点 %.1f / %.1f" % (pln_c.servo_target(0)[4],
                            pln_c.servo_target(pln_c.period / 4)[4]))

    # 多轴：各轴都有界、互不同相位、tag 带全部轴（base 取中位，两轴都有限位余量）
    base2 = [200.0] * 6
    pln_m = MotionPlan(base2, "stream_joints", [5, 6], [20.0, 10.0], 4.0,
                       [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
    mx5 = mx6 = 0.0
    for k in range(400):
        # 扫两个周期：第一个周期被启动包络压着，第二个周期才是满幅
        st = pln_m.servo_target(8.0 * k / 399.0)
        mx5 = max(mx5, abs(st[4] - base2[4]))
        mx6 = max(mx6, abs(st[5] - base2[5]))
    chk("多轴: J5/J6 都动且各自有界", mx5 <= 20.0 + 1e-9 and mx6 <= 10.0 + 1e-9
        and mx5 > 18.0 and mx6 > 9.0, "mx5=%.2f mx6=%.2f" % (mx5, mx6))
    chk("多轴: tag 记录全部轴与幅度",
        pln_m.tag() == "motion-J5+J6-20+10deg-T4s", pln_m.tag())
    try:
        MotionPlan(base2, "stream_joints", [5, 6], [20.0], 4.0,
                   [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
        chk("多轴: 幅度个数不符报错", False, "竟然没报错")
    except SystemExit:
        chk("多轴: 幅度个数不符报错", True)

    # ⭐ 起步不跳变：t=0 时所有轴必须严格 = base（否则固件会以 err=4 拒帧）
    _p0 = pln_m.servo_target(0.0)
    chk("起步包络: t=0 时所有轴 = base（不吃跳变门限）",
        max(abs(_p0[j] - base2[j]) for j in range(6)) < 1e-9,
        "max偏差=%.4f" % max(abs(_p0[j] - base2[j]) for j in range(6)))
    # 相邻帧变化量必须远低于固件门限（按 350Hz 的最坏帧间隔 3ms 估）
    chk("单帧变化量远低于固件门限(%g°)" % STREAM_MAX_JUMP_HINT,
        pln_m.max_step_deg(0.003) < STREAM_MAX_JUMP_HINT * 0.2,
        "step=%.3f° @3ms" % pln_m.max_step_deg(0.003))
    # 包络在第一个周期内是单调升的，且满幅后不再变
    _e = [pln_m._env(t) for t in (0.0, 1.0, 2.0, 3.9, 4.0, 9.0)]
    chk("启动包络: 0→1 单调、满周期后恒为 1",
        _e[0] == 0.0 and _e[-1] == 1.0 and all(_e[i] <= _e[i + 1] + 1e-9 for i in range(len(_e) - 1)),
        str([round(x, 3) for x in _e]))

    # 分桶统计
    _b = bucketize([1.0, 2.0, 3.0, 4.0, 5.0], [0.1, 0.5, 1.5, 2.5, 2.6], 1.0)
    chk("分桶: 3 个桶、样本数守恒", len(_b) == 3 and sum(x["n"] for x in _b) == 5,
        "buckets=%d n=%s" % (len(_b), [x["n"] for x in _b]))

    print("=== 9. 运动模式端到端（假串口：臂不动，验证路径与守卫） ===")
    tm = SerialTransport("FAKE", 115200, name="串口(假,运动)")
    tm.open = lambda: None
    tm.ser = _FakeSerialPort(_fake_arm_resp)
    mspec = {"axes": [5], "amps": [5.0], "period": 2.0, "restore": True,
             "probe": True, "audit": True}
    rm = run_one(tm, "move_joints", n=20, warmup=2, gap=0, timeout=0.3,
                 allow_move=True, quiet=True, motion=mspec)
    chk("move_joints+运动 端到端 20/20", rm.n_ok == 20 and rm.n_fail == 0,
        "ok=%d fail=%d err=%s" % (rm.n_ok, rm.n_fail, rm.first_err))

    # ⭐ 时长模式：按时长结束 + 分桶 + 运行中巡检（假臂姿态不动 ⇒ 巡检应通过）
    tm2 = SerialTransport("FAKE", 115200, name="串口(假,时长)")
    tm2.open = lambda: None
    tm2.ser = _FakeSerialPort(_fake_arm_resp)
    rd = run_one(tm2, "move_joints", n=10 ** 7, warmup=2, gap=0, timeout=0.3,
                 allow_move=True, quiet=True, motion=mspec,
                 duration=0.6, bucket=0.2)
    chk("时长模式: 按时长结束(不是按 n)", rd.elapsed >= 0.5 and rd.n_ok > 5,
        "elapsed=%.2fs ok=%d" % (rd.elapsed, rd.n_ok))
    chk("时长模式: 落了分桶统计", len(rd.bucket_stats or []) >= 2,
        "buckets=%d" % len(rd.bucket_stats or []))
    chk("时长模式: 分桶样本数守恒",
        sum(b["n"] for b in rd.bucket_stats) == rd.n_ok,
        "sum=%d ok=%d" % (sum(b["n"] for b in rd.bucket_stats), rd.n_ok))
    chk("时长模式: summary 里带 buckets（供落盘）",
        "buckets" in rd.summary() and len(rd.summary()["buckets"]) >= 2,
        "keys=%s" % ("buckets" in rd.summary()))

    # 位姿巡检：越界必须判为不通过（base 用假臂**实际回读**的位姿 ⇒ 界内应通过）
    _base = read_pose_servo(tm2)
    _pl = MotionPlan(_base, "stream_joints", [5], [5.0], 2.0,
                     [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
    _ok, _msg, _d = audit_pose(tm2, _pl)
    chk("巡检: 位姿在界内 ⇒ 通过", _ok, _msg)
    _pl2 = MotionPlan(_base, "stream_joints", [5], [5.0], 2.0,
                      [SERVO_OFFSET_FACTORY] * 6, JOINT_DIR_FACTORY)
    _pl2.base_servo[0] -= 999.0                    # 人为把 base 挪走 ⇒ 非摆动轴一定"被动"
    _ok2, _msg2, _ = audit_pose(tm2, _pl2)
    chk("巡检: 姿态越界 ⇒ 不通过(会中止)", not _ok2, _msg2)

    ts = SerialTransport("FAKE", 115200, name="串口(假,运动)")
    ts.open = lambda: None
    ts.ser = _FakeSerialPort(_fake_arm_resp)
    rs = run_one(ts, "stream_joints", n=20, warmup=2, gap=0, timeout=0.3,
                 allow_move=True, quiet=True, motion=mspec)
    chk("流模式: 假臂不会动 ⇒ 回读自检必须拦住(0 样本)",
        rs.n_ok == 0 and str(rs.first_err).startswith("stream motion probe"),
        "ok=%d err=%s" % (rs.n_ok, rs.first_err))

    print("=== 10. 对比表渲染 ===")
    try:
        print_table([r.summary(), r2.summary(), r3.summary()])
        chk("对比表可渲染", True)
    except Exception as e:                              # noqa: BLE001
        chk("对比表可渲染", False, repr(e))

    print("=== 11. 结果持久化 ===")
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        save_json(td, "selftest", "status", [r.summary(), r2.summary(), r3.summary()],
                  samples={r.name: r.lats})
        got_p = glob.glob(os.path.join(td, "status_*.json"))
        csv_p = glob.glob(os.path.join(td, "samples_*.csv"))
        chk("JSON 落盘", len(got_p) == 1)
        chk("CSV 落盘", len(csv_p) == 1)
        rc = do_report(td, "status")
        chk("--report 能读回并合并", rc == 0)

    print("\n自检: %d PASS, %d FAIL" % (ok[0], fail[0]))
    return 0 if fail[0] == 0 else 1


# ============================================================
# main
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        description="串口/网口/CAN 三链路响应时间基准测试",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="serial,net,can",
                    help="逗号分隔: serial,net,can （默认全测；net 与 can 硬件互斥，见文档）")
    ap.add_argument("--cmd", default="status",
                    help="测哪条命令；逗号分隔可测多个 (默认 status = 只读, 臂不动)。"
                         "可选: %s。"
                         "⭐ 比「链路本身」用 ping；比「端到端控制响应」用 status/move"
                         % ",".join(sorted(CMD_SPEC)))
    ap.add_argument("--n", type=int, default=200, help="每链路有效样本数 (默认 200)")
    ap.add_argument("--warmup", type=int, default=10, help="预热样本数, 不计入统计 (默认 10)")
    ap.add_argument("--gap", type=float, default=0.0, help="样本间隔秒 (默认 0 = 连打)")
    ap.add_argument("--timeout", type=float, default=0.3, help="单次等待响应超时秒 (默认 0.3)")
    ap.add_argument("--allow-move", action="store_true", dest="allow_move",
                    help="允许 move_joints/stream_joints（默认目标=当前角，原地不动）")
    ap.add_argument("--quiet-log", action="store_true", dest="quiet_log",
                    help="开测前发 LOG_SET_LEVEL=0 静音固件日志")

    g = ap.add_argument_group("运动模式（--motion：让臂一直在动的同时测 RTT）")
    g.add_argument("--motion", action="store_true",
                   help="开启后目标按正弦摆动（默认关 = 臂不动）")
    g.add_argument("--motion-axis", default=str(MOTION_AXIS_DEFAULT), dest="motion_axis",
                   help="摆动轴，可用逗号多轴（默认 %d = J5，只改夹爪朝向、不动末端 XYZ）。"
                        "例: 5,6 = J5 与夹爪一起动" % MOTION_AXIS_DEFAULT)
    g.add_argument("--motion-amp", default=str(MOTION_AMP_DEFAULT), dest="motion_amp",
                   help="摆动幅度 ±度，可为每个轴各给一个（默认 %g，硬上限 %g；"
                        "还会被【真实限位】再夹一次，撞不到限位）"
                        % (MOTION_AMP_DEFAULT, MOTION_AMP_HARD_MAX))
    g.add_argument("--motion-period", type=float, default=MOTION_PERIOD_DEFAULT, dest="motion_period",
                   help="摆动周期 秒（默认 %g）。大跨度建议 6~10s（峰值角速度别太高）"
                        % MOTION_PERIOD_DEFAULT)
    g.add_argument("--duration", type=float, default=None, dest="duration",
                   help="⏱ 长时间模式：跑满这么多**秒**（推荐 %g = 10 分钟）。给了它就按时长结束，"
                        "--n 只当安全上限；并自动开启**分桶统计**与**运行中位姿巡检**"
                        % DURATION_DEFAULT)
    g.add_argument("--bucket", type=float, default=BUCKET_DEFAULT, dest="bucket",
                   help="长时间模式的分桶粒度秒数（默认 %g）—— 用来看「有没有越跑越差」"
                        % BUCKET_DEFAULT)
    g.add_argument("--joint-limits", default=None, dest="joint_limits",
                   help="各关节真实限位，逗号分 6 个 lo-hi（默认用 2026-09-22 `L` 实测值）。"
                        "摆幅会被自动夹到区间内侧 %g°" % SAFE_MARGIN_DEG)
    g.add_argument("--no-motion-restore", action="store_false", dest="motion_restore",
                   default=True, help="测完不送回起始位姿（默认会送）")
    g.add_argument("--no-motion-probe", action="store_false", dest="motion_probe",
                   default=True, help="跳过流模式运动前的回读自检（⚠ 不推荐）")
    g.add_argument("--no-audit", action="store_false", dest="audit",
                   default=True, help="长时间模式里不做运行中位姿巡检（⚠ 不推荐）")
    g.add_argument("--stream-offset", default=None, dest="stream_offset",
                   help="SERVO_OFFSET，逗号分隔 6 个（默认 180×6 = 出厂中位）。"
                        "Home 校零改过就必须传真实值，否则流模式目标整体偏")
    g.add_argument("--stream-dir", default=None, dest="stream_dir",
                   help="JOINT_DIR，逗号分隔 6 个 ±1（默认 -1,1,1,1,1,1）")

    ap.add_argument("--serial-port", default="COM11", dest="serial_port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp-ip", default="192.168.7.120", dest="udp_ip")
    ap.add_argument("--udp-port", type=int, default=5000, dest="udp_port")
    ap.add_argument("--can-interface", default="pcan", dest="can_interface",
                    help="python-can 后端: pcan|usbcan|socketcan|virtual (默认 pcan, Windows)")
    ap.add_argument("--can-channel", default="PCAN_USBBUS1", dest="can_channel")
    ap.add_argument("--can-bitrate", type=int, default=500000, dest="can_bitrate")
    ap.add_argument("--can-node", type=int, default=1, dest="can_node")

    ap.add_argument("--out", default="outputs/latency_bench", help="结果 JSON 目录")
    ap.add_argument("--csv", action="store_true", help="额外导出原始样本 CSV")
    ap.add_argument("--report", action="store_true", help="只读历史结果并出对比表")
    ap.add_argument("--report-scenario", default=None, dest="report_scenario",
                    help="--report 时按场景子串过滤（如 motion / static）")
    ap.add_argument("--selftest", action="store_true", help="离线自检(不需要板子)")
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())

    # ⭐ --cmd 支持逗号分隔多命令：一次跑遍，方便看「命令处理耗时」vs「链路耗时」
    cmd_list = [x.strip() for x in a.cmd.split(",") if x.strip()]
    bad = [c for c in cmd_list if c not in CMD_SPEC]
    if bad:
        print("✗ 未知命令: %s（可选 %s）" % (",".join(bad), ",".join(sorted(CMD_SPEC))))
        sys.exit(2)
    if a.report and len(cmd_list) != 1:
        print("✗ --report 只能查单一 --cmd（结果文件按命令分开存）")
        sys.exit(2)

    if a.report:
        sys.exit(do_report(a.out, a.cmd, a.report_scenario))

    # --- 组装传输层 ---
    links = [x.strip().lower() for x in a.link.split(",") if x.strip()]
    known = {"serial", "net", "can"}
    unknowon = [x for x in links if x not in known]
    if unknowon:
        print("✗ 未知链路: %s（可选 serial,net,can）" % ",".join(unknowon))
        sys.exit(2)

    if "net" in links and "can" in links:
        print("⚠⚠ 注意: 网口与 CAN 在本板上【硬件互斥】（CAN2=PB12/PB13 与 RMII 的")
        print("         ETH_RXD1/TXD1 是同一对引脚）。同一次运行里两者不可能都通 ——")
        print("         预期一个链路 0 样本。正确做法: 分两轮烧 LINK=net / LINK=can 固件,")
        print("         各自 --out 存结果, 最后 --report 合并。\n")

    move_cmds = [c for c in cmd_list if c in NEEDS_MOVE]
    if move_cmds and not a.allow_move:
        print("✗ %s 会走控制通路。确认臂周围无人/无障碍后加 --allow-move。" % ",".join(move_cmds))
        print("  （不加 --motion 时脚本会把目标设成当前角，即原地不动，只为测控制类的真实开销。）")
        sys.exit(2)

    # ---- 量大纲换算参数 ----
    offset = parse6(a.stream_offset, "--stream-offset") if a.stream_offset \
        else [SERVO_OFFSET_FACTORY] * 6
    dirs = tuple(parse6(a.stream_dir, "--stream-dir", float)) if a.stream_dir \
        else JOINT_DIR_FACTORY
    if any(d not in (-1.0, 1.0) for d in dirs):
        print("✗ --stream-dir 只能是 ±1")
        sys.exit(2)

    # ---- 真实限位（摆幅会被夹到区间内侧）----
    limits = DEFAULT_JOINT_LIMITS
    if a.joint_limits:
        parts = [x.strip() for x in a.joint_limits.replace("，", ",").split(",") if x.strip()]
        if len(parts) != 6:
            print("✗ --joint-limits 需要 6 段 lo-hi，收到 %d 段" % len(parts))
            sys.exit(2)
        try:
            limits = tuple((float(p.split("-")[0]), float(p.split("-")[1])) for p in parts)
        except Exception:                                # noqa: BLE001
            print("✗ --joint-limits 每段要写成 lo-hi，例如 12-351")
            sys.exit(2)

    # ---- 运动模式参数校验 ----
    motion = None
    scenario = "static"
    if a.motion:
        if len(cmd_list) != 1:
            print("✗ --motion 只能配单一 --cmd（运动模式要独占控制通路）。当前: %s"
                  % ",".join(cmd_list))
            sys.exit(2)
        if cmd_list[0] not in NEEDS_MOVE:
            print("✗ --motion 只对 move_joints / stream_joints 有意义（当前 --cmd %s）。"
                  % cmd_list[0])
            print("  想让臂动、又想量查询类命令的 RTT？那就两条 --cmd 各跑一轮再对比。")
            sys.exit(2)
        try:
            axes = [int(x) for x in str(a.motion_axis).replace("，", ",").split(",") if x.strip()]
        except ValueError:
            print("✗ --motion-axis 要写成逗号分隔的数字，例如 5 或 5,6")
            sys.exit(2)
        if not axes or any(not (1 <= x <= 6) for x in axes):
            print("✗ --motion-axis 每个值必须在 1~6")
            sys.exit(2)
        if len(set(axes)) != len(axes):
            print("✗ --motion-axis 里有重复轴: %s" % a.motion_axis)
            sys.exit(2)
        try:
            amps = [float(x) for x in str(a.motion_amp).replace("，", ",").split(",") if x.strip()]
        except ValueError:
            print("✗ --motion-amp 要写成逗号分隔的数字，例如 90 或 90,20")
            sys.exit(2)
        if len(amps) == 1:
            amps = amps * len(axes)
        if len(amps) != len(axes):
            print("✗ --motion-amp 要给 1 个或 %d 个（与轴数一致），收到 %d 个"
                  % (len(axes), len(amps)))
            sys.exit(2)
        if any(not (0.0 < v <= MOTION_AMP_HARD_MAX) for v in amps):
            print("✗ --motion-amp 每个值必须 >0 且 <= %g 度（硬上限）" % MOTION_AMP_HARD_MAX)
            sys.exit(2)
        if any(v > MOTION_MAX_AMP for v in amps):
            print("⚠⚠ 摆幅超过 %g° 属【大跨度】动作：确认臂周围有足够净空、"
                  "没有线缆/工件，急停在手边。" % MOTION_MAX_AMP)
            print("   运行中每 %gs 会做一次位姿巡检，越界会自动中止并回位。" % a.bucket)
        if any(ax in (2, 3) for ax in axes):
            print("⚠ 摆动轴含 J2/J3（会明显改变末端位置与高度）—— 确认空间够、急停在手边。")
        if a.duration:
            print("⏱ 时长模式 %.0f 秒 = %.1f 分钟（--n 只作安全上限）"
                  % (a.duration, a.duration / 60.0))
            a.n = max(a.n, 10 ** 7)
        motion = {"axes": axes, "amps": amps, "period": a.motion_period,
                  "restore": a.motion_restore, "probe": a.motion_probe,
                  "audit": a.audit}
        scenario = MotionPlan.tag_of(axes, amps, a.motion_period)
    if a.cmd == "stream_joints":
        print("⚠ STREAM_JOINTS 吃【逻辑角】，与 GET_STATUS 的【舵机度】不同量纲（指令大全 §9）。")
        print("  本次换算基准: SERVO_OFFSET=%s  JOINT_DIR=%s"
              % (",".join("%g" % v for v in offset), ",".join("%g" % v for v in dirs)))
        print("  ⚠ Home 校零（zi / 零点标定）改过 SERVO_OFFSET 的话，必须用 --stream-offset 传真实值，")
        print("    否则六轴目标会整体偏移该差值，臂会甩一下。")
        if motion is not None:
            print("⚠ 流模式 + 运动：会先做回读自检（试探 %g° 后回读），不通过就中止。"
                  % PROBE_AMP_DEG)

    transports = []
    for L in links:
        if L == "serial":
            transports.append(SerialTransport(a.serial_port, a.baud))
        elif L == "net":
            transports.append(NetTransport(a.udp_ip, a.udp_port))
        else:
            transports.append(CanTransport(a.can_interface, a.can_channel,
                                           a.can_bitrate, a.can_node))

    print("链路基准测试: cmd=%s  n=%s  warmup=%d  gap=%.3fs  timeout=%.3fs  场景=%s"
          % (",".join(cmd_list),
             ("-" if a.duration else a.n), a.warmup, a.gap, a.timeout, scenario))
    if a.duration:
        print("            ⏱ 时长=%g 秒  分桶=%g 秒  限位=%s"
              % (a.duration, a.bucket,
                 " ".join("J%d:%g-%g" % (i + 1, lo, hi) for i, (lo, hi) in enumerate(limits))))
    for cn in cmd_list:
        if cn in NEEDS_MOVE:
            if motion is None:
                print("⚠ %s 模式：目标=当前角（原地），但会真实走 cmdMove/流模式通路。" % cn)
            else:
                print("⚠⚠ 运动模式：臂会【持续摆动】J%s ±%s°（周期 %gs）。"
                      % ("+".join(str(x) for x in motion["axes"]),
                         "+".join("%g" % v for v in motion["amps"]), a.motion_period))
                print("    确认：臂周围无人/无障碍、急停在手边、夹爪没夹着东西。")
                print("    RTT 里就有「固件主循环正在插补/直写」的真实开销 —— 这才是遥操作该看的数。")
    if len(cmd_list) > 1:
        print("⭐ 多命令模式：下面每个命令各出一张表 —— 把「纯链路命令」(ping) 和")
        print("   「读硬件命令」(status) 放一起跑，就能看出 RTT 里有多少是链路、多少是固件读舵机。")

    tag = time.strftime("%Y%m%d-%H%M%S")
    all_sum = []
    for cn in cmd_list:
        results, samples = [], {}
        for t in transports:
            r = run_one(t, cn, a.n, a.warmup, a.gap, a.timeout, a.allow_move, a.quiet_log,
                        motion=motion, offset=offset, dirs=dirs,
                        duration=a.duration, bucket=a.bucket, limits=limits)
            results.append(r.summary())
            if r.lats:
                samples[r.name] = r.lats

        for s in results:
            s["scenario"] = scenario
        print_table(results, cmd_name=cn)
        save_json(a.out, tag, cn, results,
                  samples=samples if a.csv else None, scenario=scenario)
        all_sum.append((cn, results))

    # 多命令 ⇒ 追加一张「命令耗时谱」横向对比（回答"到底是链路慢还是命令慢"）
    if len(cmd_list) > 1:
        print("")
        print("=" * 96)
        print("  命令耗时谱（同一链路、按命令看 —— 把「链路」与「固件读硬件」分离）")
        print("=" * 96)
        print("  %-14s %-26s %10s %10s %14s" % ("命令", "链路", "p50(ms)", "p99(ms)", "分类"))
        print("  " + "-" * 94)
        for cn, results in all_sum:
            kind = "读硬件(+%.1fms)" % HW_COST_MS if cn in HW_TOUCHING else "纯链路"
            for s in results:
                if s.get("p50") is None:
                    print("  %-14s %-26s %10s %10s %14s" % (cn, s["name"], "-", "-", "无数据"))
                    continue
                print("  %-14s %-26s %10.2f %10.2f %14s"
                      % (cn, s["name"], s["p50"], s.get("p99", 0), kind))
        print("=" * 96)
        print("  ⇒ 结论看法：同一条链路上，纯链路命令与读硬件命令的差值 = 固件读舵机的固定开销；")
        print("     而同一命令在不同链路之间的差值 = 链路本身的差距。两者要分开下结论。")

    print("\n  → 换另一套固件再跑一轮，然后: python tools/link_latency_bench.py --report --cmd %s"
          % cmd_list[0])
    if motion is not None:
        print("  → 想跟「臂不动」时的同一命令对比: 去掉 --motion 再跑一轮，然后")
        print("     python tools/link_latency_bench.py --report --cmd %s --report-scenario motion"
              % cmd_list[0])
        print("     （--report 默认会把两个场景混在一张表里，看「场景」列区分）")


if __name__ == "__main__":
    main()
