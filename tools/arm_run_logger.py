#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_run_logger.py - ArmControl_HAL 运行日志采集工具（每一次运行保存一个文件）

用途：
  代码运行调试时，每一次运行/测试自动保存一份独立日志文件到 logs/，
  便于事后故障定位。同时透传显示串口输出，并支持：
    - 记录带本地时间戳的全部串口流量（RAW 模式）
    - 识别固件 [RUN] 结构化日志行（run_id / 级别 / 模块）
    - 结束时输出 [ERROR]/[WARN] 故障摘要
    - 可选发送文本命令脚本（--script，每行一条）
    - 可选发送二进制协议帧（--proto，见下方命令表）

用法示例：
  python tools/arm_run_logger.py --port COM5
  python tools/arm_run_logger.py --port COM5 --script cmds.txt
  python tools/arm_run_logger.py --port COM5 --proto PING
  python tools/arm_run_logger.py --port COM5 --proto MOVE_CART 150 0 200 0 30 60
  python tools/arm_run_logger.py --port COM5 --proto PARAM_WRITE 16 0.20
  python tools/arm_run_logger.py --port COM5 --duration 30     # 采集 30 秒自动退出

协议命令（--proto）：
  PING                                   握手（返回协议版本+run_id）
  STATUS                                 状态快照（关节角/FK位姿/标志）
  MOVE_JOINTS a1..a6 v a                 关节空间 S 曲线移动（度, deg/s, deg/s^2）
  MOVE_CART x y z pitch v a              笛卡尔 IK-CCD 移动（mm, deg, deg/s, deg/s^2）
  FK                                     FK 查询当前位姿
  STOP                                   停止轨迹
  TORQUE joint en                        扭矩开关（joint 0~5 或 255=全部）
  SET_GRAVITY k                          重力补偿 K（deg/mm, 如 0.03）
  SET_CORRECTION en thr rounds           自适应修正配置
  PARAM_READ id                          读参数（id 见固件 LP 命令）
  PARAM_WRITE id value                   写参数
  PARAM_COUNT                            参数个数
  LOG_LEVEL 0~4                          设置固件日志级别
"""

import argparse
import datetime
import os
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: 需要 pyserial（pip install pyserial）", file=sys.stderr)
    sys.exit(2)

# ---------------- 协议常量（与 Core/Inc/arm_control_protocol.h 一致） ----------------
HEAD = b"\xAA\x55"
CMD = {
    "PING": 0x01, "STATUS": 0x02, "LOG_LEVEL": 0x03,
    "MOVE_JOINTS": 0x10, "STOP": 0x11, "TORQUE": 0x12,
    "MOVE_CART": 0x20, "FK": 0x21,
    "SET_GRAVITY": 0x30, "SET_CORRECTION": 0x31,
    "PARAM_READ": 0x40, "PARAM_WRITE": 0x41, "PARAM_COUNT": 0x42,
}
RSP_NAMES = {0x7F: "ACK", 0x7E: "NACK", 0x81: "PING_RSP", 0x82: "STATUS_RSP",
             0xA1: "FK_RSP", 0xC0: "PARAM_RSP", 0xC2: "COUNT_RSP"}
ERR_NAMES = {0: "OK", 1: "BAD_PARAM", 2: "BAD_CRC", 3: "UNREACHABLE",
             4: "OUT_OF_LIMIT", 5: "ESTOP", 6: "FK_VERIFY", 7: "READ_FAIL"}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(seq: int, cmd: int, payload: bytes) -> bytes:
    body = bytes([seq, cmd, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return HEAD + body + struct.pack("<H", crc)


def parse_frame(buf: bytes):
    """从缓冲中解析一帧，返回 (frame_dict, 剩余缓冲) 或 (None, buf)。"""
    i = buf.find(HEAD)
    if i < 0:
        return None, buf[-1:] if buf.endswith(b"\xAA") else b""
    buf = buf[i:]
    if len(buf) < 5:
        return None, buf
    ln = buf[4]
    total = 5 + ln + 2
    if len(buf) < total:
        return None, buf
    frame = buf[:total]
    seq, cmd, plen = frame[2], frame[3], frame[4]
    payload = frame[5:5 + plen]
    crc_got = struct.unpack("<H", frame[5 + plen:7 + plen])[0]
    crc_ok = crc16_ccitt(frame[2:5 + plen]) == crc_got
    return {"seq": seq, "cmd": cmd, "payload": payload, "crc_ok": crc_ok}, buf[total:]


def s16(b): return struct.unpack("<h", b)[0]
def u16(b): return struct.unpack("<H", b)[0]
def s32(b): return struct.unpack("<i", b)[0]


def decode_rsp(fr: dict) -> str:
    cmd, p = fr["cmd"], fr["payload"]
    name = RSP_NAMES.get(cmd, f"RSP_0x{cmd:02X}")
    if not fr["crc_ok"]:
        return f"{name} CRC-ERROR!"
    if cmd in (0x7F, 0x7E) and len(p) >= 2:
        return f"{name} cmd=0x{p[0]:02X} err={p[1]}({ERR_NAMES.get(p[1], '?')})"
    if cmd == 0x81 and len(p) >= 2:
        return f"PING_RSP proto_ver={p[0]} run_id=0x{p[1]:02X}"
    if cmd == 0x82 and len(p) >= 21:
        ang = [s16(p[i * 2:i * 2 + 2]) / 10 for i in range(6)]
        x, y, z = s16(p[12:14]) / 10, s16(p[14:16]) / 10, s16(p[16:18]) / 10
        pitch, flags = s16(p[18:20]) / 10, p[20]
        return (f"STATUS angles={ang} xyz=({x},{y},{z})mm pitch={pitch} "
                f"estop={flags & 1} traj={(flags >> 1) & 1} corr={(flags >> 2) & 1} "
                f"ok={(flags >> 3) & 1}")
    if cmd == 0xA1 and len(p) >= 21:
        x, y, z = s16(p[0:2]) / 10, s16(p[2:4]) / 10, s16(p[4:6]) / 10
        pitch = s16(p[6:8]) / 10
        ang = [s16(p[8 + i * 2:10 + i * 2]) / 10 for i in range(6)]
        return f"FK xyz=({x},{y},{z})mm pitch={pitch} angles={ang} ok={p[20]}"
    if cmd == 0xC0 and len(p) >= 5:
        return f"PARAM_RSP id={p[0]} value={s32(p[1:5]) / 1000}"
    if cmd == 0xC2 and len(p) >= 1:
        return f"COUNT_RSP count={p[0]}"
    return f"{name} payload={p.hex()}"


def build_proto_command(name: str, args):
    """按命令名构造 (cmd_code, payload)。"""
    f = lambda s: float(s)
    if name == "PING":
        return CMD["PING"], b""
    if name == "STATUS":
        return CMD["STATUS"], b""
    if name == "FK":
        return CMD["FK"], b""
    if name == "STOP":
        return CMD["STOP"], b""
    if name == "PARAM_COUNT":
        return CMD["PARAM_COUNT"], b""
    if name == "MOVE_JOINTS":
        if len(args) != 8:
            raise ValueError("MOVE_JOINTS 需要 8 个参数: a1..a6 v a")
        vals = [f(a) for a in args]
        payload = b"".join(struct.pack("<h", round(v * 10)) for v in vals[:6])
        payload += struct.pack("<HH", round(vals[6] * 10), round(vals[7] * 10))
        return CMD["MOVE_JOINTS"], payload
    if name == "MOVE_CART":
        if len(args) != 6:
            raise ValueError("MOVE_CART 需要 6 个参数: x y z pitch v a")
        vals = [f(a) for a in args]
        payload = struct.pack("<hhh", round(vals[0] * 10), round(vals[1] * 10),
                              round(vals[2] * 10))
        payload += struct.pack("<h", round(vals[3] * 10))
        payload += struct.pack("<HH", round(vals[4] * 10), round(vals[5] * 10))
        return CMD["MOVE_CART"], payload
    if name == "TORQUE":
        return CMD["TORQUE"], bytes([int(args[0]) & 0xFF, int(args[1]) & 0xFF])
    if name == "SET_GRAVITY":
        return CMD["SET_GRAVITY"], struct.pack("<h", round(f(args[0]) * 1000))
    if name == "SET_CORRECTION":
        return CMD["SET_CORRECTION"], bytes([int(args[0]) & 0xFF]) + \
               struct.pack("<H", round(f(args[1]) * 100)) + bytes([int(args[2]) & 0xFF])
    if name == "PARAM_READ":
        return CMD["PARAM_READ"], bytes([int(args[0]) & 0xFF])
    if name == "PARAM_WRITE":
        return CMD["PARAM_WRITE"], bytes([int(args[0]) & 0xFF]) + \
               struct.pack("<i", round(f(args[1]) * 1000))
    if name == "LOG_LEVEL":
        return CMD["LOG_LEVEL"], bytes([int(args[0]) & 0xFF])
    raise ValueError(f"未知协议命令: {name}")


def main():
    ap = argparse.ArgumentParser(description="ArmControl_HAL 运行日志采集（每次运行保存独立文件）")
    ap.add_argument("--port", required=True, help="串口号，如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--logdir", default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "..", "logs"), help="日志目录（默认 ../logs）")
    ap.add_argument("--duration", type=float, default=0,
                    help="采集秒数，0=持续直到 Ctrl+C")
    ap.add_argument("--script", help="文本命令脚本（每行一条，#开头为注释）")
    ap.add_argument("--proto", nargs="+",
                    help="发送二进制协议命令，如: --proto MOVE_CART 150 0 200 0 30 60")
    ap.add_argument("--quiet", action="store_true", help="不在终端回显串口内容")
    args = ap.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(args.logdir, f"run_{stamp}.log")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()

    logf = open(log_path, "w", encoding="utf-8", errors="replace")
    header = (f"# ArmControl_HAL run log\n# file: {os.path.basename(log_path)}\n"
              f"# start: {datetime.datetime.now().isoformat(timespec='seconds')}\n"
              f"# port: {args.port} @ {args.baud}\n")
    logf.write(header)
    print(f"[logger] 采集开始 -> {log_path}")
    print("[logger] Ctrl+C 结束并生成故障摘要")

    warnings, errors = [], []
    rx_bin = b""          # 二进制帧重组缓冲
    rx_line = b""         # 文本行缓冲
    seq = [0]             # 协议序号（列表便于闭包修改）

    def send_text(cmdline: str):
        ser.write(cmdline.encode() + b"\r\n")
        logf.write(f"# >>> {cmdline}\n")

    def send_proto(name: str, pargs):
        cmd_code, payload = build_proto_command(name, pargs)
        frame = build_frame(seq[0] & 0xFF, cmd_code, payload)
        seq[0] += 1
        ser.write(frame)
        logf.write(f"# >>> PROTO {name} {' '.join(pargs)} (frame {frame.hex()})\n")
        print(f"[proto] TX {name} -> {frame.hex()}")

    def handle_text(line: str):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        logf.write(f"[{ts}] {line}\n")
        if not args.quiet:
            print(line)
        if "[ERROR]" in line or "FAIL" in line or "TIMEOUT" in line:
            errors.append(line)
        elif "[WARN ]" in line or "WARNING" in line:
            warnings.append(line)

    # 启动后发送用户命令
    pending_scripts = []
    if args.script:
        with open(args.script, encoding="utf-8") as sf:
            pending_scripts = [l.strip() for l in sf
                               if l.strip() and not l.startswith("#")]
    if args.proto:
        send_proto(args.proto[0].upper(), args.proto[1:])
    for cmdline in pending_scripts:
        send_text(cmdline)
        time.sleep(0.15)

    t0 = time.time()
    try:
        while True:
            if args.duration and (time.time() - t0) > args.duration:
                break
            data = ser.read(256)
            if not data:
                continue
            rx_bin += data
            # 分流：队首非帧头 -> 文本行缓冲；帧头候选 -> 尝试解析协议帧；
            # 帧未收齐则保留在 rx_bin 等下一轮（不丢字节）
            while rx_bin:
                if rx_bin[0] != 0xAA:
                    rx_line += rx_bin[:1]
                    rx_bin = rx_bin[1:]
                    continue
                if len(rx_bin) < 2:
                    break                       # 等第二个帧头字节
                if rx_bin[1] != 0x55:
                    rx_bin = rx_bin[1:]         # 杂散 0xAA，丢弃（同固件状态机）
                    continue
                fr, rx_bin = parse_frame(rx_bin)
                if fr is None:
                    break                       # 帧未收齐，保留等待
                msg = f"[proto] RX {decode_rsp(fr)}"
                logf.write(f"# {msg}\n")
                print(msg)
            # 文本行处理
            while b"\n" in rx_line:
                line, rx_line = rx_line.split(b"\n", 1)
                handle_text(line.decode("utf-8", errors="replace").rstrip("\r"))
    except KeyboardInterrupt:
        pass
    finally:
        if rx_line:
            handle_text(rx_line.decode("utf-8", errors="replace").rstrip("\r"))
        logf.write(f"# end: {datetime.datetime.now().isoformat(timespec='seconds')}\n")
        logf.write(f"# summary: {len(errors)} error(s), {len(warnings)} warning(s)\n")
        logf.close()
        ser.close()

    print(f"\n[logger] 日志已保存: {log_path}")
    print(f"[logger] 摘要: {len(errors)} 个错误, {len(warnings)} 个警告")
    for line in errors[:20]:
        print(f"  [ERR] {line}")
    for line in warnings[:20]:
        print(f"  [WRN] {line}")


if __name__ == "__main__":
    main()
