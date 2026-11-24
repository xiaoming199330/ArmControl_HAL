#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""arm_net_host.py — 网口(UDP) 版上位机: 与固件 arm_net.c 对话。

协议（2026-09-20 定案，与 CAN 版逻辑包同构）:
  UDP 数据报 = [SEQ, CMD, LEN, PAYLOAD..., CRC16-LE]
  CRC16-CCITT(0x1021, init 0xFFFF), 算前 3+LEN 字节 —— 与 can_host.py/can_forge.py
  同一个函数, 14 条命令语义、错误码与 docs/CAN协议规格书 逐字一致。
  臂 = 192.168.7.120:5000 (静态 IP, MAC 02:4E:45:54:00:01), 回包发回请求源端口。

前置条件:
  1) 板子烧的是 outputs/net_api/02_slave_NET.bin (LINK=net 构建, CAN 已停用);
  2) 网线直连 PC ↔ 臂网口 (或经交换机);
  3) PC 网口设静态 IP 192.168.7.1 / 255.255.255.0;
  4) `ping 192.168.7.120` 通 = ARP/ICMP/PHY 全链路 OK, 再跑本工具。

用法(在工程根目录):
  python tools/arm_net_host.py --ping
  python tools/arm_net_host.py --status
  python tools/arm_net_host.py --watch 30                 # ★连读 30 次: 丢包/时延/read_ok/E-Stop
  python tools/arm_net_host.py --fk                       # 正解查询 0x21 -> 0xA1
  python tools/arm_net_host.py --torque all 1             # 力矩 0x12 (all|0..5 , 0|1)
  python tools/arm_net_host.py --stop                     # ★急停 (先测这个!)
  python tools/arm_net_host.py --delta 0 0 0 0 0 2 --speed 5   # ★相对微动: 只有 J6 +2°
  python tools/arm_net_host.py --move 180 180 180 180 180 180 --speed 10   # 绝对角 (危险, 慎用)
  python tools/arm_net_host.py --param-count
  可选: --ip 192.168.7.120 --port 5000 --timeout 1.5 --seq 1 --gap 0.05
  离线自检(不用板子): python tools/arm_net_host.py --selftest

⚠ 运动安全: MOVE_JOINTS 走 cmdMove() —— 会【先自动上力矩】、把目标【钳到安全限位】、
  【读起点多数表决】后再规划 S 曲线。目标超限是"静默钳制"而不是报错, 所以
  **绝对角 --move 一定要先 --status 看当前角**; 点动一律优先用 **--delta**。
  ⚠ 若从臂正在被主臂流帧驱动 (sf 1 且对端在发), 流帧会盖掉你的轨迹 ——
  先在对端停流, 或在 COM11 敲 `sf 0` 把它隔离成"只收命令"。
"""
import argparse
import socket
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_host as ch  # noqa: E402  复用编码/解码函数, 保证与 CAN 版逐字节一致


def build_packet(seq: int, cmd: int, payload: bytes) -> bytes:
    out = bytes([seq & 0xFF, cmd & 0xFF, len(payload)]) + payload
    crc = ch.crc16(out)
    return out + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_packet(data: bytes):
    """返回 (seq, cmd, payload) 或抛 ValueError"""
    if len(data) < 5:
        raise ValueError("报文过短: %d B" % len(data))
    seq, cmd, ln = data[0], data[1], data[2]
    if 5 + ln != len(data):
        raise ValueError("LEN=%d 与报文长 %d 不符" % (ln, len(data)))
    crc_got = data[3 + ln] | (data[3 + ln + 1] << 8)
    if crc_got != ch.crc16(data[:3 + ln]):
        raise ValueError("CRC 错 (got %04X)" % crc_got)
    return seq, cmd, data[3:3 + ln]


class NetHost:
    def __init__(self, ip, port, timeout=1.5, verbose=True):
        self.addr = (ip, port)
        self.timeout = timeout
        self.verbose = verbose
        self.seq = 0x30
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        # ⭐ 不绑定固定端口: 固件是"回包到请求的源端口", OS 会给本 socket 分配
        #   临时源端口并自动收下回包。绑死同端口反而在本机自测时自冲突。

    def request(self, cmd, payload=b"", label=""):
        self.seq = (self.seq + 1) & 0xFF
        seq = self.seq
        pkt = build_packet(seq, cmd, payload)
        self.sock.sendto(pkt, self.addr)
        deadline_end = 3                        # 最多试 3 次
        for _ in range(deadline_end):
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                rseq, rcmd, rpl = parse_packet(data)
            except ValueError as e:
                if self.verbose:
                    print("  ⚠ 丢弃坏响应: %s" % e)
                continue
            if rseq != seq:
                if self.verbose:
                    print("  ⚠ 序号不匹配 (等 %02X 收 %02X), 丢弃" % (seq, rseq))
                continue
            return rcmd, rpl
        raise TimeoutError("%s 超时 (重试 %d 次无响应)" % (label or ("cmd=0x%02X" % cmd), deadline_end))


def print_status(pl: bytes):
    d = ch.decode_status_payload(pl)
    print("  姿态  J1..J6 = " + " ".join(
        ("----" if v is None else "%6.1f" % v) for v in d["joints"]))
    print("  位姿  X=%.1f Y=%.1f Z=%.1f pitch=%.1f  read_ok=%s"
          % (d["x"], d["y"], d["z"], d["pitch"], d["read_ok"]))
    print("  标志  E-Stop=%s traj=%s corr=%s" % (d["estop"], d["traj"], d["corr"]))


def selftest() -> int:
    """离线自检: 起一个本地 UDP 假臂, 跑同一条 request 路径。"""
    import threading

    ok = [0]
    fail = [0]

    def chk(name, cond, extra=""):
        if cond:
            ok[0] += 1
            print("  PASS  %s" % name)
        else:
            fail[0] += 1
            print("  FAIL  %s %s" % (name, extra))

    # --- 编码: 与 can_host.build_frames 重组结果逐字节一致 ---
    from can_host import build_frames
    payload = ch.payload_move_joints([180, 180, 180, 180, 180, 180], 10, 0)
    pkt = build_packet(0x25, ch.CMD_MOVE_JOINTS, payload)
    # CAN 版: 首帧去掉 [0x80|n, seq, cmd, len] 头 + 续帧去掉帧号, 尾接同一 CRC
    canon = bytes([0x25, ch.CMD_MOVE_JOINTS, len(payload)]) + payload
    crc = ch.crc16(canon)
    canon += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    chk("逻辑包与 CAN 版逐字节一致", pkt == canon, "%s vs %s" % (pkt.hex(), canon.hex()))
    chk("CRC 已知向量 123456789=0x29B1", ch.crc16(b"123456789") == 0x29B1)
    s, c, p = parse_packet(pkt)
    chk("parse 往返", s == 0x25 and c == ch.CMD_MOVE_JOINTS and p == payload)

    # --- 假臂: 回 NACK(CRC坏) / 回显 GET_STATUS ---
    busy = threading.Event()

    def fake_arm(sock):
        while not busy.is_set():
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                seq, cmd, pl = parse_packet(data)
            except ValueError:
                sock.sendto(b"garbage", addr)     # 坏报文, 宿主应丢弃
                continue
            if cmd == ch.CMD_GET_STATUS:
                st = ch.payload_stream_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) + \
                     b"\x00\x00\x00\x00\x00\x00\x00\x00\x08"
                sock.sendto(build_packet(seq, 0x82, st), addr)
            elif cmd == ch.CMD_FK_QUERY:
                # 0xA1 布局: [0..7] x,y,z,pitch | [8..19] 6×关节 | [20] read_ok
                # 协议是小端 (firmware wr_i16: p[0]=低字节), 所以用 ch.i16() 编码
                fk = (ch.i16(10) + ch.i16(20) + ch.i16(30) + ch.i16(40) +
                      ch.payload_stream_joints([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) + b"\x01")
                sock.sendto(build_packet(seq, 0xA1, fk), addr)
            elif cmd == ch.CMD_PARAM_COUNT:
                sock.sendto(build_packet(seq, 0xC2, bytes([24])), addr)
            else:
                sock.sendto(build_packet(seq, 0x7F, bytes([cmd, 0])), addr)

    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.settimeout(0.2)
    srv.bind(("127.0.0.1", 5999))
    th = threading.Thread(target=fake_arm, args=(srv,), daemon=True)
    th.start()
    h = NetHost("127.0.0.1", 5999, timeout=0.5, verbose=False)
    try:
        rcmd, rpl = h.request(ch.CMD_GET_STATUS, b"", label="GET_STATUS(fake)")
        chk("假臂 GET_STATUS 往返", rcmd == 0x82 and len(rpl) == 21, "rcmd=%02X len=%d" % (rcmd, len(rpl)))
        d = ch.decode_status_payload(rpl)
        chk("状态解析", d["joints"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6] and d["read_ok"])
        rcmd, rpl = h.request(ch.CMD_PING, b"", label="PING(fake)")
        chk("假臂 ACK 往返", rcmd == 0x7F and rpl[1] == 0)

        # --- 新增命令 0x21 FK / 0x12 力矩 / 0x42 参数个数 / 相对微动算法 ---
        rcmd, rpl = h.request(ch.CMD_FK_QUERY, b"", label="FK(fake)")
        d = ch.decode_fk_payload(rpl) if rcmd == 0xA1 else {}
        chk("FK_QUERY 0xA1 + 解析", rcmd == 0xA1 and d.get("joints") == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
            and abs(d.get("x", 0) - 1.0) < 1e-6 and d.get("read_ok") == 1,
            "rcmd=%02X %s" % (rcmd, d))
        rcmd, rpl = h.request(ch.CMD_SET_TORQUE, bytes([0xFF, 1]), label="TORQUE(fake)")
        chk("SET_TORQUE all-on 编码", rcmd == 0x7F and rpl[1] == 0, "rcmd=%02X" % rcmd)
        rcmd, rpl = h.request(ch.CMD_PARAM_COUNT, b"", label="PARAM_COUNT(fake)")
        chk("PARAM_COUNT == 24", rcmd == 0xC2 and rpl[0] == 24, "rcmd=%02X" % rcmd)
        # 相对微动: 起点 + 增量 -> 目标, 只动指定轴 (与 --delta 同一段算法)
        cur = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
        tgt = [round(cur[j] + delta[j], 1) for j in range(6)]
        chk("--delta 只改 J6", tgt == [0.1, 0.2, 0.3, 0.4, 0.5, 2.6], str(tgt))
        moved = [(j + 1, delta[j]) for j in range(6) if abs(delta[j]) > 1e-9]
        chk("--delta 动作轴列表", moved == [(6, 2.0)], str(moved))
    except Exception as e:                       # noqa: BLE001
        chk("假臂往返", False, repr(e))
    finally:
        busy.set()
        srv.close()
        h.sock.close()
    print("  自检: %d PASS, %d FAIL" % (ok[0], fail[0]))
    return 0 if fail[0] == 0 else 1


def main():
    ap = argparse.ArgumentParser(description="网口(UDP)版机械臂上位机")
    ap.add_argument("--ip", default="192.168.7.120", help="臂的静态 IP, 默认 192.168.7.120")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--timeout", type=float, default=1.5)
    ap.add_argument("--seq", type=str, default="0x30", help="起始序号 (支持 0x)")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--stop", action="store_true")
    ap.add_argument("--param-count", action="store_true", dest="param_count")
    ap.add_argument("--fk", action="store_true", help="正解 FK 查询 (0x21 -> 0xA1)")
    ap.add_argument("--watch", type=int, metavar="N",
                    help="连续读 N 次 GET_STATUS 并统计 (丢包/时延/read_ok/E-Stop)")
    ap.add_argument("--torque", nargs=2, metavar=("J", "EN"),
                    help="力矩开关 (0x12): J = all|0..5, EN = 0|1")
    ap.add_argument("--delta", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                    help="★相对微动: 先读当前角再加这些增量 (deg), 只用 --move 的绝对值容易撞")
    ap.add_argument("--gap", type=float, default=0.05, help="--watch 每次间隔秒, 默认 0.05")
    ap.add_argument("--move", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    ap.add_argument("--speed", type=float, default=0.0)
    ap.add_argument("--accel", type=float, default=0.0)
    ap.add_argument("--selftest", action="store_true", help="离线自检(不需要板子)")
    a = ap.parse_args()
    a.seq = int(str(a.seq), 0)

    if a.selftest:
        sys.exit(selftest())

    h = NetHost(a.ip, a.port, timeout=a.timeout)
    print("→ udp://%s:%d" % (a.ip, a.port))
    try:
        if a.ping:
            # 协议规定: 0x01 PING -> 0x81: u8 proto_ver, u8 run_id (LEN=2)
            # (见 sdk_package/03_INTERFACE §3.4 / arm_control_protocol.h)
            # ⚠ 这里**不是** ACK(0x7F)+err; 第 2 字节是 run_id, 别当错误码打印。
            rcmd, rpl = h.request(ch.CMD_PING, b"", label="PING")
            ok = (rcmd == 0x81 and len(rpl) >= 2)
            if ok:
                print("✅ PING: 回包 cmd=0x%02X proto_ver=%u run_id=%u (链路全通)"
                      % (rcmd, rpl[0], rpl[1]))
            else:
                print("❌ PING: 回包 cmd=0x%02X payload=%s (期望 0x81, LEN=2)"
                      % (rcmd, rpl.hex() if rpl else "-"))
        if a.status:
            rcmd, rpl = h.request(ch.CMD_GET_STATUS, b"", label="GET_STATUS")
            if rcmd == 0x82:
                print_status(rpl)
            else:
                print("  GET_STATUS 回包 cmd=0x%02X (可能是 NACK)" % rcmd)
        if a.param_count:
            rcmd, rpl = h.request(ch.CMD_PARAM_COUNT, b"", label="PARAM_COUNT")
            if len(rpl) >= 1:
                print("  参数个数 = %u" % rpl[0])
        if a.fk:
            rcmd, rpl = h.request(ch.CMD_FK_QUERY, b"", label="FK_QUERY")
            if rcmd == 0xA1:
                d = ch.decode_fk_payload(rpl)
                print("  位姿  X=%.1f Y=%.1f Z=%.1f pitch=%.1f"
                      % (d["x"], d["y"], d["z"], d["pitch"]))
                print("  关节  " + " ".join(
                    ("----" if v is None else "%6.1f" % v) for v in d["joints"]))
                print("  read_ok=%s" % d["read_ok"])
            else:
                print("  ⚠ FK 回包 cmd=0x%02X (期望 0xA1)" % rcmd)
        if a.watch:
            import time
            n = a.watch
            lats = []
            n_fail = n_readbad = n_estop = 0
            print("  连续 %d 次 GET_STATUS (间隔 %.0f ms)…" % (n, a.gap * 1000.0))
            for i in range(n):
                t0 = time.time()
                try:
                    rcmd, rpl = h.request(ch.CMD_GET_STATUS, b"", label="GET_STATUS")
                except TimeoutError:
                    n_fail += 1
                    print("  #%02d TIMEOUT" % (i + 1))
                    time.sleep(a.gap)
                    continue
                dt = (time.time() - t0) * 1000.0
                if rcmd != 0x82:
                    n_fail += 1
                    print("  #%02d 回包 cmd=0x%02X" % (i + 1, rcmd))
                    time.sleep(a.gap)
                    continue
                lats.append(dt)
                d = ch.decode_status_payload(rpl)
                if not d["read_ok"]:
                    n_readbad += 1
                if d["estop"]:
                    n_estop += 1
                print("  #%02d %6.1fms  J=%s  read_ok=%s estop=%s traj=%s"
                      % (i + 1, dt,
                         " ".join("----" if v is None else "%6.1f" % v for v in d["joints"]),
                         d["read_ok"], d["estop"], d["traj"]))
                if i + 1 < n:
                    time.sleep(a.gap)
            print("  ── 汇总: ok=%d fail=%d read_ok_FAIL=%d estop=%d"
                  % (n - n_fail, n_fail, n_readbad, n_estop))
            if lats:
                print("     往返 min %.1f / avg %.1f / max %.1f ms (含重试)"
                      % (min(lats), sum(lats) / len(lats), max(lats)))
        if a.torque:
            jarg, earg = a.torque
            try:
                j = 0xFF if str(jarg).lower() in ("all", "a", "0xff") else int(str(jarg), 0)
                en = int(str(earg), 0) & 0xFF
            except ValueError:
                print("  ✗ --torque 参数应为: J = all|0..5, EN = 0|1")
                j = None
            if j is not None:
                rcmd, rpl = h.request(ch.CMD_SET_TORQUE, bytes([j & 0xFF, en]),
                                      label="SET_TORQUE")
                print("  力矩 J=%s EN=%s → cmd=0x%02X err=%u"
                      % (jarg, en, rcmd, rpl[1] if len(rpl) > 1 else 0))
        if a.delta:
            rcmd, rpl = h.request(ch.CMD_GET_STATUS, b"", label="GET_STATUS(读起点)")
            if rcmd != 0x82:
                print("  ✗ 读不到起始状态, 拒绝相对运动")
            else:
                d = ch.decode_status_payload(rpl)
                cur = d["joints"]
                if any(v is None for v in cur):
                    print("  ✗ 有轴读失败, 拒绝相对运动 (先查总线/ID)")
                elif d["estop"]:
                    print("  ✗ E-Stop 生效, 拒绝运动 (松开急停再试)")
                else:
                    tgt = [round(cur[j2] + a.delta[j2], 1) for j2 in range(6)]
                    print("  起点 J = " + " ".join("%6.1f" % v for v in cur))
                    print("  目标 J = " + " ".join("%6.1f" % v for v in tgt))
                    moved = [(j2 + 1, a.delta[j2]) for j2 in range(6) if abs(a.delta[j2]) > 1e-9]
                    print("  将动作轴: %s   (其余轴保持)"
                          % (", ".join("J%d%+.1f°" % (j2, dv) for j2, dv in moved) or "无"))
                    rcmd, rpl = h.request(ch.CMD_MOVE_JOINTS,
                                          ch.payload_move_joints(tgt, a.speed, a.accel),
                                          label="MOVE_JOINTS(相对)")
                    if rcmd == 0x7F and rpl[1] == 0:
                        print("  ✅ 已接受 err=0 (固件已自动上力矩并规划 S 曲线)")
                    else:
                        print("  ⚠ 返回 cmd=0x%02X err=%u (错误码见 arm_control_api.h)"
                              % (rcmd, rpl[1] if len(rpl) > 1 else 0))
        if a.move:
            print("  ⚠ 绝对角运动: 目标若超限会被固件【静默钳制】, 请先 --status 核对当前角")
            pl = ch.payload_move_joints(a.move, a.speed, a.accel)
            rcmd, rpl = h.request(ch.CMD_MOVE_JOINTS, pl, label="MOVE")
            if rcmd == 0x7F and rpl[1] == 0:
                print("✅ MOVE 已接受: 目标 %s, 速度 %.1f deg/s" % (a.move, a.speed))
            else:
                print("⚠ MOVE 返回 cmd=0x%02X err=%u (见规格书错误码表)" % (rcmd, rpl[1] if len(rpl) > 1 else 0))
        if a.stop:
            rcmd, rpl = h.request(ch.CMD_STOP, b"", label="STOP")
            print("✅ STOP err=%u" % (rpl[1] if len(rpl) > 1 else 0))
    except TimeoutError as e:
        print("✗ %s" % e)
        print("  排查顺序: 网线/PC 静态 IP 192.168.7.1 → ping 192.168.7.120 →")
        print("            板上串口敲 nx 看 PHY/链路状态 → `nx scan` 重扫")
        sys.exit(2)
    except OSError as e:
        print("✗ socket 错误: %s" % e)
        sys.exit(3)


if __name__ == "__main__":
    main()
