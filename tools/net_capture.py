#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
net_capture.py —— 用 Npcap + scapy 在网卡上抓原始以太帧，专治"对端到底回没回包"。

为什么需要它：Windows 的 IP 层会**静默丢掉校验和错误的包**，
所以 `ping` 和普通 raw socket 都看不见"发了但算错"的回包。
只有 NDIS 层（Npcap）能看到它们，从而区分：
  · 抓到了回包、但校验和是错的  ⇒ 对端固件在"算校验和"这一步有问题
  · 一个回包都没有              ⇒ 对端根本没发

用法（需管理员，已装 Npcap）:
  python tools/net_capture.py --local 192.168.7.1 --peer 192.168.7.120 --sec 6
  python tools/net_capture.py --local 192.168.7.1 --peer 192.168.7.120 --sec 6 --ping-payload 32
  python tools/net_capture.py --list          # 只列网卡
"""
import argparse
import os
import struct
import sys
import threading
import time

# scapy 在 Windows 上会去读 %ProgramFiles% 找 wireshark；Git Bash 里这个变量通常没设,
# 缺了会直接 KeyError 崩在 import 阶段。这里补默认值（只影响本进程)。
for _k, _v in (("ProgramFiles", r"C:\Program Files"),
               ("ProgramFiles(x86)", r"C:\Program Files (x86)"),
               ("ProgramW6432", r"C:\Program Files")):
    os.environ.setdefault(_k, _v)


def _iface_ips(iface):
    try:
        return list(iface.ips[4])
    except Exception:
        return []


def list_ifaces():
    from scapy.all import conf
    for i in conf.ifaces.values():
        print("%-42s %-38s %s" % (getattr(i, "name", "?"),
                                  (getattr(i, "description", "") or "")[:38],
                                  _iface_ips(i)))
    return 0


def checksum_ok(buf: bytes) -> bool:
    if len(buf) & 1:
        buf += b"\x00"
    s = 0
    for i in range(0, len(buf), 2):
        s += (buf[i] << 8) | buf[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF == 0


def correct_cksum_of(buf: bytes) -> int:
    """把校验和字段(偏移 2..3)清零后算出"正确值"，用来跟实际存的值对比。"""
    b = bytearray(buf)
    if len(b) >= 4:
        b[2] = 0
        b[3] = 0
    return checksum_of(bytes(b))


def build_echo(ident: int, seq: int, payload_len: int) -> bytes:
    payload = bytes((0x61 + i % 26) for i in range(payload_len))
    head = struct.pack("!BBHHH", 8, 0, 0, ident, seq)
    return struct.pack("!BBHHH", 8, 0, checksum_of(head + payload), ident, seq) + payload


def checksum_of(buf: bytes) -> int:
    if len(buf) & 1:
        buf += b"\x00"
    s = 0
    for i in range(0, len(buf), 2):
        s += (buf[i] << 8) | buf[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="只列网卡")
    ap.add_argument("--local", default="192.168.7.1", help="本机在该网段的 IP")
    ap.add_argument("--peer", default="192.168.7.120", help="对端 IP")
    ap.add_argument("--sec", type=float, default=6.0, help="抓包时长")
    ap.add_argument("--ping-payload", type=int, default=32,
                    help="抓包期间同时发一个 ICMP echo（-1 表示不发）")
    ap.add_argument("--no-ping", action="store_true", help="抓包期间不发任何东西")
    ap.add_argument("--dump", action="store_true", help="额外打印原始十六进制")
    ap.add_argument("--ihl6", action="store_true",
                    help="发一个带 IP 选项(IHL=6)的 echo, 用回包的 ver/ihl 识别固件代次")
    ap.add_argument("--forge-zero", action="store_true",
                    help="伪造一个请求, 使其『回包应有的正确校验和』恰为 0x0000: "
                         "若对端只是『不写校验和字段』, 它的回包就能通过校验 "
                         "⇒ 证明回包内容完好、缺陷只在那一字段")
    a = ap.parse_args()

    if a.list:
        return list_ifaces()

    from scapy.all import AsyncSniffer, conf

    target = None
    for i in conf.ifaces.values():
        if a.local in _iface_ips(i):
            target = i
            break
    if target is None:
        print("找不到拥有 %s 的网卡。用 --list 看看：" % a.local)
        list_ifaces()
        return 2

    dev = getattr(target, "network_name", None) or getattr(target, "name", None)
    print("抓包网卡: %s (%s)" % (dev, (getattr(target, "description", "") or "")[:50]))

    frames = []

    def on_pkt(pkt):
        try:
            raw = bytes(pkt)
        except Exception:
            return
        if len(raw) < 14:
            return
        src_mac = raw[6:12]
        et = (raw[12] << 8) | raw[13]
        # 只要来自对端 MAC 的帧
        frames.append(raw)

    sniffer = AsyncSniffer(iface=dev, prn=on_pkt, store=False)
    sniffer.start()
    time.sleep(0.5)

    sent = 0
    if a.ihl6:
        # 发一个 IP 头带 4 字节选项(IHL=6)的 echo request：
        #   · 新式"逐字段写头"的固件 -> 回包 Ver/IHL=0x45, totlen=20+ICMP长度
        #   · 旧式"复制请求头 12 字节"的固件 -> 回包 Ver/IHL=0x46, totlen 沿用请求的 24+ICMP长度
        import socket as _s
        tx = _s.socket(_s.AF_INET, _s.SOCK_RAW, _s.IPPROTO_RAW)
        try:
            tx.setsockopt(_s.IPPROTO_IP, getattr(_s, "IP_HDRINCL", 2), 1)
        except OSError:
            pass
        icmp = build_echo(0x1234, 9, a.ping_payload)
        ip_hdr = struct.pack("!BBHHHBBH4s4s", 0x46, 0, 24 + len(icmp), 0x4321, 0, 64, 1, 0,
                             _s.inet_aton(a.local), _s.inet_aton(a.peer)) + b"\x01\x01\x01\x00"
        assert len(ip_hdr) == 24
        ip_hdr = ip_hdr[:10] + struct.pack("!H", checksum_of(ip_hdr)) + ip_hdr[12:]
        tx.sendto(ip_hdr + icmp, (a.peer, 1))
        sent = 1
        tx.close()
        print("已发 1 个 IHL=6 的 echo request (IP 头 24 字节, 带 4 字节选项)")
    elif a.forge_zero:
        # 构造一个 echo request, 使"对方应当回的正确校验和"恰为 0x0000。
        # 若对端固件是"回包内容全对、只是没把算好的校验和写进去", 它回包里的
        # 0x0000 就恰好等于正确值 ⇒ 回包被本机判为有效 ⇒ 缺陷点被隔离到"那一字段"。
        # ⚠ 注意: 若对端"正确写了校验和", 结果同样是 0x0000, 所以本项**单独不能**
        #    证明"没写"; 要定论必须配合默认 ping (正确值非 0、回包却是 0)。
        import socket as _s
        payload = bytearray((0x61 + i % 26) for i in range(max(4, a.ping_payload)))
        good = None
        for w in range(65536):
            payload[-2] = w >> 8
            payload[-1] = w & 0xFF
            if checksum_of(struct.pack("!BBHHH", 0, 0, 0, 0x1234, 1) + bytes(payload)) == 0:
                good = w
                break
        req = struct.pack("!BBHHH", 8, 0, 0, 0x1234, 1) + bytes(payload)
        tx = _s.socket(_s.AF_INET, _s.SOCK_RAW, _s.IPPROTO_ICMP)
        tx.sendto(req, (a.peer, 1))
        tx.close()
        sent = 1
        print("已发 1 个『正确回包校验和=0』的伪造 echo (末两字节=%#06x)" % (good or 0))
    elif not a.no_ping and a.ping_payload >= 0:
        import socket as _s
        tx = _s.socket(_s.AF_INET, _s.SOCK_RAW, _s.IPPROTO_ICMP)
        tx.settimeout(1.0)
        for n in range(3):
            tx.sendto(build_echo(0x1234, n + 1, a.ping_payload), (a.peer, 1))
            sent += 1
            time.sleep(0.4)
        tx.close()
        print("已发 %d 个 ICMP echo (载荷 %d B)" % (sent, a.ping_payload))

    deadline = time.time() + a.sec
    while time.time() < deadline:
        time.sleep(0.2)
    sniffer.stop()

    print("—— 抓到 %d 帧 ——" % len(frames))
    peer_ip = bytes(int(x) for x in a.peer.split("."))
    hits = 0
    shown_hex = 0
    for raw in frames:
        et = (raw[12] << 8) | raw[13]
        desc = "ethertype=0x%04X" % et
        if et == 0x0800 and len(raw) >= 34:
            ihl = (raw[14] & 0x0F) * 4
            proto = raw[23]
            srcip = raw[26:30]
            dstip = raw[30:34]
            ipck = "IP校验✔" if checksum_ok(raw[14:14 + 20]) else "IP校验✗"
            ttl = raw[22]
            ipid = struct.unpack("!H", raw[18:20])[0]
            flg = struct.unpack("!H", raw[20:22])[0]
            direction = "?" 
            if srcip == peer_ip:
                direction = "← 板子发来"
                hits += 1
            elif dstip == peer_ip:
                direction = "→ 我发出"
            else:
                continue
            desc = ("%s IPv4 ver/ihl=0x%02X proto=%d %s -> %s ttl=%d ipid=0x%04X flg=0x%04X totlen=%d %s len=%d"
                    % (direction, raw[14], proto, ".".join(map(str, srcip)), ".".join(map(str, dstip)),
                       ttl, ipid, flg, struct.unpack("!H", raw[16:18])[0], ipck, len(raw)))
            if proto == 1 and len(raw) >= 14 + ihl + 8:
                ic = raw[14 + ihl:]
                iptot = struct.unpack("!H", raw[16:18])[0]
                ic_真 = ic[:max(0, iptot - ihl)]
                icck = "ICMP校验✔" if checksum_ok(ic_真) else "ICMP校验✗(存=0x%04X 应为0x%04X)" % (
                    struct.unpack("!H", ic[2:4])[0], correct_cksum_of(ic_真))
                desc += "  | ICMP type=%d code=%d ident=0x%04X seq=%d icmpLen=%d %s" % (
                    ic[0], ic[1], struct.unpack("!H", ic[4:6])[0],
                    struct.unpack("!H", ic[6:8])[0], len(ic_真), icck)
                if a.dump and shown_hex < 8:
                    print("   hex: " + raw[:14 + ihl + 24].hex(" "))
                    shown_hex += 1
        elif et == 0x0806:
            desc = "ARP (op=%d)" % struct.unpack("!H", raw[20:22])[0]
        print("  " + desc)
    if hits == 0:
        print("=> 板子一个 IP 帧都没发 ⇒ 根本没走到应答分支")
    if a.forge_zero and hits:
        print("=> 判读: 若上面回包显示『ICMP校验✔』, 说明回包的 type/ident/seq/payload "
              "全对、头部也对, 缺陷只在『校验和字段』; 再跑一次默认 ping 看它是不是把"
              "正确值(非0)也写成了 0x0000 —— 是则『从不写该校验和』成立。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
