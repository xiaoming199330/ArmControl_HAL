#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
icmp_probe.py —— 绕过 Windows `ping`，直接收发 ICMP Echo 的取证工具。

用途：当 `ping 192.168.7.120` 不通、而 UDP 命令正常时，用它区分下面四种情况：

  A. 板子收到了请求但回包没出来 / 回包校验和错   -> 能收到但内容不对
  B. 板子根本没收到（ARP 或空口问题）            -> 完全收不到
  C. PC 侧防火墙把回包吞了                       -> 本工具收到但 `ping` 不通
  D. 一切正常，只是 Windows ping 的行为差异      -> 本工具收到且校验正确

⚠ 本工具**看不到校验和错误的回包**（Windows IP 层会先丢掉它们）。
   "板子到底发没发、发的是什么"要用 `tools/net_capture.py`（Npcap 抓帧）。

注意：Windows 上创建原始 ICMP socket 需要管理员权限。

用法:
  python tools/icmp_probe.py                     # 默认 192.168.7.120，32 字节载荷
  python tools/icmp_probe.py --dst 192.168.7.120
  python tools/icmp_probe.py --sweep             # 依次测 0/32/56/1000/1472 字节载荷
  python tools/icmp_probe.py --count 5           # 每个长度发 5 次
"""
import argparse
import socket
import struct
import sys
import time


def ones_complement_sum(buf: bytes) -> int:
    if len(buf) & 1:
        buf = buf + b"\x00"
    s = 0
    for i in range(0, len(buf), 2):
        s += (buf[i] << 8) | buf[i + 1]
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def build_echo(ident: int, seq: int, payload_len: int) -> bytes:
    payload = bytes((0x61 + i % 26) for i in range(payload_len))
    head = struct.pack("!BBHHH", 8, 0, 0, ident, seq)
    icmp = head + payload
    cksum = ones_complement_sum(icmp)
    return struct.pack("!BBHHH", 8, 0, cksum, ident, seq) + payload


def verify_ip_header(ip: bytes) -> str:
    if ones_complement_sum(ip[:20]) == 0:
        return "IP校验✔"
    return "IP校验✗(值=0x%04X)" % struct.unpack("!H", ip[10:12])[0]


def verify_icmp(icmp: bytes) -> str:
    if ones_complement_sum(icmp) == 0:
        return "ICMP校验✔"
    return "ICMP校验✗"


def probe(dst: str, payload_len: int, count: int, ident: int, timeout: float) -> dict:
    """发 count 次 echo request，聚合统计。返回 {'sent','got','good','bad','notes'}"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    sock.settimeout(timeout)
    sent = got = good = 0
    notes = []
    try:
        for n in range(count):
            seq = (n + 1) & 0xFFFF
            sent += 1
            try:
                sock.sendto(build_echo(ident, seq, payload_len), (dst, 1))
            except OSError as e:
                notes.append("sendto 失败: %s" % e)
                continue
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    break
                if not data or (data[0] >> 4) != 4:
                    continue
                ihl = (data[0] & 0x0F) * 4
                if len(data) < ihl + 8:
                    continue
                icmp = data[ihl:]
                if icmp[0] != 0:                     # 只要 echo reply
                    notes.append("收到其它 ICMP: type=%d code=%d from %s" % (icmp[0], icmp[1], addr[0]))
                    continue
                rid, rseq = struct.unpack("!HH", icmp[4:8])
                if rid != ident or rseq != seq:
                    continue                          # 不是我这发
                got += 1
                ip_ok = ones_complement_sum(data[:20]) == 0
                icmp_ok = ones_complement_sum(icmp) == 0
                body_ok = icmp[8:] == bytes((0x61 + i % 26) for i in range(payload_len))
                if ip_ok and icmp_ok and body_ok:
                    good += 1
                else:
                    notes.append("seq=%d %s %s 数据%s len=%d (期望载荷 %d)"
                                 % (seq, verify_ip_header(data[:20]), verify_icmp(icmp),
                                    "✔" if body_ok else "✗(len=%d)" % len(icmp[8:]), len(icmp), payload_len))
                break
    finally:
        sock.close()
    return {"sent": sent, "got": got, "good": good, "notes": notes}


def craft_compatible_ip_id(src_ip: str, dst_ip: str, ident: int, payload_len: int) -> int:
    """找一个 IP `ID`，使得"校验和字段=0 时算出来的正确校验和恰好=0x0000"。

    这样这份 IP 头是**合法的**（校验和字段填 0x0000 就是它的正确值），
    同时"把旧校验和带进求和"这个 bug 也不会污染结果——因为那个旧值本身就是 0。
    """
    total_len = 20 + 8 + payload_len
    src = socket.inet_aton(src_ip)
    dst = socket.inet_aton(dst_ip)
    for ip_id in range(0x10000):
        hdr = struct.pack("!BBHHHBBH4s4s",
                          0x45, 0, total_len, ip_id, 0, 64, 1, 0, src, dst)
        if ones_complement_sum(hdr) == 0:
            return ip_id
    return -1


def legacy_probe(dst: str, payload_len: int, ident: int, timeout: float) -> dict:
    """区分"A. 板子不回 ICMP" 与 "B. 板子回了但校验和错（旧固件）"。

    关键：0920N 及更早的固件在算校验和前**没有把校验和字段清零**。
    于是回包里的校验和 = ~(正确求和 + 请求里的旧校验和)。
    只要让请求里的**两个校验和字段都是 0x0000**（且 IP 头依然合法），
    旧固件算出来的回包也**恰好是正确的** ⇒ 就能收到它。

    结论判定：
      · 正常包收到        -> 板子是 0920O+（校验和已修），问题在别处
      · 正常包收不到、特制包收到 -> **板子是旧固件（校验和 bug 仍在）** ⇒ 烧新固件
      · 两种都收不到      -> 板子根本没走到 ICMP 应答分支
    """
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.connect((dst, 5000))
    src_ip = u.getsockname()[0]
    u.close()

    total_len = 20 + 8 + payload_len
    src = socket.inet_aton(src_ip)
    dst_b = socket.inet_aton(dst)
    ip_id = craft_compatible_ip_id(src_ip, dst, ident, payload_len)
    if ip_id < 0:
        raise SystemExit("没找到校验和为 0 的 ID（不该发生）")
    payload = bytes((0x61 + i % 26) for i in range(payload_len))

    def ip_hdr(raw_id: int, cksum: int) -> bytes:
        return struct.pack("!BBHHHBBH4s4s", 0x45, 0, total_len,
                           raw_id, 0, 64, 1, cksum, src, dst_b)

    icmp_normal = build_echo(ident, 1, payload_len)
    # legacy: ICMP 校验和字段写 0x0000（请求侧不看它, 非法也照发）
    icmp_legacy = struct.pack("!BBHHH", 8, 0, 0x0000, ident, 2) + payload

    # 正常包: 校验和按规矩算（非零）
    hdr_normal = ip_hdr(0x1234, 0)
    hdr_normal = ip_hdr(0x1234, ones_complement_sum(hdr_normal))
    # 特制包: ID 选得让"校验和 = 0x0000"本身就是正确值
    hdr_legacy = ip_hdr(ip_id, 0x0000)

    tx = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_RAW)
    try:
        tx.setsockopt(socket.IPPROTO_IP, getattr(socket, "IP_HDRINCL", 2), 1)
    except OSError:
        pass                                   # Windows 上 IPPROTO_RAW 默认就含 IP 头
    rx = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    rx.settimeout(timeout)

    res = {}
    try:
        for name, frame in (("normal", hdr_normal + icmp_normal),
                            ("legacy", hdr_legacy + icmp_legacy)):
            got = 0
            detail = ""
            for _ in range(3):
                try:
                    tx.sendto(frame, (dst, 1))
                except OSError as e:
                    print("  · 发送失败(%s): %s" % (name, e))
                    break
                deadline = time.time() + timeout
                while time.time() < deadline:
                    try:
                        data, _addr = rx.recvfrom(4096)
                    except socket.timeout:
                        break
                    if not data or (data[0] >> 4) != 4 or data[9] != 1:
                        continue
                    ihl = (data[0] & 0x0F) * 4
                    icmp = data[ihl:]
                    if icmp[0] != 0:
                        continue
                    got += 1
                    detail = ("回包: IP%s ICMP%s 载荷%dB (我发 %dB)" % (
                        "✔" if ones_complement_sum(data[:20]) == 0 else "✗",
                        "✔" if ones_complement_sum(icmp) == 0 else "✗",
                        max(0, len(icmp) - 8), payload_len))
                    break
            res[name] = got
            print("  · %-7s 包: 收到 %d/3 个回包  %s" % (name, got, detail))
    finally:
        tx.close()
        rx.close()
    print("  legacy 包 IP ID = 0x%04X (其正确校验和恰为 0x0000)" % ip_id)
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dst", default="192.168.7.120")
    ap.add_argument("--count", type=int, default=3, help="每个长度发几次 (默认 3)")
    ap.add_argument("--timeout", type=float, default=1.5)
    ap.add_argument("--ident", type=int, default=0x1234)
    ap.add_argument("--sweep", action="store_true", help="依次测 0/32/56/1000/1472 字节载荷")
    ap.add_argument("--payload", type=int, default=32)
    ap.add_argument("--legacy", action="store_true",
                    help="裁定[板子不回 ICMP]还是[板子回了但校验和错/旧固件]，需管理员")
    a = ap.parse_args()

    # 先确认目标在 ARP 表里（能解析说明二层通）
    try:
        cp = __import__("subprocess").run(["arp", "-a", a.dst], capture_output=True, timeout=5)
        arp_txt = (cp.stdout or b"").decode("utf-8", "replace").strip()
        print("ARP: %s" % (arp_txt.replace("\n", " | ") if arp_txt else "(无条目)"))
    except Exception as e:
        print("ARP 查询失败:", e)

    if a.legacy:
        r = legacy_probe(a.dst, a.payload, a.ident, max(a.timeout, 1.5))
        print()
        if r["normal"] and r["legacy"]:
            print("=> 结论: 正常包也有回包 ⇒ 固件校验和已修（0920O+），问题在别处")
        elif r["legacy"] and not r["normal"]:
            print("=> 结论: ⚠ 特制包有回包、正常包没有 ⇒ **板上是旧固件**"
                  "（算校验和时没清校验和字段）⇒ 烧 0920P 即可")
        elif r["normal"] and not r["legacy"]:
            print("=> 结论: 只有正常包回、特制包不回 ⇒ 罕见, 把两组原始输出留下")
        else:
            print("=> 结论: 本工具一个回包都收不到。⚠ 这不等于[板子没发] —— "
                  "系统的 IP 层会**静默丢弃校验和错误**的 ICMP，"
                  "只有 `tools/net_capture.py`（Npcap 抓帧）才能看到真实回包。")
        return 0 if (r["normal"] or r["legacy"]) else 1

    sizes = [0, 32, 56, 1000, 1472] if a.sweep else [a.payload]
    allgood = True
    for size in sizes:
        r = probe(a.dst, size, a.count, a.ident, a.timeout)
        verdict = "✅ 正常" if r["good"] == r["sent"] else ("⚠ 部分" if r["got"] else "❌ 无回包")
        if r["good"] != r["sent"]:
            allgood = False
        print("载荷 %-5d  发出 %d  收到 %d  校验全对 %d   -> %s" % (size, r["sent"], r["got"], r["good"], verdict))
        for n in r["notes"][:6]:
            print("      ·", n)
    print("—— 结论:", "ICMP 通道完全正常" if allgood else "看上面哪一档开始出问题（红线 61/62）")
    return 0 if allgood else 1


if __name__ == "__main__":
    sys.exit(main())
