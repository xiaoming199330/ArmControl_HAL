# -*- coding: utf-8 -*-
"""
mock_server — **假臂 UDP 服务**：让没有真机的上位机也能联调。

为什么需要它
------------
`sim://` 只活在**当前 Python 进程**里 —— C# / C++ / Node / 组态软件的工程师
根本用不上。而"手上有协议文档但拿不到臂"是最常见的卡点：只能盲写代码，
等真机到了才发现 CRC 算错、字节序反了、超时设短了。

本服务把 `sim.py` 里那台**内存假臂**挂到一个真实 UDP 端口上（默认 5000），
于是任何语言写的上位机都能像连真机一样连它：

    # 终端 1：起假臂（占住 5000 口，与真机同端口 ⇒ 上位机的地址不用改）
    armctl mock --port 5000

    # 终端 2：你的上位机 / 或本包自带的最小客户端
    python host_integration/minimal_client.py --uri net://127.0.0.1:5000

行为保真度
----------
报文由**同一个** `SimArmTransport._handle()` 产生 —— 与 `sim://`、以及
`tests/` 里那 113 项离线断言走的是同一条代码路径，所以：

* ✅ 报文格式、命令码、错误码、UTF-8 分页、丢读哨兵、安全闸门、MAGIC 校验
  与固件**逐字节一致**（`gen_protocol_vectors.py` 的向量就是拿它比对的）
* ✅ 校验和错 ⇒ NACK(err=2)，与固件一致
* ⚠ **不是**真实物理仿真：没有动力学、没有力矩、没有舵机丢读；`MOVE_*` 只做
  按速度的线性插值 ⇒ 轨迹/时序类功能只能验"协议对不对"，不能验"效果好不好"
* ⚠ 真实臂可能**不回复**（丢包、主循环忙、E-Stop 冻结），假臂默认**必回**。
  要测上位机的超时/重试逻辑，用 `--loss 0.2` 或 `--drop-first N` 故意砸它

用法
----
    armctl mock                       # 0.0.0.0:5000，日志级别 1
    armctl mock --port 5000 -v        # 每收到一条就打印命令名（看上位机发了啥）
    armctl mock --loss 0.1            # 10% 丢包（测重试）
    armctl mock --latency-ms 30       # 每包延迟 30 ms（测上位机的卡顿容忍）
    armctl mock --estop               # 上电即 E-Stop 状态（测安全流程）
    armctl mock --fail-joints 2       # J3 恒读失败（测"读失败要如实上报"）
    armctl mock --selfcheck           # 起服务 → 自己打一发 PING/GET_STATUS → 退出

按 Ctrl-C 停。进程退出时端口自动释放。
"""

from __future__ import annotations

import argparse
import socket
import time
from typing import Optional

from .protocol import (RSP_NACK, ErrCode, ProtocolError, cmd_name, pack_logical,
                       unpack_logical)
from .sim import SimArmTransport

DEFAULT_PORT = 5000


class MockArmServer:
    """把 `SimArmTransport` 挂到 UDP 端口上。单线程事件循环（确定、好排查）。"""

    def __init__(self, host: str = "0.0.0.0", port: int = DEFAULT_PORT,
                 latency_ms: float = 0.0, verbose: bool = False,
                 drop_first: int = 0, log=None, **sim_kw):
        self.host = host
        self.port = int(port)
        self.latency_s = max(0.0, float(latency_ms)) / 1000.0
        self.verbose = verbose
        self.drop_first = max(0, int(drop_first))
        self._log = log or (lambda m: print(m, flush=True))

        self.sim = SimArmTransport(latency_s=0.0, **sim_kw)
        self.sim.open()

        self.sock: Optional[socket.socket] = None
        self._stop = False
        self.stats = {"rx": 0, "tx": 0, "dropped": 0, "bad": 0}

    # ------------------------------------------------------------------ 生命周期
    def bind(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 收包缓冲调大一点：上位机突发 50 Hz 时不容易被内核丢掉
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        s.bind((self.host, self.port))
        self.sock = s
        self._log("假臂服务已就绪：udp://%s:%d  (fw-tag=%s)"
                  % (self.host if self.host != "0.0.0.0" else "0.0.0.0",
                     self.port, self.sim.fw_tag))
        self._log("上位机请连  net://<本机IP>:%d   —— 报文格式与真机逐字节一致" % self.port)

    def handle_datagram(self, data: bytes) -> Optional[bytes]:
        """一个数据报 → 一个回包（或 None = 故意不回）。**纯函数式**，便于单测。"""
        try:
            seq, cmd, pl = unpack_logical(data)
        except ProtocolError as e:
            self.stats["bad"] += 1
            # 与固件一致：坏包若还能看出命令码就回 NACK(err=CRC)，否则静默丢
            if data and data[0] != 0x00 and data[0] < 0x80:
                self._log("  ✗ 坏包(%s) → NACK(CRC)" % e)
                return pack_logical(0, RSP_NACK, bytes([data[0], int(ErrCode.CRC)]))
            return None

        if self.drop_first > 0:
            self.drop_first -= 1
            self.stats["dropped"] += 1
            if self.verbose:
                self._log("  ⊘ 故意丢弃第 %d 包（--drop-first）" % self.stats["rx"])
            return None

        if self.sim.loss > 0 and self.sim._rand() < self.sim.loss:   # noqa: SLF001
            self.stats["dropped"] += 1
            return None

        resp = self.sim._handle(seq, cmd, pl)                        # noqa: SLF001
        if resp is None:
            if self.verbose:
                self._log("  · 0x%02X %s → 固件语义：不回（cmd>=0x80 或坏长度）"
                          % (cmd, cmd_name(cmd)))
            return None
        rcmd, rpl = resp
        if self.verbose:
            extra = ""
            if rcmd == RSP_NACK and rpl:
                try:
                    extra = "  err=%s" % ErrCode(rpl[1]).name
                except (ValueError, IndexError):
                    pass
            self._log("  0x%02X %-18s → 0x%02X %-14s %dB%s"
                      % (cmd, cmd_name(cmd), rcmd, cmd_name(rcmd), len(rpl), extra))
        return pack_logical(seq, rcmd, rpl)

    def serve_forever(self) -> int:
        if self.sock is None:
            self.bind()
        assert self.sock is not None
        self.sock.settimeout(0.5)
        self._stop = False
        try:
            while not self._stop:
                s = self.sock
                if s is None:
                    break                      # close() 已把 socket 摘掉
                try:
                    data, addr = s.recvfrom(2048)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    # ⚠ Windows 专有坑：UDP 收到 ICMP port-unreachable 时，
                    #   下一次 recvfrom 会抛 WSAECONNRESET(10054)，而不是超时。
                    #   上位机崩了/换了端口就会触发 —— 绝不能让它把服务打死。
                    continue
                except (OSError, AttributeError):
                    # 另一个线程正在 close()（WinError 10038「非套接字」）⇒ 正常收工，
                    # 不是错误。自检/单测里双线程同开同关一定会走到这里。
                    break
                except KeyboardInterrupt:
                    break
                self.stats["rx"] += 1
                out = self.handle_datagram(data)
                if out is None:
                    continue
                if self.latency_s > 0:
                    time.sleep(self.latency_s)
                try:
                    s.sendto(out, addr)
                except OSError:
                    continue                     # 对端已走，丢掉这次回包即可
                self.stats["tx"] += 1
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
        self._log("已停止：收 %d / 回 %d / 故意丢 %d / 坏包 %d"
                  % (self.stats["rx"], self.stats["tx"],
                     self.stats["dropped"], self.stats["bad"]))
        return 0

    def close(self) -> None:
        self._stop = True
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    # ------------------------------------------------------------------ 自查
    def selfcheck(self) -> int:
        """起服务 → 用同一个 socket 打一发 PING + GET_STATUS → 校验回包。"""
        self.bind()
        assert self.sock is not None
        self.sock.settimeout(1.0)
        fails = 0

        def rpc(cmd: int, pl: bytes = b"") -> Optional[bytes]:
            self.sock.sendto(pack_logical(1, cmd, pl), ("127.0.0.1", self.port))
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                return None
            return self.handle_datagram(data)

        print("  [1] PING …")
        r = rpc(0x01)
        ok = bool(r) and r[1] == 0x81 and len(r) >= 7
        print("      %s 回包 %s" % ("✓" if ok else "✗", r.hex() if r else "超时"))
        fails += 0 if ok else 1

        print("  [2] GET_STATUS …")
        r = rpc(0x02)
        ok = bool(r) and r[1] == 0x82 and len(r) >= 22
        print("      %s %d B（期望 22+）" % ("✓" if ok else "✗", len(r) if r else 0))
        fails += 0 if ok else 1

        # ⚠ 逻辑包布局是 SEQ(1) CMD(1) LEN(1) PAYLOAD(LEN) CRC(2)
        #   ⇒ 载荷从偏移 **3** 开始，不是 4（LEN 只有 1 字节，不是 u16）
        print("  [3] 坏 CRC ⇒ NACK(err=2) …")
        bad = bytearray(pack_logical(2, 0x02, b""))
        bad[-1] ^= 0xFF
        r = self.handle_datagram(bytes(bad))
        ok = bool(r) and r[1] == RSP_NACK and r[4] == int(ErrCode.CRC)
        print("      %s %s" % ("✓" if ok else "✗", r.hex() if r else "无回包"))
        fails += 0 if ok else 1

        print("  [4] cmd>=0x80 ⇒ 不回 …")
        r = self.handle_datagram(pack_logical(3, 0x82, b""))
        ok = r is None
        print("      %s %s" % ("✓" if ok else "✗", "无回包（正确）" if ok else "居然回了"))
        fails += 0 if ok else 1

        print("  [5] 文本透传 TEXT_EXEC …")
        import struct
        pl = struct.pack("<I", 0x5A5A5A5A) + b"h"
        r = rpc(0x0B, pl)
        ok = bool(r) and r[1] == 0x8B
        txt = ""
        if ok and len(r) > 9:
            txt = r[9:].decode("utf-8", "replace").strip()
            ok = "ArmControl" in txt or "固件" in txt or len(txt) > 40
        print("      %s 输出首行：%r" % ("✓" if ok else "✗", txt[:46]))
        fails += 0 if ok else 1

        self.close()
        print("")
        print("  mock_server 自检：5 项，%s" % ("全部通过" if not fails else
                                              "失败 %d" % fails))
        return 1 if fails else 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="python -m arm_sdk.mock_server",
        description="假臂 UDP 服务（无真机开发上位机用）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="⚠ 这是**协议级**假臂：报文与固件逐字节一致，但没有物理仿真。\n"
               "  轨迹/时序只能验协议，不能验效果。要测超时/重试请用 --loss / --drop-first。\n")
    p.add_argument("--host", default="0.0.0.0", help="默认 0.0.0.0（同网段都能连）")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help="默认 %d（与真机同端口，上位机地址不用改）" % DEFAULT_PORT)
    p.add_argument("--latency-ms", type=float, default=0.0,
                    help="每包回包前延迟（真机实测 p50 1.61 ms）")
    p.add_argument("--loss", type=float, default=0.0, help="丢包率 0~1")
    p.add_argument("--drop-first", type=int, default=0, help="开局先丢 N 包（测重试）")
    p.add_argument("--estop", action="store_true", help="上电即处于 E-Stop 状态")
    p.add_argument("--fail-joints", default="",
                   help="逗号分隔的轴号（0 基），如 2 或 0,3：这些轴恒读失败")
    p.add_argument("--unreachable", action="store_true",
                   help="MOVE_CART 一律回 err=3（IK 不可达）")
    p.add_argument("--fw-tag", default=None, help="改假臂报的固件号（测上位机版本判定）")
    p.add_argument("-v", "--verbose", action="store_true", help="打印每一条命令")
    p.add_argument("--selfcheck", action="store_true", help="自检 5 项后退出")
    a = p.parse_args(argv)

    fj = [int(x) for x in a.fail_joints.replace(",", " ").split() if x.strip()]
    srv = MockArmServer(host=a.host, port=a.port, latency_ms=a.latency_ms,
                        verbose=a.verbose, drop_first=a.drop_first,
                        loss=a.loss, estop=a.estop, fail_joints=fj,
                        unreachable=a.unreachable)
    if a.fw_tag:
        srv.sim.fw_tag = a.fw_tag

    if a.selfcheck:
        return srv.selfcheck()

    try:
        return srv.serve_forever()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
