#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""udp_vs_http_bench.py — 在同一台臂上，把 **UDP 直连** 和 **HTTP 边车** 的
延迟 / 速率 / 丢包 放在一张表里量出来。

为什么要专门做这个
------------------
"UDP 是不是容易丢包？换成 HTTP 会不会更稳？" —— 这个问题**不能靠感觉回答**，
因为三个数会被混在一起：

  ① **链路延迟**（PC 网卡 → 臂 → PC）：这是真正的物理指标，跟协议无关。
  ② **协议开销**（UDP 无连接 vs HTTP 建连 + 头部 + JSON）：这一层是 HTTP 多付的。
  ③ **可靠性语义**（UDP 丢一帧就是丢一条命令，无重传；TCP 会重传但你不知道
     它重传了多久）：这一层不是"快慢"，是"**错了会不会告诉你**"。

本工具把 ①② 分开测 —— 靠的是边车在每个响应里带的 `X-Arm-Rtt-Ms` 头：

    链路 RTT  = X-Arm-Rtt-Ms              （边车→臂 的那一跳）
    HTTP 总值 = 整个 HTTP 请求的墙钟        （含 loopback TCP + JSON）
    协议开销  = HTTP 总值 − 链路 RTT        ← 这才是"用 HTTP 要多付多少"

丢包怎么测（关键）
------------------
本协议**没有传输层 ACK**。PC 侧只能看到"超时"，但超时 = 三种原因的并集：
  (a) 请求包在路上丢了
  (b) 臂收到了、回包丢了
  (c) 臂收下了但还没处理（队列/主循环忙）

要把 (a)(c) 和 (b) 分开，必须看**板侧计数**：臂的 `nx` 打印 `udp cmd=` 累计值。
做法：扫频前后各读一次 `nx`，做差 = 臂真正收到的命令数。

    PC 发出 N 条  →  臂侧 Δudp_cmd = M
    单向丢包率 = (N − M) / N      ← 不管回包丢没丢，这个是干净的

所以：
  · `timeout_rate`（PC 侧看到超时的比例）= (a)∪(b)∪(c)
  · `board_loss`（板侧没收到）= (a)∪(c) 中"根本没到板子"的那部分
  · 两者一对比就能定位"A 是不是都在回程丢"

形态
----
    udp            直连臂，一条请求一条应答（当前 SDK 的用法）
    http-close     每条请求**新建** TCP 连接（naive：`requests.get()` 不建 Session）
    http-keepalive 复用一条 TCP 连接（正确用法；也是 `requests.Session` 的行为）

    ⚠ http-close 与 http-keepalive 的差距本身就是结论之一（Windows 上 loopback
      建连约 2~5 ms，比整条真实链路还贵）。

模式
----
    rtt     顺序请求-应答 N 次 → 延迟分位 + 超时率
    sweep   速率阶梯（50/100/200/400/800/1200 fps）→ 找到"开始丢"的拐点
    burst   连续发送不等回包 K 条 → 臂到底收下几条（这只有 UDP 能做）

用法
----
    # 无硬件：假臂 + 边车，先把方法跑通
    python tools/udp_vs_http_bench.py --selftest

    # 真机
    python tools/udp_vs_http_bench.py --mode rtt   --target-ip 192.168.7.120 --n 500
    python tools/udp_vs_http_bench.py --mode sweep --target-ip 192.168.7.120
    python tools/udp_vs_http_bench.py --mode burst --target-ip 192.168.7.120 --burst 200

    # 只跑其中几种形态
    python tools/udp_vs_http_bench.py --mode rtt --forms udp,http-keepalive
"""
from __future__ import annotations

import argparse
import http.client
import json
import os
import socket
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
SDK = os.path.join(PROJ, "sdk_package")
if SDK not in sys.path:
    sys.path.insert(0, SDK)

from arm_sdk.http_bridge import serve as bridge_serve          # noqa: E402
from arm_sdk.client import ArmClient, ArmCommandError, open_arm  # noqa: E402
from arm_sdk.protocol import CMD_PING, CMD_TEXT_EXEC, payload_text_exec  # noqa: E402

PROBE_CMD = CMD_PING            # 只读、永远有应答、不动任何东西
FORMS_ALL = ("udp", "http-close", "http-keepalive")
RATES_DEFAULT = (50, 100, 200, 400, 800, 1200)


# ============================================================================
# 小工具
# ============================================================================
def pct(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def _fmt(v: Optional[float], unit: str = "ms", nd: int = 2) -> str:
    return "—" if v is None else ("%.*f%s" % (nd, v, unit))


# ============================================================================
# 结果容器
# ============================================================================
@dataclass
class Stat:
    name: str
    n_sent: int = 0
    n_ok: int = 0
    n_timeout: int = 0
    n_err: int = 0
    wall_s: float = 0.0
    lat: list[float] = field(default_factory=list)        # 端到端观测延迟
    arm_lat: list[float] = field(default_factory=list)    # 链路 RTT（HTTP 才有）
    board_delta: Optional[int] = None                     # 臂侧 Δudp_cmd
    note: str = ""

    @property
    def achieved_fps(self) -> float:
        return self.n_sent / self.wall_s if self.wall_s > 0 else 0.0

    @property
    def timeout_rate(self) -> float:
        return (self.n_timeout / self.n_sent) if self.n_sent else 0.0

    @property
    def error_rate(self) -> float:
        return (self.n_err / self.n_sent) if self.n_sent else 0.0

    @property
    def board_loss(self) -> Optional[float]:
        if self.board_delta is None or self.n_sent == 0:
            return None
        lost = self.n_sent - self.board_delta
        return max(0, lost) / self.n_sent

    def latency(self) -> dict[str, float]:
        s = sorted(self.lat)
        return {"min": s[0] if s else 0.0, "p50": pct(s, .5), "p90": pct(s, .9),
                "p99": pct(s, .99), "max": s[-1] if s else 0.0,
                "mean": statistics.fmean(s) if s else 0.0}

    def overhead(self) -> Optional[float]:
        """HTTP 相对链路的额外开销（p50）。UDP 形态没有 arm_lat ⇒ None。"""
        if not self.arm_lat:
            return None
        a, h = sorted(self.arm_lat), sorted(self.lat)
        return pct(h, .5) - pct(a, .5)

    def as_dict(self) -> dict[str, Any]:
        d = {
            "form": self.name,
            "sent": self.n_sent, "ok": self.n_ok,
            "timeout": self.n_timeout, "error": self.n_err,
            "timeout_rate": round(self.timeout_rate, 6),
            "board_loss": None if self.board_loss is None else round(self.board_loss, 6),
            "board_delta": self.board_delta,
            "wall_s": round(self.wall_s, 4),
            "achieved_fps": round(self.achieved_fps, 2),
            "latency_ms": {k: round(v, 3) for k, v in self.latency().items()},
            "overhead_p50_ms": None if self.overhead() is None else round(self.overhead(), 3),
        }
        if self.arm_lat:
            s = sorted(self.arm_lat)
            d["link_rtt_ms"] = {k: round(v, 3) for k, v in
                                zip(("min", "p50", "p90", "p99", "max"),
                                    (s[0], pct(s, .5), pct(s, .9), pct(s, .99), s[-1]))}
        if self.note:
            d["note"] = self.note
        return d


# ============================================================================
# 探测通道
# ============================================================================
class Probe:
    """一次"逻辑 PING 往返"。三种形态对外接口一致。"""

    name = "?"

    def open(self) -> None: ...
    def close(self) -> None: ...
    def ping(self, timeout: float) -> tuple[bool, float, Optional[float], str]:
        """返回 (成功, 端到端毫秒, 链路RTT毫秒或None, 说明)"""
        raise NotImplementedError

    def send_only(self, n: int) -> None:
        raise NotImplementedError


class UdpProbe(Probe):
    name = "udp"

    def __init__(self, uri: str, timeout: float = 0.3) -> None:
        self.uri = uri
        self.timeout = timeout
        self.arm: Optional[ArmClient] = None

    def open(self) -> None:
        self.arm = open_arm(self.uri, timeout=self.timeout)
        self.arm.open()

    def close(self) -> None:
        if self.arm:
            self.arm.close()

    def ping(self, timeout: float) -> tuple[bool, float, Optional[float], str]:
        assert self.arm
        t0 = time.perf_counter()
        try:
            r = self.arm.request(PROBE_CMD, b"", timeout=timeout)
        except Exception as e:                                   # noqa: BLE001
            return False, (time.perf_counter() - t0) * 1e3, None, type(e).__name__
        dt = (time.perf_counter() - t0) * 1e3
        if r.timeout:
            return False, dt, None, "timeout"
        return bool(r.ok), dt, None, "" if r.ok else r.err_name

    def send_only(self, n: int) -> None:
        assert self.arm
        for _ in range(n):
            self.arm.send_async(PROBE_CMD, b"")


class HttpProbe(Probe):
    """HTTP 边车。keepalive=False ⇒ 每条请求新建 TCP 连接。"""

    def __init__(self, host: str, port: int, keepalive: bool, timeout: float = 5.0) -> None:
        self.host, self.port = host, port
        self.keepalive = keepalive
        self.timeout = timeout
        self.name = "http-keepalive" if keepalive else "http-close"
        self.conn: Optional[http.client.HTTPConnection] = None

    def open(self) -> None:
        if self.keepalive:
            self.conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)

    def close(self) -> None:
        if self.conn:
            try:
                self.conn.close()
            except Exception:                                    # noqa: BLE001
                pass
            self.conn = None

    def _one(self) -> tuple[int, dict, float]:
        body = json.dumps({"cmd": PROBE_CMD, "payload_hex": ""})
        hdr = {"Content-Type": "application/json",
               "Connection": "keep-alive" if self.keepalive else "close"}
        if self.keepalive:
            assert self.conn
            try:
                self.conn.request("POST", "/api/raw", body, hdr)
                resp = self.conn.getresponse()
                data = resp.read()
            except Exception:
                self.conn.close()
                self.conn = http.client.HTTPConnection(self.host, self.port,
                                                       timeout=self.timeout)
                self.conn.request("POST", "/api/raw", body, hdr)
                resp = self.conn.getresponse()
                data = resp.read()
            arm_rtt = float(resp.getheader("X-Arm-Rtt-Ms") or 0.0)
            return resp.status, json.loads(data.decode("utf-8")), arm_rtt
        c = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
        try:
            c.request("POST", "/api/raw", body, hdr)
            resp = c.getresponse()
            data = resp.read()
            arm_rtt = float(resp.getheader("X-Arm-Rtt-Ms") or 0.0)
            return resp.status, json.loads(data.decode("utf-8")), arm_rtt
        finally:
            c.close()

    def ping(self, timeout: float) -> tuple[bool, float, Optional[float], str]:
        t0 = time.perf_counter()
        try:
            st, obj, arm_rtt = self._one()
        except Exception as e:                                   # noqa: BLE001
            return False, (time.perf_counter() - t0) * 1e3, None, type(e).__name__
        dt = (time.perf_counter() - t0) * 1e3
        if st != 200:
            return False, dt, arm_rtt or None, "http=%d/%s" % (st, obj.get("error"))
        return bool(obj.get("ok")), dt, (arm_rtt or None), ("" if obj.get("ok") else obj.get("err_name", "?"))


class HttpStreamProbe(Probe):
    """HTTP 版的"只发不等"：POST /api/stream（边车只发 UDP，不等 ACK）。"""

    name = "http-stream"

    def __init__(self, host: str, port: int, keepalive: bool = True, timeout: float = 5.0) -> None:
        self.inner = HttpProbe(host, port, keepalive, timeout)

    def open(self) -> None:
        self.inner.open()

    def close(self) -> None:
        self.inner.close()

    def ping(self, timeout: float) -> tuple[bool, float, Optional[float], str]:
        return self.inner._stream_one() if hasattr(self.inner, "_stream_one") else self.inner.ping(timeout)

    def send_only(self, n: int) -> None:
        raise NotImplementedError


# ============================================================================
# 基准主体
# ============================================================================
class Bench:
    def __init__(self, target_ip: str, target_port: int, bridge_port: int = 0,
                 verbose: bool = True, sim_mode: bool = False) -> None:
        self.ip, self.port = target_ip, target_port
        self.uri = "net://%s:%d" % (target_ip, target_port)
        self.bridge_port = bridge_port
        self.verbose = verbose
        self.sim_mode = sim_mode
        self._srv = None
        self._thr: Optional[threading.Thread] = None

    # ---- 资源 ----
    def start_bridge(self) -> int:
        self._srv = bridge_serve(self.uri, "127.0.0.1", self.bridge_port, timeout=0.5)
        p = self._srv.server_address[1]
        self._thr = threading.Thread(target=self._srv.serve_forever, daemon=True)
        self._thr.start()
        time.sleep(0.1)
        return p

    def stop_bridge(self) -> None:
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
            self._srv.bridge.close()
            self._srv = None

    def make(self, form: str) -> Probe:
        if form == "udp":
            return UdpProbe(self.uri)
        if form in ("http-close", "http-keepalive"):
            return HttpProbe("127.0.0.1", self.bridge_port, form == "http-keepalive")
        raise ValueError("未知形态 %r（可选 %s）" % (form, "/".join(FORMS_ALL)))

    # ---- 板侧计数 ----
    def read_nx(self) -> Optional[int]:
        """读臂侧 udp_cmd 累计值。假臂没有 `nx` ⇒ 返回 None（不假装测到了）。

        ⚠⚠ 必须用 `text_exec`（**自动翻页**），不能用 `text_out`（只取首页）：
           `nx` 全文 1311 B ≈ 23 页 × 58 B，`udp cmd=` 在靠后的页里，
           只取首页永远匹配不到 ⇒ 会静默退化成"读不到板侧计数"，
           而这一步正是**唯一**能测出单向真丢包的手段。
           2026-09-24 在真机 192.168.7.120 上踩过一次（表现为"假臂无 nx"）。
        """
        import re
        try:
            arm = open_arm(self.uri, timeout=1.0)
            arm.open()
            try:
                text = arm.text_exec("nx")
            finally:
                arm.close()
        except Exception as e:                                   # noqa: BLE001
            self._nx_warn("读板侧计数失败：%s: %s" % (type(e).__name__, e))
            return None
        m = re.search(r"udp\s*cmd\s*=\s*(\d+)", text)
        if m is None:
            self._nx_warn("`nx` 有回（%d B）但找不到 `udp cmd=` 字段，开头是 %r"
                          % (len(text), text[:70]))
        return int(m.group(1)) if m else None

    def _nx_warn(self, msg: str) -> None:
        """只在第一次失败时提醒，避免扫频每档刷屏（但**绝不静默**）。"""
        if getattr(self, "_nx_warned", False):
            return
        self._nx_warned = True
        if self.verbose:
            print("  ⚠ %s" % msg)

    # ---- 模式 1：顺序 RTT ----
    def mode_rtt(self, forms: list[str], n: int, warmup: int, timeout: float,
                 gap: float = 0.0) -> list[Stat]:
        out = []
        for form in forms:
            p = self.make(form)
            p.open()
            try:
                for _ in range(warmup):
                    p.ping(timeout)
                st = Stat(form)
                t0 = time.perf_counter()
                for i in range(n):
                    ok, dt, arm_rtt, why = p.ping(timeout)
                    st.n_sent += 1
                    if ok:
                        st.n_ok += 1
                        st.lat.append(dt)
                        if arm_rtt:
                            st.arm_lat.append(arm_rtt)
                    elif why == "timeout":
                        st.n_timeout += 1
                        st.lat.append(dt)
                    else:
                        st.n_err += 1
                        st.lat.append(dt)
                        if st.n_err <= 3:
                            st.note = (st.note + " %s" % why).strip()
                    if gap:
                        time.sleep(gap)
                st.wall_s = time.perf_counter() - t0
            finally:
                p.close()
            out.append(st)
        return out

    # ---- 模式 2：速率阶梯（找丢包拐点）----
    def mode_sweep(self, forms: list[str], rates: tuple[int, ...], sec: float,
                   timeout: float, with_board: bool = True) -> list[Stat]:
        out = []
        for form in forms:
            p = self.make(form)
            p.open()
            try:
                for rate in rates:
                    n = max(1, int(rate * sec))
                    iv = 1.0 / rate
                    st = Stat("%s@%dfps" % (form, rate))
                    b0 = self.read_nx() if with_board else None
                    t0 = time.perf_counter()
                    nxt = t0
                    for _ in range(n):
                        nxt += iv
                        # 绝对时间轴：避免"处理慢了就自动降频"把拐点抹掉
                        slack = nxt - time.perf_counter()
                        if slack > 0:
                            time.sleep(slack)
                        ok, dt, arm_rtt, why = p.ping(timeout)
                        st.n_sent += 1
                        if ok:
                            st.n_ok += 1
                            if arm_rtt:
                                st.arm_lat.append(arm_rtt)
                        elif why == "timeout":
                            st.n_timeout += 1
                        else:
                            st.n_err += 1
                        st.lat.append(dt)
                    st.wall_s = time.perf_counter() - t0
                    if with_board:
                        b1 = self.read_nx()
                        if b0 is not None and b1 is not None:
                            st.board_delta = b1 - b0
                    out.append(st)
                    if self.verbose:
                        bl = st.board_loss
                        print("    %-26s 实际 %7.1f fps | 超时 %5.1f%% | 板侧丢 %s"
                              % (st.name, st.achieved_fps, st.timeout_rate * 100,
                                 "—" if bl is None else "%.1f%%" % (bl * 100)))
            finally:
                p.close()
        return out

    # ---- 模式 3：突发（只发不等，只有 UDP 能做）----
    def mode_burst(self, burst: int, timeout: float, with_board: bool = True) -> Stat:
        """连发 burst 条 PING 不等回包 ⇒ 看臂收下几条。

        ⚠ 这是**唯一**能压出"臂侧收包能力"上限的测法：顺序发的时候，
        RTT 本身就是限速器（50 fps 封顶），根本压不到臂的极限。
        """
        st = Stat("udp-burst%d" % burst)
        arm = open_arm(self.uri, timeout=timeout)
        arm.open()
        try:
            b0 = self.read_nx() if with_board else None
            t0 = time.perf_counter()
            for _ in range(burst):
                arm.send_async(PROBE_CMD, b"")
            send_done = time.perf_counter()
            # 收尾：等回包把 socket 排空（不计数，只为让 kernel 缓冲排掉）
            time.sleep(0.3)
            try:
                for _ in range(burst):
                    arm.request(PROBE_CMD, b"", timeout=0.02)
            except Exception:                                    # noqa: BLE001
                pass
            st.wall_s = send_done - t0
            st.n_sent = burst
            if with_board:
                b1 = self.read_nx()
                if b0 is not None and b1 is not None:
                    st.board_delta = b1 - b0
            st.n_ok = st.board_delta if st.board_delta is not None else 0
            if st.board_delta is None:
                st.note = "假臂无 nx，板侧丢包测不到（只报了发送耗时）"
        finally:
            arm.close()
        return st


# ============================================================================
# 报告
# ============================================================================
def print_rtt_table(stats: list[Stat]) -> None:
    print("")
    print("=" * 108)
    print("  顺序请求-应答（PING 往返）")
    print("=" * 108)
    hdr = ("形态", "样本", "成功", "超时", "p50", "p90", "p99", "max", "链路RTT p50", "HTTP开销 p50")
    print("  %-16s %6s %6s %6s %8s %8s %8s %8s %12s %12s" % hdr)
    print("  " + "-" * 104)
    for st in stats:
        L = st.latency()
        link = "—"
        if st.arm_lat:
            link = _fmt(pct(sorted(st.arm_lat), .5))
        ov = st.overhead()
        print("  %-16s %6d %6d %6d %8s %8s %8s %8s %12s %12s"
              % (st.name, st.n_sent, st.n_ok, st.n_timeout,
                 _fmt(L["p50"]), _fmt(L["p90"]), _fmt(L["p99"]), _fmt(L["max"]),
                 link, "—" if ov is None else _fmt(ov)))
    print("  " + "-" * 104)
    print("  读法：")
    print("    · 链路RTT   = 边车到臂那一跳（与协议无关的物理指标）")
    print("    · HTTP开销  = 端到端 p50 − 链路 RTT p50 ⇒ **这才是换成 HTTP 多付的**")
    print("    · 超时率不为 0 才叫丢包；超时=0 时 p99/max 才是你该看的数")


def print_sweep_table(stats: list[Stat]) -> None:
    print("")
    print("=" * 108)
    print("  速率阶梯（每档固定时长，绝对时间轴 ⇒ 不会因为处理慢就自动降频）")
    print("=" * 108)
    print("  %-24s %10s %10s %9s %9s %10s %10s"
          % ("形态@目标", "实测fps", "成功", "超时率", "错误率", "板侧丢包", "p99"))
    print("  " + "-" * 104)
    for st in stats:
        bl = st.board_loss
        print("  %-24s %10.1f %10d %8.1f%% %8.1f%% %10s %9s"
              % (st.name, st.achieved_fps, st.n_ok,
                 st.timeout_rate * 100, st.error_rate * 100,
                 "—" if bl is None else "%.1f%%" % (bl * 100),
                 _fmt(st.latency()["p99"])))
    print("  " + "-" * 104)
    print("  读法：")
    print("    · 实测fps 追不上目标 ⇒ 该形态**吞吐到顶**了（HTTP 常见，RTT 就是限速器）")
    print("    · 板侧丢包 = (PC发出 − 臂侧Δudp_cmd)/PC发出 ⇒ **不掺回程丢包**，最干净")
    print("    · 板侧丢包 >0 而超时率更大 ⇒ 差额是回程丢的 / 或臂没排上队")
    print("    · 两者都为 0 ⇒ 该速率下这条链路是干净的")


def print_burst_table(st: Stat) -> None:
    print("")
    print("=" * 108)
    print("  突发（连发不等回包）—— 只有 UDP 能做，用来压出臂侧收包能力")
    print("=" * 108)
    print("  连发 %d 条 → %.3f s 发完（%.1f fps 瞬时）" % (st.n_sent, st.wall_s, st.achieved_fps))
    if st.board_delta is None:
        print("  臂侧 Δudp_cmd = 测不到（%s）" % st.note)
    else:
        print("  臂侧 Δudp_cmd = %d / %d  ⇒ 丢 %.1f%%"
              % (st.board_delta, st.n_sent, (st.board_loss or 0) * 100))
        print("")
        print("  读法：这个数**明显低于 0** 才说明臂侧吃力；接近 0 丢包说明")
        print("        连 1200+ fps 的突发它都收得下 ⇒ 50 Hz 遥操根本不在话下。")


def print_verdict(stats: list[Stat]) -> None:
    by = {}
    for st in stats:
        base = st.name.split("@")[0]
        by.setdefault(base, []).append(st)
    print("")
    print("=" * 108)
    print("  结论速览")
    print("=" * 108)
    u = by.get("udp")
    hk = by.get("http-keepalive")
    hc = by.get("http-close")
    if u:
        s = u[0]
        print("  UDP 直连        : p50 %s / 超时 %.1f%% / 链路RTT p50 %s"
              % (_fmt(s.latency()["p50"]), s.timeout_rate * 100, _fmt(pct(sorted(s.arm_lat), .5)) if s.arm_lat else "—"))
    if hc:
        s = hc[0]
        print("  HTTP(每次新建)   : p50 %s / 超时 %.1f%% / 额外开销 %s"
              % (_fmt(s.latency()["p50"]), s.timeout_rate * 100, _fmt(s.overhead())))
    if hk:
        s = hk[0]
        print("  HTTP(keep-alive): p50 %s / 超时 %.1f%% / 额外开销 %s"
              % (_fmt(s.latency()["p50"]), s.timeout_rate * 100, _fmt(s.overhead())))
    if hc and hk:
        d = hc[0].latency()["p50"] - hk[0].latency()["p50"]
        print("")
        print("  ⇒ 光把连接复用起来就省 %s/次（%.0f%%）。**HTTP 客户端一定要复用连接。**"
              % (_fmt(d), 100.0 * d / max(hc[0].latency()["p50"], 1e-9)))
    if u and hk:
        print("  ⇒ HTTP 比 UDP 每次多付 %s（这是 loopback + TCP + JSON 的钱）。"
              % _fmt(hk[0].latency()["p50"] - u[0].latency()["p50"]))
    print("")
    print("  ⚠ 这些数只是**延迟**。选型还要看语义：")
    print("     UDP  : 丢一帧 = 丢一条命令，**没人告诉你**。查询无所谓；运动/写参数要自己补 SEQ+超时+重试。")
    print("     HTTP : TCP 会重传 ⇒ 不丢命令，但**你不知道它卡了多久** ⇒ 不适合 50 Hz 遥操。")
    print("     结论 : 遥操流(50Hz)走 UDP；配置/查询/写参数走 HTTP 边车。两者并存，不冲突。")
    print("=" * 108)


# ============================================================================
# 自检（假臂 + 边车，全离线）
# ============================================================================
def selftest() -> int:
    print("=" * 108)
    print("  udp_vs_http_bench 自检（假臂 + 边车 + 真 socket，无需硬件）")
    print("=" * 108)
    from arm_sdk.mock_server import MockArmServer

    mock = MockArmServer(port=0, log=lambda m: None)
    mock.bind()
    mport = mock.sock.getsockname()[1]
    threading.Thread(target=mock.serve_forever, daemon=True).start()
    time.sleep(0.15)

    b = Bench("127.0.0.1", mport)
    passed = failed = 0

    def chk(name: str, cond: bool, detail: str = "") -> None:
        nonlocal passed, failed
        if cond:
            passed += 1
            print("  ✓ %s%s" % (name, ("   " + detail) if detail else ""))
        else:
            failed += 1
            print("  ✗ %s%s" % (name, ("   " + detail) if detail else ""))

    try:
        b.bridge_port = b.start_bridge()
        chk("边车能起来", b.bridge_port > 0, "port=%d" % b.bridge_port)

        # ---- RTT：三种形态都要跑通 ----
        stats = b.mode_rtt(list(FORMS_ALL), n=40, warmup=5, timeout=0.5)
        by = {s.name: s for s in stats}
        for form in FORMS_ALL:
            s = by.get(form)
            chk("rtt 形态 %s 有样本" % form, bool(s and s.n_sent == 40),
                "ok=%s timeout=%s" % (s.n_ok if s else "?", s.n_timeout if s else "?"))
        chk("  HTTP 形态带回了链路 RTT（X-Arm-Rtt-Ms）",
            bool(by["http-close"].arm_lat) and bool(by["http-keepalive"].arm_lat),
            "keepalive n=%d" % len(by["http-keepalive"].arm_lat))
        chk("  HTTP 开销能算出来（端到端 > 链路）",
            (by["http-keepalive"].overhead() or 0) >= 0,
            "overhead p50=%s" % _fmt(by["http-keepalive"].overhead()))
        chk("  UDP 形态没有链路RTT字段（它本身就是链路）",
            not by["udp"].arm_lat)

        # ---- 假臂没有 nx ⇒ board_loss 必须是 None，不能假装测到 ----
        chk("假臂无 nx ⇒ board_loss 如实为 None（不编数）",
            b.read_nx() is None)

        # ---- sweep ----
        sw = b.mode_sweep(["udp"], (50, 200), sec=0.5, timeout=0.5, with_board=False)
        chk("sweep 跑出两档", len(sw) == 2, " | ".join("%s=%.0ffps" % (s.name, s.achieved_fps) for s in sw))
        chk("  sweep 用绝对时间轴（实测 fps 不超目标 15%）",
            all(s.achieved_fps <= int(s.name.split("@")[1].rstrip("fps")) * 1.15 for s in sw),
            " | ".join("%.1f" % s.achieved_fps for s in sw))

        # ---- burst ----
        bs = b.mode_burst(60, timeout=0.5, with_board=False)
        chk("burst 发得出去", bs.n_sent == 60 and bs.wall_s > 0,
            "%.3fs ⇒ %.0f fps 瞬时" % (bs.wall_s, bs.achieved_fps))
        chk("  burst 板侧测不到时如实说明（不报 0 丢包）",
            bs.board_delta is None and "假臂" in bs.note)

        # ---- 结果可 JSON 化 ----
        try:
            json.dumps([s.as_dict() for s in stats])
            chk("结果可序列化成 JSON（--json 导出用）", True)
        except Exception as e:                                   # noqa: BLE001
            chk("结果可序列化成 JSON", False, repr(e))

    except Exception as e:                                       # noqa: BLE001
        import traceback
        traceback.print_exc()
        chk("自检未抛异常", False, repr(e))
    finally:
        b.stop_bridge()
        mock.close()
        time.sleep(0.6)

    print("\n  udp_vs_http_bench 自检：%d 项，失败 %d" % (passed + failed, failed))
    return 0 if failed == 0 else 1


# ============================================================================
# CLI
# ============================================================================
def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="udp_vs_http_bench.py",
        description="UDP 直连 vs HTTP 边车：延迟 / 速率 / 丢包 对照（只发 PING，不动臂）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "例子：\n"
            "  python tools/udp_vs_http_bench.py --selftest\n"
            "  python tools/udp_vs_http_bench.py --mode rtt --target-ip 192.168.7.120 --n 500\n"
            "  python tools/udp_vs_http_bench.py --mode sweep --with-burst\n"
            "  python tools/udp_vs_http_bench.py --mode rtt --forms udp,http-keepalive\n"
        ))
    ap.add_argument("--selftest", action="store_true", help="离线自检（假臂，无需硬件）")
    ap.add_argument("--mode", choices=["rtt", "sweep", "burst", "all"], default="all")
    ap.add_argument("--forms", default=",".join(FORMS_ALL),
                    help="逗号分隔：%s（默认全跑）" % "/".join(FORMS_ALL))
    ap.add_argument("--target-ip", default="192.168.7.120", dest="ip")
    ap.add_argument("--target-port", type=int, default=5000, dest="port")
    ap.add_argument("--bridge-port", type=int, default=0, dest="bport",
                    help="边车监听端口（0=自动挑空闲）")
    ap.add_argument("--n", type=int, default=300, help="rtt 模式每形态样本数")
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--gap", type=float, default=0.0, help="rtt 模式样本间隔秒（0=连打）")
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--rates", default=",".join(str(r) for r in RATES_DEFAULT),
                    help="sweep 速率阶梯 fps")
    ap.add_argument("--sec", type=float, default=3.0, help="sweep 每档时长秒")
    ap.add_argument("--burst", type=int, default=200, help="burst 模式连发条数")
    ap.add_argument("--with-burst", action="store_true", dest="with_burst",
                    help="跑完 sweep 顺带跑一次 burst")
    ap.add_argument("--no-board", action="store_false", dest="board",
                    help="跳过板侧 nx 计数（假臂/不想多发命令时用）")
    ap.add_argument("--json", default=None, help="把结果写到这个 JSON 文件")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    forms = [f.strip() for f in args.forms.split(",") if f.strip()]
    for f in forms:
        if f not in FORMS_ALL:
            print("✗ 未知形态 %r（可选 %s）" % (f, "/".join(FORMS_ALL)))
            return 2
    rates = tuple(int(x) for x in args.rates.split(",") if x.strip())

    b = Bench(args.ip, args.port, bridge_port=args.bport, verbose=not args.quiet)
    print("=" * 108)
    print("  UDP vs HTTP 对照基准")
    print("=" * 108)
    print("  目标臂   : net://%s:%d" % (args.ip, args.port))
    print("  形态     : %s" % ", ".join(forms))
    print("  探针命令 : PING(0x01) —— 只读、必有应答、不动任何东西")
    print("")
    print("  ⚠ 先确认网口通：串口敲 `link net` → `link save` → 按 RESET。")
    print("    不通的表现是【一个字节都不回】，最终表现为 HTTP 502 / UDP 全超时。")

    all_stats: list[Stat] = []
    try:
        b.bridge_port = b.start_bridge()
        print("  边车     : http://127.0.0.1:%d  (in-process)" % b.bridge_port)
        if args.board:
            n0 = b.read_nx()
            print("  板侧计数 : %s" % ("udp_cmd=%d（可测真丢包）" % n0 if n0 is not None
                                       else "读不到（假臂无 nx / 链路不通）⇒ 只报 PC 侧指标"))
        if args.mode in ("rtt", "all"):
            all_stats += b.mode_rtt(forms, args.n, args.warmup, args.timeout, args.gap)
            print_rtt_table([s for s in all_stats if "@" not in s.name])
        if args.mode in ("sweep", "all"):
            print("\n  扫频中…（每档 %.1fs）" % args.sec)
            sw = b.mode_sweep(forms, rates, args.sec, args.timeout, with_board=args.board)
            if not args.quiet:
                print_sweep_table(sw)      # 已在循环里逐行打过，这里给表
            all_stats += sw
        if args.mode == "burst" or (args.mode == "all" and args.with_burst) or \
                (args.mode == "all" and args.board):
            bs = b.mode_burst(args.burst, args.timeout, with_board=args.board)
            print_burst_table(bs)
            all_stats.append(bs)
        print_verdict([s for s in all_stats if "@" not in s.name])
    except KeyboardInterrupt:
        print("\n  中断。")
    finally:
        b.stop_bridge()

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"target": "net://%s:%d" % (args.ip, args.port),
                       "when": time.strftime("%Y-%m-%d %H:%M:%S"),
                       "results": [s.as_dict() for s in all_stats]},
                      f, ensure_ascii=False, indent=2)
        print("\n  结果已写：%s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
