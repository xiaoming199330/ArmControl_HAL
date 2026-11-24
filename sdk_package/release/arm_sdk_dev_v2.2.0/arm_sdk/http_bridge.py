#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""http_bridge.py — 把臂的 UDP 逻辑包协议**包一层 HTTP**，给上位机软件用。

为什么需要它
------------
板子上的网口栈是**手写的裸 MAC + ARP + IPv4 + ICMP + UDP**（`Core/Src/arm_net.c`，
1430 行），**没有 TCP**。所以"直接给板子发 HTTP"这条路不存在 —— 要 TCP 就得在
MCU 里写重传/滑动窗口/拥塞控制/乱序重组，收益远小于代价。

但上位机工程师的诉求是真实的：用 `requests.post()` / `HttpClient` / Postman
直接调，而不是自己处理 SEQ 配对、CRC-16、超时和分页 UTF-8。

于是边车放在 **PC 侧**：

    ┌──────────┐  HTTP/1.1 (TCP, keep-alive)   ┌──────────────┐   逻辑包 UDP   ┌──────┐
    │  上位机  │ ────────────────────────────► │ http_bridge  │ ────────────► │ 臂   │
    │ (GUI/脚本)│ ◄──────────────────────────── │  (本进程)    │ ◄──────────── │ :5000│
    └──────────┘        JSON 回包               └──────────────┘     回包        └──────┘
                     ← 这一跳是 loopback，<0.3 ms →     ← 这一跳才是真正被测的链路 →

**边车不改协议、不加语义** —— 它只是把 `{"cmd":16,"payload_hex":"..."}` 翻译成
逻辑包，把回包翻译成 JSON。所以你在 HTTP 上看到的行为与 UDP 直连**完全一致**，
包括错误码、丢读哨兵、静默钳制。

接口一览
--------
    GET  /api/health          探活（含到臂的 RTT 与 fw-tag）
    GET  /api/status          六轴状态（角度/负载/温度/电压/读失败轴）
    GET  /api/pose            FK 末端位姿
    GET  /api/info            固件 tag / 构建 / 激活链路
    GET  /api/flags           E-Stop / 轨迹 / 流 / 修正 / 示教 标志位
    GET  /api/limits          运行时限位（raw + 舵机度）
    GET  /api/joint_detail    逐轴细节（含 speed/load/volt/temp/hw_err/ok）
    GET  /api/nx              ★ 板侧网络计数（rx/tx/udp cmd/crc err）—— 测丢包用
    GET  /api/metrics         边车自身统计（请求数/超时/RTT 分位）
    POST /api/raw             {"cmd":1,"payload_hex":"","timeout_ms":300}
    POST /api/text            {"text":"s"}   ← 124 条控制台命令，自动翻页
    POST /api/stream          {"delta_deg":[...6]}  ← 50 Hz 流帧（只发不等，无 JSON 回包）

每个响应都带 `X-Arm-Rtt-Ms` 头 = **边车到臂**的往返毫秒数。
用它就能把"链路延迟"和"HTTP 开销"分开算 —— 这是本边车最重要的一个设计。

⚠ 安全：本边车**不提供**任何会松力矩/改标定基准的快捷接口（`M a` / `zr` / `i` / `Y J`）。
   要走这些命令请用 `POST /api/text` 并自己承担 —— 与串口敲下去完全等价。

用法
----
    # 起边车（默认监听 127.0.0.1:8080，只绑本地回环）
    python -m arm_sdk.http_bridge --target 192.168.7.120:5000 --port 8080

    # 上位机侧
    curl http://127.0.0.1:8080/api/status
    curl -X POST http://127.0.0.1:8080/api/text -d '{"text":"s"}'

    # 无硬件演练：先起假臂，再指过来
    armctl mock --port 5000 &
    python -m arm_sdk.http_bridge --target 127.0.0.1:5000 --port 8080
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

from .client import ArmClient, ArmCommandError, Reply, open_arm
from .protocol import (cmd_name,
                       CMD_TEXT_EXEC, CMD_STREAM_JOINTS,
                       payload_stream_joints)

__all__ = ["BridgeStats", "ArmHttpBridge", "serve", "main"]

BRIDGE_VERSION = "1.0.0"
MAX_BODY = 1 << 16          # 64 KB，防呆（正常请求体 < 1 KB）

_RAW_READ_FAIL = -32768     # GET_STATUS 某轴读失败的哨兵（不是 -1、不是 0）
_TEMP_READ_FAIL = 0xFF

_NX_RX_RE = re.compile(r"rx/tx\s*:\s*(\d+)/(\d+)")
_NX_UDP_RE = re.compile(r"udp\s*cmd\s*=\s*(\d+)")
_NX_CRC_RE = re.compile(r"crc\s*err\s*=\s*(\d+)")
_NX_TXE_RE = re.compile(r"tx\s*err\s*=\s*(\d+)")
_NX_STATE_RE = re.compile(r"state\s*:\s*([A-Za-z/\-]+)")
_NX_LINK_RE = re.compile(r"link\s*:\s*(UP|down)\s*,\s*(\d+)M\s*([a-z\-]+)")


# ============================================================================
# 统计
# ============================================================================
class BridgeStats:
    """边车自身的计数器。线程安全（用同一把锁，因为都在请求路径上）。"""

    def __init__(self) -> None:
        self.started = time.time()
        self.requests = 0
        self.ok = 0
        self.failed = 0
        self.arm_timeouts = 0
        self.bytes_in = 0
        self.rtts: list[float] = []          # 边车→臂 的 RTT（毫秒）
        self.http_rtts: list[float] = []     # 整个 HTTP 请求墙钟（毫秒）

    @staticmethod
    def _pct(sorted_vals: list[float], p: float) -> float:
        if not sorted_vals:
            return 0.0
        if len(sorted_vals) == 1:
            return sorted_vals[0]
        k = (len(sorted_vals) - 1) * p
        lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
        return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)

    def snapshot(self) -> dict[str, Any]:
        a = sorted(self.rtts)
        h = sorted(self.http_rtts)
        return {
            "bridge_version": BRIDGE_VERSION,
            "uptime_s": round(time.time() - self.started, 3),
            "requests": self.requests,
            "ok": self.ok,
            "failed": self.failed,
            "arm_timeouts": self.arm_timeouts,
            "timeout_rate": round(self.arm_timeouts / self.requests, 6) if self.requests else 0.0,
            "bytes_in": self.bytes_in,
            "arm_rtt_ms": {
                "n": len(a),
                "min": round(a[0], 3) if a else 0.0,
                "p50": round(self._pct(a, 0.50), 3),
                "p90": round(self._pct(a, 0.90), 3),
                "p99": round(self._pct(a, 0.99), 3),
                "max": round(a[-1], 3) if a else 0.0,
                "mean": round(sum(a) / len(a), 3) if a else 0.0,
            },
            "http_total_ms": {
                "n": len(h),
                "p50": round(self._pct(h, 0.50), 3),
                "p99": round(self._pct(h, 0.99), 3),
                "max": round(h[-1], 3) if h else 0.0,
            },
        }


# ============================================================================
# 桥本体（与 HTTP 层解耦，便于单测）
# ============================================================================
class ArmHttpBridge:
    """把 ArmClient 的操作映射成可 JSON 化的字典。**不含 HTTP**。"""

    def __init__(self, uri: str, timeout: float = 0.3) -> None:
        self.uri = uri
        self.timeout = timeout
        self.stats = BridgeStats()
        self._lock = threading.Lock()        # UDP 回包靠 SEQ 配对，串行化最稳
        self._client: Optional[ArmClient] = None

    # ---- 生命周期 ----
    def open(self) -> None:
        if self._client is None:
            self._client = open_arm(self.uri, timeout=self.timeout)
            self._client.open()

    def close(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    @property
    def client(self) -> ArmClient:
        if self._client is None:
            self.open()
        assert self._client is not None
        return self._client

    def reconnect(self) -> None:
        try:
            self.close()
        finally:
            self.open()

    # ---- 内部：统一入口（计量 + 串行化）----
    def _call(self, fn, *a, **kw) -> tuple[Any, float]:
        with self._lock:
            self.stats.requests += 1
            t0 = time.perf_counter()
            try:
                out = fn(*a, **kw)
            except ArmCommandError as e:
                self.stats.failed += 1
                if e.reply.timeout:
                    self.stats.arm_timeouts += 1
                raise
            except Exception:
                self.stats.failed += 1
                raise
            finally:
                pass
            rtt = (time.perf_counter() - t0) * 1000.0
            self.stats.ok += 1
            self.stats.rtts.append(rtt)
            if len(self.stats.rtts) > 200_000:
                del self.stats.rtts[:100_000]
            return out, rtt

    # ---- 各端点 ----
    def health(self) -> tuple[dict[str, Any], float]:
        info, rtt = self._call(self.client.get_info)
        return {
            "ok": True,
            "target": self.uri,
            "fw_tag": info.fw_tag,
            "build": info.build,
            "build_name": info.build_name,
            "link_mode": info.link_mode,
            "n_params": info.n_params,
        }, rtt

    def status(self) -> tuple[dict[str, Any], float]:
        st, rtt = self._call(self.client.get_status)
        joints = list(st.joints)
        # ⚠ 丢读轴原样上报 None，**不兜底**（客户端已保证）
        return {
            "joints_servo_deg": [None if v is None else round(v, 3) for v in joints],
            "read_failed_joints_1based": st.failed_joints(),
            "read_ok": bool(st.read_ok),
            "pose_mm_deg": None if st.pose() is None else [round(v, 3) for v in st.pose()],
            "flags": st.flags,
            "estop": bool(st.estop),
            "traj": bool(st.traj),
            "corr": bool(st.corr),
        }, rtt

    def pose(self) -> tuple[dict[str, Any], float]:
        fk, rtt = self._call(self.client.fk_query)
        return {
            "x": fk.x, "y": fk.y, "z": fk.z, "pitch": fk.pitch,
            "joints_servo_deg": [None if v is None else round(v, 3) for v in fk.joints],
            "read_ok": fk.read_ok,
            "as_servo_deg": fk.as_servo_deg(),
        }, rtt

    def info(self) -> tuple[dict[str, Any], float]:
        info, rtt = self._call(self.client.get_info)
        return {k: getattr(info, k) for k in dir(info)
                if not k.startswith("_") and not callable(getattr(info, k))}, rtt

    def flags(self) -> tuple[dict[str, Any], float]:
        fl, rtt = self._call(self.client.get_flags)
        return {k: getattr(fl, k) for k in dir(fl)
                if not k.startswith("_") and not callable(getattr(fl, k))}, rtt

    def limits(self) -> tuple[dict[str, Any], float]:
        lim, rtt = self._call(self.client.get_limits)
        return {k: getattr(lim, k) for k in dir(lim)
                if not k.startswith("_") and not callable(getattr(lim, k))}, rtt

    def joint_detail(self, joint: int = 0xFF) -> tuple[dict[str, Any], float]:
        ds, rtt = self._call(self.client.get_joint_detail, joint)
        out = []
        for d in ds:
            out.append({
                "index": d.joint,
                # ⚠ 逐字段瞬时丢读：温度哨兵 0xFF 不是 0℃；pos_raw 哨兵 0x8000 不是 0
                "pos_raw": d.pos_raw,
                "temp_c": None if d.temp_lost else d.temp,
                "temp_raw": d.temp,
                "voltage_v": None if d.voltage is None else round(float(d.voltage), 2),
                "speed": d.speed,
                "load": d.load,
                "hw_err": d.hw_err,
                "ok": d.ok,
            })
        return {"joints": out,
                "read_failed_1based": [j["index"] + 1 for j in out if not j["ok"]]}, rtt

    def nx(self) -> tuple[dict[str, Any], float]:
        """★ 读板侧网络计数 —— 这是**唯一**能测出真实单向丢包的办法。

        本协议没有传输层 ACK，PC 侧只能看到"超时"，分不清是
        (a) 包在路上丢了 / (b) 板子收到了但回包丢了 / (c) 板子根本没排上队。
        板子的 `nx` 打印 `udp cmd=` 累计值 ⇒ 用它做差就知道**板子到底收了几条**。
        """
        text, rtt = self._call(self.client.text_exec, "nx")
        out = parse_nx(text)
        out["raw"] = text
        return out, rtt

    # ---- 写类 ----
    def raw(self, cmd: int, payload_hex: str = "", timeout_ms: Optional[int] = None) -> tuple[dict[str, Any], float]:
        payload = bytes.fromhex(payload_hex or "")
        kw = {}
        if timeout_ms is not None:
            kw["timeout"] = timeout_ms / 1000.0
        r, rtt = self._call(self.client.request, int(cmd), payload, **kw)
        return _reply_json(r), rtt

    def text(self, command: str) -> tuple[dict[str, Any], float]:
        out, rtt = self._call(self.client.text_out, command)
        return {
            "text": out.text,
            "total_bytes": getattr(out, "total", None),
            "truncated": bool(getattr(out, "truncated", False)),
            "gated": bool(getattr(out, "gated", False)),
        }, rtt

    def stream(self, delta_deg: list[float]) -> tuple[dict[str, Any], float]:
        """50 Hz 流帧：**只发不等**（等 ACK 会把帧率锁死在 RTT 上）。"""
        with self._lock:
            t0 = time.perf_counter()
            self.stats.requests += 1
            seq = self.client.send_async(CMD_STREAM_JOINTS, payload_stream_joints(delta_deg))
            rtt = (time.perf_counter() - t0) * 1000.0
            self.stats.ok += 1
            self.stats.rtts.append(rtt)
        return {"seq": seq, "sent": True, "note": "只发不等，ACK 由 /api/metrics 的 stream_acks 体现"}, rtt

    def move_joints(self, servo_deg: list[float], v: float = 0.0, a: float = 0.0) -> tuple[dict[str, Any], float]:
        r, rtt = self._call(self.client.move_joints, servo_deg, v, a)
        return _reply_json(r), rtt

    def move_cart(self, x: float, y: float, z: float, pitch: float = 0.0,
                  v: float = 0.0, a: float = 0.0) -> tuple[dict[str, Any], float]:
        r, rtt = self._call(self.client.move_cart, x, y, z, pitch, v, a)
        return _reply_json(r), rtt

    def stop(self) -> tuple[dict[str, Any], float]:
        r, rtt = self._call(self.client.stop)
        return _reply_json(r), rtt


def parse_nx(text: str) -> dict[str, Any]:
    """把控制台 `nx` 的输出解析成字典。

    真实输出长这样（2026-09-20 实机抓的，见 docs/网口PHY_Bringup_诊断_2026-09-20.md §14.8）:

        ===== nx NET status =====
        state    : RUN (phy found addr 0)
        link     : UP, 100M full-duplex
        addr     : udp://192.168.7.120:5000  (MAC 02:4E:45:54:00:01)
        rx/tx    : 92/40 帧   udp cmd=34  crc err=0  tx err=0
        peer     : arp=3  icmp(ping)=3

    `udp_cmd` 是**累计**值 ⇒ 测丢包要在发送前后各读一次做差，不能只看绝对值。
    """
    out: dict[str, Any] = {}
    m = _NX_RX_RE.search(text)
    if m:
        out["rx_pkts"], out["tx_pkts"] = int(m.group(1)), int(m.group(2))
    m = _NX_UDP_RE.search(text)
    if m:
        out["udp_cmd"] = int(m.group(1))
    m = _NX_CRC_RE.search(text)
    if m:
        out["crc_err"] = int(m.group(1))
    m = _NX_TXE_RE.search(text)
    if m:
        out["tx_err"] = int(m.group(1))
    m = _NX_STATE_RE.search(text)
    if m:
        out["state"] = m.group(1)
    m = _NX_LINK_RE.search(text)
    if m:
        out["link"] = {"up": m.group(1) == "UP",
                       "mbps": int(m.group(2)), "duplex": m.group(3)}
    out["parsed"] = "udp_cmd" in out
    return out


# 实机 nx 输出样本（自检用；改解析器时靠它防回归）
NX_SAMPLE = (
    "  ===== nx NET status =====\r\n"
    "  state    : RUN (phy found addr 0)\r\n"
    "  link     : UP, 100M full-duplex\r\n"
    "  addr     : udp://192.168.7.120:5000  (MAC 02:4E:45:54:00:01)\r\n"
    "  rx/tx    : 92/40 帧   udp cmd=34  crc err=0  tx err=0\r\n"
    "  peer     : arp=3  icmp(ping)=3\r\n"
)


def _reply_json(r: Reply) -> dict[str, Any]:
    return {
        "ok": bool(r.ok),
        "cmd": r.cmd,
        "cmd_name": cmd_name(r.req_cmd),
        "err": r.err,
        "err_name": r.err_name,
        "desc": r.desc,
        "payload_hex": r.payload.hex(),
        "latency_ms": round(r.latency_ms, 3),
        "timeout": bool(r.timeout),
        "attempts": r.attempts,
    }


# ============================================================================
# HTTP 层
# ============================================================================
class _Handler(BaseHTTPRequestHandler):
    server_version = "ArmHttpBridge/" + BRIDGE_VERSION
    protocol_version = "HTTP/1.1"        # keep-alive ⇒ 值得测的 HTTP 形态

    bridge: ArmHttpBridge                # 由 serve() 注入

    # ---- 工具 ----
    def log_message(self, fmt, *args):   # 静音默认日志（bench 时会刷屏）
        if getattr(self.server, "verbose", False):
            sys.stderr.write("  %s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code: int, obj: Any, rtt_ms: float) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Arm-Rtt-Ms", "%.3f" % rtt_ms)
        self.send_header("X-Bridge-Version", BRIDGE_VERSION)
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict[str, Any]:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        if n > MAX_BODY:
            raise ValueError("body 太大（%d > %d）" % (n, MAX_BODY))
        self.bridge.stats.bytes_in += n
        raw = self.rfile.read(n)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception as e:
            raise ValueError("body 不是合法 JSON: %s" % e)

    def _dispatch(self, fn, *a, **kw) -> None:
        t0 = time.perf_counter()
        try:
            obj, rtt = fn(*a, **kw)
            self.bridge.stats.http_rtts.append((time.perf_counter() - t0) * 1000.0)
            self._send(200, obj, rtt)
        except ArmCommandError as e:
            b = self.bridge
            b.stats.http_rtts.append((time.perf_counter() - t0) * 1000.0)
            self._send(502, {"ok": False, "error": "arm", **_reply_json(e.reply)},
                       e.reply.latency_ms)
        except Exception as e:
            self.bridge.stats.http_rtts.append((time.perf_counter() - t0) * 1000.0)
            self._send(400, {"ok": False, "error": type(e).__name__, "message": str(e)}, 0.0)

    # ---- 路由 ----
    def do_GET(self) -> None:                                  # noqa: N802
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        b = self.bridge
        routes = {
            "/api/health": b.health,
            "/api/status": b.status,
            "/api/pose": b.pose,
            "/api/info": b.info,
            "/api/flags": b.flags,
            "/api/limits": b.limits,
            "/api/joint_detail": b.joint_detail,
            "/api/nx": b.nx,
            "/api/metrics": lambda: (b.stats.snapshot(), 0.0),
            "/": lambda: ({"bridge": BRIDGE_VERSION, "target": b.uri,
                           "endpoints": sorted(_ENDPOINTS)}, 0.0),
        }
        fn = routes.get(path)
        if fn is None:
            self._send(404, {"ok": False, "error": "no such endpoint", "path": path,
                             "endpoints": sorted(_ENDPOINTS)}, 0.0)
            return
        self._dispatch(fn)

    def do_POST(self) -> None:                                 # noqa: N802
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        b = self.bridge
        try:
            body = self._read_body()
        except ValueError as e:
            self._send(400, {"ok": False, "error": "bad_body", "message": str(e)}, 0.0)
            return

        if path == "/api/raw":
            self._dispatch(b.raw, body.get("cmd", 0), body.get("payload_hex", ""),
                           body.get("timeout_ms"))
        elif path == "/api/text":
            self._dispatch(b.text, body.get("text", ""))
        elif path == "/api/stream":
            self._dispatch(b.stream, list(body.get("delta_deg") or []))
        elif path == "/api/move_joints":
            self._dispatch(b.move_joints, list(body.get("servo_deg") or []),
                           float(body.get("v", 0.0)), float(body.get("a", 0.0)))
        elif path == "/api/move_cart":
            self._dispatch(b.move_cart,
                           float(body["x"]), float(body["y"]), float(body["z"]),
                           float(body.get("pitch", 0.0)),
                           float(body.get("v", 0.0)), float(body.get("a", 0.0)))
        elif path == "/api/stop":
            self._dispatch(b.stop)
        elif path == "/api/reconnect":
            self._dispatch(lambda: (b.reconnect() or {"ok": True}, 0.0))
        else:
            self._send(404, {"ok": False, "error": "no such endpoint", "path": path,
                             "endpoints": sorted(_ENDPOINTS)}, 0.0)


_ENDPOINTS = {
    "GET /api/health", "GET /api/status", "GET /api/pose", "GET /api/info",
    "GET /api/flags", "GET /api/limits", "GET /api/joint_detail", "GET /api/nx",
    "GET /api/metrics",
    "POST /api/raw", "POST /api/text", "POST /api/stream",
    "POST /api/move_joints", "POST /api/move_cart", "POST /api/stop",
    "POST /api/reconnect",
}


def serve(target: str, host: str = "127.0.0.1", port: int = 8080,
          timeout: float = 0.3, verbose: bool = False) -> ThreadingHTTPServer:
    """起边车（返回 server，由调用方决定 serve_forever 还是 shutdown）。

    ⚠ 默认只绑 `127.0.0.1`：边车没有鉴权，暴露到 0.0.0.0 等于把机械臂
      交给同网段的任何人。
    """
    bridge = ArmHttpBridge(target, timeout=timeout)
    bridge.open()
    _Handler.bridge = bridge
    srv = ThreadingHTTPServer((host, port), _Handler)
    srv.daemon_threads = True
    srv.verbose = verbose        # type: ignore[attr-defined]
    srv.bridge = bridge          # type: ignore[attr-defined]
    return srv


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m arm_sdk.http_bridge",
        description="把臂的 UDP 逻辑包协议包一层 HTTP（边车跑在 PC 侧，板子不动）")
    ap.add_argument("--target", default="192.168.7.120:5000",
                    help="臂的 UDP 地址 IP:PORT，或 net://IP:PORT / sim://（默认 192.168.7.120:5000）")
    ap.add_argument("--bind", default="127.0.0.1",
                    help="边车监听地址（默认 127.0.0.1；⚠ 不要随便改成 0.0.0.0 —— 无鉴权）")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--timeout", type=float, default=0.3, help="到臂的单次请求超时秒")
    ap.add_argument("--verbose", action="store_true", help="打印每个 HTTP 请求")
    ap.add_argument("--selftest", action="store_true", help="离线自检（不需要臂）")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    uri = args.target
    if "://" not in uri:
        uri = "net://" + uri

    try:
        srv = serve(uri, args.bind, args.port, args.timeout, args.verbose)
    except OSError as e:
        print("✗ 起不来：%s" % e, file=sys.stderr)
        print("  端口被占？换成 --port 8081。", file=sys.stderr)
        return 1

    print("=" * 68)
    print("  HTTP ↔ UDP 边车  v%s" % BRIDGE_VERSION)
    print("=" * 68)
    print("  目标臂   : %s" % uri)
    print("  监听     : http://%s:%d" % (args.bind, args.port))
    print("  试一下   : curl http://%s:%d/api/status" % (args.bind, args.port))
    print("  端点列表 : curl http://%s:%d/" % (args.bind, args.port))
    print("")
    print("  ⚠ 首次请求若报 502，多半是 link 没开或 IP 不对：")
    print("     串口敲 `link net` → `link save` → 按 RESET")
    print("     网口不通的表现是【一个字节都不回】，不是报错")
    print("=" * 68)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  收到 Ctrl-C，收工。")
    finally:
        srv.shutdown()
        srv.server_close()          # type: ignore[attr-defined]
        srv.bridge.close()          # type: ignore[attr-defined]
    return 0


# ============================================================================
# 自检（不需要硬件：起假臂 + 边车，走真 HTTP + 真 UDP）
# ============================================================================
def selftest() -> int:
    """端到端自检：mock 假臂 → 边车 → HTTP 客户端，全走真 socket。"""
    import threading as _th
    import urllib.error as _urlerr
    import urllib.request as _url

    print("=" * 68)
    print("  http_bridge 自检（假臂 + 真 UDP + 真 HTTP，无需硬件）")
    print("=" * 68)

    from .mock_server import MockArmServer
    mock = MockArmServer(port=0, log=lambda m: None)   # 0 = 让系统分配空闲端口；静音
    mock.bind()
    mport = mock.sock.getsockname()[1]
    _th.Thread(target=mock.serve_forever, daemon=True).start()
    time.sleep(0.15)

    srv = serve("net://127.0.0.1:%d" % mport, "127.0.0.1", 0, timeout=0.5)
    hport = srv.server_address[1]
    _th.Thread(target=srv.serve_forever, daemon=True).start()
    time.sleep(0.15)

    base = "http://127.0.0.1:%d" % hport
    passed = failed = 0

    def chk(name: str, cond: bool, detail: str = "") -> None:
        nonlocal passed, failed
        if cond:
            passed += 1
            print("  ✓ %s%s" % (name, ("   " + detail) if detail else ""))
        else:
            failed += 1
            print("  ✗ %s%s" % (name, ("   " + detail) if detail else ""))

    def get(path: str) -> tuple[int, dict, float]:
        with _url.urlopen(base + path, timeout=5) as r:
            rtt = float(r.headers.get("X-Arm-Rtt-Ms") or 0.0)
            return r.status, json.loads(r.read().decode("utf-8")), rtt

    def get_any(path: str) -> tuple[int, dict, float]:
        """与 get 相同，但 4xx/5xx 也返回（不抛 HTTPError）。"""
        try:
            return get(path)
        except _urlerr.HTTPError as e:
            rtt = float(e.headers.get("X-Arm-Rtt-Ms") or 0.0) if e.headers else 0.0
            try:
                body = json.loads(e.read().decode("utf-8"))
            except Exception:
                body = {"raw": "<非 JSON>"}
            return e.code, body, rtt

    def post(path: str, obj: Any) -> tuple[int, dict, float]:
        data = json.dumps(obj).encode()
        req = _url.Request(base + path, data=data,
                           headers={"Content-Type": "application/json"}, method="POST")
        with _url.urlopen(req, timeout=5) as r:
            rtt = float(r.headers.get("X-Arm-Rtt-Ms") or 0.0)
            return r.status, json.loads(r.read().decode("utf-8")), rtt

    def post_any(path: str, obj: Any) -> tuple[int, dict, float]:
        """与 post 相同，但 4xx/5xx 也返回（不抛 HTTPError）。"""
        try:
            return post(path, obj)
        except _urlerr.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode("utf-8")), 0.0
            except Exception:
                return e.code, {"raw": "<非 JSON>"}, 0.0

    try:
        st, obj, rtt = get("/api/health")
        chk("GET /api/health", st == 200 and obj.get("ok"), "fw_tag=%s rtt=%.2fms" % (obj.get("fw_tag"), rtt))
        chk("  X-Arm-Rtt-Ms 头有值", rtt > 0, "%.3f ms" % rtt)

        st, obj, _ = get("/api/status")
        j = obj.get("joints_servo_deg") or []
        chk("GET /api/status 六轴", st == 200 and len(j) == 6, "joints=%s" % j[:2])
        chk("  丢读轴如实上报（不兜底）",
            "read_failed_joints_1based" in obj and "pose_mm_deg" in obj,
            "read_failed=%s" % obj.get("read_failed_joints_1based"))

        st, obj, _ = get("/api/joint_detail")
        chk("GET /api/joint_detail", st == 200 and len(obj.get("joints") or []) == 6)
        chk("  温度丢读哨兵 0xFF → null",
            any(d["temp_raw"] == 0xFF and d["temp_c"] is None for d in obj["joints"]) or
            all(d["temp_c"] is not None for d in obj["joints"]), "已按哨兵处理")

        # ---- nx：解析器拿**实机样本**验（sim 假臂没有 nx 命令，验不了） ----
        nx = parse_nx(NX_SAMPLE)
        chk("parse_nx 解析实机样本",
            nx.get("parsed") and nx.get("udp_cmd") == 34 and nx.get("rx_pkts") == 92
            and nx.get("tx_pkts") == 40 and nx.get("crc_err") == 0 and nx.get("tx_err") == 0,
            "udp_cmd=%s rx/tx=%s/%s" % (nx.get("udp_cmd"), nx.get("rx_pkts"), nx.get("tx_pkts")))
        chk("  parse_nx 认出链路状态",
            (nx.get("link") or {}).get("up") is True and (nx.get("link") or {}).get("mbps") == 100,
            str(nx.get("link")))
        st, obj, _ = get_any("/api/nx")
        chk("GET /api/nx 能调通（sim 无 nx ⇒ parsed=False 但不应 500）",
            st == 200 and obj.get("parsed") is False,
            (obj.get("raw") or "").strip()[:40])

        st, obj, _ = post("/api/text", {"text": "h"})
        chk("POST /api/text 文本透传", st == 200 and len(obj.get("text") or "") > 0,
            "%d 字节" % len(obj.get("text") or ""))

        st, obj, _ = post("/api/text", {"text": "cx"})
        chk("  抢引脚命令被闸门拦下（不是静默失败）",
            st == 200 and ("停用" in (obj.get("text") or "") or obj.get("gated")),
            (obj.get("text") or "")[:40].replace("\r\n", " "))

        st, obj, _ = post("/api/raw", {"cmd": 0x01, "payload_hex": ""})
        chk("POST /api/raw PING", st == 200 and obj.get("ok") is True,
            "err=%s latency=%.2fms" % (obj.get("err_name"), obj.get("latency_ms") or 0))

        st, obj, _ = post("/api/raw", {"cmd": 0xFF, "payload_hex": ""})
        chk("  非法/不存在的命令不挂掉边车", st in (200, 502), "http=%d" % st)

        # 坏 CRC 由 mock 侧保证回 NACK(err=2)：这里发一条 LEN 不符的包
        st, obj, _ = get("/api/metrics")
        m = obj.get("arm_rtt_ms") or {}
        chk("GET /api/metrics 有分位数", st == 200 and m.get("n", 0) > 0,
            "n=%s p50=%s p99=%s" % (m.get("n"), m.get("p50"), m.get("p99")))

        # ---- 关键：HTTP 开销 vs 链路延迟 ----
        arm_rtts, http_rtts = [], []
        for _ in range(20):
            t0 = time.perf_counter()
            _s, _o, r = get("/api/health")
            http_rtts.append((time.perf_counter() - t0) * 1000.0)
            arm_rtts.append(r)
        arm_rtts.sort()
        http_rtts.sort()
        p50a, p50h = arm_rtts[len(arm_rtts) // 2], http_rtts[len(http_rtts) // 2]
        chk("loopback HTTP 开销 = http_total − arm_rtt",
            p50h > p50a,
            "arm p50=%.2fms / http p50=%.2fms / 差=%.2fms" % (p50a, p50h, p50h - p50a))

        st, obj, _ = get("/")
        chk("GET / 列端点", st == 200 and len(obj.get("endpoints") or []) >= 10,
            "%d 个" % len(obj.get("endpoints") or []))

        st, obj, _ = get_any("/api/nope")
        chk("未知端点回 404（不是 500）", st == 404)

        st, obj, _ = post_any("/api/raw", {"cmd": 0x02, "payload_hex": "zz"})
        chk("坏 payload_hex 回 400（不是崩）", st == 400, str(obj.get("error")))

    except Exception as e:               # noqa: BLE001
        import traceback
        traceback.print_exc()
        chk("自检过程未抛异常", False, repr(e))
    finally:
        srv.shutdown()
        srv.server_close()
        srv.bridge.close()
        mock.close()
        time.sleep(0.6)      # 让假臂的 serve_forever 循环看到 socket 已关、干净退出
                             # （否则守护线程在解释器 finalize 阶段还在打印 ⇒ 
                             #   stderr 缓冲锁拿不到，报 _enter_buffered_busy）

    print("\n  http_bridge 自检：%d 项，失败 %d" % (passed + failed, failed))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
