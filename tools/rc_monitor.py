#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""rc_monitor.py — RC 遥控全链路监控（串口按键事件流 + UDP 姿态流，双流时间戳对齐）

为什么是两条流
    固件里 arm_rc.c / arm_rc_ctrl.c 的摇杆流帧路径**一行打印都没有**（不能每拍刷屏），
    串口只有 `[keys] ...` 按键事件。所以要同时采：
      ① 串口(COM11, 115200) 旁听: [keys] 武装/锁定/中位/折叠/夹爪(keysw)/录制/回放
         + [STALL]/[FAULT]/[BOOT] 故障 + `L` 限位（只读）。
      ② UDP(192.168.7.120:5000) 高频轮询 GET_STATUS(0x02): 六轴角度/位姿/pitch/标志
         + 往返延迟 + 丢读率（顺带量化 0924M 后的丢读退化）。
    两条流都用 PC 毫秒时钟 ⇒ 事件(按下)→姿态(动起来)的延迟可以直接对齐量化。

采集什么 → 对应哪个问题
    响应忽快忽慢    事件→运动段延迟表 + 运动段内速度分布/停滞样本占比
    X 前后末端不水平 X 变化段内 pitch 的漂移量
    夹爪延迟高      keysw 的 L2/R2 按下时刻 → J6 首动时刻
    伸直退不回来    各轴到限位距离扫描（<2° 的样本全部列出）
    折叠/中位后摇杆失效  [keys] 事件链: 折叠位→Curve complete→自动重武装 是否出现

用法（在工程根目录）
    python tools/rc_monitor.py                          # 默认: UDP 40Hz + COM11 旁听, keysw 600s
    python tools/rc_monitor.py --secs 120               # 限时 2 分钟自动结束
    python tools/rc_monitor.py --no-uart                # 只采 UDP（串口被占用时）
    python tools/rc_monitor.py --interval 0.05         # 降到 20Hz（丢读严重时用）
    python tools/rc_monitor.py --replay <log.jsonl>    # 只对已采数据跑分析
    python tools/rc_monitor.py --selftest              # 离线自检（本地假臂，不碰真板）

安全
    - 默认只**写**两条白名单命令: `L\r`（只读限位）和 `rc keysw 600\r`（只设打印窗口标志，
      固件里非阻塞，臂照常遥控）。绝无其他输出。
    - UDP 侧只发 GET_STATUS(0x02)，只读。
    - Ctrl+C 或敲 q 结束；结束时自动写 summary 分析报告。

输出
    outputs/rc_monitor_<时间戳>/log.jsonl + summary.txt，控制台同步显示。
"""
import argparse
import json
import os
import queue
import re
import socket
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_host as ch  # noqa: E402  复用 decode_status_payload, 与 arm_net_host 逐字节一致

try:
    import serial  # pyserial
except ImportError:
    serial = None

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

T0 = time.time()


def now_ms() -> float:
    return (time.time() - T0) * 1000.0


# 编译期已知限位（铁律 9，RAM calib 为准；串口 `L` 解析成功则覆盖）
FALLBACK_LIMITS = {
    0: (63.0, 303.0), 1: (75.0, 284.0), 2: (76.0, 271.0),
    3: (71.0, 284.0), 4: (12.0, 351.0), 5: (138.0, 266.0),
}

EVT_KEYS = re.compile(r"\[keys\]")
EVT_FAULT = re.compile(r"\[STALL\]|\[FAULT\]|\[BOOT\]|E-STOP|E-Stop")
EVT_MOVE = re.compile(r"Move started|Curve complete|Safe path|PRE-CLAMP|POST-CLAMP")
RE_LIMIT = re.compile(r"J([1-6])[^0-9]{1,12}(\d+(?:\.\d+)?)\s*[~\-–]{1,2}\s*(\d+(?:\.\d+)?)")

# keysw 里夹爪按键名（0924O 键位: L2=开 R2=合）
CLAW_BTNS = ("L2", "R2")


class JsonlWriter:
    def __init__(self, path):
        self.fh = open(path, "a", encoding="utf-8")
        self.lock = threading.Lock()
        self.n = 0

    def write(self, rec: dict):
        rec["t"] = round(now_ms(), 1)
        line = json.dumps(rec, ensure_ascii=False)
        with self.lock:
            if self.fh.closed:
                return
            try:
                self.fh.write(line + "\n")
                self.fh.flush()
                self.n += 1
            except (ValueError, OSError):
                pass  # 停止竞态: 线程在 close 后又写了一次, 忽略

    def close(self):
        with self.lock:
            self.fh.close()


# ============================================================
# UDP 姿态流（只发 GET_STATUS 0x02）
# ============================================================
class StatusPoller(threading.Thread):
    def __init__(self, ip, port, interval, out: JsonlWriter, stats: dict):
        super().__init__(daemon=True)
        self.addr = (ip, port)
        self.interval = interval
        self.out = out
        self.stats = stats
        self.seq = 0x40
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.12)
        self.stop_evt = threading.Event()
        self.last = None  # 最近一次成功样本 dict

    def run(self):
        while not self.stop_evt.is_set():
            t_send = time.time()
            self.seq = (self.seq + 1) & 0xFF
            pkt = bytes([self.seq, 0x02, 0])
            crc = ch.crc16(pkt)
            pkt += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
            ok = False
            try:
                self.sock.sendto(pkt, self.addr)
                deadline = 3
                for _ in range(deadline):
                    try:
                        data, _ = self.sock.recvfrom(2048)
                    except socket.timeout:
                        continue
                    try:
                        rseq, rcmd, pl = (data[0], data[1], data[3:3 + data[2]])
                    except Exception:
                        continue
                    if rseq != self.seq or rcmd != 0x82:
                        continue
                    rtt = time.time() - t_send
                    d = ch.decode_status_payload(pl)
                    j = list(d["joints"])
                    rec = {"src": "udp", "j": j, "x": d["x"], "y": d["y"], "z": d["z"],
                           "pitch": d["pitch"], "read_ok": bool(d["read_ok"]),
                           "estop": bool(d["estop"]), "traj": bool(d["traj"]),
                           "corr": bool(d["corr"]), "rtt": round(rtt * 1000, 1)}
                    self.out.write(rec)
                    self.last = rec
                    self.stats["udp_ok"] += 1
                    self.stats.setdefault("rtts", []).append(rtt * 1000)
                    ok = True
                    break
            except Exception as e:
                self.stats["udp_err"] += 1
                self.stats["udp_err_msg"] = repr(e)
            if not ok:
                # 记一条失败样本（六轴 None），分析时计入丢读
                self.out.write({"src": "udp", "j": [None] * 6, "read_ok": False,
                                "miss": True})
                self.stats["udp_miss"] += 1
            # 定节拍（扣除本轮耗时）
            dt = time.time() - t_send
            if dt < self.interval:
                self.stop_evt.wait(self.interval - dt)

    def stop(self):
        self.stop_evt.set()
        self.join(timeout=0.6)


# ============================================================
# 串口事件流（旁听 + 白名单两条命令）
# ============================================================
class UartSniffer(threading.Thread):
    WHITELIST = (b"L\r",)          # 启动时限位
    def __init__(self, port, baud, out: JsonlWriter, limits_box: dict, keysw_sec: int):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.out = out
        self.limits_box = limits_box
        self.keysw_sec = keysw_sec
        self.ser = None
        self.stop_evt = threading.Event()
        self.buf = bytearray()
        self.n_lines = 0

    def start_port(self) -> str:
        if serial is None:
            return "pyserial 未安装"
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            return "打不开 %s: %s" % (self.port, e)
        # 白名单输出: L(限位) + rc keysw(非阻塞按键监视窗口)
        try:
            self.ser.write(b"L\r")
            time.sleep(0.05)
            if self.keysw_sec > 0:
                self.ser.write(("rc keysw %d\r" % self.keysw_sec).encode())
        except Exception as e:
            return "写入失败: %s" % e
        return ""

    def _handle_line(self, line: str):
        line = line.strip()
        if not line:
            return
        self.n_lines += 1
        evt = "other"
        if EVT_KEYS.search(line):
            evt = "keys"
        elif EVT_FAULT.search(line):
            evt = "fault"
        elif EVT_MOVE.search(line):
            evt = "move"
        self.out.write({"src": "uart", "line": line[:200], "evt": evt})
        # 限位解析（L 的回显）
        m = RE_LIMIT.search(line)
        if m:
            idx = int(m.group(1)) - 1
            self.limits_box[idx] = (float(m.group(2)), float(m.group(3)))

    def run(self):
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(512)
            except Exception:
                self.stop_evt.wait(0.2)
                continue
            if not chunk:
                continue
            self.buf.extend(chunk)
            while True:
                # 控制台以 \r\n 分行
                i = self.buf.find(b"\n")
                j = self.buf.find(b"\r")
                if i < 0 and j < 0:
                    break
                if i < 0:
                    i = j
                elif 0 <= j < i:
                    i = j
                raw = bytes(self.buf[:i])
                self.buf = self.buf[i + 1:]
                try:
                    self._handle_line(raw.decode("utf-8", errors="replace"))
                except Exception:
                    pass

    def stop(self):
        self.stop_evt.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


# ============================================================
# 离线分析（采集结束自动跑; --replay 单独跑）
# ============================================================
def load_jsonl(path):
    recs = []
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                try:
                    recs.append(json.loads(ln))
                except Exception:
                    pass
    return recs


def movement_segments(recs, rate_thr=1.2, calm_thr=0.05, calm_n=4):
    """姿态流 → 运动段列表 [(t0,t1,axes)]。速度阈值 1.2°/s 起段, 连续 4 拍 <0.05°/s 收段。"""
    ups = [r for r in recs if r.get("src") == "udp" and not r.get("miss")
           and r.get("j") and all(v is not None for v in r["j"])]
    segs, cur = [], None
    prev = None
    for r in ups:
        t, j = r["t"], r["j"]
        if prev is not None:
            dt = (t - prev[0]) / 1000.0
            if dt <= 0 or dt > 0.5:
                prev = (t, j)
                continue
            v = [abs(j[k] - prev[1][k]) / dt for k in range(6)]
            vmax = max(v)
            if vmax >= rate_thr:
                if cur is None:
                    cur = {"t0": prev[0], "t1": t, "axes": set(k for k in range(6) if v[k] >= rate_thr)}
                else:
                    cur["t1"] = t
                    cur["axes"].update(k for k in range(6) if v[k] >= rate_thr)
            elif cur is not None and t - cur["t1"] > calm_n * 25:
                segs.append(cur)
                cur = None
        prev = (t, j)
    if cur is not None:
        segs.append(cur)
    return segs


def pct(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = max(0, min(len(xs) - 1, int(round(p / 100.0 * (len(xs) - 1)))))
    return xs[k]


def analyze(recs, limits, out_lines):
    W = out_lines.append
    W("=" * 64)
    W("RC 监控分析摘要")
    W("=" * 64)
    udp = [r for r in recs if r.get("src") == "udp"]
    miss = [r for r in udp if r.get("miss")]
    ok = [r for r in udp if not r.get("miss")]
    rtts = [r["rtt"] for r in ok if "rtt" in r]
    W("UDP 姿态流: 发 %d, 成功 %d, 失败 %d (丢读率 %.1f%%)" % (
        len(udp), len(ok), len(miss), 100.0 * len(miss) / max(1, len(udp))))
    if rtts:
        W("  GET_STATUS 往返: p50=%.1fms p95=%.1fms max=%.1fms" % (
            pct(rtts, 50), pct(rtts, 95), max(rtts)))
    marks = [r for r in recs if r.get("src") == "mark"]
    if marks:
        W("人工标记: %d 条" % len(marks))
        for r in marks:
            W("  t=%.1fs  %s" % (r["t"] / 1000.0, r.get("text", "")))

    # ---- 事件时间线 ----
    W("")
    W("[串口事件时间线]")
    events = [r for r in recs if r.get("src") == "uart" and r.get("evt") in ("keys", "fault")]
    segs = movement_segments(recs)
    for r in events:
        W("  %8.2fs  %s" % (r["t"] / 1000.0, r["line"]))
    if not events:
        W("  (无 —— 串口没接/被占用, 或这轮没按键)")

    # ---- 事件 → 运动段响应延迟 ----
    W("")
    W("[事件→姿态响应]  (事件时刻到其后第一个运动段起点的延迟)")
    for r in events:
        if r.get("evt") != "keys":
            continue
        nxt = [s for s in segs if s["t0"] >= r["t"] - 50]
        if nxt:
            s = min(nxt, key=lambda s: s["t0"])
            W("  %8.2fs  延迟 %6.0fms  段%.1f~%.1fs 轴%s | %s" % (
                r["t"] / 1000.0, s["t0"] - r["t"], s["t0"] / 1000, s["t1"] / 1000,
                "".join("J%d " % (k + 1) for k in sorted(s["axes"])), r["line"][:60]))
        else:
            W("  %8.2fs  其后无运动段 | %s" % (r["t"] / 1000.0, r["line"][:60]))

    # ---- 夹爪专项: L2/R2 → J6 首动 ----
    W("")
    W("[夹爪延迟]  (keysw 的 L2/R2 变化 → J6 首次 |Δ|>0.5°)")
    for r in events:
        if r.get("evt") != "keys":
            continue
        if not any((" %s=" % b) in r["line"] or (" %s=" % b) in r["line"] for b in CLAW_BTNS):
            continue
        if not any(("%s=1" % b) in r["line"] for b in CLAW_BTNS):
            continue  # 只看按下沿
        ups = [x for x in ok if x["t"] >= r["t"] and x.get("j") and x["j"][5] is not None]
        base = None
        for x in ok:
            if x["t"] <= r["t"] and x.get("j") and x["j"][5] is not None:
                base = x["j"][5]
        lat = None
        for x in ups:
            if base is not None and abs(x["j"][5] - base) > 0.5:
                lat = x["t"] - r["t"]
                break
        W("  %8.2fs  J6 响应: %s | %s" % (
            r["t"] / 1000.0, ("%.0fms" % lat) if lat is not None else "300ms 内未动", r["line"][:60]))

    # ---- 限位贴近扫描 ----
    W("")
    W("[限位贴近扫描]  (到编译限位距离 <2° 的样本, 占比与最贴近时刻)")
    for k in range(6):
        lo, hi = limits.get(k, FALLBACK_LIMITS[k])
        vals = [(x["t"], x["j"][k]) for x in ok if x.get("j") and x["j"][k] is not None]
        if not vals:
            continue
        n_lo = sum(1 for _, v in vals if v - lo < 2.0)
        n_hi = sum(1 for _, v in vals if hi - v < 2.0)
        if n_lo or n_hi:
            vmin = min(vals, key=lambda p: p[1])
            vmax = max(vals, key=lambda p: p[1])
            W("  J%d [%g~%g]: 贴下限 %d 拍 / 贴上限 %d 拍  (min=%.1f@%.1fs max=%.1f@%.1fs)" % (
                k + 1, lo, hi, n_lo, n_hi, vmin[1], vmin[0] / 1000, vmax[1], vmax[0] / 1000))

    # ---- 运动段与速度均匀性（卡顿量化）----
    W("")
    W("[运动段]  共 %d 段" % len(segs))
    for s in segs[:40]:
        dur = (s["t1"] - s["t0"]) / 1000.0
        # 段内速度均匀性: 主轴速度 p95/p50 与停滞样本占比
        in_seg = [x for x in ok if s["t0"] <= x["t"] <= s["t1"] + 60
                  and x.get("j") and all(v is not None for v in x["j"])]
        stall_note = ""
        if len(in_seg) > 6 and dur > 0.3:
            sp = []
            for a, b in zip(in_seg, in_seg[1:]):
                dt = (b["t"] - a["t"]) / 1000.0
                if 0 < dt <= 0.3:
                    sp.append(max(abs(b["j"][k] - a["j"][k]) for k in range(6)) / dt)
            if sp:
                stall = sum(1 for v in sp if v < 0.4) / len(sp)
                stall_note = " 停滞拍 %.0f%%" % (stall * 100)
        W("  %.2f~%.2fs (%.2fs) 轴%s%s" % (
            s["t0"] / 1000, s["t1"] / 1000, dur,
            "".join("J%d " % (k + 1) for k in sorted(s["axes"])), stall_note))

    # ---- pitch 稳定性（X 不水平问题）----
    W("")
    W("[pitch 稳定性]  (X 明显变化的段里 pitch 的漂移)")
    pitchs = [(x["t"], x.get("pitch")) for x in ok if x.get("pitch") is not None]
    for s in segs:
        in_seg = [p for p in pitchs if s["t0"] <= p[0] <= s["t1"] + 60]
        xs = [x for x in ok if s["t0"] <= x["t"] <= s["t1"] + 60 and x.get("x") is not None]
        if len(in_seg) > 4 and len(xs) > 4:
            dx = max(v["x"] for v in xs) - min(v["x"] for v in xs)
            if abs(dx) > 15:  # X 位移 >15mm 的段才算"X 运动"
                ps = [p[1] for p in in_seg]
                W("  段%.2f~%.2fs  ΔX=%.0fmm  pitch %.1f→%.1f (漂移 %.1f°)" % (
                    s["t0"] / 1000, s["t1"] / 1000, dx, ps[0], ps[-1], ps[-1] - ps[0]))

    # ---- 武装链验证（折叠/中位后摇杆失效）----
    W("")
    W("[武装链]  (折叠/中位 → 完成 → 自动重武装 是否闭环)")
    arm_lines = [r for r in events if any(k in r["line"] for k in
                 ("重武装", "武装", "锁定", "中位", "折叠位"))]
    for r in arm_lines:
        W("  %8.2fs  %s" % (r["t"] / 1000.0, r["line"][:80]))
    has_motion = any("折叠位" in r["line"] or "中位" in r["line"] for r in arm_lines)
    has_rearm = any("重武装" in r["line"] for r in arm_lines)
    if has_motion and not has_rearm:
        W("  ⚠ 有中位/折叠触发但没有'自动重武装'打印 ⇒ 自动重武装未生效（摇杆失效的直接解释）")
    W("=" * 64)


# ============================================================
# 主流程
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="RC 遥控全链路监控")
    ap.add_argument("--ip", default="192.168.7.120")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--interval", type=float, default=0.025, help="UDP 轮询间隔秒, 默认 0.025 (~40Hz)")
    ap.add_argument("--uart", default="COM11")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-uart", action="store_true", help="不连串口, 只采 UDP")
    ap.add_argument("--keysw", type=int, default=600, help="rc keysw 窗口秒, 0=不发")
    ap.add_argument("--secs", type=float, default=0, help="限时秒, 0=手动 Ctrl+C/q 结束")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--replay", default=None, help="只分析已采的 jsonl")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.replay:
        recs = load_jsonl(args.replay)
        lines = []
        analyze(recs, {}, lines)
        txt = "\n".join(lines)
        print(txt)
        outp = os.path.splitext(args.replay)[0] + "_summary.txt"
        with open(outp, "w", encoding="utf-8") as f:
            f.write(txt + "\n")
        print("\n→ 已写 %s" % outp)
        return 0

    outdir = args.outdir or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "outputs", "rc_monitor_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    w = JsonlWriter(os.path.join(outdir, "log.jsonl"))
    stats = {"udp_ok": 0, "udp_miss": 0, "udp_err": 0}
    limits_box = {}

    poller = StatusPoller(args.ip, args.port, args.interval, w, stats)
    poller.start()
    print("→ UDP 姿态流: %s:%d @%.0fHz" % (args.ip, args.port, 1.0 / args.interval))

    snif = None
    if not args.no_uart:
        snif = UartSniffer(args.uart, args.baud, w, limits_box, args.keysw)
        err = snif.start_port()
        if err:
            print("⚠ 串口不可用(%s) —— 降级为只采 UDP。事件流将缺失!" % err)
            snif = None
        else:
            snif.start()
            print("→ 串口旁听: %s @%d (已发 L + rc keysw %ds, 均为白名单只读/非阻塞)"
                  % (args.uart, args.baud, args.keysw))

    print("→ 输出: %s" % outdir)
    print("→ 操作提示: 现在拿手柄操作。需要精确对时就在动作瞬间敲回车(可带文字说明),")
    print("  敲 q 或 Ctrl+C 结束并自动分析。\n")

    t_end = time.time() + args.secs if args.secs > 0 else None
    mark_q = queue.Queue()

    def stdin_loop():
        while True:
            try:
                s = input()
            except EOFError:
                return
            mark_q.put(s)

    threading.Thread(target=stdin_loop, daemon=True).start()
    last_show = 0.0
    try:
        while True:
            if t_end and time.time() > t_end:
                break
            # 人工标记
            try:
                s = mark_q.get_nowait()
                if s.strip().lower() in ("q", "quit", "exit"):
                    break
                w.write({"src": "mark", "text": s.strip()})
                print("  ✎ 标记 @%.2fs: %s" % (now_ms() / 1000.0, s.strip()))
            except queue.Empty:
                pass
            # 状态行 (2s 一条)
            if time.time() - last_show > 2.0:
                last_show = time.time()
                lst = poller.last
                n_evt = snif.n_lines if snif else 0
                tot = stats["udp_ok"] + stats["udp_miss"]
                if lst and lst.get("j") and all(v is not None for v in lst["j"]):
                    print("  [%6.1fs] J1..J6= %s  rtt=%.1fms ok=%.0f%% uart行=%d" % (
                        now_ms() / 1000.0,
                        " ".join("%6.1f" % v for v in lst["j"]),
                        lst.get("rtt", -1), 100.0 * stats["udp_ok"] / max(1, tot), n_evt))
                else:
                    print("  [%6.1fs] (UDP 无有效样本: miss=%d err=%d —— 板子掉线?)" % (
                        now_ms() / 1000.0, stats["udp_miss"], stats["udp_err"]))
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    poller.stop()
    if snif:
        snif.stop()
    time.sleep(0.3)

    recs = load_jsonl(os.path.join(outdir, "log.jsonl"))
    lines = []
    analyze(recs, limits_box, lines)
    txt = "\n".join(lines)
    print("\n" + txt)
    sum_path = os.path.join(outdir, "summary.txt")
    with open(sum_path, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print("\n→ 数据: %s\n→ 摘要: %s" % (
        os.path.join(outdir, "log.jsonl"), sum_path))
    w.close()
    return 0


# ============================================================
# 离线自检: 本地假臂 UDP + 事件正则 + 运动段/分析纯函数
# ============================================================
def selftest() -> int:
    ok = [0]
    fail = [0]

    def chk(name, cond, extra=""):
        if cond:
            ok[0] += 1
            print("  PASS  %s" % name)
        else:
            fail[0] += 1
            print("  FAIL  %s %s" % (name, extra))

    # 1) 假臂: 回 GET_STATUS
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    srv.settimeout(0.5)
    port = srv.getsockname()[1]
    state = {"j": [180.0] * 6, "on": True}

    def fake():
        while state["on"]:
            try:
                data, addr = srv.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) >= 3 and data[1] == 0x02 and data[2] == 0:
                crc = ch.crc16(data[:3])
                pkt = data[:3] + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
                if pkt == data:
                    j = state["j"]
                    st = (ch.payload_stream_joints(j) +
                          bytes([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]))
                    rep = bytes([data[0], 0x82, len(st)]) + st
                    c = ch.crc16(rep[:3 + len(st)])
                    srv.sendto(rep + bytes([c & 0xFF, c >> 8]), addr)
    threading.Thread(target=fake, daemon=True).start()

    w = JsonlWriter(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "_rcmon_selftest.jsonl"))
    stats = {"udp_ok": 0, "udp_miss": 0, "udp_err": 0}
    p = StatusPoller("127.0.0.1", port, 0.005, w, stats)
    p.start()
    time.sleep(0.5)
    state["j"] = [181.0, 180, 180, 180, 180, 180]  # 制造一段运动
    time.sleep(0.4)
    p.stop()
    w.close()
    chk("UDP 轮询收到样本", stats["udp_ok"] > 10, str(stats))
    recs = load_jsonl(w.fh.name if hasattr(w, "fh") else "")
    os.remove(w.fh.name if hasattr(w, "fh") else "")
    chk("jsonl 可回读", len(recs) > 10)
    segs = movement_segments(recs)
    chk("运动段检测 ≥1 段", len(segs) >= 1, str(len(segs)))
    if segs:
        chk("运动段含 J1", 0 in segs[-1]["axes"])

    # 2) 事件正则
    for s, want in (("  [keys] 双击 START: RC 武装, 摇杆/按键生效", "keys"),
                    ("  [keys] 折叠位:", "keys"),
                    ("[STALL] J3", "fault"),
                    ("  Move started: v=35 deg/s", "move")):
        evt = "keys" if EVT_KEYS.search(s) else ("fault" if EVT_FAULT.search(s)
              else ("move" if EVT_MOVE.search(s) else "other"))
        chk("正则 %r→%s" % (s[:24], want), evt == want, evt)

    # 3) 限位正则
    m = RE_LIMIT.search("  J1  63~303   J2  75~284")
    chk("限位行解析", bool(m) and m.group(1) == "1")

    print("\n  自检: %d PASS / %d FAIL" % (ok[0], fail[0]))
    return 1 if fail[0] else 0


if __name__ == "__main__":
    sys.exit(main())
