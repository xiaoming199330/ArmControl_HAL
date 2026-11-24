# -*- coding: utf-8 -*-
"""
_common — 遥操 / 示教启动器的公共部分：配置、连接、预检、安全确认、步骤横幅。

单独放一个文件是为了让 `launch_teleop.py` / `launch_teach.py` 保持短小、
只写"各自那条流程"，公共的安全规则只有一份（改一次两处都生效）。
"""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# 让 `python launchers/xxx.py` 无需先 pip install 也能 import arm_sdk
for _p in (HERE, os.path.dirname(HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

CONFIG_NAME = "arm_link_config.json"

DEFAULT_CONFIG = {
    "_comment": "改这个文件即可，不用改代码。所有字段都有默认值，缺项会取默认。",
    "leader": "net://192.168.7.121:5000",
    "follower": "net://192.168.7.120:5000",
    "timeout_s": 0.3,
    "hz": 50.0,
    "slew_deg_per_s": 240.0,
    "gain": 1.0,
    "sync": "hold",
    "reverse": [False, False, False, False, False, False],
    "soft_limits_deg": None,
    "enable_torque": True,
    "serial": {"leader_port": "COM9", "follower_port": "COM11", "baud": 115200},
    "teach": {
        "out_dir": "trajectories",
        "name": "demo",
        "duration_s": 30.0,
        "playback_speed": 0.5,
        "playback_loop": 1,
    },
}

TOTAL_STEPS = 6


# ---------------------------------------------------------------- 输出小工具
class C:
    """终端配色（Windows 10+ 的 conhost / Windows Terminal 都认 ANSI）。"""
    R = "\033[0m"
    B = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[31m"
    GRN = "\033[32m"
    YEL = "\033[33m"
    BLU = "\033[36m"


def enable_ansi() -> None:
    """把控制台调成能吃下本脚本输出的样子。**必须在任何 print 之前调**。

    做三件事（顺序不能换）：
    1. Windows：把**控制台代码页**改成 65001（UTF-8）。这样不管用户是双击
       .bat、从 PowerShell 起、还是直接 `python launch_teleop.py`，都不用先
       自己敲 `chcp`。
    2. 把 Python 的 stdout/stderr 显式设成 UTF-8 + `errors="replace"`。
       ⚠ 这是**防崩**的关键：输出里有 `✓ ✗ ⚠ ═ ┌ │` 这些**不在 GBK 里**的字符，
       中文 Windows 默认 cp936 ⇒ 直接 UnicodeEncodeError 把脚本打死。
       宁可显示成 `?` 也不能崩。
    3. 打开 Windows 老 conhost 的 ANSI/VT 解析（否则颜色码会当成乱码打出来）。
    """
    if os.name == "nt":
        try:
            import ctypes
            k = ctypes.windll.kernel32
            k.SetConsoleOutputCP(65001)
            k.SetConsoleCP(65001)
        except Exception:                                          # noqa: BLE001
            pass

    for stream in (getattr(sys, "stdout", None), getattr(sys, "stderr", None)):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:                                          # noqa: BLE001
            pass

    if os.name != "nt":
        return
    try:
        import ctypes
        k = ctypes.windll.kernel32
        h = k.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if not k.GetConsoleMode(h, ctypes.byref(mode)):
            raise OSError("GetConsoleMode failed")
        if not k.SetConsoleMode(h, mode.value | 0x0004):
            raise OSError("SetConsoleMode failed")
    except Exception:                                              # noqa: BLE001
        for a in ("R", "B", "DIM", "RED", "GRN", "YEL", "BLU"):
            setattr(C, a, "")


def step(n: int, title: str) -> None:
    print("\n%s[%d/%d]%s %s%s%s" % (C.BLU, n, TOTAL_STEPS, C.R, C.B, title, C.R))


def ok(msg: str) -> None:
    print("   %s✓%s %s" % (C.GRN, C.R, msg))


def warn(msg: str) -> None:
    print("   %s⚠%s %s" % (C.YEL, C.R, msg))


def bad(msg: str) -> None:
    print("   %s✗%s %s" % (C.RED, C.R, msg))


def info(msg: str) -> None:
    print("   %s%s%s" % (C.DIM, msg, C.R))


# ---------------------------------------------------------------- 配置
def load_config(path: str | None) -> tuple[dict, str]:
    """读配置；不存在就**写一份带默认值的**出来，方便用户直接改。"""
    p = path or os.path.join(HERE, CONFIG_NAME)
    if not os.path.exists(p):
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            json.dump(DEFAULT_CONFIG, f, ensure_ascii=False, indent=2)
        return dict(DEFAULT_CONFIG), p
    with open(p, encoding="utf-8") as f:
        raw = json.load(f)
    cfg = dict(DEFAULT_CONFIG)
    cfg.update({k: v for k, v in raw.items() if not k.startswith("_")})
    return cfg, p


def build_teleop_config(cfg: dict, args, *, verbose=True):
    """把配置文件 + 命令行覆盖合成 `TeleopConfig`。

    用 `getattr(..., None)` 取命令行的值 ⇒ 两个启动器**不必**定义同一套参数
    （比如示教那条没有 `--gain` 也不会炸）。
    """
    from arm_sdk import TeleopConfig
    sl = getattr(args, "soft_limit", None) or cfg.get("soft_limits_deg")
    limits = None
    if sl:
        limits = [(float(a), float(b)) for a, b in sl]
    rev = cfg.get("reverse") or False
    gain = getattr(args, "gain", None)
    return TeleopConfig(
        hz=float(getattr(args, "hz", None) or cfg.get("hz", 50.0)),
        slew_deg_per_s=float(getattr(args, "slew", None)
                             or cfg.get("slew_deg_per_s", 240.0)),
        gain=float(gain if gain is not None else cfg.get("gain", 1.0)),
        sync=getattr(args, "sync", None) or cfg.get("sync", "hold"),
        reverse=rev,
        soft_limits_deg=limits,
        enable_torque=bool(cfg.get("enable_torque", True)),
        verbose=verbose,
    )


def parse_soft_limit(txt):
    """解析 `--soft-limit`。

    两种写法：`20,20,20,20,70,33` → 各轴 ±该值；
             `-20:20 -20:20 ...`    → 6 组显式 min:max。
    """
    if not txt:
        return None
    txt = str(txt).replace(",", " ").strip()
    if ":" in txt:
        out = []
        for v in txt.split():
            lo, _, hi = v.partition(":")
            out.append([float(lo), float(hi)])
        if len(out) != 6:
            raise SystemExit("--soft-limit 需要 6 组 min:max，收到 %d 组" % len(out))
        return out
    vals = [float(v) for v in txt.split()]
    if len(vals) != 6:
        raise SystemExit("--soft-limit 需要 6 个值（或 6 组 min:max），收到 %d 个"
                         % len(vals))
    return [[-abs(v), abs(v)] for v in vals]


# ---------------------------------------------------------------- 连接与预检
def connect(uri: str, timeout: float):
    """连一台臂。返回 (arm, None) 或 (None, 错误说明)。"""
    from arm_sdk import open_arm
    try:
        return open_arm(uri, timeout=timeout), None
    except Exception as e:                                         # noqa: BLE001
        return None, "%s: %s" % (type(e).__name__, e)


# ---------------------------------------------------------------- 文本解析
_FW_RE = None


def parse_fw_tag(text: str) -> str | None:
    """从 `h` 的输出里抽 fw-tag。

    固件 `h` 的首行形如 `fw-tag ARM-NET-0923A  pins=...  iface=...`；
    抽不出就退而找任何 `ARM-XXX-NNNNX` 样子的 token（老固件/仿真打印不同）。
    """
    global _FW_RE
    if _FW_RE is None:
        import re
        # 真固件是 ARM-NET-0923A / ARM-BOTH-0922A，仿真是 SIM-NET-0923A
        _FW_RE = re.compile(r"\b([A-Z]{2,4}-[A-Z]+-\d{4}[A-Z]?)\b")
    for line in (text or "").splitlines():
        low = line.lower()
        if "fw-tag" in low or "fw tag" in low:
            m = _FW_RE.search(line)
            return m.group(1) if m else line.strip()[:80]
    m = _FW_RE.search(text or "")
    return m.group(1) if m else None


def is_uncalibrated(text: str) -> bool:
    """`zi` 的输出是否表示"零点未校准"。"""
    t = text or ""
    return ("⚠" in t) or ("未校准" in t) or ("有轴为 0" in t)


def clean_lines(text: str) -> list:
    """去掉仿真/老固件那种 "Unknown command ..." 噪声行，只留真信息。"""
    keep = []
    for line in (text or "").splitlines():
        s = line.strip()
        if not s:
            continue
        if "Unknown command" in s or "Type 'h' for help" in s:
            continue
        keep.append(s)
    return keep


def preflight(arm, uri: str, label: str) -> dict:
    """一台臂的健康检查。返回 dict（不抛异常）—— 启动器据此决定要不要继续。

    检查项与 `armctl preflight` 对齐，另加两条**只有网口侧才查得到**的：
    fw-tag（`h`）和零点是否被污染（`zi`），它们决定"能不能信这台臂的角度"。
    """
    r = {"label": label, "uri": uri, "ok": True, "notes": [],
         "fw_tag": None, "calibrated": None, "joints": None,
         "estop": None, "read_ok": None, "failed": []}

    try:
        pi = arm.ping()
    except Exception as e:                                         # noqa: BLE001
        r["ok"] = False
        bad("%s PING 抛异常：%s" % (label, e))
        return r
    if pi is None:
        r["ok"] = False
        bad("%s PING 无响应" % label)
        info("查：①本机构建含网口吗 ②`link` 偏好是不是 net（改完要 save + RESET）"
             "③目的端口必须 5000 ④IP/网线/交换机 ⑤防火墙放行 UDP 5000")
        return r
    ok("%s PING 通  proto_ver=%d run_id=%d" % (label, pi.proto_ver, pi.run_id))

    try:
        st = arm.get_status()
        r["joints"] = st.as_servo_deg()
        r["estop"] = st.estop
        r["read_ok"] = st.read_ok
        r["failed"] = st.failed_joints()
        txt = ", ".join("--" if v is None else "%.1f" % v for v in r["joints"])
        if r["failed"]:
            r["ok"] = False
            bad("%s 有轴读失败 J%s ⇒ 舵机度：%s" % (label, r["failed"], txt))
            info("先排总线/接线再运动。「读失败」不能当 0 用（铁律）。")
        else:
            ok("%s 舵机度：%s" % (label, txt))
        if st.estop:
            r["ok"] = False
            bad("%s 处于 E-Stop —— 现在动不了（先排除急停原因再复位）" % label)
        if not st.read_ok:
            r["ok"] = False
            bad("%s read_ok=0 ⇒ FK 位姿字段无效，别当真值用" % label)
    except Exception as e:                                         # noqa: BLE001
        r["ok"] = False
        bad("%s GET_STATUS 失败：%s" % (label, e))

    # ---- 只有带 TEXT_EXEC 的固件才查得到（老固件会失败，属正常）----
    try:
        r["fw_tag"] = parse_fw_tag(arm.text_exec("h"))
        if r["fw_tag"]:
            ok("%s fw-tag：%s" % (label, r["fw_tag"]))
        elif uri.startswith("sim"):
            info("%s 仿真臂不打印 fw-tag（真机的 `h` 首行才有）" % label)
        else:
            warn("%s 没从 `h` 里抽到 fw-tag —— 用串口敲一次 `h` 人工确认" % label)
    except Exception as e:                                         # noqa: BLE001
        r["notes"].append("h")
        warn("%s 读 fw-tag 失败（固件可能没有 TEXT_EXEC 0x0B，改用串口敲 `h`）：%s"
             % (label, e))

    try:
        z = arm.text_exec("zi")
        if is_uncalibrated(z):
            r["calibrated"] = False
            r["ok"] = False
            bad("%s 零点未校准（`zi` 报有轴为 0）⇒ 逻辑角基准是脆的，遥操不能用" % label)
            info("先在本机串口按 `c` 做校准向导，确认 Home 姿态再用。")
        else:
            r["calibrated"] = True
            ok("%s 零点已校准（`zi` 正常）" % label)
    except Exception as e:                                         # noqa: BLE001
        r["notes"].append("zi")
        warn("%s 读 `zi` 失败：%s" % (label, e))

    return r


def print_limit_hint(arm, label: str) -> None:
    """打印运行时限位（`L`）—— 只是信息，不阻塞。

    ⚠ 仿真/老固件的文本解释器可能没有 `L`，会回 "Unknown command" —— 过滤掉，
       不让它淹没真正的输出。
    """
    try:
        for line in clean_lines(arm.text_exec("L"))[:4]:
            info("%s L | %s" % (label, line[:96]))
    except Exception:                                              # noqa: BLE001
        pass


# ---------------------------------------------------------------- 安全确认
def ask_yes(prompt: str) -> bool:
    """交互式确认。非交互（管道/双击但无输入）时一律**返回 False** —— 宁可不跑。"""
    if not sys.stdin or not sys.stdin.isatty():
        print("   （当前不是交互终端 ⇒ 不能替你确认，请加 --yes 明确授权）")
        return False
    try:
        return input("   %s [y/N] " % prompt).strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print("")
        return False


def safety_gate(label: str, assume_yes: bool) -> bool:
    """动臂前的硬闸门。列出必须人工确认的四条，缺一条就不放行。"""
    print("")
    print("   %s┌─ 动臂前必须确认（缺一条都不要继续）─────────────────────┐%s" % (C.YEL, C.R))
    for i, t in enumerate([
        "工作空间内**没有人**，臂扫过的区域清空",
        "E-Stop 就在手边（从臂本机 / 网络 STOP / 断电三选一）",
        "两臂都已用 `c` 校准，且 Home 零点对应**同一物理姿态**",
        "从臂跟随开关是开的（`sf 1`，上电默认就是 1）",
    ], 1):
        print("   %s│%s %d. %s" % (C.YEL, C.R, i, t))
    print("   %s└───────────────────────────────────────────────────────┘%s"
          % (C.YEL, C.R))
    if assume_yes:
        warn("--yes 已指定 ⇒ 跳过人工确认（请自行确保上面 4 条都成立）")
        return True
    return ask_yes("以上 4 条都确认了吗？输入 y 继续，其它任意键退出")
