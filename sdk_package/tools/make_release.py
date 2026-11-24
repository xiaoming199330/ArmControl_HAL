#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_release.py — 把 sdk_package/ 组装成**两个**交付包 + 一份独立文档。

为什么是两个包
--------------
原来只有一个 `arm_sdk_linux_<version>`，里面混了两类人需要的东西：

  A. **写上位机软件的人**（开发方）——要读懂协议、写代码、调 IK/3D
  B. **把 SDK 装到 Linux 机器上跑的人**（部署方）——要装包、连臂、验收

两拨人的入口不一样。混在一个包里，部署方会在 15 MB 的 STL 和 7 份研发参考文档里
找不到"我该敲哪条命令"。所以拆成：

    release/arm_sdk_dev_<version>/      ← 开发包（含示例 / 模型 / 研发参考）
    release/arm_sdk_linux_<version>/    ← 部署包（含 wheel / 部署指南，不含上面那些）

外加一份**独立交付物**（不进包，谁要给谁）：

    release/三链路延迟基准_简版_<date>.md

分包判据（写在这个脚本里，不靠人记）
--------------------------------
    这个文件是"**写代码用**"还是"**跑起来用**"？
      写代码用            → 只进开发包（examples / model / docs/reference）
      跑起来 / 排障 / 验收 → 只进部署包（dist wheel / Linux部署与打包.md）
      两拨人都要          → 两边都放（arm_sdk 源码 / js / 契约 JSON / 协议文档 / tests）

**网口协议文档两个包都有**（`网口协议速查.md` 精简版 + `网口指令大全.md` 全量版）。

用法
----
    python tools/make_release.py                  # 默认版本 2.0.0
    python tools/make_release.py --version 2.0.1
    python tools/make_release.py --only dev       # 只出开发包
    python tools/make_release.py --only linux     # 只出部署包

产出
----
    release/arm_sdk_dev_<version>/       + .tar.gz + .zip
    release/arm_sdk_linux_<version>/     + .tar.gz + .zip
    release/三链路延迟基准_简版_<date>.md
    release/SHA256SUMS.txt               # 4 个归档 + 独立文档的指纹
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
import tarfile
import zipfile
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                        # = sdk_package/
PROJ = os.path.dirname(ROOT)                        # = 工程根（docs/ 在这下面）
RELEASE_DIR = os.path.join(ROOT, "release")

# 源路径前缀：@proj/ = 相对工程根（RD 侧文档），其余 = 相对 sdk_package/
PROJ_PREFIX = "@proj/"

# --------------------------------------------------------------------------
# 两包共有的东西
# --------------------------------------------------------------------------
COMMON = [
    ("arm_sdk", "arm_sdk", True,
     "★ SDK 源码。分层：protocol.py（逻辑包编解码）→ transport.py（UDP/串口/CAN）→ "
     "client.py（ArmClient）；teleop / teach / kinematics / console / sim / cli。"
     "另含 `mock_server.py`（假臂，无真机联调）与 `http_bridge.py`（HTTP↔UDP 边车，"
     "`armctl http-bridge`）"),

    ("__WHEEL__", "dist", True,
     "★ 预构建 wheel（py3-none-any）—— Linux 上直接 pip install，免编译"),

    ("pyproject.toml", "pyproject.toml", True,
     "★ pip 安装声明（console_scripts: armctl）；`pip install -e .` 走它"),

    ("README.md", "README.md", True, "★ SDK 总说明（30 秒上手 / 10 个坑 / 自检表）"),

    ("js", "js", True,
     "★ JS SDK（零依赖，Node ≥ 18）：Node 原生 UDP + 浏览器 Web Serial + "
     "bridge.js 边车；与 Python 版逐字节一致（跨语言对拍 404 项）"),

    ("config/arm_config.json", "config/arm_config.json", True,
     "★ 接口契约（114 文本命令 + 38 网口命令 + 40 章节）。"
     "**参数不要硬编码，从这里读**"),

    ("tests", "tests", True,
     "★ 离线端到端验收测试（integration_test 51 + new_commands_test 62 断言；"
     "无 numpy 时少 1 项（.npz 往返），会打印跳过）"),

    # ---- 配置文件说明：两个包都必须有 ----
    ("docs/配置文件说明.md", "docs/配置文件说明.md", True,
     "★【配置文件】两个 JSON 的分工与全字段：`config/arm_config.json`（只读接口契约，39 节）"
     "vs `launchers/arm_link_config.json`（你要改的部署配置）；含「改完怎么验」与三处非 JSON 配置"),

    # ---- 网口协议：两个包都必须有（用户明确要求）----
    ("docs/网口协议速查.md", "docs/网口协议速查.md", True,
     "★【网口协议】字段级速查（14 章 / 463 行）—— 第一优先级，先看这份"),
    (PROJ_PREFIX + "docs/网口指令大全.md", "docs/网口指令大全.md", True,
     "★【网口协议】全量版（1419 行）：38 条命令（14 基础 + 24 扩展）逐条详解 + TEXT_EXEC 文本透传 + 故障对照表 + 13 条网口红线"),

    (PROJ_PREFIX + "docs/网口下发操作手册_2026-09-30.md", "docs/网口下发操作手册.md", True,
     "★【网口上手】操作步骤在前（探活→观测→运动→收工）+ armctl 全量命令注释 + 真实回传样例注解——拿到包先看这份再动臂"),
    (PROJ_PREFIX + "docs/串口下发操作手册_2026-09-30.md", "docs/串口下发操作手册.md", True,
     "★【串口上手】screen 控制台：操作步骤在前（判活→状态→运动）+ 固件原生命令注释 + 真机 cprint 格式回显注解"),

    ("docs/主从遥操与示教.md", "docs/主从遥操与示教.md", True,
     "★ 主从遥操与遥操示教：选型、参数、排查"),

    (PROJ_PREFIX + "docs/UDP_vs_HTTP_实测_2026-09-24.md", "docs/UDP_vs_HTTP_实测.md", True,
     "★【链路选型】UDP vs HTTP 的速率/延迟/丢包对照：为什么板子没 TCP、HTTP 边车多付多少、"
     "板侧 nx 差分测真丢包的方法、以及「遥操走 UDP / 配置走 HTTP」的结论"),

    # ---- ★ 上位机对接包：写代码的人第一份要看的东西 ----
    ("host_integration", "host_integration", True,
     "★★【上位机对接】场景→发哪条指令决策表（502 行）+ 自动生成的协议规格 JSON（38 条命令）"
     "+ 三份参考实现（C 头文件 / C# ArmLink / 零依赖 Python 独立实现）+ 假臂联调用法"),

    # ---- ★ 一键启动文件：不写代码也要能跑起来 ----
    ("launchers", "launchers", True,
     "★★【一键启动】主从遥操 / 遥操示教启动器（6 步打印 + 预检 + 安全闸门）+ 唯一配置入口 "
     "arm_link_config.json + Windows/Linux 启动脚本 + 《启动步骤.md》；带 --preflight-only 只读预检"),
]

# --------------------------------------------------------------------------
# 只有【开发包】要的：写上位机软件用
# --------------------------------------------------------------------------
DEV_ONLY = [
    ("examples", "examples", True,
     "7 个可运行示例（01~07，含 sim 无硬件版）—— 写代码的样板"),

    ("model", "model", False,
     "URDF + link_geometry.json + 15 个 STL（15.4 MB）—— 上位机做 **3D 显示 / IK 校验**才要。"
     "只做协议通讯不用它"),

    ("01_ARCHITECTURE(系统总体架构).md",
     "docs/reference/01_ARCHITECTURE(系统总体架构).md", False, "系统总体架构"),
    ("02_MECHANICAL(机械臂尺寸与运动学).md",
     "docs/reference/02_MECHANICAL(机械臂尺寸与运动学).md", False,
     "机械尺寸 + 运动学（画 3D / 算 IK 用）"),
    ("03_INTERFACE(详细接口数据).md",
     "docs/reference/03_INTERFACE(详细接口数据).md", False,
     "详细接口数据（串口文本命令全表 + 寄存器 + 二进制协议）"),
    ("04_SDK_SPEC(SDK规格书).md",
     "docs/reference/04_SDK_SPEC(SDK规格书).md", False,
     "串口控制台 SDK 规格书 ⚠ §6「主从臂不在范围」已被 v2 取代"),
    ("05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md",
     "docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md", False,
     "★ 上位机**功能范围与验收标准** —— 开发方的需求依据"),
    ("AI_BRIEF(上位机软件制作任务要求参考文件).md",
     "docs/reference/06_AI_BRIEF(上位机任务书).md", False,
     "上位机任务书（交给 AI 做上位机时的输入）"),
    ("READM（该开发包摘要）E.md", "docs/reference/07_开发包摘要.md", False,
     "开发包摘要（09-11 版，整体背景）"),
]

# --------------------------------------------------------------------------
# 只有【部署包】要的：装到 Linux 上跑
# --------------------------------------------------------------------------
LINUX_ONLY = [
    ("docs/Linux部署与打包.md", "docs/Linux部署与打包.md", True,
     "★ Linux 部署指南：安装三法 / 离线部署 / udev 固定串口名 / 网络调优 / "
     "systemd / Docker / 升级回滚 / 常见问题"),
]

# --------------------------------------------------------------------------
# 独立交付物（不进任何包）：(源, 包外文件名, 说明)
# --------------------------------------------------------------------------
STANDALONE = [
    ("@proj/docs/三链路延迟基准_简版_2026-09-22.md",
     "三链路延迟基准_简版_2026-09-22.md",
     "★ 三链路（网口/CAN/串口）延迟测试**结论页** —— 494 行完整报告压成 90 行"),
]

# --------------------------------------------------------------------------
# 明确排除：(路径, 原因) —— 两个包都不放
# --------------------------------------------------------------------------
EXCLUDE = [
    ("python/", "⚠ 已被取代：旧版**纯串口** SDK（arm_sdk.py / arm_fk.py）。"
                "能力已并入 arm_sdk/console.py + arm_sdk/kinematics.py。"
                "带两份会让部署方不知道 import 哪个。"),
    ("web/", "⚠ 已被取代：旧版**纯串口**浏览器 SDK（Web Serial，不支持网口/遥操）。"
             "要做 Web/Electron 且走网口 → 用包内 `js/`。"),
    ("tools/", "研发侧工具（打包器 / 配置生成器 / 模型生成器），交付方不需要。"),
    ("data/", "与 config/arm_config.json **完全重复**，只保留一份。"),
    ("build/ · *.egg-info/", "打包中间产物，重新构建即可生成。"),
    ("__pycache__/", "字节码缓存。"),
]

SKIP_NAMES = {"__pycache__", ".pytest_cache", ".mypy_cache", ".ipynb_checkpoints"}
SKIP_EXTS = {".pyc", ".pyo", ".pyd", ".egg-info"}

# 目录内**只保留白名单**（键 = 相对 sdk_package/ 的路径，用 / 分隔）。
# 用途：把"本地跑出来的测试数据"挡在交付包外，但保留目录与说明文件。
KEEP_ONLY = {
    # 示教输出目录：只发 README.txt，不收本地录的 smoke 轨迹
    "launchers/trajectories": {"README.txt"},
}


# ==========================================================================
# 基础工具
# ==========================================================================
def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def resolve_src(src_rel: str) -> str:
    """支持 @proj/ 前缀；其余相对 sdk_package/。"""
    if src_rel.startswith(PROJ_PREFIX):
        return os.path.join(PROJ, *src_rel[len(PROJ_PREFIX):].split("/"))
    return os.path.join(ROOT, *src_rel.split("/"))


def copy_tree(src: str, dst: str) -> int:
    n = 0
    for dirpath, dirnames, filenames in os.walk(src):
        dirnames[:] = [d for d in dirnames if d not in SKIP_NAMES
                       and not d.endswith(".egg-info")]
        rel = os.path.relpath(dirpath, src)
        target_dir = dst if rel == "." else os.path.join(dst, rel)
        # 白名单：命中则只放列出的文件名（见 KEEP_ONLY）
        keep = KEEP_ONLY.get(os.path.relpath(dirpath, ROOT).replace(os.sep, "/"))
        os.makedirs(target_dir, exist_ok=True)
        for fn in filenames:
            if fn in SKIP_NAMES or os.path.splitext(fn)[1] in SKIP_EXTS:
                continue
            if keep is not None and fn not in keep:
                continue
            shutil.copy2(os.path.join(dirpath, fn), os.path.join(target_dir, fn))
            n += 1
    return n


def _human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.1f %s" % (n, unit) if unit != "B" else "%d B" % int(n)
        n /= 1024.0
    return str(n)


def pick_wheel() -> str:
    d = os.path.join(ROOT, "dist")
    if not os.path.isdir(d):
        return ""
    whls = sorted((f for f in os.listdir(d) if f.endswith(".whl")),
                  key=lambda f: os.path.getmtime(os.path.join(d, f)))
    return os.path.join(d, whls[-1]) if whls else ""


def verify_wheel_fresh(whl: str) -> list[str]:
    """★ 防呆：whl 里的 arm_sdk/*.py 必须与当前源码逐字节一致。

    改了源码忘了重新 `pip wheel`，发出去的就是旧包 —— 而且**版本号还一样**，
    收包方根本看不出来。和固件里"重编后 md5 变了但 fw-tag 没变"是同一类事故。
    """
    problems: list[str] = []
    src_dir = os.path.join(ROOT, "arm_sdk")
    with zipfile.ZipFile(whl) as z:
        names = set(z.namelist())
        for fn in sorted(os.listdir(src_dir)):
            if not fn.endswith(".py"):
                continue
            arc = "arm_sdk/" + fn
            if arc not in names:
                problems.append("whl 里缺少 %s（源码有）" % arc)
                continue
            if z.read(arc) != open(os.path.join(src_dir, fn), "rb").read():
                problems.append("%s 与源码不一致（改了源码没重新构建 whl）" % fn)
        arc = "arm_sdk/arm_config.json"
        src = os.path.join(src_dir, "arm_config.json")
        if os.path.exists(src) and arc in names:
            if z.read(arc) != open(src, "rb").read():
                problems.append("arm_sdk/arm_config.json 与源码不一致")
    return problems


# ==========================================================================
# 组装一个包
# ==========================================================================
def build_pkg(name: str, items, starter, version: str, wheel: str):
    out = os.path.join(RELEASE_DIR, name)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out, exist_ok=True)

    manifest: list[tuple[str, bool, str, int]] = []
    missing_required: list[str] = []

    for src_rel, dst_rel, required, note in items:
        # ---- wheel 特例：只放最新那一个 ----
        if src_rel == "__WHEEL__":
            if not wheel:
                missing_required.append("dist/*.whl")
                print("  [跳过-缺失] dist/*.whl%s" % ("  ← 必需！" if required else ""))
                continue
            dst = os.path.join(out, "dist", os.path.basename(wheel))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(wheel, dst)
            manifest.append(("dist/%s" % os.path.basename(wheel), required, note,
                             os.path.getsize(dst)))
            print("  [必需] dist/%s" % os.path.basename(wheel))
            continue

        src = resolve_src(src_rel)
        dst = os.path.join(out, *dst_rel.split("/"))
        if not os.path.exists(src):
            if required:
                missing_required.append(src_rel)
            print("  [跳过-缺失] %s%s" % (src_rel, "  ← 必需！" if required else ""))
            continue
        if os.path.isdir(src):
            os.makedirs(dst, exist_ok=True)
            cnt = copy_tree(src, dst)
            size = sum(os.path.getsize(os.path.join(dp, f))
                       for dp, _dn, fns in os.walk(dst) for f in fns)
            manifest.append((dst_rel + "/（%d 个文件）" % cnt, required, note, size))
        else:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(src, dst)
            manifest.append((dst_rel, required, note, os.path.getsize(dst)))
        print("  [%s] %s" % ("必需" if required else "可选", dst_rel))

    if missing_required:
        print("\n❌ %s 缺少必需项，终止：%s" % (name, missing_required))
        return None, None, None

    with open(os.path.join(out, "START_HERE.md"), "w", encoding="utf-8") as f:
        f.write(starter(version, manifest))
    _write_manifest(out, name, manifest)

    return out, _archive_targz(out), _archive_zip(out)


def _write_manifest(out: str, name: str, manifest) -> None:
    lines = []
    A = lines.append
    A("%s  —  文件清单与校验和" % name)
    A("生成(UTC): %s" % datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"))
    A("=" * 100)
    A("")
    for path, req, note, size in manifest:
        A("[%s] %-52s %10s  %s" % ("必需" if req else "可选", path, _human(size), note))
    A("")
    A("-" * 100)
    A("逐文件 SHA256（只列代码/配置类，便于抽查一致性）")
    A("-" * 100)
    HASHED = (".py", ".json", ".toml", ".js", ".mjs", ".h", ".cs", ".csproj", ".bat", ".sh")
    for dirpath, dirnames, filenames in os.walk(out):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_NAMES)
        for fn in sorted(filenames):
            if not fn.endswith(HASHED):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, out).replace(os.sep, "/")
            A("%s  %s" % (sha256(full), rel))
    A("")
    with open(os.path.join(out, "MANIFEST.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


# ==========================================================================
# 两个 START_HERE
# ==========================================================================
_HARD_CONSTRAINTS = [
    "## 4. 部署前必须知道的硬约束（两个包的收件人都要看）",
    "",
    "1. **两套角度混用是头号 bug 源**：`GET_STATUS` / `FK_QUERY` / `MOVE_JOINTS` / `MOVE_CART` "
    "用**舵机度（0~360）**；`STREAM_JOINTS` / `#P` 用**逻辑角（相对 Home 零点）**。",
    "2. **点动必须发增量**（`--delta`）：发绝对角会被固件 `clamp_all_angles()` **静默钳制**，"
    "回 err=0 但值被改了。",
    "3. **静默钳制有三处**：`MOVE_JOINTS` 目标角 / `PARAM_WRITE` 值 / `STREAM_JOINTS` 单轴。"
    "要确认真值必须**回读**。",
    "4. **`err=0` 不代表动了**：读起点失败时固件也回 ACK 但轨迹没起步 ⇒ 判断“动没动”"
    "必须看 `GET_STATUS` 角度变化。",
    "5. **单帧跳变 >25° 会整帧拒收，且不更新基准 ⇒ 自锁冻结**。发送端必须限速"
    "（SDK 里 `TeleopSession` 已内置）。",
    "6. **断流 >500 ms** 从臂退出流模式并原地自锁 ⇒ 频率必须 >2 Hz，推荐 50 Hz。"
    "`STREAM_JOINTS` **丢帧不重发**。",
    "7. **读失败必须如实上报**：某轴返回 `None` / `-32768` / `read_ok=0` ⇒ "
    "**不要兜底、不要读位姿字段**。SDK 已强制这个行为。",
    "8. **两条响应的负载顺序相反**：`GET_STATUS` 先关节后位姿；`FK_QUERY` 先位姿后关节，"
    "且 offset 20 是**裸 0/1**，不是 bit3。",
    "9. **六种“一个字节都不回”**：LEN 不符 / 错端口 / 错 IP / `CMD>=0x80`（防死锁）/ "
    "CRC 错（回 NACK err=2）/ 本固件构建没开网口。",
    "10. **链路不热切换**：`link net` → `link save` → **按 RESET**。"
    "板子默认**两条链路都关**，只留串口。",
    "11. **CAN 与网口物理互斥**（`PB13 = ETH_TXD1 = CAN2_TX`）。串口恒常通。",
    "12. **烧录/复位必丢力矩、臂会下沉**（实测 J2 曾掉到距下限仅 1.3°）。"
    "上运动测试前先回中位；**绝对不要用 `M a`**（那是标定，会废掉量纲基准）。",
    "",
]

_FIELD_INFO = [
    "## 5. 现场信息（直接填进你的配置）",
    "",
    "| 项 | 值 |",
    "|---|---|",
    "| 从臂 IP:端口 | `192.168.7.120:5000`（UDP） |",
    "| 主臂 IP:端口 | `192.168.7.121:5000`（UDP） |",
    "| 串口 | USART6，**115200 8N1 无流控**，行尾统一 `\\r\\n` |",
    "| CAN | 2.0A，**500 kbit/s**，命令 `0x100+node`，响应 `0x180+node` |",
    "| 板内固件（三构建，2026-09-29 重编） | `ARM-NET-0924AV` 263,128 B md5 `4c74c6177fd39d2794107fd871e8c2cb`；"
    "`ARM-CAN-0918H` 215,096 B md5 `cd11183e6d6aa59cf81cad77c6ca3b5c`；"
    "`ARM-BOTH-0924AV` 263,092 B md5 `d55f6202f20825936b31ad7481276da6` |",
    "| ⚠ fw-tag 两处硬编码 | 控制台 `h` 与 `GET_INFO(0x07)` 现已**同步为 0924AV**，但仍是 main.c / "
    "arm_control_api.c 各自硬编码 —— 固件升版时必须两处同改；上位机**以 `h` / PING 为准** |",
    "| 网口前提 | 必须 `link net` → `link save` → RESET 后网口才通 |",
    "| 运行时限位（`L`） | J1 63~303 / J2 75~284 / J3 76~271 / J4 71~284 / J5 12~351 / J6 138~266 |",
    "",
]

_EXCLUDE_SECTION = [
    "## 6. 刻意没有放进来的东西（别去找）",
    "",
    "| 未包含 | 原因 |",
    "|---|---|",
] + ["| `%s` | %s |" % (p, w) for p, w in EXCLUDE] + [
    "",
]


def _pkg_table(manifest) -> list[str]:
    L = ["| 路径 | 必需 | 用途 | 体积 |", "|---|---|---|---|"]
    for path, req, note, size in manifest:
        L.append("| `%s` | %s | %s | %s |"
                 % (path, "是" if req else "可选", note, _human(size)))
    L.append("")
    return L


def start_here_dev(version: str, manifest) -> str:
    total = sum(m[3] for m in manifest)
    whl = next((m[0] for m in manifest if m[0].startswith("dist/")), "dist/*.whl")
    L = []
    A = L.append

    A("# 六轴机械臂 SDK — 开发包 %s" % version)
    A("")
    A("> **给写上位机软件的人。** 这一页看完就能开始写代码。")
    A("> 生成时间（UTC）：%s ｜ 包内 %d 项 / %s"
      % (datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M"), len(manifest), _human(total)))
    A("")
    A("> 📦 **本包是「开发包」。** 另一份 `arm_sdk_linux_v%s` 是给**部署方**的"
      "（少了示例 / 3D 模型 / 研发参考文档，多了 Linux 部署指南）。" % version)
    A("")
    A("---")
    A("")
    A("## 1. 三分钟上手")
    A("")
    A("```bash")
    A("# 装（三选一）")
    A("python3 -m pip install %s     # A. 装 wheel（最快，无编译）" % whl)
    A("python3 -m pip install -e .                    # B. 可编辑安装（要改源码用这个）")
    A("export PYTHONPATH=$PWD                         # C. 不装直接跑")
    A("")
    A("# 不接硬件也能跑：内存假臂")
    A("armctl --uri sim:// preflight                  # 上板前置检查（模拟器）")
    A("armctl --uri sim:// status                     # 读六轴角度")
    A("armctl info                                    # 能力/链路/配置摘要")
    A("```")
    A("")
    A("```python")
    A("from arm_sdk import open_arm")
    A("with open_arm('net://192.168.7.120:5000') as arm:")
    A("    print(arm.ping())          # 探活")
    A("    print(arm.get_status())    # 六轴状态")
    A("```")
    A("")
    A("```bash")
    A("python3 examples/02_offline_sim.py               # ① 先跑这个（不用硬件）")
    A("python3 examples/01_quickstart.py                # ② 网口最小例子")
    A("python3 examples/07_trajectory_tools.py          # ③ 轨迹 JSON/CSV/NPZ 转换")
    A("```")
    A("")
    A("### JS 侧（Node ≥ 18，**零依赖**，不需要 npm install）")
    A("")
    A("```bash")
    A("cd js && node src/cli.js selftest     # 971 项离线自检")
    A("node src/cli.js --uri sim:// ping     # 无硬件冒烟")
    A("```")
    A("")
    A("```js")
    A("import { openArm } from './js/src/index.js';")
    A("const arm = await openArm('net://192.168.7.120:5000');   // Node / Electron 主进程")
    A("console.log(await arm.ping());")
    A("```")
    A("")
    A("> ⚠ **浏览器发不了原始 UDP**（沙箱铁律，页面内没有绕法）。要网页控制机械臂，"
      "必须起边车：")
    A("> `node js/src/bridge.js --target 192.168.7.120:5000`，"
      "然后页面里 `openArm('ws://127.0.0.1:8765/')`。")
    A("> 边车**每个 WS 连接独占一个 UDP socket**，所以**一个进程能同时带主/从两路**"
      "（`ws://127.0.0.1:8765/?target=192.168.7.121:5000`）。详见 `js/docs/JS快速上手.md`。")
    A("")
    A("### 不写代码也能跑：一键启动主从遥操 / 遥操示教")
    A("")
    A("```bash")
    A("cd launchers")
    A("python install_deps.bat        # Windows 装依赖（Linux: ./install_deps.sh）")
    A("# 改 arm_link_config.json 里的 leader / follower 地址（默认 192.168.7.121 / .120:5000）")
    A("")
    A("python launch_teleop.py --preflight-only   # ① 只预检，不动臂（安全）")
    A("python launch_teleop.py                    # ② 正式起主从遥操（6 步打印 + 安全闸门）")
    A("")
    A("python launch_teach.py record --sec 30 --name demo   # ③ 录一段示教")
    A("python launch_teach.py list                          # ④ 列已有轨迹")
    A("python launch_teach.py play trajectories/demo.json --speed 0.5   # ⑤ 回放（先半速）")
    A("```")
    A("")
    A("Windows 双击 `start_teleop.bat` / `start_teach.bat` 也行（自带 `chcp 65001`，不会因中文乱码崩）。")
    A("**完整步骤、排错表、三条路线选型 →  `launchers/启动步骤.md`。**")
    A("")
    A("> ⚠ **遥操/示教是会让臂真动的。** 先跑 `--preflight-only`；第一次上真机把 "
      "`arm_link_config.json` 的 `gain` 调到 0.5、`slew_deg_per_s` 保持 240 不要加。")
    A("> 没接线也能全流程演练：把地址写成 `sim://`。")
    A("")
    A("### 想用 HTTP 调臂？起边车（板子没 TCP，边车跑在 PC 侧）")
    A("")
    A("```bash")
    A("armctl mock --port 5000                                   # 无真机时先起假臂")
    A("armctl http-bridge --target 192.168.7.120:5000 --port 8080 # 边车")
    A("curl http://127.0.0.1:8080/api/status                       # 就是普通 HTTP")
    A("```")
    A("")
    A("> 边车不改协议、不加语义：错误码/丢读哨兵/静默钳制与 UDP 直连**完全一致**。")
    A("> 每个响应带 `X-Arm-Rtt-Ms` ⇒ 能把「链路延迟」和「HTTP 开销」分开算。")
    A("> ⚠ **50 Hz 遥操流不要走 HTTP**（每帧等应答会把帧率锁死在 RTT 上）；"
      "急停也走 UDP。**遥操走 UDP，配置/查询走 HTTP**，两条路并存。")
    A("")
    A("**依赖**：核心纯标准库（网口 `net://` 什么都不用装）。可选：")
    A("")
    A("| 依赖 | 什么时候需要 | 装法 |")
    A("|---|---|---|")
    A("| `pyserial` | 走串口 `serial:///dev/ttyUSB0` | `pip install pyserial` |")
    A("| `python-can` | 直接驱动 USB-CAN 适配器（不走板子） | `pip install python-can` |")
    A("| `numpy` | 轨迹存 `.npz` | `pip install numpy` |")
    A("")
    A("---")
    A("")
    A("## 2. 包里有什么")
    A("")
    L.extend(_pkg_table(manifest))
    A("### 先读哪份文档")
    A("")
    A("| 你要干什么 | 看哪份 |")
    A("|---|---|")
    A("| ★★ 写代码：**这个功能该发哪条指令** | `host_integration/上位机对接指南.md` §3 场景决策表 |")
    A("| ★★ 拿协议真值写代码 / 生成代码 | `host_integration/protocol_spec.json`（38 条，机读） |")
    A("| ★★ 抄一份参考实现 | `host_integration/arm_protocol.h`（C）/ `csharp/ArmLink.cs`（C#）/ "
      "`minimal_client.py`（Python，零依赖） |")
    A("| ★ 搞懂网口协议（**第一优先级**） | `docs/网口协议速查.md`（463 行，字段级） |")
    A("| 排障 / 现象→原因对照 | `docs/网口指令大全.md` §11 故障对照表（1419 行全量版） |")
    A("| ★ 两个配置文件的分工与全字段 | `docs/配置文件说明.md` |")
    A("| 拿协议真值写代码 | `config/arm_config.json` → `arm_config.network_protocol` |")
    A("| 做 3D 显示 / 校验 IK | `model/arm_model.urdf` + `model/link_geometry.json` |")
    A("| 上位机功能范围与验收标准 | `docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md` |")
    A("| 整体架构 / 机械参数 | `docs/reference/01_...` / `02_...` |")
    A("| 主从遥操怎么接 | `docs/主从遥操与示教.md` |")
    A("| ★ **不写代码**，直接把遥操/示教跑起来 | `launchers/启动步骤.md` |")
    A("| ★ 不想自己处理 CRC/SEQ/分页 → 用 HTTP | `docs/UDP_vs_HTTP_实测.md` + `armctl http-bridge` |")
    A("| 链路选哪条、实测多快 | 独立文档《三链路延迟基准_简版》 |")
    A("")
    A("> **`config/arm_config.json` 是接口真值源**：114 条串口文本命令、38 条网口命令（字段级）、"
      "三条承载、链路开关、关节限位、舵机偏移、运行时参数表、错误码。"
      "**不要把这些硬编码进你的代码** —— 从 JSON 读，固件升级也不用改代码。")
    A("")
    A("---")
    A("")
    A("## 3. 验收：确认你手上的包是好的")
    A("")
    A("```bash")
    A("# ① Python：离线自检（不需要机械臂）")
    A("armctl selftest                    # 228 项（无 numpy 为 227，少的 1 项会打印跳过）")
    A("python3 tests/integration_test.py  # 51 项 (无 numpy 50)")
    A("python3 tests/new_commands_test.py # 62 项（2026-09-23 扩展命令）")
    A("")
    A("# ② JS：零依赖")
    A("cd js && npm test                  # 7 组 / 971 项（含 JS↔Python 逐字节对拍 + 真 UDP 回环）")
    A("```")
    A("")
    A("---")
    A("")
    L.extend(_HARD_CONSTRAINTS)
    A("---")
    A("")
    L.extend(_FIELD_INFO)
    A("---")
    A("")
    L.extend(_EXCLUDE_SECTION)
    A("> 要 **Starter / 部署** 相关内容 → 找 `arm_sdk_linux_v%s` 那份包。" % version)
    A("")
    A("---")
    A("")
    A("## 7. 出问题找谁")
    A("")
    A("* 协议 / 字段 / 错误码 → `docs/网口协议速查.md`，查不到 → `docs/网口指令大全.md`")
    A("* 主从遥操 / 示教 → `docs/主从遥操与示教.md`")
    A("* 上位机功能范围与验收口径 → `docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md`")
    A("* 以上都没有 → 回头找固件/上位机对接人（本包生成方）")
    A("")
    return "\n".join(L)


def start_here_linux(version: str, manifest) -> str:
    total = sum(m[3] for m in manifest)
    whl = next((m[0] for m in manifest if m[0].startswith("dist/")), "dist/*.whl")
    L = []
    A = L.append

    A("# 六轴机械臂 SDK — Linux 部署包 %s" % version)
    A("")
    A("> **给负责把 SDK 装到 Linux 机器上的人。** 先看这一页，再进 "
      "`docs/Linux部署与打包.md`。")
    A("> 生成时间（UTC）：%s ｜ 包内 %d 项 / %s"
      % (datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M"), len(manifest), _human(total)))
    A("")
    A("> 📦 **本包是「部署包」。** 写上位机软件的人要的是另一份 "
      "`arm_sdk_dev_v%s`（多给示例 / 3D 模型 / 上位机需求与验收标准）。" % version)
    A("")
    A("---")
    A("")
    A("## 1. 三分钟上手")
    A("")
    A("```bash")
    A("# ★ 方式 A（推荐）：装 wheel，纯 Python 无编译、零依赖")
    A("python3 -m pip install %s" % whl)
    A("")
    A("# 方式 B：源码可编辑安装（要打补丁 / 二次打包时用）")
    A("python3 -m pip install -e .")
    A("")
    A("# 方式 C：不安装，直接跑（零依赖也能用）")
    A("export PYTHONPATH=$PWD")
    A("python3 -m arm_sdk info")
    A("```")
    A("")
    A("```bash")
    A("armctl info                                    # 能力/链路/配置摘要")
    A("armctl selftest                                # 全部离线自检（无硬件：228 项，无 numpy 227）")
    A("armctl --uri sim:// preflight                  # 无硬件冒烟")
    A("")
    A("armctl --uri net://192.168.7.120:5000 ping     # ① 真机探活")
    A("armctl --uri net://192.168.7.120:5000 status   # ② 读六轴状态（只读，安全）")
    A("armctl --uri net://192.168.7.120:5000 preflight  # ③ 上板前置检查")
    A("```")
    A("")
    A("```bash")
    A("# 离线部署（目标机不能上外网时）")
    A("# 有网机器：python3 -m pip download --only-binary=:all: -d /tmp/whl pyserial")
    A("# 目标机：  python3 -m pip install --no-index --find-links=/tmp/whl pyserial")
    A("```")
    A("")
    A("### JS 侧（Node ≥ 18，**零依赖**，不需要 npm install）")
    A("")
    A("```bash")
    A("cd js && node src/cli.js selftest                      # 971 项离线自检")
    A("node src/cli.js --uri net://192.168.7.120:5000 status # 真机读状态")
    A("```")
    A("")
    A("> ⚠ **浏览器发不了原始 UDP**（沙箱铁律）。要网页控制机械臂，起边车：")
    A("> `node js/src/bridge.js --target 192.168.7.120:5000`，页面里 "
      "`openArm('ws://127.0.0.1:8765/')`。")
    A("> 边车**每个 WS 连接独占一个 UDP socket** ⇒ **一个进程可同时带主/从两路**。")
    A("> 生产环境建议用 `systemd` 常驻（见 `docs/Linux部署与打包.md` §5）。")
    A("")
    A("### 一键启动主从遥操 / 遥操示教（部署方最常用）")
    A("")
    A("```bash")
    A("cd launchers && ./install_deps.sh          # 装依赖（Windows: install_deps.bat）")
    A("# 改 arm_link_config.json 的 leader / follower 地址")
    A("python3 launch_teleop.py --preflight-only  # ① 只预检（PING/状态/读失败轴/E-Stop/fw-tag/零点/限位）")
    A("python3 launch_teleop.py                   # ② 起主从遥操")
    A("python3 launch_teach.py record --sec 30 --name demo && python3 launch_teach.py list")
    A("python3 launch_teach.py play trajectories/demo.json --speed 0.5")
    A("```")
    A("")
    A("> `./start_teleop.sh` / `./start_teach.sh` 是等价的封装（自动 `-X utf8`）。")
    A("> **完整步骤与排错表 → `launchers/启动步骤.md`**；"
      "想接成 systemd 常驻，把上面命令塞进 unit 即可（见 `docs/Linux部署与打包.md` §5）。")
    A("")
    A("**依赖**：核心纯标准库。**网口 `net://` 不需要任何第三方库** —— 这是首选链路"
      "（实测 p50 1.61 ms、547 帧/s、零超时）。可选：")
    A("")
    A("| 依赖 | 什么时候需要 | 装法 |")
    A("|---|---|---|")
    A("| `pyserial` | 走串口 `serial:///dev/ttyUSB0` | `pip install pyserial` |")
    A("| `numpy` | 轨迹存 `.npz` | `pip install numpy` |")
    A("")
    A("---")
    A("")
    A("## 2. 包里有什么")
    A("")
    L.extend(_pkg_table(manifest))
    A("### 关键目录说明")
    A("")
    A("* **`dist/*.whl`** — 唯一的安装入口。`py3-none-any` ⇒ 纯 Python，"
      "Linux 上不需要 gcc / 不需要联网编译。")
    A("* **`arm_sdk/`** — 源码。部署只需要 wheel；源码留着是为了**打补丁 / 用 "
      "`pip install -e .` / 用 `PYTHONPATH=$PWD` 直跑**。")
    A("* **`config/arm_config.json`** — 全部接口真值（114 条文本命令、38 条网口命令字段级、"
      "关节限位、舵机偏移、参数表）。**要改参数从这里读，别硬编码。**")
    A("* **`docs/Linux部署与打包.md`** — 部署主文档：安装三法 / 离线部署 / "
      "udev 固定串口名 / 网络调优（50 Hz 遥操）/ systemd / Docker / 升级回滚 / 常见问题。")
    A("* **`js/`** — 零依赖 JS SDK。要出 Web/Electron 界面时用得上；"
      "`src/bridge.js` 是浏览器走网口的唯一通路。")
    A("* **`tests/` + `docs/网口协议*.md`** — 验收与排障。")
    A("* **`launchers/`** — ★ 一个目录搞定「把臂跑起来」："
      "`launch_teleop.py`（主从遥操）/ `launch_teach.py`（示教录放）+ 唯一配置入口 "
      "`arm_link_config.json` + `start_teleop.sh` / `start_teach.sh` + 《启动步骤.md》。"
      "先跑 `python launch_teleop.py --preflight-only`（只读预检）。")
    A("* **`host_integration/`** — ★ 给**写上位机软件的人**：`上位机对接指南.md`（场景→发哪条指令 "
      "决策表）+ `protocol_spec.json`（38 条命令机读规格）+ 三份参考实现"
      "（`arm_protocol.h` / `csharp/ArmLink.cs` / `minimal_client.py`）。")
    A("* **`docs/配置文件说明.md`** — 两个 JSON 的分工与全字段，改配置前先看这页。")
    A("* **`docs/UDP_vs_HTTP_实测.md`** — ★ 要不要用 HTTP、多付多少延迟、吞吐砍到几分之一；"
      "配 `armctl http-bridge` 用（板子没 TCP，边车跑在 PC 侧）。")
    A("")
    A("---")
    A("")
    A("## 3. 验收动作（交付方与部署方照这个对）")
    A("")
    A("```bash")
    A("# ① 离线：不需要机械臂")
    A("armctl selftest                    # 228 项（无 numpy 227；少的 1 项是 .npz 往返，会打印跳过）")
    A("python3 tests/integration_test.py  # 51 项（无 numpy 50）")
    A("python3 tests/new_commands_test.py # 62 项（2026-09-23 扩展命令）")
    A("")
    A("# JS 侧（Node ≥ 18；零依赖）")
    A("cd js && npm test                  # 7 组 / 971 项（含跨语言逐字节对拍 + 真 UDP 回环）")
    A("```")
    A("")
    A("```bash")
    A("# ② 联机：只读、不要动")
    A("armctl --uri net://192.168.7.120:5000 preflight")
    A("armctl --uri net://192.168.7.120:5000 ping --repeat 200   # 兼测 P50/P99")
    A("armctl --uri net://192.168.7.120:5000 status")
    A("```")
    A("")
    A("> ⚠ **网口不通先看这两条**：① 板子默认**两条链路都关**，必须先串口敲 "
      "`link net` → `link save` → **按 RESET**；② PC 网段要配到 `192.168.7.x`。")
    A("> 网口不通时的表现是**一个字节都不回**，不是报错。")
    A("")
    A("> ⚠ **上运动测试之前**：先回中位 "
      "`armctl --uri net://... move --delta 0,0,0,0,0,0` 或串口 "
      "`m 180 180 180 180 180 180 15`。烧录/复位必丢力矩、臂会下沉。"
      "**绝对不要用 `M a`**（那是标定，会废掉量纲基准）。")
    A("")
    A("---")
    A("")
    L.extend(_HARD_CONSTRAINTS)
    A("---")
    A("")
    L.extend(_FIELD_INFO)
    A("---")
    A("")
    L.extend(_EXCLUDE_SECTION)
    A("> 需要**示例代码 / 3D 模型 / 上位机需求与验收标准** → 找 `arm_sdk_dev_v%s` 那份包。" % version)
    A("")
    A("---")
    A("")
    A("## 7. 出问题找谁")
    A("")
    A("* 协议 / 字段 / 错误码 → `docs/网口协议速查.md`（查不到 → `docs/网口指令大全.md`，"
      "含**故障对照表**与 **13 条网口红线**）")
    A("* 连不上 / 装不上 / 容器 / systemd / 网络调优 → `docs/Linux部署与打包.md`")
    A("* 主从遥操 / 示教 → `docs/主从遥操与示教.md`")
    A("* 链路选哪条、实测多快 → 独立文档《三链路延迟基准_简版》")
    A("* 以上都没有 → 回头找固件/上位机对接人（本包生成方）")
    A("")
    return "\n".join(L)


# ==========================================================================
# 归档
# ==========================================================================
def _archive_targz(out: str) -> str:
    """打 tar.gz：路径排序 + 清 uid/gid（避免带上本机用户名），保留源文件 mtime。

    注意 START_HERE.md 里嵌了构建时间，所以**每次构建 hash 都会变** ——
    这是有意的：hash 就是这一份产物的指纹，交接时用它核对收到的文件有没有被改过。
    """
    path = out + ".tar.gz"
    if os.path.exists(path):
        os.remove(path)
    base = os.path.basename(out)
    with tarfile.open(path, "w:gz") as tar:
        for dirpath, dirnames, filenames in os.walk(out):
            dirnames[:] = sorted(d for d in dirnames if d not in SKIP_NAMES)
            for fn in sorted(filenames):
                full = os.path.join(dirpath, fn)
                arc = os.path.join(base, os.path.relpath(full, out).replace(os.sep, "/"))
                ti = tar.gettarinfo(full, arcname=arc)
                ti.uid = ti.gid = 0
                ti.uname = ti.gname = "root"
                with open(full, "rb") as f:
                    tar.addfile(ti, f)
    return path


def _archive_zip(out: str) -> str:
    path = out + ".zip"
    if os.path.exists(path):
        os.remove(path)
    base = os.path.basename(out)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for dirpath, dirnames, filenames in os.walk(out):
            dirnames[:] = sorted(d for d in dirnames if d not in SKIP_NAMES)
            for fn in sorted(filenames):
                full = os.path.join(dirpath, fn)
                z.write(full, os.path.join(base, os.path.relpath(full, out)))
    return path


# ==========================================================================
# 主流程
# ==========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description="组装交付包（开发包 + 部署包 + 独立文档）")
    ap.add_argument("--version", default="2.0.0")
    ap.add_argument("--only", choices=["dev", "linux"], default=None,
                    help="只出其中一个包（默认两个都出）")
    args = ap.parse_args()
    v = args.version

    os.makedirs(RELEASE_DIR, exist_ok=True)
    artifacts: list[str] = []          # 归档
    standalone_out: list[str] = []     # 独立文档

    # ---- 0. wheel 新鲜度（两个包都靠它，先验一次）----
    whl = pick_wheel()
    if not whl:
        print("  [缺少] dist/*.whl —— 先跑： python -m pip wheel . -w dist --no-deps")
    else:
        bad = verify_wheel_fresh(whl)
        if bad:
            print("\n❌ wheel 与源码不一致，拒绝打包（避免发旧包）：")
            for b in bad:
                print("     · %s" % b)
            print("\n   修复：python -m pip wheel . -w dist --no-deps --no-build-isolation")
            return 1
        print("  [校验] %s 与源码逐字节一致 ✓" % os.path.basename(whl))

    # ---- 1. 开发包 ----
    if args.only in (None, "dev"):
        name = "arm_sdk_dev_v%s" % v
        print("\n组装 %s" % name)
        print("-" * 72)
        out, tgz, zp = build_pkg(name, COMMON + DEV_ONLY, start_here_dev, v, whl)
        if not out:
            return 1
        artifacts += [tgz, zp]
        print("  → %s" % out)

    # ---- 2. 部署包 ----
    if args.only in (None, "linux"):
        name = "arm_sdk_linux_v%s" % v
        print("\n组装 %s" % name)
        print("-" * 72)
        out, tgz, zp = build_pkg(name, COMMON + LINUX_ONLY, start_here_linux, v, whl)
        if not out:
            return 1
        artifacts += [tgz, zp]
        print("  → %s" % out)

    # ---- 3. 独立交付物（不进包）----
    print("\n独立交付物")
    print("-" * 72)
    for src_rel, out_name, _note in STANDALONE:
        src = resolve_src(src_rel)
        if not os.path.exists(src):
            print("  [跳过-缺失] %s" % src_rel)
            continue
        dst = os.path.join(RELEASE_DIR, out_name)
        shutil.copy2(src, dst)
        standalone_out.append(dst)
        print("  [独立] %s  (%s)" % (out_name, _human(os.path.getsize(dst))))

    # ---- 4. 指纹表 ----
    sums = os.path.join(RELEASE_DIR, "SHA256SUMS.txt")
    with open(sums, "w", encoding="utf-8") as f:
        f.write("arm_sdk 交付物指纹（sha256）\n")
        f.write("生成(UTC): %s\n" % datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S"))
        f.write("=" * 96 + "\n\n")
        for p in artifacts + standalone_out:
            f.write("%s  %s\n" % (sha256(p), os.path.basename(p)))

    print("\n" + "=" * 72)
    for p in artifacts:
        print("✅ %-46s %10s" % (os.path.basename(p), _human(os.path.getsize(p))))
        print("     sha256 = %s" % sha256(p))
    for p in standalone_out:
        print("📄 %-46s %10s （独立文档，不在任何包内）"
              % (os.path.basename(p), _human(os.path.getsize(p))))
    print("\n指纹表 : %s" % sums)
    print("开发包入口 : %s" % os.path.join(RELEASE_DIR, "arm_sdk_dev_v%s" % v, "START_HERE.md"))
    print("部署包入口 : %s" % os.path.join(RELEASE_DIR, "arm_sdk_linux_v%s" % v, "START_HERE.md"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
