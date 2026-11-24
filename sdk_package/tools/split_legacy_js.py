#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""split_legacy_js.py — 把 web/arm_sdk.js（单文件 1265 行）按段落切成分层模块。

为什么要切
----------
web/arm_sdk.js 是"一个大文件装完 FK/IK + 二进制协议 + 文本控制台"的形态，
在浏览器里 `<script type=module>` 直接引用还行，但要和新的网口层（UDP）拼成
一个 SDk 包就不好复用了 —— 每加一层都要跨文件拷代码。

切法（按文件里已有的 §1~§10 段落标记，零猜测）
---------------------------------------------
  §1~§5  (27..417)  → js/src/kinematics.js   几何常量 / 角度换算 / FK / 限位 / IK
  §6     (419..519)  → 由 protocol.js 重写（只复用 crc16 的实现，语义对齐 Python 版）
  §7~§10 (521..1265) → js/src/console.js      响应判定 / 解析器 / ArmSDK(Web Serial)

切完会**自动比对导出符号**并生成 import 语句，避免手抄漏。
"""
from __future__ import annotations

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "web", "arm_sdk.js")
OUT = os.path.join(ROOT, "js", "src")

# ⚠ 起点必须是 `/* ===...` 那一行本身，不能落在块注释中间（会切出 SyntaxError）
KIN_FROM, KIN_TO = 26, 417          # 1-based, 含两端
CON_FROM, CON_TO = 521, 1265

EXPORT_RE = re.compile(
    r"^export\s+(?:default\s+)?(?:async\s+)?(?:const|let|var|function|class)\s+(\w+)",
    re.M)
EXPORT_LIST_RE = re.compile(r"^export\s*\{([^}]*)\}", re.M)


def read_lines(path: str) -> list[str]:
    with open(path, "r", encoding="utf-8") as f:
        return f.read().split("\n")


def exported_names(text: str) -> list[str]:
    names = EXPORT_RE.findall(text)
    for grp in EXPORT_LIST_RE.findall(text):
        for part in grp.split(","):
            part = part.strip()
            if not part:
                continue
            names.append(part.split(" as ")[-1].strip())
    seen, out = set(), []
    for n in names:
        if n and n not in seen:
            seen.add(n)
            out.append(n)
    return out


def used_names(text: str, candidates: list[str]) -> list[str]:
    return [n for n in candidates if re.search(r"\b%s\b" % re.escape(n), text)]


KIN_HEADER = '''/**
 * kinematics.js — 六轴机械臂运动学（FK / IK / 限位 / 角度换算）
 *
 * 从 web/arm_sdk.js 的 §1~§5 原样切出，**逻辑一字未改**。
 * 几何常量与固件 Core/Inc/kinematics.h 逐位一致 —— 不要改！
 * 已对拍：rho=131.39 / z=3.98 / pitch=150.03
 *
 * 本模块**不依赖任何传输层**，浏览器 / Node / Electron 都能直接用。
 */

'''

CON_HEADER = '''/**
 * console.js — 串口文本控制台 SDK（Web Serial / 浏览器 / Electron）
 *
 * 从 web/arm_sdk.js 的 §7~§10 原样切出，只把内部依赖改成显式 import。
 * 用于走 **USART6 文本控制台**（115200 8N1，行尾 `\\r\\n`，响应以 `\\r\\n> ` 分帧）。
 *
 * ⚠ 这是"文本命令"通道，和 protocol.js 的"二进制逻辑包"通道**并行存在**：
 *   - 文本通道：`h` `s` `f` `m` … 人机调试、读链路开关
 *   - 二进制通道：14 条网口命令，实时遥操 / 遥测
 * 两条通道共用同一个物理串口（AA55 帧与文本混流，由 FrameParser 分流）。
 *
 * 若只需要网口（UDP），用 client.js 的 ArmClient 即可，不必加载本模块。
 */

'''


def main() -> int:
    lines = read_lines(SRC)

    # ---- 边界自检：起点必须是块注释开头，终点必须是代码收尾 ----
    # （曾因为起点落在块注释中间，切出一个 SyntaxError 的 kinematics.js）
    problems = []
    if not lines[KIN_FROM - 1].lstrip().startswith("/*"):
        problems.append("KIN_FROM=%d 不在块注释开头: %r" % (KIN_FROM, lines[KIN_FROM - 1][:60]))
    if lines[KIN_TO - 1].strip() not in ("}", "};", "*/"):
        problems.append("KIN_TO=%d 不像收尾: %r" % (KIN_TO, lines[KIN_TO - 1][:60]))
    if not lines[CON_FROM - 1].lstrip().startswith("/*"):
        problems.append("CON_FROM=%d 不在块注释开头: %r" % (CON_FROM, lines[CON_FROM - 1][:60]))
    if lines[CON_TO - 1].strip() not in ("}", "};", "*/", ""):
        problems.append("CON_TO=%d 不像收尾: %r" % (CON_TO, lines[CON_TO - 1][:60]))
    if problems:
        print("❌ 切分边界不对（源文件行号可能已漂移，请重核）：")
        for p in problems:
            print("   · " + p)
        return 1

    kin_body = "\n".join(lines[KIN_FROM - 1:KIN_TO])
    con_body = "\n".join(lines[CON_FROM - 1:CON_TO])

    kin_exports = exported_names(kin_body)
    con_exports = exported_names(con_body)

    # console 里对 kinematics / protocol 的依赖 —— **自动探测**，
    # 不手写列表（手写过一次，漏了 readI16，运行期才 ReferenceError）
    protocol_src = os.path.join(OUT, "protocol.js")
    with open(protocol_src, "r", encoding="utf-8") as f:
        proto_text = f.read()
    proto_exports = exported_names(proto_text)

    kin_needed = used_names(con_body, kin_exports)
    proto_needed = used_names(con_body, proto_exports)
    # 名字同时出现在两个模块时，kinematics 优先（语义相同，但更贴近运动学上下文）
    proto_needed = [n for n in proto_needed if n not in kin_needed]

    os.makedirs(OUT, exist_ok=True)

    kin_txt = KIN_HEADER + kin_body.rstrip() + "\n"
    with open(os.path.join(OUT, "kinematics.js"), "w", encoding="utf-8") as f:
        f.write(kin_txt)

    def wrap_import(names, mod):
        if not names:
            return None
        if len(", ".join(names)) <= 72:
            return "import { %s } from './%s';" % (", ".join(names), mod)
        body = ",\n         ".join(names)
        return "import {\n         %s,\n       } from './%s';" % (body, mod)

    imports = [x for x in (wrap_import(kin_needed, "kinematics.js"),
                           wrap_import(proto_needed, "protocol.js")) if x]
    con_txt = CON_HEADER + "\n".join(imports) + "\n\n" + con_body.rstrip() + "\n"
    with open(os.path.join(OUT, "console.js"), "w", encoding="utf-8") as f:
        f.write(con_txt)

    print("kinematics.js  %5d 行  / %d 个导出" % (len(kin_txt.split("\n")), len(kin_exports)))
    print("console.js     %5d 行  / %d 个导出" % (len(con_txt.split("\n")), len(con_exports)))
    print()
    print("console.js ← kinematics.js : %s" % (", ".join(kin_needed) or "无"))
    print("console.js ← protocol.js   : %s" % (", ".join(proto_needed) or "无"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
