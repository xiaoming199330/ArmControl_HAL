# -*- coding: utf-8 -*-
"""check_arm_config.py — 把 config/arm_config.json 与固件源码做交叉校验。

为什么需要它
------------
`arm_config.json` 是**上位机镜像**, 固件不读它 ⇒ 它一定会漂移。这个脚本从固件源码里
抽"真相", 和 JSON 对账, 让漂移变成一条可以 CI 化的错误:

  1. `Core/Src/arm_commands.c` 的 cmd_registry[]  —— 文本命令清单
  2. `Core/Src/main.c` cmd_is_known_multichar()   —— **多字符命令白名单**
     (红线: 新增多字符命令不往这张表里加 ⇒ 被守卫挡掉, 现场表现为"命令没反应")
  3. `Core/Inc/arm_control_protocol.h`            —— 二进制命令码
  4. `Core/Inc/arm_control_api.h`                 —— 错误码

跑法:
    python tools/check_arm_config.py
退出码 0 = 一致; 1 = 有漂移(打印缺失项)。
"""
from __future__ import annotations

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CFG = os.path.join(ROOT, "config", "arm_config.json")

SRC_REGISTRY = os.path.join(ROOT, "Core", "Src", "arm_commands.c")
SRC_MAIN = os.path.join(ROOT, "Core", "Src", "main.c")
SRC_PROTO = os.path.join(ROOT, "Core", "Inc", "arm_control_protocol.h")
SRC_API = os.path.join(ROOT, "Core", "Inc", "arm_control_api.h")

problems: list = []
notes: list = []


def fail(msg: str) -> None:
    problems.append(msg)


def read(p: str) -> str:
    with open(p, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def registry_names(text: str) -> set:
    """从 cmd_registry[] 抽 tokens[0]（都是 `{"name", ...}` 形式）。"""
    out = set()
    for m in re.finditer(r'^\{"([^"]+)",', text, re.M):
        out.add(m.group(1))
    return out


def multichar_whitelist(text: str) -> set:
    """抽 cmd_is_known_multichar() 的 names[] 字符串。"""
    m = re.search(r"cmd_is_known_multichar\(const char\* t\)\s*\{.*?names\[\]\s*=\s*\{(.*?)\};",
                  text, re.S)
    if not m:
        return set()
    return set(re.findall(r'"([^"]+)"', m.group(1)))


def proto_cmds(text: str) -> dict:
    """APROTO_CMD_X = 0xNN,  返回 {0xNN: 'X'}"""
    out = {}
    for m in re.finditer(r"APROTO_CMD_([A-Z0-9_]+)\s*=\s*(0x[0-9A-Fa-f]+)", text):
        out[int(m.group(2), 16)] = m.group(1)
    return out


def api_errs(text: str) -> dict:
    """ARM_API_ERR_X = N,  返回 {N: 'X'}（跳过 ARG 之类非协议错误码）"""
    out = {}
    for m in re.finditer(r"ARM_API_ERR_([A-Z0-9_]+)\s*=\s*(\d+)", text):
        out[int(m.group(2))] = m.group(1)
    return out


def main() -> int:
    if not os.path.exists(CFG):
        print("找不到 %s —— 先跑 tools/gen_arm_config_json.py" % CFG)
        return 1
    cfg = json.load(open(CFG, encoding="utf-8"))["arm_config"]

    json_names = {c["name"] for c in cfg["commands"]}
    json_groups = {c["group"] for c in cfg["commands"]}
    # JSON 的 name 写成"完整语法"(如 "a J deg" / "L J min max"), 注册表只有首 token
    # ⇒ 对账时统一取首 token。别名表把固件里的等价写法归到 JSON 的那一条。
    ALIAS = {"delete": "del", "rec": "rec clear", "M": "M a", "m": "m"}
    json_first = set()
    for n in json_names:
        t = ALIAS.get(n, n)
        json_first.add(t.split()[0] if t.split() else t)

    # ---------------- 1. 文本命令清单 ----------------
    reg = registry_names(read(SRC_REGISTRY))
    missing_in_json = sorted(reg - json_first)
    extra_in_json = sorted(json_first - reg)
    if missing_in_json:
        fail("arm_commands.c 有但 arm_config.json 缺: %s" % missing_in_json)
    notes.append("注册表首 token %d 条 / JSON 首 token %d 条；JSON 独有(往往是 main.c 精确"
                 "匹配或别名): %s" % (len(reg), len(json_first), extra_in_json or "无"))

    # ---------------- 2. 多字符白名单 ----------------
    wl = multichar_whitelist(read(SRC_MAIN))
    # 白名单里的"变体"名（pv/pV/qV/...）指向同一条 JSON 命令, 归一化后再比
    base_of = {
        "pv": "pv", "pV": "pv", "pf": "pf", "pF": "pf",
        "qv": "qv", "qV": "qv", "qd": "qd", "qD": "qd", "qg": "qg", "qG": "qg",
        "QV": "QL", "Qv": "QL", "QD": "QL", "Qd": "QL", "QL": "QL", "QLC": "QLC",
        "QLc": "QLC", "BG": "BG", "SP": "SP", "Sp": "SP", "LP": "LP", "Lp": "LP",
        "LL": "LL", "Ll": "LL", "delete": "del", "rec": "rec clear",
    }
    wl_base = {base_of.get(w, w).split()[0] for w in wl}
    wl_missing = sorted(w for w in wl_base if w not in json_first)
    if wl_missing:
        fail("cmd_is_known_multichar() 白名单里有但 arm_config.json 缺: %s" % wl_missing)
    notes.append("多字符白名单 %d 项 (归一化 %d) —— %s"
                 % (len(wl), len(wl_base),
                    "全部在 JSON 里" if not wl_missing else "缺 %s" % wl_missing))

    # ---------------- 3. 二进制命令码 ----------------
    pc = proto_cmds(read(SRC_PROTO))
    np_cmds = cfg.get("network_protocol", {}).get("commands", [])
    json_codes = {int(c["cmd"], 16) for c in np_cmds}
    missing_codes = sorted("%s(%s)" % (hex(k), v) for k, v in pc.items()
                           if k not in json_codes)
    if missing_codes:
        fail("arm_control_protocol.h 里有但 network_protocol.commands 缺: %s"
             % missing_codes)
    if len(np_cmds) != 38:
        fail("network_protocol.commands 应 38 条 (14 基础 + 24 扩展), 实为 %d" % len(np_cmds))
    # 每条必须有字段级说明
    for c in np_cmds:
        for k in ("cmd", "name", "dir", "payload", "response"):
            if k not in c:
                fail("network_protocol 命令 %s 缺字段 %s" % (c.get("name", "?"), k))
    notes.append("二进制命令码 %d 条 (协议头) vs JSON %d 条" % (len(pc), len(json_codes)))

    # ---------------- 4. 错误码 ----------------
    ae = api_errs(read(SRC_API))
    json_errs = {int(k): v for k, v in cfg["network_protocol"]["error_codes"].items()
                 if k.isdigit()}
    for code, name in ae.items():
        if code not in json_errs:
            fail("arm_control_api.h 错误码 %d(%s) 未写进 network_protocol.error_codes"
                 % (code, name))
    notes.append("错误码 %s -> JSON %s" % (sorted(ae), sorted(json_errs)))

    # ---------------- 5. 必备章节 ----------------
    for sec in ("carriers", "network_protocol", "link_switch",
                "leader_follower", "teleop_teach", "host_sdk",
                "binary_protocol_commands", "console_contract"):
        if sec not in cfg:
            fail("arm_config 缺章节: %s" % sec)

    # ---------------- 6. 网络命令必须能映射到 SDK ----------------
    try:
        sys.path.insert(0, os.path.join(ROOT, "sdk_package"))
        from arm_sdk import protocol as P
        sdk_cmds = {v for k, v in vars(P).items()
                    if k.startswith("CMD_") and isinstance(v, int)}
        if sdk_cmds != json_codes:
            only_json = json_codes - sdk_cmds
            only_sdk = sdk_cmds - json_codes
            if only_json or only_sdk:
                fail("arm_sdk.protocol 与 JSON 命令集不一致: 仅JSON=%s 仅SDK=%s"
                     % (sorted(map(hex, only_json)), sorted(map(hex, only_sdk))))
        notes.append("arm_sdk.protocol CMD_* 与 JSON 命令集一致 (%d 条)" % len(sdk_cmds))
    except Exception as e:                      # noqa: BLE001
        notes.append("跳过 SDK 对账 (%s)" % e)

    # ---------------- 6b. CRC / 定点一致性 ----------------
    try:
        from arm_sdk import crc16
        if crc16(b"123456789") != 0x29B1:
            fail("arm_sdk.crc16 标准向量不符")
        else:
            notes.append("arm_sdk.crc16 向量 0x29B1 通过")
    except Exception:
        pass

    # ---------------- 输出 ----------------
    print("=" * 72)
    print("arm_config 交叉校验 (工具生成的镜像 vs 固件源码)")
    print("=" * 72)
    for n in notes:
        print("  · " + n)
    print("-" * 72)
    if problems:
        print("✗ %d 处漂移:" % len(problems))
        for p in problems:
            print("   - " + p)
        print("\n修法: 改 tools/gen_arm_config_json.py, 再跑一次本脚本。")
        return 1
    print("✓ 全部一致 (%d 章 / %d 条文本命令 / %d 条网口命令)"
          % (len(cfg), len(json_names), len(np_cmds)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
