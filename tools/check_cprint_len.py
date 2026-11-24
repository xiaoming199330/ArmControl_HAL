#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 main.c 里每条 cprint(...) 的**字面量字节数**有没有超过 255。

为什么需要这个脚本
------------------
`cprint()` 用 `char buf[256]` + `vsnprintf` 钳位: 单次输出的成品必须 <= 255 字节,
超了会**静默截断**(不报错、不返回错误)。

这个坑在 2026-09-21 实机咬过一次: NET 构建的 `fw-tag` 横幅, fix 清单加到 20 项后
整行拼到 **368 字节** ⇒ 现场敲 `h` 只能看到 `...np-text-fix,ne` 就断了。
后果不是"少看几个字", 而是**没法判断固件到底烧进去没有** —— 排查方向被带偏。

所以把这条纪律做成自动检查: 加 fix token / 改横幅之后跑一下, 别靠肉眼数。

用法
----
    python tools/check_cprint_len.py [源文件...]      # 默认 Core/Src/main.c
退出码 0 = 全过; 1 = 有超限项(会列出文件:行号和长度)。

注意
----
* 只统计**字面量**字节数。`%lu` 这类格式符在运行期还会再撑开, 所以
  算出来的值是**下界** —— 超过 255 一定有问题, 没超也可能在运行期超
  (留点余量, 别贴着 255 写)。
* `cprint_long()` 不受 255 限制(它自己按换行分块), 只做提示不报错。
* 转义按真实字节算: "\\r\\n" 算 2 字节而不是 4。
"""

import os
import re
import sys

LIMIT = 255

# 匹配  cprint(         或 cprint_long(
CALL_RE = re.compile(r"\b(cprint|cprint_long)\s*\(")
# 匹配一个 C 字符串字面量(含转义)
STR_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')


def decode_c_string(body: str) -> bytes:
    """把一个 C 字符串字面量的**内容**按 C 语义解成字节。"""
    out = bytearray()
    i = 0
    simple = {
        "n": 0x0A, "r": 0x0D, "t": 0x09, "0": 0x00,
        "\\": 0x5C, '"': 0x22, "'": 0x27, "a": 0x07, "b": 0x08,
        "f": 0x0C, "v": 0x0B,
    }
    while i < len(body):
        c = body[i]
        if c != "\\":
            out.extend(c.encode("utf-8"))
            i += 1
            continue
        i += 1
        if i >= len(body):
            break
        e = body[i]
        if e == "x":                     # \xHH
            j = i + 1
            hexs = ""
            while j < len(body) and body[j] in "0123456789abcdefABCDEF":
                hexs += body[j]
                j += 1
            if hexs:
                out.append(int(hexs, 16) & 0xFF)
                i = j
                continue
        if e in "01234567":              # \NNN (八进制, 最多 3 位)
            j = i
            octs = ""
            while j < len(body) and body[j] in "01234567" and len(octs) < 3:
                octs += body[j]
                j += 1
            out.append(int(octs, 8) & 0xFF)
            i = j
            continue
        if e == "u" or e == "U":         # \uXXXX / \UXXXXXXXX
            n = 4 if e == "u" else 8
            hexs = body[i + 1: i + 1 + n]
            try:
                out.extend(chr(int(hexs, 16)).encode("utf-8"))
            except ValueError:
                pass
            i += 1 + n
            continue
        out.append(simple.get(e, ord(e) & 0xFF))
        i += 1
    return bytes(out)


def literal_bytes(text: str, open_idx: int):
    """从 `(` 之后开始, 收集相邻字符串字面量, 返回 (字节数, 结束下标)。

    允许跨行的相邻字面量(C 的自动拼接), 也允许中间夹空白/换行/注释-ish 空白。
    """
    total = b""
    i = open_idx + 1
    while True:
        m = STR_RE.search(text, i)
        if not m:
            break
        # 两个字面量之间只准有空白, 否则说明这个字符串属于别的实参(比如 %s 的实参)
        between = text[i:m.start()]
        if between.strip() != "":
            break
        total += decode_c_string(m.group(1))
        i = m.end()
    return len(total), i


def scan(path: str):
    try:
        text = open(path, "r", encoding="utf-8", errors="replace").read()
    except OSError as e:
        print("✗ 读不了 %s: %s" % (path, e))
        return []
    bad = []
    for m in CALL_RE.finditer(text):
        func = m.group(1)
        n, _ = literal_bytes(text, m.end() - 1)
        if n == 0:
            continue
        line = text.count("\n", 0, m.start()) + 1
        if func == "cprint":
            if n > LIMIT:
                bad.append((path, line, n, func))
        else:
            if n > 100000:
                bad.append((path, line, n, func))
    return bad


def main(argv):
    files = argv[1:] or ["Core/Src/main.c"]
    allbad = []
    checked = 0
    for f in files:
        if not os.path.isfile(f):
            print("✗ 找不到 %s" % f)
            continue
        allbad += scan(f)
        checked += 1
    if checked == 0:
        return 2

    print("检查 %d 个文件, cprint 单次输出上限 %d 字节" % (checked, LIMIT))
    if not allbad:
        print("✔ 没有超限的 cprint —— 每条都在 255 字节以内。")
        print("  ⚠ 这是**字面量下界**: `%lu`/`%s` 在运行期还会撑开, 别贴着 255 写。")
        return 0

    # 超限的报错
    print("")
    for path, line, n, func in allbad:
        if n > LIMIT:
            print("✗ %s:%d  %s() 字面量 %d 字节  ⇒ 超 %d" % (path, line, func, n, n - LIMIT))
    print("")
    print("  修法: 拆成多条 cprint(每条 <=255), 或改用 cprint_long()。")
    print("  现场症状: `h` 的输出**半句话就断**(例如 `...np-text-fix,ne`),")
    print("            于是 [固件烧进去没有] 都判不出来 —— 见 2026-09-21 事故。")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
