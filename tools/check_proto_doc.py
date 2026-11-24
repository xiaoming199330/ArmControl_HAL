#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_proto_doc.py —— 协议文档里的"示例报文"自动守卫

背景（与工程红线 5 同族）：
    `docs/网口指令大全_*.md` 与 `docs/CAN协议规格书_*.md` 里手写了大量示例报文
    （`SEQ CMD LEN PAYLOAD CRC_LO CRC_HI`）。**只要有一个十六进制字符抄错，
    读者照抄就会得到一条 CRC 错或长度错的报文** —— 而长度错时臂**完全不回包**，
    表现为"超时"，比 CRC 错更难查。人眼校对几十条报文不现实，所以做成脚本。

它做什么：
    扫描 md 文件里所有 ``` 代码块，把"看起来是完整报文"的 hex 行抓出来，逐条验证：
      ① 长度  = 5 + LEN           （不等 ⇒ 臂静默丢弃，只表现为超时）
      ② CRC16 = 帧里那两字节       （不等 ⇒ 臂回 NACK err=2）
    全部通过输出 OK，任一失败则打印行号与原因并非零退出。

⚠ 作用范围：只扫 **``` 围栏代码块**里的 hex。CAN 规格书 §8 的示例在 markdown 表格里，
   而且那是 **CAN 帧**（带 `0x81|n` 帧头、按 7 字节切分）而不是逻辑包 ⇒ 不适用本脚本，
   属于有意跳过（避免误报）。

用法：
    py -3 tools/check_proto_doc.py                          # 默认查网口指令大全
    py -3 tools/check_proto_doc.py "docs/CAN协议规格书_上位机对接_2026-09-18.md"
"""
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_host import crc16  # 与固件 Aproto_Crc16 同一实现

HEXTOK = re.compile(r"\b[0-9A-Fa-f]{2}\b")
RUN = re.compile(r"((?:[0-9A-Fa-f]{2}[ \t]+){4,}[0-9A-Fa-f]{2})")
CUT = re.compile(r"[←]")          # 行内注释起始符


def check(path):
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()

    ok = bad = 0
    problems = []
    in_block = False

    for lineno, raw in enumerate(lines, 1):
        line = raw.rstrip("\n")
        if line.strip().startswith("```"):
            in_block = not in_block
            continue
        if not in_block:
            continue

        body = CUT.split(line)[0]
        if len(HEXTOK.findall(body)) < 5:
            continue
        m = RUN.search(body)
        if not m:
            continue

        data = bytes.fromhex(re.sub(r"\s+", "", m.group(1)))
        ln = data[2]
        if len(data) != 5 + ln:
            bad += 1
            problems.append((lineno, "长度不符: 实测 %d B, 应为 5+LEN(=%d)=%d B"
                             % (len(data), ln, 5 + ln), line.strip()))
            continue
        got = data[3 + ln] | (data[3 + ln + 1] << 8)
        want = crc16(data[:3 + ln])
        if got != want:
            bad += 1
            problems.append((lineno, "CRC 错: 帧里 %04X, 应 %04X" % (got, want),
                             line.strip()))
            continue
        ok += 1

    print("%s" % path)
    print("  合法报文 %d 条, 问题 %d 条" % (ok, bad))
    for lineno, why, txt in problems:
        print("  行 %d: %s" % (lineno, why))
        print("      %s" % txt)
    return bad


def main():
    if len(sys.argv) > 1:
        targets = sys.argv[1:]
    else:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        targets = sorted(glob.glob(os.path.join(root, "docs", "网口指令大全_*.md")))
    if not targets:
        print("没找到目标文档")
        return 2
    total = 0
    for t in targets:
        if not os.path.exists(t):
            print("✗ 文件不存在: %s" % t)
            total += 1
            continue
        total += check(t)
        print()
    if total:
        print("✗ 共 %d 处问题，请修正（读者会照抄这些 hex）" % total)
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
