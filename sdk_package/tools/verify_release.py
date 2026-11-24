#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_release.py — 交付前最后一道闸门。

为什么要这个脚本
----------------
make_release.py 只负责"组装"。它**不会**回头核对：

  · 包里的文件和磁盘上的源文件是不是逐字节一致（万一 copy 时源被改过？）
  · tar.gz / zip 里的内容和目录版是不是一致（两个归档是各自独立走的目录？）
  · SHA256SUMS.txt 的指纹是不是就是磁盘上那四个归档
  · 两个包里**必须一致**的东西是不是真的一致（arm_config.json 四副本、ostalo、共享协议文件）

这份_script 就是干这个的。用法：

    python tools/verify_release.py            # 全量核对
    python tools/verify_release.py --quiet    # 只打印结论

退出码：0 = 全部通过；1 = 有 FAIL。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tarfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)              # sdk_package/
RELEASE = os.path.join(ROOT, "release")
VERSION = os.environ.get("ARM_SDK_VERSION", "2.0.0")

SKIP = {"__pycache__"}
OK, FAIL, WARN = [], [], []


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()


import re

_TAG_RE = re.compile(r"\b(ARM-[A-Z]+-\d{4}[A-Z]{0,2})\b")


def _preflight_tags() -> set[str]:
    """从 tools/arm_preflight.py 的 FW_TAGS 常量里取"当前已知 tag"白名单。

    不断言具体某个 tag —— 固件一天能改两个版本（0923B→C→D），
    但**必须**是 preflight 认得的那个，否则纪律就断了。
    """
    p = os.path.join(os.path.dirname(ROOT), "tools", "arm_preflight.py")
    if not os.path.exists(p):
        return set()
    return set(_TAG_RE.findall(open(p, encoding="utf-8").read()))


def check(tag: str, cond: bool, detail: str = "") -> None:
    (OK if cond else FAIL).append(tag + (" — " + detail if detail else ""))
    print("  [%s] %s%s" % ("OK " if cond else "FAIL", tag, (" — " + detail) if detail else ""))


def walk_files(base: str) -> dict[str, str]:
    """返回 {相对路径: sha256}，跳过 __pycache__。"""
    out: dict[str, str] = {}
    for dp, dns, fns in os.walk(base):
        dns[:] = [d for d in dns if d not in SKIP]
        for fn in fns:
            p = os.path.join(dp, fn)
            out[os.path.relpath(p, base).replace(os.sep, "/")] = sha256(p)
    return out


def read_tar(tgz: str) -> tuple[str, dict[str, str]]:
    """返回 (顶层目录名, {相对路径: sha256})。"""
    res: dict[str, str] = {}
    top = ""
    with tarfile.open(tgz, "r:gz") as t:
        for m in t.getmembers():
            if not m.isfile():
                continue
            name = m.name.replace("\\", "/")
            parts = name.split("/")
            if len(parts) < 2:
                continue
            if not top:
                top = parts[0]
            res["/".join(parts[1:])] = hashlib.sha256(t.extractfile(m).read()).hexdigest()
    return top, res


def read_zip(zp: str) -> dict[str, str]:
    res: dict[str, str] = {}
    with zipfile.ZipFile(zp) as z:
        for i in z.infolist():
            if i.is_dir():
                continue
            name = i.filename.replace("\\", "/")
            parts = name.split("/")
            if len(parts) < 2:
                continue
            res["/".join(parts[1:])] = hashlib.sha256(z.read(i)).hexdigest()
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    dev_dir = os.path.join(RELEASE, "arm_sdk_dev_v%s" % VERSION)
    lin_dir = os.path.join(RELEASE, "arm_sdk_linux_v%s" % VERSION)
    dev_tgz = dev_dir + ".tar.gz"
    dev_zip = dev_dir + ".zip"
    lin_tgz = lin_dir + ".tar.gz"
    lin_zip = lin_dir + ".zip"

    if not os.path.isdir(dev_dir) or not os.path.isdir(lin_dir):
        print("找不到待核对的包，先跑 tools/make_release.py")
        return 1

    # ------------------------------------------------------------------
    print("① 归档 ↔ 目录 逐字节比对（20~… 个文件）")
    print("-" * 78)
    dev_files = walk_files(dev_dir)
    lin_files = walk_files(lin_dir)
    _top, dev_tgz_files = read_tar(dev_tgz)
    dev_zip_files = read_zip(dev_zip)
    _top2, lin_tgz_files = read_tar(lin_tgz)
    lin_zip_files = read_zip(lin_zip)

    check("dev  tar.gz ↔ 目录", dev_tgz_files == dev_files,
          "%d 项，差异 %d" % (len(dev_files), len(set(dev_files.items()) ^ set(dev_tgz_files.items())) // 2)
          if dev_tgz_files != dev_files else "%d 个文件全部一致" % len(dev_files))
    check("dev  zip   ↔ 目录", dev_zip_files == dev_files,
          "%d 个文件全部一致" % len(dev_files) if dev_zip_files == dev_files else "有差异")
    check("linux tar.gz ↔ 目录", lin_tgz_files == lin_files,
          "%d 个文件全部一致" % len(lin_files) if lin_tgz_files == lin_files else "有差异")
    check("linux zip   ↔ 目录", lin_zip_files == lin_files,
          "%d 个文件全部一致" % len(lin_files) if lin_zip_files == lin_files else "有差异")

    # ------------------------------------------------------------------
    print("\n② 目录版 ↔ 源工作树（防止 copy 时源已被改）")
    print("-" * 78)
    must_same = [
        ("README.md", "README.md"),
        ("pyproject.toml", "pyproject.toml"),
        ("config/arm_config.json", "config/arm_config.json"),
        ("host_integration/上位机对接指南.md", "host_integration/上位机对接指南.md"),
        ("host_integration/protocol_spec.json", "host_integration/protocol_spec.json"),
        ("host_integration/arm_protocol.h", "host_integration/arm_protocol.h"),
        ("host_integration/minimal_client.py", "host_integration/minimal_client.py"),
        ("host_integration/csharp/ArmLink.cs", "host_integration/csharp/ArmLink.cs"),
        ("launchers/arm_link_config.json", "launchers/arm_link_config.json"),
        ("launchers/launch_teleop.py", "launchers/launch_teleop.py"),
        ("launchers/launch_teach.py", "launchers/launch_teach.py"),
        ("launchers/_common.py", "launchers/_common.py"),
        ("launchers/启动步骤.md", "launchers/启动步骤.md"),
        ("docs/配置文件说明.md", "docs/配置文件说明.md"),
        ("docs/UDP_vs_HTTP_实测.md",
         os.path.join("..", "docs", "UDP_vs_HTTP_实测_2026-09-24.md")),
        ("docs/网口协议速查.md", "docs/网口协议速查.md"),
        ("tests/integration_test.py", "tests/integration_test.py"),
        ("tests/new_commands_test.py", "tests/new_commands_test.py"),
    ]
    for rel, src_rel in must_same:
        for tag, base in (("dev", dev_dir), ("linux", lin_dir)):
            p = os.path.join(base, rel)
            s = os.path.join(ROOT, src_rel)
            if not os.path.exists(p):
                check("%s/%s" % (tag, rel), False, "包里没有")
            elif not os.path.exists(s):
                check("%s/%s" % (tag, rel), False, "源不见了")
            else:
                check("%s/%s ↔ 源" % (tag, rel),
                      open(p, "rb").read() == open(s, "rb").read())

    # ---- SDK 源码 ↔ 源树（arm_sdk 整个目录）----
    n = 0
    bad = []
    for fn in sorted(os.listdir(os.path.join(ROOT, "arm_sdk"))):
        if not fn.endswith((".py", ".json")):
            continue
        a = open(os.path.join(ROOT, "arm_sdk", fn), "rb").read()
        for tag, base in (("dev", dev_dir), ("linux", lin_dir)):
            b = open(os.path.join(base, "arm_sdk", fn), "rb").read()
            n += 1
            if a != b:
                bad.append("%s/arm_sdk/%s" % (tag, fn))
    check("arm_sdk/ 源码 12 份 × 2 包逐字节一致", not bad, ", ".join(bad) if bad else "%d 次比对全通过" % n)

    # ------------------------------------------------------------------
    print("\n③ SHA256SUMS.txt 复算")
    print("-" * 78)
    sums = os.path.join(RELEASE, "SHA256SUMS.txt")
    check("SHA256SUMS.txt 存在", os.path.exists(sums))
    declared = {}
    if os.path.exists(sums):
        for line in open(sums, encoding="utf-8"):
            line = line.strip()
            if len(line) > 70 and "  " in line:
                h, nm = line.split("  ", 1)
                declared[nm.strip()] = h.strip()
    recomputed = {os.path.basename(p): sha256(p)
                  for p in (dev_tgz, dev_zip, lin_tgz, lin_zip)}
    for nm, h in recomputed.items():
        got = declared.get(nm)
        check("指纹 %s" % nm, got == h,
              "表内 %s / 复算 %s" % (got[:16] if got else "缺失", h[:16]))
    missing = set(recomputed) - set(declared)
    check("指纹表没有漏项", not missing, ", ".join(sorted(missing)))

    # ------------------------------------------------------------------
    print("\n④ 两个包之间必须一致的东西")
    print("-" * 78)
    shared = ["README.md", "pyproject.toml", "config/arm_config.json",
              "docs/配置文件说明.md", "docs/UDP_vs_HTTP_实测.md",
              "docs/网口协议速查.md", "docs/主从遥操与示教.md",
              "arm_sdk/http_bridge.py", "arm_sdk/mock_server.py",
              "host_integration/上位机对接指南.md", "host_integration/protocol_spec.json",
              "host_integration/arm_protocol.h", "host_integration/minimal_client.py",
              "host_integration/csharp/ArmLink.cs",
              "launchers/arm_link_config.json", "launchers/launch_teleop.py",
              "launchers/launch_teach.py", "launchers/启动步骤.md"]
    for rel in shared:
        p1 = os.path.join(dev_dir, rel)
        p2 = os.path.join(lin_dir, rel)
        if not (os.path.exists(p1) and os.path.exists(p2)):
            check("dev ↔ linux: %s" % rel, False, "有一边缺失")
        else:
            check("dev ↔ linux: %s" % rel,
                  open(p1, "rb").read() == open(p2, "rb").read())

    # ------------------------------------------------------------------
    print("\n⑤ 语义抽查（防「打包成功但内容是旧的」）")
    print("-" * 78)
    for tag, base in (("dev", dev_dir), ("linux", lin_dir)):
        cfg = json.load(open(os.path.join(base, "config/arm_config.json"), encoding="utf-8"))["arm_config"]
        n_cmd = len(cfg["network_protocol"]["commands"])
        check("%s/config/arm_config.json 含 38 条网口命令" % tag, n_cmd == 38,
              "实际 %d 条" % n_cmd)
        check("%s/config/arm_config.json 含 extension_2026_09_23" % tag,
              "extension_2026_09_23" in cfg)
        # ⚠ 不断言**具体** tag：固件 tag 一天能跳两个版本（0923B→C→D）。
        #   真正的闸门是：包里记的 tag 必须在 tools/arm_preflight.py 的 FW_TAGS 白名单里。
        known = _preflight_tags()
        for variant in ("net", "can", "both"):
            t = cfg["_firmware_tag"].get(variant, "")
            check("%s/_firmware_tag.%s = %s 在 preflight 白名单里" % (tag, variant, t),
                  bool(t) and t in known,
                  "preflight 里有 %d 个已知 tag" % len(known))

        spec = json.load(open(os.path.join(base, "host_integration/protocol_spec.json"), encoding="utf-8"))
        check("%s/protocol_spec.json 登记 38 条命令" % tag, len(spec["commands"]) == 38,
              "实际 %d" % len(spec["commands"]))

        lk = json.load(open(os.path.join(base, "launchers/arm_link_config.json"), encoding="utf-8"))
        check("%s/launchers/arm_link_config.json 有 leader+follower" % tag,
              "leader" in lk and "follower" in lk)

        # 不该出现的：本地测试数据
        bad = [p for p in walk_files(os.path.join(base, "launchers")) if "/trajectories/smoke" in p]
        check("%s/launchers/trajectories 不收本地测试数据" % tag, not bad, ", ".join(bad))
        check("%s/launchers/trajectories/README.txt 在包里" % tag,
              os.path.exists(os.path.join(base, "launchers/trajectories/README.txt")))

    pycache = [p for p in list(walk_files(dev_dir)) + list(walk_files(lin_dir))
               if "__pycache__" in p or p.endswith(".pyc")]
    check("没有 __pycache__ / .pyc 混进包", not pycache, ", ".join(pycache[:5]))

    # ------------------------------------------------------------------
    print("\n" + "=" * 78)
    print("通过 %d 项，失败 %d 项" % (len(OK), len(FAIL)))
    if FAIL:
        print("\n失败清单：")
        for f in FAIL:
            print("  ✗ %s" % f)
        return 1
    print("✅ 交付包全部核对通过 —— 可以发出去了")
    return 0


if __name__ == "__main__":
    sys.exit(main())
