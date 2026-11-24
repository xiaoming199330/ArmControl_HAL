#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_text_move.py — 用 CAN 的 TEXT_EXEC(0x0B) 下发文本命令，全程轮询回传
用法:
  python can_text_move.py                          # z a -> 等到位 -> P 0 -> 等到位
  python can_text_move.py --no-move                # 只读状态
  python can_text_move.py --cmd "h"                # 发任意文本命令(自动翻页取全回显)
  python can_text_move.py --cmd "L" --cmd "zi"     # 依次发多条
"""
import sys, time, argparse
sys.path.insert(0, r"C:\Users\Administrator\Desktop\ArmControl_HAL_V5.1\tools")
from can_host import (ArmCanHost, open_bus_with_retry, i32,
                      CMD_GET_STATUS, decode_status_payload)

CMD_TEXT_EXEC  = 0x0B
CMD_TEXT_FETCH = 0x0C
APROTO_MAGIC   = 0x5A5A5A5A


def text_exec(host, line, timeout=6.0):
    """发一行文本命令, 收 0x8B 分页回传, 返回完整文本。"""
    payload = i32(APROTO_MAGIC) + line.encode("ascii") + b"\x00"
    seq = host.send(CMD_TEXT_EXEC, payload)
    res = host.wait_response(seq, timeout)
    if res is None:
        return None, "TEXT_EXEC 超时(%.1fs)" % timeout
    rcmd, pl = res
    if rcmd != 0x8B:
        return None, "回包 cmd=0x%02X (非 0x8B)" % rcmd
    total   = pl[0] | (pl[1] << 8)
    flags   = pl[2]
    off     = pl[3] | (pl[4] << 8)
    n       = pl[5]
    text    = pl[6:6 + n]
    got     = off + n
    while got < total:                      # 翻页: 0x0C 负载 = u16 off, u8 n
        seq = host.send(CMD_TEXT_FETCH, bytes([got & 0xFF, (got >> 8) & 0xFF, 58]))
        res = host.wait_response(seq, 2.0)
        if res is None:
            text += ("\n[FETCH 超时, 已收 %d/%d]" % (got, total)).encode("ascii", "replace")
            break
        rcmd, pl = res
        if rcmd != 0x8C and rcmd != 0x8B:
            text += ("\n[FETCH 异常 cmd=0x%02X]" % rcmd).encode()
            break
        off2 = pl[3] | (pl[4] << 8)
        n2   = pl[5]
        text += pl[6:6 + n2]
        got  = off2 + n2
    out = text.decode("gbk", "replace")
    if flags & 1:
        out += "\n[输出被 2048B 截断]"
    return out, None


def get_status(host, timeout=1.0):
    seq = host.send(CMD_GET_STATUS)
    res = host.wait_response(seq, timeout)
    if res is None or res[0] != 0x82:
        return None
    return decode_status_payload(res[1])


def fmt(st):
    if st is None:
        return "  (无应答)"
    js = " ".join("?" if v is None else "%.1f" % v for v in st["joints"])
    fl = []
    if st["estop"]: fl.append("ESTOP")
    if st["traj"]:  fl.append("TRAJ")
    if not st["read_ok"]: fl.append("READ-FAIL")
    return "  J=[%s] xyz=(%.1f,%.1f,%.1f) %s" % (
        js, st["x"], st["y"], st["z"], ",".join(fl) or "ok")


def wait_stable(host, label, target=None, timeout=60.0, dt=0.7):
    """轮询直到: read_ok 且连续 4 次位移 < 0.2°/次; 若 target 给出则要求贴近。"""
    print("== 等待到位: %s (timeout %.0fs)" % (label, timeout))
    t0 = time.monotonic()
    prev = None
    stable = 0
    last = None
    while time.monotonic() - t0 < timeout:
        st = get_status(host)
        last = st
        print(fmt(st))
        if st is not None and st["read_ok"] and not st["estop"]:
            cur = st["joints"]
            if all(v is not None for v in cur):
                d = 0.0 if prev is None else max(abs(a - b) for a, b in zip(cur, prev))
                near = True
                if target:
                    near = all(abs(a - b) <= 4.0 for a, b in zip(cur, target))
                stable = stable + 1 if (d < 0.2 and near) else (1 if near else 0)
                prev = cur
                if stable >= 4:
                    print("  → 到位 (连续 %d 次稳定)\n" % stable)
                    return cur
        else:
            stable = 0
        time.sleep(dt)
    print("  ⚠ 超时未稳定, 最后状态:\n" + fmt(last))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-move", action="store_true")
    ap.add_argument("--cmd", action="append", default=None,
                    help="直接下发文本命令(可多次); 给了就只跑文本, 不跑 z a/P 0 流程")
    args = ap.parse_args()

    bus = open_bus_with_retry("pcan", "PCAN_USBBUS1", 500000)
    host = ArmCanHost(bus, node=1)
    try:
        st = get_status(host)
        print("== 初始状态 ==\n" + fmt(st))
        if args.cmd:
            for line in args.cmd:
                out, err = text_exec(host, line, timeout=8.0)
                print("== 下发 %r ==" % line)
                print((err or out) or "(空回传)")
            return 0
        if args.no_move:
            return 0

        out, err = text_exec(host, "z a")
        print("== 下发 'z a' ==\n" + (err or out))
        if err or (out and "read-only" in out):
            print("!! z a 未执行, 终止")
            return 1
        wait_stable(host, "z a 中位(各轴≈180)",
                    target=[180.0] * 6, timeout=60.0)

        out, err = text_exec(host, "P 0")
        print("== 下发 'P 0' (折叠位) ==\n" + (err or out))
        if err:
            print("!! P 0 回传异常")
            return 1
        # 折叠位参考(2026-11-09 实测 P 0 落点): 179 76.6 270.2 250.8 182.2 145.8
        wait_stable(host, "P 0 折叠位",
                    target=[178.8, 76.2, 270.3, 250.6, 182.7, 145.7],
                    timeout=60.0)

        st = get_status(host)
        print("== 最终状态 ==\n" + fmt(st))
        return 0
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
