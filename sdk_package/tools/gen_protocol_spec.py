#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_protocol_spec.py — 从 SDK 源码**自动生成**机器可读的网口协议定义。

产物：`host_integration/protocol_spec.json`

给谁用
------
* 上位机团队要**代码生成**（自动生成 C# 常量类 / TypeScript 类型 / 枚举）时的输入；
* CI 里做"文档 ↔ 实现"一致性校验；
* 不想读 Markdown 的工程师用 `jq` 直接查。

为什么是"生成"而不是"手写"
--------------------------
手写的协议 JSON 一个版本内就会和实现分叉（本项目已经吃过这个亏：
`arm_config.json` 里的命令表曾少算一条、且被手工注入过一整块）。
这里的所有事实都**从源码读出来**：

* 命令码 / 回应码 / 错误码 ↔ `arm_sdk/protocol.py` 的常量
* 请求/响应字段说明        ↔ 对应 `payload_*` / `decode_*` 函数的 **docstring**
  （原样搬运，**零二次转写** —— 改函数注释即改规范）
* 常量与上限              ↔ `protocol.py` 的模块级常量

跑法：
    python tools/gen_protocol_spec.py            # 生成
    python tools/gen_protocol_spec.py --check     # 只校验是否已同步（CI 用）
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from arm_sdk import protocol as P                                    # noqa: E402

OUT = os.path.join(PKG, "host_integration", "protocol_spec.json")

# ---------------------------------------------------------------------------
# CMD → (分组, 风险, 请求构造函数名, 响应码名, 备注)
# 风险等级：read=无副作用 / motion=会动电机 / write=改配置或落盘 / danger=不可逆
# ---------------------------------------------------------------------------
TABLE = [
    ("PING",             "query",  "read",   None,                     "RSP_PING",        None),
    ("GET_STATUS",       "query",  "read",   None,                     "RSP_STATUS",      "21 B；读失败轴 = -32768 哨兵"),
    ("LOG_SET_LEVEL",    "query",  "write",  "payload_log_set_level",  "ACK",             "level 0~4；越大越啰嗦，会占网口带宽"),
    ("GET_JOINT_DETAIL", "query",  "read",   "payload_get_joint_detail", "RSP_JOINT_DETAIL", "每轴 10 B；温度/硬件错有 0xFF 丢读哨兵"),
    ("GET_LIMITS",       "query",  "read",   None,                     "RSP_LIMITS",      "48 B = 6×8B(min,pos,max,zero，u16 raw)"),
    ("GET_INFO",         "query",  "read",   None,                     "RSP_INFO",        "20 B；fw_tag + build + link + 参数个数"),
    ("GET_FLAGS",        "query",  "read",   None,                     "RSP_FLAGS",       "2 B；不碰总线，可 10~50 Hz 轮询做心跳"),
    ("TEXT_EXEC",        "text",   "motion", "payload_text_exec",      "RSP_TEXT_OUT",    "★ 把整条控制台搬上网口；正文 ASCII + NUL，≤58 字符"),
    ("TEXT_FETCH",       "text",   "read",   "payload_text_fetch",     "RSP_TEXT_FETCH",  "取 TEXT_EXEC 输出的后续页；u16 off + u8 n"),
    ("MOVE_JOINTS",      "motion", "motion", "payload_move_joints",    "ACK",             "★ 绝对角，**舵机度**；v/a 是 u16×10"),
    ("STOP",             "motion", "motion", None,                     "ACK",             "随时可发，含 E-Stop 期间"),
    ("SET_TORQUE",       "motion", "motion", "payload_set_torque",     "ACK",             "joint=0xFF 全部；掉力矩的臂必须先上力"),
    ("STREAM_JOINTS",    "motion", "motion", "payload_stream_joints",  "ACK",             "★ 50 Hz 流控，**逻辑角增量**；单帧单轴 >25° 整帧拒收"),
    ("MOVE_JOINT_ONE",   "motion", "motion", "payload_move_joint_one", "ACK",             "单轴版 MOVE_JOINTS；任一轴丢读 ⇒ ERR_READ"),
    ("SET_BUS_MODE",     "config", "write",  "payload_set_bus_mode",   "ACK",             "0=正常 1=全A 2=全B"),
    ("SERVO_INIT",       "config", "write",  "payload_servo_init",     "ACK",             "舵机初始化（会动！）"),
    ("MOVE_CART",        "motion", "motion", "payload_move_cart",      "ACK",             "12 B；坐标原点是夹爪中心 TCP；**会归零 J5/J6**"),
    ("FK_QUERY",         "query",  "read",   None,                     "RSP_FK",          "正解：X/Y/Z/pitch + 六轴舵机度 + read_ok"),
    ("IK_QUERY",         "query",  "read",   "payload_ik_query",       "RSP_IK",          "逆解；无解回 ERR_UNREACH，位姿不符回 ERR_FK_VERIFY"),
    ("SET_GRAVITY",      "config", "write",  "payload_set_gravity",    "ACK",             "i16 K(×1000)；本项目 gc.k 为占位"),
    ("SET_CORRECTION",   "config", "write",  "payload_set_correction", "ACK",             "重力补偿开关/阈值/轮数"),
    ("GET_GRAVITY",      "query",  "read",   None,                     "RSP_GRAVITY",     "读回 K"),
    ("GET_CORRECTION",   "query",  "read",   None,                     "RSP_CORRECTION",  "读回补偿配置"),
    ("GET_PID",          "query",  "read",   "payload_get_pid",        "RSP_PID",         "读某轴 PID(P,I,D)"),
    ("PARAM_READ",       "param",  "read",   "payload_param_read",     "RSP_PARAM_READ",  "u8 id → u8 id + i32(×1000)"),
    ("PARAM_WRITE",      "param",  "write",  "payload_param_write",    "ACK",             "改运行时参数（要 PARAM_SAVE 才落盘）"),
    ("PARAM_COUNT",      "param",  "read",   None,                     "RSP_PARAM_COUNT", "现役 24 项"),
    ("PARAM_SAVE",       "param",  "write",  None,                     "ACK",             "★ 落 Flash；**必须带 MAGIC**；写扇区 11 区 A"),
    ("PARAM_LIST",       "param",  "read",   "payload_param_list",     "RSP_PARAM_LIST",  "分页：每页 ≤6 条 ×10 B；vmin/vmax 是 i32"),
    ("TEACH_LIST",       "teach",  "read",   "payload_teach_list",     "RSP_TEACH_LIST",  "示教文件枚举（有 tombstone 语义）"),
    ("TEACH_READ",       "teach",  "read",   "payload_teach_read",     "RSP_TEACH_READ",  "读采样点，每页 ≤5 点 ×12 B"),
    ("TEACH_DELETE",     "teach",  "write",  "payload_teach_delete",   "ACK",             "删一个文件"),
    ("TEACH_COMPACT",    "teach",  "write",  None,                     "ACK",             "压实（回收 tombstone）"),
    ("TEACH_TOTAL",      "teach",  "read",   None,                     "RSP_TEACH_TOTAL", "容量/剩余空间统计"),
    ("CENTER_CALIB",     "danger", "danger", None,                     "ACK",             "★ 六轴中位标定；**会废掉量纲基准**；必须 MAGIC"),
    ("CALIB_ENTER",      "danger", "danger", None,                     "NACK",            "★ 交互式向导 ⇒ 网口下发**一律 NACK**（会死等本机键盘，且本工程无看门狗）"),
    ("ZERO_RESET",       "danger", "danger", "payload_zero_reset",     "ACK",             "★ 清 Home 零点；flags bit0=是否落盘；**无关节号**（一次全六轴）"),
    ("SYS_RESET",        "danger", "danger", None,                     "ACK",             "★ 软复位；固件**延后到回包之后**执行，所以能收到 ACK"),
]


def _doc(fn_name):
    """取 payload_/decode_ 函数的 docstring 首段（原样，不转写）。"""
    if not fn_name:
        return None
    fn = getattr(P, fn_name, None)
    if fn is None or not fn.__doc__:
        return None
    lines = [l.rstrip() for l in fn.__doc__.strip().splitlines()]
    return "\n".join(lines)


def build() -> dict:
    cmds = []
    for name, group, risk, req_fn, rsp_name, note in TABLE:
        code = getattr(P, "CMD_" + name, None)
        if code is None:
            raise SystemExit("protocol.py 里没有 CMD_%s —— 本表要同步更新" % name)
        rsp = getattr(P, rsp_name, None) if rsp_name else None
        req = None
        if req_fn:
            req = {"builder": req_fn, "spec": _doc(req_fn)}
        else:
            req = {"builder": None, "spec": "无载荷（空 payload）"}
        cmds.append({
            "code": code,
            "hex": "0x%02X" % code,
            "name": name,
            "group": group,
            "risk": risk,
            "request": req,
            "response": {"name": rsp_name, "code": rsp,
                         "hex": ("0x%02X" % rsp) if rsp else None},
            "note": note,
        })
    cmds.sort(key=lambda c: c["code"])

    # 名字对不上就报错 —— 防止"表里写了但实现没有"
    have = {v for k, v in vars(P).items() if k.startswith("CMD_") and isinstance(v, int)}
    listed = {c["code"] for c in cmds}
    missing = sorted(have - listed)
    if missing:
        raise SystemExit("有 %d 条命令没登记进本生成器：%s"
                         % (len(missing), [hex(m) for m in missing]))

    consts = {}
    for k in ("MAX_PAYLOAD", "MAGIC", "RAW_READ_FAIL", "TEMP_LOST", "HWERR_LOST",
              "TEXT_BUF_SZ", "TEXT_EXEC_MAX", "TEXT_CHUNK", "TEXT_FLAG_TRUNC",
              "TEXT_FLAG_GATE", "DETAIL_SIZE", "TEACH_READ_MAX", "TEACH_SAMPLE_SIZE",
              "PARAM_LIST_MAX"):
        consts[k] = getattr(P, k, None)

    params = []
    for pid, (name, *_rest) in sorted(P.PARAM_TABLE.items()):
        params.append({"id": pid, "name": name})

    return {
        "_": "由 tools/gen_protocol_spec.py 从 arm_sdk/protocol.py 自动生成 —— 不要手改。",
        "meta": {
            "sdk_version": getattr(__import__("arm_sdk"), "__version__", "?"),
            "generator": "tools/gen_protocol_spec.py",
            "command_count": len(cmds),
        },
        "transport": {
            "kind": "udp",
            "port": P.DEFAULT_PORT if hasattr(P, "DEFAULT_PORT") else 5000,
            "one_datagram_per_packet": True,
            "packet_layout": [
                {"field": "SEQ", "type": "u8", "off": 0,
                 "note": "自增；回包原样带回，**必须按它匹配**（UDP 会乱序）"},
                {"field": "CMD", "type": "u8", "off": 1,
                 "note": "命令码 < 0x80；≥ 0x80 是回包号段，固件不回"},
                {"field": "LEN", "type": "u8", "off": 2,
                 "note": "**1 字节**（0~%d），不是 u16" % consts["MAX_PAYLOAD"]},
                {"field": "PAYLOAD", "type": "u8[LEN]", "off": 3, "note": ""},
                {"field": "CRC", "type": "u16le", "off": "-2",
                 "note": "小端；范围 = 前 3+LEN 字节"},
            ],
            "total_len_formula": "3 + LEN + 2",
            "bad_packet_policy": "长度/CRC 不符 ⇒ 固件静默丢弃（只表现为超时）；"
                                 "若能看出 CMD 且 < 0x80 则回 NACK(err=2)",
        },
        "crc16": {
            "name": "CRC-16/CCITT-FALSE",
            "poly": 0x1021, "init": 0xFFFF, "reflect_in": False,
            "reflect_out": False, "xor_out": 0x0000,
            "check_vector": {"input": "123456789", "expect": "0x29B1"},
            "scope": "bytes[0 : 3+LEN]",
            "endian": "little",
        },
        "error_codes": {e.value: {"name": e.name,
                                  "hint": _ERR_HINT.get(e.value, "")}
                        for e in P.ErrCode},
        "ack_nack": {"0x7F": "ACK（无载荷）", "0x7E": "NACK（payload = [原cmd, err]）"},
        "commands": cmds,
        "constants": consts,
        "param_table": params,
        "risk_levels": {
            "read": "无副作用，可随时发",
            "motion": "会动电机（流控/绝对角/单轴/笛卡尔）",
            "write": "改配置或落 Flash，UI 上要二次确认",
            "danger": "不可逆或会废掉量纲基准，**必须带 MAGIC**，UI 上要红字二次确认",
        },
    }


_ERR_HINT = {
    0: "成功", 1: "参数非法（量纲用错/越界/枚举无此值）", 2: "CRC 或长度错",
    3: "IK 无解 / 目标不可达", 4: "越限（含单帧跳变 >25° 被整帧拒收）",
    5: "急停生效", 6: "上报位姿与正解不符（安全拒动）", 7: "舵机读失败（有轴丢读）",
    8: "危险命令缺/错 MAGIC", 9: "轨迹/示教/回放进行中，此刻不受理", 10: "未校准",
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="从 SDK 源码生成机器可读协议定义")
    ap.add_argument("--check", action="store_true", help="只校验是否已同步（CI 用）")
    a = ap.parse_args(argv)

    spec = build()
    text = json.dumps(spec, ensure_ascii=False, indent=2, sort_keys=False) + "\n"

    if a.check:
        if not os.path.exists(OUT):
            print("  ✗ %s 不存在 —— 跑 python tools/gen_protocol_spec.py" % OUT)
            return 1
        cur = io.open(OUT, encoding="utf-8").read()
        if cur != text:
            print("  ✗ protocol_spec.json 已过期 —— 跑 python tools/gen_protocol_spec.py")
            return 1
        print("  ✓ protocol_spec.json 与源码一致（%d 条命令）" % spec["meta"]["command_count"])
        return 0

    io.open(OUT, "w", encoding="utf-8", newline="\n").write(text)
    print("  已写 %s（%d 条命令 / %d B）"
          % (os.path.relpath(OUT, PKG), spec["meta"]["command_count"], len(text)))
    # 分组统计
    g = {}
    for c in spec["commands"]:
        g[c["group"]] = g.get(c["group"], 0) + 1
    print("  分组：%s" % ", ".join("%s=%d" % kv for kv in sorted(g.items())))
    r = {}
    for c in spec["commands"]:
        r[c["risk"]] = r.get(c["risk"], 0) + 1
    print("  风险：%s" % ", ".join("%s=%d" % kv for kv in sorted(r.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
