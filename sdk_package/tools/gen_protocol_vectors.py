#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_protocol_vectors.py — 用 **Python 版** protocol.py 生成对拍向量。

产物：js/test/protocol_vectors.json

用途：让 JS 版 protocol.js 拿这些向量逐字节比对，证明两个实现完全一致。
     任何一边改了协议，这个对拍就会红 —— 比人肉 review 可靠。

向量种类
--------
  pack    : (seq, cmd, payload_hex) → 逻辑包 hex（不含前缀）
  frame   : 同上 → 带 AA55 前缀的 hex（串口承载）
  can     : 同上 → CAN 帧列表 hex
  status  : 21 字节负载 hex → decode_status 结果
  fk      : 21 字节负载 hex → decode_fk 结果
  param   : 5 字节负载 hex → decode_param_read 结果
  calib   : (servo_deg, ) → 逻辑角 / 反过来
"""
from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "arm_sdk"))

import protocol as P  # noqa: E402


def u32(v: int) -> bytes:
    return int(v & 0xFFFFFFFF).to_bytes(4, "little")


def hx(b) -> str:
    return " ".join("%02X" % x for x in bytes(b))


def main() -> int:
    vec = {"_gen": "tools/gen_protocol_vectors.py", "_note": "JS 版必须逐字节命中"}

    # ---- 1. 打包向量 ----
    pack_cases = [
        (0x31, P.CMD_PING, b""),
        (0x43, P.CMD_PARAM_COUNT, b""),
        (0x00, P.CMD_GET_STATUS, b""),
        (0xFF, P.CMD_STOP, b""),
        (0x11, P.CMD_STREAM_JOINTS, P.payload_stream_joints([0, 10, 20, 30, 40, 50])),
        (0x11, P.CMD_STREAM_JOINTS, P.payload_stream_joints([-180, -90, 0, 90, 179.9, 180])),
        (0x22, P.CMD_MOVE_JOINTS, P.payload_move_joints([180] * 6)),
        (0x22, P.CMD_MOVE_JOINTS, P.payload_move_joints([0, 90, 180, 270, 359.9, 12.34], 25, 80)),
        (0x33, P.CMD_MOVE_CART, P.payload_move_cart(300.5, -12.3, 200, 90, 20, 50)),
        (0x34, P.CMD_SET_TORQUE, P.payload_set_torque(0xFF, True)),
        (0x34, P.CMD_SET_TORQUE, P.payload_set_torque(0, False)),
        (0x35, P.CMD_SET_GRAVITY, P.payload_set_gravity(0.123)),
        (0x36, P.CMD_SET_CORRECTION, P.payload_set_correction(True, 2.5, 10)),
        (0x40, P.CMD_PARAM_READ, P.payload_param_read(42)),
        (0x41, P.CMD_PARAM_WRITE, P.payload_param_write(3, 42.5)),
        (0x41, P.CMD_PARAM_WRITE, P.payload_param_write(33, -1.234)),
        (0x03, P.CMD_LOG_SET_LEVEL, P.payload_log_set_level(4)),
        # ---- 2026-09-23 扩展：全指令覆盖 ----
        (0x50, P.CMD_GET_JOINT_DETAIL, P.payload_get_joint_detail(0xFF)),
        (0x51, P.CMD_GET_JOINT_DETAIL, P.payload_get_joint_detail(2)),
        (0x52, P.CMD_GET_LIMITS, b""),
        (0x53, P.CMD_GET_INFO, b""),
        (0x54, P.CMD_GET_FLAGS, b""),
        (0x55, P.CMD_MOVE_JOINT_ONE, P.payload_move_joint_one(2, 100.5, 30, 80)),
        (0x56, P.CMD_MOVE_JOINT_ONE, P.payload_move_joint_one(0, 180, 0, 0)),
        (0x57, P.CMD_SET_BUS_MODE, P.payload_set_bus_mode(2)),
        (0x58, P.CMD_SERVO_INIT, P.payload_servo_init(0xFF)),
        (0x59, P.CMD_IK_QUERY, P.payload_ik_query(200, -12.3, 150, 90)),
        (0x5A, P.CMD_GET_GRAVITY, b""),
        (0x5B, P.CMD_GET_CORRECTION, b""),
        (0x5C, P.CMD_GET_PID, P.payload_get_pid(3)),
        (0x5D, P.CMD_PARAM_SAVE, P.payload_magic()),
        (0x5E, P.CMD_PARAM_LIST, P.payload_param_list(6)),
        (0x5F, P.CMD_TEACH_LIST, P.payload_teach_list(2)),
        (0x60, P.CMD_TEACH_READ, P.payload_teach_read(1, 300, 5)),
        (0x61, P.CMD_TEACH_READ, P.payload_teach_read(0, 0, 99)),      # n 被钳到 5
        (0x62, P.CMD_TEACH_DELETE, P.payload_teach_delete(7)),
        (0x63, P.CMD_TEACH_COMPACT, b""),
        (0x64, P.CMD_TEACH_TOTAL, b""),
        (0x65, P.CMD_CENTER_CALIB, P.payload_magic()),
        (0x66, P.CMD_CALIB_ENTER, P.payload_magic()),
        (0x67, P.CMD_ZERO_RESET, P.payload_zero_reset(False)),
        (0x68, P.CMD_ZERO_RESET, P.payload_zero_reset(True)),
        (0x69, P.CMD_SYS_RESET, P.payload_magic()),
        (0x6A, P.CMD_TEXT_EXEC, P.payload_text_exec("s")),
        (0x6B, P.CMD_TEXT_EXEC, P.payload_text_exec("link can")),
        (0x6C, P.CMD_TEXT_EXEC, P.payload_text_exec("M a")),
        (0x6D, P.CMD_TEXT_FETCH, P.payload_text_fetch(58, 58)),
        (0x6E, P.CMD_TEXT_FETCH, P.payload_text_fetch(0, 58)),
    ]
    vec["pack"] = [
        {"seq": s, "cmd": c, "payload": hx(p), "logical": hx(P.pack_logical(s, c, p)),
         "frame": hx(P.APROTO_HEAD + P.pack_logical(s, c, p))}
        for s, c, p in pack_cases
    ]

    # ---- 2. CAN 分片 ----
    vec["can"] = [
        {"seq": s, "cmd": c, "payload": hx(p),
         "frames": [hx(f) for f in P.can_build_frames(s, c, p)]}
        for s, c, p in pack_cases
    ]

    # ---- 3. 解码向量 ----
    def status_payload(joints, x, y, z, pitch, flags):
        b = b""
        for v in joints:
            b += P._i16(v * 10)
        b += P._i16(x * 10) + P._i16(y * 10) + P._i16(z * 10) + P._i16(pitch * 10)
        return b + bytes([flags])

    st_cases = [
        ([180] * 6, 300.0, 0.0, 200.0, 90.0, 0x08),
        ([180, 180, P.RAW_READ_FAIL, 200, 180, 180], 0.0, 0.0, 0.0, 0.0, 0x00),
        ([0, 90, 180, 270, 359.9, 12.3], -12.5, 33.3, 45.6, -80.0, 0x0F),
    ]
    vec["status"] = []
    for j, x, y, z, pi, fl in st_cases:
        pl = status_payload(j, x, y, z, pi, fl)
        s = P.decode_status(pl)
        vec["status"].append({
            "payload": hx(pl),
            "joints": s.joints, "x": s.x, "y": s.y, "z": s.z, "pitch": s.pitch,
            "flags": s.flags, "estop": s.estop, "traj": s.traj,
            "corr": s.corr, "read_ok": s.read_ok,
            "failed_joints": s.failed_joints(),
        })

    def fk_payload(x, y, z, pitch, joints, read_ok):
        b = P._i16(x * 10) + P._i16(y * 10) + P._i16(z * 10) + P._i16(pitch * 10)
        for v in joints:
            b += P._i16(v * 10)
        return b + bytes([read_ok])

    fk_cases = [
        (62.0, -12.3, 200.5, 90.0, [180] * 6, 1),
        (0.0, 0.0, 0.0, 0.0, [180, P.RAW_READ_FAIL, 180, 180, 180, 180], 0),
    ]
    vec["fk"] = []
    for x, y, z, pi, j, ro in fk_cases:
        pl = fk_payload(x, y, z, pi, j, ro)
        r = P.decode_fk(pl)
        vec["fk"].append({"payload": hx(pl), "x": r.x, "y": r.y, "z": r.z,
                          "pitch": r.pitch, "joints": r.joints, "read_ok": r.read_ok})

    vec["param"] = [
        {"payload": hx(P.payload_param_read(3) + b""),
         "read_payload": hx(bytes([3]) + P._i32(42500)),
         "id": 3, "value": 42.5},
    ]
    vec["param_count"] = {"payload": hx(bytes([24])), "count": 24}

    # ---- 4. 角度换算 ----
    cal = P.JointCalib()
    cal_cases = [[180] * 6, [170, 190, 180, 175, 185, 200],
                 [0, 90, 180, 270, 359.9, 12.3]]
    vec["calib"] = [
        {"servo": s, "logical": cal.to_logical(s), "back": cal.to_servo(cal.to_logical(s))}
        for s in cal_cases
    ]
    vec["wrap180"] = [{"in": v, "out": P.wrap180(v)}
                      for v in [-180, -90, 0, 90, 180, 190, 360, 540, -190]]

    # ---- 5. 定点编码（负半轴） ----
    vec["round"] = [{"in": v, "out": P.round_half_away(v)}
                    for v in [0.5, -0.5, 2.5, -2.5, 1.234, -1.234, 1.567, -1.567,
                              0.0, -0.0, 0.49, -0.49, 12.345, -12.345]]

    # ---- 5b. 2026-09-23 扩展：新命令的响应解码 ----
    #  ⚠ 哨兵必须逐字节对上：TEMP_LOST=0xFF / HWERR_LOST=0xFF 是"这一拍丢读"，
    #    不是 0 度、也不是"硬件正常"。JS 侧解码里保留了 *_lost 布尔，正是为此。
    def jd_item(pos_raw, speed, load, vbat, temp, hw, ok):
        return (P._u16(pos_raw) + P._i16(speed) + P._i16(load)
                + bytes([vbat, temp, hw, ok]))

    jd_6 = (bytes([6, 0]) + b"".join(
        jd_item(2048 + 100 * k, -12, 33, 120, 42, 0, 1) for k in range(6)))
    jd_lost = (bytes([2, 2]) + jd_item(0, 0, 0, 0, P.TEMP_LOST, P.HWERR_LOST, 0)
               + jd_item(3000, 5, -7, 118, 39, 0, 1))
    vec["joint_detail"] = []
    for pl, tag in ((jd_6, "6axis"), (jd_lost, "lost")):
        items = P.decode_joint_detail(pl)
        vec["joint_detail"].append({
            "tag": tag, "payload": hx(pl),
            "items": [{"joint": d.joint, "pos_raw": d.pos_raw, "speed": d.speed,
                       "load": d.load, "voltage": d.voltage, "temp": d.temp,
                       "hw_err": d.hw_err, "ok": d.ok,
                       "temp_lost": d.temp_lost, "hw_err_lost": d.hw_err_lost}
                      for d in items],
        })

    lim_pl = b"".join(P._u16(mn) + P._u16(pos) + P._u16(mx) + P._u16(z)
                      for mn, pos, mx, z in
                      [(10, 2048, 4090, 2048), (0, 1000, 4095, 0),
                       (100, 2000, 3000, 1500), (5, 2048, 4000, 2048),
                       (50, 2048, 3900, 0), (0, 2048, 4095, 2048)])
    lm = P.decode_limits(lim_pl)
    vec["limits"] = {"payload": hx(lim_pl), "min_raw": lm.min_raw,
                     "pos_raw": lm.pos_raw, "max_raw": lm.max_raw,
                     "zero_raw": lm.zero_raw}

    tag = b"ARM-NET-0921A"
    info_pl = bytes([len(tag)]) + tag + bytes(16 - len(tag)) + bytes([0, 0, 24])
    iv = P.decode_info(info_pl)
    vec["info"] = {"payload": hx(info_pl), "fw_tag": iv.fw_tag, "build": iv.build,
                   "link_mode": iv.link_mode, "n_params": iv.n_params}

    vec["flags"] = []
    for f, rid in ((0x00, 0x5A), (0x01, 0x5A), (0x3F, 0x11), (0x08, 0x00)):
        fl = P.decode_flags(bytes([f, rid]))
        vec["flags"].append({"payload": hx(bytes([f, rid])), "raw": fl.raw,
                             "run_id": fl.run_id})

    ik_pl = b"".join(P._i16(v) for v in
                     [1800, 3277, 464, 781, 1800, 1800])
    vec["ik_query"] = {"payload": hx(ik_pl), "servo_deg": P.decode_ik_query(ik_pl)}

    vec["gravity"] = [
        {"payload": hx(P._i16(v)), "value": P.decode_gravity(P._i16(v))}
        for v in (123, -45, 0)
    ]

    corr_pl = bytes([1]) + P._u16(250) + bytes([10, 0])
    cv = P.decode_correction(corr_pl)
    vec["correction"] = {"payload": hx(corr_pl), "enabled": cv.enabled,
                         "threshold_deg": cv.threshold_deg,
                         "max_rounds": cv.max_rounds, "pending": cv.pending}

    vec["pid"] = []
    for p_, i_, d_, ee in ((32, 0, 1, 1), (8, 2, 16, 0)):
        pv = P.decode_pid(bytes([p_, i_, d_, ee]))
        vec["pid"].append({"payload": hx(bytes([p_, i_, d_, ee])),
                           "p": pv.p, "i": pv.i, "d": pv.d,
                           "from_eeprom": pv.from_eeprom})

    # PARAM_LIST 一页 6 条（2 + 6×10 = 62 ≤ 64）
    ids = sorted(P.PARAM_TABLE.keys())[:6]
    pl_pl = bytes([len(P.PARAM_TABLE), len(ids)])
    metas = []
    for pid in ids:
        spec = P.PARAM_TABLE[pid]
        vmin, vmax = spec[2], spec[3]
        pl_pl += bytes([pid, 0]) + P._i32(int(round(vmin * 1000))) \
            + P._i32(int(round(vmax * 1000)))
    tot, items = P.decode_param_list(pl_pl)
    for m in items:
        metas.append({"id": m.id, "type": m.type, "vmin": m.vmin, "vmax": m.vmax})
    vec["param_list"] = {"payload": hx(pl_pl), "total": tot, "items": metas,
                         "max_per_page": P.PARAM_LIST_MAX}

    tl_pl = bytes([3, 1, 1]) + b"demo run" + b"\x00" * 4 + u32(1234)
    tv = P.decode_teach_list(tl_pl)
    vec["teach_list"] = {"payload": hx(tl_pl), "n_files": tl_pl[0], "idx": tv.idx,
                         "valid": tv.valid, "name": tv.name, "samples": tv.samples}

    tr_pl = bytes([2]) + b"".join(P._u16(2048 + 7 * k + j) for k in range(2) for j in range(6))
    vec["teach_read"] = {"payload": hx(tr_pl), "points": P.decode_teach_read(tr_pl),
                         "max_per_page": P.TEACH_READ_MAX,
                         "sample_size": P.TEACH_SAMPLE_SIZE}

    tt_pl = bytes([5, 32]) + u32(4321) + u32(60000)
    tv2 = P.decode_teach_total(tt_pl)
    vec["teach_total"] = {"payload": hx(tt_pl), "n_files": tv2.n_files,
                          "capacity": tv2.capacity, "total_samples": tv2.total_samples,
                          "free_bytes": tv2.free_bytes}

    # 文本透传：中文必须是 UTF-8 字节，且 offset/total 都是**字节**
    txt = "  J1 就绪 —— ✓\r\n".encode("utf-8")
    to_pl = P._u16(len(txt)) + bytes([P.TEXT_FLAG_TRUNC | P.TEXT_FLAG_GATE]) \
        + P._u16(3) + bytes([len(txt) - 3]) + txt[3:]
    to = P.decode_text_out(to_pl)
    vec["text_out"] = {"payload": hx(to_pl), "total": to.total, "off": to.off,
                       "truncated": to.truncated, "gated": to.gated,
                       "raw_hex": hx(to.raw), "text": to.text}

    # ---- 6. 常量 ----
    vec["const"] = {
        "crc_123456789": P.crc16(b"123456789"),
        "max_payload": P.MAX_PAYLOAD,
        "head": [P.APROTO_HEAD0, P.APROTO_HEAD1],
        "cmd": {k[4:]: v for k, v in vars(P).items()
                if k.startswith("CMD_") and isinstance(v, int)},
        "rsp": {k[4:]: v for k, v in vars(P).items()
                if k.startswith("RSP_") and isinstance(v, int)},
        "err": {k: int(v) for k, v in P.ErrCode.__members__.items()},
        "param_ids": sorted(P.PARAM_TABLE.keys()),
        "joint_dir": list(P.JOINT_DIR),
        "default_offset": list(P.DEFAULT_SERVO_OFFSET),
        # ---- 2026-09-23 扩展 ----
        "magic": P.MAGIC,
        "text_buf_sz": P.TEXT_BUF_SZ,
        "text_exec_max": P.TEXT_EXEC_MAX,
        "text_chunk": P.TEXT_CHUNK,
        "text_flag_trunc": P.TEXT_FLAG_TRUNC,
        "text_flag_gate": P.TEXT_FLAG_GATE,
        "detail_size": P.DETAIL_SIZE,
        "teach_sample_size": P.TEACH_SAMPLE_SIZE,
        "teach_read_max": P.TEACH_READ_MAX,
        "param_list_max": P.PARAM_LIST_MAX,
        "temp_lost": P.TEMP_LOST,
        "hwerr_lost": P.HWERR_LOST,
    }

    out = os.path.join(ROOT, "js", "test", "protocol_vectors.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(vec, f, ensure_ascii=False, indent=1)
    print("写出 %s" % out)
    print("  pack %d / can %d / status %d / fk %d / calib %d / round %d"
          % (len(vec["pack"]), len(vec["can"]), len(vec["status"]),
             len(vec["fk"]), len(vec["calib"]), len(vec["round"])))
    print("  扩展 %d 组" % len([k for k in vec
                               if k in ("joint_detail", "limits", "info", "flags",
                                        "ik_query", "gravity", "correction", "pid",
                                        "param_list", "teach_list", "teach_read",
                                        "teach_total", "text_out")]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
