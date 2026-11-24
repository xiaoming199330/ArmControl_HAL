# -*- coding: utf-8 -*-
"""new_commands_test.py — 2026-09-23 扩展命令的离线端到端回归测试。

全部跑在 `SimArmTransport`（纯内存假臂）上，不需要硬件。覆盖：
  · 只读类：GET_JOINT_DETAIL / GET_LIMITS / GET_INFO / GET_FLAGS
  · 运动类：MOVE_JOINT_ONE / SET_BUS_MODE / IK_QUERY
  · 补偿类：GET_GRAVITY / GET_CORRECTION / GET_PID
  · 参数类：PARAM_LIST（自动翻页）
  · 示教类：TEACH_TOTAL / TEACH_LIST / TEACH_READ（分页）/ DELETE / COMPACT
  · 危险类：PARAM_SAVE / CENTER_CALIB / ZERO_RESET / SYS_RESET / CALIB_ENTER
  · ⭐ 文本透传：TEXT_EXEC / TEXT_FETCH（含闸门拒绝、急停白名单、
     引脚命令拦截、magic 校验、分页）

用法:  python tests/new_commands_test.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_sdk import open_arm                                   # noqa: E402
from arm_sdk.client import ArmCommandError                     # noqa: E402
from arm_sdk.protocol import (ErrCode, MAGIC, TEXT_BUF_SZ,     # noqa: E402
                              TEXT_EXEC_MAX, TEXT_FLAG_GATE,
                              TEXT_FLAG_TRUNC, decode_text_out,
                              payload_text_exec,
                              payload_text_fetch)

PASS = 0
FAIL = 0


def chk(name, cond, extra=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS %s%s" % (name, ("  " + str(extra)) if extra else ""))
    else:
        FAIL += 1
        print("  FAIL %s%s" % (name, ("  " + str(extra)) if extra else ""))


def expect_nack(name, fn, err):
    """fn 可以是**返回 Reply** 的控制类方法，也可以是**抛 ArmCommandError**
    的读取类方法 —— 两类都要能验，因为 SDK 对它们的行为不同。"""
    global PASS, FAIL
    try:
        r = fn()
    except ArmCommandError as e:
        r = e.reply
    except Exception as e:                                    # noqa: BLE001
        FAIL += 1
        print("  FAIL %s -> 抛了非 ArmCommandError: %r" % (name, e))
        return
    if r is None or r.ok:
        FAIL += 1
        print("  FAIL %s -> 期望 NACK，但成功了" % name)
        return
    if r.err == int(err):
        PASS += 1
        print("  PASS %s -> err=%s" % (name, r.err_name))
    else:
        FAIL += 1
        print("  FAIL %s -> 期望 err=%d, 实际 err=%d(%s)"
              % (name, int(err), r.err, r.err_name))


def main() -> int:
    c = open_arm("sim://", timeout=0.5)

    # ================= 只读类 =================
    print("\n[只读: 遥测 / 限位 / 信息 / 状态]")
    det = c.get_joint_detail(0xFF)
    chk("GET_JOINT_DETAIL 六轴返回 6 条", len(det) == 6, "n=%d" % len(det))
    chk("六轴 joint 索引 0..5", [d.joint for d in det] == list(range(6)))
    chk("单轴返回 1 条且 joint 正确", [d.joint for d in c.get_joint_detail(3)] == [3])
    chk("轴 0 读数正常 ok=True", det[0].ok and det[0].temp == 42
        and abs(det[0].voltage - 12.0) < 1e-6)
    chk("各轴 pos_raw 与舵机度自洽",
        all(abs(d.pos_raw - round(180 / 360 * 4095)) <= 1 for d in det))

    c.transport.set_joint_fail([2])           # 让 J3 整组丢读
    d3 = c.get_joint_detail(0xFF)[2]
    chk("丢读轴 ok=False", not d3.ok)
    chk("丢读轴 temp 是 0xFF 哨兵（不是 0 度）", d3.temp == 0xFF and d3.temp_lost)
    chk("丢读轴 hw_err 是 0xFF 哨兵（≠ 硬件报错）", d3.hw_err == 0xFF and d3.hw_err_lost)
    c.transport.set_joint_fail([])

    lim = c.get_limits()
    chk("GET_LIMITS 48B 解码出 6 轴", len(lim.min_raw) == 6 and len(lim.zero_raw) == 6)
    chk("未校准时 calibrated 全 False（zero_raw==0）",
        lim.calibrated == [False] * 6)

    info = c.get_info()
    chk("GET_INFO fw_tag 非空", info.fw_tag.startswith("SIM"), repr(info.fw_tag))
    chk("GET_INFO n_params = 24", info.n_params == 24)
    chk("GET_INFO build_name 可读", info.build_name == "net", info.build_name)

    fl = c.get_flags()
    chk("GET_FLAGS 初始无 E-Stop / 无轨迹", not fl.estop and not fl.traj)
    c.transport.set_estop(True)
    chk("GET_FLAGS 能反映 E-Stop", c.get_flags().estop)
    c.transport.set_estop(False)

    # ================= 运动类 =================
    print("\n[运动: 单轴 / 总线模式 / IK 查询]")
    r = c.move_joint_one(0, 100.0, 30.0, 10.0)
    chk("MOVE_JOINT_ONE ACK", r.ok)
    chk("MOVE_JOINT_ONE 目标已写入",
        abs(c.transport.target_deg[0]
            - c.transport._clamp_servo([100, 180, 180, 180, 180, 180])[0]) < 1e-6)

    c.transport.set_joint_fail([4])
    expect_nack("MOVE_JOINT_ONE 任一轴丢读 -> ERR_READ", lambda: c.move_joint_one(0, 150), ErrCode.READ)
    c.transport.set_joint_fail([])

    c.transport.set_estop(True)
    expect_nack("MOVE_JOINT_ONE 急停 -> ERR_ESTOP", lambda: c.move_joint_one(0, 150), ErrCode.ESTOP)
    c.transport.set_estop(False)

    chk("SET_BUS_MODE 0/1/2 全部 ACK",
        all(c.set_bus_mode(m).ok for m in (0, 1, 2)))
    chk("SET_BUS_MODE 状态落进假臂", c.transport.bus_mode == 2)
    try:
        c.set_bus_mode(3)
        chk("SET_BUS_MODE 3 在客户端就被拦住（非法值不发帧）", False)
    except ValueError:
        chk("SET_BUS_MODE 3 在客户端就被拦住（非法值不发帧）", True)

    ik = c.ik_query(0.0, 250.0, 200.0, 0.0)
    chk("IK_QUERY 可达点返回 6 个关节角", len(ik) == 6,
        "deg=%s" % [round(v, 1) for v in ik])
    expect_nack("IK_QUERY 远超臂展 -> ERR_UNREACH",
                lambda: c.ik_query(0.0, 2000.0, 2000.0, 0.0), ErrCode.UNREACH)

    # ================= 补偿 / PID =================
    print("\n[补偿 / PID 补读侧]")
    chk("GET_GRAVITY 与 SET_GRAVITY 闭环",
        c.set_gravity(0.05).ok and abs(c.get_gravity() - 0.05) < 1e-6,
        "K=%.4f" % c.get_gravity())
    cc = c.set_correction(True, 1.50, 8)
    corr = c.get_correction()
    chk("GET_CORRECTION 与 SET_CORRECTION 闭环",
        cc.ok and corr.enabled and abs(corr.threshold_deg - 1.50) < 1e-6
        and corr.max_rounds == 8,
        "en=%s thr=%.2f rounds=%d" % (corr.enabled, corr.threshold_deg, corr.max_rounds))

    pid = c.get_pid(2)
    chk("GET_PID 返回 p/i/d 且读自 EEPROM", pid.from_eeprom and pid.p == 32,
        "p=%d i=%d d=%d" % (pid.p, pid.i, pid.d))
    c.transport.set_joint_fail([2])
    chk("GET_PID 丢读轴 from_eeprom=False", not c.get_pid(2).from_eeprom)
    c.transport.set_joint_fail([])

    # ================= 参数表分页 =================
    print("\n[参数表 PARAM_LIST（自动翻页）]")
    metas = c.param_list()
    chk("PARAM_LIST 取满 24 条", len(metas) == 24, "n=%d" % len(metas))
    chk("PARAM_LIST id 去重且升序", [m.id for m in metas] == sorted(set(m.id for m in metas)))
    m32 = [m for m in metas if m.id == 32][0]
    chk("PARAM_LIST 元数据范围合理", m32.vmax > m32.vmin,
        "id32 vmin=%.3f vmax=%.3f" % (m32.vmin, m32.vmax))

    # ================= 示教存储 =================
    print("\n[示教存储]")
    tt = c.teach_total()
    chk("TEACH_TOTAL 返回文件数与容量", tt.n_files == 2 and tt.capacity == 32,
        "n=%d cap=%d total=%d free=%d" % (tt.n_files, tt.capacity,
                                          tt.total_samples, tt.free_bytes))
    files = c.teach_list_all()
    chk("TEACH_LIST 列全 2 个槽位", len(files) == 2, [f.name for f in files])
    chk("TEACH_LIST 能区分 tombstone", [f.valid for f in files] == [True, False])

    tr = c.teach_trace(0)
    chk("TEACH_READ 分页取回 12 点", len(tr) == 12, "n=%d" % len(tr))
    chk("TEACH_READ 每点 6 个 raw", all(len(p) == 6 for p in tr))

    chk("TEACH_DELETE ACK", c.teach_delete(0).ok)
    chk("TEACH_COMPACT ACK", c.teach_compact().ok)
    # idx1 本来就是 tombstone，idx0 刚被删 ⇒ compact 后应该一个都不剩
    chk("COMPACT 后 tombstone 全部回收（0 个槽位）", c.teach_total().n_files == 0,
        "n=%d" % c.teach_total().n_files)
    expect_nack("TEACH_LIST 越界 idx -> ERR_PARAM",
                lambda: c.teach_list(9), ErrCode.PARAM)

    # ================= 危险操作 =================
    print("\n[危险操作（magic 由 SDK 自动填）]")
    chk("PARAM_SAVE ACK", c.param_save().ok)
    chk("CENTER_CALIB ACK 且中位被写到 2048",
        c.center_calib_danger().ok and c.transport.pos_raw == [2048] * 6)
    chk("ZERO_RESET(save=False) ACK", c.zero_reset_danger().ok)
    chk("ZERO_RESET(save=True) ACK", c.zero_reset_danger(save=True).ok)
    n0 = c.transport.sys_reset_count
    chk("SYS_RESET ACK 且记账一次",
        c.sys_reset_danger().ok and c.transport.sys_reset_count == n0 + 1)
    expect_nack("CALIB_ENTER 固件一律拒绝（控制台专属）",
                lambda: c.calib_enter_danger(), ErrCode.PARAM)

    # magic 错必须被拒
    bad = c.request(0x70, b"\x00\x00\x00\x00")      # CENTER_CALIB, magic=0
    chk("magic 不符 -> NACK err=8(MAGIC)", (not bad.ok) and bad.err == int(ErrCode.MAGIC),
        bad.err_name)

    # ================= 文本透传 =================
    print("\n[⭐ 文本命令透传 TEXT_EXEC / TEXT_FETCH]")
    out = c.text_exec("s")
    chk("text_exec('s') 有回显", "J1" in out and "J6" in out, repr(out[:60]))

    out = c.text_exec("zi")
    chk("text_exec('zi') 输出含未校准警告", "未校准" in out)

    out = c.text_exec("F")
    chk("text_exec('F') = 参数落盘", "saved" in out.lower())

    out = c.text_exec("M a")
    chk("text_exec('M a') 六轴中位", "centered" in out and
        c.transport.pos_raw == [2048] * 6)

    out = c.text_exec("zr")
    chk("text_exec('zr') 复位 Home 零点", "zr" in out and c.transport.zero_raw == [0] * 6)

    out = c.text_exec("cx")
    chk("抢 ETH 引脚命令被拦下并**说明原因**（不是静默失败）",
        "已停用" in out and "ETH" in out, repr(out[:50]))

    out = c.text_exec("c")
    chk("单独 'c' 被安全闸门拒绝（会卡死主循环）", "[GATE]" in out, repr(out[:50]))

    c.transport.set_estop(True)
    out = c.text_exec("s")
    chk("急停中 s 放行", "J1" in out)
    out = c.text_exec("zi")
    chk("急停中 zi 被拒（只放行 s/h）", "[GATE]" in out and "E-Stop" in out)
    c.transport.set_estop(False)

    out = c.text_exec("nosuchcmd")
    chk("未知命令回 Unknown command", "Unknown" in out)

    # 分页：把假臂的输出撑到 > 2048 B，验证 flags.bit0 与 TEXT_FETCH 翻页
    big = c.transport
    big._text_buf = ("X" * 70 + "\n") * 40          # 2840 B，超过 2048 缓冲
    big._text_trunc = True
    big._text_gate = False
    total = len(big._text_buf)
    acc, off, guard = [], 0, 0
    while off < total and guard < 80:
        guard += 1
        r2 = c.request(0x0C, payload_text_fetch(off))
        if not r2.ok:
            break
        p = decode_text_out(r2.payload)
        if not p.text:
            break
        if guard == 1:
            chk("长输出的第一页带截断标记（flags bit0）", p.truncated)
        acc.append(p.text)
        off += len(p.text)
    joined = "".join(acc)
    chk("TEXT_FETCH 分页拼回完整长输出", joined == big._text_buf,
        "total=%d got=%d pages=%d" % (total, len(joined), len(acc)))

    # 长度守卫
    try:
        payload_text_exec("x" * (TEXT_EXEC_MAX + 1))
        chk("超长命令必须抛错（不许静默执行半条）", False)
    except ValueError:
        chk("超长命令必须抛错（不许静默执行半条）", True)
    try:
        c.text_exec("x" * (TEXT_EXEC_MAX + 5))
        chk("client.text_exec 同样在本地就拦住超长", False)
    except ValueError:
        chk("client.text_exec 同样在本地就拦住超长", True)

    # magic 校验（直接发一帧错的）
    bad = c.request(0x0B, b"\x00\x00\x00\x00s\x00")
    chk("TEXT_EXEC magic 不符 -> err=8(MAGIC)", (not bad.ok) and bad.err == 8, bad.err_name)

    c.transport.close()
    print("\n================ 结果 ================")
    print("PASS=%d  FAIL=%d" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
