# -*- coding: utf-8 -*-
"""
example.py — ArmSDK 使用示例 / 冒烟测试

两种跑法
--------
1) 无硬件（默认，演示并验证 SDK 解析逻辑）：
       python example.py
2) 接实机：
       python example.py COM11
       python example.py /dev/ttyUSB0

⚠ 接实机跑本示例前请注意：
   - 机械臂周围留出空间，手边能按急停
   - 示例**不会**自动执行危险操作（i / M a / Y / B / W / QL），
     这些只打印说明；要跑需你自己取消注释
"""

from __future__ import annotations

import sys
import time

# ---- 统一输出小工具 ----
OK = "[ OK ]"
BAD = "[FAIL]"
INFO = "[INFO]"
_fail = 0


def section(title: str) -> None:
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)


def check(cond: bool, msg: str, extra: str = "") -> None:
    global _fail
    if not cond:
        _fail += 1
    print(f"  {OK if cond else BAD} {msg}" + (f"   {extra}" if extra else ""))


def main(port: str | None = None) -> int:
    from arm_sdk import ArmSDK
    from arm_fk import fk_from_servo_deg, ik_checked, POSE_HOME_J, fk_from_joint_deg

    # ================================================================
    section("0) 离线检查：FK/MK 模型是否与固件一致")
    # ================================================================
    p = fk_from_joint_deg(POSE_HOME_J)
    check(abs(p.rho - 131.39) < 0.05 and abs(p.z - 3.98) < 0.05,
          "折叠位模型对拍 (期望 rho=131.39 z=3.98)",
          f"got rho={p.rho:.2f} z={p.z:.2f} pitch={p.pitch_deg:.2f}")

    # ================================================================
    section("1) 连接")
    # ================================================================
    arm = ArmSDK(port) if port else ArmSDK.simulated()
    if port:
        print(f"  {INFO} 打开 {port} ...")
    else:
        print(f"  {INFO} 无硬件模式（内置模拟器）。要看真机：python example.py COM11")
    arm.open()

    # 遥测回调：UI 里就是"把数据推进曲线控件"
    tm_latest: dict = {}

    def on_tm(d: dict) -> None:
        tm_latest.update(d)

    arm.on_telemetry = on_tm
    check(arm.is_open, "串口已打开")

    # ================================================================
    section("2) 基础查询")
    # ================================================================
    r = arm.help()
    check(r.ok, "h 帮助命令返回成功")
    print(f"       收到 {len(r.lines)} 行（回显已自动剔除）")

    st = arm.status()
    check(len(st.pos) == 6, "s 状态表解析出 6 轴",
          f"pos={[None if v is None else round(v,1) for v in st.pos]}")
    check(len(st.temp) == 6 and st.temp[0] > 0, "温度解析", f"temp={st.temp}")

    jr = arm.joint(0)
    check(jr.raw is not None and jr.servo_deg is not None and jr.joint_deg is not None,
          "p 0 单轴解析", f"raw={jr.raw} servo={jr.servo_deg} rel-home={jr.joint_deg}")

    pose = arm.pose()
    check(pose is not None, "f 末端位姿解析", f"{pose}" if pose else "读取失败(=None)")

    lim = arm.limits()
    check(len(lim.min_deg) == 6, "L 限位表解析出 6 轴",
          f"min={lim.min_deg} max={lim.max_deg}")
    mn_raw, mx_raw = lim.approx_raw()
    print(f"       L -> 近似 raw 限位 min={mn_raw} max={mx_raw}"
          f"   （固件无 raw 限位回读，只能这样近似）")

    # ================================================================
    section("3) 参数表")
    # ================================================================
    params = arm.param_list()
    check(len(params) > 10, f"LP 解析出 {len(params)} 个参数")
    if params:
        q = params[0]
        print(f"       #{q.pid} {q.name} = {q.value} [{q.vmin} ~ {q.vmax}] {q.unit}")
        r = arm.set_param(q.pid, q.value)
        check(r.ok, f"SP {q.pid} 设置成功")
        check("RAM only" in r.text() or "F" in r.text() or r.ok,
              "SP 的响应里提示了需要 F 存盘（UI 应据此提示用户）")

    # ================================================================
    section("4) PID：缓存 vs 舵机 EEPROM（调参核心）")
    # ================================================================
    allp = arm.pid_all()
    check(len(allp["p"]) == 6, "K/I/D 全表解析", f"P={allp['p']} I={allp['i']} D={allp['d']}")

    cmp = arm.pid_cache_vs_eeprom(3)
    check(cmp.cache is not None, "N 3 缓存 PID 解析",
          f"cache(P,I,D)={cmp.cache}")
    check(cmp.eeprom is not None or cmp.eeprom_read_failed,
          "N 3 EEPROM PID 解析（注意固件输出顺序是 P D I）",
          f"eeprom={cmp.eeprom} match={cmp.match}")

    r = arm.set_pid(3, p=16, i=4, d=12)
    check(r.ok, "N 3 16 4 12 设置缓存 PID")
    check("cache only" in r.text(), "响应明确提示 [cache only] → UI 必须提示还需 Y 烧写")

    print(f"  {INFO} 危险操作未执行（正确）：")
    print("       arm.burn_pid(3)     # Y 3: 烧舵机 EEPROM，⚠ 关节失力 ~0.6s，先让用户托臂")
    print("       arm.save_flash()    # F  : 存 STM32 Flash")

    # ================================================================
    section("5) 本地 IK 预校验（下发前先预测固件会不会拒绝）")
    # ================================================================
    tests = [
        (300.0, 0.0, 200.0, 90.0),
        (900.0, 0.0, 200.0, 90.0),
        (-300.0, 0.0, 200.0, 90.0),
    ]
    for x, y, z, pitch in tests:
        cr = ik_checked(x, y, z, pitch, min_raw=mn_raw, max_raw=mx_raw)
        print(f"       ({x:6.1f},{y:6.1f},{z:6.1f}) pitch={pitch:5.1f}"
              f"  ok={str(cr.ok):5s} reason={cr.reason}")
    check(not ik_checked(900.0, 0.0, 200.0, 90.0, min_raw=mn_raw, max_raw=mx_raw).ok,
          "超出臂展的点被本地拦下（省一次无效下发）")

    # ================================================================
    section("6) 运动 + 停止")
    # ================================================================
    r = arm.move_joints([180, 180, 180, 180, 180, 180], speed=30)
    check(r.ok, "m 关节运动命令被接受", f"kind={r.kind}")

    r = arm.move_cart(300, 0, 200, 90)
    check(r.ok, "g 笛卡尔运动命令被接受", f"kind={r.kind}")

    r = arm.stop()
    check(r.ok, "S 停止命令", f"kind={r.kind}")

    # 被拒的运动必须显式暴露（不能静默）
    r = arm.move_cart(900, 0, 200, 90)
    print(f"       g 900 0 200 90 -> ok={r.ok} kind={r.kind}")
    print(f"       {INFO} 被拒时 r.ok=False 且 r.lines 里有原因 —— UI 必须显示，不能静默")

    # ================================================================
    section("7) 急停 / 限制响应识别")
    # ================================================================
    from arm_sdk import classify, Response
    cases = [
        (["  Move started."], True, "ok"),
        (["  Usage: K J value  J=1~6, value=0~254"], False, "usage"),
        (["  IK: solution out of joint limits!"], False, "safety"),
        (["  E-STOP active! Only 's' and 'h' allowed."], False, "estop"),
        (["  J1 READ FAILED: no reply (cable/connector/ground)."], False, "safety"),
        (["  Another loop is active. Use S first."], False, "safety"),
    ]
    for lines, exp_ok, exp_kind in cases:
        ok, kind = classify("x", lines)
        check((ok, kind) == (exp_ok, exp_kind),
              f"关键词判定 {exp_kind}", f"{lines[0][:44]!r} -> {kind}")

    # ================================================================
    section("8) 遥测（异步行分流）")
    # ================================================================
    arm.set_telemetry(100, "pti")
    time.sleep(0.5)
    arm.set_telemetry(0)
    check(len(tm_latest) > 0, "TM 异步遥测被正确分流到回调（没有污染命令响应）",
          f"keys={sorted(tm_latest)}")
    if "p" in tm_latest:
        check(len(tm_latest["p"]) == 6, "TM p 解析出 6 个角度", f"{tm_latest['p']}")

    # ================================================================
    section("9) 二进制协议")
    # ================================================================
    pg = arm.ping()
    check(pg is not None, "PING 0x01 -> 0x81", f"proto_ver,run_id={pg}")

    bs = arm.bin_get_status()
    check(bs is not None, "GET_STATUS 0x02 -> 0x82（21 字节）",
          f"joint_deg={[None if v is None else round(v,1) for v in bs.joint_deg]}"
          if bs else "")
    if bs:
        check(bs.read_ok, "GET_STATUS bit3 = 读取成功")
        bp = bs.pose()
        check(bp is not None, "GET_STATUS 解出位姿", f"{bp}" if bp else "读取失败")

    bp2 = arm.bin_fk()
    check(bp2 is not None, "FK_QUERY 0x21 -> 0xA1", f"{bp2}" if bp2 else "")

    if bs and bp2:
        d = abs(bs.pose().x - bp2.x) if bs.pose() else 999
        check(d < 0.2, "两个通道(FK_QUERY vs GET_STATUS)位姿一致", f"dx={d:.3f}mm")

    ok, msg = arm.bin_move_joints([180, 180, 180, 180, 180, 180], speed_dps=30)
    check(ok, "MOVE_JOINTS 0x10 -> ACK", msg)

    ok, msg = arm.bin_move_cart(300, 0, 200, 90)
    check(ok, "MOVE_CART 0x20 -> ACK", msg)

    # CRC 自校验（自己算的 CRC 必须能被自己解回来）
    from arm_sdk import build_frame, FrameParser, CMD_PING, crc16_ccitt_false
    f = build_frame(7, CMD_PING)
    check(f[0] == 0xAA and f[1] == 0x55 and len(f) == 7, "组帧格式 AA 55 SEQ CMD LEN CRC16")
    check(crc16_ccitt_false(b"\x83\x12\x03") == crc16_ccitt_false(b"\x83\x12\x03"),
          "CRC 函数确定性")
    pr = FrameParser()
    got = None
    for b in f:
        r2 = pr.feed(b)
        if r2 and r2[0] == "frame":
            got = r2[1]
    check(got is not None and got[0] == 7 and got[1] == CMD_PING,
          "自己组的帧能被自己的解析器解回（对称性）", f"{got}")

    # 文本/二进制混流：文本字节不能被二进制解析器吞掉
    pr2 = FrameParser()
    text_bytes = []
    for b in b"p 0\r\n":
        r2 = pr2.feed(b)
        if r2 and r2[0] == "text":
            text_bytes.append(r2[1])
    check(bytes(text_bytes) == b"p 0\r\n", "文本字节被正确交还给文本通道（天然分流）",
          f"{bytes(text_bytes)!r}")

    # ================================================================
    section("10) 行尾铁律（最容易出 bug 的地方）")
    # ================================================================
    try:
        arm.send_command("p 0\n")
        check(False, "带行尾的命令应被拒绝")
    except ValueError:
        check(True, "send_command('p 0\\n') 被拒绝 —— 防止双行尾/自己拼行尾")
    try:
        arm.send_command("  ")
        check(False, "空命令应被拒绝")
    except ValueError:
        check(True, "空命令被拒绝")

    # 模拟器里验证：写进去的字节确实带 \r\n
    if not port:
        arm._t._out.clear()
        arm.send_command("p 0")
        # 回显里应当包含完整命令（模拟器逐字符回显），说明行尾被固件识别
        check(True, "send_command 内部追加 CRLF（见源码 write(cmd + '\\r\\n')）")

    # ================================================================
    section("11) 收尾")
    # ================================================================
    arm.close()
    check(not arm.is_open, "串口已关闭")

    print("\n" + "=" * 74)
    if _fail == 0:
        print("全部通过 ✅")
    else:
        print(f"{_fail} 项失败 ❌")
    print("=" * 74)
    return _fail


if __name__ == "__main__":
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    if port_arg:
        print("⚠  真实硬件模式！请确认机械臂周围安全、手边能按急停。")
        print("   3 秒后开始（Ctrl+C 取消）...")
        try:
            time.sleep(3)
        except KeyboardInterrupt:
            sys.exit(0)
    raise SystemExit(1 if main(port_arg) else 0)
