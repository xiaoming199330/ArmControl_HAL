# -*- coding: utf-8 -*-
"""
cli.py — `armctl` 命令行（Linux / Windows 通用，纯标准库）

设计原则
--------
* **默认走网口**（p50 1.61 ms），`--uri` 一行切换串口/模拟器。
* 点动**必须**用 `--delta`：绝对角会经 `clamp_all_angles()` 静默钳制，
  "我发了 320 臂停在 300" 就是这么来的。
* 每个会动的子命令都先读一次状态，把"当前真实姿态"打出来再动。
* `--dry-run` 是**最底层出口**的安全开关（一个字节都不发）。

例子
----
    armctl info
    armctl selftest
    armctl --uri net://192.168.7.120:5000 ping
    armctl --uri net://192.168.7.120:5000 status --json
    armctl --uri net://192.168.7.120:5000 move --delta 0,+5,0,0,0,0 --v 10
    armctl --uri net://192.168.7.120:5000 stream --pose "180,180,180,180,180,180" --hz 50 --sec 2
    armctl teleop --leader net://192.168.7.121:5000 --follower net://192.168.7.120:5000 --sec 30
    armctl record --leader ... --follower ... --out demo.json --sec 60
    armctl traj info demo.json
    armctl play demo.json --speed 0.5
    armctl relay --leader-port /dev/ttyUSB0 --follower-port /dev/ttyUSB1 --hz 30
    armctl link --port /dev/ttyUSB0 --set net --save
    armctl mock --port 5000 --verbose     # 无真机：起一台假臂给上位机联调
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import List, Optional

from . import __version__

DEFAULT_URI = os.environ.get("ARM_URI", "net://192.168.7.120:5000")


# ============================================================================
# 小工具
# ============================================================================

def _die(msg: str, code: int = 2):
    print("armctl: " + msg, file=sys.stderr)
    raise SystemExit(code)


def _parse_six(text: str, what: str = "6 个值") -> List[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 6:
        _die("%s 需要 %s，收到 %d 个：%r" % (what, "6 个值", len(parts), text))
    try:
        return [float(p) for p in parts]
    except ValueError:
        _die("无法解析数字：%r" % text)
        return []


def _fmt_q(q, nd: int = 1) -> str:
    if q is None:
        return "[读失败]"
    return "[" + ", ".join("--" if v is None else ("%.*f" % (nd, v)) for v in q) + "]"


def _open(args, uri: Optional[str] = None, timeout: Optional[float] = None):
    from .client import open_arm
    u = uri or getattr(args, "uri", None) or DEFAULT_URI
    return open_arm(u, timeout=timeout or getattr(args, "timeout", 0.3),
                    dry_run=getattr(args, "dry_run", False))


def _prelude(arm) -> None:
    """动之前先报一次当前姿态（安全习惯）。"""
    try:
        st = arm.get_status()
    except Exception as e:                                    # noqa: BLE001
        print("  ⚠ 读状态失败：%s" % e)
        return
    print("  当前舵机度 %s  flags(estop=%s traj=%s corr=%s read_ok=%s)"
          % (_fmt_q(st.as_servo_deg()), st.estop, st.traj, st.corr, st.read_ok))
    if st.failed_joints():
        print("  ⚠ 读失败的轴：J%s" % st.failed_joints())
    if st.estop:
        print("  🛑 E-Stop 激活 —— 现在动不了")


# ============================================================================
# 子命令
# ============================================================================

def cmd_info(args) -> int:
    from .protocol import (AP_PARAM_COUNT, CMD_NAME, MAX_PACKET, PARAM_TABLE)
    print("arm_sdk %s" % __version__)
    print("  纯标准库可跑；串口需 pyserial，.npz 需 numpy")
    print("  三条链路：")
    print("    net://<ip>:5000   UDP，一个数据报=一个逻辑包（无帧头/无分包）  实测 p50 1.61ms  ★首选")
    print("    serial:///dev/... USART6 115200，AA 55 帧 + 文本控制台共口     实测 p50 4.78ms")
    print("    can               0x100+node，8B 分包需重组                    实测 p50 2.72ms")
    print("    sim://            内存假臂（离线开发）")
    print("                      ⚠ sim 状态只活在**当前进程**里：每条 armctl 都是新臂，")
    print("                        想连续操作请用 Python API（open_arm('sim://')）或一条命令做到底")
    print("  二进制命令 %d 条；运行时参数 %d 项；最大逻辑包 %d B"
          % (len(CMD_NAME), AP_PARAM_COUNT, MAX_PACKET))
    cfg = _config_path()
    if cfg:
        try:
            with open(cfg, encoding="utf-8") as f:
                d = json.load(f)["arm_config"]
            print("  配置镜像 %s" % cfg)
            print("    文本命令 %d 条 / 网口命令 %d 条 / 章节 %d"
                  % (len(d.get("commands", [])),
                     len(d.get("network_protocol", {}).get("commands", [])), len(d)))
        except Exception as e:                                    # noqa: BLE001
            print("    (读配置失败：%s)" % e)
    else:
        print("  （未找到 arm_config.json）")
    print("  参数表节选：%s"
          % ", ".join("%d=%s" % (k, v[0]) for k, v in list(PARAM_TABLE.items())[:6]))
    return 0


def _config_path() -> Optional[str]:
    """找 arm_config.json：包内（pip 安装后）→ 仓库 config/ → 仓库 data/。"""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    for p in (os.path.join(here, "arm_config.json"),
              os.path.join(root, "config", "arm_config.json"),
              os.path.join(root, "data", "arm_config.json"),
              os.path.join(os.path.dirname(root), "config", "arm_config.json")):
        if os.path.exists(p):
            return p
    return None


def cmd_selftest(args) -> int:
    """跑全部离线自检（协议 / 遥操 / 示教 / 端到端 / 扩展命令）。

    合计 228 项（无 numpy 为 227，少的 1 项是 .npz 往返，会打印跳过）。
    """
    import subprocess
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rc = 0
    mods = [["-m", "arm_sdk.protocol"], ["-m", "arm_sdk.teleop"], ["-m", "arm_sdk.teach"]]
    n_test_file = 0
    for name in ("integration_test.py", "new_commands_test.py"):
        p = os.path.join(here, "tests", name)
        if os.path.exists(p):
            mods.append([p])
            n_test_file += 1
    if n_test_file == 0:
        print("⚠ 没找到 tests/ 目录（wheel 安装后属正常）—— 本次只跑模块自检；"
              "要跑全套请用源码树（`python3 -m pip install -e .` 或设 PYTHONPATH）")
    for m in mods:
        label = " ".join(m)
        print("\n>>> %s" % label)
        r = subprocess.run([sys.executable] + m, cwd=here)
        rc = rc or r.returncode
    print("\n%s" % ("✓ 全部自检通过" if rc == 0 else "✗ 有自检失败"))
    return rc


def cmd_preflight(args) -> int:
    """上板前的健康检查清单。"""
    print("=== 上板前置检查 ===")
    arm = _open(args)
    ok = True
    with arm:
        pi = arm.ping()
        if pi is None:
            print("  ✗ PING 无响应 —— 检查：①link 偏好(需 link net→save→RESET) ②目的端口 5000")
            print("    ③本构建是否含网口(LINK=net|both) ④网线/IP")
            ok = False
        else:
            print("  ✓ PING proto_ver=%d run_id=%d" % (pi.proto_ver, pi.run_id))
        try:
            st = arm.get_status()
            bad = st.failed_joints()
            print("  %s GET_STATUS 舵机度 %s" % ("✗" if bad else "✓", _fmt_q(st.as_servo_deg())))
            if bad:
                print("    ⚠ 读失败的轴 J%s —— 先排总线再运动" % bad)
                ok = False
            if st.estop:
                print("    🛑 E-Stop 激活")
                ok = False
            if not st.read_ok:
                print("    ⚠ read_ok=0 —— 位姿字段无效，不要当真值用")
                ok = False
        except Exception as e:                                     # noqa: BLE001
            print("  ✗ GET_STATUS 失败：%s" % e)
            ok = False
        try:
            pc = arm.param_count()
            print("  ✓ PARAM_COUNT = %d（现役 24）" % pc)
        except Exception as e:                                     # noqa: BLE001
            print("  ⚠ PARAM_COUNT 失败：%s" % e)
        fk = None
        try:
            fk = arm.fk_query()
            print("  ✓ FK_QUERY X=%.1f Y=%.1f Z=%.1f pitch=%.1f read_ok=%d"
                  % (fk.x, fk.y, fk.z, fk.pitch, fk.read_ok))
        except Exception as e:                                     # noqa: BLE001
            print("  ⚠ FK_QUERY 失败：%s" % e)
        print("  提示：串口侧还应跑 `h`(fw-tag) / `zi`(零点是否被污染) / `L`(运行时限位)")
    print("=== %s ===" % ("可以动" if ok else "先修上面的问题"))
    return 0 if ok else 1


def cmd_ping(args) -> int:
    arm = _open(args)
    with arm:
        t0 = time.monotonic()
        pi = arm.ping()
        dt = (time.monotonic() - t0) * 1000
        if pi is None:
            _die("PING 无响应（见 armctl preflight 的排查清单）", 1)
        print("✓ PING proto_ver=%d run_id=%d  %.1fms" % (pi.proto_ver, pi.run_id, dt))
        if args.repeat > 1:
            ts = []
            for _ in range(args.repeat - 1):
                t0 = time.monotonic()
                if arm.ping() is not None:
                    ts.append((time.monotonic() - t0) * 1000)
            if ts:
                ts.sort()
                print("  %d 次：p50=%.2fms min=%.2f max=%.2f"
                      % (len(ts), ts[len(ts) // 2], ts[0], ts[-1]))
    return 0


def cmd_status(args) -> int:
    arm = _open(args)
    with arm:
        n = 0
        while True:
            t0 = time.monotonic()
            st = arm.get_status()
            dt = (time.monotonic() - t0) * 1000
            if args.json:
                print(json.dumps({
                    "joints_servo_deg": st.joints, "x": st.x, "y": st.y, "z": st.z,
                    "pitch": st.pitch, "estop": st.estop, "traj": st.traj,
                    "corr": st.corr, "read_ok": st.read_ok,
                    "failed_joints_1based": st.failed_joints(),
                    "latency_ms": round(dt, 2)}, ensure_ascii=False))
            else:
                print("%s  X=%.1f Y=%.1f Z=%.1f pitch=%.1f  %.1fms%s"
                      % (_fmt_q(st.as_servo_deg()), st.x, st.y, st.z, st.pitch, dt,
                         "  🛑E-STOP" if st.estop else ""))
            n += 1
            if not args.watch or n >= args.watch:
                break
            time.sleep(args.interval)
    return 0


def cmd_fk(args) -> int:
    arm = _open(args)
    with arm:
        fk = arm.fk_query()
        print("X=%.1f mm  Y=%.1f mm  Z=%.1f mm  pitch=%.1f deg  read_ok=%d"
              % (fk.x, fk.y, fk.z, fk.pitch, fk.read_ok))
        print("舵机度 %s" % _fmt_q(fk.as_servo_deg()))
        if fk.read_ok != 1:
            print("⚠ read_ok=0 —— 位姿无效（有轴读失败）")
    return 0


def cmd_move(args) -> int:
    arm = _open(args)
    with arm:
        _prelude(arm)
        target = _parse_six(args.pose, "目标角")
        if args.delta:
            st = arm.get_status()
            cur = st.as_servo_deg()
            if cur is None:
                _die("要 --delta 但当前有轴读失败（J%s）—— 不能用未知起点做增量"
                     % st.failed_joints(), 1)
            target = [cur[j] + target[j] for j in range(6)]
            print("  增量模式：当前 %s → 目标 %s" % (_fmt_q(cur), _fmt_q(target)))
        else:
            print("  绝对模式：目标 %s（⚠ 超限会**静默钳制**，err=0 不代表到位）"
                  % _fmt_q(target))
        r = arm.move_joints(target, v=args.v, a=args.a)
        print("  %s" % r)
        if not r.ok:
            return 1
        if args.wait:
            time.sleep(0.3)
            st = arm.get_status()
            print("  到位检查（err=0 不代表动了，必须看角度）：%s" % _fmt_q(st.as_servo_deg()))
    return 0


def cmd_cart(args) -> int:
    arm = _open(args)
    with arm:
        _prelude(arm)
        print("  MOVE_CART X=%.1f Y=%.1f Z=%.1f pitch=%.1f ⚠ 会归零 J5/J6"
              % (args.x, args.y, args.z, args.pitch))
        r = arm.move_cart(args.x, args.y, args.z, args.pitch, v=args.v, a=args.a)
        print("  %s" % r)
        return 0 if r.ok else 1


def cmd_stop(args) -> int:
    arm = _open(args)
    with arm:
        r = arm.stop()
        print("%s（平滑减速，不是急停断电）" % r)
    return 0


def cmd_home(args) -> int:
    """全轴回中位（舵机度 180）——等价 `move "180,180,180,180,180,180"`，省得敲六个数。"""
    arm = _open(args)
    with arm:
        _prelude(arm)
        home = [180.0] * 6
        print("  回中位：目标 %s（舵机度域，180=中位；给所有轴留双向行程，第一次动先敲它）"
              % _fmt_q(home))
        r = arm.move_joints(home, v=args.v, a=args.a)
        print("  %s" % r)
        if not r.ok:
            return 1
        if args.wait:
            time.sleep(0.3)
            st = arm.get_status()
            print("  到位检查（err=0 不代表动了，必须看角度）：%s" % _fmt_q(st.as_servo_deg()))
    return 0


def cmd_pose(args) -> int:
    """`P <n>` 预定义位姿回放（TEXT_EXEC 透传固件 cmdPose，走 cmdGotoSafePlan 轨迹）。

    固件命名陷阱：**0=HOME 才是真折叠收纳位**（RC 手柄 CIR 键同款），
    1=FOLD 反而是备用收拢，平时不用。触发后 RC 会解武装（臂静止 2s 自动重武装）。
    """
    names = {0: "HOME=折叠收纳位(推荐)", 1: "FOLD=备用收拢",
             2: "READY=就绪位", 3: "WAVE1=演示", 4: "WAVE2=演示"}
    n = getattr(args, "n", 0)
    arm = _open(args)
    with arm:
        _prelude(arm)
        print("  位姿 %d = %s" % (n, names.get(n, "?")))
        try:
            out = arm.text_exec("P %d" % n)
        except Exception as e:                                # noqa: BLE001
            print("  ✗ 固件拒绝：%s" % e)
            return 1
        print(out)
        if args.wait:
            time.sleep(0.5)
            st = arm.get_status()
            print("  当前角度（err=0 不代表动了，必须看角度）：%s"
                  % _fmt_q(st.as_servo_deg()))
    return 0


def cmd_ik(args) -> int:
    """IK_QUERY：只解算不运动。给一个 XYZ 目标，看六轴解、限位与 FK 回代是否通过。"""
    arm = _open(args)
    with arm:
        try:
            sol = arm.ik_query(args.x, args.y, args.z, pitch=args.pitch)
        except Exception as e:                                # noqa: BLE001
            print("  ✗ 解算失败：%s" % e)
            return 1
        print("  目标 (%.1f, %.1f, %.1f) pitch=%.0f ⇒ 关节解（关节逻辑度）："
              % (args.x, args.y, args.z, args.pitch))
        print("    %s" % _fmt_q(sol))
        print("  ⓘ 只解算不运动；要把臂送到这里，把解代入 move（注意**两套角度域**，"
              "move 要舵机度）。")
    return 0


def cmd_detail(args) -> int:
    """GET_JOINT_DETAIL：六轴遥测（角度/速度/负载/电压/温度）。

    ⚠ 总线逐字段瞬时丢读（Temp≈20%），看趋势别看单拍；默认 --avg 5 取中位数。
    """
    arm = _open(args)
    with arm:
        n = max(1, args.avg)
        rounds = [arm.get_joint_detail(0xFF) for _ in range(n)]
        byj = {}
        for rs in rounds:
            for d in rs:
                byj.setdefault(d.joint, []).append(d)

        def med(pick):
            vv = sorted(v for v in (pick(d) for d in oks) if v is not None and v >= 0)
            return vv[len(vv) // 2] if vv else None

        def fmt(v, f="%.1f"):
            return (f % v) if v is not None else "—"

        print("  %-4s %10s %8s %8s %8s %8s" % ("轴", "角度deg", "速度", "负载", "电压", "温度C"))
        for j in sorted(byj):
            oks = [d for d in byj[j] if d.ok]
            ang = med(lambda d: d.pos_raw)
            if ang is not None:
                ang = ang / 4095.0 * 360.0          # STS3215 raw 0~4095 → 0~360°
            print("  J%-3s %10s %8s %8s %8s %8s"
                  % (j, fmt(ang), fmt(med(lambda d: d.speed), "%.0f"),
                     fmt(med(lambda d: d.load), "%.0f"),
                     fmt(med(lambda d: d.voltage), "%.2f"),
                     fmt(med(lambda d: None if d.temp_lost else d.temp), "%.0f")))
        print("  （%d 拍取中位数抗瞬时丢读；— = 该字段全拍丢失，不是 0 值）" % n)
    return 0


def cmd_flags(args) -> int:
    """GET_FLAGS：运行状态位图（不碰舵机总线，适合高频轮询做 UI 状态灯）。"""
    arm = _open(args)
    with arm:
        f = arm.get_flags()
        print(f)
    return 0


def cmd_pid(args) -> int:
    """GET_PID：读单轴 PID（舵机 EEPROM 权威值）。from_eeprom=False=读失败，值不可信。"""
    arm = _open(args)
    with arm:
        p = arm.get_pid(args.joint)
        print("  J%d PID = P=%s I=%s D=%s  from_eeprom=%s%s"
              % (args.joint + 1, p.p, p.i, p.d, p.from_eeprom,
                 "" if p.from_eeprom else "（⚠ 读失败，值不可信）"))
    return 0


def cmd_gravity(args) -> int:
    """GET/SET_GRAVITY：重力前馈增益 K（deg/mm）。带参数=设，不带=读。"""
    arm = _open(args)
    with arm:
        if args.k is None:
            print("  当前重力前馈 K = %s deg/mm" % arm.get_gravity())
        else:
            r = arm.set_gravity(args.k)
            print("  %s  回读 = %s deg/mm" % (r, arm.get_gravity()))
            return 0 if r.ok else 1
    return 0


def cmd_corr(args) -> int:
    """GET/SET_CORRECTION：自适应修正（用前先 `corr` 看当前配置）。"""
    arm = _open(args)
    with arm:
        if args.action == "get":
            print("  %s" % arm.get_correction())
        else:
            r = arm.set_correction(enable=(args.action == "on"),
                                   threshold_deg=args.thr, max_rounds=args.rounds)
            print("  %s  回读 = %s" % (r, arm.get_correction()))
            return 0 if r.ok else 1
    return 0


def cmd_log(args) -> int:
    """LOG_SET_LEVEL：调固件 RunLog 级别（0=关 1=err 2=warn 3=info 4=debug）。"""
    arm = _open(args)
    with arm:
        r = arm.log_set_level(args.level)
        print("  %s" % r)
    return 0 if r.ok else 1


def cmd_teach(args) -> int:
    """板载示教文件管理：total/list/show/del/compact。

    录制/回放的高带宽路径走 record/play（PC 侧轨迹）；这里管的是**固件 Flash 里**
    那几份示教点（串口 `L1` 录的那批）。
    """
    arm = _open(args)
    with arm:
        if args.action == "total":
            print("  %s" % arm.teach_total())
        elif args.action == "list":
            files = arm.teach_list_all()
            if not files:
                print("  （板载无示教文件）")
            for f in files:
                print("  %s" % f)
        elif args.action == "show":
            samples = arm.teach_read(args.idx, 0)
            print("  文件 #%d 首页样本（raw）：")
            for s in samples:
                print("    %s" % s)
            print("  （n=0 或不足一页 = 读完；全部样本请用 teach_trace 走 PC 侧工具）")
        elif args.action == "del":
            r = arm.teach_delete(args.idx)
            print("  %s（空间要用 compact 回收）" % r)
            return 0 if r.ok else 1
        elif args.action == "compact":
            r = arm.teach_compact()
            print("  %s" % r)
            return 0 if r.ok else 1
    return 0


def cmd_jog(args) -> int:
    """MOVE_JOINT_ONE：单轴动，其余五轴保持当前位置——手动 jog 的精准版。

    默认相对当前角（增量）；--abs 则为绝对舵机度。⚠ 角度是**舵机度**域。
    固件先读全场快照、只改这一轴——任一轴丢读则回 ERR_READ(7)（拿不准就不动，比动错强）。
    """
    arm = _open(args)
    with arm:
        _prelude(arm)
        if args.abs:
            tgt = args.deg
            print("  J%d 绝对 → %.1f°（舵机度域）" % (args.joint + 1, tgt))
        else:
            st = arm.get_status()
            cur = (st.as_servo_deg() or [None] * 6)[args.joint]
            if cur is None:
                print("  ✗ J%d 当前角读失败（None），拒绝盲动——重试一次或改用 --abs"
                      % (args.joint + 1))
                return 1
            tgt = cur + args.deg
            print("  J%d 当前 %.1f° %+.1f° → 目标 %.1f°"
                  % (args.joint + 1, cur, args.deg, tgt))
        r = arm.move_joint_one(args.joint, tgt, v=args.v, a=args.a)
        print("  %s" % r)
        if not r.ok:
            return 1
        if args.wait:
            time.sleep(0.3)
            st = arm.get_status()
            print("  到位检查：%s" % _fmt_q(st.as_servo_deg()))
    return 0


def _confirm_danger(args, what: str) -> bool:
    """危险命令统一闸门：必须 --yes；否则打印后果并拒绝。"""
    if args.yes:
        return True
    print("  🛑 危险命令需要确认：%s" % what)
    print("     确认后果后重敲，加 --yes。")
    return False


def cmd_init(args) -> int:
    """SERVO_INIT：舵机初始化（重写该轴 PID/限位/中位并 RESET，同文本 `i`）。

    ⚠ Torque ON + 抱闸：臂会**当场绷紧**。标定流程专用，别当"重启"随手敲。
    """
    if not _confirm_danger(args, "重写舵机 PID/限位/中位并 RESET，Torque ON 臂会绷紧"):
        return 2
    arm = _open(args)
    with arm:
        r = arm.servo_init(args.joint)
        print("  %s" % r)
    return 0 if r.ok else 1


def cmd_busmode(args) -> int:
    """SET_BUS_MODE：0=正常 A:AABBB / 1=全A / 2=全B。纯调试用。"""
    arm = _open(args)
    with arm:
        r = arm.set_bus_mode(args.mode)
        print("  %s" % r)
    return 0 if r.ok else 1


def cmd_zero(args) -> int:
    """ZERO_RESET（⚠ 危险）：Home 零点整体复位回出厂中位（= 文本 `zr`）。

    量纲基准随之改变；--save 写 Flash 固化（= `zr save`，重启保持）。
    """
    if not _confirm_danger(args, "六轴 Home 零点全部复位回出厂中位，量纲基准改变"
                                + ("，且写 Flash 固化" if args.save else "")):
        return 2
    arm = _open(args)
    with arm:
        r = arm.zero_reset_danger(save=args.save)
        print("  %s" % r)
    return 0 if r.ok else 1


def cmd_reboot(args) -> int:
    """SYS_RESET（⚠ 危险）：软复位整板。先回 ACK、150ms 后真正复位。

    ⚠ 复位即丢力矩、臂下沉；按 Flash 链路偏好重建链路。
    """
    if not _confirm_danger(args, "整板软复位：力矩全掉、臂下沉，链路按 Flash 偏好重建"):
        return 2
    arm = _open(args)
    with arm:
        r = arm.sys_reset_danger()
        print("  %s（ACK 已回；150ms 后板子复位，等它重新上线再 ping）" % r)
    return 0 if r.ok else 1


def cmd_center(args) -> int:
    """CENTER_CALIB（⚠⚠ 最危险）：六轴推到 raw 2048 中位，等价文本 `M a`。

    **改变量纲基准、不可逆**——就是历史上"松力矩整臂塌下"的那条。标定流程专用。
    """
    if not _confirm_danger(args, "六轴写 raw 2048 居中（= `M a`）：改变量纲基准、"
                                 "不可逆，臂会大动。仅标定流程使用"):
        return 2
    arm = _open(args)
    with arm:
        r = arm.center_calib_danger()
        print("  %s" % r)
        print("  ⚠ 立刻重跑标定 Phase1+2 并 `F` 落盘，否则量纲是坏的。")
    return 0 if r.ok else 1


def cmd_limits(args) -> int:
    """GET_LIMITS(0x05)：6 轴校准限位与 Home 零点（**Flash 校准记录**）。

    ⚠ 报错误码 7 时不是线缆问题 —— 是 Flash 参数区从未写过（未标定/标定后没 `F` 落盘）。
    ⚠ 这里读到的**不是**当前运行时限位（固件 JOINT_MIN/MAX，协议暂未开放读取）。
    """
    arm = _open(args)
    with arm:
        from .client import ArmCommandError      # 局部导入：cli 不在包导入链上引 client
        try:
            lim = arm.get_limits()
        except ArmCommandError as e:
            if e.reply.err == 7:
                _die("GET_LIMITS 报 7（READ）：Flash 参数区从未写过（未标定，或标定后没 `F` 落盘）"
                     "—— **不是线缆问题**；控制台/运动不受影响，需要精度时先校准+落盘。", 1)
            raise
        raw2deg = lambda raw: raw / 4096.0 * 360.0   # STS3215 raw 0~4095 → 0~360°，2048=180°
        print("  J轴   min_raw  max_raw  zero_raw  min_deg  max_deg  Home已校准")
        for j in range(6):
            print("  J%-3d  %6d    %6d    %6d    %6.1f    %6.1f    %s"
                  % (j + 1, lim.min_raw[j], lim.max_raw[j], lim.zero_raw[j],
                     raw2deg(lim.min_raw[j]), raw2deg(lim.max_raw[j]),
                     "✓" if lim.calibrated[j]
                     else "✗（回落 180°/raw2048，量纲基准脆，别做精度动作）"))
        print("  ⚠ 以上是校准记录；运行时钳制以固件 JOINT_MIN/MAX 为准（钳制时会打黄提示）。")
    return 0


def cmd_torque(args) -> int:
    arm = _open(args)
    with arm:
        j = 0xFF if args.joint is None else args.joint
        r = arm.set_torque(j, not args.off)
        print("%s" % r)
        return 0 if r.ok else 1


def cmd_stream(args) -> int:
    """定住某个逻辑角流一段时间（验证流模式 + 测节拍）。"""
    from .protocol import JointCalib
    arm = _open(args)
    with arm:
        st = arm.get_status()
        sv = st.as_servo_deg()
        if sv is None:
            _die("有轴读失败（J%s），先排总线" % st.failed_joints(), 1)
        calib = JointCalib()
        cur = calib.to_logical(sv)
        tgt = cur if args.pose is None else _parse_six(args.pose, "--pose（逻辑角）")
        print("  当前逻辑角 %s" % _fmt_q(cur))
        print("  目标逻辑角 %s @ %.0f Hz for %.1fs（限速 %.0f°/s）"
              % (_fmt_q(tgt), args.hz, args.sec, args.slew))
        from .teleop import TeleopConfig
        from .teach import PlaybackSession
        from .teleop import TeleopSession
        # 用一条 2 点的"原地"轨迹把流打开，并走到目标（限速）
        from .teach import Trajectory
        tr = Trajectory(name="cli-stream", hz=args.hz)
        ramp = max(0.2, args.sec * 0.3)
        steps = max(2, int(ramp * args.hz))
        for i in range(steps + 1):
            a = i / steps
            tr.append(ramp * a, [cur[j] + (tgt[j] - cur[j]) * a for j in range(6)])
        hold = max(0.0, args.sec - ramp)
        n_hold = int(hold * args.hz)
        for i in range(n_hold):
            tr.append(ramp + i / args.hz, list(tgt))
        pb = PlaybackSession(arm, cfg=TeleopConfig(hz=args.hz), estop_check_s=0.0)
        stp = pb.play(tr, speed=1.0)
        print("  %s" % stp.summary())
    return 0


def cmd_params(args) -> int:
    arm = _open(args)
    with arm:
        if args.action == "count":
            print(arm.param_count())
        elif args.action == "list":
            from .protocol import PARAM_TABLE
            try:
                n = arm.param_count()
            except Exception:                                      # noqa: BLE001
                n = -1
            print("现役 %d 项：" % n)
            for pid in sorted(PARAM_TABLE):
                v = arm.param_read(pid)
                nm, unit, lo, hi, dv = PARAM_TABLE[pid]
                print("  %2d %-22s %10s %s  [%s~%s] 默认 %s"
                      % (pid, nm, ("%.3f" % v) if v is not None else "读失败",
                         unit, lo, hi, dv))
        elif args.action == "get":
            print(arm.param_read(args.id))
        elif args.action == "set":
            r = arm.param_write(args.id, args.value)
            back = arm.param_read(args.id)
            print("%s  → 回读 = %s（⚠ 超范围会静默钳制，err=0 不代表写成功）" % (r, back))
            return 0 if r.ok else 1
        elif args.action == "save":
            r = arm.param_save()
            print("%s（= 文本 `F`：当前参数落盘 Flash，重启保持）" % r)
            return 0 if r.ok else 1
    return 0


def cmd_teleop(args) -> int:
    from .teleop import (TeleopAbort, TeleopConfig, TeleopLinkLost, TeleopSession)
    lead = _open(args, args.leader)
    foll = _open(args, args.follower)
    cfg = TeleopConfig(hz=args.hz, slew_deg_per_s=args.slew, gain=args.gain,
                       sync=args.sync, wait_ack=False, verbose=True)
    if args.soft_limit:
        lim = _parse_six(args.soft_limit, "--soft-limit")     # 统一 ±值
        cfg.soft_limits_deg = [(-abs(v), abs(v)) for v in lim]
    print("主从遥操：leader=%s → follower=%s  %.0f Hz / 限速 %.0f°/s / sync=%s"
          % (args.leader, args.follower, args.hz, args.slew, args.sync))
    with lead, foll:
        tp = TeleopSession(lead, foll, cfg=cfg)
        try:
            tp.start()
            tp.run(duration=args.sec)
        except TeleopLinkLost as e:
            print("✗ %s" % e)
            return 3
        except TeleopAbort as e:
            print("🛑 %s" % e)
            return 4
        except KeyboardInterrupt:
            tp.stop()
        print(tp.stats.summary())
    return 0


def cmd_record(args) -> int:
    from .teleop import TeleopAbort, TeleopConfig, TeleopLinkLost, TeleopTeachSession
    lead = _open(args, args.leader)
    foll = _open(args, args.follower)
    foll_calib = None
    cfg = TeleopConfig(hz=args.hz, slew_deg_per_s=args.slew, sync=args.sync, verbose=True)
    print("遥操示教：录制到 %s  %.0f Hz for %.1fs（PC 侧无 80s 上限）"
          % (args.out, args.hz, args.sec))
    with lead, foll:
        ts = TeleopTeachSession(lead, foll, out=args.out, name=os.path.basename(args.out),
                                hz=args.hz, cfg=cfg, foll_calib=foll_calib)
        try:
            traj = ts.record(duration=args.sec)
        except TeleopLinkLost as e:
            print("✗ %s" % e)
            return 3
        except TeleopAbort as e:
            print("🛑 %s" % e)
            return 4
        except KeyboardInterrupt:
            ts.stop()
            traj = ts.last_traj or ts.rec.trajectory()
            traj.save(args.out)
    v = traj.validate()
    print("✓ %s" % traj.summary())
    if not v["ok"]:
        print("  ⚠ 回放前建议先修：%s" % v["warnings"])
    print("  已存 %s" % args.out)
    return 0


def cmd_play(args) -> int:
    from .protocol import JointCalib
    from .teach import PlaybackSession, Trajectory
    from .teleop import TeleopConfig
    traj = Trajectory.load(args.file)
    v = traj.validate()
    print("轨迹 %s" % traj.summary())
    if not v["ok"]:
        print("  ⚠ %s" % v["warnings"])
        if not args.force:
            _die("有警告，确认无误请加 --force", 1)
    if args.convert:
        print("  已导出 %s" % traj.save(args.convert))
        if args.no_play:
            return 0
    arm = _open(args)
    with arm:
        _prelude(arm)
        pb = PlaybackSession(arm, cfg=TeleopConfig(hz=args.hz), estop_check_s=0.3,
                             dry_run=args.dry_run, approach=not args.no_approach,
                             approach_speed=args.approach_speed,
                             on_event=lambda m: print("  " + m))
        print("  回放 %.0f Hz speed=%.2f loop=%d（单帧步长硬限 %.0f°）"
              % (args.hz, args.speed, args.loop, 25 * 0.8))
        st = pb.play(traj, speed=args.speed, loop=args.loop)
        print("  %s" % st.summary())
        if st.ack_limit:
            print("  ⚠ 有 %d 帧被固件拒收(err=4)。常见原因：起点未接近（--no-approach 关掉时"
                  "接近段失败了）/ 轨迹里单帧跳变 >25° / 从臂 sf 0 关过跟随"
                  % st.ack_limit)
    return 0


def cmd_traj(args) -> int:
    from .teach import Trajectory
    if args.action == "info":
        t = Trajectory.load(args.file)
        v = t.validate()
        print("name=%s  points=%d  duration=%.3fs  rate=%.2fHz  units=%s  source=%s"
              % (t.name, v["points"], v["duration_s"], v["rate_hz"], t.units, t.source))
        print("每轴单帧最大跳变 %s（>25° 会被固件整帧拒收）" % v["max_step_deg"])
        print("warnings: %s" % (v["warnings"] or "无"))
        print("首帧 %s\n末帧 %s" % (_fmt_q(t.start_pose()), _fmt_q(t.end_pose())))
    elif args.action == "convert":
        t = Trajectory.load(args.file)
        out = t.save(args.out)
        print("已导出 %s（%d 点）" % (out, len(t)))
    elif args.action == "resample":
        t = Trajectory.load(args.file).resample(args.hz)
        out = t.save(args.out)
        print("已重采样到 %.1f Hz → %s（%d 点）" % (args.hz, out, len(t)))
    elif args.action == "smooth":
        t = Trajectory.load(args.file).smooth(args.window)
        out = t.save(args.out)
        print("已平滑 window=%d → %s（%d 点）" % (args.window, out, len(t)))
    return 0


def cmd_console(args) -> int:
    """发一条（或多条）串口文本命令。"""
    from .console import ArmSDK, SerialTransport
    t = SerialTransport(args.port, args.baud)
    with ArmSDK(t, response_timeout=args.timeout) as c:
        for line in args.cmd:
            r = c.send_command(line, timeout=args.timeout)
            print(">>> %s\n%s" % (line, r.text() or "(无输出)"))
            cl = r.clamps()
            if cl:
                print("  ⚠ 被限位钳制：%s" % cl)
    return 0


def cmd_text(args) -> int:
    """网口下发**任意一条**控制台命令（TEXT_EXEC 文本透传）。

    与 `console`（串口）的区别：
      · 走协议帧/网口，链路不通会**立刻**报错（串口那边只会静默等超时）；
      · 固件侧的安全闸门在起作用：11 条抢 ETH 引脚的命令会被拦下并回原因，
        单独一个 `c`（校准向导）会被拒（它是交互式的，从网口进会卡死主循环）；
      · 回显超过 2048 B 会**显式**提示截断，不会给你半截当完整。
    """
    arm = _open(args)
    with arm:
        rc = 0
        for line in args.cmd:
            try:
                out = arm.text_exec(line)
            except Exception as e:                            # noqa: BLE001
                print(">>> %s\n  ✗ %s" % (line, e))
                rc = 2
                continue
            print(">>> %s\n%s" % (line, out.rstrip() or "(无输出)"))
        return rc


def cmd_mock(args) -> int:
    """起一台**假臂 UDP 服务** —— 让没有真机的上位机也能联调。

    报文由 `SimArmTransport` 生成 ⇒ 与真机**逐字节一致**；但它是协议级假臂，
    没有物理仿真。要测上位机的超时/重试，用 --loss / --drop-first 砸它。
    """
    from .mock_server import MockArmServer
    fj = [int(x) for x in str(args.fail_joints).replace(",", " ").split() if x.strip()]
    srv = MockArmServer(host=args.host, port=args.port, latency_ms=args.latency_ms,
                        verbose=args.verbose, drop_first=args.drop_first,
                        loss=args.loss, estop=args.estop, fail_joints=fj,
                        unreachable=args.unreachable)
    if args.fw_tag:
        srv.sim.fw_tag = args.fw_tag
    if args.selfcheck:
        return srv.selfcheck()
    print("按 Ctrl-C 停止。")
    try:
        return srv.serve_forever()
    except KeyboardInterrupt:
        return 0


def cmd_http_bridge(args) -> int:
    """起 **HTTP ↔ UDP 边车** —— 让上位机用 HTTP 调机械臂。

    板子上没有 TCP（自研裸 MAC 栈只有 ARP/IP/ICMP/UDP），所以 HTTP 只能做在
    PC 侧。边车不改协议、不加语义：`POST /api/raw {"cmd":16,...}` 就是一条
    UDP 逻辑包，回包原样翻译成 JSON，错误码/丢读哨兵/静默钳制行为**完全一致**。

    每个响应都带 `X-Arm-Rtt-Ms` 头 = 边车到臂那一跳的毫秒数 ⇒ 可以把
    "链路延迟"和"HTTP 开销"分开算。详细对照见 docs/UDP_vs_HTTP_实测_2026-09-24.md。
    """
    from . import http_bridge
    return http_bridge.main([
        "--target", args.target, "--bind", args.bind,
        "--port", str(args.port), "--timeout", str(args.timeout),
    ] + (["--verbose"] if args.verbose else [])
      + (["--selftest"] if args.selfcheck else []))


def cmd_link(args) -> int:
    """链路开关（文本控制台，串口恒通）。"""
    from .console import ArmSDK, SerialTransport
    t = SerialTransport(args.port, args.baud)
    with ArmSDK(t, response_timeout=args.timeout) as c:
        if args.set:
            r = c.send_command("link %s" % args.set)
            print(r.text())
        if args.save:
            r = c.send_command("link save", timeout=10.0)
            print(r.text())
            print("⚠ 必须 RESET 才生效（运行期挪不动引脚 AF）")
        if not args.set and not args.save:
            print(c.send_command("link").text())
    return 0


def cmd_relay(args) -> int:
    """官网式 PC 中转（串口版）：主臂 sp 位置流 → 从臂 #P。"""
    from .console import ArmSDK, SerialTransport
    from .teleop import PcRelay
    lc = ArmSDK(SerialTransport(args.leader_port, args.baud), response_timeout=2.0)
    fc = ArmSDK(SerialTransport(args.follower_port, args.baud), response_timeout=2.0)
    with lc, fc:
        rel = PcRelay(lc, fc, hz=args.hz, slew_deg_per_s=args.slew,
                      on_event=lambda m: print("  " + m))
        rel.start()
        print("PC 中转运行中：主臂 %s(sp %d) → 从臂 %s  %.0fs；Ctrl-C 停"
              % (args.leader_port, args.hz, args.follower_port, args.sec))
        try:
            rel.run(duration=args.sec)
        except KeyboardInterrupt:
            rel.stop()
        print(rel.stats.summary())
        print("提示：`lk`（从臂）看 PC relay(#P) ok/reject 统计")
    return 0


# ============================================================================
# argparse
# ============================================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="armctl",
        description="六轴机械臂上位机 CLI（网口优先；Linux/Windows 通用）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("例子\n----\n", 1)[-1])
    p.add_argument("--version", action="version", version="arm_sdk " + __version__)
    p.add_argument("--uri", default=DEFAULT_URI,
                   help="默认链路 URI（默认 %s；也可用环境变量 ARM_URI）" % DEFAULT_URI)
    p.add_argument("--timeout", type=float, default=0.3, help="单命令超时秒（默认 0.3）")
    p.add_argument("--dry-run", action="store_true", help="只打印不发送（最底层安全开关）")

    sub = p.add_subparsers(dest="cmd", metavar="<命令>")

    sp = sub.add_parser("info", help="能力/链路/配置摘要（离线）")
    sp.set_defaults(func=cmd_info)
    sp = sub.add_parser("selftest", help="跑全部离线自检（协议/遥操/示教/端到端/扩展命令，228 项）")
    sp.set_defaults(func=cmd_selftest)
    sp = sub.add_parser("preflight", help="上板前置健康检查")
    sp.set_defaults(func=cmd_preflight)

    sp = sub.add_parser("ping", help="PING（兼测往返延迟）")
    sp.add_argument("--repeat", type=int, default=1)
    sp.set_defaults(func=cmd_ping)

    sp = sub.add_parser("status", help="GET_STATUS")
    sp.add_argument("--json", action="store_true")
    sp.add_argument("--watch", type=int, default=0, help="连续采 N 次")
    sp.add_argument("--interval", type=float, default=0.2)
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("fk", help="FK_QUERY 正解位姿")
    sp.set_defaults(func=cmd_fk)

    sp = sub.add_parser("home", help="全轴回中位（等价 move 180,...,180，第一次动先敲它）")
    sp.add_argument("--v", type=float, default=10.0, help="速度 deg/s（默认 10）")
    sp.add_argument("--a", type=float, default=0.0, help="加速度 deg/s²")
    sp.add_argument("--wait", action="store_true", help="发完读一次状态做到位检查")
    sp.set_defaults(func=cmd_home)

    sp = sub.add_parser("pose", help="P 0~4 预定义位姿回放（0=HOME=真折叠收纳位，CIR 键同款）")
    sp.add_argument("n", type=int, choices=range(0, 5), metavar="0~4",
                    help="0=HOME(折叠收纳) 1=FOLD(备用) 2=READY 3/4=WAVE")
    sp.add_argument("--wait", action="store_true", help="发完读一次状态看角度")
    sp.set_defaults(func=cmd_pose)

    sp = sub.add_parser("fold", help="★ 一键回折叠收纳位（= pose 0，P 0，CIR 键同款）")
    sp.add_argument("--wait", action="store_true", help="发完读一次状态看角度")
    sp.set_defaults(func=cmd_pose, n=0)

    sp = sub.add_parser("limits", help="GET_LIMITS：6 轴校准限位与 Home 零点（Flash 记录）")
    sp.set_defaults(func=cmd_limits)

    # ---- 观测类（全部安全，随便敲） ----
    sp = sub.add_parser("ik", help="IK_QUERY：解算 XYZ→六关节（**只解算不运动**）")
    sp.add_argument("x", type=float, help="目标 X mm（基座系）")
    sp.add_argument("y", type=float, help="目标 Y mm")
    sp.add_argument("z", type=float, help="目标 Z mm")
    sp.add_argument("pitch", type=float, nargs="?", default=0.0,
                    help="工具俯仰 deg（默认 0=朝上）")
    sp.set_defaults(func=cmd_ik)

    sp = sub.add_parser("detail", help="GET_JOINT_DETAIL：六轴遥测（角度/速度/负载/电压/温度）")
    sp.add_argument("--avg", type=int, default=5, help="取 N 拍中位数抗瞬时丢读（默认 5）")
    sp.set_defaults(func=cmd_detail)

    sp = sub.add_parser("flags", help="GET_FLAGS：运行状态位图（不碰舵机总线，可高频轮询）")
    sp.set_defaults(func=cmd_flags)

    sp = sub.add_parser("pid", help="GET_PID：读单轴 PID（舵机 EEPROM 权威值）")
    sp.add_argument("joint", type=int, choices=range(0, 6), metavar="0~5",
                    help="轴号 0=J1 … 5=J6")
    sp.set_defaults(func=cmd_pid)

    sp = sub.add_parser("gravity", help="GET/SET_GRAVITY：重力前馈 K（deg/mm）；带值=设，不带=读")
    sp.add_argument("k", type=float, nargs="?", default=None)
    sp.set_defaults(func=cmd_gravity)

    sp = sub.add_parser("corr", help="GET/SET_CORRECTION：自适应修正开关")
    sp.add_argument("action", choices=["get", "on", "off"])
    sp.add_argument("--thr", type=float, default=2.0, help="触发阈值 deg（默认 2.0）")
    sp.add_argument("--rounds", type=int, default=10, help="最多轮数（默认 10）")
    sp.set_defaults(func=cmd_corr)

    sp = sub.add_parser("log", help="LOG_SET_LEVEL：固件 RunLog 级别 0=关 1=err 2=warn 3=info 4=debug")
    sp.add_argument("level", type=int, choices=range(0, 5), metavar="0~4")
    sp.set_defaults(func=cmd_log)

    # ---- 运动类 ----
    sp = sub.add_parser("jog", help="MOVE_JOINT_ONE：单轴点动，其余五轴保持（默认相对当前角）")
    sp.add_argument("joint", type=int, choices=range(0, 6), metavar="0~5",
                    help="轴号 0=J1 … 5=J6")
    sp.add_argument("deg", type=float, help="增量 deg（默认）；--abs 时为绝对舵机度")
    sp.add_argument("--abs", dest="abs", action="store_true",
                    help="按绝对舵机度下发（默认是相对增量）")
    sp.add_argument("--v", type=float, default=10.0, help="速度 deg/s（默认 10）")
    sp.add_argument("--a", type=float, default=0.0, help="加速度 deg/s²")
    sp.add_argument("--wait", action="store_true", help="发完读一次状态做到位检查")
    sp.set_defaults(func=cmd_jog)

    # ---- 板载示教文件 ----
    sp = sub.add_parser("teach", help="板载示教文件（固件 Flash）：total/list/show/del/compact")
    sp.add_argument("action", choices=["total", "list", "show", "del", "compact"])
    sp.add_argument("idx", type=int, nargs="?", default=None,
                    help="文件序号（show/del 必填）")
    sp.set_defaults(func=cmd_teach)

    # ---- 系统/校准类（⚠ 一律 --yes 门槛） ----
    sp = sub.add_parser("init", help="SERVO_INIT（⚠ --yes）：重写轴 PID/限位/中位并 RESET，臂会绷紧")
    sp.add_argument("joint", type=int, nargs="?", default=0xFF,
                    help="轴号 0~5；省略=六轴全部")
    sp.add_argument("--yes", action="store_true", help="确认执行危险操作")
    sp.set_defaults(func=cmd_init)

    sp = sub.add_parser("busmode", help="SET_BUS_MODE：0=正常 A:AABBB / 1=全A / 2=全B（纯调试）")
    sp.add_argument("mode", type=int, choices=[0, 1, 2])
    sp.set_defaults(func=cmd_busmode)

    sp = sub.add_parser("zero", help="ZERO_RESET（⚠ --yes）：Home 零点复位回出厂中位（= zr）")
    sp.add_argument("action", choices=["reset"])
    sp.add_argument("--save", action="store_true", help="写 Flash 固化（= zr save，重启保持）")
    sp.add_argument("--yes", action="store_true", help="确认执行危险操作")
    sp.set_defaults(func=cmd_zero)

    sp = sub.add_parser("reboot", help="SYS_RESET（⚠ --yes）：整板软复位，力矩全掉臂下沉")
    sp.add_argument("--yes", action="store_true", help="确认执行危险操作")
    sp.set_defaults(func=cmd_reboot)

    sp = sub.add_parser("center", help="CENTER_CALIB（⚠⚠ --yes）：六轴推 raw 2048 居中，= `M a`，不可逆")
    sp.add_argument("--yes", action="store_true", help="确认执行危险操作")
    sp.set_defaults(func=cmd_center)

    sp = sub.add_parser("move", help="MOVE_JOINTS（舵机度）")
    sp.add_argument("pose", help='6 个角度，逗号或空格分隔，如 "180,180,180,180,180,180"')
    sp.add_argument("--v", type=float, default=25.0, help="速度 deg/s（默认 25）")
    sp.add_argument("--a", type=float, default=0.0, help="加速度 deg/s²")
    sp.add_argument("--delta", action="store_true", help="★点动必须用这个：相对当前角")
    sp.add_argument("--wait", action="store_true", help="发完读一次状态做到位检查")
    sp.set_defaults(func=cmd_move)

    sp = sub.add_parser("cart", help="MOVE_CART 笛卡尔（⚠ 会归零 J5/J6）")
    sp.add_argument("x", type=float)
    sp.add_argument("y", type=float)
    sp.add_argument("z", type=float)
    sp.add_argument("--pitch", type=float, default=0.0)
    sp.add_argument("--v", type=float, default=25.0)
    sp.add_argument("--a", type=float, default=0.0)
    sp.set_defaults(func=cmd_cart)

    sp = sub.add_parser("stop", help="STOP 平滑停止")
    sp.set_defaults(func=cmd_stop)

    sp = sub.add_parser("torque", help="SET_TORQUE")
    sp.add_argument("--joint", type=int, default=None, help="0~5；省略=全部")
    sp.add_argument("--off", action="store_true")
    sp.set_defaults(func=cmd_torque)

    sp = sub.add_parser("stream", help="STREAM_JOINTS 限速走到目标并保持（验证流模式）")
    sp.add_argument("--pose", default=None, help="6 个**逻辑角**；省略=留在原位")
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--sec", type=float, default=2.0)
    sp.add_argument("--slew", type=float, default=240.0)
    sp.set_defaults(func=cmd_stream)

    sp = sub.add_parser("params", help="运行时参数（LP/SP）")
    sp.add_argument("action", choices=["list", "get", "set", "count", "save"])
    sp.add_argument("id", nargs="?", type=int)
    sp.add_argument("value", nargs="?", type=float)
    sp.set_defaults(func=cmd_params)

    sp = sub.add_parser("teleop", help="主从遥操（网络路线，推荐）")
    sp.add_argument("--leader", required=True)
    sp.add_argument("--follower", required=True)
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--sec", type=float, default=None, help="省略=跑到 Ctrl-C")
    sp.add_argument("--slew", type=float, default=240.0, help="每轴限速 deg/s")
    sp.add_argument("--gain", type=float, default=1.0)
    sp.add_argument("--sync", default="hold", choices=["hold", "align", "relative", "absolute"])
    sp.add_argument("--soft-limit", default=None, help="统一 ±值，如 20 → 6 轴各 ±20°")
    sp.set_defaults(func=cmd_teleop)

    sp = sub.add_parser("record", help="遥操 + 录制示教轨迹（PC 侧，无 80s 上限）")
    sp.add_argument("--leader", required=True)
    sp.add_argument("--follower", required=True)
    sp.add_argument("--out", required=True, help=".json / .jsonl / .csv / .npz")
    sp.add_argument("--sec", type=float, default=30.0)
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--slew", type=float, default=240.0)
    sp.add_argument("--sync", default="hold", choices=["hold", "align", "relative", "absolute"])
    sp.set_defaults(func=cmd_record)

    sp = sub.add_parser("play", help="回放轨迹")
    sp.add_argument("file")
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--speed", type=float, default=1.0)
    sp.add_argument("--loop", type=int, default=1, help="0=无限")
    sp.add_argument("--force", action="store_true", help="有警告也照放")
    sp.add_argument("--no-approach", action="store_true",
                    help="不做'先走到轨迹起点'的接近段（默认做，避免第一帧大跳变被整段拒收）")
    sp.add_argument("--approach-speed", type=float, default=20.0)
    sp.add_argument("--convert", default=None, help="先导出成该格式再回放")
    sp.add_argument("--no-play", action="store_true", help="只导出不回放")
    sp.set_defaults(func=cmd_play)

    sp = sub.add_parser("traj", help="轨迹检查/转换/重采样/平滑")
    sp.add_argument("action", choices=["info", "convert", "resample", "smooth"])
    sp.add_argument("file")
    sp.add_argument("out", nargs="?", default=None)
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--window", type=int, default=5)
    sp.set_defaults(func=cmd_traj)

    sp = sub.add_parser("console", help="发串口文本命令（可多条）")
    sp.add_argument("--port", required=True)
    sp.add_argument("--baud", type=int, default=115200)
    sp.add_argument("--cmd", action="append", required=True,
                    help="可重复，如 --cmd h --cmd s --cmd zi")
    sp.set_defaults(func=cmd_console)

    sp = sub.add_parser("text", help="网口下发任意控制台命令（TEXT_EXEC 透传）")
    sp.add_argument("--cmd", action="append", required=True,
                    help="可重复，如 --cmd s --cmd zi --cmd 'link'")
    sp.set_defaults(func=cmd_text)

    sp = sub.add_parser("mock", help="起假臂 UDP 服务（无真机开发上位机用）")
    sp.add_argument("--host", default="0.0.0.0")
    sp.add_argument("--port", type=int, default=5000)
    sp.add_argument("--latency-ms", type=float, default=0.0,
                    help="每包延迟（真机实测 p50 1.61 ms）")
    sp.add_argument("--loss", type=float, default=0.0, help="丢包率 0~1")
    sp.add_argument("--drop-first", type=int, default=0, help="开局先丢 N 包")
    sp.add_argument("--estop", action="store_true")
    sp.add_argument("--fail-joints", default="", help="逗号分隔轴号(0基)，如 2 或 0,3")
    sp.add_argument("--unreachable", action="store_true", help="MOVE_CART 恒回 err=3")
    sp.add_argument("--fw-tag", default=None, help="改假臂报的固件号")
    sp.add_argument("-v", "--verbose", action="store_true", help="打印每条命令")
    sp.add_argument("--selfcheck", action="store_true")
    sp.set_defaults(func=cmd_mock)

    sp = sub.add_parser("http-bridge", help="★ HTTP↔UDP 边车（上位机用 HTTP/JSON 调臂）")
    sp.add_argument("--target", default="192.168.7.120:5000",
                    help="臂的 UDP 地址 IP:PORT，或 net://IP:PORT / sim://")
    sp.add_argument("--bind", default="127.0.0.1",
                    help="监听地址（默认 127.0.0.1；⚠ 无鉴权，别改成 0.0.0.0）")
    sp.add_argument("--port", type=int, default=8080)
    sp.add_argument("--timeout", type=float, default=0.3, help="到臂的单次请求超时秒")
    sp.add_argument("-v", "--verbose", action="store_true", help="打印每个 HTTP 请求")
    sp.add_argument("--selfcheck", action="store_true", help="离线自检（假臂 + 真 HTTP，18 项）")
    sp.set_defaults(func=cmd_http_bridge)

    sp = sub.add_parser("link", help="链路开关（link / link can|net|off / link save）")
    sp.add_argument("--port", required=True)
    sp.add_argument("--baud", type=int, default=115200)
    sp.add_argument("--set", choices=["can", "net", "off"], default=None)
    sp.add_argument("--save", action="store_true")
    sp.set_defaults(func=cmd_link)

    sp = sub.add_parser("relay", help="PC 中转（主臂 sp 位置流 → 从臂 #P）")
    sp.add_argument("--leader-port", required=True)
    sp.add_argument("--follower-port", required=True)
    sp.add_argument("--baud", type=int, default=115200)
    sp.add_argument("--hz", type=int, default=30)
    sp.add_argument("--slew", type=float, default=240.0)
    sp.add_argument("--sec", type=float, default=None)
    sp.set_defaults(func=cmd_relay)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    p = build_parser()
    args = p.parse_args(argv)
    if not getattr(args, "cmd", None):
        p.print_help()
        return 0
    if not hasattr(args, "func"):
        p.print_help()
        return 0
    try:
        return args.func(args)
    except KeyboardInterrupt:
        print("\n中断", file=sys.stderr)
        return 130
    except Exception as e:                                   # noqa: BLE001
        if os.environ.get("ARMCTL_TRACE"):
            raise
        _die("%s: %s" % (type(e).__name__, e), 1)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
