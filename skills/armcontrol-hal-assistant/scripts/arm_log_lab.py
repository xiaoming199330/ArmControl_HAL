#!/usr/bin/env python3
"""
ArmControl_HAL 串口日志实验室：采集 / 分析 / 回测 三合一。

子命令:
  capture   串口采集到日志文件（需 pyserial）
      python arm_log_lab.py capture --port COM13 --baud 115200 --out logs/session.txt

  analyze   故障签名分析（内置 references/analysis-rules.md 规则）
      python arm_log_lab.py analyze --log logs/session.txt

  backtest  运动学/轨迹/修正回测
      python arm_log_lab.py backtest --log logs/session.txt --report
      python arm_log_lab.py backtest --ik 150 0 150 0
      python arm_log_lab.py backtest --cases memory/fault-cases.md

运动学模型为固件 kinematics.c / trajectory.c 的忠实 Python 移植：
  d1=58 a2=a3=a4=54 a5=64 (mm)，SERVO_OFFSET=150°，默认限位同 main.c JOINT_MIN/MAX。
"""

import argparse
import math
import re
import sys
import time
from pathlib import Path

# ---------------- 固件常量（与 main.c / kinematics.h / arm_config.h 对齐） ----------------
D1, A2, A3, A4, A5 = 58.0, 54.0, 54.0, 54.0, 64.0
LINKS = [A2, A3, A4, A5]
ARM_MAX_REACH = A2 + A3 + A4 + A5
SERVO_OFFSET = [150.0] * 6
JOINT_MIN = [40.0, 70.0, 50.0, 40.0, 30.0, 0.0]
JOINT_MAX = [240.0, 240.0, 210.0, 260.0, 280.0, 300.0]
CCD_MAX_ITER = 100
CCD_CONVERGE_MM = 0.5
IK_FAIL_THRESHOLD = 1.0
SCURVE_TJ = 0.15
TRAJ_HZ = 200.0
CORR_THRESHOLD = 2.0

# ---------------- 运动学移植（kinematics.c） ----------------

def servo_deg_to_rad(deg, joint):
    return (deg - SERVO_OFFSET[joint]) * math.pi / 180.0


def rad_to_servo_deg(rad, joint):
    deg = rad * 180.0 / math.pi + SERVO_OFFSET[joint]
    return max(0.0, min(300.0, deg))


def fk_solve(joints_rad):
    """返回 (x, y, z, pitch_rad)。移植 FK_Solve（WORLD_OFFSET=0）。"""
    t1 = joints_rad[0]
    sum_ang, r, h = 0.0, 0.0, 0.0
    for i in range(4):
        sum_ang += joints_rad[i + 1]
        r += LINKS[i] * math.sin(sum_ang)
        h += LINKS[i] * math.cos(sum_ang)
    pitch = sum_ang + joints_rad[5]
    return r * math.cos(t1), r * math.sin(t1), D1 + h, pitch


def _ccd_run(ang, r_target, h_target, jlim_min, jlim_max):
    for _ in range(CCD_MAX_ITER):
        s, er, eh = 0.0, 0.0, 0.0
        for i in range(4):
            s += ang[i]
            er += LINKS[i] * math.sin(s)
            eh += LINKS[i] * math.cos(s)
        if math.hypot(r_target - er, h_target - eh) < CCD_CONVERGE_MM:
            break
        for j in range(3, -1, -1):
            js, jr, jh = 0.0, 0.0, 0.0
            for i in range(j):
                js += ang[i]
                jr += LINKS[i] * math.sin(js)
                jh += LINKS[i] * math.cos(js)
            s, er, eh = 0.0, 0.0, 0.0
            for i in range(4):
                s += ang[i]
                er += LINKS[i] * math.sin(s)
                eh += LINKS[i] * math.cos(s)
            ver, veh = er - jr, eh - jh
            vtr, vth = r_target - jr, h_target - jh
            ve_len = math.hypot(ver, veh)
            vt_len = math.hypot(vtr, vth)
            if ve_len < 0.001 or vt_len < 0.001:
                continue
            dot = max(-1.0, min(1.0, (ver * vtr + veh * vth) / (ve_len * vt_len)))
            cross = (ver * vth - veh * vtr) / (ve_len * vt_len)
            ang[j] -= math.atan2(cross, dot)
            ang[j] = max(jlim_min[j + 1], min(jlim_max[j + 1], ang[j]))
    s, er, eh = 0.0, 0.0, 0.0
    for i in range(4):
        s += ang[i]
        er += LINKS[i] * math.sin(s)
        eh += LINKS[i] * math.cos(s)
    return math.hypot(r_target - er, h_target - eh)


def ik_solve(x, y, z, pitch_deg=0.0, joint_min=None, joint_max=None):
    """移植 IK_Solve。返回 (ok, joints_rad[6], best_err, detail)。"""
    jmin_deg = joint_min or JOINT_MIN
    jmax_deg = joint_max or JOINT_MAX
    jlim_min = [servo_deg_to_rad(jmin_deg[i], i) for i in range(6)]
    jlim_max = [servo_deg_to_rad(jmax_deg[i], i) for i in range(6)]

    target_pitch = pitch_deg * math.pi / 180.0
    theta1 = math.atan2(y, x)
    theta1 = max(jlim_min[0], min(jlim_max[0], theta1))
    r_target = math.hypot(x, y)
    h_target = z - D1
    planar_dist = math.hypot(r_target, h_target)
    if planar_dist > ARM_MAX_REACH + 0.5:
        return False, None, planar_dist, {"reason": "beyond reach", "planar_dist": planar_dist}

    aim = math.atan2(r_target, h_target)
    fold = 0.0
    if 0.1 < planar_dist < ARM_MAX_REACH:
        fold = math.acos(min(1.0, planar_dist / ARM_MAX_REACH)) * 0.5
    starts = [[0, 0, 0, 0], [aim * 0.5, 0, 0, 0], [fold, fold, -fold, -fold],
              [fold, 0, fold, 0], [aim * 0.25] * 4]
    best_ang, best_err, best_si = None, 1e10, -1
    for si, st in enumerate(starts):
        ang = [max(jlim_min[i + 1], min(jlim_max[i + 1], st[i])) for i in range(4)]
        err = _ccd_run(ang, r_target, h_target, jlim_min, jlim_max)
        if err < best_err:
            best_err, best_ang, best_si = err, list(ang), si
        if err < CCD_CONVERGE_MM:
            break
    joints = [theta1] + best_ang + [0.0]
    joints[5] = target_pitch - sum(best_ang)
    joints[5] = max(jlim_min[5], min(jlim_max[5], joints[5]))
    ok = best_err <= IK_FAIL_THRESHOLD
    return ok, joints, best_err, {"start": best_si, "planar_dist": planar_dist}


# ---------------- S 曲线规划移植（trajectory.c 时间参数） ----------------

def scurve_plan(start, end, v_max, accel):
    """返回 dict(tj, ta, tv, t_acc, t_total, v_peak, ticks, grade)。移植 Trajectory_Plan。"""
    max_d = max(abs(e - s) for s, e in zip(start, end))
    if max_d < 0.01:
        return {"active": 0, "t_total": 0.0, "ticks": 0}
    tj = SCURVE_TJ
    if v_max / accel >= tj:
        ta, v_peak = v_max / accel - tj, v_max
        grade = "full"
    else:
        ta, v_peak, grade = 0.0, accel * tj, "triangle"
    d_acc = accel * tj * tj + 0.5 * accel * ta * (3 * tj + ta)
    if max_d >= 2.0 * d_acc:
        tv = (max_d - 2.0 * d_acc) / v_peak
    elif ta > 0.0:
        M = max_d / (2.0 * accel) - tj * tj
        if M <= 0.0:
            tj = math.sqrt(max_d / (2.0 * accel))
            ta, v_peak, d_acc, grade = 0.0, accel * tj, accel * tj * tj, "short"
        else:
            ta = -1.5 * tj + math.sqrt(2.25 * tj * tj + 2.0 * M)
            v_peak, d_acc, grade = accel * (tj + ta), max_d / 2.0, "mid"
        tv = 0.0
    else:
        tj = math.sqrt(max_d / (2.0 * accel))
        ta, v_peak, d_acc, tv, grade = 0.0, accel * tj, accel * tj * tj, 0.0, "short"
    t_acc = 2.0 * tj + ta
    t_total = 2.0 * t_acc + tv
    return {"active": 1, "tj": tj, "ta": ta, "tv": tv, "t_acc": t_acc,
            "t_total": t_total, "v_peak": v_peak, "max_d": max_d,
            "ticks": int(t_total * TRAJ_HZ) + 1, "grade": grade}


# ---------------- 日志解析 ----------------

RE_MOVE_START = re.compile(r"Move started:\s*v=(\d+)\s*deg/s\s*a=(\d+)")
RE_FROM = re.compile(r"From:\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)")
RE_TO = re.compile(r"To:\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)")
RE_PV = re.compile(r"J(\d)=(\d+)\.(\d+)deg")
RE_CORR = re.compile(r"J(\d) corr #(\d+): actual=(-?\d+) tgt=(-?\d+) -> (-?\d+) \(err=(-?\d+), gain=(\d+)%\)")
RE_CORR_DONE = re.compile(r"Adaptive correction (DONE|STOP).*?\((\d+) rounds\)|Adaptive correction (STOP): max")
RE_IK_LINE = re.compile(r"IK: x=(-?\d+) y=(-?\d+) z=(-?\d+) ->")
RE_IK_REJECT = re.compile(r"IK: (unreachable|solution out of joint limits|error too large)")


def parse_log(text):
    """解析采集日志为结构化事件列表。"""
    events = {"moves": [], "corr": [], "pv": [], "ik": [], "raw_lines": text.splitlines()}
    cur_move = None
    for line in text.splitlines():
        m = RE_MOVE_START.search(line)
        if m:
            cur_move = {"v": int(m.group(1)), "a": int(m.group(2)),
                        "start": None, "target": None, "last_pv": None, "completed": False}
            events["moves"].append(cur_move)
            continue
        m = RE_FROM.search(line)
        if m and cur_move is not None:
            cur_move["start"] = [int(g) for g in m.groups()]
            continue
        m = RE_TO.search(line)
        if m and cur_move is not None:
            cur_move["target"] = [int(g) for g in m.groups()]
            continue
        if "Move complete" in line and cur_move is not None:
            cur_move["completed"] = True
            continue
        pv = RE_PV.findall(line)
        if len(pv) >= 2:
            angles = [0.0] * 6
            for j, di, dd in pv:
                idx = int(j) - 1
                if 0 <= idx < 6:
                    angles[idx] = float(f"{di}.{dd}")
            events["pv"].append(angles)
            if cur_move is not None:
                cur_move["last_pv"] = angles
            continue
        m = RE_CORR.search(line)
        if m:
            events["corr"].append({"joint": int(m.group(1)), "round": int(m.group(2)),
                                   "actual": int(m.group(3)), "target": int(m.group(4)),
                                   "corrected": int(m.group(5)), "err": int(m.group(6)),
                                   "gain": int(m.group(7))})
            continue
        m = RE_IK_LINE.search(line)
        if m:
            events["ik"].append({"x": int(m.group(1)), "y": int(m.group(2)), "z": int(m.group(3)), "ok": True})
            continue
        m = RE_IK_REJECT.search(line)
        if m:
            events["ik"].append({"reject": m.group(1), "ok": False})
    return events


# ---------------- analyze：故障签名匹配 ----------------

SIGNATURES = [
    ("RCA-BUS-LOOPBACK-FAIL", "CRITICAL", ["LOOPBACK FAILED"],
     "TX→74HC126→RX 硬件链路断：检查焊接/126供电/方向脚"),
    ("RCA-BUS-NO-RESPONSE", "CRITICAL", ["LOOPBACK OK", "No servo response"],
     "回环正常但舵机无应答：查舵机供电/DATA接线/ID/波特率"),
    ("RCA-IK-UNREACHABLE", "MAJOR", ["IK: unreachable"],
     "目标不可达：超臂展/偏移设错/限位钳制致CCD发散 → backtest 复现"),
    ("RCA-IK-LIMIT-REJECT", "MINOR", ["solution out of joint limits"],
     "IK解越限位：检查 L 限位表与 IK_SetJointLimits 同步"),
    ("RCA-IK-FK-VERIFY", "MAJOR", ["IK error too large"],
     "FK验证超差：CCD收敛到次优解 → backtest 查看5起点细节"),
    ("RCA-CORR-OSC", "MAJOR", ["OVERSHOOT"],
     "修正振荡：增益过高/D不足 → 查 A g 初增益与 auto-tune 序列", 4),
    ("RCA-CORR-STALL", "MAJOR", ["STALL"],
     "修正停滞：负载/P不足/速度过低 → 查 T温度 V速度", 3),
    ("RCA-LOOP-STUCK", "MAJOR", ["Another loop is active"],
     "循环状态残留：发 S 清理；查多点到位容差±2°是否可达"),
    ("RCA-FLASH-INVALID", "MINOR", ["No valid PID/correction/speed config"],
     "Flash记录无效：版本升级/范围校验/从未保存 → 重新 F"),
    ("RCA-CLAMP", "MINOR", ["clamped"],
     "目标被限位钳制：核对当前限位表"),
    ("RCA-ISR-OVERRUN", "CRITICAL", ["ISR OVERRUN"],
     "主循环过慢：审查阻塞调用/降低反馈频率/日志分频"),
    ("RCA-ESTOP", "MAJOR", ["E-STOP active"],
     "急停触发：PE11电平与接线；E-Stop下仅 s/h 放行"),
]


def cmd_analyze(log_path):
    text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    print(f"== 故障签名分析: {log_path} ==\n")
    hits = []
    for rid, sev, patterns, advice, *minc in SIGNATURES:
        need = minc[0] if minc else 1
        matched, total = [], 0
        for p in patterns:
            c = text.count(p)
            total += c
            if c:
                matched.append(f"{p} x{c}")
        if matched and total >= need:
            conf = min(100, len(matched) * 20 + total * 5 + (10 if sev == "CRITICAL" else 0))
            hits.append((conf, sev, rid, matched, advice))
    if not hits:
        print("未命中已知故障签名。建议: 注入日志复现采集, 或将新案例沉淀到 analysis-rules.md")
        return 0
    hits.sort(key=lambda h: (-h[0], h[1]))
    for conf, sev, rid, matched, advice in hits:
        print(f"[{sev:8}] {rid}  置信度 {conf}%")
        print(f"    证据: {'; '.join(matched)}")
        print(f"    处置: {advice}\n")
    print(f"共 {len(hits)} 条命中。详见 references/analysis-rules.md")
    return 0


# ---------------- backtest ----------------

def bt_ik(x, y, z, pitch=0.0, verbose=True):
    ok, joints, err, detail = ik_solve(x, y, z, pitch)
    if not ok:
        if verbose:
            print(f"  IK({x},{y},{z},{pitch}°): 不可达  best_err={err:.2f}mm  {detail}")
        return False, err
    deg = [rad_to_servo_deg(joints[i], i) for i in range(6)]
    fx, fy, fz, fp = fk_solve(joints)
    rt_err = math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
    in_lim = all(JOINT_MIN[i] <= deg[i] <= JOINT_MAX[i] for i in range(6))
    if verbose:
        print(f"  IK({x},{y},{z},{pitch}°) -> J={['%.1f' % d for d in deg]}")
        print(f"    FK往返误差={rt_err:.3f}mm  CCD best_err={err:.3f}mm  "
              f"起点#{detail['start']}  限位内={'是' if in_lim else '否'}")
    return rt_err <= IK_FAIL_THRESHOLD and in_lim, rt_err


def bt_log(log_path, report=False):
    text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    ev = parse_log(text)
    print(f"== 回测报告: {log_path} ==\n")
    n_pass = n_fail = 0
    for i, mv in enumerate(ev["moves"], 1):
        if mv["start"] is None or mv["target"] is None:
            continue
        plan = scurve_plan(mv["start"], mv["target"], float(mv["v"]), float(mv["a"]))
        print(f"--- 运动 #{i}: v={mv['v']} a={mv['a']} ---")
        print(f"  From={mv['start']}")
        print(f"  To  ={mv['target']}")
        print(f"  规划: 档位={plan.get('grade')} v_peak={plan.get('v_peak', 0):.1f}deg/s "
              f"t_total={plan.get('t_total', 0):.2f}s ticks={plan.get('ticks')}")
        verdict, note = "PASS", ""
        if mv["last_pv"]:
            errs = [abs(p - t) for p, t in zip(mv["last_pv"], mv["target"])]
            worst = max(errs)
            wj = errs.index(worst) + 1
            print(f"  末次反馈={['%.1f' % a for a in mv['last_pv']]}")
            print(f"  到位偏差: max={worst:.1f}° (J{wj})  容差={CORR_THRESHOLD}°")
            if worst > CORR_THRESHOLD and not ev["corr"]:
                verdict, note = "FAIL", "到位超差且未见修正记录"
            elif worst > CORR_THRESHOLD:
                verdict, note = "WARN", "到位超差, 修正中"
        else:
            note = "无 pv 反馈数据(采集时未开 pv)"
        if ev["corr"]:
            rounds = max(c["round"] for c in ev["corr"])
            print(f"  修正: {len(ev['corr'])} 条记录, 最大轮数={rounds}")
        print(f"  结论: {verdict} {note}\n")
        n_pass += verdict == "PASS"
        n_fail += verdict == "FAIL"
    if ev["ik"]:
        print("--- IK 事件回放 ---")
        for e in ev["ik"]:
            if e.get("ok"):
                bt_ik(e["x"], e["y"], e["z"])
            else:
                print(f"  固件拒绝: {e['reject']}")
        print()
    print(f"汇总: {len(ev['moves'])} 次运动 | PASS={n_pass} FAIL={n_fail} | "
          f"pv帧={len(ev['pv'])} corr条={len(ev['corr'])}")
    if report:
        print("\n建议: 将本次日志与结论追加到 memory/run-journal.md；"
              "FAIL 案例沉淀到 memory/fault-cases.md")
    return 1 if n_fail else 0


def bt_cases(cases_path):
    text = Path(cases_path).read_text(encoding="utf-8", errors="replace")
    blocks = re.findall(r"```CASE\n(.*?)```", text, re.S)
    print(f"== 黄金案例回归: {cases_path} ({len(blocks)} 个 CASE) ==\n")
    total_pass = total_fail = 0
    for b in blocks:
        bid = re.search(r"id:\s*(\S+)", b)
        pts = re.findall(r"-\s*\[([^\]]+)\]", b)
        bid = bid.group(1) if bid else "?"
        print(f"CASE {bid} (ik-roundtrip, {len(pts)} 点):")
        for p in pts:
            vals = [float(v) for v in p.split(",")]
            ok, _ = bt_ik(*vals)
            total_pass += ok
            total_fail += (not ok)
            if not ok:
                print("    ^^ FAIL")
        print()
    print(f"回归汇总: PASS={total_pass} FAIL={total_fail}")
    return 1 if total_fail else 0


# ---------------- capture ----------------

def cmd_capture(port, baud, out, duration=0):
    try:
        import serial
    except ImportError:
        print("错误: 需要 pyserial（pip install pyserial）", file=sys.stderr)
        return 1
    outp = Path(out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    print(f"[*] {port} @ {baud} → {outp}  (Ctrl+C 停止)")
    t0 = time.time()
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser, \
             outp.open("w", encoding="utf-8") as f:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    text = data.decode("utf-8", errors="replace")
                    f.write(text)
                    f.flush()
                    sys.stdout.write(text)
                    sys.stdout.flush()
                if duration and time.time() - t0 > duration:
                    break
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"[!] 串口错误: {e}")
        return 1
    print(f"\n[*] 已保存 → {outp}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="ArmControl_HAL 串口日志实验室")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture", help="串口采集")
    c.add_argument("--port", default="COM13")
    c.add_argument("--baud", type=int, default=115200)
    c.add_argument("--out", required=True)
    c.add_argument("--duration", type=int, default=0, help="秒, 0=手动停止")

    a = sub.add_parser("analyze", help="故障签名分析")
    a.add_argument("--log", required=True)

    b = sub.add_parser("backtest", help="运动学/轨迹/修正回测")
    b.add_argument("--log", help="采集日志文件")
    b.add_argument("--report", action="store_true")
    b.add_argument("--ik", nargs="+", type=float, metavar="X",
                   help="x y z [pitch] 单点 IK 验证")
    b.add_argument("--cases", help="fault-cases.md 黄金案例回归")
    args = ap.parse_args()

    if args.cmd == "capture":
        return cmd_capture(args.port, args.baud, args.out, args.duration)
    if args.cmd == "analyze":
        return cmd_analyze(args.log)
    if args.cmd == "backtest":
        if args.ik:
            v = args.ik + [0.0] * (4 - len(args.ik))
            ok, _ = bt_ik(v[0], v[1], v[2], v[3])
            return 0 if ok else 1
        if args.cases:
            return bt_cases(args.cases)
        if args.log:
            return bt_log(args.log, args.report)
        print("错误: backtest 需要 --log / --ik / --cases 之一", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
