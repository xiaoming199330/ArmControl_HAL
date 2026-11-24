#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
launch_teach.py — **主从臂遥操 + 示教（录制 / 检查 / 回放）一键启动**

四种模式：

    record  拖主臂，从臂跟随，同时把主臂轨迹录下来（PC 侧，**无固件 80 s 上限**）
    info    只看轨迹：多少帧 / 多长 / 单帧最大跳变 / 会不会被固件拒
    play    把轨迹回放到某台臂上（默认先走到轨迹起点，避免第一帧大跳变被整段拒收）
    list    列出已经录过的轨迹文件

快速开始
--------
    # ① 录 10 秒（先离线演练，不需要硬件）
    python launch_teach.py --mode record --sec 10 --name demo1 \\
        --leader sim:// --follower sim:// --yes --allow-uncalibrated

    # ② 看看录成什么样
    python launch_teach.py --mode info trajectories/demo1.json

    # ③ 回放（第一遍一定先 0.5 倍速）
    python launch_teach.py --mode play trajectories/demo1.json --speed 0.5 \\
        --follower sim:// --yes --allow-uncalibrated

退出码
------
    0 正常   2 配置/连接/预检/文件问题   3 链路丢失   4 急停中止   5 未确认安全
"""

from __future__ import annotations

import argparse
import os
import time

import _common as K


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="launch_teach.py",
        description="主从臂遥操示教：录制 / 检查 / 回放",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="轨迹格式按扩展名决定：.json（默认，可读）/ .jsonl（流式）/ .csv（给 Excel、"
               "LeRobot）/ .npz（给 numpy，需装 numpy）。\n"
               "配置文件：%s\\%s" % (K.HERE, K.CONFIG_NAME))
    p.add_argument("--config", default=None)
    p.add_argument("--mode", default="record",
                   choices=["record", "info", "play", "list"], help="默认 record")
    p.add_argument("file_pos", nargs="?", default=None,
                   help="轨迹文件（位置参数，等价于 --file），如 trajectories/demo1.json")
    p.add_argument("--file", default=None, help="info/play 的轨迹文件")
    p.add_argument("--name", default=None, help="record 的轨迹名（默认取配置 teach.name）")
    p.add_argument("--out-dir", default=None, help="轨迹目录（默认取配置 teach.out_dir）")
    p.add_argument("--leader", default=None)
    p.add_argument("--follower", default=None)
    p.add_argument("--sec", type=float, default=None, help="录/放多少秒")
    p.add_argument("--hz", type=float, default=None)
    p.add_argument("--slew", type=float, default=None)
    p.add_argument("--sync", default=None, choices=["hold", "align", "relative", "absolute"])
    p.add_argument("--gain", type=float, default=None, help="跟随增益，默认 1.0")
    p.add_argument("--soft-limit", default=None,
                   help="6 个逻辑角软限位，如 20,20,20,20,70,33 → 各轴 ±该值")
    p.add_argument("--speed", type=float, default=None, help="回放倍速，默认 0.5")
    p.add_argument("--loop", type=int, default=None, help="回放次数，0=无限")
    p.add_argument("--no-approach", action="store_true",
                   help="回放时不做「先走到起点」的接近段（不推荐：首帧大跳变会被整段拒收）")
    p.add_argument("--dry-run", action="store_true", help="回放时一帧都不发，只走流程")
    p.add_argument("--force", action="store_true", help="轨迹有警告也照放")
    p.add_argument("--convert", default=None, help="额外导出成该格式（csv/json/jsonl/npz）")
    p.add_argument("--allow-uncalibrated", action="store_true")
    p.add_argument("--yes", action="store_true")
    return p


def _paths(cfg: dict, a) -> tuple[str, str]:
    out_dir = a.out_dir or cfg.get("teach", {}).get("out_dir", "trajectories")
    if not os.path.isabs(out_dir):
        out_dir = os.path.join(K.HERE, out_dir)
    name = a.name or cfg.get("teach", {}).get("name", "demo")
    return out_dir, name


def _load_arm(uri: str, cfg: dict, label: str):
    """连接 + 预检。返回 (arm, 预检结果) 或 (None, None)。"""
    arm, err = K.connect(uri, cfg.get("timeout_s", 0.3))
    if err:
        K.bad("连不上%s %s：%s" % (label, uri, err))
        return None, None
    arm.open()
    return arm, K.preflight(arm, uri, label)


def main(argv=None) -> int:
    K.enable_ansi()
    a = build_parser().parse_args(argv)
    if a.soft_limit:
        a.soft_limit = K.parse_soft_limit(a.soft_limit)
    a.file = a.file or a.file_pos          # 位置参数与 --file 等价
    cfg, path = K.load_config(a.config)
    leader_uri = a.leader or cfg["leader"]
    follower_uri = a.follower or cfg["follower"]
    out_dir, name = _paths(cfg, a)
    teach_cfg = cfg.get("teach", {})
    sec = a.sec if a.sec is not None else float(teach_cfg.get("duration_s", 30.0))
    speed = a.speed if a.speed is not None else float(teach_cfg.get("playback_speed", 0.5))
    loop = a.loop if a.loop is not None else int(teach_cfg.get("playback_loop", 1))

    print("%s═══ 遥操示教 · 一键启动 ═══%s" % (K.C.BLU, K.C.R))
    print("%s模式：%s%s" % (K.C.B, a.mode, K.C.R))

    # ============================================================ info / list
    if a.mode in ("info", "list"):
        K.TOTAL_STEPS = 3
        from arm_sdk import Trajectory
        K.step(1, "读配置")
        K.ok("配置：%s" % path)

        if a.mode == "list":
            K.step(2, "列出轨迹（%s）" % out_dir)
            if not os.path.isdir(out_dir):
                K.warn("目录不存在：%s" % out_dir)
                return 0
            files = sorted(f for f in os.listdir(out_dir)
                           if os.path.splitext(f)[1].lower() in
                           (".json", ".jsonl", ".csv", ".npz"))
            if not files:
                K.warn("目录里还没有轨迹文件 —— 先用 --mode record 录一条")
                return 0
            for f in files:
                fp = os.path.join(out_dir, f)
                try:
                    t = Trajectory.load(fp)
                    K.ok("%-28s %2d 帧 / %6.2fs / %.0f Hz  %s"
                         % (f, t.point_count, t.duration, t.rate,
                            time.strftime("%m-%d %H:%M",
                                          time.localtime(os.path.getmtime(fp)))))
                except Exception as e:                             # noqa: BLE001
                    K.bad("%-28s 读不了：%s" % (f, e))
            K.step(3, "收尾")
            K.info("要看细节：--mode info --file <上面任一文件>")
            return 0

        K.step(2, "检查轨迹")
        fp = a.file or os.path.join(out_dir, name + ".json")
        if not os.path.exists(fp):
            K.bad("文件不存在：%s" % fp)
            K.info("用 --out-dir / --file 指定，或先 --mode record 录一条")
            return 2
        t = Trajectory.load(fp)
        K.ok(t.summary())
        v = t.validate()
        if v["ok"]:
            K.ok("校验通过：没有会被固件拒收的单帧跳变")
        else:
            K.warn("校验有警告（回放需 --force）：%s" % v["warnings"])
        ms = t.max_step()
        K.info("单帧最大跳变（度）：%s" % ", ".join("%.1f" % x for x in ms))
        K.info("固件硬门限是 25°/帧；超过的帧会被整段拒收 ⇒ 回放时建议 --speed 调低")
        if a.convert:
            dst = os.path.splitext(fp)[0] + "." + a.convert.lstrip(".")
            t.save(dst)
            K.ok("已导出 %s" % dst)
        K.step(3, "收尾")
        K.info("回放：python launch_teach.py --mode play %s --speed 0.5" % fp)
        return 0

    # ============================================================ play
    if a.mode == "play":
        K.TOTAL_STEPS = 6
        from arm_sdk import PlaybackSession, TeleopConfig, Trajectory
        K.step(1, "读配置与轨迹")
        K.ok("配置：%s" % path)
        fp = a.file or os.path.join(out_dir, name + ".json")
        if not os.path.exists(fp):
            K.bad("轨迹文件不存在：%s" % fp)
            return 2
        traj = Trajectory.load(fp)
        K.ok(traj.summary())
        v = traj.validate()
        if not v["ok"]:
            K.warn("轨迹有警告：%s" % v["warnings"])
            if not a.force:
                K.bad("先按提示修（或 --force 明确承担风险）")
                return 2
        K.info("回放目标臂：%s   倍速 %.2f   次数 %d" % (follower_uri, speed, loop))
        if a.convert:
            dst = os.path.splitext(fp)[0] + "." + a.convert.lstrip(".")
            traj.save(dst)
            K.ok("已先导出 %s" % dst)

        K.step(2, "读配置：连接目标臂")
        arm, r = _load_arm(follower_uri, cfg, "回放臂")
        if arm is None:
            return 2
        if r and not r["ok"] and not (a.allow_uncalibrated
                                      and r.get("calibrated") is False):
            K.bad("回放臂没通过预检 —— 先修")
            return 2
        K.step(3, "目标臂当前姿态（回放会从这开始）")
        try:
            st = arm.get_status()
            K.ok("舵机度：%s" % ", ".join(
                "--" if x is None else "%.1f" % x for x in st.as_servo_deg()))
        except Exception as e:                                     # noqa: BLE001
            K.warn("读状态失败：%s" % e)

        K.step(4, "安全确认（回放会真的动臂）")
        if not K.safety_gate("轨迹回放", a.yes):
            K.bad("未确认 ⇒ 退出")
            return 5

        K.step(5, "开始回放")
        with arm:
            pb = PlaybackSession(arm, cfg=TeleopConfig(hz=a.hz or cfg.get("hz", 50.0)),
                                 estop_check_s=0.3, dry_run=a.dry_run,
                                 approach=not a.no_approach,
                                 approach_speed=20.0,
                                 on_event=lambda m: K.info(m))
            if a.dry_run:
                K.warn("--dry-run：一帧都不会发出去")
            K.info("先做接近段（走到轨迹起点）→ 再按 %.2f 倍速放 %d 次" % (speed, loop))
            st = pb.play(traj, speed=speed, loop=loop)
            K.ok(st.summary())
            if st.ack_limit:
                K.warn("有 %d 帧被固件拒收(err=4)。常见原因：单帧跳变 >25° / 从臂 `sf 0` / "
                       "起点没接近到位" % st.ack_limit)

        K.step(6, "收尾")
        K.ok("回放结束，已停臂。")
        return 0

    # ============================================================ record
    K.TOTAL_STEPS = 6
    from arm_sdk import (TeleopAbort, TeleopLinkLost, TeleopTeachSession, Trajectory)

    K.step(1, "读配置")
    K.ok("配置：%s" % path)
    tc = K.build_teleop_config(cfg, a)
    tc.wait_ack = False
    K.info("主臂 %s" % leader_uri)
    K.info("从臂 %s" % follower_uri)
    K.info("参数：%.0f Hz / 限速 %.0f°/s / gain %.2f / sync=%s"
           % (tc.hz, tc.slew_deg_per_s, tc.gain, tc.sync))
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, name + ".json")
    K.info("输出：%s（录制 %.1f 秒）" % (out_file, sec))
    K.info("⚠ PC 侧录制**没有固件的 80 秒上限**，想录多久录多久")

    K.step(2, "连主臂并预检")
    lead, r1 = _load_arm(leader_uri, cfg, "主臂")
    if lead is None:
        return 2
    if r1 and not r1["ok"] and not (a.allow_uncalibrated
                                    and r1.get("calibrated") is False):
        K.bad("主臂没通过预检 —— 先修")
        return 2

    K.step(3, "连从臂并预检")
    foll, r2 = _load_arm(follower_uri, cfg, "从臂")
    if foll is None:
        return 2
    if r2 and not r2["ok"] and not (a.allow_uncalibrated
                                    and r2.get("calibrated") is False):
        K.bad("从臂没通过预检 —— 先修")
        return 2

    K.step(4, "安全确认")
    if not K.safety_gate("遥操示教（录制）", a.yes):
        K.bad("未确认 ⇒ 退出（没有发出任何运动命令）")
        return 5

    K.step(5, "开始录：拖主臂，从臂跟随，同时记录")
    rc, traj = 0, None
    try:
        with lead, foll:
            ts = TeleopTeachSession(lead, foll, out=out_file, name=name, hz=tc.hz,
                                    cfg=tc, on_event=lambda m: K.info(m))
            try:
                ts.start()
                K.ok("已进入跟随+录制状态。按 Ctrl-C 提前结束。")
                traj = ts.record(duration=sec)
            except TeleopLinkLost as e:
                K.bad("链路丢失：%s" % e)
                rc = 3
            except TeleopAbort as e:
                K.bad("中止：%s" % e)
                rc = 4
            except KeyboardInterrupt:
                K.warn("用户中断 ⇒ 已录的部分会保存")
                ts.stop()
                traj = ts.last_traj or ts.rec.trajectory()
            K.info(ts.stats.summary())
    except Exception as e:                                         # noqa: BLE001
        K.bad("录制异常：%s: %s" % (type(e).__name__, e))
        if os.environ.get("ARMCTL_TRACE"):
            raise
        return 3

    K.step(6, "收尾")
    if traj is None or traj.point_count == 0:
        K.bad("没录到帧（是不是没拖动主臂？）")
        return rc or 2
    traj.save(out_file)
    K.ok("已存 %s" % out_file)
    K.ok(traj.summary())
    v = traj.validate()
    if v["ok"]:
        K.ok("校验通过：没有会被固件拒收的单帧跳变")
    else:
        K.warn("校验警告：%s" % v["warnings"])
        K.info("回放时先 --speed 0.3~0.5，或先 --mode info 看看哪一帧跳了")
    if a.convert:
        dst = os.path.splitext(out_file)[0] + "." + a.convert.lstrip(".")
        traj.save(dst)
        K.ok("已导出 %s" % dst)
    print("")
    K.info("下一步：")
    K.info("  python launch_teach.py --mode info %s" % out_file)
    K.info("  python launch_teach.py --mode play %s --speed 0.5" % out_file)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
