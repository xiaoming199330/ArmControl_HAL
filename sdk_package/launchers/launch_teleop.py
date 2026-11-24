#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
launch_teleop.py — **主从臂遥操 一键启动**（网口路线，推荐）

拖主臂 → 从臂实时跟随。走 UDP：实测往返 p50 **1.61 ms**，50 Hz 有 15× 余量，
两臂之间**一根线都不用接**（PC 就是那根线）。

快速开始
--------
    # ① 先离线演练（不需要任何硬件，验证流程和参数）
    python launch_teleop.py --leader sim:// --follower sim:// --sec 8 --yes

    # ② 真机（地址在 arm_link_config.json 里改）
    python launch_teleop.py

    # ③ 只想验证"两臂通不通、能不能动"，不想真动
    python launch_teleop.py --preflight-only

    # ④ 拖 30 秒就停
    python launch_teleop.py --sec 30

退出码
------
    0 正常结束   2 配置/连接/预检不过   3 链路丢失   4 急停中止   5 未确认安全
"""

from __future__ import annotations

import argparse
import os
import sys

import _common as K


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="launch_teleop.py",
        description="主从臂遥操一键启动（网口路线 1.61 ms）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="配置文件：%s\\%s（改它，不用改代码）\n"
               "停机：Ctrl-C —— 会自动给从臂发 STOP，不是「甩手不管」。"
               % (K.HERE, K.CONFIG_NAME))
    p.add_argument("--config", default=None, help="配置文件路径（默认同目录）")
    p.add_argument("--leader", default=None, help="主臂，如 net://192.168.7.121:5000")
    p.add_argument("--follower", default=None, help="从臂，如 net://192.168.7.120:5000")
    p.add_argument("--sec", type=float, default=None, help="跑多少秒（省略=跑到 Ctrl-C）")
    p.add_argument("--hz", type=float, default=None, help="流频率，默认 50")
    p.add_argument("--slew", type=float, default=None, help="每轴限速 deg/s，默认 240")
    p.add_argument("--gain", type=float, default=None, help="跟随增益，默认 1.0")
    p.add_argument("--sync", default=None,
                   choices=["hold", "align", "relative", "absolute"],
                   help="启动同步方式，默认 hold（不动从臂，最安全）")
    p.add_argument("--soft-limit", default=None,
                   help="6 个逻辑角软限位，如 20,20,20,20,70,33 → 各轴 ±该值；"
                        "也可写 6 组 min:max")
    p.add_argument("--preflight-only", action="store_true",
                   help="只做步骤 1~4（连接+预检+确认），不发任何运动包")
    p.add_argument("--allow-uncalibrated", action="store_true",
                   help="⚠ 零点未校准也放行（只降级为警告）。sim:// 演练时用")
    p.add_argument("--yes", action="store_true", help="跳过人工确认（CI/脚本用）")
    return p


def main(argv=None) -> int:
    K.enable_ansi()
    a = build_parser().parse_args(argv)
    if a.soft_limit:
        # 解析后放回 args.soft_limit —— build_teleop_config() 读的是这个字段
        a.soft_limit = K.parse_soft_limit(a.soft_limit)

    print("%s╔══════════════════════════════════════════════════════════╗%s"
          % (K.C.BLU, K.C.R))
    print("%s║  主从臂遥操 · 一键启动（网口 UDP，p50 1.61 ms）          ║%s"
          % (K.C.BLU, K.C.R))
    print("%s╚══════════════════════════════════════════════════════════╝%s"
          % (K.C.BLU, K.C.R))

    # ------------------------------------------------------------ [1/6] 配置
    K.step(1, "读配置")
    cfg, path = K.load_config(a.config)
    leader_uri = a.leader or cfg["leader"]
    follower_uri = a.follower or cfg["follower"]
    K.ok("配置：%s" % path)
    K.info("主臂 %s" % leader_uri)
    K.info("从臂 %s" % follower_uri)

    tc = K.build_teleop_config(cfg, a)
    K.info("参数：%.0f Hz / 限速 %.0f°/s / gain %.2f / sync=%s"
           % (tc.hz, tc.slew_deg_per_s, tc.gain, tc.sync))
    if tc.soft_limits_deg:
        K.info("软限位（逻辑角）：%s"
               % " ".join("[%.0f,%.0f]" % (lo, hi) for lo, hi in tc.soft_limits_deg))
    if leader_uri.startswith("sim"):
        K.warn("用到了 sim:// —— 这是内存假臂，不是真机；两臂互相独立")

    # ------------------------------------------------------ [2/6] 连主臂
    K.step(2, "连主臂并预检")
    lead, err = K.connect(leader_uri, cfg.get("timeout_s", 0.3))
    if err:
        K.bad("连不上主臂 %s：%s" % (leader_uri, err))
        return 2
    lead.open()
    r_lead = K.preflight(lead, leader_uri, "主臂")
    if not r_lead["ok"] and not (a.allow_uncalibrated
                                 and r_lead.get("calibrated") is False):
        K.bad("主臂没通过预检 —— 先按上面的提示修，别急着动臂")
        return 2
    K.print_limit_hint(lead, "主臂")

    # ------------------------------------------------------ [3/6] 连从臂
    K.step(3, "连从臂并预检")
    foll, err = K.connect(follower_uri, cfg.get("timeout_s", 0.3))
    if err:
        K.bad("连不上从臂 %s：%s" % (follower_uri, err))
        return 2
    foll.open()
    r_foll = K.preflight(foll, follower_uri, "从臂")
    if not r_foll["ok"] and not (a.allow_uncalibrated
                                 and r_foll.get("calibrated") is False):
        K.bad("从臂没通过预检 —— 先按上面的提示修")
        return 2
    K.print_limit_hint(foll, "从臂")

    # --------------------------------------------------------- [4/6] 确认
    K.step(4, "安全确认")
    if leader_uri.startswith("sim") and follower_uri.startswith("sim"):
        K.warn("两臂都是 sim:// ⇒ 确认这一步只影响流程演练，不会真的动臂")
    if not K.safety_gate("主从遥操", a.yes):
        K.bad("未确认 ⇒ 退出（没有发出任何运动命令）")
        return 5

    if a.preflight_only:
        K.step(5, "只做预检 —— 到此为止")
        K.ok("两臂均连通、预检通过。去掉 --preflight-only 即可真正开始遥操。")
        K.step(6, "收尾")
        K.info("未发送任何运动命令。")
        return 0

    # ------------------------------------------------------ [5/6] 开始遥操
    K.step(5, "开始遥操（拖主臂，从臂跟随）")
    from arm_sdk import TeleopAbort, TeleopLinkLost, TeleopSession
    rc = 0
    try:
        with lead, foll:
            tp = TeleopSession(lead, foll, cfg=tc)
            try:
                tp.start()
                K.ok("已进入跟随状态。%s"
                     % ("跑 %.1f 秒后自动停。" % a.sec if a.sec else
                        "按 Ctrl-C 停止（会自动停从臂）。"))
                tp.run(duration=a.sec)
            except TeleopLinkLost as e:
                K.bad("链路丢失：%s" % e)
                rc = 3
            except TeleopAbort as e:
                K.bad("中止：%s" % e)
                rc = 4
            except KeyboardInterrupt:
                K.warn("用户中断")
                tp.stop()
            print("")
            K.info(tp.stats.summary())
    except Exception as e:                                         # noqa: BLE001
        K.bad("遥操异常：%s: %s" % (type(e).__name__, e))
        if os.environ.get("ARMCTL_TRACE"):
            raise
        return 3

    # ------------------------------------------------------------ [6/6] 收尾
    K.step(6, "收尾")
    K.ok("已停从臂（STOP 已发）。主臂恢复手拖自由状态。")
    K.info("要录下来回放，用  python launch_teach.py --mode record")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
