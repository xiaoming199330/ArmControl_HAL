# -*- coding: utf-8 -*-
"""J5 限位一致性检查 — 离线复刻 apply_calib_limits() 的 **旧版** J5 分支。

⚠⚠ 【已作废 · 2026-09-16】本脚本针对的是"J5 限位 = zero ± 硬编码偏移"的旧固件。
    该机制已整体删除: J3/J5 与其余四轴统一, 直接采用 Phase 2 手推实测的
    calib_min_raw / calib_max_raw。当前源码树上再跑本脚本, 结论无意义,
    仅作历史留档 (当时用于证明"两条分支逐位相同"以防重复排查)。

为什么单独查 J5 (当时的背景)
------------------------------
J5 曾是六轴里**唯一**一个"限位 = calib_zero_raw ± 一对硬编码偏移"、且这对偏移
在 arm_config.h 里标着 `待重测 (旧舵机值)` 的关节:

    #define J5_LIMIT_OFF_MIN    (-1033)
    #define J5_LIMIT_OFF_MAX    (  787)

而 main.c 的 apply_calib_limits() 对 J5 有两条互斥路径:

    if (calib_zero_raw[4] != 0) {          /* 已校准 */
        JOINT_MIN_RAW[4] = wrap(zero - 1033);
        JOINT_MAX_RAW[4] = wrap(zero +  787);
    } else {                               /* 未校准 / zr 复位后 */
        JOINT_MIN_RAW[4] = deg_to_servo_raw(180.0f);
        JOINT_MAX_RAW[4] = deg_to_servo_raw(340.0f);
    }

关键: SERVO_OFFSET[4] = zero / 4095 * 360  (kinematics.c:126)。
所以"rel-home 关节角" = raw/4095*360 - SERVO_OFFSET[4]。

  * 走偏移分支时, zero 被减掉了 => 合法区间**恒定** = [-1033, +787] 折算的
    [-90.80, +69.19] 度, 与 zero 标定成多少无关。
  * 走兜底分支时, SERVO_OFFSET 恒为 180 (zr 把 zero 全清 0) => 合法区间
    = [180-180, 340-180] = [0, +160] 度。

两者相差 90.8 度。J3 也有类似结构, 但它的 else 兜底 (77.8/275.9 度) 恰好与
偏移分支自洽, 所以只有 J5 会在"校准过 / 没校准过"之间跳变。

本脚本不连硬件, 只做算术, 用来判断从臂 J5 会不会被静默夹住。
"""

DEG_PER_RAW = 360.0 / 4095.0
OFF_MIN, OFF_MAX = -1033, 787
ELSE_MIN_DEG, ELSE_MAX_DEG = 180.0, 340.0


def wrap(v):
    """wrap_raw: 折回 0~4095"""
    return v % 4096


def deg_to_raw(deg):
    return int(deg / 360.0 * 4095.0 + 0.5)


def raw_to_deg(raw):
    return raw / 4095.0 * 360.0


def j5_limits(zero):
    """复刻 apply_calib_limits() 的 J5 分支。返回 (min_raw, max_raw, offset_deg, src)"""
    if zero != 0:
        mn = wrap(zero + OFF_MIN)
        mx = wrap(zero + OFF_MAX)
        src = "offset branch (calibrated: [zero-1033, zero+787])"
    else:
        mn = deg_to_raw(ELSE_MIN_DEG)
        mx = deg_to_raw(ELSE_MAX_DEG)
        src = "else fallback (uncalibrated / after `zr`: [180deg, 340deg])"
    offset = raw_to_deg(zero) if zero != 0 else 180.0
    return mn, mx, offset, src


def relhome(raw, offset):
    """servo raw -> rel-home 关节角(度)"""
    return raw_to_deg(raw) - offset


def main():
    print("=" * 78)
    print("J5 limits vs calib_zero_raw[4]  (SERVO_OFFSET[4] = zero/4095*360)")
    print("=" * 78)
    print("%-10s %-9s %-9s %-9s %-8s %-22s %s"
          % ("zero[4]", "min_raw", "max_raw", "offset", "span", "legal rel-home deg",
             "source"))
    print("-" * 78)

    for zero in (0, 2048, 3081, 2022, 2331, 1567):
        mn, mx, off, src = j5_limits(zero)
        lo, hi = relhome(mn, off), relhome(mx, off)
        tag = ""
        if zero == 2048:
            tag = "  <- 'ideal center' (zi says 2048 is healthy)"
        elif zero == 3081:
            tag = "  <- makes offset branch == [180,340]"
        elif zero == 0:
            tag = "  <- after `zr` / never calibrated"
        print("%-10d %-9d %-9d %-9.1f %-8.0f [%+7.2f, %+7.2f]%s"
              % (zero, mn, mx, off, mx - mn if mx > mn else mx + 4096 - mn, lo, hi, tag))
        if zero in (0, 2048):
            print("%-10s %s" % ("", src))

    print("-" * 78)
    print()

    # 判据: 主臂 J5 的合法工作区间 (offset 分支的设计意图)
    design_lo, design_hi = relhome(wrap(OFF_MIN + 4096), 0.0), OFF_MAX * DEG_PER_RAW
    print("Offset-branch design window  = [%+.2f, %+.2f] deg  (span %.2f)"
          % (-1033 * DEG_PER_RAW, 787 * DEG_PER_RAW, (787 + 1033) * DEG_PER_RAW))
    print("Else-fallback window         = [%+.2f, %+.2f] deg  (span %.2f)"
          % (ELSE_MIN_DEG - 180.0, ELSE_MAX_DEG - 180.0, ELSE_MAX_DEG - ELSE_MIN_DEG))
    print("Mismatch between the two     = %.2f deg  (the whole J5 mystery)"
          % (0.0 - (-1033 * DEG_PER_RAW)))
    print()

    # J5 机械卡区 (arm_config.h:118): servo 77~174 deg 禁止进入
    print("Servo-degree check (card-protect zone is 77..174 deg):")
    for zero in (0, 2048, 3081):
        mn, mx, off, _ = j5_limits(zero)
        lo_servo, hi_servo = raw_to_deg(mn), raw_to_deg(mx)
        hit = "ENTERS CARD ZONE" if lo_servo < 174.0 and hi_servo > 77.0 else "clear"
        print("  zero=%-5d servo window = [%6.1f, %6.1f]  -> %s"
              % (zero, lo_servo, hi_servo, hit))

    print()
    print("How to read this:")
    print("  * zero != 0  => J5 legal rel-home is [%+.2f, %+.2f]."
          % (-1033 * DEG_PER_RAW, 787 * DEG_PER_RAW))
    print("  * zero == 0  => J5 legal rel-home is [%+.2f, %+.2f]." % (0.0, 160.0))
    print("  The clamp in cmdStreamJoints() is SILENT: clamp_raw() then falls")
    print("  through and reports success, so a clamped J5 prints NOTHING.")
    print("  Symptom is exactly 'J5 lags / won't follow' with all other axes fine.")


if __name__ == "__main__":
    main()
