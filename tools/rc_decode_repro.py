#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rc_decode_repro.py — 离线复刻 arm_rc.c 的 RC PWM 解码逻辑并自证。

为什么要有这个脚本
------------------
`Core/Src/arm_rc.c` 的中断里用的是**"按区间长度分段"**，而不是常见的
"记住上次是上升沿还是下降沿"：

    dt = CNT - prev_CNT
    dt >= 5ms   -> 刚才那段是低电平段(丢弃)
    0.4~3ms     -> 刚才那段是高电平段 = 一个有效脉宽

两种写法的差别**只在丢沿时才暴露**，而丢沿在现场是必然会发生的
（ISR 迟延、CCxOF 溢出、毛刺被输入滤波器挡掉一个边沿）。
本脚本把两种写法都跑一遍，用**合成波形**证明：

  ① 不丢沿时两者都对；
  ② 丢 **一个**边沿(奇数个, 打乱奇偶性)时:
       - 本工程写法: 只少 1 个样本, 其余全对  ✅ 自恢复
       - "记住极性"写法: 从那一刻起**永久错位**, 而且量出来的还是
         "看起来合理的整数"(≈20ms 的低电平宽度) ⇒ 现场极难发现 ❌

⚠ 这不是"示意代码", 算法与 arm_rc.c 的 ISR 逐行对应
  (含 16 位 CNT 的无符号回绕相减)。

用法:
    py -3 tools/rc_decode_repro.py
退出码 0 = 两种预期都成立; 非 0 = 有一项不符(说明有人改了解码逻辑)。
"""
import sys

# ---- 与 arm_rc.h 保持一致的常量 -------------------------------------------
W_MIN    = 400     # ARM_RC_W_MIN_US    有效脉宽下限
W_MAX    = 3000    # ARM_RC_W_MAX_US    有效脉宽上限
LOW_MIN  = 5000    # ARM_RC_LOW_MIN_US  >= 这个长度一定是低电平段
CNT_MASK = 0xFFFF  # TIM4 ARR = 0xFFFF, PSC = 83 -> 1 tick = 1us, 每 65.5ms 回绕

FRAME_US = 20000   # RC 帧周期 20ms (50Hz)


class Ours:
    """arm_rc.c ISR 的逐行复刻: 按区间长度分段。"""

    def __init__(self):
        self.prev = 0
        self.have_prev = False
        self.frames = []
        self.edges = 0

    def edge(self, t_us):
        cnt = t_us & CNT_MASK          # 硬件 CNT 只有 16 位, 自然回绕
        self.edges += 1
        if not self.have_prev:
            self.prev = cnt
            self.have_prev = True
            return
        dt = (cnt - self.prev) & CNT_MASK   # 无符号回绕相减
        self.prev = cnt
        if dt >= LOW_MIN:                    # 低电平段
            return
        if dt < W_MIN or dt > W_MAX:         # 毛刺 / 非 RC 波形
            return
        self.frames.append(dt)


class Naive:
    """反例: 记住"上次是上升还是下降沿"。
    故意保留, 用来演示它为什么会**永久错位**。"""

    def __init__(self):
        self.rise = None
        self.up = True
        self.frames = []

    def edge(self, t_us):
        if self.up:
            self.rise = t_us
        elif self.rise is not None:
            self.frames.append(t_us - self.rise)
        self.up = not self.up


def synth(widths):
    """生成边沿时刻序列: 每帧 = 上升沿, 下降沿。返回 (边沿列表, 脉宽列表)。"""
    edges, t = [], 0
    for w in widths:
        edges += [t, t + w]
        t += FRAME_US
    return edges, list(widths)


def run(cls, edges):
    o = cls()
    for e in edges:
        o.edge(e)
    return o.frames


def main():
    widths = [1500, 1500, 1000, 2000, 1480, 1520, 1750]
    edges, exp = synth(widths)
    fails = []

    # ---- ① 不丢沿: 两者都应当完全正确 -----------------------------------
    ours_ok = run(Ours, edges)
    naive_ok = run(Naive, edges)
    print("① 不丢沿")
    print("     波形      : %s" % exp)
    print("     ours      : %s  %s" % (ours_ok, "OK" if ours_ok == exp else "FAIL"))
    print("     naive     : %s  %s" % (naive_ok, "OK" if naive_ok == exp else "FAIL"))
    if ours_ok != exp:
        fails.append("① ours 在理想波形下就不对 ⇒ arm_rc.c 的解码逻辑被改坏了")
    if naive_ok != exp:
        fails.append("① naive 在理想波形下不对(不该发生)")

    # ---- ② 丢 1 个下降沿(奇数个沿 ⇒ 打乱奇偶性) -------------------------
    #      丢掉索引 7 = 第 4 帧的下降沿
    dropped = edges[:7] + edges[8:]
    exp2 = [w for i, w in enumerate(widths) if i != 3]     # 只应少第 4 帧那一个样本

    ours_d = run(Ours, dropped)
    naive_d = run(Naive, dropped)

    print("\n② 丢掉 1 个下降沿 (第 4 帧的下降沿)")
    print("     ours      : %s" % ours_d)
    print("     期望      : %s  %s" % (exp2, "OK 自恢复" if ours_d == exp2 else "FAIL"))
    if ours_d != exp2:
        fails.append("② ours 没能自恢复 ⇒ 解码逻辑被改成了依赖沿极性的写法")

    corrupted = any(x > LOW_MIN for x in naive_d)
    print("     naive     : %s" % naive_d)
    print("     判据      : %s" % ("已永久错位(出现 >5ms 的'脉宽')" if corrupted
                                  else "本样本未暴露"))
    if not corrupted:
        fails.append("② naive 应当暴露永久错位(样本失效? 请检查 synth/dropped)")

    print("\n" + "-" * 62)
    if fails:
        print("结果: FAIL")
        for f in fails:
            print("  - %s" % f)
        return 1
    print("结果: PASS —— '按区间长度分段' 在丢沿后可自恢复; '记住沿极性'会永久错位。")
    print("      ⇒ 不要为了'省一次比较'把 arm_rc.c 的 ISR 改成记住极性的写法。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
