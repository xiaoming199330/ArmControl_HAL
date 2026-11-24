#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""pc_relay.py — 主从臂"PC 中转"转发线（官网 LeRobot 路线）

官网 Seeed SO-100/101 + LeRobot 的主从遥操里，两条臂之间**没有任何电气连接**：
    主臂 ──USB串口──► PC(teleop_loop) ──USB串口──► 从臂
PC 每拍读主臂位置、写从臂目标。本脚本就是那根"线"——把主臂固件吐出的
位置行原样转给从臂固件。

数据格式（两端固件已实现，见 docs/官网主从遥操架构对照_2026-09-15.md）
    主臂: `sp <Hz>`  → 按节拍打印一行   #P a1 a2 a3 a4 a5 a6
    从臂: 认这一行（`#P` 命令）→ 直写关节角（与 0x13 流帧同一条通路）
    aN = 关节角(rel-home, 度) × 10 的定点整数，与 0x13 帧同编码

用法
    # 1) 先看有哪些串口
    py tools/pc_relay.py --list

    # 2) 转发（两块板各接一个 USB-TTL；端口按实际改）
    py tools/pc_relay.py --master COM11 --slave COM12

    # 3) 想省事，让脚本自动敲开机命令（主臂 mf 0 + sp <--hz>，从臂 sf 1）
    py tools/pc_relay.py --master COM11 --slave COM12 --start          # 默认 sp 60
    py tools/pc_relay.py --master COM11 --slave COM12 --start --hz 100 # 更低延迟
    #   ⚠ `mf 0` 是 `--start` 自带的、且**必须有**（见下 3b 的说明）；

    # 3b) 只想补个别命令（不开 --start），用 --master-cmd / --slave-cmd（可重复）：
    py -3 tools/pc_relay.py --master COM22 --slave COM11 --master-cmd "mf 0" --master-cmd "sp 60" --slave-cmd "ec 0"
    #
    #   ⚠ **换行续行符跟着终端走，别照抄别人的**（2026-09-17 现场踩到）：
    #       PowerShell : 反引号 ` 作行尾（后面**不能有空格**），或干脆写成一行
    #       cmd.exe    : ^ 作行尾
    #     PowerShell 里用 ^ 会报 `一元运算符"--"后面缺少表达式` /
    #     `意外的标记"start"` —— 因为 `--` 在 PS 里是自减运算符，被当成算术表达式解析。
    #     最稳的写法永远是**一行到底**。
    #   ↑ 这是**唯一**能往主臂控制台送字的方式：Windows 串口独占，脚本占着口，
    #     你没有第二个窗口去手敲 `sp`/`mf`。
    #   ⚠ **`mf 0` 不能省**。`sp`(控制台流) 与 `mf`(自有直连链路) 在固件里是两路
    #     独立输出；不静默直连链路，主臂每拍要读两遍舵机，控制台流帧率实测从
    #     ~65 Hz 掉到 ~46 Hz。帧率一低，从臂那条固定的「每帧 25°」跳变判据就更容易
    #     被触发 → 整帧拒绝 → 自锁 → 看门狗 → 手感上就是「卡住」。
    #   ℹ 想核实帧率与丢拍：加 `--echo --master-cmd "ms"`，主臂会打印
    #     `Master forwarding: ON/OFF` / `Console stream: ON (16 ms, 62 Hz)` /
    #     `lines=N skipped=M`。lines ÷ 用时 = 真实帧率；skipped 大 = 读舵机在失败。

    # ⚠ `--start` 会**覆盖**你在主臂控制台手敲的 sp 档位（2026-09-15 现场踩到：
    #    手敲 `sp 100` 再带 --start 启动 → 被脚本发回的 `sp <--hz>` 顶回去 →
    #    现象是"主臂怎么设都固定在 --hz 那个值"）。两种正确做法：
    #      a) 打开机命令交给脚本：把档位写进 --hz（默认 60）
    #      b) 保留手敲的档位：加 --hz 0（脚本不发 sp，只发 mf 0 / sf 1）

依赖
    pip install pyserial

⚠ 本机 Windows PowerShell 里 `python` **不在 PATH**（脚本本体没问题），
  敲 `python tools/pc_relay.py ...` 会得到：
      python : 程序"python"无法运行: 系统找不到指定的文件。
  改用 `py` 启动器即可。⚠ **必须带 `-3`**（2026-09-16 实测：裸 `py` 会落到一个
  没装 pyserial 的解释器上，报"缺少 pyserial"；`py -3` = 系统 Python 3.12 +
  pyserial 3.5，已验证可跑）：

    py -3 tools/pc_relay.py --list
    py -3 tools/pc_relay.py --master COM11 --slave COM12 --start

  用绝对路径则不必切目录（脚本内部不依赖 cwd）：
    py -3 "C:\...\ArmControl_HAL（1）\tools\pc_relay.py" --list
  （工程路径带全角括号 `（1）`，任何位置都要整体加双引号。）

排障（现场最费时间的一类问题：状态行一直「转发 0 行」）
    **先看「主臂收到 N B」这一栏**，它把两种完全不同的故障分开了：

    主臂收到 0 B         → 分两种，**先排除第一种**（2026-09-15 现场误判过一次）：
                          (a) **压根没人给主臂发过命令**。主臂的横幅 + `> ` 提示符
                              只在开机那一次打印，待命时主循环一条都不吐；而本脚本
                              对主臂口是纯只读。所以"脚本启动时主臂早开完机了"这种情况
                              下，0 B 是**必然、正常**的，不是故障。
                              自检：你是不是没带 `--start` 也没带 `--master-cmd`？
                              修法（Windows 串口独占，你没法再开助手手敲）：
                                  ... --start --hz 60
                                  ... --master-cmd "sp 60"
                              发一条 `sp` 出去，主臂立刻开始吐，mrx 马上 >0。
                          (b) 上面那条排除了，才是**端口选错**（或板子没跑起来/线没接）。
                              这时不要怀疑固件、不要怀疑 `sp`。
                              用只读嗅探器找真身（一个字节都不写，安全）：
                                  py tools/serial_sniff.py --list
                                  py tools/serial_sniff.py COM8 COM11 COM12 COM22 --secs 6
                              打出「✅ 主臂位置流」的那个口才是 --master。
                              注意：串口助手/别的脚本占着的口嗅探器会报"打不开" —— 那也
                              是线索，说明你正在读的口是另一条线。
                              还有一条零成本自查：**按一下主臂 RESET**，听有没有两短声
                              蜂鸣（固件在 main() 开头 BuzzerBeep×2 = "代码在跑"）。
                              没声音 ⇒ 板子停在 bootloader（BOOT0 还高/漏了烧完那次
                              RESET），它在收到 0x7F 之前一个字节都不吐。
    主臂收到 >0 B 但无 #P → 主臂没敲 `sp <Hz>`，或被示教/单键模式吃掉输入。
                          这种会看到提示符、日志（「其它」计数在涨）。

    ⚠ 本脚本**从不打印** `#P` 行本身（那会刷屏）。所以：如果你在屏幕上看见了
      `#P ...` 而本脚本的计数却是 0，那串 `#P` **一定来自另一条串口路径**
      （板载 USB 串口 / 串口助手窗口），不是这个脚本读到的 —— 拿它当依据会跑偏。

降延迟（按收益从大到小，前两条都不用改固件）
    整条链路的延迟 ≈ 主臂采样量化 + 采样跨度 + 线路传输 + 驱动批处理 + 从臂下发。
    逐项拆开（`sp 30` / 115200 下的量级）：

    ① 主臂采样周期 `sp <Hz>`      ~16.5 ms(均值)  ← **最大的一项，直接提频最有效**
                                     sp 30→60 砍掉约 16 ms。主臂敲 `sp 60`
                                     （上限 100；5 ms 一个周期为下限）。
                                     ⚠ 手敲 `sp 60` 就别再带 `--start` —— 它默认发
                                     `sp 30`，会把你的档位顶回去；要么用 `--start --hz 60`。
                                     提完看本脚本「当前 Hz」确认真的到了 ——
                                     到不了说明主臂主循环被占满，见 ④。
    ② USB 串口驱动 LatencyTimer   **最高 ~15 ms**  ← 最隐蔽的一项，且是常数偏移
                                     设备管理器 → 端口(COM & LPT) → 对应适配器 →
                                     属性 → 端口设置 → 高级 → Latency Timer，
                                     从默认 16 ms 改成 **1 ms**。
                                     ⚠ CP210x / FTDI 的驱动默认就是 16 ms；它是"攒够
                                     16 ms 或缓冲满才交给应用"，所以每帧都被压后。
                                     CH340 驱动多数不暴露该项，那就换 CP210x/FTDI 用。
                                     本项与波特率无关，改 baud 不会自动解决。
    ③ cprint 阻塞发送 + 线路传输  ~4.0 ms         ← 46 B @115200，且是**阻塞**发送
                                     提控制台波特率到 460800 → 1.0 ms。
                                     需两端固件 `make CBAUD=460800` 重建烧录，
                                     本脚本同步 `--baud 460800`（三处必须同值）。
    ④ 主臂主循环占用（决定 ① 的上限）
                                     每帧固定开销 ≈ 6 轴 ×(2 次读 + 1 ms 投票间隔)
                                     ≈ 6 ms 采样跨度 + ③ 的 4 ms 阻塞发送 ≈ 10 ms
                                     ⇒ 理论上限 ~100 Hz，实际 60~80 Hz 就见顶。
                                     前提：先 `mf 0` 静默直连转发，否则两路各读一遍
                                     舵机，开销翻倍、能到的频率腰斩。
    ⑤ 从臂下发                    ~1-2 ms        2 次 SyncWrite @1 Mbps，已是最优
                                     （stream_enter 已把 GOAL_SPEED 写 0=max）

    不要动的（都是安全机制，动了是拿保护换几十微秒）：
      - STREAM_MAX_JUMP_DEG=25°/帧        (30 Hz 下 = 750°/s，远超臂能力，不是瓶颈)
      - read_pos_vote 的 1 ms 采样间隔    (它就是 ④ 里的 6 ms 来源，但同时是抗总线
                                           噪声/帧跳变的判据)
      - STREAM_TIMEOUT_MS=500             (断流看门狗，与延迟无关)

    先看本脚本状态行的两栏再决定动哪个：
      当前 Hz 没到 sp 设的值      → 瓶颈在 ④，提 baud(③) 或降 sp
      积压 持续 > 0               → 瓶颈在 PC/从臂侧，位置在排队
      以上都正常但还是"手感慢"    → 几乎一定是 ②，去改 LatencyTimer

现场顺序（两臂必须都摆好、且都在非示教模式）
    主臂: mf 0        # 静默自有的 0x13 直连链路，避免舵机被读两遍
    主臂: sp 60       # 开始吐位置行（想要什么频率就敲什么，1~100）
    从臂: sf 1        # 跟随门闩（上电默认就是 ON，敲过 sf 0 才需要）
    然后跑本脚本。停：主臂 `sp 0`，或 Ctrl+C 退出脚本。
    ⚠ 手敲了 sp 就别再带 `--start`，否则脚本会重发自己的 --hz 覆盖掉；
      想省事就干脆全交给脚本：`--start --hz 60`。

遥操中录轨迹（示教）—— 现场热键
    需求：主臂拖、从臂跟，同时把这段动作**录下来存盘**，以后从臂能自己重放。
    做法：从臂开"跟随录制"（固件 `teach rec`，只读不碰舵机），本脚本用热键触发。

    从臂准备（脚本起来之前手敲一次，或加 `--slave-cmd "teach rec"` 自动开）：
        从臂: ec 0          # 关控制台回显，省带宽（可选但强烈建议）
        从臂: sf 1          # 跟随门闩
        从臂: teach rec     # 开始跟随录制（不松力、不改 PID —— 只采样）

    然后跑脚本，**运行中按单键**（不用回车，不用切窗口）：

        r   从臂 `teach rec`   开始/重新开始录制
        x   从臂 `teach exit`  停止录制（保留录音，不丢）
        t   从臂 `teach list`  看当前录了多少样本 / 多少秒
        w   从臂 `save <名字>` 存进从臂 Flash（`--teach-name` 给名字，默认自动命名）
        l   从臂 `list`        列出已存文件
        s   主臂 `sp 0`        暂停位置流（从臂原地锁定）—— **存盘前按这个**
        g   主臂 `sp <--hz>`   恢复位置流（继续遥操）

    推荐顺序：拖 → `s` 停流 → `w` 存盘 → `g` 续拖（或 `x` 收尾）。
    ⚠ 为什么存盘前要 `s`：`save` 是把整段写进从臂内部 Flash，**阻塞 1~3 秒**
      （首存/满 80 s 的大录音最坏 ~5 s，可能附带扇区擦除）。
      这期间主循环不转，位置流照旧往从臂串口灌 —— 从臂 128 B 接收环装不下
      （60 Hz × 约 40 B/行 ⇒ 只够 3 行）⇒ 帧被丢弃；主循环恢复后看门狗发现
      已断流 >500 ms，会打 `Stream watchdog ... -> hold` 并重新锚定当前位置作
      基准。若你在这 1~3 秒里继续拖主臂，那段位移会被整段跳过；J5 这类手腕轴
      还可能因 `每帧 25°` 跳变判据被连帧拒绝 → 自锁。停流再存就没有这回事。

    从臂可录上限：4000 点 × 20 ms ≈ **80 秒**（RAM 录音缓冲，超过就停记并提示）。
    存盘后回放（脚本可以先退出）：
        从臂: sf 0          # 先停跟随，否则回放会被流顶回去
        从臂: play rec1     # 回放刚存的那条（`play rec1 loop` 循环）

    ⚠ 回放时固件会**自动把流式跟随关掉**（回放与 `#P` 流抢同一个 goal），
      放完想继续遥操要重新 `sf 1`。
    ⚠ 录下来的是**从臂实际走出来的**轨迹（已过限位 clamp 与跳变守卫），
      所以它对从臂是 1:1 可复现的；但它不等于主臂的原始轨迹。

⚠ 安全
    - 从臂一收到流就会跟动：**先摆齐两臂再启动**，手离开从臂行程。
    - 跳变保护仍在从臂侧生效（首帧以从臂当前位置为基准，之后限单步跳变），
      所以即使主臂姿态不同也不会瞬间飞车，但请仍按上一条做。
    - 从臂急停硬手段仍是 PE11；脚本退出不会让从臂失力（它有位置环自锁）。
"""

import argparse
import re
import sys
import time

try:
    import msvcrt            # Windows: 单键热键 (kbhit/getch), 不吃 Ctrl+C
except ImportError:
    msvcrt = None
try:
    import select as _select
except ImportError:
    _select = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write(
        "缺少 pyserial。安装：\n"
        "    python -m pip install pyserial\n"
    )
    sys.exit(2)

FIELDS = 6

# 从臂侧"必须立刻看见"的诊断关键词。这些行直接回答"为什么某个轴不跟"：
#   Stream reject: J5 jump 43 deg (> 25)   —— 整帧被跳变判据丢掉（J5 是手腕, 人手最容易甩快）
#   Stream reject: pose unsafe             —— 桌面/立柱判据拒绝
#   Stream watchdog: no cmd for 500 ms     —— 断流
#   Stream mode ON / OFF                   —— 进/出流模式
# 旧版主循环**只读主臂口**, 从臂这几行永远不被人看见 ⇒ 出问题只能靠猜。
#   Feedback: J1=.. J5=..   —— `pv <ms>` 打出的从臂实读角。**不需要 --echo** 就显示：
#   它正是判断"某轴是否被人卡在固定值"的第一手数据，不该藏在调试开关后面。
SLAVE_HOT = re.compile(
    rb"reject|unstable|watchdog|clamp|fail|unreadable|error|stream mode|Feedback:"
    rb"|record|teach|samples",       # 跟随录制的开/停/完整性回显 (2026-09-16)
    re.I)


def make_key_reader():
    """返回 read_key() -> str|None（小写），非阻塞。

    Windows 用 msvcrt；POSIX 用 select；两者都不可用（stdin 被重定向、
    没有真实控制台）时退回一个恒返回 None 的桩 —— **绝不能因此影响转发**。
    方向键等返回 b'\\x00'/b'\\xe0' 引导字节，这里吃掉并忽略。"""
    if msvcrt is not None:
        def _win():
            try:
                if not msvcrt.kbhit():
                    return None
                ch = msvcrt.getch()
                if ch in (b"\x00", b"\xe0"):     # 功能键的引导字节
                    msvcrt.getch()
                    return None
                return ch.decode("ascii", "ignore").lower()
            except Exception:
                return None
        return _win
    if _select is not None and sys.stdin is not None and sys.stdin.isatty():
        def _posix():
            try:
                r, _, _ = _select.select([sys.stdin], [], [], 0)
                return sys.stdin.read(1).lower() if r else None
            except Exception:
                return None
        return _posix
    return lambda: None


def run_selftest() -> int:
    """离线自检：只验解析/过滤逻辑，不碰硬件。现场开跑前可以先跑一遍。"""
    cases = [
        (b"#P 12 -1017 929 29 72 -484\r\n", "pos", "正常 6 字段"),
        (b"#P -3600 3600 0 0 0 0\r\n",      "pos", "极值"),
        (b"#P 12 -1017 929 29 72\r\n",      "bad", "少一个字段"),
        (b"#P 1 2 3 4 5 6 7\r\n",           "bad", "多一个字段"),
        (b"> ",                             "other", "提示符"),
        (b"  Master mode: READ-ONLY\r\n",   "other", "日志行"),
        (b"#P\r\n",                         "bad", "只有前缀"),
        (b"",                               "other", "空行"),
    ]
    ok = True
    for raw, want, why in cases:
        got, _ = parse_line(raw)
        flag = "ok " if got == want else "FAIL"
        if got != want:
            ok = False
        print("  [%s] %-40r %-6s (期望 %s / %s)" % (flag, raw, got, want, why))

    # 模拟一轮完整的"主臂输出流" -> 统计应转发几行
    stream = [
        b"> ",
        b"  Console position stream ON: 30 Hz (33 ms/line)\r\n",
        b"#P 10 20 30 40 50 60\r\n",
        b"#P 11 21 31 41 51 61\r\n",
        b"#P 12 22 32 42 52\r\n",
        b"#P 13 23 33 43 53 63\r\n",
    ]
    fwd = sum(1 for l in stream if parse_line(l)[0] == "pos")
    exp = 3
    print("  转发计数模拟: 得到 %d, 期望 %d -> %s" % (fwd, exp, "ok" if fwd == exp else "FAIL"))
    ok = ok and (fwd == exp)

    print("\n自检结果: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def cmd_list():
    ports = list(list_ports.comports())
    if not ports:
        print("没有发现任何串口。检查 USB-TTL 是否插好、驱动是否装了。")
        return 0
    print("可用串口：")
    for p in ports:
        print("  %-10s %s" % (p.device, p.description or ""))
    print("\n两块板各接一个 USB-TTL，会看到两个不同端口（如 COM11 / COM12）。")
    return 0


def parse_line(raw: bytes):
    """返回 (kind, payload)
    kind: 'pos'  = 可转发的 #P 行（恰好 6 个值）
          'bad'  = 以 #P 开头但字段数不对（含被串口缓冲截断的半行）
          'other'= 其它控制台输出（日志、提示符等）

    ⚠ 截断是"失败关闭"的：半行会落进 bad 被丢弃，绝不会被当成合法位置下发。
      串口 readline 超时切分在最坏情况下只影响某一拍的数值，下一拍 33ms 后
      自然补上 —— 位置流本来就不需要重传。
    """
    s = raw.strip()
    if not s.startswith(b"#P"):
        return ("other", s)
    parts = s.split()
    if not parts or parts[0] != b"#P" or len(parts) != FIELDS + 1:
        return ("bad", s)
    return ("pos", s)


def main():
    ap = argparse.ArgumentParser(
        description="主从臂 PC 中转转发线（把主臂 #P 位置行转给从臂）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--master", help="主臂控制台串口，如 COM11")
    ap.add_argument("--slave", help="从臂控制台串口，如 COM12")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200，两端一致）")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="转发限速（默认 0=不限速，原样转发；卡顿时可设 30）")
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="主臂超过这么多秒没有位置行就告警（默认 1.0 s）")
    ap.add_argument("--start", action="store_true",
                    help="启动时自动敲：主臂 mf 0 + sp <--hz>，从臂 sf 1")
    ap.add_argument("--hz", type=int, default=60,
                    help="`--start` 时发给主臂的 `sp <Hz>`（默认 60 —— **别低于 60**；"
                         "传 0 = 不碰主臂的 sp, 保留你在控制台手敲的档位）"
                         "。⚠ 从臂的跳变判据是「每帧 25°」的固定值 ⇒ 它实际是一条速率"
                         "上限 25°×fps：60Hz→1500°/s（正常拖拽过得去）；"
                         "30Hz→只有 750°/s，正常手速就会被整帧拒绝 → 自锁 → 看门狗 → 「卡住」。")
    ap.add_argument("--master-cmd", action="append", default=[],
                    help="启动时发给主臂的命令（可重复）")
    ap.add_argument("--slave-cmd", action="append", default=[],
                    help="启动时发给从臂的命令（可重复）")
    ap.add_argument("--teach-name", default=None,
                    help="热键 w 存盘时用的文件名（默认不带名字 -> 从臂自动命名 rec1/rec2…）")
    ap.add_argument("--no-keys", action="store_true",
                    help="关掉运行中的单键热键（r/x/t/w/l 走从臂，s/g 走主臂），避免误触")
    ap.add_argument("--echo", action="store_true", help="把主臂的其它输出也打印出来（调试）")
    ap.add_argument("--dry-run", action="store_true", help="只解析不写串口")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--selftest", action="store_true", help="离线自检解析逻辑（不碰硬件）后退出")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()
    if args.list:
        return cmd_list()
    if not args.master or not args.slave:
        ap.error("必须同时给 --master 和 --slave（先用 --list 看端口）")
    if args.master == args.slave:
        ap.error("--master 与 --slave 不能是同一个串口：一块 USB-TTL 只能接一块板，"
                 "本方案需要两个 USB-TTL 适配器")

    try:
        sm = serial.Serial(args.master, args.baud, timeout=0.05)
    except Exception as e:
        sys.stderr.write("打不开主臂串口 %s: %s\n" % (args.master, e))
        return 1
    try:
        ss = serial.Serial(args.slave, args.baud, timeout=0.05)
    except Exception as e:
        sm.close()
        sys.stderr.write("打不开从臂串口 %s: %s\n" % (args.slave, e))
        return 1
    # 从臂口在主循环里**非阻塞**轮询读（只为了看它的诊断输出）。
    # ⚠ 不能留 0.05 s 超时：那会给每一拍都塞进 50 ms 阻塞, 转发的延迟直接翻倍。
    ss.timeout = 0.0

    def send(port, text):
        # ⚠ `--dry-run` 必须在**这里**就拦掉，只在转发处拦是不够的。
        #   2026-09-17 事故：用 `--dry-run` 试跑参数，开机命令照样发到了真臂上，
        #   其中一条 `teach rec` 走进 cmdTeachEnterRec() -> teach_mode_prepare()，
        #   而后者会执行 `teach_rec_cnt = 0U` ⇒ **把从臂 RAM 里 80 s 未存盘的
        #   录音静默清空了**。教训：任何"只测参数"的路径都不能有改变现场状态的
        #   副作用；安全开关要放在最底层的出口，而不是调用点上。
        if args.dry_run:
            print("  (dry-run 未发送) -> %s : %s" % (port.port, text))
            return
        port.write((text + "\r\n").encode("ascii"))
        port.flush()
        print("  -> %s : %s" % (port.port, text))

    print("PC 中转已启动")
    print("  主臂 : %s @ %d" % (args.master, args.baud))
    print("  从臂 : %s @ %d" % (args.slave, args.baud))
    if args.dry_run:
        print("  ⚠ --dry-run：只解析参数、**一条命令都不会发**（热键也不会发）。")
        print("     想真跑就去掉 --dry-run（并且先确认从臂 buffer 里没有未存盘的录音）。")

    # ---- 现场热键：遥操中录轨迹（从臂侧 teach rec / save / list）----------
    #  为什么要有它：Windows 串口被本脚本独占，你**没有第二个窗口**去从臂
    #  控制台敲 `teach rec`；而录制必须能在"臂正在跟动"的当下开始/结束。
    read_key = (lambda: None) if args.no_keys else make_key_reader()
    save_cmd = ("save " + args.teach_name) if args.teach_name else "save"
    sp_hz = args.hz if args.hz > 0 else 60
    #  值 = (**要发的命令列表**, 说明)。一条热键发多条命令是为了 `r`：
    #  固件里 `teach rec` 在"已经在录"时只回 `Already recording (N samples)`、
    #  **不会清缓冲** —— 清缓冲只发生在 `teach_mode_prepare()` 全新进入时，
    #  而它在"已 active"分支里就 return 了。⇒ 想让 `r` 真做到"重录"，
    #  必须先 `rec clear`。两条都在示教白名单里（`rec`/`teach` 都在），不会被
    #  auto-exit 顶掉。
    #  ⚠ 2026-09-17 现场踩到：上一轮录音没存就 Ctrl+C，从臂**仍在录**；
    #    下一次 `r` 报 "Already recording (3964 samples)"，本次拖拽只录进去
    #    36 点（0.7 s）就满了 —— 而操作者以为录的是这一遍。
    hot_keys = {
        "r": (["rec clear", "teach rec"], "从臂: 清空缓冲并重新开始录制 (只采样, 不松力)"),
        "x": (["teach exit"],             "从臂: 停止录制 (录音保留)"),
        "t": (["teach list"],             "从臂: 看已录样本数/时长"),
        "w": ([save_cmd],                 "从臂: 存盘 (Flash)"),
        "l": (["list"],                   "从臂: 列出已存文件"),
    }
    # 主臂侧热键（2026-09-17）：存盘写内部 Flash 是**阻塞 1~3 s** 的，
    # 期间从臂控制台 128 B 接收环装不下 60 Hz 的位置流 ⇒ 丢帧；主循环恢复后
    # 看门狗超 500 ms 会 `Stream watchdog -> hold` 并重锚基准，那 1~3 s 里主臂
    # 的位移就被整段跳过（J5 还可能被 `每帧 25°` 判据连帧拒 → 自锁）。
    # ⇒ 存盘/收尾前先按 `s` 把 `sp` 关掉，存完按 `g` 恢复。
    hot_keys_master = {
        "s": (["sp 0"],              "主臂: 停位置流 (从臂原地锁定) —— 存盘前按"),
        "g": (["sp %d" % sp_hz],     "主臂: 恢复位置流 (继续遥操)"),
    }
    if not args.no_keys:
        print()
        print("运行中按键（不用回车）:")
        for k in ("r", "x", "t", "w", "l"):
            print("   %s  -> 从臂 %-24s %s"
                  % (k, "`%s`" % " + ".join(hot_keys[k][0]), hot_keys[k][1]))
        for k in ("s", "g"):
            print("   %s  -> 主臂 %-24s %s"
                  % (k, "`%s`" % " + ".join(hot_keys_master[k][0]), hot_keys_master[k][1]))
        print("   h  -> 重印这份热键表")
        print("   ⚠ 从臂要先 `teach rec` 才在录；上限 ~80 s（4000 点 × 20 ms）。")
        print("   ⚠ 顺序：拖 → `s` 停流 → `w` 存盘 → `g` 续拖 / `x` 收尾。")
        print("      （存盘要写 Flash 1~3 s，流不停会被从臂 128 B 环丢帧 + 看门狗重锚）")
        print("   ⚠ 回放前记得从臂 `sf 0`。")
        print("   ⚠ `r` 要先看从臂回的那行：若写 `Already recording` 说明上一轮没存，"
              "按 `w` 存掉或补 `r` 重录。")

    # 本脚本是否**主动给主臂发过**命令。主臂口的告警 A 要靠它区分
    # "端口选错了" 和 "谁都不说话当然没声音"(2026-09-15 现场踩到, 见告警 A 注释)。
    master_told = False

    if args.start:
        print("自动开机命令：")
        send(sm, "mf 0")
        master_told = True
        if args.hz > 0:
            # ⚠ 这一条会**覆盖**你在主臂控制台手敲的 sp 档位 (2026-09-15 踩到):
            #   手敲 `sp 100` 再带 --start 启动, 会被这里的 `sp <--hz>` 顶回去,
            #   现象就是"主臂怎么设都是 --hz 那个值"。要别的档位就改 --hz。
            #   别把 --hz 调到 60 以下: 从臂的「每帧 25°」判据在低帧率下会误伤
            #   (30 Hz 时只允许 750°/s, 正常手速就会被整帧拒 → 自锁 → 卡住)。
            send(sm, "sp %d" % args.hz)
        else:
            print("  (--hz 0：不发 sp，沿用你在主臂控制台手敲的档位)")
        time.sleep(0.2)
        send(ss, "sf 1")
        time.sleep(0.2)
        if args.hz > 0:
            print("  ⚠ 上面这条 `sp %d` 会覆盖你手敲过的 sp 值 —— 想改成别的档位"
                  "直接改 --hz，别在手敲之后再带 --start 启动。" % args.hz)
    for c in args.master_cmd:
        send(sm, c)
        master_told = True
    for c in args.slave_cmd:
        send(ss, c)

    # ---- 帧率自伤的坑：发了 `sp` 却没先 `mf 0` (2026-09-15 现场实测) -------
    #  固件里 `sp` 与 `mf` 是**两路独立**输出，共用同一拍读法，只有"两路同拍到期"
    #  才只读一次（main.c master_service：do_fwd / do_sp）。`sp` 周期 16 ms、
    #  `mf` 周期 33 ms，二者最小公倍数 528 ms ⇒ 其余每一拍都各读一遍 6 轴。
    #  实测代价：`--master-cmd "sp 60"`（mf 留在默认 ON）→ 控制台流只有 **46 Hz**，
    #  而先 `mf 0` 再 `sp` 能跑到 **65 Hz**。帧率掉 30% 的直接后果是
    #  **同一个手速下每帧角度变化大 43%**，而从臂的跳变判据是"每帧 25°"的固定阈值
    #  ⇒ 正常拖拽就开始被整帧拒绝 → 拒帧不更新基准 → 自锁 → 500 ms 看门狗 → "卡住"。
    #  固件自己的 `sp` 帮助里就印着"建议先敲 mf 0 静默直连链路, 否则总线负载翻倍"。
    sent_txt = [c.strip().lower() for c in args.master_cmd]
    has_sp  = args.start or any(c.startswith("sp") for c in sent_txt)
    # ⚠ `--start` 自带 `mf 0`（见上面的"自动开机命令"块）⇒ 必须把它算进 has_mf0，
    #   否则用 `--start` 的正常启动会被误报成"没先发 mf 0"（2026-09-17 现场踩到，
    #   用户看到一句自相矛盾的告警，还以为自己命令写错了）。
    has_mf0 = args.start or any(c.startswith("mf") and "0" in c for c in sent_txt)
    if has_sp and not has_mf0:
        print()
        print("⚠ 提醒：你让脚本发 `sp` 了，但**没有先发 `mf 0`**。")
        print("  主臂会同时跑两路输出（自有直连链路 33 ms + 控制台流），舵机总线被读约")
        print("  1.5 倍 ⇒ 控制台流实际帧率会掉（实测 65 Hz → 46 Hz）。帧率一掉，同样的")
        print("  手速每帧角度变化就更大，从臂那条固定的『每帧 25°』跳变判据更容易被触发。")
        print("  ⇒ 要么用 `--start`（它自带 `mf 0`），要么显式加 `--master-cmd \"mf 0\"`。")
        print("  ⇒ 想核实：加 `--echo --master-cmd \"ms\"`，看主臂打印的")
        print("     `Master forwarding: ON/OFF`、`Console stream: ON (16 ms, 62 Hz)`、")
        print("     `lines=N skipped=M`。lines/用时 就是真实帧率；skipped 大 = 读舵机失败。")

    print()
    if not has_sp:
        print("提醒：主臂要先用 `mf 0` + `sp <Hz>` 才开始吐位置行（没吐就只会看到「主臂静默」告警）。")
        print("      ⚠ 脚本占着主臂口（Windows 串口独占），你**没机会**再开串口助手手敲 `sp`；")
        print("        要么带 `--start --hz 60`，要么 `--master-cmd \"mf 0\" --master-cmd \"sp 60\"`。")
    else:
        print("已按你的参数把主臂开起来了（`mf 0` + `sp`）—— 拖主臂即开始转发。")
    print("      两臂请先摆齐再启动；Ctrl+C 退出。")
    print("-" * 62)

    fwd = 0
    bad = 0
    other = 0
    mrx = 0                       # 主臂串口收到的原始字节数 —— 判"端口选错"的关键指标
    srx = 0                       # 从臂串口收到的原始字节数（诊断用）
    sother = 0                    # 从臂的其它输出行数
    sbuf = b""                    # 从臂口非阻塞读的半行缓存
    seen = {}                     # 从臂诊断去重计数
    last_pos = 0.0
    warned = False                # 收到过位置行、之后又静默
    t0 = time.time()
    next_rate_t = t0 + 2.0
    rate_mark = 0
    next_send_ok = 0.0
    next_dead_t = t0 + 3.0        # 主臂口"一个字节都没有"的首次告警时刻（之后每 10 s 复述）
    dead_hits = 0
    interval = (1.0 / args.fps) if args.fps > 0 else 0.0

    # ---- 帧间隔抖动：用来判"延迟到底出在哪一段" --------------------------
    #  USB 串口驱动的 Latency Timer 会把收到的字节**攒到时间边界**才交给应用
    #  ⇒ 表现是"帧成簇到达"：一批短间隔扎堆，然后一个长间隔补回来。
    #  所以看两个数就够：标准差(抖动) 与 最大间隔。二者同时抬高 = LT 在作祟；
    #  两者都很小 = 链路平滑，延迟来自上游(采样节拍/固件发送)。
    #  在 `sp 60`(16.7ms) 下 LT=16ms 恰好一个周期一帧、看不出簇 ⇒ 要提频到
    #  `sp 100`(10ms) 再看，簇才显形。
    gaps = []                     # 最近 N 帧的到达间隔（秒）
    last_gap_t = 0.0
    next_jit_warn = t0 + 8.0      # 抖动告警限流
    jit_warned = 0

    try:
        while True:
            # ---- 现场热键轮询（非阻塞；默认没按键就返回 None）------------
            #  放在 readline 之前：readline 最多阻塞 50 ms，所以按键延迟 ≤50 ms。
            key = read_key()
            if key:
                if key in hot_keys:
                    cmds, desc = hot_keys[key]
                    for i, c in enumerate(cmds):
                        if i:
                            time.sleep(0.05)   # 两条命令之间给固件一个解析间隙
                        send(ss, c)
                    print("     [热键 %s] %s" % (key, desc))
                elif key in hot_keys_master:
                    cmds, desc = hot_keys_master[key]
                    for i, c in enumerate(cmds):
                        if i:
                            time.sleep(0.05)
                        send(sm, c)
                        master_told = True    # 主臂口确证有流量, 免得误报"端口选错"
                    print("     [热键 %s] %s" % (key, desc))
                elif key == "h":
                    print()
                    for k in ("r", "x", "t", "w", "l"):
                        print("   %s  -> 从臂 `%s`  %s"
                              % (k, " + ".join(hot_keys[k][0]), hot_keys[k][1]))
                    for k in ("s", "g"):
                        print("   %s  -> 主臂 `%s`  %s"
                              % (k, " + ".join(hot_keys_master[k][0]), hot_keys_master[k][1]))

            line = sm.readline()
            now = time.time()

            if line:
                mrx += len(line)
                kind, payload = parse_line(line)
                if kind == "pos":
                    if interval:
                        if now < next_send_ok:
                            continue
                        next_send_ok = now + interval
                    if not args.dry_run:
                        ss.write(payload + b"\r\n")
                        ss.flush()
                    fwd += 1
                    if last_gap_t:
                        gaps.append(now - last_gap_t)
                        if len(gaps) > 240:
                            del gaps[:-240]
                    last_gap_t = now
                    last_pos = now
                    warned = False
                elif kind == "bad":
                    bad += 1
                    if bad <= 3 or bad % 100 == 0:
                        print("\n[跳过] #P 字段数不对：%r" % (payload,))
                else:
                    other += 1
                    if args.echo and payload:
                        print("\n[主臂] %s" % payload.decode("utf-8", "replace"))

            # ---- 从臂口：非阻塞读，只看它的诊断（不转发它的任何东西）----
            #  旧版主循环从不读从臂口 ⇒ `Stream reject: ...` 这类行永远无人看见，
            #  "某个轴不跟"就只能靠猜。这里把它变成可观测信号。
            try:
                avail = ss.in_waiting
            except Exception:
                avail = 0
            if avail:
                chunk = ss.read(avail)
                srx += len(chunk)
                sbuf += chunk
                while b"\n" in sbuf:
                    raw_s, sbuf = sbuf.split(b"\n", 1)
                    s = raw_s.strip()
                    if not s:
                        continue
                    sother += 1
                    txt = s.decode("utf-8", "replace")
                    if SLAVE_HOT.search(s):
                        # 拒绝/告警类：永远打出来（去重 + 限流，避免刷屏）
                        key = txt[:70]
                        seen[key] = seen.get(key, 0) + 1
                        n = seen[key]
                        if n <= 5 or n % 50 == 0:
                            print("\n[从臂] %s%s" % (txt, "" if n == 1 else "   (×%d)" % n))
                    elif args.echo:
                        print("\n[从臂] %s" % txt)

            # 告警 A：主臂口"一个字节都没收到"。
            #   ⚠ 旧注释（2026-09-15 现场被推翻）说"没敲 sp 也至少能看到提示符和日志" ——
            #     **错的**。抓 main.c 的实证：
            #       · 横幅 + `> ` 提示符只在 **main() 里开机那一次** cprint（main.c:6999~7005），
            #         以及每条命令执行完之后重打；
            #       · 待命时（不动手敲命令）主循环**一条都不打印**；
            #       · 脚本对主臂口是**纯只读**——全文只有 --start / --master-cmd 会写主臂。
            #     所以"启动脚本时主臂早开完机了 + 脚本又不发命令" ⇒ mrx==0 是**必然、正常**的。
            #     Windows 串口独占，用户也没法再开串口助手去手敲 `sp`。
            #     ⇒ 先判 master_told：没主动发过命令就别渲染成"端口选错"。
            if mrx == 0 and now >= next_dead_t:
                dead_hits += 1
                next_dead_t = now + 10.0
                print("\n" + "!" * 62)
                print("⚠ 主臂串口 %s 已 %.0f s **一个字节都没收到**（其它输出也是 0）。"
                      % (args.master, now - t0))
                if not master_told:
                    print("  ★ 首因八成是这个：你这次**既没带 `--start`、也没带 `--master-cmd`**，")
                    print("    也就是**没有任何人给主臂发过命令**。主臂的横幅 + `> ` 提示符只在")
                    print("    开机那一次吐（早吐完了），待命时主循环一条都不打印；")
                    print("    而主臂口是独占的、本脚本又只管读 —— 它当然一个字都没有。")
                    print("    ⇒ 这不是硬件故障。Ctrl+C 退出，改成下面任意一种：")
                    print("       py tools/pc_relay.py --master %s --slave %s --start --hz 60"
                          % (args.master, args.slave))
                    print("       py tools/pc_relay.py --master %s --slave %s "
                          "--master-cmd \"mf 0\" --master-cmd \"sp 60\""
                          % (args.master, args.slave))
                    print("    ⚠ 两条都要带上 `mf 0`：只发 `sp` 不静默直连链路，主臂会同时跑")
                    print("      两路输出、舵机总线被读 1.5 倍，控制台流帧率会从 ~65 Hz 掉到")
                    print("      ~46 Hz（实测）；帧率一低，从臂固定的『每帧 25°』跳变判据")
                    print("      更容易被触发 → 整帧拒绝 → 自锁 → 看门狗 → 手感上就是「卡住」。")
                    print("    `sp` 一放出去主臂就该吐 `#P`（顺带回显几个字节，mrx 立刻 >0）。")
                    print("    真要是发了还是 0 字节，才轮到下面 1)~5)。")
                print("  其余可能（按概率）：")
                print("    1) 端口选错：主臂控制台不在 %s 上。" % args.master)
                print("       py tools/serial_sniff.py --list")
                print("       py tools/serial_sniff.py COM8 COM11 COM12 COM22 --secs 6")
                print("       打出「✅ 主臂位置流」的那个口才是 --master。")
                print("    2) 主臂板根本没在跑应用：BOOT0 还拉高/漏了烧完那次 RESET，")
                print("       板子停在 bootloader 里 —— 它在收到 0x7F 之前**一个字节都不吐**。")
                print("       （零成本自查：按一下主臂 RESET，听有没有两短声蜂鸣 = 固件在跑；")
                print("         或看这块口的 mrx 会不会跳一下。）")
                print("    3) 这个口被串口助手等占着/或根本是另一根线。")
                print("    4) 外接 USB-TTL 的 RX 没接到板子控制台 TX（或 RX/TX 接反）。")
                print("    5) 两端波特率不一致（本脚本 115200，固件控制台 USART6 也是 115200）。")
                print("  （每 10 s 复述一次；主臂口一旦有数据就停）")
                print("!" * 62)

            # 告警 B：收到过位置行、之后静默
            if not warned and last_pos and (now - last_pos) > args.timeout:
                warned = True
                print("\n⚠ 主臂已静默 %.1f s —— 检查：主臂是否敲了 `sp <Hz>`？"
                      "是否被示教/单键模式吃掉输入？" % (now - last_pos))

            # 每 2 秒刷新一行速率
            #  `积压` = 主臂串口驱动缓冲里还没被读走的字节数。稳态应恒为 0;
            #  持续 > 一行(46 B) 说明 PC/从臂这一侧跟不上, 位置在排队 ⇒ 延迟在累积。
            if now >= next_rate_t:
                dt = now - t0
                cur = (fwd - rate_mark) / 2.0
                rate_mark = fwd
                next_rate_t = now + 2.0
                try:
                    backlog = sm.in_waiting
                except Exception:
                    backlog = -1

                # 抖动：σ 与 峰值/中位 比。正常链路 σ 小、比值接近 1；
                # 若帧被驱动"攒批"交付，比值会跳到 2 以上。
                if len(gaps) >= 12:
                    n = len(gaps)
                    mean = sum(gaps) / n
                    sd = (sum((g - mean) ** 2 for g in gaps) / n) ** 0.5
                    mx = max(gaps)
                    med = sorted(gaps)[n // 2]
                    ratio = (mx / med) if med > 1e-9 else 0.0
                    jit_txt = "%.1f ms" % (sd * 1000.0)
                else:
                    mean = sd = mx = ratio = 0.0
                    n = 0
                    jit_txt = "--"

                print("\r转发 %d 行 | 当前 %.1f Hz | 坏行 %d | 其它 %d | 主臂收到 %d B "
                      "| 积压 %d B | 从臂 %d B | 抖动 %s | 用时 %.0f s"
                      % (fwd, cur, bad, other, mrx, backlog, srx, jit_txt, dt),
                      end="", flush=True)

                # 抖动告警：用 σ 与峰值。
                #  ⚠ 别用"峰值/中位"当判据 —— 帧成簇时**中位数本身也被拉到长间隔那一侧**
                #  （0 与 33 ms 交替时中位=33），比值恒为 1，反而检不出来。σ 才单调。
                #  实测标定：sp 60 平滑链路 σ≈0.3 ms；LT 攒批(0/33 交替) σ≈16.7 ms。
                if (n >= 12 and sd > 0.003 and mx > 0.015 and now >= next_jit_warn):
                    next_jit_warn = now + 30.0
                    jit_warned += 1
                    print("\n" + "~" * 62)
                    print("⚠ 帧到达不均匀：σ=%.1f ms，峰值 %.0f ms，中位 %.1f ms"
                          % (sd * 1000.0, mx * 1000.0, med * 1000.0))
                    print("  两种可能，按概率排：")
                    print("   1) USB 串口驱动的 **Latency Timer**（默认 16 ms）不是「有数据就"
                          "给应用」，而是「攒够 16 ms 或缓冲满才给」⇒ 每帧被压后一个 16 ms 边界，"
                          "帧成簇到达。**与波特率无关**，改 baud 不会顺带解决。")
                    print("      确认：主臂提到 `sp 100`(10 ms) 再看 —— 若峰值≈16/32 ms 且"
                          "间隔成对出现，即坐实。")
                    print("      处理：设备管理器 → 端口 → 高级 → Latency Timer 改 1 ms。"
                          "CH340 驱动常无此项，用 CP210x/FTDI 那一端做主臂口。")
                    print("   2) 主臂主循环被占满（提频后「当前 Hz」到不了设定值），"
                          "或从臂控制台回显 `#P` 吃带宽造成周期性停顿。")
                    print("~" * 62)
    except KeyboardInterrupt:
        print("\n" + "-" * 62)
        print("已停止：转发 %d 行，坏行 %d，主臂收到 %d B，从臂收到 %d B，用时 %.1f s"
              % (fwd, bad, mrx, srx, time.time() - t0))
        print("从臂仍保持最后姿态（位置环自锁）。要它停止跟随：从臂敲 `sf 0`。")
    finally:
        sm.close()
        ss.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
