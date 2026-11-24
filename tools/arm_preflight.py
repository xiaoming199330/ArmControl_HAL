#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_preflight.py — 往臂控制台敲命令之前的**安全前置检查**

■ 为什么非要有它（2026-09-18 实测踩到，两次）

臂侧命令分发是「先精确匹配 → 全都失败则按 tokens[0][0] 落进一个首字母 switch」。
这个设计对单键命令（s / h / n / i …）很方便，但对**打错的多字符命令**是致命的：
**在老固件上敲新命令不会报"未知命令"，而是掉进首字母那个 case 直接执行**。

    实测 cx → case 'c' → 阻塞式校准向导 → **松全部力矩**（整臂当场塌下来）
    cw / cy / cz / ps 同理；ze → 回零点；ge → IK 运动（都是真动作）

而这台板子 DTR/RTS 没接 BOOT0/NRST，重烧固件要**手动拉 BOOT0 + 复位**，
于是"想试一条新命令"和"手里这块板到底是不是新固件"之间没有便宜的确认手段 ——
一不小心就把臂放倒了。

■ 解决：固件指纹 + 只读探针

新固件在 `h`（help）的**第一行**印一个指纹：

    fw-tag ARM-CAN-0918G  pins=CAN1 PA11(RX)/PA12(TX)  iface=cx-switchable+cxpa
                          fix=cprint-overread,frameseq,tec-latch  cmds=cx,cp,cy,cz,cw,ps

`h` 是**所有固件版本都安全**的只读命令（case 'h' → cmdHelp，从不改状态）。
本模块先敲 `h`，认到指纹才放行后续命令；认不到就**拒绝发任何东西**，
并打印"板子大概率正卡在校准向导里，请敲 q 退出、千万别敲 s"。

（`--force` 可以强行继续，但 `cx` **仍然会被拦住** —— 它只存在于本轮固件，
 在老固件上不可能成功，却会按首字母兜底成"校准向导 → 松全部力矩"。）

⚠ 指纹 = 版本号。固件那边动了 `cmdHelp()` 的首行就要同步改下面的 `FW_TAG`。
⚠ 发命令**只发 \\r**：发 \\r\\n 时那个多余的 \\n 会掉进校准向导的按键读入，
   回显 "Unknown key"，看着像随机的怪输出。
"""
import time

# 已知的固件指纹（按 LINK 分）。`h` 首行出现**任意一条**即放行。
#   ARM-CAN-0918H : LINK=can —— CAN 从站（0918H 起开机默认 CAN2）
#                   ⚠ tag 未随 0922 重编改动 ⇒ 光看 tag 分不出新旧包。
#                     判"是不是 0922 重编版"要看**同一行有没有 `rc-tim4-capture`**（
#                     0922 之前导出的 outputs/can_api/02_slave_CANslave.bin 没有该 token）。
#   ARM-NET-0920A : LINK=net —— 网口 UDP 承载（CAN / 主从链路停用, 见红线 3）
#   ARM-BOTH-0922A : LINK=both —— **单固件, 运行期选 CAN / 网口**（2026-09-22 新增）。
#                   ⚠ 开机默认**两条都不开**（Flash 区 C 里没记录）⇒ 烧完必须先
#                     `link can|net` -> `link save` -> RESET，否则网口/CAN 都不通，
#                     现场表现和"烧坏了/没烧进去"一模一样。
#                     详见 docs/链路开关_link命令_2026-09-22.md。
FW_TAGS = (
    "fw-tag ARM-MASTER-1030E",
    "fw-tag ARM-MASTER-1030A",
    "fw-tag ARM-NET-0924BF",
    "fw-tag ARM-NET-0924BA",
    "fw-tag ARM-NET-0924AY",
    "fw-tag ARM-NET-0924AX",
    "fw-tag ARM-NET-0924AW",
    "fw-tag ARM-NET-0924AT",
    "fw-tag ARM-NET-0924AU",
    "fw-tag ARM-NET-0924AV",
    "fw-tag ARM-NET-0924AP",
    "fw-tag ARM-NET-0924AO",
    "fw-tag ARM-NET-0924AN",
    "fw-tag ARM-NET-0924AM",
    "fw-tag ARM-NET-0924AL",
    "fw-tag ARM-NET-0924AK",
    "fw-tag ARM-NET-0924AJ",
    "fw-tag ARM-NET-0924AF",
    "fw-tag ARM-NET-0924AE",
    "fw-tag ARM-NET-0924AD",
    "fw-tag ARM-NET-0924AC",
    "fw-tag ARM-NET-0924AB",
    "fw-tag ARM-NET-0924Y",
    "fw-tag ARM-NET-0924Z",
    "fw-tag ARM-NET-0924X",
    "fw-tag ARM-NET-0924W",
    "fw-tag ARM-NET-0924V",
    "fw-tag ARM-NET-0924U",
    "fw-tag ARM-NET-0924T",
    "fw-tag ARM-NET-0924R",
    "fw-tag ARM-NET-0924Q",
    "fw-tag ARM-NET-0924P",
    "fw-tag ARM-NET-0924O",
    "fw-tag ARM-NET-0924N",
    "fw-tag ARM-NET-0924L",
    "fw-tag ARM-NET-0924K",
    "fw-tag ARM-NET-0924J",
    "fw-tag ARM-NET-0924I",
    "fw-tag ARM-NET-0924H",
    "fw-tag ARM-NET-0924G",
    "fw-tag ARM-NET-0924F",
    "fw-tag ARM-NET-0924A",
    "fw-tag ARM-NET-0923F",
    "fw-tag ARM-NET-0923E",
    "fw-tag ARM-NET-0923D",
    "fw-tag ARM-NET-0923C",
    "fw-tag ARM-NET-0923B",
    "fw-tag ARM-NET-0923A",
    "fw-tag ARM-BOTH-0924BF",
    "fw-tag ARM-BOTH-0924BA",
    "fw-tag ARM-BOTH-0924AY",
    "fw-tag ARM-BOTH-0924AV",
    "fw-tag ARM-BOTH-0924AU",
    "fw-tag ARM-BOTH-0922A",
    "fw-tag ARM-NET-0921A",
    "fw-tag ARM-NET-0920R",
    "fw-tag ARM-NET-0920Q",
    "fw-tag ARM-NET-0920P",
    "fw-tag ARM-NET-0920O",
    "fw-tag ARM-NET-0920N",
    "fw-tag ARM-NET-0920M",
    "fw-tag ARM-NET-0920L",
    "fw-tag ARM-CAN-0918H",
    "fw-tag ARM-NET-0920K",
    "fw-tag ARM-NET-0920J",
    "fw-tag ARM-NET-0920I",
    "fw-tag ARM-NET-0920H",
    "fw-tag ARM-NET-0920G",
    "fw-tag ARM-NET-0920F",
    "fw-tag ARM-NET-0920E",
    "fw-tag ARM-NET-0920D",
    "fw-tag ARM-NET-0920C",
    "fw-tag ARM-NET-0920B",
)
# ⚠ 兼容既有引用（flash_arm.py / board_alive.py）；新代码请用 FW_TAGS。
FW_TAG = FW_TAGS[0]
CALIB_MARK = "Joint Calibration"

# 状态机（check() 的返回值就是它）
ST_VERIFIED = "verified"     # 认到指纹 —— 随便发
ST_FORCED = "forced"         # 用户加了 --force —— 放行, 但下面这条仍然拦住

# ⚠ 即使 --force 也**绝不发**的命令: **只有本轮固件里才有**的那些。
#   理由很干脆 —— 在没认到指纹的板子上, 它们**没有任何成功的可能**(老固件没这条命令),
#   但代价却是最高的一档: 老固件会按首字母兜底, 而 `cx` 落在
#       case 'c' → 阻塞式校准向导 → **松全部力矩**(整臂当场塌下来)
#   零收益 + 最高代价 ⇒ 无条件拦。
#   (对比: `cw`/`ps` 在上一版固件里真实存在, 所以 --force 下放行是合理的。)
NEW_ONLY_CMDS = ("cx",)


class PreflightError(RuntimeError):
    """前置检查没过 —— 调用方应当直接终止，不要接着敲命令。"""


def read_for(ser, secs):
    """非阻塞分段收 secs 秒，返回解码后的文本。"""
    t0 = time.time()
    buf = b""
    while time.time() - t0 < secs:
        buf += ser.read(4096)
    return buf.decode("utf-8", "replace")


def send_line(ser, cmd):
    """发一条命令。⚠ 只发 \\r，绝不发 \\r\\n（理由见模块头）。"""
    ser.write((cmd + "\r").encode("ascii", "replace"))


def check(ser, quiet=False, force=False, wait=1.6):
    """敲 `h` 认固件指纹。返回状态字符串（见 ST_VERIFIED / ST_FORCED）。

    - 认到指纹            → 返回 ST_VERIFIED
    - 没认到 + force=True → 打印告警后返回 ST_FORCED（调用方后续要靠
                            assert_safe_cmd 拦危险首字母）
    - 没认到 + force=False→ 抛 PreflightError，调用方**必须直接退出**
    """
    ser.reset_input_buffer()
    send_line(ser, "h")
    txt = read_for(ser, wait)

    hit = next((t for t in FW_TAGS if t in txt), None)
    if hit:
        if not quiet:
            print("  \u2714 固件指纹认到: %s" % hit)
            if "ARM-NET" in hit:
                print("     ⇒ 这是 **LINK=net** 构建: CAN / 主从链路(UART4) 均停用,")
                print("        控制台仍在 USART6(PC6/PC7)。要回 CAN 请重烧 02_slave_CANslave.bin。")
            elif "ARM-BOTH" in hit:
                print("     ⇒ 这是 **LINK=both** 构建: 串口常通, CAN / 网口 由 `link` 选一条。")
                print("        ⚠ 开机**默认两条都不开** ⇒ 先 `link can|net` -> `link save` -> RESET,")
                print("          否则网口 ping 不通 / CAN 不应答，会被误判成烧坏了。")
        return ST_VERIFIED

    lines = [l.rstrip() for l in txt.replace("\r", "").split("\n") if l.strip()]

    # ⚠ 必须把两种"没认到指纹"分开说, 否则会把人往错方向带(2026-09-18 实测):
    #   (a) `h` **一个字节都没回** ⇒ 板子根本没在跑(没上电/线没接/挂死)。
    #       这时叫人"重烧固件"是错的 —— 重烧也救不了一块没电的板子。
    #   (b) `h` **有回显但不含指纹** ⇒ 真·老固件, 这才该烧。
    # 判据只用"有没有回显字节", 不依赖中文编码(现场串口助手编码五花八门)。
    if not lines:
        msg = ["\u2717 **拒绝继续**: `h` 一个字节都没回, 无法确认固件版本。",
               "  口是开着的, 但板子没输出 —— **板子没在跑**。常见原因:",
               "      ① 12V/5V 没供电(CH340 靠 USB 供电, 所以\"口在\"不代表板子有电)",
               "      ② 线没插好 / 插错口(CH340 也可能是另一块板子的)",
               "      ③ 固件挂死(本工程没有看门狗, 一旦挂住就是永久静默)",
               "  \u21d2 先断电重上电 / 按 RESET, 看到启动打印(`Init ...` / `Init done.`)再跑本脚本。",
               "     \u26a0 **不要去重烧固件** —— 重烧救不了一块没上电的板子。"]
    else:
        msg = ["\u2717 **拒绝继续**: 没认到固件指纹 `%s`。" % "/".join(FW_TAGS),
               "  `h` **有回显但不含指纹** \u21d2 这块板跑的是老固件, 不是本轮固件。",
               "  在老固件上敲 cx/cw/ps 不会报\"未知命令\", 而是掉进首字母兜底:",
               "      cx \u2192 case 'c' \u2192 校准向导 \u2192 **松全部力矩**(整臂会塌下来)",
               "  \u21d2 先烧 outputs/can_api/02_slave_CANslave.bin(手动拉 BOOT0), 再跑本脚本。"]
    if CALIB_MARK in txt:
        msg += ["  \u26a0 板子**现在正卡在校准向导里** \u2014\u2014 请在串口终端敲 `q` 退出,",
                "     \u26a0\u26a0 千万别敲 `s`(会把当前姿态固化成零点, 见工程红线 4)。"]
    if lines:
        msg += ["  --- `h` 实际回显(前 8 行) ---"] + ["    " + l for l in lines[:8]]
        # 只有"真有板子在回话"时才谈 --force 硬跑; 全静默时 --force 毫无意义
        # (对一块不说话的板子发命令什么都不会发生, 只是浪费时间)。
        msg += ["  (确知风险仍要硬跑: 加 --force; 但 %s 仍然会被拦住 ——"
                " 它们只存在于本轮固件, 在老固件上零收益却会松力矩)"
                % "/".join(NEW_ONLY_CMDS)]

    if force:
        if not quiet:
            print("\n".join(msg))
            print("  \u26a0 --force: 忽略上述风险继续; %s 仍然会被拦。"
                  % "/".join(NEW_ONLY_CMDS))
        return ST_FORCED
    raise PreflightError("\n".join(msg))


def assert_safe_cmd(cmd, state):
    """发命令前的第二道闸。state = check() 的返回值。

    ST_VERIFIED → 全放行。
    ST_FORCED   → 放行, 但 NEW_ONLY_CMDS（只存在于本轮固件的命令）仍然拦下:
                  在没认到指纹的板子上它们不可能成功, 却会按首字母兜底成
                  **松全部力矩**。零收益 + 最高代价 ⇒ 无条件不许发。
    """
    if state == ST_VERIFIED:
        return
    stem = (cmd.strip().split() or [""])[0]
    if stem in NEW_ONLY_CMDS:
        raise PreflightError(
            "\u2717 固件指纹未认到, 而 `%s` 只存在于本轮固件 \u21d2 拒绝发送。\n"
            "    老固件上它会按首字母兜底成 `case 'c'` \u2192 校准向导 \u2192 **松全部力矩**,\n"
            "    而且在这块板子上它本来也不可能成功。\n"
            "    (先烧 outputs/can_api/02_slave_CANslave.bin; 只想取证可以先跑 --cmds cw,ps)"
            % cmd)
