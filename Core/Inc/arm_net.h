/* ============================================================
 * arm_net.h - 二进制控制协议的 网口(UDP) 承载（声明）
 *
 * 背景（2026-09-20）: 本板装有 LAN8720A PHY（RMII）。
 * RMII 固定脚 : PA1=REF_CLK PA2=MDIO PA7=CRS_DV PC1=MDC PC4=RXD0 PC5=RXD1
 * RMII 发送脚 : F407 有**两套**合法脚位, 本板取 A 组 **PG11/PG13/PG14**
 *               （B 组 PB11/PB12/PB13 全被占: PB11=USART3_RX 舵机总线A,
 *                 PB12/PB13 = CAN2）—— 详见 docs/工程红线速查 第 40 条。
 * ⇒ net 构建同时废掉 **CAN 从站**与**主从链路(UART4, PA1 让给 REF_CLK)**，
 *   只能重烧不能运行期切换。板载 LED_R(PC5)/LED_G(PC4) 让位给 RXD1/RXD0
 *   （用户 2026-09-20 确认"板载LED都不要了"）。
 *
 * 承载格式：UDP 数据报 = 逻辑包 [SEQ, CMD, LEN, PAYLOAD, CRC16-LE]
 * —— 与 CAN 分包重组后的逻辑包**完全同构**（CRC 同为
 * Aproto_Crc16(前 3+LEN 字节)），所以 14 条命令、语义、错误码
 * 与 docs/CAN协议规格书 逐字一致，上位机编码函数直接复用。
 * ============================================================ */
#ifndef ARM_NET_H
#define ARM_NET_H

#include <stdint.h>

#ifndef ARM_LINK_NET
#define ARM_LINK_NET  0
#endif

#if ARM_LINK_NET

void NetProto_Init(void);
void NetProto_Poll(void);
void NetProto_Rescan(void);          /* 重跑 PHY 扫描（`nx scan` 用） */

/* ---- `np` 引脚级取证: 采样在 arm_net.c, 打印在 main.c ----------------
 * 为什么拆开: cprint 是 main.c 的 static, 这边够不着; 而且诊断输出不想要
 * RLOG 的 `[RUN][07][INFO ]` 前缀。 */
typedef struct {
    uint8_t  valid;                  /* 0 = MAC 还没初始化完, 其余字段无效 */
    uint8_t  rmii_sel;               /* SYSCFG->PMC 的 RMII 选择位 */
    uint32_t macmiiar;               /* bit[4:2] 是 CR 分频, 168MHz 应为 4 */
    uint8_t  afr[9];                 /* PA1 PA2 PA7 | PC1 PC4 PC5 | PG11 PG13 PG14 */
    uint8_t  idr_pa2, idr_pa7, idr_pc1, idr_pc4, idr_pc5;
    uint32_t refclk_hi, refclk_n;    /* PA1 采样: 高电平次数 / 总次数 */

    /* ---- PA1 静态电平 × 内部上下拉 对照 (2026-09-20) -------------------
     * 上面那个 refclk_hi/refclk_n 比例**判不了有没有时钟**: 50MHz 真时钟下
     * 伪随机采样必然 ~50%, 而**悬空脚捡耦合噪声也是 ~50%** ⇒ 两种情况的
     * 读数一模一样(红线 44)。v1 却据此打印"在跳 => REF_CLK 有", 那是把
     * 猜测印成了结论; 更糟的是它还可能与 `nt`(硬件数边沿) 相反, 而两个
     * 矛盾读数必须先判"尺子坏了"再选(红线 45)。
     *
     * 这三个量测补上真正能分开"悬空"与"被驱动"的判据, **不用万用表**:
     * 把 PA1 临时配成普通输入, 分别在 内部不拉 / 上拉 / 下拉 下采样电平。
     *   · 三态都是高 ⇒ 外部**推挽拉高**(内部下拉压不动)
     *   · 三态都是低 ⇒ 外部**主动拉低**
     *   · 跟着内部上下拉走 ⇒ **悬空**(没有外部源) ⇒ B1 没接通 / 走线断
     * ⚠ 它只回答"这根脚上有没有外部源在驱动", **不回答**"有没有 50MHz"
     *   (那归 `nt` 的硬件边沿计数 + 自检)。两个问题分开问, 别混。 */
    uint32_t pa1_np, pa1_pu, pa1_pd; /* NOPULL / PULLUP / PULLDOWN 下的高电平次数 */
    uint32_t pa1_ln;                 /* 每种状态采样总次数 */
    uint16_t mdio_raw[8];            /* addr 0..7 的 IDR1 原始值 */
    uint8_t  mdio_ok[8];             /* 对应读操作是否 HAL_OK (0=忙位超时) */
} NetProtoProbe;

void NetProto_ProbeSample(NetProtoProbe* out);

/* ---- `nm` 软 MIIM（bit-bang）取证: 回答"MDIO 读回全 FFFF"到底是
 * "PHY 不应答" 还是 "MDIO 这条线本身有问题" --------------------------
 * MAC 的 ReadPHYRegister 只会给一个 0xFFFF，而 0xFFFF 有三种来源:
 *   ① PA2 根本没接到 PHY（悬空）② 线被外部硬拉高 ③ 接线都对但 PHY 不应答。
 * 这个探针用**开漏手动拉线** + **极慢的 MDC(≈180kHz)** 把它们拆开:
 *   - 主动写 0 能不能把线拉低    → 排除 ②
 *   - 松手后能不能自己回高、多快  → 判断有没有上拉电阻(线是不是真实网络) → 排除 ①
 *   - 读帧 TA+16bit 数据段里 MDIO 被拉低过几次 → 分清 ③ 与"值读错" */
typedef struct {
    uint8_t  valid;                  /* 0 = MAC 还没初始化完 */
    uint8_t  mdio_idle;              /* 松手(idle)时 PA2 的电平 */
    uint8_t  mdio_driven0;           /* 主动写 0 之后读回的电平 */
    uint8_t  rel_ok;                 /* 松手后是否自己回到 1 */
    uint32_t rel_loops;              /* 松手→回高 的循环次数 */
    uint32_t rel_cycles;             /* 同上, DWT 周期数 (0 = DWT 不可用) */
    uint8_t  dwt_ok;
    uint8_t  mdc_drv0, mdc_drv1;     /* PC1 写 0/写 1 读回, 自检能不能驱动 */
    uint16_t id1[32];                /* 软 MIIM 读 addr0..31 的 reg2(IDR1) 原始值 */
    uint8_t  lows[32];               /* 每个读帧里 TA+16bit 段被拉低的 bit 数 */
    uint8_t  phy_hit;                /* 软 MIIM 认出 LAN8720A 的地址, 0xFF=没认出 */
    uint8_t  any_hit;                /* 有任何地址给出非 FFFF/0000 的应答 */
    uint8_t  any_addr;               /* 该地址 (any_hit 有效时) */
} NetProbeMIIM;

void NetProto_MiimProbe(NetProbeMIIM* out);

/* ---- `nt` REF_CLK 时钟体检: 回答"PHY 到底在不在工作" ------------------
 * `np` 只能证明 PA1 上有跳变, 但**跳变 ≠ 时钟**: 悬空脚捡噪声同样是
 * "有时高有时低"。所以这里用**两条独立路径**的硬件量测把它拆开:
 *   [1][2] 复位模式(SMS=100,TS=TI2FP2): 每个上升沿清 CNT
 *          ⇒ **读到的 CNT = 距上一个上升沿几拍**; 有沿 ⇒ CNT 很小, 无沿 ⇒ 涨到成千上万。
 *          NOPULL / PULLDOWN 各跑一遍 = 对照实验(红线 34):
 *          PHY 的 REFCLKO 是推挽输出, 内部 40k 上下拉压不动它 ⇒ 两遍应一致;
 *          若 NOPULL "有沿"、PULLDOWN "消失" ⇒ 悬空脚噪声。
 *   [3][4] 外部时钟模式1(SMS=111,TS=TI2FP2): 每个上升沿 **CNT+1**,
 *          再用 DWT CYCCNT 卡一个**固定时间窗** ⇒ 边沿数/窗口 = 频率。
 *          这条路与 [1][2] 是**不同的计数机理**, 互为独立证据。
 *   [5]    **自检**: 把触发源换成已知的 TIM3_TRGO(100kHz), 同样量一遍 ——
 *          用来证明"计数链路本身是好的"。自检不过 ⇒ 本命令任何结论都不可信。
 * ⚠ 本工程 TIM5 挂在 **APB1**(42MHz×2 = **84MHz**), 不是 168MHz:
 *   写死 168MHz 会让所有时间换算差 2 倍 (0920F 的老 bug)。
 * ⚠ 量程: 外部时钟模式要求 f_in < f_TIM/2; 50MHz 已接近 84MHz 的极限,
 *   若读到"很高但不精确"的值, 只当"有快速信号"用, 别当频率读数。 */
typedef struct {
    uint8_t  valid;                  /* 0 = MAC 还没初始化完 */
    uint32_t tim_hz;                 /* TIM5 实际时钟 (APB1 定时器时钟; 本工程 84MHz) */
    uint8_t  dwt_ok;                 /* DWT CYCCNT 可用 (定窗需要) */
    uint8_t  pa1_level;              /* 量测前 PA1 的瞬时电平 */

    /* [1][2] 复位模式: 每个上升沿清 CNT ⇒ 读到的 CNT = 距上个沿几拍 */
    uint32_t np_max, np_low, np_n;   /* NOPULL   : 最大拍数 / 小样本数 / 总样本 */
    uint32_t pd_max, pd_low, pd_n;   /* PULLDOWN : 对照 */

    /* [3][4] 外部时钟模式1 + DWT 定窗: 窗口内数到几个沿 */
    uint32_t ec_np_edge, ec_np_us;
    uint32_t ec_pd_edge, ec_pd_us;

    /* [5] 自检: 触发源换成已知的 TIM3_TRGO(100kHz)
     * ⚠ 触发源必须查**本从定时器那一行**的内部触发表(RM0090 Table 76):
     *     TIM5 -> ITR0=TIM2  ITR1=TIM3  ITR2=TIM4  ITR3=TIM8
     *     TIM4 -> ITR0=TIM1  ITR1=TIM2  ITR2=TIM3  ITR3=TIM8
     *   接 TIM3_TRGO ⇒ **TIM5 用 TS=1(ITR1)**、TIM4 用 TS=2(ITR2)。
     *   0920J 初版对 TIM5 写成 TS=2 ⇒ 实际连的是 TIM4(本工程没跑) ⇒ 自检
     *   **永远不过** ⇒ 它给出的"整段作废"是**工具自己坏了**, 不是被测对象。 */
    uint8_t  st_ok;                  /* 1 = 自检通过 (否则结论都不可信) */
    uint32_t st_edge, st_us;         /* TIM5(ITR1) 数到的沿 */
    uint32_t st_w4;                  /* TIM4(ITR2) 数到的沿 —— **独立见证** */
    uint8_t  st_t3_run;              /* TIM3 自己在不在跑 (CNT 有变化) */

    /* [5b] 自由跑 + DWT 定窗: 验证"定窗"与"计数器"本身, 不依赖任何触发源。
     * 期望 ΔCNT ≈ tim_hz × 窗长µs / 1e6 (本工程 84MHz × 20ms = 1.68e6) */
    uint8_t  fr_ok;
    uint32_t fr_cnt, fr_us;

    /* 量测期间 PA1 的 AFR 回读 (应为 2 = AF2_TIM5): 证明"借用"真的生效 */
    uint8_t  af_ref;

    /* [6] 内部上下拉方波 × TI2 对照 —— 标定"引脚 → TI2FP2"这条通路。
     * 手法: 用软件来回搬 PA1 的内部上下拉(不动输出驱动 ⇒ 电气无风险),
     *   同时用**两条互不依赖的路**观测同一个激励:
     *     · IDR 软采样  → 电平有多少次"跟着我们走"
     *     · TIM5/TI2    → 同期数到多少边沿
     * 判据: 两者同量级 ⇒ 通路好, [1]~[4] 的读数成立;
     *       电平跟着走但 TI2 一个边沿都没有 ⇒ **TI2 通路坏**, [1]~[4] 全部作废;
     *       电平根本不跟着走 ⇒ 节点有外部驱动器(不是悬空) ⇒ 需断电把 B1 拨到
     *       OFF 让节点变被动, 再跑一次(不需重烧)。 */
    uint32_t lb_toggle;              /* 施加的上下拉脉冲次数 */
    uint32_t lb_follow;              /* 电平跟着走的次数 */
    uint32_t lb_ti2;                 /* 同期 TI2 数到的边沿 */
} NetProtoClock;

void NetProto_ClockProbe(NetProtoClock* out);

typedef struct {
    uint8_t  state;                  /* 0=MAC初始化 1=扫描PHY 2=协商/等链路 3=运行 */
    uint8_t  phy_found;              /* MDIO 上找到了 LAN8720A */
    uint8_t  phy_addr;               /* PHY 地址 0~31 */
    uint16_t phy_id;                 /* (IDR1<<5)|IDR2高6位 供辨认 */
    uint8_t  link;                   /* 1=链路 up */
    uint8_t  speed100;               /* 1=100M 0=10M */
    uint8_t  fulldup;
    uint8_t  forced;                 /* 1=自协商超时, 强制 100M 全双工 */
    uint32_t rx_pkts, tx_pkts;
    uint32_t udp_cmd, crc_err, tx_err, arp_req, icmp_req;
    /* ICMP 应答自检: calc=算出来的校验和, frame=组帧后帧里实际的校验和。
     * 两者不一致 ⇒ 缺陷在"存写/拷贝", 不在"算"(见 arm_net.c 的 s_icmp_calc 说明)。 */
    uint16_t icmp_calc, icmp_frame, icmp_ilen;
    /* ---- 0920R 发送路径体检 ----
     * 症状: f2 里校验和是对的(f2[36..37] 已由 icmp_frame 证明), 网线上却是 0x0000。
     *   ⇒ 缺陷只可能发生在 "f2 → s_txpool → DMA/MAC" 之间。这组探针把这段拆开:
     *     pool24/pool36 = 发送**返回后**从 s_txpool[slot] 回读的同一位置。
     *       若 pool36 仍=正确值 而网线=0  ⇒ 帧在池里是好的, 是 MAC 在发的时候改的(硬件校验和插入)。
     *       若 pool36 变成了 0          ⇒ 拷贝/内存被写, 与 MAC 无关。
     *     txd0[] 的 bit23:22 = CIC(校验和插入控制), 非 0 即硬件在插。
     *     rxd2[] = 接收描述符缓冲地址, 用来排除 "RX DMA 写进了 TX 池" 的别名。
     * 以上都是只读探针, 不改发送行为。 */
    uint32_t maccr;
    uint32_t txd0[4], txd2[4], rxd2[4];
    uint16_t pool24, pool36;
    uint8_t  pool_slot, pool38;
} NetProtoStatus;

void NetProto_GetStatus(NetProtoStatus* out);

/* 0920R 发送路径诊断开关(只影响诊断, 不影响正常协议行为):
 *   which = 0 : cic_fix  1=发送前显式把描述符 CIC 位写成 BYPASS(不插校验和, 正确行为)
 *   which = 1 : sentinel 1=故意把 ICMP 校验和写成 0xAABB, 判"软件写的值有没有上网线"
 *   which = 2 : hwck     1=反过来让 MAC 自己插校验和(CIC=TCP/UDP/ICMP 全计算)
 *   which = 0xFF : 只读, 不改
 * 返回: bit0=旧 cic_fix, bit1=旧 sentinel, bit2=旧 hwck */
uint8_t NetProto_DebugCtl(uint8_t which, uint8_t val);

#else  /* ---- 非 net 构建: 桩模式，零开销 ---- */

static inline void NetProto_Init(void)      { (void)0; }
static inline void NetProto_Poll(void)      { (void)0; }
static inline void NetProto_Rescan(void)    { (void)0; }
typedef struct { uint8_t valid; } NetProtoProbe;
static inline void NetProto_ProbeSample(NetProtoProbe* out) { out->valid = 0U; }
typedef struct { uint8_t valid; } NetProbeMIIM;
static inline void NetProto_MiimProbe(NetProbeMIIM* out) { out->valid = 0U; }
typedef struct { uint8_t valid; } NetProtoClock;
static inline void NetProto_ClockProbe(NetProtoClock* out) { out->valid = 0U; }
typedef struct { uint8_t state; uint16_t icmp_calc, icmp_frame, icmp_ilen;
                 uint32_t maccr; uint32_t txd0[4], txd2[4], rxd2[4];
                 uint16_t pool24, pool36; uint8_t pool_slot, pool38; } NetProtoStatus;
static inline void NetProto_GetStatus(NetProtoStatus* out)
{
    out->state = 0xFFU;
    out->icmp_calc = 0U; out->icmp_frame = 0U; out->icmp_ilen = 0U;
    out->maccr = 0U; out->pool24 = 0U; out->pool36 = 0U;
    out->pool_slot = 0U; out->pool38 = 0U;
    for (int i = 0; i < 4; i++) { out->txd0[i] = 0U; out->txd2[i] = 0U; out->rxd2[i] = 0U; }
}
static inline uint8_t NetProto_DebugCtl(uint8_t w, uint8_t v)
{ (void)w; (void)v; return 0U; }

#endif /* ARM_LINK_NET */

#endif /* ARM_NET_H */
