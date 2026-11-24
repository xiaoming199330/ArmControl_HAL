/* ============================================================
 * arm_net.c - 二进制控制协议的 网口(UDP) 承载（实现）
 *
 * 2026-09-20 新增。为什么自己写极简 UDP 栈而不是 lwIP：
 *   - 本工程是裸机主循环（无 RTOS），lwIP NO_SYS 也要背 25KB+ 代码
 *     和一套 socket 语义；这里只需要 ARP/IPv4/ICMP/UDP 四个头，
 *     静态 IP + 单端口，~500 行即可，且**全部路径有界**（红线 32）。
 *   - 上位机对接走 UDP 数据报 = 逻辑包，一包一报文，**没有分包
 *     也没有流重组**，比 CAN 承载还简单；TCP 版留待需要时再加。
 *
 * 链路形态（与 CAN 版完全对应）：
 *   上位机 --UDP:5000--> 臂
 *     数据报 = [SEQ, CMD, LEN, PAYLOAD..., CRC16-LE]（Aproto_Crc16 over 前 3+LEN 字节）
 *   臂 --UDP 回源--> 上位机（回包同构；源 MAC/IP/端口 = 请求方）
 *   附带：ARP 应答 + ICMP Echo 应答（PC 直接 `ping` 即可判通）。
 *
 * ⚠ 硬件约束（别忘）：
 *   - ETH_TXD1 在 F407 上只有 PB13 ⇒ 与 CAN2_TX 冲突 ⇒ LINK=net 构建里
 *     **不初始化 CAN**（main.c 按 ARM_LINK_NET 分流）。
 *   - PC4/PC5（原 LED_G/LED_R）= ETH_RXD0/RXD1，net 版 LED 让位。
 *   - LAN8720A 的 PHY 地址由 PHYAD0 引脚 strap 决定（0 或 1 常见），
 *     这里全地址段扫描，不做假设（红线 2 的精神：别信默认，去实测）。
 * ============================================================ */
#include "arm_net.h"
#include "main.h"
#include "arm_control_protocol.h"
#include "arm_control_api.h"
#include "arm_runlog.h"
#include "arm_link_cfg.h"   /* LinkCfg_TouchNet: auto 双链路门控(0924CE) */
#include <string.h>

#if ARM_LINK_NET

/* ---------------- 编译期网络配置（后续可学 CAN 落盘，先固定） ---------------- */
#define NET_UDP_PORT        5000U
static const uint8_t  NET_IP[4]  = {192, 168, 7, 120};        /* 臂的静态 IP */
static const uint8_t  NET_MAC[6] = {0x02, 0x4E, 0x45, 0x54, 0x00, 0x01}; /* 本地管理地址 "NET" */

#define NET_BUF_SIZE     1536U     /* ≥1518 以太网最大帧，包不跨描述符 */
#define NET_TX_TIMEOUT_MS  10U

/* ---------------- DMA 描述符与缓冲池 ---------------- */
static ETH_DMADescTypeDef s_dmrx[ETH_RX_DESC_CNT] __attribute__((aligned(4)));
static ETH_DMADescTypeDef s_dmtx[ETH_TX_DESC_CNT] __attribute__((aligned(4)));
static uint8_t s_rxpool[ETH_RX_DESC_CNT][NET_BUF_SIZE];
static uint8_t s_txpool[ETH_TX_DESC_CNT][NET_BUF_SIZE];
static uint8_t  s_rxpool_idx = 0U;
static uint8_t  s_txi = 0U;
static ETH_BufferTypeDef        s_rxnode;      /* 单描述符包的链回调用 */
static ETH_BufferTypeDef        s_txnode;
static ETH_TxPacketConfigTypeDef s_txcfg;

static ETH_HandleTypeDef s_eth;

/* ---------------- 状态机 ---------------- */
#define NET_ST_MAC_WAIT   0U   /* MAC 初始化完成，等下一拍开始扫 PHY */
#define NET_ST_PHY_SCAN   1U
#define NET_ST_AN_WAIT    2U   /* 自协商 + 等链路（超时则强制 100F） */
#define NET_ST_RUN        3U
static uint8_t  s_state = NET_ST_MAC_WAIT;
static uint32_t s_state_t0 = 0U;
static uint8_t  s_scan_idx = 0U;
static uint32_t s_scan_next = 0U;      /* 下一次扫地址的最早时刻（冷却用, 回绕安全） */
static uint16_t s_scan_raw[8];         /* addr 0..7 的 IDR1 原始值（诊断用） */
static uint8_t  s_scan_st[8];          /* 对应读操作是否 HAL_OK */
static uint8_t  s_scan_any = 0xFFU;    /* 本轮扫描里**任何**非 FFFF/0000 应答的地址 */
static uint16_t s_scan_anyval = 0U;    /* 该地址读到的值 */
static uint8_t  s_phy_addr = 0xFFU;   /* 0xFF = 未找到 */
static uint8_t  s_phy_found = 0U;
static uint8_t  s_link = 0U;
static uint8_t  s_speed100 = 1U, s_fulldup = 1U, s_forced = 0U;
/* 自协商超时的"已经报过一次"标志。
 * 网线没插时 net_step_an 会在 autoneg(6s) → 强制 100M(3s) → 回 autoneg
 * 之间无限循环, 每圈都打一条 WARN, 串口 9 秒刷一行噪声, 把真正的告警埋掉。
 * 这里只在**第一次**（或链路曾经起来又掉）时打, 中间循环静默。 */
static uint8_t  s_an_warned = 0U;

#define NET_SCAN_PERIOD_MS   2U      /* 每次轮询只扫 1 个地址（MDIO 读有内部超时） */
#define NET_AN_TIMEOUT_MS    6000U
#define NET_LINK_TIMEOUT_MS  3000U
#define NET_RETRY_MS         5000U

/* ---------------- 计数器 ---------------- */
static uint32_t s_rx_pkts, s_tx_pkts, s_udp_cmd, s_crc_err, s_tx_err, s_arp_req, s_icmp_req, s_dispatch;

/* ---------------- ICMP 应答自检（诊断用, 由 `nx` 打印） ----------------
 * 把"算出来的校验和"与"真正进了发送帧的校验和"**分开**记录, 一次就能分开三种缺陷:
 *   · 算得 == 帧里 == 正确值   ⇒ 通路完好, 包要是还被丢就是 PC 侧的事
 *   · 算得 == 正确值, 帧里 !=  ⇒ **算了没写进去**(存写漏了 / 写晚于拷贝 / 缓冲区被踩)
 *   · 算得 == 0xFFFF 或 0      ⇒ 根本没算(长度 0 / 提前 return)
 * ⚠ 只读不写, 在 net_process 主循环上下文里赋值, 无并发。
 *   为什么在 arm_net.c 里存而不是直接打印: cprint 是 main.c 的 static, 这边够不着
 *   (见 arm_net.h 的说明)。
 */
static uint16_t s_icmp_calc  = 0U;   /* ip_checksum() 算出来的值 */
static uint16_t s_icmp_frame = 0U;   /* 组帧后 f2 里实际躺着的校验和字段 */
static uint16_t s_icmp_ilen  = 0U;   /* 参与计算的 ICMP 字节数 */
/* ---- 0920R 发送路径体检 (只读探针 + 两个开关) ---- */
static uint16_t s_pool24 = 0U, s_pool36 = 0U;   /* 发送返回后, s_txpool[slot] 回读的 IP/ICMP 校验位 */
static uint8_t  s_pool_slot = 0U, s_pool38 = 0U;
static uint8_t  s_cic_fix   = 1U;   /* 1=发送前让 HAL 显式把 CIC 写成 BYPASS(不插校验和) */
static uint8_t  s_icmp_sentinel = 0U; /* 1=故意把 ICMP 校验和写成 0xAABB, 用来判"软件写的值有没有上网线" */
static uint8_t  s_hwck      = 0U;   /* 1=反过来让 MAC 自己插校验和(CIC=TCP/UDP/ICMP 全计算) */

/* ---------------- PHY 寄存器（LAN8720A = 标准 MII 寄存器） ---------------- */
#define PHY_REG_BMCR   0U
#define PHY_REG_BSR    1U
#define PHY_REG_IDR1   2U
#define PHY_REG_IDR2   3U
#define PHY_REG_SPECIAL 31U    /* LAN8720A 专用: 解决后的速度/双工 */
#define BMCR_RESET      0x8000U
#define BMCR_SPEED100   0x2000U
#define BMCR_ANENABLE   0x1000U
#define BMCR_ANRESTART  0x0200U
#define BMCR_FULLDUPLEX 0x0100U
#define BSR_LINK        0x0004U   /* ⚠ 锁存位: 低过一次就保持 1, 必须连读两次 */
#define BSR_AN_DONE     0x0020U

/* ============================================================
 * 底层
 * ============================================================ */

/* RMII 引脚（STM32F407ZGT6 / LQFP144）:
 *     PA1=REF_CLK   PA2=MDIO   PA7=CRS_DV
 *     PC1=MDC       PC4=RXD0   PC5=RXD1      <- 这 6 只**没有第二选择**
 *     TX_EN/TXD0/TXD1 有两套可选脚位, 本设计取 A:
 *         A) PG11 / PG13 / PG14    (AF11 = ETH_MII_TX_EN/TXD0/TXD1 与 RMII 版)
 *         B) PB11 / PB12 / PB13
 *
 * ⚠ 2026-09-20 **二次修正**（第一次的"修正"本身是错的）:
 *   初版写 PG11/PG13 + PB13(TXD1); 第一次"修正"时断言"F407 上 TX_EN
 *   只有 PB11、PG11 只有 RXER、PG13 只有 PPS_OUT" —— 查 ST 数据手册
 *   **DS8626 Table 7 后确认该断言不成立**: PG11/PG13/PG14 的 AF11 明列
 *   ETH_MII_TX_EN / ETH_MII_TXD0 / ETH_MII_TXD1（及 RMII 版）。
 *   ⇒ 不是"只有一套脚位", 而是**两套都合法, 要按"板上哪些脚已被占用"选**。
 *   本板 PB11 = USART3_RX(舵机总线A)、PB12/PB13 = CAN2_RX/TX 三个全占
 *   ⇒ 只有 PG 那套能自洽 ⇒ 回到 A 组, 并补上 TXD1 = **PG14**（初版漏了它）。
 *   教训(红线 40 修订): ① AF 表必须按具体型号查; ② **更要按"板子上的
 *   占用情况"交叉验证** —— 两套都合法时, 被别的外设占掉的那套就出局。
 *
 * ⚠ PB12/PB13 是 CAN2 的脚。net 构建里 CAN 外设不使能, 但**板载 CAN
 *   收发器的 RXD 是推挽输出、会持续驱动 PB12**, 与舵机/其它线路打对台
 *   ⇒ 跑 net 版时把 CAN 收发器从板上断开。 */
#ifndef NET_TX_PINS_ON_PG
#define NET_TX_PINS_ON_PG   1     /* 0 = 改用 B 组 PB11/PB12/PB13 试 */
#endif

static void net_gpio_init(void)
{
    GPIO_InitTypeDef g;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();                      /* PE2 = PHY nRST (V6.1) */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;   /* 必须在 ETH MAC 复位/使用前生效 */

    /* PE2: 官方 V6.1 例程中 LAN8720A 的硬件复位脚。
     * 先拉低 ≥50ms 再拉高，确保 PHY 真正退出复位。若某块板 nRST 没接 PE2，
     * 这条 GPIO 只是多一个推挽高输出，不会破坏别的功能。 */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(55);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(10);

    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF11_ETH;
    g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;      /* REF_CLK / MDIO / CRS_DV */
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;      /* MDC / RXD0 / RXD1       */
    HAL_GPIO_Init(GPIOC, &g);
#if NET_TX_PINS_ON_PG
    g.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;   /* TX_EN / TXD0 / TXD1     */
    HAL_GPIO_Init(GPIOG, &g);
#else
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;   /* TX_EN / TXD0 / TXD1     */
    HAL_GPIO_Init(GPIOB, &g);
#endif
}

static uint16_t ip_checksum(const uint8_t* h, uint16_t len)
{
    uint32_t sum = 0U;
    for (uint16_t i = 0U; i + 1U < len; i += 2U) {
        sum += ((uint32_t)h[i] << 8) | h[i + 1U];
    }
    if (len & 1U) sum += (uint32_t)h[len - 1U] << 8;
    while (sum >> 16) sum = (sum & 0xFFFFU) + (sum >> 16);
    return (uint16_t)(~sum & 0xFFFFU);
}

/* 以太网帧发送（阻塞带超时，红线 32） */
static uint8_t eth_send(const uint8_t* frame, uint16_t len)
{
    if (s_state != NET_ST_RUN) return 0U;
    uint8_t idx = s_txi;
    s_txi = (uint8_t)((s_txi + 1U) % ETH_TX_DESC_CNT);
    memcpy(s_txpool[idx], frame, len);
    s_txnode.buffer = s_txpool[idx];
    s_txnode.len = len;
    s_txnode.next = NULL;
    s_txcfg.Length = len;
    s_txcfg.TxBuffer = &s_txnode;
    /* ⚠ 0920R: 显式把 CIC(校验和插入控制) 写成 BYPASS。
     *   HAL 只在 `Attributes & ETH_TX_PACKETS_FEATURES_CSUM` 时才动 DESC0 的 CIC 位,
     *   我们早先 Attributes=0 ⇒ HAL **从不写** CIC ⇒ 描述符里那两位一直是它的初始值。
     *   万一它非 0(硬件校验和插入), MAC 会在发送时**改写**协议校验和字段 —— 这正是
     *   "f2 里是对的、网线上是 0x0000" 的唯一硬件级解释。这里显式清零, 不留余地。
     *   (红线 40 的同族: 外设字段没人设 ≠ 默认值是对的。) */
    if (s_hwck) {
        /* 反向对照: 让 MAC 自己插校验和。
         *   假设成立(MAC 在改写 ICMP 校验和位) ⇒ 它会算出**正确**值 ⇒ ping 通。
         *   假设不成立 ⇒ 网线值不变 ⇒ 排除"MAC 改写"这条线。 */
        s_txcfg.Attributes |= ETH_TX_PACKETS_FEATURES_CSUM;
        s_txcfg.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    } else if (s_cic_fix) {
        s_txcfg.Attributes |= ETH_TX_PACKETS_FEATURES_CSUM;
        s_txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;      /* = CIC_BYPASS = 0 */
    } else {
        s_txcfg.Attributes &= ~ETH_TX_PACKETS_FEATURES_CSUM;
        s_txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
    }
    HAL_StatusTypeDef txst = HAL_ETH_Transmit(&s_eth, &s_txcfg, NET_TX_TIMEOUT_MS);
    /* 发送返回后回读同一个槽: 这两组字节一个"能上网线"(IP 校验, 已实测 0x9136 正确),
     * 一个"上不去"(ICMP 校验)。同槽同时刻对比, 就能把"内存被写"和"MAC 改写"分开。 */
    s_pool_slot = idx;
    s_pool24 = (uint16_t)(((uint16_t)s_txpool[idx][24] << 8) | (uint16_t)s_txpool[idx][25]);
    s_pool36 = (uint16_t)(((uint16_t)s_txpool[idx][36] << 8) | (uint16_t)s_txpool[idx][37]);
    s_pool38 = s_txpool[idx][38];
    if (txst != HAL_OK) {
        s_tx_err++;
        return 0U;
    }
    s_tx_pkts++;
    return 1U;
}

/* 组 UDP 报文并发送（IPv4 头校验和软件算; UDP 校验和=0, IPv4 允许） */
static void udp_send(const uint8_t* dstmac, const uint8_t* dstip,
                     uint16_t dport, const uint8_t* data, uint16_t dlen)
{
    static uint8_t f[14 + 20 + 8 + APROTO_MAX_PAYLOAD + 2 + 4]; /* 静态, 避免栈压力 */
    uint16_t iplen = (uint16_t)(20U + 8U + dlen);
    uint16_t flen  = (uint16_t)(14U + iplen);
    uint16_t udp_len = (uint16_t)(8U + dlen);

    memcpy(&f[0], dstmac, 6);
    memcpy(&f[6], NET_MAC, 6);
    f[12] = 0x08; f[13] = 0x00;                       /* IPv4 */

    f[14] = 0x45; f[15] = 0x00;                       /* Ver/IHL, TOS */
    f[16] = (uint8_t)(iplen >> 8); f[17] = (uint8_t)iplen;
    f[18] = 0x00; f[19] = 0x01;                       /* ID */
    f[20] = 0x00; f[21] = 0x00;                       /* flags/frag */
    f[22] = 64U;  f[23] = 17U;                        /* TTL, UDP */
    f[24] = f[25] = 0U;                               /* 校验和占位 */
    memcpy(&f[26], NET_IP, 4);
    memcpy(&f[30], dstip, 4);
    {
        uint16_t c = ip_checksum(&f[14], 20U);
        f[24] = (uint8_t)(c >> 8); f[25] = (uint8_t)c;
    }

    f[34] = (uint8_t)(NET_UDP_PORT >> 8); f[35] = (uint8_t)NET_UDP_PORT;
    f[36] = (uint8_t)(dport >> 8);        f[37] = (uint8_t)dport;
    f[38] = (uint8_t)(udp_len >> 8);      f[39] = (uint8_t)udp_len;
    f[40] = f[41] = 0U;                               /* UDP 校验和 = 0 */
    memcpy(&f[42], data, dlen);

    (void)eth_send(f, flen);
}

/* ---- 逻辑包构建（与 CAN 版同一格式: [SEQ,CMD,LEN,PAYLOAD,CRC16-LE]） ---- */
static uint16_t build_logical(uint8_t seq, uint8_t cmd,
                              const uint8_t* pl, uint8_t len, uint8_t* out)
{
    out[0] = seq; out[1] = cmd; out[2] = len;
    if (len) memcpy(&out[3], pl, len);
    uint16_t crc = Aproto_Crc16(out, (uint16_t)(3U + len));
    out[3U + len]     = (uint8_t)(crc & 0xFFU);
    out[3U + len + 1U] = (uint8_t)(crc >> 8);
    return (uint16_t)(5U + len);
}

/* ============================================================
 * 协议处理
 * ============================================================ */

static void handle_cmd(const uint8_t* pl, uint16_t plen,
                       const uint8_t* srcmac, const uint8_t* srcip, uint16_t sport)
{
    s_udp_cmd++;
    if (plen < 5U) return;

    uint8_t len = pl[2];
    if ((uint16_t)(5U + len) != plen) { s_crc_err++; return; }

    uint16_t crc_got = (uint16_t)pl[3U + len] | ((uint16_t)pl[3U + len + 1U] << 8);
    if (crc_got != Aproto_Crc16(pl, (uint16_t)(3U + len))) {
        s_crc_err++;
        uint8_t nack_pl[2] = { pl[1], ARM_API_ERR_CRC };
        static uint8_t out[APROTO_MAX_PAYLOAD + 8U];
        uint16_t n = build_logical(pl[0], APROTO_RSP_NACK, nack_pl, 2U, out);
        udp_send(srcmac, srcip, sport, out, n);
        return;
    }

    uint8_t rspf[APROTO_MAX_FRAME];
    LinkCfg_TouchNet();   /* auto 门控: 合法包(CRC 已过)授/刷新 NET active(0924CE) */
    uint8_t rlen = Aproto_Dispatch(pl[0], pl[1], &pl[3], len,
                                   rspf, (uint8_t)sizeof(rspf), RunLog_RunId());
    s_dispatch++;
    if (rlen >= 7U && rspf[0] == 0xAAU && rspf[1] == 0x55U &&
        rspf[4] <= APROTO_MAX_PAYLOAD) {
        static uint8_t out[APROTO_MAX_PAYLOAD + 8U];
        uint16_t n = build_logical(rspf[2], rspf[3], &rspf[5], rspf[4], out);
        udp_send(srcmac, srcip, sport, out, n);
    }
}

static void net_process(uint8_t* p, uint16_t len)
{
    s_rx_pkts++;
    if (len < 14U) return;
    uint16_t et = (uint16_t)((p[12] << 8) | p[13]);

    if (et == 0x0806U) {                                   /* ARP */
        if (len < 42U) return;
        uint16_t op = (uint16_t)((p[20] << 8) | p[21]);
        if (op != 1U) return;                              /* 只应答请求 */
        if (memcmp(&p[38], NET_IP, 4) != 0) return;        /* 问的不是我 */
        s_arp_req++;
        static uint8_t a[42];
        memcpy(&a[0], &p[6], 6);                           /* 请求方 MAC */
        memcpy(&a[6], NET_MAC, 6);
        a[12] = 0x08; a[13] = 0x06;
        a[14] = 0x00; a[15] = 0x01;                        /* 以太网 */
        a[16] = 0x08; a[17] = 0x00;                        /* IPv4 */
        a[18] = 6;    a[19] = 4;
        a[20] = 0x00; a[21] = 0x02;                        /* reply */
        memcpy(&a[22], NET_MAC, 6);
        memcpy(&a[28], NET_IP, 4);
        memcpy(&a[32], &p[6], 6);                          /* 请求方 MAC */
        memcpy(&a[38], &p[28], 4);                         /* 请求方 IP */
        (void)eth_send(a, 42U);
        return;
    }

    if (et != 0x0800U) return;
    if (len < (uint16_t)(14U + 20U)) return;
    if ((p[14] >> 4) != 4U) return;                        /* 只收 IPv4 */
    uint16_t ihl = (uint16_t)(p[14] & 0x0FU) * 4U;
    if (ihl < 20U || len < (uint16_t)(14U + ihl)) return;
    if (memcmp(&p[30], NET_IP, 4) != 0) return;            /* 目的 IP 必须是我 */
    if (ip_checksum(&p[14], ihl) != 0U) return;            /* IPv4 头校验和必须对 */

    uint8_t proto = p[23];
    if (proto == 1U && len >= (uint16_t)(14U + ihl + 8U)) {   /* ICMP Echo */
        uint8_t* ic = &p[14 + ihl];
        if (ic[0] != 8U) return;                           /* 只要 echo request */
        s_icmp_req++;
        /* ⚠ 必须先把 type/code **和校验和字段** 一起清零, 再算校验和。
         *   只清 ic[0]/ic[1] 会把请求里那个旧 ICMP 校验和一起算进去 ⇒ 回包被 PC 丢弃。
         *   (与 udp_send 里 `f[24]=f[25]=0` 是同一条纪律: 见红线 61) */
        ic[0] = 0U; ic[1] = 0U;                            /* type=echo reply, code=0 */
        ic[2] = 0U; ic[3] = 0U;                            /* 校验和占位 */
        uint16_t ilen = (uint16_t)(len - 14U - ihl);
        uint16_t c = ip_checksum(ic, ilen);
        ic[2] = (uint8_t)(c >> 8); ic[3] = (uint8_t)c;

        static uint8_t hdr[34];
        memcpy(hdr, &p[6], 6);                             /* 目的 MAC = 请求方 */
        memcpy(&hdr[6], NET_MAC, 6);
        hdr[12] = 0x08; hdr[13] = 0x00;
        /* ⚠ IPv4 头**逐字段写**, 不要"复制请求头再改两处地址":
         *   那样会把请求的**旧头校验和**留在 hdr[24..25] 上参与求和 ⇒ 回包头校验错。
         *   顺带把 IHL 固定成 5(无选项), 免得请求带 IP 选项时长度对不上。 */
        hdr[14] = 0x45; hdr[15] = 0x00;                    /* Ver=4, IHL=5, TOS=0 */
        {
            uint16_t rtlen = (uint16_t)(20U + ilen);
            hdr[16] = (uint8_t)(rtlen >> 8); hdr[17] = (uint8_t)rtlen;
        }
        hdr[18] = p[18]; hdr[19] = p[19];                  /* ID 沿用请求 */
        hdr[20] = 0x00; hdr[21] = 0x00;                    /* 不分片 */
        hdr[22] = 64U;  hdr[23] = 1U;                      /* TTL=64, proto=ICMP */
        hdr[24] = 0U;   hdr[25] = 0U;                      /* 校验和占位 */
        memcpy(&hdr[26], NET_IP, 4);
        memcpy(&hdr[30], &p[26], 4);
        {
            uint16_t c2 = ip_checksum(&hdr[14], 20U);
            hdr[24] = (uint8_t)(c2 >> 8); hdr[25] = (uint8_t)c2;
        }
        /* 组完整帧: hdr(34) + ICMP(不含原 IP 头)
         * ⚠ 缓冲区必须按 NET_BUF_SIZE 取, 不能按"默认 ping(32B)"取:
         *   对端可以发 `ping -l 1472`, 那时 ilen≈1488, 写 34+74 的固定数组会被写穿,
         *   踩掉相邻静态变量 (红线 62: 缓冲区尺寸按"对端最大能发多少"定)。 */
        static uint8_t f2[NET_BUF_SIZE];
        if ((uint16_t)(34U + ilen) > (uint16_t)sizeof(f2)) return;   /* 理论不可达, 保险 */
        memcpy(f2, hdr, 34U);
        memcpy(&f2[34], ic, ilen);
        /* 诊断: 记下"算得的"和"帧里实际的"。这两个不一致 ⇒ 缺陷在存写/拷贝,
         * 不在算。见文件上方 s_icmp_calc 的说明。（f2[36..37] = f2[34]+2..3 = ICMP 校验和位） */
        /* 哨兵模式(nx sent): 故意把校验和改成一个**错的值**。
         *   若网线上出现 AABB ⇒ 软件写进去的字节**能**上网线 ⇒ 正常模式下网线上是 0x0000,
         *      说明"写 0"发生在更早的环节(而非 MAC 改写);
         *   若网线上仍是 0000 ⇒ 帧在离开软件后、到网线前被**协议感知地**改写了(硬件校验和插入)。 */
        if (s_icmp_sentinel) { f2[36] = 0xAA; f2[37] = 0xBB; }
        s_icmp_calc  = c;
        s_icmp_frame = (uint16_t)(((uint16_t)f2[36] << 8) | (uint16_t)f2[37]);
        s_icmp_ilen  = ilen;
        (void)eth_send(f2, (uint16_t)(34U + ilen));
        return;
    }

    if (proto != 17U) return;
    if (len < (uint16_t)(14U + ihl + 8U)) return;
    uint8_t* uh = &p[14 + ihl];
    uint16_t dport = (uint16_t)((uh[2] << 8) | uh[3]);
    uint16_t ulen  = (uint16_t)((uh[4] << 8) | uh[5]);
    if (dport != NET_UDP_PORT) return;
    if (ulen < 8U || (uint16_t)(ulen - 8U) > (uint16_t)(len - 14U - ihl)) return;
    handle_cmd(&uh[8], (uint16_t)(ulen - 8U), &p[6], &p[26],
               (uint16_t)((uh[0] << 8) | uh[1]));
}

/* ============================================================
 * HAL 回调覆盖（弱定义 → 这里生效）
 * ============================================================ */

/* 静态池轮转发缓冲: 池大小 == 描述符数, 主循环同步处理, 轮转即安全 */
void HAL_ETH_RxAllocateCallback(uint8_t** buff)
{
    *buff = s_rxpool[s_rxpool_idx];
    s_rxpool_idx = (uint8_t)((s_rxpool_idx + 1U) % ETH_RX_DESC_CNT);
}

/* 单描述符一包 (NET_BUF_SIZE ≥ 最大帧), 链表只挂一个节点 */
void HAL_ETH_RxLinkCallback(void** pStart, void** pEnd, uint8_t* buff, uint16_t Length)
{
    s_rxnode.buffer = buff;
    s_rxnode.len = Length;
    s_rxnode.next = NULL;
    *pStart = &s_rxnode;
    *pEnd = &s_rxnode;
}

/* ============================================================
 * 状态机步进（每次 Poll 走一小步, 全程无阻塞长循环）
 * ============================================================ */

static void net_step_mac(void)          /* MAC 初始化（只跑一次, ~ms 级） */
{
    static uint8_t done = 0U;
    if (done) { s_state = NET_ST_PHY_SCAN; s_state_t0 = HAL_GetTick(); return; }

    net_gpio_init();
    __HAL_RCC_ETHMAC_CLK_ENABLE();
    __HAL_RCC_ETHMACTX_CLK_ENABLE();
    __HAL_RCC_ETHMACRX_CLK_ENABLE();

    memset(&s_eth, 0, sizeof(s_eth));
    s_eth.Instance = ETH;
    s_eth.Init.MACAddr = (uint8_t*)NET_MAC;
    s_eth.Init.MediaInterface = HAL_ETH_RMII_MODE;
    s_eth.Init.TxDesc = s_dmtx;
    s_eth.Init.RxDesc = s_dmrx;
    s_eth.Init.RxBuffLen = NET_BUF_SIZE;
    HAL_StatusTypeDef init_st = HAL_ETH_Init(&s_eth);
    if (init_st != HAL_OK) {
        /* ⚠ 这条失败的**含义是确定的**, 别再靠猜 (2026-09-20 查 ST HAL 源码 + 官方注释):
         *
         *   HAL_ETH_Init() 干的第一件事就是 `DMABMR |= SR`(DMA 软复位) 然后等硬件自动清零,
         *   超过 ETH_SWRESET_TIMEOUT(500ms) 就 return HAL_ERROR。
         *   这一步发生在**任何 MDIO / PHY 操作之前** ⇒ 它跟"PHY 应不应答"无关。
         *
         *   ST 官方在该等待处的注释原文:
         *     "The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are not
         *      available, please check your external PHY or the IO configuration"
         *
         *   RMII 模式下 RX_CLK / TX_CLK 合起来就是那**一根 REF_CLK** ⇒
         *   **这条 log 等价于 "REF_CLK 不可用"**, 与 `nt` 数不到边沿是同一结论的两种测法
         *   (一个数引脚频率、一个看 MAC 时钟域能否走完复位)。
         *
         *   ⇒ 所以**不许**在这里加"跳过软复位检查"的 hack (社区里常见的 return→break):
         *     没有 REF_CLK 时网口物理上不可能工作, 绕过它只会把"没时钟"伪装成"通了"。
         */
        uint32_t       sr = (uint32_t)(s_eth.Instance->DMABMR & ETH_DMABMR_SR);
        static uint8_t n_fail = 0U;
        if (n_fail < 0xFFU) n_fail++;
        if (n_fail == 1U) {
            if (sr) {
                RLOG_W("NET", "HAL_ETH_Init failed (st=%u err=0x%08lX DMABMR.SR=%lu) ⇒ REF_CLK 不可用",
                       (unsigned)init_st, (unsigned long)s_eth.ErrorCode, (unsigned long)sr);
                RLOG_W("NET", "  SR 位不自清 = ETH_RX_CLK/TX_CLK 缺失 (RMII 下就是那根 REF_CLK)");
                RLOG_W("NET", "  ⇒ 与 `nt`(数 PA1 边沿) 同一结论; 查 PHY 供电/nRST/25MHz 晶振/REFCLKO 输出脚");
                RLOG_W("NET", "  ⇒ 与 `nm`(MDIO 无应答) 是**两件独立的事**, 别混为一谈");
            } else {
                RLOG_W("NET", "HAL_ETH_Init failed (st=%u err=0x%08lX) 但 SR 已清零 ⇒ 不是时钟问题",
                       (unsigned)init_st, (unsigned long)s_eth.ErrorCode);
                RLOG_W("NET", "  ⇒ 改查 MAC/DMA 配置: 描述符对齐、Init 结构体字段、Use_HAL_ETH_REGISTER_CALLBACKS");
            }
            RLOG_W("NET", "  只打这一次; 之后每 20 次重试(约 10s)报一行");
        } else if ((n_fail % 20U) == 0U) {
            RLOG_W("NET", "ETH 仍未起来 (第 %u 次重试) ⇒ REF_CLK 依旧不可用", (unsigned)n_fail);
        }
        return;                                    /* 下一拍重试 */
    }

    /* ⚠ MDC 时钟分频必须**手动设**(2026-09-20 发现):
     *   HAL_ETH_Init() 里不碰 MACMIIAR, 而 HAL_ETH_ReadPHYRegister /
     *   WritePHYRegister 每次都是 `tmpreg1 &= ~ETH_MACMIIAR_CR_MASK`
     *   之后**不回填 CR** ⇒ CR 恒为复位值 0 = Div42。
     *   本板 HCLK = 168 MHz ⇒ MDC = 168/42 = **4 MHz**, 超 LAN8720A
     *   的 MIIM 上限 2.5 MHz ⇒ PHY 不应答, MDIO 全地址扫不到。
     *   150-168 MHz 段必须用 Div102: 168/102 = 1.65 MHz ✔
     *   (红线 40: 外设的时分频字段没人设 ≠ 默认值是对的, 复位值只保证
     *    "能编译", 不保证"合规格"; 一定要按 HCLK 现算。) */
    s_eth.Instance->MACMIIAR = (s_eth.Instance->MACMIIAR & ~ETH_MACMIIAR_CR_Msk)
                               | ETH_MACMIIAR_CR_Div102;

    /* MAC/DMA 配置: 先取寄存器默认值, 再覆盖关键项 */
    ETH_MACConfigTypeDef mc;
    (void)HAL_ETH_GetMACConfig(&s_eth, &mc);
    mc.Speed = ETH_SPEED_100M;
    mc.DuplexMode = ETH_FULLDUPLEX_MODE;
    mc.ChecksumOffload = DISABLE;                  /* 校验和全软件, 行为可预期 */
    (void)HAL_ETH_SetMACConfig(&s_eth, &mc);

    ETH_DMAConfigTypeDef dc;
    (void)HAL_ETH_GetDMAConfig(&s_eth, &dc);
    dc.DMAArbitration = ETH_DMAARBITRATION_RX;     /* 收优先 */
    dc.ReceiveStoreForward = ENABLE;
    dc.TransmitStoreForward = ENABLE;
    dc.RxDMABurstLength = ETH_RXDMABURSTLENGTH_4BEAT;
    dc.TxDMABurstLength = ETH_TXDMABURSTLENGTH_4BEAT;
    dc.BurstMode = ETH_BURSTLENGTH_FIXED;
    dc.AddressAlignedBeats = DISABLE;
    dc.DropTCPIPChecksumErrorFrame = DISABLE;
    dc.ForwardErrorFrames = DISABLE;
    dc.ForwardUndersizedGoodFrames = ENABLE;       /* 短帧(ARP 42B)别被丢 */
    (void)HAL_ETH_SetDMAConfig(&s_eth, &dc);

    /* TX 包配置模板 */
    memset(&s_txcfg, 0, sizeof(s_txcfg));
    s_txcfg.Attributes = 0U;
    s_txcfg.SrcAddrCtrl = ETH_SOURCEADDRESS_DISABLE;  /* 源 MAC 我们自己写进帧头 */
    s_txcfg.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    s_txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;

    s_scan_idx = 0U;
    s_scan_any = 0xFFU;
    s_scan_anyval = 0U;
    done = 1U;
    s_state = NET_ST_PHY_SCAN;
    s_state_t0 = HAL_GetTick();
    RLOG_I("NET", "MAC init ok, scanning PHY (MDIO PA2/PC1)");
}

static void net_step_scan(void)         /* 每 NET_SCAN_PERIOD_MS 扫一个 PHY 地址 */
{
    /* ⚠ 冷却计时用**独立变量 + 有符号差**: 早先写成 `s_state_t0 = now + 3000`
     *   再和 `now` 做无符号比较 —— 未来时刻会让无符号差回绕成 ~4.29e9,
     *   判据恒真 ⇒ 冷却失效, 日志每 64ms(32 地址×2ms) 就刷一次。
     *   (红线 41: 时间戳做差一律强转 int32_t 再比, 别用无符号。) */
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_scan_next) < 0) return;
    s_scan_next = now + NET_SCAN_PERIOD_MS;

    uint32_t v1 = 0xFFFFU, v2 = 0U;
    uint8_t ok = (HAL_ETH_ReadPHYRegister(&s_eth, s_scan_idx, PHY_REG_IDR1, &v1) == HAL_OK);
    if (s_scan_idx < 8U) {
        s_scan_raw[s_scan_idx] = (uint16_t)v1;   /* 原始值: 诊断用(FFFF? 0000? 超时?) */
        s_scan_st[s_scan_idx]  = ok;
    }
    /* 只看 0x0007 太窄: 本板若其实不是 LAN8720A(或地址在 8..31), 会一律报"没有 PHY"。
     * 只要某地址给出的不是纯 FFFF/纯 0000, 就说明**有芯片在驱动 MDIO** ⇒ 记下来。 */
    if (ok && v1 != 0xFFFFU && v1 != 0x0000U && s_scan_any == 0xFFU) {
        s_scan_any = (uint8_t)s_scan_idx;
        s_scan_anyval = (uint16_t)v1;
    }
    if (ok && (v1 & 0xFFFFU) == 0x0007U &&
        HAL_ETH_ReadPHYRegister(&s_eth, s_scan_idx, PHY_REG_IDR2, &v2) == HAL_OK &&
        (v2 & 0xFFF0U) == 0xC0F0U) {
        s_phy_addr = s_scan_idx;
        s_phy_found = 1U;
        RLOG_I("NET", "LAN8720A found at PHY addr %u (id 0007.%04X)",
               (unsigned)s_scan_idx, (unsigned)(v2 & 0xFFFFU));
        /* 软复位 PHY 后进自协商 */
        (void)HAL_ETH_WritePHYRegister(&s_eth, s_phy_addr, PHY_REG_BMCR, BMCR_RESET);
        s_state = NET_ST_AN_WAIT;
        s_state_t0 = HAL_GetTick();
        s_forced = 0U;
        return;
    }
    s_scan_idx++;
    if (s_scan_idx >= 32U) {
        s_scan_idx = 0U;
        /* 打原始值: 全 FFFF = 线被上拉但 PHY 不应答(供电/nRST/地址);
         *           全 0000 = 线被拉死;            T = MDIO 忙位超时(MDC 或线) */
        RLOG_W("NET", "no PHY on MDIO; raw a0..a7=%04X%c %04X%c %04X%c %04X%c"
                      " %04X%c %04X%c %04X%c %04X%c (T=timeout)",
               (unsigned)s_scan_raw[0], s_scan_st[0] ? 'R' : 'T',
               (unsigned)s_scan_raw[1], s_scan_st[1] ? 'R' : 'T',
               (unsigned)s_scan_raw[2], s_scan_st[2] ? 'R' : 'T',
               (unsigned)s_scan_raw[3], s_scan_st[3] ? 'R' : 'T',
               (unsigned)s_scan_raw[4], s_scan_st[4] ? 'R' : 'T',
               (unsigned)s_scan_raw[5], s_scan_st[5] ? 'R' : 'T',
               (unsigned)s_scan_raw[6], s_scan_st[6] ? 'R' : 'T',
               (unsigned)s_scan_raw[7], s_scan_st[7] ? 'R' : 'T');
        s_scan_next = now + 5000U;                 /* 真的延后 5 s 再扫一轮 */
        if (s_scan_any != 0xFFU) {
            RLOG_W("NET", "no LAN8720A, but addr %u answered %04X ⇒ MDIO 通, 只是颗不是 8720A/判据不符",
                   (unsigned)s_scan_any, (unsigned)s_scan_anyval);
        } else {
            RLOG_W("NET", "全 32 地址无任何应答 ⇒ MDIO 上没芯片在驱动 (线/供电/nRST 三选一)");
        }
        s_scan_any = 0xFFU;                        /* 下一轮重新统计 */
    }
}

static void net_step_an(void)
{
    uint32_t bsr = 0U;
    if (HAL_ETH_ReadPHYRegister(&s_eth, s_phy_addr, PHY_REG_BSR, &bsr) != HAL_OK) return;

    if (!s_forced) {
        /* 自协商路径 */
        uint32_t bmsr = 0U;
        (void)HAL_ETH_ReadPHYRegister(&s_eth, s_phy_addr, PHY_REG_BSR, &bmsr);
        if ((bsr & BSR_AN_DONE) && (bmsr & BSR_LINK) && ((bmsr & BSR_LINK) == (bsr & BSR_LINK))) {
            s_link = 1U;
        } else if ((uint32_t)(HAL_GetTick() - s_state_t0) > NET_AN_TIMEOUT_MS) {
            /* 自协商超时 ⇒ 强制 100M 全双工（最常见的"交换机/网卡协商僵住"逃生门） */
            (void)HAL_ETH_WritePHYRegister(&s_eth, s_phy_addr, PHY_REG_BMCR,
                                           BMCR_SPEED100 | BMCR_FULLDUPLEX);
            s_forced = 1U;
            s_state_t0 = HAL_GetTick();
            /* 只在第一次超时时报一次。之后 autoneg<->forced 的循环继续跑
             * (那是有用的逃生门, 插上网线立刻就能连上), 但不再刷屏。 */
            if (!s_an_warned) {
                s_an_warned = 1U;
                RLOG_W("NET", "no link after %ums, tried forced 100M-FD once "
                              "(cable unplugged? retrying silently)",
                       (unsigned)NET_AN_TIMEOUT_MS);
            }
        }
    } else {
        /* 强制路径: 只等链路 */
        uint32_t bmsr2 = 0U;
        (void)HAL_ETH_ReadPHYRegister(&s_eth, s_phy_addr, PHY_REG_BSR, &bmsr2);
        if ((bsr & BSR_LINK) && (bmsr2 & BSR_LINK)) s_link = 1U;
        else if ((uint32_t)(HAL_GetTick() - s_state_t0) > NET_LINK_TIMEOUT_MS) {
            s_forced = 0U;
            (void)HAL_ETH_WritePHYRegister(&s_eth, s_phy_addr, PHY_REG_BMCR,
                                           BMCR_ANENABLE | BMCR_ANRESTART);
            s_state_t0 = HAL_GetTick();
            return;
        }
    }

    if (!s_link) return;

    /* 链路起来了: 从 LAN8720A reg31 读解决后的速度/双工, 配进 MAC 再 Start */
    uint32_t sp = 0U;
    (void)HAL_ETH_ReadPHYRegister(&s_eth, s_phy_addr, PHY_REG_SPECIAL, &sp);
    uint32_t mode = (sp >> 2) & 0x7U;
    s_speed100 = (mode == 5U || mode == 6U || mode == 7U) ? 1U : 0U;
    s_fulldup  = (mode == 2U || mode == 5U || mode == 6U) ? 1U : 0U;

    ETH_MACConfigTypeDef mc;
    (void)HAL_ETH_GetMACConfig(&s_eth, &mc);
    mc.Speed = s_speed100 ? ETH_SPEED_100M : ETH_SPEED_10M;
    mc.DuplexMode = s_fulldup ? ETH_FULLDUPLEX_MODE : 0U;
    (void)HAL_ETH_SetMACConfig(&s_eth, &mc);

    if (HAL_ETH_Start(&s_eth) != HAL_OK) {
        RLOG_W("NET", "HAL_ETH_Start failed");
        s_link = 0U;
        return;
    }
    s_state = NET_ST_RUN;
    s_an_warned = 0U;          /* 链路通了: 武装"下次掉线再报一次" */
    RLOG_I("NET", "link up: %sM %s-duplex, udp://192.168.7.120:5000 ready",
           s_speed100 ? "100" : "10", s_fulldup ? "full" : "half");
}

void NetProto_Poll(void)
{
    switch (s_state) {
    case NET_ST_MAC_WAIT: net_step_mac(); break;
    case NET_ST_PHY_SCAN: net_step_scan(); break;
    case NET_ST_AN_WAIT:  net_step_an();  break;
    default: break;
    }

    if (s_state != NET_ST_RUN) return;

    /* 收包: 最多 4 包/拍, 防主循环饿死（同 CanProto_Poll 的 guard 姿势） */
    uint8_t* pkt = NULL;
    void* p = NULL;
    uint8_t guard = 4U;
    while (guard-- && HAL_ETH_ReadData(&s_eth, &p) == HAL_OK) {
        pkt = (uint8_t*)((ETH_BufferTypeDef*)p)->buffer;
        uint16_t plen = (uint16_t)((ETH_BufferTypeDef*)p)->len;
        if (plen && ((ETH_BufferTypeDef*)p)->next == NULL) {
            net_process(pkt, plen);
        }
    }
}

void NetProto_Init(void)
{
    /* 延迟到第一次 Poll 再动硬件: 让启动打印先出去, MAC 初始化里也有可重试路径 */
    s_state = NET_ST_MAC_WAIT;
    s_state_t0 = HAL_GetTick();
}

void NetProto_Rescan(void)
{
    if (s_state == NET_ST_RUN) (void)HAL_ETH_Stop(&s_eth);
    s_state = NET_ST_PHY_SCAN;
    s_state_t0 = HAL_GetTick();
    s_scan_idx = 0U;
    s_scan_any = 0xFFU;
    s_scan_anyval = 0U;
    s_scan_next = 0U;
    s_phy_addr = 0xFFU;
    s_phy_found = 0U;
    s_link = 0U;
    s_forced = 0U;
}

/* ============================================================
 * 引脚级取证的数据采集 (`np`) —— 回答"网口不通, 是固件的锅还是硬件的锅"
 *   哲学与 CAN 的 ps/cw 一致: 只用 MCU 自己的引脚做观测量,
 *   不依赖示波器/逻辑分析仪。
 *   ⚠ 全流程只读: 不改引脚配置、不动状态机。
 *   ⚠ 打印交给 main.c —— cprint 是那边的 static; 也避免诊断输出混上
 *     RLOG 的 `[RUN][07][INFO ]` 前缀。
 * ============================================================ */
#define AFNUM(port, pin)  ((uint8_t)(((port)->AFR[(pin) >> 3U] >> (((pin) & 7U) * 4U)) & 0xFU))

static uint32_t idr_ones(GPIO_TypeDef* port, uint16_t pin, uint32_t n)
{
    uint32_t ones = 0U;
    for (uint32_t i = 0U; i < n; i++) {
        if ((port->IDR & pin) != 0U) ones++;
    }
    return ones;
}

/* ---- PA1 静态电平 × 内部上下拉 对照 (2026-09-20) -----------------------
 * 为什么需要: `np` 原有的"2000 次采样高/低计数"**判不了有没有时钟** ——
 *   ① 50MHz 真时钟 ⇒ 伪随机采样必然 ~50% 高; 悬空脚捡耦合噪声**也是 ~50%**
 *      ⇒ 这个读数在"有时钟"与"没时钟"两种情况下完全一样(红线 44)。
 *   ② 它还能与 `nt`(硬件数边沿) 的结论相反 —— 而两个矛盾读数必须先判
 *      "哪把尺子坏了"再选, 不能挑顺眼的信(红线 45)。
 *
 * 这条补上真正能分开"悬空"与"被驱动"的量测, 而且不用万用表:
 *   临时把 PA1 配成普通输入, 在 内部不拉 / 上拉 / 下拉 三种状态下采样电平。
 *     · 三态都是高 ⇒ 外部**推挽拉高**压不动内部下拉
 *     · 三态都是低 ⇒ 外部**主动拉低**
 *     · 跟着内部上下拉走 ⇒ **悬空** ⇒ B1 没接通 / REF_CLK 走线断
 * ⚠ 它只回答"这根脚上有没有外部源在驱动", 不回答"有没有 50MHz"(那归 `nt`)。
 */
#define PA1LVL_N     400U

static uint32_t pa1_level_under_pull(uint8_t pupd)  /* 0=NOPULL 1=PULLUP 2=PULLDOWN */
{
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof(g));
    g.Pin   = GPIO_PIN_1;
    g.Mode  = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pull  = (pupd == 1U) ? GPIO_PULLUP
            : (pupd == 2U) ? GPIO_PULLDOWN : GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    for (volatile uint32_t i = 0U; i < 4000U; i++) __NOP();   /* ≈0.25 ms 建立 */

    uint32_t hi = 0U;
    for (uint32_t i = 0U; i < PA1LVL_N; i++) {
        if (GPIOA->IDR & GPIO_PIN_1) hi++;
    }
    return hi;
}

static void pa1_restore_af11(void)
{
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof(g));
    g.Pin       = GPIO_PIN_1;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &g);
}

void NetProto_ProbeSample(NetProtoProbe* out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (s_eth.Instance != ETH) { out->valid = 0U; return; }
    out->valid = 1U;

    out->rmii_sel = (SYSCFG->PMC & SYSCFG_PMC_MII_RMII_SEL) ? 1U : 0U;
    out->macmiiar = s_eth.Instance->MACMIIAR;

    out->afr[0] = AFNUM(GPIOA, 1);  out->afr[1] = AFNUM(GPIOA, 2);  out->afr[2] = AFNUM(GPIOA, 7);
    out->afr[3] = AFNUM(GPIOC, 1);  out->afr[4] = AFNUM(GPIOC, 4);  out->afr[5] = AFNUM(GPIOC, 5);
    out->afr[6] = AFNUM(GPIOG, 11); out->afr[7] = AFNUM(GPIOG, 13); out->afr[8] = AFNUM(GPIOG, 14);

    /* REF_CLK 判据: 50MHz 方波下随机采样必然"有时高有时低"(只读 1 位,
     * 不追求准确占空比 —— 要的是**它到底会不会变**)。恒定 = 没时钟。 */
    out->refclk_n = 2000U;
    out->refclk_hi = idr_ones(GPIOA, GPIO_PIN_1, out->refclk_n);

    /* PA1 静态电平对照 (做完立刻把 PA1 还给 AF11, MDIO 读取不受影响) */
    out->pa1_ln = PA1LVL_N;
    out->pa1_np = pa1_level_under_pull(0U);
    out->pa1_pu = pa1_level_under_pull(1U);
    out->pa1_pd = pa1_level_under_pull(2U);
    pa1_restore_af11();

    out->idr_pa2 = (uint8_t)((GPIOA->IDR >> 2) & 1U);
    out->idr_pa7 = (uint8_t)((GPIOA->IDR >> 7) & 1U);
    out->idr_pc1 = (uint8_t)((GPIOC->IDR >> 1) & 1U);
    out->idr_pc4 = (uint8_t)((GPIOC->IDR >> 4) & 1U);
    out->idr_pc5 = (uint8_t)((GPIOC->IDR >> 5) & 1U);

    for (uint32_t a = 0U; a < 8U; a++) {
        uint32_t v = 0xFFFFU;
        out->mdio_raw[a] = 0xFFFFU;
        out->mdio_ok[a] = (HAL_ETH_ReadPHYRegister(&s_eth, a, PHY_REG_IDR1, &v) == HAL_OK) ? 1U : 0U;
        if (out->mdio_ok[a]) out->mdio_raw[a] = (uint16_t)(v & 0xFFFFU);
    }
}

/* ============================================================
 * `nm` —— 软 MIIM(bit-bang) + MDIO 线体检
 *
 * 要回答的问题: MAC 读 PHY 只回 0xFFFF。但 0xFFFF 有**三种来源**,
 *   光看这个值分不出来:
 *     ① PA2 根本没接到 PHY（悬空, 靠杂散电容显 1）
 *     ② 线被外部硬拉高（短路/被别的推挽输出占）
 *     ③ 接线都对, 但 PHY 没工作（供电 / nRST / 晶振 / PHYAD）⇒ 不应答
 * 手法: 把 PA2 改成**开漏输出**（能主动拉低, 也能松手读回真实电平）,
 *   PC1 改推挽输出, 用 ≈180kHz 的**慢**时钟手工发 MIIM 读帧。
 * 三个判据:
 *   写 0 能拉低          ⇒ 排除 ②
 *   松手能自己回高且很快  ⇒ 有上拉电阻 ⇒ 线是真实网络 ⇒ 排除 ①
 *   TA+16bit 段被拉低过几次 ⇒ PHY 到底有没有在驱动这根线(有, 值才有意义)
 * ⚠ 退出前把 PA2/PC1 还给 AF11, 状态机照常接着跑。
 * ⚠ 全程只用 MCU 自己的引脚当观测量, 不要示波器 —— 与 CAN 的 ps/cw 同哲学。
 * ============================================================ */
#define MIIM_DLY()  do { for (volatile uint32_t _i = 0U; _i < 120U; _i++) __NOP(); } while (0)

static inline void miim_mdc(uint8_t v)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline void miim_mdio(uint8_t one)   /* 开漏: 1 = 松手(靠外部上拉), 0 = 主动拉低 */
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, one ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline uint8_t miim_mdio_r(void)
{
    return (uint8_t)((GPIOA->IDR & GPIO_PIN_2) ? 1U : 0U);
}
static void miim_clk(void)                  /* 数据先建立 → 抬 MDC(PHY 采上升沿) → 落下 */
{
    MIIM_DLY(); miim_mdc(1U); MIIM_DLY(); miim_mdc(0U);
}

static void miim_pins_mode(uint8_t bitbang)
{
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof(g));
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    if (bitbang) {
        g.Pin = GPIO_PIN_2; g.Mode = GPIO_MODE_OUTPUT_OD; g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_1; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &g);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   /* 先松手 */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Alternate = GPIO_AF11_ETH;
        g.Pin = GPIO_PIN_2; HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOC, &g);
    }
}

/* 一帧 MIIM 读。*lows 出参 = TA+16bit 数据段里 MDIO 被拉低过的 bit 数:
 *   PHY 应答 ⇒ 至少 1(TA 的 0) ⇒ 通常十几;
 *   PHY 不应答 ⇒ **恒 0**(线被上拉钉在 1)。 */
static uint16_t miim_read(uint8_t phy, uint8_t reg, uint8_t* lows)
{
    uint8_t nb = 0U;

    miim_mdio(1U);
    for (uint8_t i = 0U; i < 32U; i++) miim_clk();      /* preamble = 32 个 1 */
    miim_mdio(0U); miim_clk();                          /* ST = 01 */
    miim_mdio(1U); miim_clk();
    miim_mdio(1U); miim_clk();                          /* OP = 10 (读) */
    miim_mdio(0U); miim_clk();
    for (int8_t b = 4; b >= 0; b--) {                   /* PHYAD, 高位先 */
        miim_mdio((uint8_t)((phy >> b) & 1U)); miim_clk();
    }
    for (int8_t b = 4; b >= 0; b--) {                   /* REGAD, 高位先 */
        miim_mdio((uint8_t)((reg >> b) & 1U)); miim_clk();
    }

    miim_mdio(1U); miim_clk();                          /* TA 第 1 拍: 我们松手 */
    if (miim_mdio_r() == 0U) nb++;
    miim_mdio(1U); miim_clk();                          /* TA 第 2 拍: PHY 应给 0 */
    if (miim_mdio_r() == 0U) nb++;

    uint16_t v = 0U;
    for (uint8_t i = 0U; i < 16U; i++) {                /* DATA 16 bit */
        miim_mdio(1U); miim_clk();
        uint8_t b = miim_mdio_r();
        if (b == 0U) nb++;
        v = (uint16_t)((v << 1) | b);
    }
    if (lows) *lows = nb;
    return v;
}

void NetProto_MiimProbe(NetProbeMIIM* out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->phy_hit = 0xFFU;
    if (s_eth.Instance != ETH) return;         /* MAC 还没起来 */
    out->valid = 1U;

    /* DWT 周期计数器: 给"松手→回高"一个真实的时间刻度(168MHz) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t c0 = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 300U; i++) __NOP();
    out->dwt_ok = (DWT->CYCCNT != c0) ? 1U : 0U;

    miim_pins_mode(1U);
    /* ⚠ 刚切成 GPIO 那一刻, MDIO 还停在 MAC 刚才驱动的电平上(多半是低),
     * 直接读会觉得"idle=0、没有上拉"。必须等线靠上拉自己稳下来再读
     * —— 否则一条结论是**时序假象**造出来的(2026-09-20 第一次跑 nm 踩过)。 */
    for (volatile uint32_t i = 0U; i < 20000U; i++) __NOP();

    /* [1] MDIO(PA2) 线体检 ------------------------------------------- */
    out->mdio_idle    = miim_mdio_r();                 /* 松手**稳定后**的电平 */
    miim_mdio(0U); MIIM_DLY(); MIIM_DLY();
    out->mdio_driven0 = miim_mdio_r();                 /* 我们拉低, 线跟不跟 */
    miim_mdio(1U);                                     /* 松手, 数它多快自己回高 */
    uint32_t cyc0 = DWT->CYCCNT;
    uint32_t n = 0U;
    while (n < 200000U && miim_mdio_r() == 0U) n++;
    out->rel_loops  = n;
    out->rel_cycles = DWT->CYCCNT - cyc0;
    out->rel_ok     = (n < 200000U) ? 1U : 0U;

    /* [2] MDC(PC1) 能不能驱动 ---------------------------------------- */
    miim_mdc(0U); MIIM_DLY();
    out->mdc_drv0 = (uint8_t)((GPIOC->IDR & GPIO_PIN_1) ? 1U : 0U);
    miim_mdc(1U); MIIM_DLY();
    out->mdc_drv1 = (uint8_t)((GPIOC->IDR & GPIO_PIN_1) ? 1U : 0U);

    /* [3] 软 MIIM 扫 addr 0..31 的 reg2(IDR1) ------------------------ */
    out->phy_hit = 0xFFU;
    out->any_hit = 0U;
    out->any_addr = 0xFFU;
    for (uint32_t a = 0U; a < 32U; a++) {
        uint8_t lows = 0U;
        out->id1[a]  = miim_read((uint8_t)a, PHY_REG_IDR1, &lows);
        out->lows[a] = lows;
        /* 认 ID: LAN8720A = 0007.C0Fx ⇒ IDR1 恰为 0x0007 */
        if (out->id1[a] == 0x0007U && out->phy_hit == 0xFFU) out->phy_hit = (uint8_t)a;
        /* 只认 IDR1==0x0007 太窄: 若这块板上其实不是 LAN8720A(或地址在 8..31),
         * 只要 chip 真的驱动过 MDIO, 值就不会是纯 FFFF / 纯 0000。 */
        if (out->id1[a] != 0xFFFFU && out->id1[a] != 0x0000U && out->any_hit == 0U) {
            out->any_hit  = 1U;
            out->any_addr = (uint8_t)a;
        }
    }

    miim_pins_mode(0U);                        /* 两只脚还给 ETH */
    s_scan_next = HAL_GetTick() + 200U;        /* 状态机尽快接着跑 */
}

/* ============================================================
 * `nt` REF_CLK 时钟体检 —— 第 2 版 (2026-09-20 夜, 0920G)
 *
 * 为什么重写: 第 1 版(0920F)在实机上**自己打自己**——
 *   [1] 复位模式说"一个沿都没有"(最大 CNT 30789 = 整个采样循环的长度,
 *       说明 CNT 从头到尾没被清过); 可是
 *   [3] 捕获模式却报"64 个相邻沿, 间隔又整齐地都是 27 拍"。
 *   两条路数出来的是矛盾的, 那就说明**至少有一条路本身是坏的** ⇒
 *   在这种状态下写任何结论(不管写"有"还是"没")都是猜。
 *   第 1 版 [3] 的病根: 它用**软件 while 轮询 CC2IF** 读捕获值, 于是
 *   当信号快过轮询、或干脆一个沿都没有时, 它数到的"间隔"其实是
 *   **自己的轮询周期**(≈27 拍 —— 与实测值逐字吻合)。
 *   ⇒ 教训同红线 11/44: **观测量必须来自硬件, 不能来自"我的循环跑多快"**。
 *
 * 第 2 版改成两条**互相独立**的硬件路径 + 一条**自检**:
 *   [1][2] 复位模式 (SMS=100, TS=TIM_TS_TI2FP2): 每个上升沿清 CNT
 *          无沿 ⇒ CNT 爬到成千上万; 有沿 ⇒ CNT 只在个位数。
 *          NOPULL / PULLDOWN 各跑一遍 (红线 34 对照实验)。
 *   [3][4] 外部时钟模式1 (SMS=111, TS=TIM_TS_TI2FP2): 每个上升沿 **CNT+1**,
 *          配 **DWT CYCCNT 卡的固定时间窗** ⇒ 直接给出 **边沿数/秒**。
 *          这条路不吃 "轮询快慢" 的亏 (只读窗口首尾两次 CNT)。
 *
 * ⚠⚠ 0920K 的第二个触发源 bug: `TS=7` 在 F4 上是 **TIM_TS_ETRF**，不是 TI2FP2；
 *    TI2FP2 必须写 `TIM_TS_TI2FP2`(=6)。所以 0920K 的 [1]~[4]/[6] 实际都在看
 *    **ETR 脚(根本没接)**，[6] 出现"IDR 跟随但 TI2=0"正是它暴露出来的。
 *    ⇒ 定时器触发源不要手写数字，直接用 CMSIS/HAL 宏。
 *   [5] 自检: 把触发源从引脚换成 **TIM3_TRGO @100kHz**(已知频率),
 *          两条路再各量一遍。**自检不过 ⇒ 本命令的输出整段作废**。
 *          (这就是"先把尺子校准, 再量零件"—— 缺了这一步, 前面那个矛盾就没法判。)
 *
 * ⚠ 本工程 TIM5 在 APB1: PCLK1 = 42MHz ⇒ **定时器时钟 = 84MHz**(不是 168MHz;
 *   老版写死 168 让所有换算差 2 倍。旁证: tim.c 里 TIM2 用 PSC=83 得 1MHz)。
 * ⚠ 量程: 外部时钟模式要求 f_in < f_TIM/2 ≈ 42MHz。50MHz 的 REF_CLK 处在
 *   量程之外, 会"数得到但不准" ⇒ 那时候只当 **"有快速信号"** 的证据, 别读频率。
 *
 * 收尾: 关 TIM3/TIM5, PA1 还给 AF11。⚠ 量测期间 REF_CLK 断过,
 * 跑完 `nt` 建议 RESET 一次再做业务。
 * ============================================================ */
#define NETCLK_PIN      GPIO_PIN_1
#define NETCLK_CN       512U
#define NETCLK_WIN_US   20000U          /* 定窗长度: 20 ms */
#define NETCLK_ST_US    20000U          /* 自检窗口: 20 ms ⇒ 100kHz 期望 2000 个沿 */

static uint8_t s_nc_af = 0U;   /* 借用 PA1 之后 AFR 的回读值 (应为 2 = AF2_TIM5) */

static void netclk_pin_af(uint8_t pupd)         /* 0=NOPULL 1=PULLUP 2=PULLDOWN */
{
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof(g));
    g.Pin       = NETCLK_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF2_TIM5;
    g.Pull      = (pupd == 1U) ? GPIO_PULLUP
                : (pupd == 2U) ? GPIO_PULLDOWN : GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    /* 回读 AFR: "我把脚配好了"和"脚真的是这个功能"是两件事。
     * 任何后续配置点若把这只脚抢回去, 这里会当场暴露(红线 49/52)。 */
    s_nc_af = AFNUM(GPIOA, 1);
}

static void netclk_pin_eth(void)                /* 还给 ETH_RMII_REF_CLK */
{
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof(g));
    g.Pin       = NETCLK_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF11_ETH;
    g.Pull      = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

/* TIM5 的**真实**时钟。APB1 预分频 =1 时定时器时钟 = PCLK1, 否则 = 2×PCLK1。 */
static uint32_t netclk_tim_hz(void)
{
    uint32_t hclk  = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return (pclk1 == hclk) ? pclk1 : (pclk1 * 2U);
}

static void netclk_tim_open(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1   = 0U;
    TIM5->PSC   = 0U;                    /* 直数定时器时钟 (84MHz) */
    TIM5->ARR   = 0xFFFFFFFFU;
    TIM5->CCMR1 = (1U << 8);             /* CC2S=01: IC2 直连 TI2; IC2F=0 不滤波 */
    TIM5->CCER  = (1U << 4);             /* CC2E=1, CC2P=0 (上升沿) */
    TIM5->SR    = 0U;
}

static void netclk_tim_reset_mode(uint32_t ts)  /* ts 用 HAL 宏, 如 TIM_TS_TI2FP2 */
{
    TIM5->CR1  = 0U;
    TIM5->CNT  = 0U;
    /* HAL 触发源宏(TIM_TS_*) 已经是 TS 字段移位后的值, 不能再 <<4 */
    TIM5->SMCR = ts | TIM_SLAVEMODE_RESET;
    TIM5->SR   = 0U;
    TIM5->CR1  = 1U;                     /* CEN */
}

static void netclk_tim_extclk_mode(uint32_t ts) /* ts 用 HAL 宏, 如 TIM_TS_ITR1/TI2FP2 */
{
    TIM5->CR1  = 0U;
    TIM5->CNT  = 0U;
    TIM5->SMCR = ts | TIM_SLAVEMODE_EXTERNAL1;
    TIM5->SR   = 0U;
    TIM5->CR1  = 1U;                     /* CEN */
}

static void netclk_tim_free_run(void)           /* SMS=0: CNT 直数定时器时钟 */
{
    TIM5->CR1  = 0U;
    TIM5->CNT  = 0U;
    TIM5->SMCR = 0U;
    TIM5->SR   = 0U;
    TIM5->CR1  = 1U;                     /* CEN */
}

static void netclk_settle(void)
{
    for (volatile uint32_t i = 0U; i < 20000U; i++) __NOP();   /* ≈1.2ms */
}

static uint8_t s_nc_dwt_ok = 0U;                 /* 本次量测 DWT 是否真的在跑 */

static void netclk_dwt_open(uint8_t* ok)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;
    uint32_t c0 = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 64U; i++) __NOP();
    s_nc_dwt_ok = (DWT->CYCCNT != c0) ? 1U : 0U;
    if (ok) *ok = s_nc_dwt_ok;
}

/* 固定时间窗内数 CNT 的增量 ⇒ 边沿个数。返回**实际窗口长度**(µs)。
 * ⚠ 本工程无看门狗 ⇒ 这里的 while **必须**带上界(红线 6):
 *   DWT 一旦不跑(CYCCNT 不前进), 没有界就是永久静默。 */
static uint32_t netclk_win(uint32_t us, uint32_t* delta)
{
    uint32_t hz     = HAL_RCC_GetHCLKFreq();
    uint32_t target = (uint32_t)(((uint64_t)hz * (uint64_t)us) / 1000000ULL);
    if (target == 0U) target = 1U;
    uint32_t c0 = DWT->CYCCNT;
    uint32_t n0 = TIM5->CNT;
    uint32_t guard = target * 4U;                   /* 宽限: DWT 正常时 1 次就够 */
    if (guard == 0U) guard = 1U;
    while ((uint32_t)(DWT->CYCCNT - c0) < target && guard > 0U) guard--;
    uint32_t n1 = TIM5->CNT;
    uint32_t c1 = (uint32_t)(DWT->CYCCNT - c0);
    *delta = n1 - n0;
    if (c1 < target) return 0U;                     /* 窗口没走完(DWT 不前进) ⇒ 无效 */
    return (uint32_t)(((uint64_t)c1 * 1000000ULL) / (uint64_t)hz);
}

/* 复位模式采样: 丢掉起步爬升的样本, 再统计"最大拍数 / 小样本数(=被清过)"。
 * ⚠ 必须丢起步样本: 计数器从 0 起涨, 头几个读数天然很小, 不丢就会
 *   把"没边沿"误读成"有边沿"。 */
static void netclk_sample_cnt(uint32_t* mx, uint32_t* low, uint32_t* n_out, uint32_t thr)
{
    uint32_t mxv = 0U, nlow = 0U;
    for (uint32_t i = 0U; i < 24U; i++) (void)TIM5->CNT;      /* 爬升期丢弃 */
    for (uint32_t i = 0U; i < NETCLK_CN; i++) {
        uint32_t v = TIM5->CNT;
        if (v > mxv) mxv = v;
        if (v <= thr) nlow++;
        for (volatile uint32_t d = 0U; d < 6U; d++) __NOP();
    }
    *mx = mxv; *low = nlow; *n_out = NETCLK_CN;
}

/* [5] 自检: TIM3 出 100kHz 的 TRGO(=update), 同时喂给**两个互相独立的从定时器**。
 *
 * ⚠⚠ 触发源必须查**本从定时器那一行**的内部触发表(RM0090 Table 76, TIM2..TIM5):
 *       从定时器  ITR0   ITR1   ITR2   ITR3
 *       TIM2      TIM1   TIM8   TIM3   TIM4
 *       TIM3      TIM1   TIM2   TIM5   TIM4
 *       TIM4      TIM1   TIM2   TIM3   TIM8
 *       TIM5      TIM2   TIM3   TIM4   TIM8
 *     接 TIM3_TRGO ⇒ **TIM5 必须用 TS=1(ITR1)**、**TIM4 用 TS=2(ITR2)**。
 *     v2 初版对 TIM5 写成 TS=2 ⇒ 它实际去听 **TIM4**(本工程从没配过、没跑过) 的
 *     TRGO ⇒ 一个沿都数不到 ⇒ **自检 100% 不过**。
 *     ⇒ 它输出的"整段作废"是**工具自己的接线错误**, 与被测对象无关。
 *     ⭐ 所以纪律是: **"自检失败"这句话本身也是要被审的对象** —— 不能因为自检
 *       举手, 就反过来证明了被测对象有问题(红线 53)。
 *
 * 为什么让 TIM4 也数一遍: 万一 TIM5 那条 ITR 真的坏了, 光看 TIM5=0 分不清
 *   "TIM3 没发 TRGO" 与 "TIM5 的 ITR 通路坏"。两条同时数 ⇒ 四方对照:
 *     T5≈T4≈期望 且 TIM3 在跑   ⇒ 整个计数链路完好 ✔
 *     T4≈期望, T5=0             ⇒ **TIM5 的 ITR 通路坏**
 *     T4=T5=0, TIM3 不在跑      ⇒ TIM3 根本没起振(PSC/ARR/时钟)
 *     T4=T5=0, TIM3 在跑        ⇒ TRGO 没出去(CR2.MMS)
 * 通过判据 ±5%。 */
static void netclk_selftest(uint8_t* ok, uint32_t* e5, uint32_t* us,
                            uint32_t* e4, uint8_t* t3run)
{
    *ok = 0U; *e5 = 0U; *us = 0U; *e4 = 0U; *t3run = 0U;
    if (!s_nc_dwt_ok) return;            /* 定窗不可用 ⇒ 自检直接判"不过" */

    /* ---- TIM3: 100kHz 的 TRGO(update 事件) ---- */
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->PSC = 83U;                     /* 84MHz/84 = 1MHz */
    TIM3->ARR = 9U;                      /* 1MHz/10 = 100kHz */
    TIM3->CNT = 0U;
    TIM3->CR2 = (2U << 4);               /* MMS=010: TRGO = update 事件 */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR  = 0U;
    TIM3->CR1 = 1U;

    /* ---- TIM4: 独立见证, 听 ITR2(=TIM3_TRGO), 与 TIM5 同一激励 ---- */
    __HAL_RCC_TIM4_CLK_ENABLE();
    TIM4->CR1  = 0U;
    TIM4->PSC  = 0U;
    TIM4->ARR  = 0xFFFFU;
    TIM4->CNT  = 0U;
    TIM4->SMCR = TIM_TS_ITR2 | TIM_SLAVEMODE_EXTERNAL1; /* 听 TIM3_TRGO */
    TIM4->SR   = 0U;
    TIM4->CR1  = 1U;

    /* ---- TIM5: 被测的那条路, 听 ITR1(=TIM3_TRGO) ---- */
    netclk_tim_extclk_mode(TIM_TS_ITR1);

    /* ---- 同一个窗口里两条路一起数 ---- */
    uint32_t d  = 0U;
    uint32_t w  = netclk_win(NETCLK_ST_US, &d);
    uint32_t d4 = TIM4->CNT;

    /* ---- TIM3 自己到底在不在跑(不能只看 T5/T4) ---- */
    uint32_t a = TIM3->CNT;
    for (volatile uint32_t i = 0U; i < 300U; i++) __NOP();
    uint32_t b = TIM3->CNT;
    for (volatile uint32_t i = 0U; i < 300U; i++) __NOP();
    uint32_t c = TIM3->CNT;
    *t3run = ((a != b) || (b != c)) ? 1U : 0U;

    TIM3->CR1 = 0U;
    TIM4->CR1 = 0U;
    TIM5->CR1 = 0U;
    __HAL_RCC_TIM3_CLK_DISABLE();
    __HAL_RCC_TIM4_CLK_DISABLE();

    *e5 = d; *us = w; *e4 = d4;
    if (w > 0U) {
        uint32_t exp = w / 10U;          /* 100kHz × 窗长µs / 1e6 */
        uint32_t tol = exp / 20U;
        if (tol < 2U) tol = 2U;
        uint32_t q5 = (d  > exp) ? (d  - exp) : (exp - d);
        uint32_t q4 = (d4 > exp) ? (d4 - exp) : (exp - d4);
        *ok = (q5 <= tol && q4 <= tol) ? 1U : 0U;
    }
}

/* [5b] 自由跑 + DWT 定窗 —— 验证"定窗机制"与"计数器"本身。
 * 不依赖任何触发源, 所以它是 [5] 之外的**第三条独立证据**:
 *   期望 ΔCNT ≈ tim_hz × 窗长µs / 1e6 (本工程 84MHz × 20ms ≈ 1.68e6)。
 * 若它也不过 ⇒ `netclk_win` 或 TIM5 计数器本身有问题, 前面所有窗口读数都作废。 */
static uint8_t netclk_selftest_freerun(uint32_t* cnt, uint32_t* us)
{
    uint32_t d = 0U, w = 0U;
    *cnt = 0U; *us = 0U;
    if (!s_nc_dwt_ok) return 0U;
    netclk_tim_free_run();
    w = netclk_win(NETCLK_ST_US, &d);
    TIM5->CR1 = 0U;
    *cnt = d; *us = w;
    if (w == 0U) return 0U;
    uint32_t exp = (uint32_t)(((uint64_t)netclk_tim_hz() * (uint64_t)w) / 1000000ULL);
    if (exp == 0U) return 0U;
    uint32_t tol = exp / 20U;
    if (tol < 10U) tol = 10U;
    uint32_t q = (d > exp) ? (d - exp) : (exp - d);
    return (q <= tol) ? 1U : 0U;
}

/* [6] 内部上下拉方波 × TI2 对照 (2026-09-20) —— 标定"引脚 → TI2FP2"这段通路
 *
 * 要回答的问题: [1]~[4] **全都**经由 "PA1 → TI2FP2 → TIM5" 这一条路。
 *   若这条路本身坏了, 那"0 个沿"既可能是"线上真没信号", 也可能是
 *   "信号有、但进不了定时器" —— 而这两种情形的处置**完全相反**。
 *   [5] 只能校准"计数 + 定窗"那一段(它走 ITR), **校准不到引脚这一段**。
 *
 * 手法: PA1 临时配成 AF2_TIM5 的**输入**通道(CC2S=01 ⇒ 输出驱动关闭, 只有
 *   内部上下拉在起作用 ⇒ **电气上不可能和外部驱动器对着打**), 再由软件来回
 *   搬动内部上下拉(40k), 制造一串**已知的人工方波**; 同时用两条互不依赖的
 *   路观测同一次激励:
 *     · IDR 软采样  → 电平有多少次"跟着我们走"
 *     · TIM5 / TI2  → 同期数到多少边沿
 * 判据(两边看的是同一次激励, 可直接比):
 *   follow ≈ toggle 且 ti2 ≈ toggle  ⇒ **TI2 通路完好** ⇒ [1]~[4] 成立
 *   follow ≈ toggle 但 ti2 ≈ 0       ⇒ **TI2 通路坏** ⇒ [1]~[4] 全部作废
 *   follow 很小                      ⇒ 节点有**外部驱动器**(不是悬空) ⇒
 *        断电把 B1 拨到 OFF 让节点变被动, 再跑一次即可(**不需重烧**)
 * ⚠ 它只在"节点能被 40k 搬动"时构成阳性对照; 搬不动本身也是一条信息。 */
#define NETCLK_LB_N      200U     /* 上下拉脉冲轮数 ⇒ 期望 ≈ 400 个边沿 */
#define NETCLK_LB_SETTLE 1000U    /* ≈27µs: 40k × 几十 pF 也只要几 µs, 留足余量 */

static inline void netclk_pa1_pupdr(uint32_t mode)   /* 0=NOPULL 1=PULLUP 2=PULLDOWN */
{
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U << (1U * 2U))) | (mode << (1U * 2U));
}

static void netclk_loopback(uint32_t* toggle, uint32_t* follow, uint32_t* ti2)
{
    uint32_t tg = 0U, fo = 0U;
    *toggle = 0U; *follow = 0U; *ti2 = 0U;

    netclk_pin_af(0U);                       /* AF2_TIM5(输入型) + NOPULL */
    netclk_settle();
    netclk_tim_extclk_mode(TIM_TS_TI2FP2);              /* TS=TI2FP2, SMS=111 */

    for (uint32_t i = 0U; i < NETCLK_LB_N; i++) {
        netclk_pa1_pupdr(1U);                /* 拉高 */
        for (volatile uint32_t d = 0U; d < NETCLK_LB_SETTLE; d++) __NOP();
        if ((GPIOA->IDR & NETCLK_PIN) != 0U) fo++;
        netclk_pa1_pupdr(2U);                /* 拉低 */
        for (volatile uint32_t d = 0U; d < NETCLK_LB_SETTLE; d++) __NOP();
        if ((GPIOA->IDR & NETCLK_PIN) == 0U) fo++;
        tg += 2U;
    }
    *ti2 = TIM5->CNT;

    netclk_pa1_pupdr(0U);
    TIM5->CR1 = 0U;
    *toggle = tg; *follow = fo;
}

void NetProto_ClockProbe(NetProtoClock* out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (s_eth.Instance != ETH) return;          /* MAC 还没起来, valid=0 */
    out->valid     = 1U;
    out->tim_hz    = netclk_tim_hz();
    out->pa1_level = (uint8_t)((GPIOA->IDR & NETCLK_PIN) ? 1U : 0U);

    netclk_dwt_open(&out->dwt_ok);
    netclk_tim_open();

    /* "小样本"判据: CNT ≤ 定时器时钟/1MHz ⇒ 等价于该处边沿率 ≥1MHz */
    uint32_t thr = out->tim_hz / 1000000U;
    if (thr < 4U) thr = 4U;

    /* [1] 复位模式 · NOPULL */
    netclk_pin_af(0U);
    netclk_settle();
    netclk_tim_reset_mode(TIM_TS_TI2FP2);
    netclk_sample_cnt(&out->np_max, &out->np_low, &out->np_n, thr);

    /* [2] 复位模式 · PULLDOWN 对照 */
    netclk_pin_af(2U);
    netclk_settle();
    netclk_tim_reset_mode(TIM_TS_TI2FP2);
    netclk_sample_cnt(&out->pd_max, &out->pd_low, &out->pd_n, thr);

    /* [3] 外部时钟模式1 · NOPULL —— 与 [1] 是**不同机理**的第二条路 */
    netclk_pin_af(0U);
    netclk_settle();
    netclk_tim_extclk_mode(TIM_TS_TI2FP2);
    if (out->dwt_ok) out->ec_np_us = netclk_win(NETCLK_WIN_US, &out->ec_np_edge);

    /* [4] 外部时钟模式1 · PULLDOWN 对照 */
    netclk_pin_af(2U);
    netclk_settle();
    netclk_tim_extclk_mode(TIM_TS_TI2FP2);
    if (out->dwt_ok) out->ec_pd_us = netclk_win(NETCLK_WIN_US, &out->ec_pd_edge);

    /* [5] 自检(ITR 路): 已知 100kHz 的 TIM3_TRGO 同时喂 TIM5(ITR1) 与 TIM4(ITR2) */
    netclk_selftest(&out->st_ok, &out->st_edge, &out->st_us,
                    &out->st_w4, &out->st_t3_run);

    /* [5b] 自检(计数 + 定窗路): 自由跑, 不依赖任何触发源 */
    out->fr_ok = netclk_selftest_freerun(&out->fr_cnt, &out->fr_us);

    /* 量测期间 PA1 的 AFR 回读 ⇒ 证明"把脚借给 TIM5"这一动作真的生效 */
    out->af_ref = s_nc_af;

    /* [6] 标定"引脚 → TI2FP2"这段通路(前面四条全都要经过它) */
    netclk_loopback(&out->lb_toggle, &out->lb_follow, &out->lb_ti2);

    /* 收尾: 关定时器 + PA1 还给 ETH */
    TIM5->CR1 = 0U;
    __HAL_RCC_TIM5_CLK_DISABLE();
    netclk_pin_eth();
    s_scan_next = HAL_GetTick() + 200U;         /* 状态机尽快接着跑 */
}

void NetProto_GetStatus(NetProtoStatus* out)
{
    if (!out) return;
    out->state = s_state;
    out->phy_found = s_phy_found;
    out->phy_addr = s_phy_addr;
    out->phy_id = 0x0007U;
    out->link = s_link;
    out->speed100 = s_speed100;
    out->fulldup = s_fulldup;
    out->forced = s_forced;
    out->rx_pkts = s_rx_pkts;
    out->tx_pkts = s_tx_pkts;
    out->udp_cmd = s_udp_cmd;
    out->crc_err = s_crc_err;
    out->tx_err = s_tx_err;
    out->arp_req = s_arp_req;
    out->icmp_req = s_icmp_req;
    out->icmp_calc = s_icmp_calc;
    out->icmp_frame = s_icmp_frame;
    out->icmp_ilen = s_icmp_ilen;
    /* ---- 0920R 发送路径体检: 现场直读寄存器/描述符/池 ---- */
    out->pool24 = s_pool24;
    out->pool36 = s_pool36;
    out->pool_slot = s_pool_slot;
    out->pool38 = s_pool38;
    out->maccr = s_eth.Instance->MACCR;
    {
        int i;
        for (i = 0; i < 4; i++) {
            out->txd0[i] = s_dmtx[i].DESC0;
            out->txd2[i] = s_dmtx[i].DESC2;
            out->rxd2[i] = s_dmrx[i].DESC2;
        }
    }
    (void)s_dispatch;
}

uint8_t NetProto_DebugCtl(uint8_t which, uint8_t val)
{
    uint8_t old = (uint8_t)((s_cic_fix ? 1U : 0U) | (s_icmp_sentinel ? 2U : 0U) | (s_hwck ? 4U : 0U));
    uint8_t v = (uint8_t)(val ? 1U : 0U);
    switch (which) {
    case 0U: s_cic_fix = v; break;
    case 1U: s_icmp_sentinel = v; break;
    case 2U: s_hwck = v; break;
    default: break;                                   /* 0xFF 等: 只读 */
    }
    return old;
}

#else /* !ARM_LINK_NET —— 桩模式（由 arm_net.h 的 inline 提供, 此文件不产码） */
typedef int arm_net_dummy_t;
#endif /* ARM_LINK_NET */
