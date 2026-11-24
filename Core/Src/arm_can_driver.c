/* ============================================================
 * arm_can_driver.c - STM32F407 bxCAN1 driver for arm_proto_can
 *
 * Pins: 默认 PB8 = CAN1_RX, PB9 = CAN1_TX (AF9, 与 .ioc 一致)
 *       可选 PA11 = CAN1_RX, PA12 = CAN1_TX (AF9, 见 CANPIN=pa 开关)
 *       **CAN2 = PB12(RX)/PB13(TX) (AF9) —— 运行时可切, 见 `cx` 命令**
 *
 * 接口(CAN1/CAN2)自 2026-09-18 起是**运行期**变量 s_iface, 由 ArmCanDrv_SetInterface()
 * 改。所以本文件里**任何**对引脚/实例/中断号的引用都必须走 can_rx_port() 这类
 * 取值函数, 不能再直接用宏 —— 用宏就会在"已经切到 CAN2"的时候仍然去读 CAN1。
 *
 * Bit rate: 运行时可选 100/125/250/500/1000 kbit/s
 *   固定 14TQ (1 + BS1=11 + BS2=2, 采样点 85.7%), 只改预分频:
 *     Prescaler = PCLK1 / (kbps * 1000 * 14)
 *   42MHz PCLK1 下: 1000k→3, 500k→6, 250k→12, 125k→24, 100k→30
 *   由 ArmCanDrv_SetBitrate() 在启动时按 arm_can_config 的设置落实。
 *   ⚠ 不能再把预分频写死 —— 上位机 CAN 卡基本都跑 250k/500k 两档,
 *     写死就意味着换一套组网要重编重烧。
 * ============================================================ */
#include "main.h"
#include "arm_proto_can.h"
#include "arm_can_config.h"   /* CAN_CFG_DEF_IFACE —— 开机默认接口(0918H 起 = CAN2) */
#include "arm_runlog.h"
#include <string.h>

CAN_HandleTypeDef hcan1;

#define ARM_CAN_RX_QUEUE_SIZE            16U
#define ARM_CAN_TX_TIMEOUT_MS            20U

typedef struct {
    uint16_t id;
    uint8_t  len;
    uint8_t  data[8];
} ArmCanRxFrame;

static volatile ArmCanRxFrame s_rx_queue[ARM_CAN_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint32_t s_rx_dropped = 0U;
static volatile uint32_t s_tx_frames = 0U;  /* 成功入发送邮箱的协议帧数 */
static uint16_t s_bitrate_kbps = 500U;      /* 当前/目标波特率, 见 SetBitrate */
static volatile uint32_t s_test_tx = 0U;   /* ct: 自检帧已入发送邮箱 */

/* 前向声明: SetBitrate 要复用"恢复成协议正常模式"这段 */
static uint8_t can_restore_normal(void);
static volatile uint32_t s_test_rx = 0U;   /* ct: 自检帧已被本板收到 */
static volatile uint8_t  s_hw_ok   = 0U;   /* CAN 是否已成功初始化 */
/* 是否**曾经**成功初始化过总线。用来区分两种情况:
 *   s_hw_ok=1          -> 总线上跑着, 切接口需要停->重配->再起
 *   s_ever_inited=0    -> 从没起过(启动早期), 切接口只改状态, 不许碰硬件
 * 只有这两条合起来才能让"启动时选接口"不提前把总线拉起来。 */
static uint8_t s_ever_inited = 0U;

/* 当前使用的 CAN 接口(ARM_CAN_IF_CAN1 / ARM_CAN_IF_CAN2)。
 * 只用这一条状态来决定"用哪个控制器、哪对脚、哪个中断号、从哪个银行起配过滤器" ——
 * 这样切换接口只需要改这一个变量 + 一次重新初始化, 不会有"改了一半"的半状态。
 * ⭐ 0918H 起开机默认 = `CAN_CFG_DEF_IFACE` = **CAN2**（本板卡的实际接线, 见其定义处）。
 *    只是初值: `CanCfg_Load()` 落地后会按 flash 配置覆盖(flash 里的记录优先),
 *    `cx 1|2` 运行期随时可切。 */
static uint8_t s_iface = (uint8_t)CAN_CFG_DEF_IFACE;
/* CAN1 的两组候选引脚, **运行期**可切 (2026-09-18 现场):
 *   0 = PB8(RX)/PB9(TX)   —— .ioc 的默认
 *   1 = PA11(RX)/PA12(TX) —— 现有样机的**实际接法**(ps 取证: 灌帧时只有 PA11 出现边沿)
 * 开机默认值仍由编译期 CANPIN 决定(`make ... CANPIN=pa`), 之后可用 `cx pa|pb` 切。
 * ⚠ 不落盘: 复位后回到编译期默认 —— 改 Flash 配置区布局的风险不值得换这点便利。 */
static uint8_t s_can1_pa =
#ifdef ARM_CAN_PINS_PA
    1U;
#else
    0U;
#endif

#define CAN_FILTER_BANKS_PER_IF  4U
/* CAN1 用银行 0..13 中最低的 4 个; CAN2 用 14..27 中最低的 4 个。
 * F407 两颗 bxCAN 共享 28 个过滤器银行, FMR.CAN2SB=SlaveStartFilterBank 决定
 * 分界: 编号 < CAN2SB 的归 CAN1, >= 的归 CAN2。CAN2 的过滤器**必须**落在
 * >= CAN2SB 的编号上, 否则配置成功但永远匹配不上 —— 一种"看起来全对却收不到"的坑。 */
#define CAN_FILTER_CAN2SB        14U
static uint8_t can_filter_base(void)
{
    return (s_iface == ARM_CAN_IF_CAN2) ? (uint8_t)CAN_FILTER_CAN2SB : 0U;
}

static CAN_TypeDef*   can_regs(void)
{
    return (s_iface == ARM_CAN_IF_CAN2) ? CAN2 : CAN1;
}
/* 当前选择那对脚所在端口的时钟。
 * ⚠ HAL_CAN_MspInit 里**不走这个函数**: 那边要把 GPIOA/GPIOB 两个时钟都开,
 *   因为它还要 DeInit 掉没选中的那一组。这里(cp/cw/cz 的引脚级测试)只是确保
 *   下面 HAL_GPIO_Init 之前时钟是开的。 */
static void can_gpio_clk_enable(void)
{
    if (s_iface == ARM_CAN_IF_CAN2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (s_can1_pa) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
}

static GPIO_TypeDef* can_rx_port(void)
{
    if (s_iface == ARM_CAN_IF_CAN2) return ARM_CAN2_RX_PORT;
    return s_can1_pa ? GPIOA : GPIOB;              /* PA11 或 PB8 */
}
static uint16_t can_rx_pin(void)
{
    if (s_iface == ARM_CAN_IF_CAN2) return ARM_CAN2_RX_PIN;
    return s_can1_pa ? GPIO_PIN_11 : GPIO_PIN_8;
}
static GPIO_TypeDef* can_tx_port(void)
{
    if (s_iface == ARM_CAN_IF_CAN2) return ARM_CAN2_TX_PORT;
    return s_can1_pa ? GPIOA : GPIOB;              /* PA12 或 PB9 */
}
static uint16_t can_tx_pin(void)
{
    if (s_iface == ARM_CAN_IF_CAN2) return ARM_CAN2_TX_PIN;
    return s_can1_pa ? GPIO_PIN_12 : GPIO_PIN_9;
}
/* 注: CAN1 的两组候选脚跨 GPIOA(PA11/PA12) 与 GPIOB(PB8/PB9), 下面
 * ArmCanDrv_SetCan1Pins() 切换时会显式 DeInit 掉旧那组, 所以 MspInit 里
 * 两个端口时钟都必须开 —— 见 HAL_CAN_MspInit。 */

uint8_t ArmCanDrv_GetInterface(void) { return s_iface; }

const char* ArmCanDrv_Can1PinDesc(uint8_t which)
{
    return (which == ARM_CAN1_PINS_PA) ? "PA11(RX)/PA12(TX)" : "PB8(RX)/PB9(TX)";
}

uint8_t ArmCanDrv_GetCan1Pins(void) { return s_can1_pa; }

const char* ArmCanDrv_PinDesc(void)
{
    return (s_iface == ARM_CAN_IF_CAN2) ? ARM_CAN2_PIN_DESC
                                        : ArmCanDrv_Can1PinDesc(s_can1_pa);
}

const char* ArmCanDrv_IfName(void)
{
    return (s_iface == ARM_CAN_IF_CAN2) ? "CAN2" : "CAN1";
}

/* bxCAN 原始寄存器只读访问 —— 一定要走 can_regs(), 否则会读到"另一个控制器" */
uint32_t ArmCanDrv_RegMCR(void) { return can_regs()->MCR; }
uint32_t ArmCanDrv_RegMSR(void) { return can_regs()->MSR; }
uint32_t ArmCanDrv_RegESR(void) { return can_regs()->ESR; }
uint32_t ArmCanDrv_RegTSR(void) { return can_regs()->TSR; }
uint8_t  ArmCanDrv_BusRxLevel(void)
{
    return (uint8_t)((can_regs()->MSR >> 11) & 1UL);
}

/* ---- 波特率 -> 预分频 (2026-09-17) ----
 * 固定 14TQ(1+BS1 11+BS2 2), 采样点 (1+11)/14 = 85.7% —— 这是 CAN 的
 * 常用取值, 各档之间保持一致意味着**改波特率不需要重新评估采样点**。
 * 返回 0 表示该波特率在本板 PCLK1 下无法用 14TQ 整除得到(拒绝配置),
 * 而不是"四舍五入给个近似值" —— 波特率偏一点就是偶发 ACK 错, 极难查。 */
static uint16_t can_prescaler_for(uint16_t kbps)
{
    uint32_t pclk1;
    uint32_t div;

    if (kbps == 0U) return 0U;
    pclk1 = HAL_RCC_GetPCLK1Freq();
    if (pclk1 == 0U) return 0U;
    div = pclk1 / ((uint32_t)kbps * 1000U * 14U);
    if (div == 0U || div > 1024U) return 0U;
    if (pclk1 != ((uint32_t)kbps * 1000U * 14U * div)) return 0U;
    return (uint16_t)div;
}

/* 按当前 s_bitrate_kbps 填 hcan1.Init 的时间参数(不动 Mode 以外的使用者)。 */
static uint8_t can_apply_timing(uint32_t mode)
{
    uint16_t ps = can_prescaler_for(s_bitrate_kbps);
    if (ps == 0U) return 0U;

    hcan1.Instance = can_regs();
    hcan1.Init.Prescaler = ps;
    hcan1.Init.Mode = mode;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    /* AutoRetransmission:
     * 正常模式关(见下), 回环自检必须开 —— 回环下帧是本机自发自收, 但
     * 若没人"应答"也会重传, 关掉会让自检直接失败。 */
    hcan1.Init.AutoRetransmission =
        (mode == CAN_MODE_NORMAL) ? DISABLE : ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    return 1U;
}

/* ---- 接收过滤器 (2026-09-17: 从 3 个银行扩到 4 个; 2026-09-18: 银行号随接口走) ----
 *  bank0: 0x100..0x10F  命令方向  上位机 -> 机械臂
 *  bank1: 0x180..0x18F  响应方向  机械臂 -> 上位机(本机自发时也要能收)
 *  bank2: 0x280..0x28F  状态/事件 机械臂 -> 上位机(周期状态帧 0x53 / 事件帧 0x45)
 *  bank3: 0x7E0..0x7EF  链路自检  (不过协议层, 见 ct 命令)
 * 掩码 0x7F0 = "高 7 位参与比较, 低 4 位任意" ⇒ 一个银行覆盖 16 个 ID。
 * ⚠ 过去只开 bank0 时**从臂的回包在硬件层就被丢掉了**, 主臂永远看不到
 *   "从臂到底回了几帧"; 同理, 少了 bank2 上位机(或另一块监听板)看不到状态帧。
 * ⚠ 2026-09-18: 银行号**不能写死成 0..3**。CAN2 只看编号 >= CAN2SB(14) 的银行,
 *   把 CAN2 的过滤器配到 0..3 会"配置成功但一帧都收不到"; 反过来 CAN1 也只看
 *   编号 < CAN2SB 的。所以基址由 can_filter_base() 按当前接口给。 */
static const uint16_t s_filter_ids[CAN_FILTER_BANKS_PER_IF] = {
    0x100U, 0x180U, 0x280U, 0x7E0U
};

/* enable=1 装好并启用; enable=0 只关掉这 4 个银行(切换接口时用)。
 * ⚠ 即使只关也要走 HAL_CAN_ConfigFilter: FMR.CAN2SB 只在这个函数里被写。 */
static uint8_t can_config_filters_ex(uint8_t base, uint8_t enable)
{
    CAN_FilterTypeDef f = {0};

    for (uint8_t i = 0U; i < CAN_FILTER_BANKS_PER_IF; i++) {
        f.FilterBank = (uint32_t)(base + i);
        f.FilterMode = CAN_FILTERMODE_IDMASK;
        f.FilterScale = CAN_FILTERSCALE_32BIT;
        f.FilterIdHigh = (uint16_t)(s_filter_ids[i] << 5);
        f.FilterIdLow = 0U;
        f.FilterMaskIdHigh = (uint16_t)(0x7F0U << 5);
        f.FilterMaskIdLow = 0U;
        f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        f.FilterActivation = enable ? ENABLE : DISABLE;
        f.SlaveStartFilterBank = CAN_FILTER_CAN2SB;
        if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t can_config_filters(void)
{
    return can_config_filters_ex(can_filter_base(), 1U);
}

uint8_t ArmCanDrv_HwInit(void)
{
    if (!can_apply_timing(CAN_MODE_NORMAL)) {
        RLOG_E("CAN", "unsupported bitrate %u kbit/s (PCLK1=%lu Hz)",
               (unsigned)s_bitrate_kbps, (unsigned long)HAL_RCC_GetPCLK1Freq());
        return 0U;
    }
    /* AutoRetransmission = DISABLE (2026-09-14):
     * 点对点直连场景下"对端不在/线没插"是常态。开着自动重传时, 无人 ACK
     * 的帧会一直占着发送邮箱 → HAL_CAN_GetTxMailboxesFreeLevel() 永远 0
     * → ArmCanDrv_Tx 每次卡满 20 ms 超时 → **主臂主循环被拖死**。
     * 关掉后丢帧就丢帧 (33 ms 后下一帧照来), 邮箱立即释放。
     * ⚠ 代价: 总线受扰时不再自动重发。上位机侧需自己按 SEQ 做超时重试。 */
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return 0U;
    }

    /* 接收过滤: 命令 + 响应 + 状态/事件 + 自检 四个 ID 段 */
    if (!can_config_filters()) {
        return 0U;
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_dropped = 0U;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return 0U;
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return 0U;
    }
    s_hw_ok = 1U;
    s_ever_inited = 1U;
    RLOG_I("CAN", "%s %s, %lu bit/s (PS=%u)",
           ArmCanDrv_IfName(), ArmCanDrv_PinDesc(),
           (unsigned long)((uint32_t)s_bitrate_kbps * 1000U),
           (unsigned)can_prescaler_for(s_bitrate_kbps));
    return 1U;
}

uint16_t ArmCanDrv_GetBitrate(void) { return s_bitrate_kbps; }

uint8_t ArmCanDrv_SetBitrate(uint16_t kbps)
{
    if (can_prescaler_for(kbps) == 0U) return 0U;
    if (kbps == s_bitrate_kbps) return 1U;
    s_bitrate_kbps = kbps;
    if (!s_hw_ok) return 1U;      /* 还没初始化: 由 HwInit 一次性落实 */

    /* 已初始化: 停 -> DeInit -> 用新预分频重来。
     * ⚠ 改波特率会让总线上的其它节点立刻失联 —— 现场必须两端一起改。 */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    s_hw_ok = 0U;
    return can_restore_normal();
}

uint32_t ArmCanDrv_TxCount(void)     { return s_tx_frames; }
uint32_t ArmCanDrv_RxDropCount(void) { return s_rx_dropped; }

uint8_t ArmCanDrv_Tx(uint16_t can_id, const uint8_t* data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint32_t start;

    if (data == NULL || len > 8U || can_id > 0x7FFU) {
        return 0U;
    }
    start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((uint32_t)(HAL_GetTick() - start) >= ARM_CAN_TX_TIMEOUT_MS) {
            RLOG_W("CAN", "TX mailbox timeout");
            return 0U;
        }
    }

    header.StdId = can_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t*)data, &mailbox) != HAL_OK) {
        return 0U;
    }
    s_tx_frames++;
    return 1U;
}

/* 等到 3 个发送邮箱**全部空出**(即已挂起的帧都已真的发完 / 被放弃)。
 * 用途: 多帧打包流必须"一帧真发完再排下一帧"。否则第 4 帧会抢走刚空出的 MB0,
 * 硬件按"ID 相同则邮箱号小者先发"调度 ⇒ 总线顺序变成 F1,F4,F2,F3。
 * 判据用 freeLevel==3 而不是"某个邮箱空闲": 后者只保证有位置放, 不保证前几帧发完。
 * NART=1 时发送失败也会立刻放弃并释放邮箱, 所以这里**不会被无 ACK 卡死**。 */
uint8_t ArmCanDrv_TxDrain(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
            return 0U;
        }
    }
    return 1U;
}

uint8_t ArmCanDrv_Rx(uint16_t* can_id, uint8_t* data, uint8_t* len)
{
    uint8_t tail;
    if (can_id == NULL || data == NULL || len == NULL || s_rx_tail == s_rx_head) {
        return 0U;
    }
    tail = s_rx_tail;
    *can_id = s_rx_queue[tail].id;
    *len = s_rx_queue[tail].len;
    memcpy(data, (const void*)s_rx_queue[tail].data, *len);
    s_rx_tail = (uint8_t)((tail + 1U) % ARM_CAN_RX_QUEUE_SIZE);
    return 1U;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t head;
    uint8_t next;

    if (hcan != &hcan1 || HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        return;
    }
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U) {
        return;
    }
    /* 链路自检帧 (0x7E0): 只计数, 不进队列 —— 不能让它污染协议重组 */
    if (header.StdId == ARM_CAN_TEST_ID) {
        s_test_rx++;
        return;
    }
    head = s_rx_head;
    next = (uint8_t)((head + 1U) % ARM_CAN_RX_QUEUE_SIZE);
    if (next == s_rx_tail) {
        s_rx_dropped++;
        return;
    }
    s_rx_queue[head].id = (uint16_t)header.StdId;
    s_rx_queue[head].len = (uint8_t)header.DLC;
    memcpy((void*)s_rx_queue[head].data, data, header.DLC);
    s_rx_head = next;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t is_can2;

    /* 两颗 bxCAN 都支持: 具体哪一颗由 hcan->Instance 决定(MspInit 每次
     * HAL_CAN_Init 都会按当时设置的 Instance 重新调用)。 */
    if (hcan->Instance != CAN1 && hcan->Instance != CAN2) {
        return;
    }
    is_can2 = (hcan->Instance == CAN2) ? 1U : 0U;

    /* ⚠ CAN2 的寄存器与**过滤器**都要经 CAN1(主实例)访问, 所以 CAN1 的时钟
     *   在 CAN2 模式下也必须开着 —— 只开 CAN2 会直接硬件错误。 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    if (is_can2) {
        __HAL_RCC_CAN2_CLK_ENABLE();
    }
    /* 两个端口时钟都开: CAN1 的两组候选脚跨 GPIOA/GPIOB, 下面还要 DeInit
     * 掉没选中的那一组(时钟没开就写寄存器 = 白写)。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 先摘掉 CAN1 的**两组**候选脚, 再按当前选择装上去。
     * 不摘的后果: 从 pa 切到 pb 之后, 两个 CAN_RX(PA11 与 PB8)会同时挂在
     * CAN1 的输入上互相打架 —— 现场表现是偶发位错误, 比"完全收不到"更难查。 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    gpio.Pin = is_can2 ? (uint32_t)(ARM_CAN2_RX_PIN | ARM_CAN2_TX_PIN)
                       : (uint32_t)(can_rx_pin() | can_tx_pin());
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    /* CAN1 与 CAN2 的复用功能号**都是 9**, 但 HAL 里是两个宏, 按实例选才不会看错 */
    gpio.Alternate = is_can2 ? GPIO_AF9_CAN2 : GPIO_AF9_CAN1;
    HAL_GPIO_Init(can_rx_port(), &gpio);
    if (can_tx_port() != can_rx_port()) {
        gpio.Pin = (uint32_t)can_tx_pin();
        HAL_GPIO_Init(can_tx_port(), &gpio);
    }

    /* 只开当前这一路的中断 —— 另一路的向量留着, 但永不被触发 */
    if (is_can2) {
        HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    } else {
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    }
}

/* ============================================================
 * cx: 运行时切换 CAN 接口 (2026-09-18)
 *
 * 动机见 arm_proto_can.h 的 ARM_CAN_IF_* 注释: 现场有一个"卡其实插在板子的
 * 另一个 CAN 口上"的假设, 而验证它过去要重烧一次固件 —— 一次半小时, 还要
 * 手动拉 BOOT0。做成运行时之后, 一次烧录就能 `cx 2` 直接试 CAN2。
 *
 * 顺序很讲究:
 *   1) 停总线 + 关中断 + **关掉旧那 4 个过滤器银行**
 *      (两级 CAN 共享 28 个银行, 旧银行留着会在切回来时与新的抢帧)
 *   2) DeInit(把引脚从 AF 上摘下来)
 *   3) 两个 CAN RX0 中断向量都 Disable(避免切换瞬间的悬挂中断)
 *   4) 改 s_iface —— 从这一步起所有取值函数都指向新接口
 *   5) can_restore_normal(): 按新接口重配 GPIO/NVIC/过滤器/中断并重新上总线
 * ============================================================ */
uint8_t ArmCanDrv_SetInterface(uint8_t iface)
{
    uint8_t base;

    if (iface != ARM_CAN_IF_CAN1 && iface != ARM_CAN_IF_CAN2) {
        return 0U;
    }
    if (iface == s_iface) {
        return 1U;   /* 已经是这一路, 幂等 */
    }

    /* 还没上过总线(启动阶段调用 / 之前 hw init 就没成功):
     * **不要**在这里初始化硬件 —— 启动流程要求"接口 + 波特率都定下来之后"
     * 才由 CanProto_Init() 一次性起总线, 早起会在总线上留一段错误波特率的窗口。
     * 所以这里只改状态, 硬件交给后面的 HwInit/can_restore_normal()。 */
    if (!s_ever_inited) {
        s_iface = iface;
        RLOG_I("CAN", "interface (deferred) -> %s %s",
               ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        return 1U;
    }

    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 关掉旧那一组银行(CAN2SB 只在 ConfigFilter 里写, 所以必须走它) */
    if (s_hw_ok) {
        base = can_filter_base();
        (void)can_config_filters_ex(base, 0U);
    }

    HAL_CAN_DeInit(&hcan1);
    s_hw_ok = 0U;

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

    s_iface  = iface;
    s_rx_head = 0U;
    s_rx_tail = 0U;

    if (!can_restore_normal()) {
        return 0U;
    }
    RLOG_I("CAN", "interface -> %s %s", ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
    return 1U;
}

/* ============================================================
 * cx pa / cx pb: 运行时切换 **CAN1 的那一对脚** (2026-09-18 现场)
 *
 * 起因是一次代价很大的误判: 板子的 CAN1 收发器实际接在 PA11(RX)/PA12(TX),
 * 固件却按 .ioc 驱动 PB8/PB9 ⇒ 现象是「卡在拼命发、臂侧 MSR.RX 恒 1、
 * 四个收帧计数器全 0」——**和"线没接/接错端子"一模一样**。定位它花了一整轮,
 * 因为改一次引脚假设就得拉 BOOT0 重烧一次。
 * 做成运行期后: 烧一次, 之后 `cx pa` / `cx pb` 直接换, 再不用重烧。
 *
 * 顺序与 SetInterface 一致(两条路径都是"停总线 -> 摘引脚 -> 重配 -> 再上"):
 *   1) 停总线 + 关 RX 通知 + 关掉旧过滤器银行
 *   2) DeInit(把引脚从 AF 上摘下来)
 *   3) 两个 RX0 向量都 Disable
 *   4) 把**两组**候选脚显式 DeInit(只摘当前那组不够, 见下面注释)
 *   5) 改 s_can1_pa -> can_restore_normal() 按新脚重配并重新上总线
 * ============================================================ */
uint8_t ArmCanDrv_SetCan1Pins(uint8_t which)
{
    uint8_t base;

    if (which != ARM_CAN1_PINS_PB && which != ARM_CAN1_PINS_PA) {
        return 0U;
    }
    if (which == s_can1_pa) {
        return 1U;                    /* 幂等 */
    }

    /* 还没起过总线(启动阶段 / hw init 从未成功), 或当前根本不在 CAN1 上:
     * **不要**在这里动硬件 —— 引脚是在 HAL_CAN_MspInit 里按当时的 s_can1_pa
     * 配的, 所以只记状态, 等下次上总线自然生效。 */
    if (!s_ever_inited || s_iface == ARM_CAN_IF_CAN2) {
        s_can1_pa = which;
        RLOG_I("CAN", "CAN1 pins (deferred) -> %s", ArmCanDrv_Can1PinDesc(which));
        return 1U;
    }

    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (s_hw_ok) {
        base = can_filter_base();
        (void)can_config_filters_ex(base, 0U);
    }

    HAL_CAN_DeInit(&hcan1);           /* 会经 MspDeInit 摘掉当前那组引脚 */
    s_hw_ok = 0U;

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

    /* ⚠ 只摘 HAL_CAN_DeInit 摘掉的那一组是不够的: 若旧那组引脚还留在 AF 上,
     *   切换后两个 CAN_RX(PA11 与 PB8)会同时驱动 CAN1 的输入 —— 表现为偶发
     *   位错误, 比"完全收不到"难查得多。所以两组都显式摘掉。 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    s_can1_pa = which;
    s_rx_head = 0U;
    s_rx_tail = 0U;

    if (!can_restore_normal()) {
        return 0U;
    }
    RLOG_I("CAN", "CAN1 pins -> %s", ArmCanDrv_PinDesc());
    return 1U;
}

/* ---- CAN Loopback self-test ---- */
/* return: 1=PASS, 0=RX timeout, 0xFE=init failed, 0xFF=TX failed */
uint8_t ArmCanDrv_LoopbackTest(void)
{
    CAN_FilterTypeDef filter = {0};
    CAN_TxHeaderTypeDef txhdr = {0};
    CAN_RxHeaderTypeDef rxhdr;
    uint8_t txdata[8] = {0xA5, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    uint8_t rxdata[8] = {0};
    uint32_t mailbox;
    uint32_t start;
    uint8_t ok = 0;

    /* Stop and fully reset CAN controller before changing mode */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);

    /* Re-init in Loopback mode (用与协议一致的波特率, 否则自检通过≠通信能通) */
    if (!can_apply_timing(CAN_MODE_LOOPBACK)) {
        return 0xFE;
    }

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return 0xFE;
    }

    /* Configure filter to accept all in loopback (银行号随接口, 见 can_filter_base) */
    filter.FilterBank = (uint32_t)can_filter_base();
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CAN_FILTER_CAN2SB;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        return 0xFE;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return 0xFE;
    }

    /* Send test frame */
    txhdr.StdId = 0x123;
    txhdr.ExtId = 0;
    txhdr.IDE = CAN_ID_STD;
    txhdr.RTR = CAN_RTR_DATA;
    txhdr.DLC = 8;
    txhdr.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &txhdr, txdata, &mailbox) != HAL_OK) {
        HAL_CAN_DeInit(&hcan1);
        return 0xFF;  /* TX failed */
    }

    /* Wait for loopback receive (poll FIFO0 directly, not IRQ queue) */
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 500U) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxhdr, rxdata) == HAL_OK) {
                if (rxhdr.StdId == 0x123 && rxdata[0] == 0xA5 && rxdata[1] == 0x5A) {
                    ok = 1;
                }
                break;
            }
        }
    }

    /* Restore Normal mode (复用统一入口: 波特率/过滤器/中断一次装好) */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    (void)can_restore_normal();

    return ok;
}

/* ============================================================
 * ct 链路自检 (2026-09-14) —— CAN 版的"帧计数证明"
 *
 * 为什么需要它: UART 那一轮花了很多轮也没能证明"线到底通没通", 因为字节数
 * 会被浮空脚蒙混。CAN 有硬件 CRC + 自动 ACK, 所以判据极简:
 *   A 板执行 `ct 50`  -> 发 50 帧到 0x7E0
 *   B 板执行 `ct`     -> 打印收到了几帧
 * 两边的数对上 = 物理链路(收发器/双绞线/终端电阻/共地)真的通了。
 * ============================================================ */

uint16_t ArmCanDrv_SendTestFrames(uint16_t n)
{
    uint8_t  d[8];
    uint16_t ok = 0U;

    for (uint16_t i = 0U; i < n; i++) {
        d[0] = ARM_CAN_TEST_MAGIC0;
        d[1] = ARM_CAN_TEST_MAGIC1;
        d[2] = (uint8_t)(i & 0xFFU);
        d[3] = (uint8_t)(i >> 8);
        d[4] = 0U; d[5] = 0U; d[6] = 0U; d[7] = 0U;
        if (ArmCanDrv_Tx((uint16_t)ARM_CAN_TEST_ID, d, 8U)) {
            ok++;
            s_test_tx++;
        }
        HAL_Delay(2);   /* 给总线与邮箱留时间, 别把三个邮箱打满 */
    }
    return ok;
}

uint32_t ArmCanDrv_TestRxCount(void) { return s_test_rx; }
uint32_t ArmCanDrv_TestTxCount(void) { return s_test_tx; }

/* ============================================================
 * cp 引脚探测 (2026-09-14)
 * 把 CAN 引脚临时切成普通 GPIO, 直接量电平 —— 见 arm_proto_can.h 注释。
 * ============================================================ */
static uint8_t can_sample8(GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t b = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
            b |= (uint8_t)(1U << i);
        }
    }
    return b;
}

/* 把 CAN1 恢复成协议用的正常模式(过滤器/中断全部重装)。返回 1=OK。 */
static uint8_t can_restore_normal(void)
{
    if (!can_apply_timing(CAN_MODE_NORMAL)) {
        s_hw_ok = 0U;
        return 0U;
    }
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        s_hw_ok = 0U;
        return 0U;
    }
    if (!can_config_filters()) {
        s_hw_ok = 0U;
        return 0U;
    }
    s_rx_head = 0U;
    s_rx_tail = 0U;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        s_hw_ok = 0U;
        return 0U;
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        s_hw_ok = 0U;
        return 0U;
    }
    s_hw_ok = 1U;
    s_ever_inited = 1U;
    return 1U;
}

uint8_t ArmCanDrv_PinProbe(uint8_t* rx_pd, uint8_t* rx_pu,
                           uint8_t* tx_lo, uint8_t* tx_hi)
{
    GPIO_InitTypeDef g = {0};

    if (rx_pd == NULL || rx_pu == NULL || tx_lo == NULL || tx_hi == NULL) {
        return 0U;
    }

    /* 先把 CAN 内核彻底停掉, 引脚才能当普通 IO 用 */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    can_gpio_clk_enable();

    /* --- RX 脚: 输入 + 内部下拉 / 内部上拉, 各连采 8 次 --- */
    g.Pin = can_rx_pin();
    g.Mode = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(can_rx_port(), &g);
    *rx_pd = can_sample8(can_rx_port(), can_rx_pin());
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(can_rx_port(), &g);
    *rx_pu = can_sample8(can_rx_port(), can_rx_pin());

    /* --- TX 脚: 推挽输出自检 (证明该脚没被短死) --- */
    g.Pin = can_tx_pin();
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(can_tx_port(), &g);
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_RESET);
    *tx_lo = can_sample8(can_tx_port(), can_tx_pin());
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_SET);
    *tx_hi = can_sample8(can_tx_port(), can_tx_pin());

    /* --- 恢复 CAN (MspInit 会把这两只脚重新配成 AF9) --- */
    return can_restore_normal();
}

/* ============================================================
 * cw 外环测试 (2026-09-17) —— 回答 cp 回答不了的那个问题
 *
 * cp 只验了两件"局部"的事: ① RX 脚被外部驱高(模块上电) ② TX 脚自己能推挽。
 * 但它两次读的都是**引脚自己**, 完全没碰
 *     TX -> 收发器 TXD -> 收发器驱动总线 -> 收发器 RXD -> RX
 * 这条**外环**。TXD 那根线断掉时 cp 依然全绿, 症状却是"帧发出去了却
 * 谁也收不到"(总线恒定隐性)。这在 UART 那一轮就吃过一次。
 *
 * cw 的做法: 停掉 bxCAN, 把 TX 脚当普通 GPIO **持续驱动显性(低)**,
 * 直接读 RX 脚。标准 CAN 收发器(TJA1050/SN65HVD230/TJA1042...)在正常
 * 模式下会把**总线状态**回馈到 RXD ⇒ 这一刻本机 RX 必须跟着变低。
 * 不需要万用表, 也不需要总线上有第二个节点 —— 单节点自环即可。
 *
 * 返回 1 = 探测完成且 CAN 已恢复。
 * dom/rec = TX 驱动显性/隐性时 RX 脚的 8 次采样位图。
 * ============================================================ */
uint8_t ArmCanDrv_WireLoopTest(uint8_t* dom, uint8_t* rec)
{
    GPIO_InitTypeDef g = {0};

    if (dom == NULL || rec == NULL) {
        return 0U;
    }

    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    can_gpio_clk_enable();

    /* RX 脚: 纯输入(与平时 AF 输入的角色一致, 唯一区别是不接 bxCAN) */
    g.Pin = can_rx_pin();
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(can_rx_port(), &g);

    /* TX 脚: 推挽输出 —— 我们直接扮演 bxCAN 的 TXD */
    g.Pin = can_tx_pin();
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(can_tx_port(), &g);

    /* 驱动显性(TXD=低): 收发器会立刻把 CANH 拉高 / CANL 拉低, 并把总线
     * 状态透传到 RXD ⇒ 本机 RX 应变低。 */
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_RESET);
    HAL_Delay(2);
    *dom = can_sample8(can_rx_port(), can_rx_pin());

    /* 驱动隐性(TXD=高) */
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_SET);
    HAL_Delay(2);
    *rec = can_sample8(can_rx_port(), can_rx_pin());

    return can_restore_normal();
}

/* 持续驱动总线显性 ms 毫秒, 然后恢复。
 * 用途: 普通万用表只有 ~3 次/秒刷新率, bxCAN 那种间歇发送(每帧 ~190us)
 * 它根本抓不到。把 TXD **按住不放**, 就能在**卡那一端**的 H/L 端子上量到
 * 稳定电平 —— 这是"线到底通没通"最直白的证据:
 *   卡端量到 CANH≈3.5V / CANL≈1.5V  ⇒ 两根线都通到卡了
 *   卡端仍是 CANH≈CANL≈2.5V        ⇒ 卡没接到臂的这一对端子上
 * ⚠ 显性期间若总线上还有别的活节点, 它们会看到一串错误 —— 联调时无所谓。 */
uint8_t ArmCanDrv_DriveDominant(uint16_t ms)
{
    GPIO_InitTypeDef g = {0};

    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    can_gpio_clk_enable();

    g.Pin = can_tx_pin();
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(can_tx_port(), &g);
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_RESET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(can_tx_port(), can_tx_pin(), GPIO_PIN_SET);

    return can_restore_normal();
}

/* ============================================================
 * ps 引脚嗅探 (2026-09-18)
 *
 * 为什么需要它: cp / cw 都只认"当前这一组 CAN 脚"(默认 PB8/PB9)。它们回答不了
 * 更前面的那个问题 —— **板子上是不是还有第二个 CAN 口? 手里的 USB-CAN 到底挂在
 * 哪只脚上?** 这两种情况的现场症状一模一样: 帧发得出去(TEC+8/lec=3)、谁也收不到、
 * RX 脚一动不动。而用户对"我接的就是 CAN1"的判断往往来自丝印, 未必等于 MCU 引脚。
 *
 * 做法: 把所有与 CAN 有关的候选脚逐个切成 GPIO 输入, 在几百微秒内连采 N 次,
 * 统计"低电平次数"和"跳变次数"; 再用内部下拉复采一遍, 用来区分"悬空"与
 * "被外部驱动"(悬空脚会被 40k 内部下拉轻松拉满, 外部驱动的不会)。
 *
 * ⚠ 必须有输入源才说明问题: **在卡那一端持续灌帧的同时跑**, 否则悬空脚一律
 *   读成"没东西", 只能证明"没在发", 不能证明"接错脚"。
 *
 * ⚠ 2026-09-18 更新: 结论若是"★落在 PB12/PB13"(= 卡接在板子的 CAN2 口),
 *   **不用重烧固件**了 —— 直接 `cx 2` 把控制器+引脚切到 CAN2 再验一遍。
 * ============================================================ */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    const char*   name;
} can_scan_pin_t;

static const can_scan_pin_t s_scan_pins[ARM_CAN_PINSCAN_N] = {
    { GPIOA, GPIO_PIN_11, "PA11" },   /* CAN1_RX 备选 */
    { GPIOA, GPIO_PIN_12, "PA12" },   /* CAN1_TX 备选 */
    { GPIOB, GPIO_PIN_6,  "PB6"  },   /* CAN2_TX 备选(PB5/PB6 组) */
    { GPIOB, GPIO_PIN_8,  "PB8"  },   /* CAN1_RX ← 本工程在用 */
    { GPIOB, GPIO_PIN_9,  "PB9"  },   /* CAN1_TX ← 本工程在用 */
    { GPIOB, GPIO_PIN_12, "PB12" },   /* CAN2_RX (F407, AF9) */
    { GPIOB, GPIO_PIN_13, "PB13" },   /* CAN2_TX (F407, AF9) */
    { GPIOB, GPIO_PIN_14, "PB14" },   /* F407 上无 CAN 复用, 但板子可能这么引 */
    { GPIOB, GPIO_PIN_15, "PB15" },   /* 同上 */
    { GPIOD, GPIO_PIN_0,  "PD0"  },   /* CAN1_RX 备选 */
    { GPIOD, GPIO_PIN_1,  "PD1"  },   /* CAN1_TX 备选 */
};

/* 这只脚是不是"当前编译版本/当前接口正在用的 CAN 脚"。
 * 是的话扫描后不置回模拟输入, 交给 can_restore_normal() 重配回 AF9。
 * ⚠ 2026-09-18: 接口运行时可切, 所以这里要按 s_iface 取, 不能再用编译期宏 ——
 *   切到 CAN2 之后还在按 PB8/PB9 判断, 会把 CAN2 那两只脚误置成模拟输入。 */
static uint8_t can_scan_pin_is_can(GPIO_TypeDef* port, uint16_t pin)
{
    if (port == can_rx_port() && pin == can_rx_pin()) {
        return 1U;
    }
    if (port == can_tx_port() && pin == can_tx_pin()) {
        return 1U;
    }
    return 0U;
}

const char* ArmCanDrv_PinScanName(uint8_t idx)
{
    return (idx < ARM_CAN_PINSCAN_N) ? s_scan_pins[idx].name : "??";
}

uint8_t ArmCanDrv_PinScanCount(void)
{
    return (uint8_t)ARM_CAN_PINSCAN_N;
}

uint8_t ArmCanDrv_PinScan(ArmCanPinStat_t* st)
{
    GPIO_InitTypeDef g = {0};

    if (st == NULL) {
        return 0U;
    }

    /* 候选脚里含 CAN 自己那两只, 必须先把外设摘掉才能当普通 IO 用 */
    HAL_CAN_Stop(&hcan1);
    HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_DeInit(&hcan1);
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    for (uint8_t i = 0U; i < ARM_CAN_PINSCAN_N; i++) {
        const can_scan_pin_t* p = &s_scan_pins[i];
        uint16_t low = 0U, edge = 0U, low_pd = 0U;
        uint8_t  prev = 0xFFU;

        /* --- 第一遍: 纯输入, 直接读 IDR(不走 HAL 函数, 省掉调用开销) --- */
        g.Pin   = p->pin;
        g.Mode  = GPIO_MODE_INPUT;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(p->port, &g);
        for (uint16_t k = 0U; k < ARM_CAN_SCAN_SAMPLES; k++) {
            uint8_t lv = ((p->port->IDR & p->pin) != 0U) ? 1U : 0U;
            if (lv == 0U) {
                low++;
            }
            if (prev != 0xFFU && prev != lv) {
                edge++;
            }
            prev = lv;
        }

        /* --- 第二遍: 内部下拉复采。悬空脚会被拉满(全低), 被外部驱动的不会 --- */
        g.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(p->port, &g);
        for (uint16_t k = 0U; k < ARM_CAN_SCAN_SAMPLES; k++) {
            if ((p->port->IDR & p->pin) == 0U) {
                low_pd++;
            }
        }

        st[i].low    = low;
        st[i].edge   = edge;
        st[i].low_pd = low_pd;
        st[i].total  = (uint16_t)ARM_CAN_SCAN_SAMPLES;
    }

    /* 非 CAN 的候选脚扫完置回复位态(模拟输入), 不留浮空输入;
     * CAN 那两只交给 can_restore_normal() 重配回 AF9。 */
    for (uint8_t i = 0U; i < ARM_CAN_PINSCAN_N; i++) {
        if (can_scan_pin_is_can(s_scan_pins[i].port, s_scan_pins[i].pin)) {
            continue;
        }
        g.Pin   = s_scan_pins[i].pin;
        g.Mode  = GPIO_MODE_ANALOG;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(s_scan_pins[i].port, &g);
    }

    return can_restore_normal();
}

/* bxCAN 错误状态。ESR:
 *   TEC[23:16] 发送错误计数 (≥256 = error-passive; AutoBusOff 会自动恢复)
 *   REC[31:24] 接收错误计数
 *   LEC[2:0]   最近一次错误码 0=无错 1=位填充 2=格式 3=ACK 4=位隐性 5=位显性 6=CRC
 * ⚠ "LEC=3(ACK)" 是"对端不在 / 没人应答"最直接的证据 —— 线插了但对面没上电时看这个。 */
uint8_t ArmCanDrv_ErrorState(uint8_t* tec, uint8_t* rec, uint8_t* lec)
{
    uint32_t esr = can_regs()->ESR;
    if (tec != NULL) *tec = (uint8_t)((esr >> 16) & 0xFFU);
    if (rec != NULL) *rec = (uint8_t)((esr >> 24) & 0xFFU);
    if (lec != NULL) *lec = (uint8_t)(esr & 0x07U);
    return s_hw_ok;
}
