/* ============================================================
 * arm_rc.c — RC 遥控输入层: PS2 手柄解码 (2026-09-24, R9DS 方案废除)
 *
 * 引脚 (0924M 定案): CS=PB1(输出) SCK=PB3 MISO=PB4 MOSI=PB5
 *   ⭐0924CW: **软件 bit-bang 取代硬件 SPI1**。硬件 SPI1 @APB2 84MHz/256≈328kHz
 *   已是最慢档, 对这只 2.4G 克隆接收器每字节末位(MSB)随机采样错
 *   (0x73→F3 / 0x41→C1 / 0xFF→7F / 0x80→00, 粘性错位, ESP32 同接收器
 *   慢速 bit-bang 字节全干净 ⇒ 时序不足非协议问题)。bit-bang 半周期
 *   5µs(≈50kHz) 换 1.4ms/帧, 50Hz 轮询预算内绰绰有余。
 *   接线: 插头 1 DAT→PB4  2 CMD→PB5  4 GND  5 VCC3.3  6 ATT→PB1  7 CLK→PB3。
 * 协议: 手册 V1.5 —— CS 拉低, 发 0x01 0x42 + 7 字节 0x00, 同位收 DAT;
 *   Data[1]=ID(0x73 红灯模拟/0x41 绿灯数字), Data[2]=0x5A 标记,
 *   Data[3]/Data[4]=按键(按下=0), Data[5..8]=RX RY LX LY (0~255, 中位~128)。
 *
 * ⭐ 本文件不放打印/映射/运动代码: 打印在 main.c `rc` 命令里; 运动在 arm_rc_ctrl。
 * ⭐ 对上层契约: 6 路脉宽(1000~2000, 中位 1500) + fresh/live + w_min/w_max。
 *   摇杆→脉宽的**方向吸收在本层**(上/右=+), ctrl 层实测确认过的 RC_SIGN_* 不动。
 * ============================================================ */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_rc.h"
#include "flash_cfg.h"
#include <stddef.h>
#include <string.h>

/* ---------------------------------------------------------------- */
/* 状态                                                              */
/* ---------------------------------------------------------------- */
static ArmRcState s_state;
static uint8_t    s_live_prev;
static uint32_t   s_last_cfg_ms;     /* 上次跑 SetInit 的时刻            */
static uint32_t   s_exit_try_ms;     /* 0924CP: 上次补发退出帧的时刻     */
static uint8_t    s_exit_tries;      /* 0924CQ: 退出帧已尝试次数 (升级用) */
static uint16_t   s_claw_us = 1500U; /* CH5 夹爪合成绝对量               */
static uint32_t   s_last_poll_ms;    /* 上次(不论成败)尝试轮询的时刻     */
static uint8_t    s_dwt_ok;
static uint8_t    s_probe_hold;      /* rc pin 探测期: 暂停 Poll 免得打架 */
static uint8_t    s_cfg_en;          /* ⭐0924CT: 0=纯轮询(默认) 1=跑配置序列 */
static uint8_t    s_sf_pend[4];      /* ⭐0924CV: 摇杆 MSB 滤波 pending 计数 */

/* ⭐0924CV 摇杆 MSB 毛刺滤波: 这只克隆手柄每字节末位采样随机错
 * (实测 0x80<->00 / 0x7F<->FF 成对出现, 真实中位 0x7F/0x80)。
 * 与上次值**只差 bit7** 的单帧跳变 = 毛刺 ⇒ 保持上次值;
 * 同一毛刺值连续 2 帧出现 = 真跳变 (如 7F<->FF 极限往返) ⇒ 放行。 */
static uint8_t stick_msb_filter(uint8_t prev, uint8_t raw, uint8_t* pend)
{
    if (raw == prev) {
        *pend = 0U;
        return prev;
    }
    if (((uint8_t)(prev ^ raw) & 0x7FU) == 0U) {   /* 只差 MSB */
        if (*pend != 0U) {
            *pend = 0U;
            return raw;                /* 连续 2 帧同值: 真跳变 */
        }
        *pend = 1U;
        return prev;                   /* 首见: 判为毛刺, 保持 */
    }
    *pend = 0U;
    return raw;                        /* 低位有变化 = 真实动作 */
}

/* ---------------------------------------------------------------- */
/* 按键位图 / 键位映射 (0924H)                                        */
/* ---------------------------------------------------------------- */
/* 代码 1..16 <-> 位图 bit0..15; 按下=1 (原始字节按下=0, 已取反)。 */
static const char* const BTN_NAMES[ARM_RC_NBTN] = {
    "SELECT", "L3", "R3", "START", "UP", "RIGHT", "DOWN", "LEFT",
    "L2", "R2", "L1", "R1", "TRI", "CIR", "CRS", "SQR"
};

/* ⭐0924X 默认键位 (用户 2026-09-28 现场定案, 原样固化):
 *   L2=夹爪张开  R2=夹爪闭合  TRI=中位  CIR=折叠
 *   SQR=示教录制  CRS=回放   R1=J5逆时针  L1=J5顺时针
 *   UP/DOWN/LEFT/RIGHT 保留预留, START/SELECT 保留系统武装/锁定双击 */
static const uint8_t MAP_DEF[ARM_RC_FN_COUNT] = {
    /* open=L2(9) close=R2(10) home=TRI(13) fold=CIR(14)
     * rec=SQR(16) play=CRS(15) j5ccw=R1(12) j5cw=L1(11)  —— 代码 1~16 */
    9U, 10U, 13U, 14U, 16U, 15U, 12U, 11U
};

/* 区 E 记录 (16B 含 pad, < 4KB 区)。sum = 前 sizeof-1 字节和取反。
 * ⭐ ver=4: 0924X 关节空间改版 + 键位按用户现场版固化 (见 MAP_DEF)。
 *   ver 升号 = 板上旧记录读入即弃, 烧录后直接吃到新默认 —— 不必再敲
 *   `rc map def`/`rc map save`。旧 ver=1/2/3 记录一律忽略。 */
#define ARM_RC_MAP_MAGIC 0x52434D50U       /* "RCMP" 小端 */
#define ARM_RC_MAP_VER   4U
typedef struct {
    uint32_t magic;
    uint16_t ver;
    uint8_t  map[ARM_RC_FN_COUNT];
    uint8_t  sum;
} ArmRcMapRec;

static uint8_t  s_map[ARM_RC_FN_COUNT];
static uint16_t s_btn_bm    = 0U;          /* 当前按下的键位图               */
static uint16_t s_edge_pend = 0U;          /* 未取走的新按下边沿             */

/* 手柄配置序列的命令模板 (手册 V1.5 §4 原样) */
static const uint8_t C_SHORTPOLL[5] = {0x01, 0x42, 0x00, 0x00, 0x00};
static const uint8_t C_ENTER_CFG[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t C_ANALOG[9]    = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
                                   /*          analog=01  锁存: MODE 键不可改 */
                                   /* ⭐0924CS: 震动命令 0x4D 整条删除 (见 ps2_config_controller 注释) */
static const uint8_t C_EXIT_CFG[9]  = {0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                   /* ⭐0924CU: 尾部 0x5A→0x00 —— PS2X 官方库退出帧
                                    * = sendCommand(0x43, 0x00) 全零尾; 旧模板 5A 尾
                                    * 疑似被克隆手柄当垃圾数据吞掉 = F3 永久卡死主嫌 */

/* 最近一帧解码结果 (poll 成功才刷新) */
static uint8_t s_id, s_btn3, s_btn4;
static uint8_t s_rx, s_ry, s_lx, s_ly;

/* 字节间间隔 (µs): 官方 YFRobot 例程 50µs, 四驱车例程 ~30µs, 我们取 16。
 * `rc diag` 慢速帧临时改大做对比 (SPI 位时钟固定, 只能调字节隔)。 */
static uint32_t s_gap_us  = ARM_RC_PS2_BYTE_GAP_US;

/* ---------------------------------------------------------------- */
/* 微秒延时: DWT CYCCNT (168MHz, 精确到 1 cycle)                     */
/* ---------------------------------------------------------------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ok = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ? 1U : 0U;
}

static void ps2_delay_us(uint32_t us)
{
    if (!s_dwt_ok) {                       /* DWT 不可用: 退化 NOP 环(粗)  */
        volatile uint32_t n = us * 40U;    /* ~168MHz 下 ≈1µs/40 拍        */
        while (n--) { __NOP(); }
        return;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

/* ---------------------------------------------------------------- */
/* 传输原语 (0924CW 软件 bit-bang, 取代硬件 SPI1)                      */
/* ---------------------------------------------------------------- */
#define MISO_READ() ((GPIOB->IDR & ARM_RC_PS2_MISO_PIN) ? 1U : 0U)
#define CS_H()      (ARM_RC_PS2_CS_PORT->BSRR  = ARM_RC_PS2_CS_PIN)
#define CS_L()      (ARM_RC_PS2_CS_PORT->BSRR  = (uint32_t)ARM_RC_PS2_CS_PIN << 16U)
#define CLK_H()     (GPIOB->BSRR = ARM_RC_PS2_SCK_PIN)
#define CLK_L()     (GPIOB->BSRR = (uint32_t)ARM_RC_PS2_SCK_PIN << 16U)
#define MOSI_H()    (GPIOB->BSRR = ARM_RC_PS2_MOSI_PIN)
#define MOSI_L()    (GPIOB->BSRR = (uint32_t)ARM_RC_PS2_MOSI_PIN << 16U)

/* 位半周期 (µs): 5µs×4/位 ≈ 50kHz 位时钟, 72bit 帧约 1.5ms。
 * ESP32 独测同接收器慢速翻转字节全干净; `rc diag` 三帧用 5/12/24µs 对比。 */
static uint32_t s_half_us = ARM_RC_PS2_BIT_HALF_US;
static uint8_t  s_cpha1   = 1U;            /* Mode3 默认 (0924N 可切)      */
/* ⭐0924CX 采样相位: 1=CWb 提前一槽 (旧克隆手柄), 0=标准 Mode3 (新接收机)。
 * s_phase_auto=1 时按 mark(0x5A) 移位特征自动判型 (见 ArmRc_Poll)。 */
static uint8_t  s_early      = 1U;
static uint8_t  s_phase_auto = 1U;
static uint8_t  s_phase_hit  = 0U;         /* 连续移位帧计数 (抗毛刺)      */
/* ⭐0924CYb 模式闸门 (用户 11-20 定案): 0=红灯族(模拟) 工作/绿灯族(数字)
 * 全部禁行 (默认 —— "模拟才是好的": 模式相位自适应后模拟数据读得准,
 * 数字族只按键且用户实测切到数字族臂就乱 ⇒ 禁行)。1=绿灯族工作/红灯族
 * 禁行 (备用, 万一某手柄 LED 逻辑相反)。禁行 = 该帧按键/摇杆一律清零
 * 回中 + 不进控制链, 等效未武装。运行时 `rc gate g|r` 切换。
 * s_gate_block 给诊断/总览显示"禁行中"。 */
static uint8_t  s_gate_digital_work = 0U;
static uint8_t  s_gate_block        = 0U;

/* 收发一个字节: bit-bang, LSB 先行。两种采样相位 (0924CX 可自适应切换):
 * s_early=1 (0924CWb): 旧克隆手柄 DAT 流比 Mode3 提前 1 位 —— bit0 在 CS↓
 *   后立即可读, bit k 从 falling#(k-1) 起有效 ⇒ bit0 帧首采, bit1..7 在
 *   rising#0..6 后采 (各提前一槽); 第 8 个脉冲只为 CMD bit7。
 * s_early=0 (标准): bit k 在 rising#k 后采 (教科书 Mode3 时序, 换新接收机用)。 */
static uint8_t ps2_xfer_byte(uint8_t cmd)
{
    uint8_t res = 0U;
    if (s_early) {
        if (MISO_READ()) res |= 0x01U;     /* bit0: 本字节首槽之前即可得    */
        for (uint8_t k = 0U; k < 7U; k++) {
            uint8_t nbit = (uint8_t)(1U << (k + 1U));
            if (s_cpha1) {                 /* ---- Mode3 ----              */
                CLK_L();                   /* 下降沿: 从机换出下一位 DAT   */
                ps2_delay_us(s_half_us);
                if (cmd & (1U << k)) MOSI_H(); else MOSI_L();
                ps2_delay_us(s_half_us);
                CLK_H();                   /* 上升沿: 从机采样 CMD         */
                ps2_delay_us(s_half_us);
                if (MISO_READ()) res |= nbit;   /* 此时 DAT = bit(k+1)     */
                ps2_delay_us(s_half_us);
            } else {                       /* ---- Mode2 ----              */
                if (cmd & (1U << k)) MOSI_H(); else MOSI_L();
                ps2_delay_us(s_half_us);
                CLK_L();
                ps2_delay_us(s_half_us);
                if (MISO_READ()) res |= nbit;
                ps2_delay_us(s_half_us);
                CLK_H();
                ps2_delay_us(s_half_us);
            }
        }
        /* 第 8 个脉冲: 只为 CMD bit7 (DAT 此槽已是下一字节 bit0) */
        CLK_L();
        ps2_delay_us(s_half_us);
        if (cmd & 0x80U) MOSI_H(); else MOSI_L();
        ps2_delay_us(s_half_us);
        CLK_H();
        ps2_delay_us(s_half_us);
        MOSI_H();                          /* CMD 空闲回高                 */
        ps2_delay_us(s_half_us);
    } else {
        for (uint8_t k = 0U; k < 8U; k++) {
            uint8_t bit = (uint8_t)(1U << k);
            if (s_cpha1) {                 /* ---- Mode3 标准 ----          */
                CLK_L();
                ps2_delay_us(s_half_us);
                if (cmd & bit) MOSI_H(); else MOSI_L();
                ps2_delay_us(s_half_us);
                CLK_H();
                ps2_delay_us(s_half_us);
                if (MISO_READ()) res |= bit;
                ps2_delay_us(s_half_us);
            } else {                       /* ---- Mode2 标准 ----          */
                if (cmd & bit) MOSI_H(); else MOSI_L();
                ps2_delay_us(s_half_us);
                CLK_L();
                ps2_delay_us(s_half_us);
                if (MISO_READ()) res |= bit;
                ps2_delay_us(s_half_us);
                CLK_H();
                ps2_delay_us(s_half_us);
            }
        }
    }
    ps2_delay_us(s_gap_us);                /* 字节间间隔 (官方例程 10~50µs) */
    return res;
}

/* 一帧 9 字节 (CS 低电平期间一次传完 —— 手册明确: 不是每字节拉高) */
static void ps2_frame(const uint8_t* cmd, uint8_t* out, uint32_t n)
{
    CS_L();
    ps2_delay_us(s_gap_us);
    for (uint32_t i = 0U; i < n; i++) {
        out[i] = ps2_xfer_byte(cmd[i]);
    }
    CS_H();
    ps2_delay_us(s_gap_us);
}

/* ---------------------------------------------------------------- */
/* 手柄配置 (手册 PS2_SetInit 原样序列): 强制红灯模拟模式并锁存       */
/* ---------------------------------------------------------------- */
static void ps2_config_controller(void)
{
    uint8_t r[9];
    /* ⭐0924CT: 默认纯轮询 —— 直接跳过整条配置序列。ESP32 直测证据: 这只
     * 2.4G 克隆接收器对裸轮询回 ID=0x20 且按键正常; 而"进配置→锁模拟→
     * 退出"序列会把它推进配置模式 (ID=0xF3) 且所有退出手段 (补发/组合拳/
     * 静默窗) 都被吞 = 永久卡死 (CS 实测 reinit 后 22s 不动)。原装手柄
     * 需要拉回红灯时用 `rc cfg 1` 打开。 */
    if (!s_cfg_en) return;
    for (uint8_t i = 0U; i < 3U; i++) {
        ps2_frame(C_SHORTPOLL, r, 5U);     /* 建立/恢复连接 ×3             */
    }
    /* ⭐0924CQ: 每条配置命令之间插入 5ms 间隔 —— 克隆手柄 (ID=0x20/0x2A)
     * 的内部状态机处理慢, 背靠背帧会让"退出配置"被吞 (实测卡死 ID=0xF3)。
     * 参考 Arduino PS2X 库: 配置命令间均有 delay。总耗时 ~20ms, 仅在
     * 开机/重配时执行一次, 不影响热路径。
     * ⭐0924CS: 删除震动命令 C_VIBMODE —— 0x4D 按主流实现需 6 字节而模板
     * 只有 5 字节, 字节数不合会把克隆手柄的配置状态机打错位, 下一条退出
     * 命令被当成残缺命令的尾数据吞掉 = 永久卡 F3 (CR 实测组合拳也救不回)。
     * 遥控不用震动, 少一条命令少一个翻车面。 */
    HAL_Delay(5);
    ps2_frame(C_ENTER_CFG, r, 9U);         /* 进入配置模式                 */
    HAL_Delay(5);
    ps2_frame(C_ANALOG,    r, 9U);         /* 模拟模式 + 0x03 锁存         */
    HAL_Delay(5);
    ps2_frame(C_EXIT_CFG,  r, 9U);         /* 完成并保存                   */
    /* ⭐0924CR: 退出配置后静默 50ms —— 克隆手柄此刻在把"模拟模式"写进
     * 自己的 EEPROM, 立刻恢复 20ms 高频轮询会把保存打断, 实测表现为
     * "拉出 F3 后只回 1 帧 0x73 就永久沉默"。给足保存窗口再恢复轮询。 */
    HAL_Delay(50);
    s_last_cfg_ms = HAL_GetTick();
}

/* ---------------------------------------------------------------- */
/* GPIO 初始化 (0924CW bit-bang: PB3/PB5=GPIO 输出, PB4=输入上拉)      */
/* ---------------------------------------------------------------- */

/* SCK=PB3 / MOSI=PB5 配 GPIO 输出(空闲高), MISO=PB4 输入上拉
 * (PinDrive 探测后恢复也走这里) */
static void ps2_bb_pins(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = ARM_RC_PS2_SCK_PIN | ARM_RC_PS2_MOSI_PIN;
    HAL_GPIO_Init(GPIOB, &gi);
    CLK_H();
    MOSI_H();                              /* CPOL=1: CLK/MOSI 空闲高      */
    gi.Pin   = ARM_RC_PS2_MISO_PIN;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_PULLUP;                /* DAT 空闲高                   */
    HAL_GPIO_Init(GPIOB, &gi);
}

static void ps2_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CS/ATT = PB1: 推挽输出, 空闲高 */
    gi.Pin   = ARM_RC_PS2_CS_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARM_RC_PS2_CS_PORT, &gi);
    CS_H();

    ps2_bb_pins();
    /* ⭐0924CW: 硬件 SPI1 寄存器配置整体移除 — 见文件头注释 */
}

/* ---------------------------------------------------------------- */
/* CPHA 运行时切换 (0924N 引入; 0924CW 起 bit-bang 采样沿切换)         */
/* ---------------------------------------------------------------- */
void ArmRc_SetCpha(uint8_t cpha1)
{
    s_cpha1 = cpha1 ? 1U : 0U;             /* Mode3=上升沿采 / Mode2=下降沿采 */

    ps2_config_controller();               /* 重跑红灯模拟+锁存配置序列   */
    s_state.fail_run = 0U;
}

uint8_t ArmRc_GetCpha(void)
{
    return s_cpha1;
}

/* ---- 0924CX 采样相位 ----
 * en=0: 手动提前一槽 (旧克隆手柄); en=1: 手动标准时序 (新接收机);
 * en=2: 自动判型 (mark 移位特征, 默认) */
void ArmRc_SetPhase(uint8_t mode)
{
    if (mode == 2U) {
        s_phase_auto = 1U;
        return;
    }
    s_phase_auto = 0U;
    s_early = mode ? 0U : 1U;              /* mode 0=提前(旧克隆) 1=标准    */
    s_phase_hit = 0U;
    s_state.fail_run = 0U;
}

uint8_t ArmRc_GetPhaseEarly(void)  { return s_early; }
uint8_t ArmRc_GetPhaseAuto(void)   { return s_phase_auto; }

/* ⭐0924CYb 模式闸门: 0=红灯族(模拟)工作/绿灯族(数字)禁行 (默认);
 * 1=绿灯族工作/红灯族禁行 (备用)。 */
void ArmRc_SetGate(uint8_t digital_work)
{
    s_gate_digital_work = digital_work ? 1U : 0U;
}
uint8_t ArmRc_GetGate(void)        { return s_gate_digital_work; }
uint8_t ArmRc_GetGateBlock(void)   { return s_gate_block; }

/* ---- 手柄配置序列开关 (0924CT) ---- */
void ArmRc_SetCfgEn(uint8_t en)
{
    en = en ? 1U : 0U;
    if (en == s_cfg_en) {
        if (en) ps2_config_controller();   /* 已开再开 = 重跑一遍序列 */
        return;
    }
    s_cfg_en = en;
    if (en) {
        ps2_config_controller();           /* 打开: 立刻跑一遍配置序列   */
        s_state.t1_inited = 1U;
    } else {
        /* 关闭: 只停发配置命令; 若当前卡 F3, Poll 里的补发退出帧照常救 */
        s_exit_tries = 0U;
    }
    s_state.fail_run = 0U;
}

uint8_t ArmRc_GetCfgEn(void)
{
    return s_cfg_en;
}

/* ---- 模拟命令旁路 (0924CU): 不进配置模式直接发 0x44 ----
 * 动机: 这只 2.4G 克隆接收器原生绿灯(摇杆 FF), 走 SetInit 必卡 F3。
 * 真手柄上 0x44 在配置模式外会被忽略, 但克隆手柄行为未知 —— 试探性
 * 直发: 3×轮询 → 5ms → 0x44(模拟+锁存) → 5ms → 3×轮询。
 * 期间挂起 Poll (s_probe_hold) 防止 50Hz 轮询抢 SPI。 */
void ArmRc_CfgAnalogDirect(void)
{
    uint8_t r[9];
    s_probe_hold = 1U;                     /* 暂停 Poll, 独占 SPI */
    for (uint8_t i = 0U; i < 3U; i++) {
        ps2_frame(C_SHORTPOLL, r, 5U);     /* 建立连接 ×3 */
    }
    HAL_Delay(5);
    ps2_frame(C_ANALOG,    r, 9U);         /* 模拟模式+0x03 锁存, 不进配置 */
    HAL_Delay(5);
    for (uint8_t i = 0U; i < 3U; i++) {
        ps2_frame(C_SHORTPOLL, r, 5U);     /* 恢复轮询 */
    }
    HAL_Delay(20);
    s_probe_hold = 0U;
    s_state.fail_run = 0U;
}

/* ---------------------------------------------------------------- */
/* 通道合成: 摇杆/按键 -> 脉宽                                        */
/* ---------------------------------------------------------------- */

/* 摇杆 count(0~255, 中位 128) -> 脉宽: 上/右 = >1500 (方向吸收在本层) */
static uint16_t stick_to_us(uint8_t v, uint8_t invert)
{
    int32_t d = (int32_t)v - ARM_RC_PS2_STICK_MID;
    if (invert) d = -d;
    int32_t a = (d < 0) ? -d : d;
    if (a <= ARM_RC_PS2_STICK_DZ) return 1500U;          /* 死区: 中心即停 */
    if (d >  127) d =  127;
    if (d < -127) d = -127;
    int32_t us = 1500 + (int32_t)((float)d * ARM_RC_PS2_US_FULL / 127.0f);
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    return (uint16_t)us;
}

static void synth_channels(uint32_t now, float dt_ms)
{
    /* ⭐0924X: 按 CSDN 教程改为**关节空间**映射 (避开 IK/FK 校验抖动):
     *   CH1=J1 底座 = LX (左杆左右, 左=− 右=+)
     *   CH2=J2 大臂 = LY (左杆上下, 上=− 下=+)
     *   CH3=J3 小臂 = RY (右杆上下, 上=− 下=+)
     *   CH4=J4 腕俯仰 = RX (右杆左右, 左=− 右=+)
     *   J5 改由 L1/R1 按键控制, 夹爪由 L2/R2 控制。 */
    s_lx = s_state.stick_lx;
    s_state.ch[0].width_us = stick_to_us(s_lx, 0U);
    s_ly = s_state.stick_ly;
    s_state.ch[1].width_us = stick_to_us(s_ly, 0U);
    s_ry = s_state.stick_ry;
    s_state.ch[2].width_us = stick_to_us(s_ry, 0U);
    s_rx = s_state.stick_rx;
    s_state.ch[3].width_us = stick_to_us(s_rx, 0U);

    /* CH5 夹爪 = 双按键长按 (0924H): 张开键→2000, 闭合键→1000, 按住扫描,
     * 都松开 = 原地保持。按键来自键位映射表 (`rc map` 可改, 默认 R1/L1)。
     * RC_SIGN_J6=+1 ⇒ 高脉宽=张开(lim_max), 与 ctrl 层语义一致。 */
    {
        float step = (1000.0f / (float)ARM_RC_CLAW_SWEEP_MS) * dt_ms;
        uint8_t op = ArmRc_BtnDown(s_map[ARM_RC_FN_CLAW_OPEN]);
        uint8_t cl = ArmRc_BtnDown(s_map[ARM_RC_FN_CLAW_CLOSE]);
        if (op && !cl) {
            s_claw_us = (uint16_t)(s_claw_us + step);
            if (s_claw_us > 2000U) s_claw_us = 2000U;
        } else if (cl && !op) {
            s_claw_us = (uint16_t)(s_claw_us - step);
            if (s_claw_us < 1000U) s_claw_us = 1000U;
        }
        s_state.ch[ARM_RC_CH_CLAW].width_us = s_claw_us;
    }

    /* CH6 心跳 = 链路状态: 通信正常 = 1000 (RC_HB_EN=0 时上层不读, 仅供显示) */
    s_state.ch[ARM_RC_CH_HB].width_us = 1000U;

    for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
        s_state.ch[i].edges  = s_state.ch[i].frames;   /* 兼容字段 */
        s_state.ch[i].last_ms = now;
        if (s_state.ch[i].width_us < s_state.ch[i].w_min) {
            s_state.ch[i].w_min = s_state.ch[i].width_us;
        }
        if (s_state.ch[i].width_us > s_state.ch[i].w_max) {
            s_state.ch[i].w_max = s_state.ch[i].width_us;
        }
    }
}

/* ---------------------------------------------------------------- */
/* 对外接口                                                          */
/* ---------------------------------------------------------------- */
void ArmRc_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
        s_state.ch[i].w_min = 0xFFFFU;
    }
    ps2_gpio_init();
    dwt_init();
    s_state.inited = 1U;
    s_state.exti_inited = 1U;              /* 弃用字段, 恒 1 */

    ps2_config_controller();
    s_state.t1_inited = s_cfg_en;          /* 0924CT: 纯轮询=0(未发配置)   */

    /* ⭐0924CV: 开机自动直发 0x44 模拟命令 —— 这只 2.4G 克隆手柄原生绿灯
     * (无摇杆), SetInit 必卡 F3, 唯有 cfg a 能开出模拟。真·原装手柄在
     * 配置模式外收到 0x44 会忽略, 无副作用。
     * ⭐0924CY: 闸门=绿灯族工作(默认)时**不发** —— 用户要绿灯模式工作,
     * 强拉红灯只会把自己禁行。`rc gate r` 切回红灯族工作后 reinit 才拉。 */
    if (!s_gate_digital_work) {
        ArmRc_CfgAnalogDirect();
    }

    /* ⭐ 中位开机自动标记: PS2 摇杆中位恒 1500(死区内), `rc mid` 不再必需 */
    for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
        s_state.mid[i] = 1500U;
        s_state.mid_mask |= (uint8_t)(1U << i);
    }
    s_state.mid_valid = 1U;

    ArmRc_MapDefaults();
    ArmRc_MapLoad();                       /* 区 E 有有效记录则覆盖默认 */
}

void ArmRc_Reinit(void)
{
    /* PS2 下没有 TIM4 可抢 —— `nt` 之后 GPIO 也不会被动过。
     * 保留接口做"救急重配": GPIO 重配 + 手柄配置序列重跑。 */
    ps2_gpio_init();
    dwt_init();
    ps2_config_controller();
    if (!s_gate_digital_work) {            /* ⭐0924CY: 绿灯族工作时不强拉模拟 */
        ArmRc_CfgAnalogDirect();           /* ⭐0924CV: 重配后也直发模拟命令   */
    }
    s_state.fail_run = 0U;
}

void ArmRc_Poll(void)
{
    uint8_t d[9] = {0};
    uint32_t now  = HAL_GetTick();
    if (s_probe_hold) return;              /* rc pin 拉低探测中: 别动 GPIO */
    float    dt_ms = (s_last_poll_ms != 0U) ? (float)(now - s_last_poll_ms) : 20.0f;
    if (dt_ms > 100.0f) dt_ms = 100.0f;
    s_last_poll_ms = now;

    /* ⭐ 单帧 9 字节: 0x01 0x42 + 7×0x00 —— Data[1]=ID, Data[2]=0x5A,
     *   Data[3]/Data[4]=按键, Data[5..8]=RX RY LX LY 一次全带回。
     *   (短帧+全帧两次事务是浪费; 手册的 PS2_ReadData 本就是一帧全收) */
    {
        uint8_t cmd[9] = {0x01, 0x42, 0, 0, 0, 0, 0, 0, 0};
        ps2_frame(cmd, d, 9U);
    }

    /* ⭐0924CX: 采样相位自适应 —— mark(0x5A) 移位特征判型。
     * 读成 0xB4/0xB5 (=5A<<1) = 接收机标准时序而我们提前采 ⇒ 切标准;
     * 读成 0x2D/0xAD (=5A>>1, 尾位被邻字节污染) = 接收机提前 1 位而我们
     * 标准采 (旧克隆手柄) ⇒ 切提前。连续 3 帧才切, 切完下一帧即对齐。
     * 移位帧 ok=0 不进控制 ⇒ 自动校正窗内机械臂不受影响。`rc phase` 可覆盖。 */
    if (s_phase_auto) {
        if (d[2] == ARM_RC_PS2_MARK) {
            s_phase_hit = 0U;
        } else if ((d[2] == 0xB4U || d[2] == 0xB5U) && s_early) {
            if (++s_phase_hit >= 3U) { s_early = 0U; s_phase_hit = 0U; }
        } else if ((d[2] == 0x2DU || d[2] == 0xADU) && !s_early) {
            if (++s_phase_hit >= 3U) { s_early = 1U; s_phase_hit = 0U; }
        } else {
            s_phase_hit = 0U;
        }
    }

    /* ⭐0924CU: ID 低 7 位匹配 —— 这只 2.4G 克隆手柄每字节**末位采样不可靠**
     * (实测: 真实绿灯 0x41 读回 0xC1, 真实红灯 0x73 读回 0xF3, 按键字节
     * 0xFF 读回 0x7F), 0x5A 在位 + 低 7 位命中即可信。
     * 安全策略: 只有**干净** 0x73/0x20/0x79 才按全模拟解析 (摇杆直通);
     * 带毛刺 ID (0xC1 数字 / 0xF3 疑似配置模式) 一律按【数字模式】处理:
     * 按键可用, 摇杆字节不可信 ⇒ 通道强制回中, 杜绝 FF→满偏跑臂。 */
    uint8_t id7 = d[1] & 0x7FU;
    /* 摇杆合理性: 四轴全离轨值 (00/FF) = 数据不可信; 模拟模式空闲应 ~0x80 */
    uint8_t sane = (d[5] != 0x00U && d[5] != 0xFFU &&
                    d[6] != 0x00U && d[6] != 0xFFU &&
                    d[7] != 0x00U && d[7] != 0xFFU &&
                    d[8] != 0x00U && d[8] != 0xFFU) ? 1U : 0U;
    uint8_t ok = 0U, digital = 0U;
    if (d[2] == ARM_RC_PS2_MARK) {
        if (d[1] == ARM_RC_PS2_ID_RED || ARM_RC_ID_ANALOG_OK(d[1])) {
            ok = 1U;                       /* 干净 0x73/0x20/0x79 = 全模拟 */
        } else if (d[1] == ARM_RC_PS2_ID_CFG && sane) {
            /* ⭐0924CU: F3 = 0x73+末位毛刺 且摇杆全部离轨值 ⇒ 真红灯模拟,
             * 按模拟解析 (克隆手柄即使开了模拟, ID 也读不出干净 0x73) */
            ok = 1U;
        } else if (d[1] != ARM_RC_PS2_ID_CFG &&
                   (id7 == ARM_RC_PS2_ID_RED || id7 == ARM_RC_PS2_ID_GREEN ||
                    ARM_RC_ID_ANALOG_OK(id7))) {
            ok = 1U;                       /* 毛刺 ID: C1(绿灯) 等 */
            digital = 1U;                  /* 一律按数字模式: 只信按键 */
        }
    }

    /* ⭐0924CYb 模式闸门: 默认 红灯(模拟族)=工作, 绿灯(数字族)=全禁
     * ("模拟才是好的")。禁行帧: 按键清零+摇杆回中+保持中性通道输出
     * (链路本身算 ok, 不触发 fail_run 重配逻辑)。被禁行的模式看不到任何动作。 */
    s_gate_block = 0U;
    if (ok) {
        uint8_t frame_analog = digital ? 0U : 1U;
        uint8_t allow = s_gate_digital_work ? (uint8_t)!frame_analog : frame_analog;
        if (!allow) {
            s_gate_block = 1U;
            s_id = d[1];
            s_state.id = s_id;
            s_state.analog_ok = 0U;
            s_state.stick_rx = ARM_RC_PS2_STICK_MID;
            s_state.stick_ry = ARM_RC_PS2_STICK_MID;
            s_state.stick_lx = ARM_RC_PS2_STICK_MID;
            s_state.stick_ly = ARM_RC_PS2_STICK_MID;
            s_btn3 = 0xFFU;  s_btn4 = 0xFFU;
            s_btn_bm = 0U;                       /* 边沿队列也清空: 禁行模式按啥都没反应 */
            s_sf_pend[0] = s_sf_pend[1] = s_sf_pend[2] = s_sf_pend[3] = 0U;
            s_state.btn3 = 0xFFU;  s_state.btn4 = 0xFFU;
            s_state.poll_ok++;
            if (s_state.fail_run != 0U) s_state.fail_run = 0U;
            synth_channels(now, dt_ms);          /* 全中性通道: 等效未武装 */
            s_state.last_frame_ms = now;
            return;                              /* ⭐ 提前返回: 不走正常解析 */
        }
    }

    if (ok) {
        s_id = d[1];
        s_exit_tries = 0U;                 /* 拉出 F3 成功, 升级计数归零 */
        s_btn3 = (uint8_t)(d[3] & 0x7FU);  /* ⭐0924CU: byte3 bit7 毛刺屏蔽 */
        s_btn4 = d[4];
        s_state.id        = s_id;
        s_state.analog_ok = digital ? 0U : 1U;
        s_state.btn3 = s_btn3;  s_state.btn4 = s_btn4;
        if (digital) {
            /* 数字模式: 摇杆字节恒 FF(实测) ⇒ 全部回中, 只用按键边沿 */
            s_state.stick_rx = ARM_RC_PS2_STICK_MID;
            s_state.stick_ry = ARM_RC_PS2_STICK_MID;
            s_state.stick_lx = ARM_RC_PS2_STICK_MID;
            s_state.stick_ly = ARM_RC_PS2_STICK_MID;
            s_sf_pend[0] = s_sf_pend[1] = s_sf_pend[2] = s_sf_pend[3] = 0U;
        } else {
            /* ⭐0924CV: 摇杆过 MSB 毛刺滤波 (每轴独立 pending) */
            s_state.stick_rx = stick_msb_filter(s_state.stick_rx, d[5], &s_sf_pend[0]);
            s_state.stick_ry = stick_msb_filter(s_state.stick_ry, d[6], &s_sf_pend[1]);
            s_state.stick_lx = stick_msb_filter(s_state.stick_lx, d[7], &s_sf_pend[2]);
            s_state.stick_ly = stick_msb_filter(s_state.stick_ly, d[8], &s_sf_pend[3]);
        }
        /* ⭐ 按键位图 (按下=1) + 新按下边沿累积 (漏调不丢)
         * 0924CU: byte3 只取低 7 位 (bit7=毛刺, 恒读 0 会假触发 LEFT) */
        {
            uint16_t bm = (uint16_t)(((uint16_t)(~s_btn3) & 0x7FU) |
                                     (uint16_t)(((uint16_t)(~d[4]) & 0xFFU) << 8));
            s_edge_pend |= (uint16_t)(bm & (uint16_t)~s_btn_bm);
            s_btn_bm = bm;
        }
        s_state.poll_ok++;
        if (s_state.fail_run != 0U) s_state.fail_run = 0U;
        synth_channels(now, dt_ms);
        s_state.last_frame_ms = now;
    } else if (d[1] == ARM_RC_PS2_ID_CFG && d[2] == ARM_RC_PS2_MARK) {
        /* ⭐0924CP: 手柄活着但卡在配置模式 (ID=0xF3, 0x5A 在位) —— 克隆手柄
         * 常见: 完整重配序列(再进配置→开模拟→退出)的退出帧被它吞掉,
         * 永久停在 F3; 而 fail_run 累积又会触发重配 ⇒ 进配置死循环。
         * 对策: 只补发退出帧 (0x43/0x00), 200ms 节流; 不算链路失败、
         * 不清 fail_run 触发逻辑 (fail_run 保持 0 防止重配再进配置)。 */
        s_state.poll_fail++;
        s_state.fail_run = 0U;
        if ((uint32_t)(now - s_exit_try_ms) > 200U) {
            uint8_t r[9];
            /* ⭐0924CQ: 前 4 次只补一发退出帧 (侵入最小); 仍 F3 说明克隆
             * 手柄把退出帧也吞了 —— 升级为 "进配置→5ms→退出" 组合拳,
             * 重建一个干净的配置会话再退出 (带帧间延时才有效)。 */
            s_exit_tries++;
            if (s_exit_tries >= 5U) {
                ps2_frame(C_ENTER_CFG, r, 9U);   /* 重进配置会话 */
                HAL_Delay(5);
                ps2_frame(C_EXIT_CFG,  r, 9U);   /* 再退出       */
                HAL_Delay(50);                   /* ⭐0924CR: 保存窗口 */
                if (s_exit_tries >= 250U) s_exit_tries = 0U;
            } else {
                ps2_frame(C_EXIT_CFG, r, 9U);    /* 直接补一发退出 */
            }
            s_exit_try_ms = now;
        }
    } else {
        s_state.poll_fail++;
        if (s_state.fail_run < 0xFFFFFFFFU) s_state.fail_run++;
        /* ⭐0924CT: 纯轮询模式下掉线 **不跑** 重配序列 —— 跑了会把刚配对
         * 回来的克隆手柄又推进 F3。原装手柄绿灯问题用 `rc cfg 1` 手动救。 */
        if (s_cfg_en &&
            s_state.fail_run >= ARM_RC_FAIL_RECFG &&
            (uint32_t)(now - s_last_cfg_ms) > ARM_RC_CFG_RETRY_MS) {
            ps2_config_controller();       /* 手柄掉线重配对/被拨回绿灯 ⇒ 拉回 */
            s_state.fail_run = 0U;
        }
    }

    /* live/新鲜度 (⭐ PS2 主动轮询: 没响应就是没有帧, fresh 恢复本义) */
    {
        uint8_t live = (s_state.last_frame_ms != 0U &&
                        (uint32_t)(now - s_state.last_frame_ms) <= ARM_RC_TIMEOUT_MS) ? 1U : 0U;
        if (s_live_prev && !live) s_state.lost_events++;
        s_live_prev       = live;
        s_state.live      = live;
        s_state.now_ms    = now;
        if (!live) {
            s_btn_bm    = 0U;              /* ⭐ 防卡键: 无链路 = 全部松开 */
            s_edge_pend = 0U;
        }
        for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
            s_state.ch[i].fresh = (uint8_t)((s_state.ch[i].last_ms != 0U &&
                                (uint32_t)(now - s_state.ch[i].last_ms) <= ARM_RC_TIMEOUT_MS)
                                ? 1U : 0U);
        }
    }
}

void ArmRc_ResetStats(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
        s_state.ch[i].w_min = 0xFFFFU;
        s_state.ch[i].w_max = 0U;
        s_state.ch[i].frames = 0U;
        s_state.ch[i].edges  = 0U;
    }
    s_state.live          = 0U;
    s_state.last_frame_ms = 0U;
    s_state.lost_events   = 0U;
    s_state.poll_ok       = 0U;
    s_state.poll_fail     = 0U;
    s_state.fail_run      = 0U;
    __set_PRIMASK(pm);
}

void ArmRc_SetMid(void)
{
    /* PS2 摇杆中位恒 1500 且开机已自动标记 —— 保留命令兼容:
     * 把"当前有效通道"的合成脉宽记为中位(对杆通道=死区中心, 无操作)。 */
    for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
        if (s_state.ch[i].frames != 0U) {
            s_state.mid[i] = s_state.ch[i].width_us;
            s_state.mid_mask |= (uint8_t)(1U << i);
        }
    }
    s_state.mid_valid = 1U;
}

const ArmRcState* ArmRc_Get(void)
{
    s_state.now_ms = HAL_GetTick();
    return &s_state;
}

/* ---------------------------------------------------------------- */
/* 按键层 (0924H)                                                    */
/* ---------------------------------------------------------------- */
uint16_t ArmRc_Buttons(void) { return s_btn_bm; }

uint16_t ArmRc_BtnEdges(void)
{
    uint16_t e = s_edge_pend;
    s_edge_pend = 0U;
    return e;
}

uint8_t ArmRc_BtnDown(uint8_t code)
{
    if (code < 1U || code > ARM_RC_NBTN) return 0U;
    return (uint8_t)((s_btn_bm >> (uint8_t)(code - 1U)) & 1U);
}

const char* ArmRc_BtnName(uint8_t code)
{
    if (code < 1U || code > ARM_RC_NBTN) return "?";
    return BTN_NAMES[code - 1U];
}

/* 忽略大小写比较 (不依赖 strcasecmp, newlib 版本差异) */
static uint8_t btn_ci_eq(const char* a, const char* b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - 'a' + 'A');
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - 'a' + 'A');
        if (ca != cb) return 0U;
        a++;  b++;
    }
    return (uint8_t)(*a == 0 && *b == 0);
}

uint8_t ArmRc_BtnFromName(const char* s)
{
    if (!s || !*s) return 0U;
    if (btn_ci_eq(s, "none") || strcmp(s, "0") == 0) return 0U;
    for (uint8_t i = 0U; i < ARM_RC_NBTN; i++) {
        if (btn_ci_eq(s, BTN_NAMES[i])) return (uint8_t)(i + 1U);
    }
    {   /* 数字直给 1~16 */
        unsigned long v = 0U;
        const char* p = s;
        uint8_t any = 0U;
        while (*p >= '0' && *p <= '9') { v = v * 10U + (unsigned long)(*p - '0'); p++; any = 1U; }
        if (any && *p == 0 && v >= 1U && v <= ARM_RC_NBTN) return (uint8_t)v;
    }
    return 0U;
}

/* ---------------------------------------------------------------- */
/* 键位映射 (扇区 11 区 E)                                            */
/* ---------------------------------------------------------------- */
static uint8_t map_sum(const ArmRcMapRec* r)
{
    const uint8_t* p = (const uint8_t*)r;
    uint8_t s = 0U;
    for (uint32_t i = 0U; i < (uint32_t)sizeof(*r) - 1U; i++) {
        s = (uint8_t)(s + p[i]);
    }
    return (uint8_t)~s;
}

const uint8_t* ArmRc_MapGet(void) { return s_map; }

uint8_t ArmRc_FnDown(uint8_t fn)
{
    if (fn >= ARM_RC_FN_COUNT || s_map[fn] == 0U) return 0U;
    return ArmRc_BtnDown(s_map[fn]);
}

uint8_t ArmRc_MapSet(uint8_t fn, uint8_t code)
{
    if (fn >= ARM_RC_FN_COUNT || code > ARM_RC_NBTN) return 1U;
    if (code != 0U) {
        for (uint32_t i = 0U; i < ARM_RC_FN_COUNT; i++) {
            if (i != (uint32_t)fn && s_map[i] == code) return 2U;
        }
    }
    s_map[fn] = code;
    return 0U;
}

void ArmRc_MapDefaults(void)
{
    memcpy(s_map, MAP_DEF, sizeof(s_map));
}

void ArmRc_MapLoad(void)
{
    ArmRcMapRec rec;
    if (CfgFlash_ReadRegion(CFG_OFF_RCMAP, &rec, (uint32_t)sizeof(rec)) != CFG_FLASH_OK) {
        return;                            /* 空区/非法 off: 保持默认 */
    }
    if (rec.magic != ARM_RC_MAP_MAGIC || rec.ver != ARM_RC_MAP_VER) return;
    if (rec.sum != map_sum(&rec)) return;
    /* 条目合法性: 每项 0~16, 且非零项互不重复 (一个键只干一件事) */
    for (uint32_t i = 0U; i < ARM_RC_FN_COUNT; i++) {
        if (rec.map[i] > ARM_RC_NBTN) return;
        for (uint32_t j = i + 1U; j < ARM_RC_FN_COUNT; j++) {
            if (rec.map[i] != 0U && rec.map[i] == rec.map[j]) return;
        }
    }
    memcpy(s_map, rec.map, sizeof(s_map));
}

uint8_t ArmRc_MapSave(void)
{
    ArmRcMapRec rec;
    memset(&rec, 0, sizeof(rec));          /* pad 清零, sum 覆盖全 15 字节 */
    rec.magic = ARM_RC_MAP_MAGIC;
    rec.ver   = ARM_RC_MAP_VER;
    memcpy(rec.map, s_map, sizeof(rec.map));
    rec.sum = map_sum(&rec);
    return (uint8_t)((CfgFlash_WriteRegion(CFG_OFF_RCMAP, &rec, (uint32_t)sizeof(rec))
                      == CFG_FLASH_OK) ? 1U : 0U);
}

/* ---------------------------------------------------------------- */
/* 接线诊断 (0924J/0924L): 三帧原始 DI 原样吐给控制台                  */
/* 帧1=现役半周期(5µs≈50kHz) 帧2=12µs(≈21kHz) 帧3=24µs(≈10kHz)
 * ⭐0924CW: bit-bang 下位时钟可调, 三帧对比找最稳速度 (硬件 SPI 时代只能
 * 调字节间隔, 已过时) */
/* ---------------------------------------------------------------- */
uint8_t ArmRc_DiagRaw(uint8_t buf[27], uint8_t* di_idle)
{
    if (!s_state.inited) return 0U;
    *di_idle = MISO_READ();                /* CS 高空闲态读 DAT: 应为高    */

    static const uint8_t cmd[9] = {0x01, 0x42, 0, 0, 0, 0, 0, 0, 0};
    static const uint32_t G[3] = {
        ARM_RC_PS2_BIT_HALF_US, 12U, 24U,
    };
    for (uint32_t f = 0U; f < 3U; f++) {
        s_half_us = G[f];
        ps2_frame(cmd, buf + f * 9U, 9U);
        ps2_delay_us(2000U);
    }
    s_half_us = ARM_RC_PS2_BIT_HALF_US;    /* 恢复现役参数                 */
    return 1U;
}

/* ---------------------------------------------------------------- */
/* 接线定位 (0924K/0924M): 分阶段拉低单个输出脚, 万用表在插头孔上探测  */
/* ---------------------------------------------------------------- */
/* phase 0 = 恢复 bit-bang 引脚态并解除探测保持; 1=只拉低 CS(PB1)->插头6;
 * 2=只拉低 SCK(PB3)->插头7; 3=只拉低 MOSI(PB5)->插头2。
 * MISO(PB4) 是输入不参与。⭐0924CW: bit-bang 下 PB3/PB5 本就是 GPIO 输出,
 * 直接 BSRR 拉低即可, 不再需要停/开 SPI。
 * 探测期间 ArmRc_Poll 挂起 (s_probe_hold), 否则 50Hz 轮询会抢线。 */
void ArmRc_PinDrive(uint8_t phase)
{
    if (!s_state.inited) return;
    if (phase == 0U) {
        ps2_bb_pins();                     /* PB3/5 回输出+空闲高, PB4 回输入 */
        CS_H();
        s_probe_hold = 0U;
        return;
    }
    s_probe_hold = 1U;
    CS_H();
    if (phase == 1U) {
        CS_L();                            /* PB1 本就是 GPIO 输出         */
    } else {
        uint32_t pin = (phase == 2U) ? ARM_RC_PS2_SCK_PIN : ARM_RC_PS2_MOSI_PIN;
        GPIOB->BSRR = (uint32_t)pin << 16U;/* 拉低                         */
    }
}
