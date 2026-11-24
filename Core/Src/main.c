/* ===== 6-Axis Robotic Arm Controller (STM32F407 + Feetech STS3215) =====
   Bus A (USART3 PB10/PB11): J1(ID=1) J2(ID=2) J3(ID=3)  74HC126 DIR=PA8
   Bus B (UART5 PC12/PD2):  J4(ID=4) J5(ID=5) J6(ID=6)  74HC126 DIR=PA10
   Console: USART6 (PC6=TX PC7=RX AF8) @ 115200 bps

   ⚠ 主臂 ROLE=master 例外 (2026-11-02): 已拆 74HC126, 6 颗舵机(J1~J6, ID1~6)
     全部并联在 **USART3 (PB10/PB11)** 一条开漏单总线上; UART5 旁路不用。
     见 ARM_ONE_BUS_BUILD + ONE_BUS 别名宏。集成板 BUS_ONE=1 仍走 UART5。

   SCS Protocol, half-duplex TTL, 1Mbps servo bus.
   STS3215: 0-360° / 0-4095 raw, center=180°=2048.
   决定了做什么
*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "servo_bus_hal.h"
#include "scs_protocol.h"
#include "servo_driver.h"
#include "kinematics.h"
#include "trajectory.h"
#include "pid_flash.h"
#include "teach_store.h"          /* 命名示教文件（Flash 持久） */
#include "arm_commands.h"         /* 指令速查清单(注释/帮助可维护) */
#include "arm_params.h"           /* 统一参数表（LP/SP 命令） */
#include "arm_control_api.h"      /* 统一控制接口 */
#include "arm_control_protocol.h" /* 二进制协议核心 */
#include "arm_proto_serial.h"     /* 串口协议承载 */
#include "arm_proto_can.h"        /* CAN 协议承载 */
#include "arm_net.h"              /* 网口(UDP) 协议承载 (LINK=net 构建) */
#include "arm_can_config.h"       /* CAN 节点号/波特率/心跳 配置(可掉电保存) */
#include "arm_link_cfg.h"         /* 链路偏好: 串口常通 + CAN/网口 二选一(可掉电保存) */
#include "geo_cfg.h"              /* 0924AS: 区F 几何偏好 (C 偏移 + zf 桌面下限) */
#include "mstzero_cfg.h"          /* 0924BI: 区H 主臂零点 (fi 板内遥操反解用) */
#include "flash_cfg.h"            /* 0924AS: CFG_FLASH_OK 返回码 (zf/C 自动存盘用) */
#include "arm_main_exports.h"     /* 本文件对控制接口层导出的符号清单(含 ArmMain_Flags 的位定义) */
#include "arm_runlog.h"           /* 运行日志 */
#include "arm_fault.h"            /* 上次运行: CPU 故障? 主循环卡死? (见 arm_fault.h) */
#include "arm_rc.h"               /* RC 遥控输入层 (PS2 手柄, GPIO 模拟 SPI, 只读) */
#include "arm_rc_ctrl.h"          /* R1: RC -> 末端速度流 (rc go on 后才驱臂) */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

/* ---- NET 生效时专用的「CAN/引脚命令拦截」开关 ------------------------
 * 背景: 一批命令(cx/ps/can 与 mg/mp/mo/ml)会把 PB12/PB13 重配成 CAN2 的
 *   AF9, 或把 PA0/PA1 写回 AF8_UART4 —— 而在 NET 生效时这两组脚分别是
 *   ETH_TXD1 与 ETH_RMII_REF_CLK ⇒ 敲一次就把网口掐死, 且现象像"PHY 坏了",
 *   从网络侧查不出原因。
 *   ⇒ NET 生效时必须"接住 + 说清楚"。**不能删这些分支**: 删了它们会落进
 *     首字母兜底(case 'c' → 阻塞式校准向导 → 松全部力矩), 整臂当场塌
 *     (工程红线 3/4)。
 * 取值: 旧 LINK=net 单链路构建 ⇒ 恒 1(编译期常量, 生成代码与改动前一致);
 *       LINK=both 构建 ⇒ 由**运行期链路**决定(选 NET 才拦, 选 CAN/OFF 放行)。
 *       (0924CB 曾试过 NET+CAN 并行常开, 用户 2026-11-05 决定不要, 回退,
 *        保持链路切换语义: 串口恒通 + CAN/NET 按 link 偏好二选一。) */
#if ARM_LINK_MULTI
#define NET_BLOCK_CAN_CMDS()   LINK_IS_NET()
#else
#define NET_BLOCK_CAN_CMDS()   1
#endif

/* CAN Loopback test (defined in arm_can_driver.c) */
extern uint8_t ArmCanDrv_LoopbackTest(void);

/*
 * 主控制层：负责控制台命令、运动任务调度、安全检查和补偿状态机。
 * 中断只产生接收/轨迹更新事件，耗时的舵机通信在主循环中执行。
 */

/* ===== Console UART (USART6 @ PC6/PC7) ===== */
#define CONSOLE_UART       USART6
/* 波特率可由 `make CBAUD=460800` 覆盖 (-DCONSOLE_BAUD=...)。必须 #ifndef 卫兵:
 * 源码里的 #define 会顶掉命令行 -D, 不加卫兵则 CBAUD 开关静默失效。 */
#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD       115200
#endif

/* The S-curve remains calculated on its 200 Hz timebase, but goal packets are
 * sent at 50 Hz.  Each transmit advances four 5 ms interpolation ticks, which
 * is smooth at 舵机 resolution and leaves bus time for feedback/control. */
#define TRAJ_SEND_PERIOD_TICKS  4U

UART_HandleTypeDef hconsole;

/* 配置 USART6 控制台，并开启基于 RXNE 的接收中断。 */
static void Console_Init(void) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC6 = USART6_TX (AF8), PC7 = USART6_RX (AF8) */
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &g);

    hconsole.Instance          = CONSOLE_UART;
    hconsole.Init.BaudRate     = CONSOLE_BAUD;
    hconsole.Init.WordLength   = UART_WORDLENGTH_8B;
    hconsole.Init.StopBits     = UART_STOPBITS_1;
    hconsole.Init.Parity       = UART_PARITY_NONE;
    hconsole.Init.Mode         = UART_MODE_TX_RX;
    hconsole.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    hconsole.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&hconsole);

    /* Enable USART6 RX interrupt (interrupt-based receive, no polling) */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 0);   /* lower priority than TIM2 */
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    __HAL_UART_ENABLE_IT(&hconsole, UART_IT_RXNE);
}

/* ===== Ring buffer for interrupt-based console RX ===== */
#define RX_BUF_SIZE  128
static volatile uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* ISR writes here */
static volatile uint16_t rx_tail = 0;   /* main loop reads here */
/* 环满被丢弃的字节数 (2026-09-15 补)。**旧版这里完全静默** —— 所以"从臂
 * 偶发整帧拒绝 + 冻结几秒"一直没有第一手证据。丢掉的若是 '\r', 两行
 * `#P` 会粘成一行超长垃圾(>127 字符, 超出部分再丢) ⇒ 六个数值全错 ⇒
 * StreamJoints 整帧拒 ⇒ 自锁冻结。`lk` 读出这个数即可定论。 */
static volatile uint32_t rx_ovf = 0;

/* USART6 RX interrupt handler */
void USART6_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&hconsole, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(hconsole.Instance->DR);  /* read clears flag */
        /* 二进制协议分流：0xAA 帧头状态机消费协议字节（ISR 安全），
         * 返回 0 的普通文本字节照原路径进环形缓冲。 */
        if (SerialProto_FeedByte(ch)) {
            /* consumed by binary protocol */
        } else {
            uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
            if (next != rx_tail) {   /* buffer not full */
                rx_ring[rx_head] = ch;
                rx_head = next;
            } else {
                rx_ovf++;            /* 环满丢字节 (见 rx_ovf 声明处注释) */
            }
        }
    }
    /* Clear overrun flag if set (prevents stuck RX) */
    if (__HAL_UART_GET_FLAG(&hconsole, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&hconsole);
    }
}

/* UART4 RX interrupt handler -- 主从臂链路 (纯二进制帧, 无文本分流) */
void UART4_IRQHandler(void) {
    /* STM32F4 必须"先读 SR 再读 DR"才能清掉 RXNE/FE/NE/ORE —— 顺序不能反,
     * 否则标志早被 DR 读清掉, `lk` 的 FE/NE/ORE 永远是 0。
     * 一次读 SR 把三个错误标志一起取出登记, 再读 DR 取字节。
     * FE = 停止位采样不对(波特率/电平失配); NE = 采样点电平不稳(干扰);
     * ORE = 上一字节没取走(丢字节, 会让后续整段帧错位)。 */
    uint32_t sr = huart4.Instance->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t ch = (uint8_t)(huart4.Instance->DR);   /* 读 DR 清全部标志 */
        LinkProto_NoteRxErrors((sr & USART_SR_FE)  ? 1U : 0U,
                               (sr & USART_SR_NE)  ? 1U : 0U,
                               (sr & USART_SR_ORE) ? 1U : 0U);
        (void)LinkProto_FeedByte(ch);
    }
}

/* ===== 控制台输出: 全工程唯一出口 =====
 * ⚠⚠ 2026-09-18 现场事故(板子"上电十几秒必死、只发 h 也死、跟 CAN 无关"的真因):
 *   这个函数原来是
 *        int len = vsnprintf(buf, sizeof(buf), fmt, args);
 *        if (len > 0) HAL_UART_Transmit(&hconsole, (uint8_t*)buf, len, 100);
 *   而 C99 / newlib 的 vsnprintf **返回"本该写多少字节"**, 不是"实际写了多少"。
 *   `h` 的帮助表有 5406 字节, buf 只有 256 ⇒ len = 5406 ⇒ 从这 256 字节的
 *   **栈缓冲往外读 5150 字节**当数据发出去。现场表现完全吻合:
 *     · 帮助打了几行就变乱码, 断点正好在第 255 字节处(把一个词劈成两半);
 *     · 乱码里能认出启动时留在栈上的 "Limits: J1[63~303]...";
 *     · 属于**越界读**, 读穿 SRAM 顶端(0x20020000)就是 BusFault ⇒ 默认
 *       handler `while(1)` ⇒ 永久静默(USB 还在枚举、板子一个字都不说)。
 * 下面 con_write 统一做两件事: ① 传输前自愈句柄状态; ② 超长一律由调用方
 * 用 cprint_long 分块, cprint 自己则**钳位**——宁可截断也绝不越界读。 */
/* ---- 控制台输出**改道**开关 (2026-09-23) ----
 * 用途: 网口 `TEXT_EXEC` 要让控制台的 124 条命令整体可通过 UDP 下发。
 *       做法是把一次 ProcessCommand 的**全部回显**收进内存再回传, 所以
 *       需要一个"输出去哪"的间接层。默认直通真串口, 行为与以前完全一致。
 * ⚠ 只在主循环上下文内、且只包住一次 ProcessCommand; 用完立刻还原,
 *   否则真实控制台 (COM11) 会静默失声, 极难反查。 */
typedef void (*ConSinkFn)(const uint8_t*, uint16_t);

static void con_write_uart(const uint8_t* p, uint16_t n);   /* 前向 */

static ConSinkFn con_sink = con_write_uart;

static void con_write_uart(const uint8_t* p, uint16_t n) {
    /* 自愈: 上一次传输若被超时打断, 句柄可能停在非 READY, 之后每一次打印都会
     * 直接 HAL_BUSY 返回 —— 现象是"板子还在跑但再也不说话"。先复位再发。 */
    if (hconsole.gState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortTransmit(&hconsole);
    }
    (void)HAL_UART_Transmit(&hconsole, (uint8_t*)p, n, 1000);
}

static void con_write(const uint8_t* p, uint16_t n) {
    con_sink(p, n);
}

/* Printf via console UART。单次输出的成品必须 <= 255 字节(见上)。 */
static void cprint(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len <= 0) return;
    /* ⚠ 这一行是本次事故的修复点: C99 语义下 len 是"本该写多少", 必须钳位。 */
    if (len > (int)sizeof(buf) - 1) len = (int)sizeof(buf) - 1;
    con_write((const uint8_t*)buf, (uint16_t)len);
}

/* 打印**超长常量文本**(5406 字节的帮助表这一类)。
 * ⚠ 不能直接 cprint(超长字面量): cprint 的钳位会把帮助**截成 255 字节**。
 *   这里按换行切成 <=200 字节的块逐块发, 内容完整且每块都在缓冲范围内。
 * ⚠ 阻塞: 5.4 KB @115200 ≈ 470 ms。只在人敲 `h` 时走, 别在位置流期间调。 */
static void cprint_long(const char* s) {
    char chunk[208];
    while (*s != '\0') {
        uint16_t n = 0U, cut = 0U;
        while (s[n] != '\0' && n < 200U) {
            if (s[n] == '\n') cut = (uint16_t)(n + 1U);  /* 记住最后一个换行, 优先在那断 */
            n++;
        }
        if (s[n] != '\0' && cut > 0U) n = cut;   /* 还没到结尾 ⇒ 在换行处断开, 不打半行 */
        memcpy(chunk, s, n);
        con_write((const uint8_t*)chunk, n);
        s += n;
    }
}

/* ===== 网口文本透传: 输出捕获缓冲 (2026-09-23) =====
 * 2048 B 的取舍: `s`/`zi`/`LP`/`lk`/`ms`/诊断类都在 1 KB 以内 ⇒ 一页装得下;
 * 只有 `h`(5406 B 帮助表) 这类会截断, 但**截断是有标记的**(flags bit0),
 * 上位机不会把半截当完整。刻意不做成 8 KB: 那 4 KB 换来的只是"帮助能一次
 * 读完", 而帮助本来就是静态文本, 给文档看更合适。 */
#define ARM_TEXT_BUF_SZ   2048U
static char     g_text_buf[ARM_TEXT_BUF_SZ];
static uint16_t g_text_len   = 0U;
static uint8_t  g_text_trunc = 0U;

static void con_capture(const uint8_t* p, uint16_t n) {
    uint16_t room = (uint16_t)(ARM_TEXT_BUF_SZ - g_text_len);
    uint16_t k = (n < room) ? n : room;
    if (k) memcpy(&g_text_buf[g_text_len], p, k);
    g_text_len = (uint16_t)(g_text_len + k);
    if (k < n) g_text_trunc = 1U;      /* 满了只置标记, 绝不越界写 */
}

/* ===== 主循环进度标记 (2026-09-18) =====
 * TIM2 中断(与主循环独立)每次进来检查 g_main_progress_ms 有没有推进; 超过
 * ARM_STALL_MS 没动就记一条"主循环卡死"进 CCM RAM, 下次开机自报(见 arm_fault.h)。
 * 这样"CPU 故障 / 主循环卡死 / 掉电"三种同样静默的死法就能分开。
 * ⚠ main_progress() 里**先写 phase/cmd, 最后写时间戳**: ISR 以时间戳变化作为
 *   "推进过"的判据, 先写分支标记才能保证它看到的 phase/cmd 属于这一次推进。 */
#define ARM_PHASE_IDLE  1U   /* 命令循环空转 */
#define ARM_PHASE_CMD   2U   /* 正在执行一条控制台命令 */
#define ARM_PHASE_TRAJ  3U   /* 50Hz 轨迹/示教 tick */

volatile uint32_t g_main_progress_ms = 0U;
volatile uint32_t g_main_phase = ARM_PHASE_IDLE;
volatile uint32_t g_main_cmd   = 0U;

static void main_progress(uint32_t phase, uint32_t cmd) {
    g_main_phase = phase;
    g_main_cmd   = cmd;
    g_main_progress_ms = HAL_GetTick();
}

/* 8 次采样位图 -> "00001111" 串 (左=第 1 次采样)。buf 至少 9 字节。
 * ⚠ 必须由调用者给缓冲: 同一次 cprint 里出现两个位图时, 若共用一个 static
 *   缓冲, 两个 %s 会打出同一个值(经典坑)。 */
static const char* bin8(uint8_t pat, char* buf) {
    for (uint8_t i = 0U; i < 8U; i++) {
        buf[i] = (pat & (uint8_t)(1U << i)) ? '1' : '0';
    }
    buf[8] = '\0';
    return buf;
}

/* 精简版 newlib 默认未链接 printf 浮点格式化；控制台参数回显必须
 * 避免使用 %f。此函数以固定 3 位小数输出，并保留数值符号。 */
static void cprint_fixed3(float value) {
    char sign = '+';
    if (value < 0.0f) {
        sign = '-';
        value = -value;
    }
    uint32_t scaled = (uint32_t)(value * 1000.0f + 0.5f);
    cprint("%c%lu.%03lu", sign,
           (unsigned long)(scaled / 1000U),
           (unsigned long)(scaled % 1000U));
}

/* Receive a single line from console (non-blocking, reads from ring buffer) */
static uint8_t console_rx[128];
static uint16_t console_rx_len = 0;
/* 控制台回显开关 (`ec 0|1`), 默认 ON = 与旧版行为一致。
 * 关掉后从臂收到的每行 `#P` 不再被逐字符打回 PC —— 省 35 次阻塞发送/帧,
 * 也就是省掉从臂 ~2.2 kB/s 的发送线占用和挤掉收帧的主循环时间。 */
static uint8_t console_echo_en = 1U;

/* 从环形缓冲区逐字节组装一行命令；函数始终非阻塞。 */
static int Console_TryReadLine(void) {
    /* Check if any character available in ring buffer */
    if (rx_head == rx_tail) return 0;   /* buffer empty */

    uint8_t ch = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

    if (ch == '\r' || ch == '\n') {
        if (console_rx_len == 0) return 0;   /* skip empty lines */
        console_rx[console_rx_len] = '\0';    /* null-terminate for strtok */
        if (console_echo_en) cprint("\r\n");
        int len = console_rx_len;
        console_rx_len = 0;
        return len;
    }
    if (ch == 0x08 || ch == 0x7F) {  /* Backspace */
        if (console_rx_len > 0) {
            console_rx_len--;
            if (console_echo_en) cprint("\b \b");
        }
        return 0;
    }
    if (console_rx_len < sizeof(console_rx) - 1) {
        console_rx[console_rx_len++] = ch;
        /* Echo: 逐字符**阻塞**发送 (115200 下每字符 ~87us)。一帧 `#P` 35
         * 字符 ⇒ 35 次阻塞发送 + 35 拍主循环 ≈ 3 ms/帧。65 Hz 下白占从臂
         * 发送线 ~2.2 kB/s, 并挤掉收帧时间 ⇒ 环溢出 ⇒ 丢字节。`ec 0` 可关。 */
        if (console_echo_en) HAL_UART_Transmit(&hconsole, &ch, 1, 10);
    }
    return 0;
}

/* ===== LED indicators ===== */
/* net 构建: PC5/PC4 = ETH_RXD1/RXD0, LED 让位 (用户 2026-09-20 确认)
 * both 构建: 由**本次启动 NET 外设是否初始化**决定(0924CE) —— 初始化过就
 *   整段让位, 且 auto 模式下**永不让回**(active 是动态的, 静默期也不能把
 *   RMII 引脚当 LED 翻)。判据必须是 LinkCfg_NetHwInited(), 不能 LINK_IS_NET()。 */
static void LED_R(uint8_t on) {
#if ARM_LINK_MULTI
    if (!LinkCfg_NetHwInited()) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        (void)on;
    }
#elif !ARM_LINK_NET
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}
static void LED_G(uint8_t on) {
#if ARM_LINK_MULTI
    if (!LinkCfg_NetHwInited()) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        (void)on;
    }
#elif !ARM_LINK_NET
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}
static void LED_B(uint8_t on) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* ===== 状态指示灯 PE4~PE7 (0924CN, 实用版语义) =====
 * PE4=电源灯   常亮 (gpio.c 上电即置高, 本文件只在自检失败时回灭)
 * PE5=状态灯   三态一眼读懂:
 *              慢闪 1Hz = 力矩全关 —— "臂是软的, 可以手掰"(示教/校准/解除)
 *              快闪 5Hz = 运动中(轨迹/遥操/回放/RC/单轴, 目标下发 300ms 内)
 *              常亮     = 力矩 ON 静止 —— "刚性锁定待机"
 * PE6=报警灯   (0924DA: 本板低电平点亮) 0924CZ 优先级: 软件急停(S 下发)常亮
 *              > 总线丢读超标快闪 4Hz > 灭=正常
 * PE7=通信灯   每收到一条指令翻转一次(流量越大闪越快), 2s 无指令自动熄灭 */
static void LED_Power(uint8_t on) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void LED_Run(uint8_t on)   { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
/* ⭐0924DA: 报警灯极性宏 —— 0924CWb 新板实测 PE6 报警灯为**低电平点亮**
 * (接 3.3V, S 急停拉高反而灭灯, 用户 2026-11-24 实锤)。换回高有效板
 * (如旧 0924L) 时置 0 重编即可。 */
#ifndef ARM_ALARM_LED_ACTIVE_LOW
#define ARM_ALARM_LED_ACTIVE_LOW 1
#endif
static void LED_Alarm(uint8_t on) {
#if ARM_ALARM_LED_ACTIVE_LOW
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}
static void LED_Comm(uint8_t on)  { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

static uint32_t g_last_cmd_tick = 0U;   /* 最近一次收到指令的时刻, 0=开机以来没有 */
static uint8_t  g_comm_st = 0U;         /* 通信灯翻转态 (0924CN: 每条指令翻一次) */
/* ⭐0924CZ 软件急停标志: 下发 `S` 后置位 (报警灯常亮), `i`/`t J 1` 上力矩
 * 或再次开始运动后自动清除。用户定案: 本系统急停=手动下发 S, 无物理
 * 急停按键, 报警灯跟软件态走, 不再读 PE11。 */
static uint8_t  g_soft_stop = 0U;
static uint32_t g_soft_stop_tick = 0U;  /* S 下发时刻: 运动自动清除要晚于此 */
static uint32_t g_boot_tick = 0U;       /* main() 初始化完成时刻 */
/* 总线健康窗口计数 (0924CN): FeedbackSampleService 10ms 节拍逐轴采样,
 * 2s 结算一次失败率 —— 新板排查丢读的常开仪表, 不用再手动 d p */
static uint16_t g_busw_ok = 0U, g_busw_err = 0U;

static void Led_CommMark(void) {
    g_comm_st ^= 1U;                    /* 每条指令翻转: 流量大=快闪, 直观 */
    g_last_cmd_tick = HAL_GetTick();
}

/* ⭐ 挂在主循环 50Hz 轨迹节拍上 (traj_do_update 分支), 20ms 一拍 */
static void Led_Service(void) {
    static uint8_t slow_div = 0U, slow_st = 0U;   /* 1Hz 慢闪相位 (50 拍) */
    static uint8_t fast_div = 0U, fast_st = 0U;   /* 5Hz 快闪相位 (10 拍) */
    static uint8_t bus_bad = 0U;
    static uint16_t bus_win_div = 0U;             /* 100 拍 = 2s 结算窗 */
    uint32_t now = HAL_GetTick();

    if (++slow_div >= 50U) { slow_div = 0U; slow_st ^= 1U; }
    if (++fast_div >= 10U) { fast_div = 0U; fast_st ^= 1U; }

    /* ---- PE5 状态灯: 力矩/运动三态 ----
     * 运动判定用驱动层"最近目标下发"活动戳 (SERVO_LastGoalTick): 轨迹/
     * 遥操/回放/RC/单轴全走那几个写入函数, 300ms 无新目标即视为静止。 */
    uint8_t tq = SERVO_TorqueOnCount();
    uint32_t last_goal = SERVO_LastGoalTick();
    uint8_t moving = (last_goal != 0U) &&
                     ((uint32_t)(now - last_goal) < 300U);
    if (tq == 0U)    LED_Run(slow_st);   /* 全松力矩: 慢闪 "臂是软的" */
    else if (moving) LED_Run(fast_st);   /* 运动中: 快闪 */
    else             LED_Run(1);         /* 刚性锁定待机: 常亮 */

    /* ---- PE6 报警灯 (优先级从高到低, 0924CZ 改) ----
     * ①软件急停(下发过 S): 常亮; `i`/`t J 1` 上力矩或 S 之后又有新运动
     *   目标(晚于 S 时刻) ⇒ 自动解除。
     * ②总线丢读率>30%: 快闪 4Hz (用户确认保留)。
     * ③上次卡死重启慢闪 5s: 用户要求删除。 */
    uint8_t alarm = 0U;
    if (g_soft_stop) {
        alarm = 1U;
        if (moving && last_goal > g_soft_stop_tick) {
            g_soft_stop = 0U;             /* S 之后又开始动了: 解除急停态 */
        }
    } else if (bus_bad) {
        alarm = fast_st;                              /* 总线丢读超标: 快闪 */
    }
    LED_Alarm(alarm);

    /* 总线健康 2s 结算: 采样失败率 >30% 判差 (窗口样本 <50 个不判, 防开机误报) */
    if (++bus_win_div >= 100U) {
        bus_win_div = 0U;
        uint16_t tot = (uint16_t)(g_busw_ok + g_busw_err);
        bus_bad = (tot >= 50U) && (((uint32_t)g_busw_err * 100U) / tot > 30U);
        g_busw_ok = 0U; g_busw_err = 0U;
    }

    /* ---- PE7 通信灯: 翻转态 + 2s 无指令熄灭 ---- */
    LED_Comm(g_comm_st && (g_last_cmd_tick != 0U) &&
             ((uint32_t)(now - g_last_cmd_tick) < 2000U));
}

static void cmdHelp(void);
static void cmdLink(int ntok, char** tokens);   /* link: 链路偏好(2026-09-22) */
static void SystemClock_Config(void);  /* forward decl */

/* ===== E-Stop & limit switch ===== */
uint8_t IsEStop(void) {          /* 导出：arm_control_api 使用（见 arm_main_exports.h） */
    return !HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);  /* active low */
}

static uint8_t IsLimit(uint8_t joint) {
    if (joint >= 6) return 0;
    uint16_t pins[] = {
        GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2,  /* J1-J3 on PG */
        GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5   /* J4-J6 on PG */
    };
    return !HAL_GPIO_ReadPin(GPIOG, pins[joint]);  /* active low */
}

/* ===== 单总线形态判定 (2026-11-02, 0924AZ) =====
 * 两种板的物理形态不同, 但"6 轴全在一条 UART5 上"这一点相同:
 *   ARM_MASTER_BUILD —— 主臂: 已拆 74HC126, 开漏单总线;
 *   BUS_ONE          —— 集成板: 74HC126 焊死拆不掉, 把门1 的 1Y 与门3 的 3Y
 *                       短接成一条总线(门2/门4 的 OE 与 A 接地闲置)。
 * 判定为真时: 只扫 Bus B 的 ID 1..6, 并且**绝不触碰 BUS_A**。
 * ⚠ BUS_ONE 下 DIR_A 一旦被拉高, 门1 使能后与门3 的输出去驱动同一条线
 *   = 两个推挽对打。所以这里连"扫一下 Bus A"都不允许。
 */
#if ARM_MASTER_BUILD || (defined(BUS_ONE) && BUS_ONE)
  #define ARM_ONE_BUS_BUILD 1
#else
  #define ARM_ONE_BUS_BUILD 0
#endif

/* ===== 单总线形态的"那条线"到底是谁 (2026-11-02) =====
 * 两种板都是"6 轴一条线", 但**不是同一条线**:
 *   主臂   ROLE=master —— 已拆 74HC126, 6 颗舵机现接 USART3 (PB10/PB11) = BUS_A。
 *                           原设计在 UART5(PC12/PD2), 因 UART5 疑似失效于 11-02 改线。
 *   集成板 BUS_ONE=1   —— 74HC126 拆不掉, 1Y+3Y 短接, 仍是 UART5(PC12/PD2) = BUS_B。
 * ⚠ 历史坑: 这些宏出现前, 单总线分支把 "Bus B (UART5)" 和 found_b/count_b 写死在
 *   cmdScan / cmdFullScan / sid 里。主臂改接 USART3 之后 ScanBothBuses() 确实扫的是
 *   BUS_A 并存进 found_a, 但打印仍读 found_b ⇒ **6 颗全在线却逐条打 MISSING**
 *   (只有尾行 Total 用 count_a+count_b 才露真相)。所有单总线路径必须走下面这组别名。 */
#if ARM_ONE_BUS_BUILD
  #if ARM_MASTER_BUILD
    #define ONE_BUS          BUS_A
    #define ONE_BUS_TAG      "A"
    #define ONE_BUS_PORT     "USART3 (PB10/PB11)"
    #define ONE_BUS_SHORT    "USART3"
    #define ONE_BUS_CNT      count_a
    #define ONE_BUS_FOUND    found_a
  #else
    #define ONE_BUS          BUS_B
    #define ONE_BUS_TAG      "B"
    #define ONE_BUS_PORT     "UART5 (PC12/PD2)"
    #define ONE_BUS_SHORT    "UART5"
    #define ONE_BUS_CNT      count_b
    #define ONE_BUS_FOUND    found_b
  #endif
#endif

/* ===== Scan buses on startup =====开始扫描的应用层
 * 固定布局识别: BusA 期望 ID 1/2/3 (J1-J3), BusB 期望 ID 4/5/6 (J4-J6)。
 * 判定参照 ESP32 改 ID 工具: 完整 6B 回应 + 帧头 FF FF + ID 回显匹配。
 * 只 Ping 期望 ID (旧版全量扫 253 个 ID, 空总线一个等 80ms, 慢 ~80 倍)。 */
#if ARM_ONE_BUS_BUILD
/* 单总线: 6 颗全部并联在一条线上, ID 1..6 都要扫。
 * 主臂走 BUS_A(USART3), 集成板走 BUS_B(UART5), 两个缓冲都要有。 */
static const uint8_t EXPECT_ALL[6] = {1U, 2U, 3U, 4U, 5U, 6U};
static uint8_t    found_a[8], found_b[8];
#else
    /* 0924BD: 不再有"编译期固定 ID 分组"的 EXPECT_A/EXPECT_B —— 扫描按
     * JOINT_BUS[] 里当前的实际路由分组, 单线板子上才不会漏数。 */
    static uint8_t    found_a[8], found_b[8];
#endif
static uint8_t    count_a, count_b;
/* 0924DA: 主臂在线探测 (Bus A ID 7~12, 双总线构建专用) */
static uint8_t    found_ma[8];
static uint8_t    count_ma;

static void ScanBothBuses(void) {
#if ARM_ONE_BUS_BUILD
    /* 单总线形态: 舵机全在一条线上。
     * 主臂 = USART3 (BUS_A, UART5 已旁路); 集成板 = UART5 (BUS_B, 1Y+3Y 短接)。 */
    count_a = 0U;
    count_b = 0U;
  #if ARM_MASTER_BUILD
    count_a = DXL_ScanIds(BUS_A, EXPECT_ALL, 6, found_a, 8, 120);
  #else
    count_b = DXL_ScanIds(BUS_B, EXPECT_ALL, 6, found_b, 8, 120);
  #endif
#else
    /* 0924BD: 按**当前路由**把六个 ID 分到两条总线再扫。
     * 旧写法把 ID1-3 固定派给 Bus A、ID4-6 固定派给 Bus B(EXPECT_A/EXPECT_B),
     * 在"六颗全接 B 段"的单线板子上必然扫成 "Bus A: 0 / Bus B: 3" ——
     * 那 3 颗不是"在线 3 颗", 而是另外 3 颗被派去查了一条空线。
     * 现在 A 段分不到 ID 就整段跳过(省时间), 计数与真实拓扑一致。 */
    uint8_t id_a[6], id_b[6];
    uint8_t n_a = 0U, n_b = 0U;
    for (uint8_t j = 0U; j < 6U; j++) {
        if (JOINT_BUS[j] == BUS_A) id_a[n_a++] = JOINT_ID[j];
        else                       id_b[n_b++] = JOINT_ID[j];
    }
    count_a = (n_a != 0U) ? DXL_ScanIds(BUS_A, id_a, n_a, found_a, 8, 120) : 0U;
    if (n_a != 0U && n_b != 0U) HAL_Delay(50);
    count_b = (n_b != 0U) ? DXL_ScanIds(BUS_B, id_b, n_b, found_b, 8, 120) : 0U;

    /* ⭐0924DA: 主臂在线探测 (fi 板内遥操用) —— Bus A 上主臂舵机 ID 7~12,
     * 旧扫描只 ping 本臂关节 ID(1~6), 主臂挂着也报 "Bus A: 0"(2026-11-24
     * 用户实测)。这里单独扫 7~12 存 count_ma/found_ma, 开机单独一行显示,
     * fi 就绪情况一眼可读。主臂没接时空扫最多 ~0.7s(6 ID×120ms 超时)。 */
    {
        static const uint8_t MASTER_IDS[6] = {7U, 8U, 9U, 10U, 11U, 12U};
        count_ma = DXL_ScanIds(BUS_A, MASTER_IDS, 6, found_ma, 8, 120);
        /* 0924DD: 总线随机丢帧实测(每次开机漏的 ID 都不一样: 4~5/6 乱跳),
         * 对没扫到的 ID 各补一轮单独 ping, 把误报"离线"压下去。
         * fi 运行时是直读不受此影响, 这里只管开机那行的显示可信度。 */
        if (count_ma < 6U) {
            uint8_t found2[8];
            uint8_t n2 = DXL_ScanIds(BUS_A, MASTER_IDS, 6, found2, 4, 120);
            for (uint8_t p2 = 0U; p2 < n2; p2++) {
                uint8_t seen = 0U;
                for (uint8_t q2 = 0U; q2 < count_ma; q2++)
                    if (found_ma[q2] == found2[p2]) { seen = 1U; break; }
                if (!seen && count_ma < 8U) found_ma[count_ma++] = found2[p2];
            }
            /* 恢复升序显示 */
            for (uint8_t s2 = 0U; s2 < count_ma; s2++)
                for (uint8_t t2 = (uint8_t)(s2 + 1U); t2 < count_ma; t2++)
                    if (found_ma[t2] < found_ma[s2]) {
                        uint8_t tmp = found_ma[s2];
                        found_ma[s2] = found_ma[t2];
                        found_ma[t2] = tmp;
                    }
        }
    }
#endif
}

/* ===== Buzzer ===== */
static void BuzzerBeep(uint32_t ms) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

/* ===== Trajectory state ===== */
static volatile uint16_t traj_tick_div = 0;   /* 2kHz divider -> 50Hz packet scheduler */
static volatile uint8_t  traj_do_update = 0;   /* flag set by TIM2 */
static uint32_t          traj_tick = 0;        /* 200Hz S-curve interpolation step counter */

/* ===== 轨迹到位确认 (settle) =====
 * S 曲线按时间推进规划目标, 不等舵机实际位置 —— STS3215 扭矩余量足够,
 * 规划器跑在前面产生的落后量由舵机内部位置环自行补齐。
 * 曲线结束后要求六轴全部进入目标 1° 内并持续稳定 500ms 才算运动完成,
 * 完成后才放行 correction / RC 自动重武装等后续流程。 */
#define TRAJ_SETTLE_TOL_DEG           1.0f    /* 单轴到位容差 */
#define TRAJ_SETTLE_POLL_MS           80U     /* 到位轮询周期 */
#define TRAJ_SETTLE_STABLE_MS         500U    /* 容差内需持续的时长 */
static uint8_t  traj_settle_pending = 0U;      /* 曲线已走完, 正在等实际到位 */
static uint8_t  traj_settle_joint = 0U;        /* 当前轮询的轴 */
static uint8_t  traj_settle_mask = 0U;         /* 已确认到位的轴位图 */
static uint8_t  traj_settle_pass = 1U;         /* 轮转轮次(全轴到位后可再来一轮复核) */
static uint32_t traj_settle_next_poll = 0U;    /* 下次读反馈的时刻 */
static uint32_t traj_settle_deadline = 0U;     /* 5s 超时兜底(机械稳不到 1° 时强制放行) */
static uint32_t traj_settle_stable_since = 0U; /* 单轴进入容差的起始时刻 */
static uint8_t  traj_settle_adaptive = 0U;     /* 已根据滞后量设自适应收尾速 */

/* ===== 轨迹规划诊断输出 (dt 1/0) =====
 * 置 1 后每个插补步打印规划目标角(下发前), 离线判曲线是否平滑用。 */
uint8_t  diag_plan_enable = 0U;

/* ===== 尾段参数 (SP 42/43/45) =====
 * 0924AP: 这些参数保留给 LP/SP 显示与保存, 但代码中不再执行收尾降速
 * 与尾段交接, 避免限制轨迹正常运行。默认值不再影响运动。 */
float TRAJ_APPROACH_SPEED = 5.0f;    /* (保留参数, 0924AP 起不影响运动) */
float TRAJ_APPROACH_FRAC  = 0.05f;   /* (保留参数, 0924AP 起不影响运动) */
float TRAJ_HANDOFF_DEG    = 0.0f;    /* (保留参数, 0924AP 起不影响运动) */

/* ===== Periodic real-time feedback =====
 * pv <interval_ms>: print actual angles of all six joints periodically.
 * pf <interval_ms>: calculate FK from actual angles and print end coordinates.
 * pv 0 / pf 0 stops the respective polling. These commands run in main loop. */
static uint32_t feedback_period_ms = 0;
static uint32_t feedback_next_tick = 0;
static uint32_t pose_feedback_period_ms = 0;
static uint32_t pose_feedback_next_tick = 0;

/* ===== 单总线"边发边收"遥测 (2026-09-10) =====
 * 半双工单总线物理上不可能同时收发: 任一时刻总线要么在发、要么在收。
 * 要做到"运动中实时回传且不干扰运动", 靠的是【时间片轮转 + 帧长快路径】:
 *
 *   1) 帧长快路径 (servo_bus_hal.c receive_common): 状态帧自带 Length,
 *      定位到 `FF FF <id>` 后收满即返回, 单次读 5ms -> ~0.35ms。
 *      旧路径 6 轴一次要 ~30ms(每次等 5ms 静默判停) —— 直接吃掉一个半
 *      50Hz 控制节拍, 舵机内部 profile 被拖成走走停停 = 观感上的抖动。
 *
 *   2) 轮转采样: 每个 20ms 控制节拍只读 1 个关节(整组状态 2 次事务,
 *      ~0.7ms), 6 拍(120ms)轮完 6 轴 -> 总线占用约 3.5%。
 *
 *   3) 采样点固定在 6 轴 GoalPos 批量下发【之后】 —— 绝不插进同步写中间,
 *      否则 6 轴不会同时起跑, 机械上就是歪的。
 *
 *   4) 打印只读缓存, 与总线采样彻底解耦: 打印周期设多快都不会增加总线负载
 *      (唯一的代价是控制台 UART 阻塞: 115200 下每字符 ~87us, 一屏 100 字符
 *      约 9ms —— 所以运动中使用建议 >=100ms 周期, 或提高控制台波特率)。
 *
 * `pv`/`pf` 走的也是同一份缓存, 所以 pv 的打印频率不再影响总线负载。 */
/* ⭐0924CI: 采样节拍 20->10ms (全表刷新 120->60ms) —— GET_STATUS 改走这份
 * 缓存(见 ArmMain_GetCachedJointDeg), 30Hz /joint_states 轮询要求每轴
 * 数据新鲜度 <100ms; 总线占用 ~0.7ms/10ms = 7%, 与轨迹 50Hz 同拍安全性
 * 由调用点顺序保证(采样永远在 GoalPos 批量下发之后), 节拍缩短不影响。 */
#define TELEM_SAMPLE_PERIOD_MS  10U     /* 采样节拍 (0924CI: 20->10ms) */
#define TELEM_CACHE_TTL_MS      600U    /* 缓存有效期: 超时视为无效(显示 --) */
/* 字段位掩码 */
#define TELEM_F_POS   0x01U   /* p 角度 */
#define TELEM_F_TEMP  0x02U   /* t 温度 */
#define TELEM_F_CUR   0x04U   /* i 电流 */
#define TELEM_F_VOLT  0x08U   /* v 电压 */
#define TELEM_F_SPD   0x10U   /* s 速度 */
#define TELEM_F_LOAD  0x20U   /* l 负载 */
#define TELEM_F_ALL   0x3FU

static uint32_t telem_period_ms   = 0U;   /* 打印周期, 0=关闭遥测 */
static uint32_t telem_sample_next = 0U;   /* 下次采样时基 */
static uint32_t telem_print_next  = 0U;   /* 下次打印时基 */
static uint8_t  telem_joint       = 0U;   /* 轮转指针 0~5 */
static uint8_t  telem_fields      = (TELEM_F_POS | TELEM_F_TEMP | TELEM_F_CUR);
static SERVO_Telemetry_t telem_cache[6];
static uint32_t telem_stamp[6];           /* 该关节上次采样成功时刻(0=从未) */
static uint8_t  telem_err[6];             /* 1 = 上次采样失败 */

/* 是否需要后台采样: ⭐0924CI 起常开 —— GET_STATUS(0x02) 改走遥测缓存,
 * 必须有人持续采样缓存才有意义。代价 ~0.7ms/10ms (7% 总线占用),
 * 换 GET_STATUS 从 ~20ms 降到 <1ms。pv/pf/tm 依旧共用这份缓存。 */
static uint8_t feedback_sampler_on(void) {
    (void)feedback_period_ms; (void)pose_feedback_period_ms; (void)telem_period_ms;
    return 1U;
}

/* 缓存是否可用: 采样成功且未过期 */
static uint8_t telem_ok(uint8_t j, uint32_t now) {
    if (j >= 6U || telem_err[j] != 0U || telem_stamp[j] == 0U) return 0U;
    return ((uint32_t)(now - telem_stamp[j]) < TELEM_CACHE_TTL_MS) ? 1U : 0U;
}

/* 轮转采样 1 个关节。调用点必须在轨迹批量下发之后。 */
static void FeedbackSampleService(void) {
    if (!feedback_sampler_on()) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - telem_sample_next) < 0) return;
    telem_sample_next = now + TELEM_SAMPLE_PERIOD_MS;

    uint8_t j = telem_joint;
    telem_joint = (uint8_t)((j + 1U) % 6U);

    if (SERVO_ReadTelemetry(j, &telem_cache[j])) {
        telem_stamp[j] = now;
        telem_err[j]   = 0U;
        if (g_busw_ok < 0xFFFFU) g_busw_ok++;    /* 0924CN: 总线健康窗 */
    } else {
        telem_err[j]   = 1U;   /* 保留旧数据但标记无效 */
        if (g_busw_err < 0xFFFFU) g_busw_err++;
    }
}

/* 按字段打印缓存(不碰总线)。 */
static void TelemetryPrintService(void) {
    if (telem_period_ms == 0U) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - telem_print_next) < 0) return;
    telem_print_next = now + telem_period_ms;

    if (telem_fields & TELEM_F_POS) {
        cprint("TM p:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now)) cprint(" %5.1f", servo_raw_to_deg(telem_cache[j].pos));
            else                   cprint("    --");
        }
        cprint("\r\n");
    }
    if (telem_fields & TELEM_F_TEMP) {
        cprint("TM t:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now)) cprint(" %4d", (int)telem_cache[j].temp);
            else                   cprint("   --");
        }
        cprint("   (C)\r\n");
    }
    if (telem_fields & TELEM_F_CUR) {
        cprint("TM i:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now) && telem_cache[j].cur_valid)
                cprint(" %4d", (int)(((int32_t)telem_cache[j].current * 13) / 2)); /* 6.5mA/LSB */
            else cprint("   --");
        }
        cprint("   (mA)\r\n");
    }
    if (telem_fields & TELEM_F_VOLT) {
        cprint("TM v:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now)) cprint(" %4.1f", (float)telem_cache[j].voltage * 0.1f);
            else                   cprint("   --");
        }
        cprint("   (V)\r\n");
    }
    if (telem_fields & TELEM_F_SPD) {
        cprint("TM s:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now)) cprint(" %4d", (int)telem_cache[j].speed);
            else                   cprint("   --");
        }
        cprint("\r\n");
    }
    if (telem_fields & TELEM_F_LOAD) {
        cprint("TM l:");
        for (uint8_t j = 0; j < 6U; j++) {
            if (telem_ok(j, now)) cprint(" %4d", (int)telem_cache[j].load);
            else                   cprint("   --");
        }
        cprint("\r\n");
    }
}

/* Unified 舵机 internal Moving Speed (address 32) for all six servos.
 * This is distinct from the S-curve trajectory speed in deg/s. */
static uint16_t all_servo_moving_speed = STS_DEFAULT_GOAL_SPEED;

/* ===== User-defined point loop state =====
 * B/BG: two-point loop. Q: up to 8 saved joint-angle points.
 * rounds=0 means infinite loop. */
#define MULTI_POINT_MAX 8
/* A point is considered reached only when every joint feedback is within this error. */
#define MULTI_POINT_REACHED_TOL_DEG 2.0f
/* 到位判定读位置的超时。必须短: 该判定在轨迹结束后的等待期被主循环反复调用
 * (cmdMultiPointService), 用 100ms 默认超时的话, 一轴连续读失败就把主循环
 * 按 100~200ms/趟 拖住 —— 控制台、遥测、软件 E-STOP 一起停摆, 而第一条
 * 线索 0.5s 后才打印。10ms 与 SERVO_GetPositionRawTO / ReadTelemetry 一致。 */
#define USED_POS_READ_TIMEOUT_MS 10U
/* ===== 2026-09-17: 到位判定的三处加固 (现场: QL 卡在点 1 永不前进) =====
 * 现场指纹: `QL` 之后只打一句 "QL: J6 feedback blocked - corrupt frame", 循环
 * 再也不动; 之后的 `f` 六轴全能读到, 说明不是"轴坏了"而是**判位这件事本身**
 * 在 1Mbps 半双工总线上做不下去。三条独立原因, 一起改:
 *
 * ① 风暴读: 旧版每趟主循环都读六轴(且本服务在同一趟被调用两次 ⇒ 最多 12 次
 *    事务/趟), 主循环空转时等于对总线连续读, 没有一微秒空闲。总线上每次读都是
 *    "每字节 10us 硬实时"的接收, 被中断抢占丢 1 字节即整帧作废 —— 读得越密,
 *    撞上丢字节的概率越高, 而判位要求"六轴一次全过" ⇒ 越读越判不出来。
 *    60ms 一次判定足够(稳定窗口本就是 500ms), 总线占用降到约 1/50。
 * ② 裸读: 判位用单次读(+1 次立即重试), 是全固件最后一处没有多数表决的读。
 *    `f`/`P 0`/`g` 早已统一改用 read_pos_vote()(3 采样多数表决 + 重试 + 间隔),
 *    就是为了治这个 1Mbps 丢字节。判位改用同一套。
 * ③ 永不放弃: 只要有一轴够不到 2.0deg、或某轴一直读不到, 循环就永久卡死,
 *    而且只打一句误导性的 "corrupt frame"(它只报"最后一个失败轴", 其它轴
 *    的失败全被吞掉)。加"单点确认超时" + **逐轴真相报告**, 然后放行到下一点。 */
#define MULTI_POINT_POLL_PERIOD_MS 60U
/* 单点确认超时(从"轨迹结束、开始等判位"算起)。超时则打印逐轴 目标/实测/误差
 * + 各轴读失败次数 + 失败原因 + 疑似不动那轴的**力矩开关**状态, 然后进入下一点。
 * 不设上限 ⇒ 现场只能看到"循环不动", 完全看不出是哪一轴、差多少、为什么。 */
#define MULTI_POINT_CONFIRM_TIMEOUT_MS 8000U
/* 单轴"最近一次有效读数"的有效期: 判位期间某轴偶发读不到时先用这份缓存兜底,
 * 不让一次丢字节把整个点判废(过期则视为真的读不到)。 */
#define MULTI_POINT_LASTGOOD_MS 400U
/* 最近一次到位判定读失败的具体原因, 供 cmdMultiPointService 打印。
 * 刻意与"check bus/cable"区分: 舵机告警(过载/过温)和帧损坏都不是线缆问题。 */
static uint8_t  mp_fail_joint = 0U;
static uint8_t  mp_fail_diag  = DXL_RD_OK;
static uint8_t  mp_fail_err   = 0U;
/* 逐轴统计(旧版只有"最后一个失败轴", 会把早早就在偶发失败的其它轴藏掉) */
static uint8_t  mp_fail_cnt[6] = {0};      /* 本段各轴读失败次数 */
static uint8_t  mp_fail_dg[6]  = {0};      /* 各轴最近一次失败原因 DXL_RD_* */
static uint8_t  mp_fail_eb[6]  = {0};      /* 各轴最近一次失败 Err 字节 */
static float    mp_lastgood[6] = {0};      /* 各轴最近一次有效读数(deg) */
static uint32_t mp_lastgood_ms[6] = {0};   /* 该读数时刻, 0 = 无 */
static uint32_t multi_point_poll_next = 0U;   /* 判位节流: 下次允许判定的时刻 */
static uint32_t multi_point_leg_since = 0U;   /* 本段"开始等判位"的时刻, 0 = 未开始 */
static uint8_t  multi_point_readfail_cnt = 0U;
/* 段(两点之间)开始/结束时复位逐轴统计, 定义在 cmdMultiPointTargetReached 之后 */
static void multi_point_leg_reset(void);
static uint8_t  point_loop_active = 0;
static uint8_t  point_loop_phase = 0;       /* 0=A, 1=B */
static uint16_t point_loop_legs_left = 0;   /* 0xFFFF=infinite */
static float    point_loop_a[6];
static float    point_loop_b[6];
static float    point_loop_speed = 30.0f;
static float    multi_points[MULTI_POINT_MAX][6];
static uint8_t  multi_point_count = 0;
static uint8_t  multi_point_index = 0;
static uint16_t multi_point_legs_left = 0;
static uint8_t  multi_point_started = 0;    /* initial move to point 1 pending */
static uint8_t  multi_point_waiting = 0;   /* dwell before next segment */
static uint32_t multi_point_wait_until = 0;
static float    multi_point_segment_speed[MULTI_POINT_MAX] =
    {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f};
static uint32_t multi_point_dwell_ms[MULTI_POINT_MAX] = {0};
static float    multi_point_speed = 30.0f;
/* QLC: 连续流水线多点循环(段间不停顿/不测量/不驻留), 复用 multi_points[] */
static uint8_t  qlc_active = 0;
static uint8_t  qlc_started = 0;    /* 初始段(去点1)还没算段数 */
static uint8_t  qlc_index = 0;
static uint8_t  qlc_count = 0;
static uint32_t qlc_legs_left = 0;  /* 0xFFFFFFFF=无限 */

/* ===== Hand-teaching mode (continuous record + timed replay) =====
 * teach_mode_active: 1 = torque OFF and continuous auto-sampling.
 * While the user drags the arm by hand, every 50 Hz main-loop tick snapshots
 * the six Present Position raw values into teach_rec[]. No key press is needed
 * between points: teach == start recording.
 * play replays the samples at the recorded rate (same 50 Hz tick), so the
 * taught motion -- including gripper J6 open/close timing -- is reproduced.
 * Samples are raw servo units (0..4095); replay sync-writes them raw, so the
 * trajectory is immune to calibration drift. The buffer survives until
 * 'rec clear' or until the next 'teach'. */
#define TEACH_REC_MAX        4000U   /* 4000 x 12B = 48KB, ~80s @50Hz(单条临时录音) */
#define TEACH_REC_MS         20U     /* one sample per 50 Hz tick */
#define TEACH_PLAY_NONE      0U
#define TEACH_PLAY_RAMP      1U      /* S-curve return to recorded start */
#define TEACH_PLAY_STREAM    2U      /* replay one sample per 50 Hz tick */
#define TEACH_STREAM_SPEED   600U    /* servo internal speed during replay */
static uint16_t teach_rec[TEACH_REC_MAX][6];   /* 临时(未存盘)录音 */
static uint32_t teach_rec_cnt = 0U;
static uint8_t  teach_rec_full = 0U;
/* ===== 示教录音完整性仪表 (2026-09-16) =====
 * 现场问题: "示教保存后回放, 中后段没被复刻出来, 但尾段是对的"。
 * 排查结论(代码级): 回放侧 teach_stream_tick() 每个节拍只把索引 +1,
 *   严格按数组顺序走, **没有任何跳样本的路径**; 而 TeachStore_Save() 写入后
 *   立刻对整段做 CRC 回读比对, Flash 里的数据与录音数组逐字节相同
 *   ⇒ 回放里看到的"缺一段"只可能是**录音数组里本来就缺**。
 * 录音为什么会缺: 节拍标志 traj_do_update 只有 1 bit (TIM2 50Hz 置位),
 *   主循环只要有一拍超过 20ms, 中间的节拍就被**静默合并**掉; 且录音拍里
 *   一次总线读失败(旧超时 100ms)会让 teach_record_tick() 直接 return,
 *   连这一拍样本一起丢。下面四个计数器把这两件事量化出来。 */
#define TEACH_READ_TO_MS  10U    /* 录音拍单轴读超时 (旧版 100ms -> 一次失败吞 5 拍) */
static volatile uint32_t teach_tick_lost = 0U;   /* 50Hz 节拍被主循环合并掉的次数 (TIM2 ISR 写) */
static uint32_t teach_smp_held    = 0U;   /* 单轴读失败 -> 沿用上一拍值 的轴次 */
static uint32_t teach_smp_dropped = 0U;   /* 首拍读失败 -> 整拍放弃 的次数 */
static uint16_t teach_read_fail[6] = {0}; /* 各轴读失败次数 (定位是哪个关节抖动) */
static void teach_report_integrity(void);  /* 定义在 teach_record_tick 之后 */
static uint8_t  teach_mode_active = 0U;
/* ===== 跟随录制 (2026-09-16): 遥控遥操下的"纯观察者"示教 =====
 * 1 = 本场示教**完全是只读的** —— 不写扭矩 / PID / PUNCH / 速度 / 运行模式,
 *     只按 50Hz 节拍把六轴 present 位置存进 teach_rec[]。
 * 用途: 主臂经 PC 中转驱动从臂时(`sp` → pc_relay.py → `#P` → 从臂 `sf 1`),
 *   从臂一边跟流一边录 —— 录下来的就是**从臂自己实际走出来的轨迹**
 *   (已过限位 clamp 与跳变守卫), 存盘后可脱离主臂单独 `play`。
 * 进入: `teach rec`。0 = 传统手拖示教(悬浮/全松/PWM, 见 cmdTeachEnter*)。 */
static uint8_t  teach_record_only = 0U;
static uint8_t  teach_play_state = TEACH_PLAY_NONE;
static uint8_t  teach_play_loop = 0U;
static uint32_t teach_play_idx = 0U;
/* 回放数据源: 临时录音(teach_rec)或已保存的 Flash 文件(teach_store) */
static const uint16_t (*teach_play_src)[6] = NULL;
static uint32_t teach_play_src_cnt = 0U;
static uint8_t  teach_play_from_file = 0U;
static char     teach_play_file_name[TSTORE_NAME_MAX + 1U] = {0};

/* ===== 悬浮示教(位置环导纳软跟随 / 零力控制) =====
 * 2026-09-08 B方案: 不切换舵机模式, 保持 Mode 0 位置伺服。
 * 进入 teach 时把 P/D/I 降到很低, GoalSpeed/Acceleration 提到很高,
 * 然后每 50Hz tick 让 GoalPosition 紧跟 PresentPosition (±死区)。
 * 位置误差≈0 → 舵机输出力矩≈0, 人手只对抗齿轮摩擦和惯性,
 * 就能拖动; 松手后 Goal 停在当前位置, 臂被托住不下坠。
 * 旧"扭矩全松"模式用 'teach free'; 旧 PWM 力控用 'teach pwm'。 */
#define TEACH_FOLLOW_SPEED     1023U   /* 悬浮期 GoalSpeed (最快跟随) */
#define TEACH_FOLLOW_ACCEL     120U    /* 悬浮期加速度 (默认 120, 高响应) */
#define TEACH_FOLLOW_P         2U      /* 悬浮期 P 增益 (越小越软, 0=无力) */
#define TEACH_FOLLOW_I         0U      /* 悬浮期 I 增益 (必须为 0, 否则积分项会托/坠) */
#define TEACH_FOLLOW_D         0U      /* 悬浮期 D 增益 */
#define TEACH_FOLLOW_DEADBAND  2U      /* 死区 raw: |Goal-Present|≤此值不更新 */
#define TEACH_FOLLOW_STEP_MAX  120U    /* Goal 每 tick 最大跟随步长(防读突跳) */
#define TEACH_FOLLOW_OFF_MAX   80U     /* 自动重力配重偏移上限 raw */
static uint8_t  teach_assist_active = 0U;    /* 本场是否悬浮模式 */
static uint16_t teach_assist_goal_raw[6] = {0}; /* virtual goal (紧跟 present) */
static uint8_t  teach_soft_p = TEACH_FOLLOW_P;
static uint8_t  teach_soft_i = TEACH_FOLLOW_I;
static uint8_t  teach_soft_d = TEACH_FOLLOW_D;
/* PUNCH (addr24, 启动最小输出功率): P=0 时仍可能输出 punch 脉冲力矩,
 * 悬浮期压到 0, 退出恢复。teach n <0~254> 可调。 */
static uint8_t  teach_soft_punch = 0U;
static uint8_t  teach_punch_backup[6] = {0};

/* 自动重力配重: 检测到某关节以低速持续漂移(重力下坠/过助上飘)时,
 * 缓慢偏移 virtual_goal, 直到舵机恰好托住。 */
static int16_t  teach_off_raw[6] = {0};
static uint8_t  teach_int_gain   = 1U;

/* 旧字段保留兼容: k/s 不再生效但命令不报错 */
static float    assist_k2 = 1.2f;
static float    assist_k3 = 1.6f;
static float    assist_k4 = 1.0f;
static uint16_t assist_scale_pct = 100U;

/* ===== PWM 模式真·重力补偿 (2026-09-08, 社区方案) =====
 * 依据: SO-101 Leader 臂靠换低减速比舵机实现"拖动轻"; STS3215 Mode 2 =
 * PWM 开环 = 直接输出力矩 (占空比写 addr44, ±1000 带符号)。
 * teach pwm: 6 轴切 Mode2, 每 tick 写 duty = 重力前馈(FK 水平杠杆, 带符号)
 * + 速度阻尼; 拖动只对抗摩擦。teach exit 切回 Mode0。
 *
 * 【关键发现 2026-09-08 17:50】PWM 驱动芯片在 duty≈0 附近存在死区(deadband):
 * 当 duty 落在死区内时，外力拖动产生自激振荡(疯狂上下甩动)。
 * 解决: 永远不让 duty 落在死区 —— |duty|<deadband 时推到 deadband
 * (保持方向), 或给一个极小的破静摩擦 dither。 */
#define PWM_DEADBAND_DEFAULT  20   /* 默认死区避开值 duty (~2% 占空比) */
static uint8_t  teach_pwm_active = 0U;
/* Per-joint PWM 零力参数: J2/J3/J4 承担主要重力矩, J1/J5/J6 默认无 FF。
 * 2026-09-09: damp 默认 J1/J5/J6=2 (小负载轴, damp 大则拖动阻力大=硬);
 * 新增 off (恒定偏置 duty, 补 J5/J6 偏心力矩——ff 对它们恒 0, 调 gain 无效)
 * 与 cap (per-joint duty 限幅, J3 原写死 ±40, 大负载姿态托不住=下坠)。 */
static int      teach_pwm_gain[6]   = {0, 6, 6, 6, 0, 0};  /* ×0.1 duty/mm (mass-weighted model) */
static int      teach_pwm_damp[6]   = {0, 3, 3, 3, 0, 0};   /* J1/J5/J6=0 完全零输出 */
static int      teach_pwm_flip[6]   = {1, 1, 1, 1, 1, 1};   /* 重力方向翻转 */
static int      teach_fc_duty[6]    = {0, 0, 0, 0, 0, 0};   /* 库伦摩擦补偿 */
static int      teach_dither[6]   = {0, 0, 0, 0, 0, 0};   /* 静摩擦 dither */
static int      teach_pwm_off[6]  = {0, 0, 0, 0, 0, 0};   /* 手动恒定偏置 duty (teach o) */
static int      teach_pwm_cap[6]  = {100, 100, 40, 100, 100, 100}; /* per-joint duty 限幅 */
static int      teach_test_joint  = 0;   /* 0=全轴输出; 1~6=仅该轴有输出(单轴标定) */
/* 2026-09-09 用户否决自动学习: 重力补偿只针对 J2/J3/J4 (J1/J5/J6 无重力矩,
 * 完全零输出), 稳态支撑靠手动 g(姿态主项)+o(微调) 逐轴标定。 */
static uint8_t  teach_dith_ph    = 0;
static int      teach_pwm_db     = PWM_DEADBAND_DEFAULT; /* 死区避开值 (全局) */
static uint16_t teach_assist_last_raw[6] = {0};

/* QL repeatability experiment statistics. Error is actual - target:
 * positive means the actual angle is greater than the commanded angle.
 * Each saved point keeps an independent six-joint error data set. */
static float    multi_point_err_sum[MULTI_POINT_MAX][6] = {{0}};
static float    multi_point_err_min[MULTI_POINT_MAX][6] = {{0}};
static float    multi_point_err_max[MULTI_POINT_MAX][6] = {{0}};
static uint32_t multi_point_err_samples[MULTI_POINT_MAX] = {0};

/* Multi-point nominal-position settling: before recording an "actual-reached
 * visit" we require the pose to stay within tolerance for a sustained window
 * (like cmdTrajectorySettleService). This prevents sampling while the arm is
 * still crawling in the approach phase (approach slow). */
static uint32_t multi_point_stable_since = 0U;

/* ===== Swing/wave state ===== */
static uint8_t  swing_active = 0;     /* 1 = swing mode active */
static uint8_t  swing_remaining = 0;  /* remaining half-swings */
static uint8_t  swing_phase = 0;     /* 0 = next target WAVE1, 1 = next target WAVE2 */
/* W 命令: 准备段(->WAVE1)走安全序列, 序列完成后由 safe_seq_service 置
 * swing_active=1 开始挥动。直接置 swing_active 会被准备段的 cmdMove 清零
 * (cmdMove 入口取消 swing), 挥动从未真正执行过 —— 2026-09-02 修复。 */
static volatile uint8_t swing_resume_pending = 0;

/* ===== Gravity compensation for J2 (shoulder) ===== */
/* When arm extends forward, gravity creates torque on J2 causing sag.
 * Layer 1: Higher P-Gain (32 vs 16) set in SERVO_InitJoint
 * Layer 2: Feedforward offset = -K * horizontal_reach (applied every interpolation step)
 * Layer 3: Post-move correction (read actual, compare target, correct if >2deg)
 * K = offset(deg) per mm of horizontal reach. 0 = disabled.
 * Typical: 0.02~0.05, tune with 'G' command. */
float GRAVITY_COMP_K = 0.0f;   /* 导出：arm_params AP_GC_K 绑定生效点 */

/* ===== Post-move adaptive gravity correction ===== */
/* Adaptive: gain auto-adjusts per joint each round.
 * - Overshoot (sign flip) -> halve gain, count oscillation
 * - Undershoot (same sign) -> increase gain 25%
 * - 2+ oscillations -> auto-raise D_Gain (damping)
 * - 2+ stalls -> auto-raise P_Gain (torque); 3+ -> add I_Gain
 * Goal: every corrected joint within threshold of target */
/* ⭐0924DA: 默认改为 0 (用户 2026-11-24 明令: 开机默认关自适应修正)。
 * 存量板 Flash 里若已存过 1, 敲 `A e 0` 后再敲 `F` 覆盖保存一次即可。 */
uint8_t  correction_enabled = 0;       /* 导出：arm_params AP_CORR_ENABLED 绑定 */
uint8_t  correction_pending = 0;       /* 导出：arm_control_api 状态快照用 */
static uint32_t correction_timer = 0;
static float    correction_target[6] = {180,180,180,180,180,180};
static float    correction_offset[6] = {0,0,0,0,0,0}; /* accumulated correction command */
static uint8_t  correction_count = 0;
/* Dynamic per-joint gain (adapts during correction, reset each sequence) */
static float    corr_gain[6] = {0.2f, 0.5f, 0.3f, 0.2f, 0.2f, 0.2f};
/* Initial gain (modifiable at runtime via A g J val)
 * 导出：arm_params AP_CORR_GAIN_J1~J6 绑定生效点 */
float    corr_gain_init[6] = {0.2f, 0.5f, 0.3f, 0.2f, 0.2f, 0.2f};
/* Oscillation / stall tracking */
static float    prev_error[6] = {0,0,0,0,0,0};
static uint8_t  oscillation_count[6] = {0,0,0,0,0,0};
static uint8_t  stall_count[6] = {0,0,0,0,0,0};
/* Error threshold (deg): all joints must reach within this (A t val)
 * 导出：arm_params AP_CORR_THRESHOLD 绑定 */
float    corr_threshold = 2.0f;
/* Max correction rounds before giving up (A r val)
 * 导出：arm_params AP_CORR_MAX_ROUNDS 绑定 */
uint8_t  corr_max_rounds = 10;
/* All six joints receive adaptive correction: J1~J6 */
#define CORR_FIRST_JOINT 0
#define CORR_LAST_JOINT  5

/* ===== Cached PID gains (synced with servo RAM) =====
 * ⭐0924DA: 默认值改为用户 G 表实测认可值 (2026-11-24):
 *   J1=12/0/12 J2=15/0/50 J3=13/0/48 J4=18/0/15 J5=18/0/13 J6=10/0/10 */
static uint8_t  pid_p[6] = {12, 15, 13, 18, 18, 10};
static uint8_t  pid_i[6] = {0, 0, 0, 0, 0, 0};
static uint8_t  pid_d[6] = {12, 50, 48, 15, 13, 10};

/* Predefined poses — 2026-09-02 触地事故后重构为"相对官方 Home 零点的关节角"(deg)。
 * 旧常量是舵机度 (默认 zero=180 时代设计), Phase1 校准后 zero 实测为
 * 692/1849/3221/2160/3081/2118, 旧值语义全部漂移 (如旧 POSE_HOME J3=30 舵机度
 * 在新 zero=283.2 下对应关节角 -253°, 越过死区撞挡块; 旧 WAVE 的 J5=120 违反
 * J5 卡区保护 180~340)。现改为关节角定义, 由 refresh_pose_servo() 换算成
 * servo = SERVO_OFFSET + JOINT_DIR*joint, 随 Phase1 校准自适应。
 * 2026-09-02 深夜 (P 0 二次卡死): J3 合法区 = servo 177~360~0~10 的包裹弧,
 * 关节角 [-106.2, +86.9]; HOME/FOLD 旧值 J3=-150/-160 (servo 133/123) 落
 * 在禁入区, 是 P 0 撞挡块的直接原因, 已改为 J3 留足挡块余量。
 * 2026-09-02 深夜置中后: 用户实机摆折叠位 f=(186,7,58), J3 已压到前挡块
 * (relhome +87.8 > 限位 +86.9), 收纳位不能贴挡块, 故 J3 收至 +80 (servo 260°,
 * 距挡块 6.9°), 爪尖≈(196,96) 即用户初摆位置; 其余 5 关节照抄实测。全部位姿
 * 经 Python FK 验证: raw 限位内, 爪尖 z 安全, J3/J4 轴心不进立柱圆柱。
 * 2026-09-10 (换 J2/J3 舵机重校准后): 以用户 f 实测折叠收纳位为准
 * {+0.35,-102.77,+91.16,+73.41,-5.27,-55.38} -> Python FK 复刻实机验证:
 * 爪尖 (131, 10) pitch 149.6, 与 f 打印完全一致 (+0.88/-102.86/+91.25/
 * +15.38 那组是 J3 前挡块读数, 不是收纳位, 已废弃)。
 * 2026-09-11 再次以用户 f 实测为准 (机械臂自然停靠位, 比 09-10 那组深 2.9°):
 *   servo 179.87/77.01/273.58/251.69/179.16/129.76 -> relhome
 *   {+0.18,-102.95,+93.54,+71.65,-0.88,-50.29}; Python FK 复刻: rho=131.4
 *   z=3.98 pitch=150.03, 与 f 打印 (131, 0, 3, 150) 完全一致。
 *   ⚠ 该姿态模型 z 只有 4.0mm, 而用户实测爪尖离桌面 10mm —— 模型整链偏保守
 *   ~6mm。故桌面下限不再写死, 改由 update_home_dock() 按本姿态推导。
 *   ⚠ J3 servo 273.58 = raw 3112, 超出 09-10 记录的挡块 raw 3086 有 26 raw
 *   (2.3°): 手推实测的挡块偏浅。当时的处置是把 J3 前挡块偏移放宽到 +1090
 *   (raw 3138) 把该姿态包进合法弧 —— 该内置偏移宏已于 2026-09-16 随"J3/J5 改用
 *   Phase 2 手推值"一并删除, 本段仅作历史记录 (收纳位无条件可达另由折叠位白名单保障)。
 *   J2 servo 77.01 -> raw 876 (下限 860, 余量 16); J6 -50.29 -> raw 1475
 *   (下限 1398, 余量 77)。J3/J4 轴心 rho=-85/48 均在立柱圆柱 (r=25) 之外。
 *
 * 2026-09-16 用户重新 f 实测停靠位 (自然停靠, 爪尖离桌面仍约 10mm):
 *   relhome {+1.05,-103.65,+90.29,+70.59,+2.64,-39.56}
 *   -> Python FK 复刻 (tools/pose_check.py): rho=141.6 z=19.5 pitch=145.0
 *   比 09-11 那组 J3 浅 3.25° (J2 浅 0.70°, J4 浅 1.06°) ⇒ 模型爪尖 z 抬高
 *   15.5mm ⇒ traj_z_floor_mm 随之由 0.98 抬到 16.5mm, 全局收紧 15.5mm:
 *   **贴近桌面的 g 目标 / a 安全序列可能开始被 "TCP z=.. < floor .." 拒掉**,
 *   这是跟随常量的预期行为, 不是新 bug。真要放宽, 改的是 POSE_HOME_J 或
 *   TRAJ_Z_MARGIN_MM, 不要去动检查代码。
 *   ⚠ 安全性与模型绝对精度无关: 下限锚在 FK(POSE_HOME).z - 3mm, 而路径检查
 *     用的也是同一套 FK ⇒ 模型偏置在比较中自动抵消, 判据的真实含义恒为
 *     "爪尖不低于停靠位真实高度 - 3mm"。模型 z 报 4mm 还是 19mm 都不改变这点。
 *   ⚠ 但**不要**拿 f 打印的 z 当桌面间隙读数: 09-11 记"模型 4mm vs 实测 10mm",
 *     09-16 记"模型 19.5mm vs 实测 10mm" —— 两次偏差符号相反, 至少一条目测
 *     不准。判据: 摆到停靠位敲 f, 看打印的 z 是 ~19 还是 ~10, 一次就能定。 */
static const float POSE_HOME_J[6]  = {   0.75f, -103.38f, 90.20f, 70.68f, 2.29f, -34.29f }; /* 折叠收纳(实测 f 2026-09-16, FK z=19.5) */
static const float POSE_FOLD_J[6]  = {   0,  40,  -70,  40,   0,   0 }; /* 备用收拢(servo J3=213) */
static const float POSE_READY_J[6] = {   0.97,  -49.05,  27.08,  44.31,  2.81,  -39.56 }; /* 前伸待命(FK 爪尖≈245,213; 2026-09-16 改, 原值爪尖≈340,282) */
static const float POSE_WAVE1_J[6] = {  -87.47, 14.15, -69.27, 26.20, 2.90, -39.56 }; /* 挥动位 A(FK z=423 rho=520) */
static const float POSE_WAVE2_J[6] = {  -89.90, -42.02, -70.86, 22.68, 2.55, -39.56 }; /* 挥动位 B(J1 反向, FK z=512) */
/* 运行时舵机度版本 (cmdPose/cmdMove 消费)。zero 变化后必须调 refresh_pose_servo() */
static float POSE_HOME[6], POSE_FOLD[6], POSE_READY[6], POSE_WAVE1[6], POSE_WAVE2[6];

/* ⭐0924CG: 折叠位(休息位)运行时标定 —— `P here` 把当前实际位姿存为
 * 新的 P 0 目标 (relhome 关节角, 与 POSE_HOME_J 同坐标系), 区F v3 持久化。
 * 上电 GeoCfg_Load3 恢复; 未标定 = 编译默认 POSE_HOME_J。
 * P 0 / 折叠位白名单 fk_is_home_dock / 桌面下限推导 全部自动跟随。 */
static float  pose_home_j_rt[6];
static uint8_t pose_home_cal = 0U;
/* 懒初始化标志 (file 作用域): 开机区F 恢复折叠位时置 1, 防止随后的
 * refresh_pose_servo() 懒初始化把标定值覆盖回编译默认 */
static uint8_t pose_home_rt_init = 0U;

/* 按当前 SERVO_OFFSET 把关节角位姿刷成舵机度。
 * 调用点: 开机 pid_load_startup 之后 (Flash 恢复 zero)、校准 's' 保存之后。 */
static void refresh_pose_servo(void) {
    /* 0924CG: HOME 位姿改走运行时数组 (P here 可覆盖); 首次调用前懒初始化 */
    if (!pose_home_rt_init) {
        memcpy(pose_home_j_rt, POSE_HOME_J, sizeof(pose_home_j_rt));
        pose_home_rt_init = 1U;
    }
    const float* src[5] = { pose_home_j_rt, POSE_FOLD_J, POSE_READY_J, POSE_WAVE1_J, POSE_WAVE2_J };
    float*       dst[5] = { POSE_HOME,   POSE_FOLD,   POSE_READY,   POSE_WAVE1,   POSE_WAVE2 };
    for (uint8_t p = 0; p < 5; p++) {
        for (uint8_t j = 0; j < 6; j++) {
            float s = SERVO_OFFSET[j] + JOINT_DIR[j] * src[p][j];
            s = fmodf(s, 360.0f);
            if (s < 0.0f) s += 360.0f;
            dst[p][j] = s;
        }
    }
}

/* ===== 折叠位白名单 (HOME dock, 2026-09-11) =====
 * 用户明确要求: 折叠收纳位是"贴死 J3 前挡块"的姿态, 必须**无条件可达** —— 不许
 * 因为该姿态略越挡块、或爪尖模型 z 偏低而被路径检查拒掉 (此前反复踩: 桌面阈值
 * 15 与 8 两版都让 P 0 起点/终点双向被拒 = 规划死锁, 只能 `a J` 单轴救援)。
 * 两道保障, 均由 update_home_dock() 在开机 / 校准保存后重算, 改 POSE_HOME_J
 * 即自动跟随:
 *   1) 桌面下限 traj_z_floor_mm = 折叠位模型爪尖 z − TRAJ_Z_MARGIN_MM;
 *   2) 路径检查豁免: 采样点 6 轴全在折叠位 ± HOME_DOCK_TOL_DEG 内时, 跳过
 *      raw 限位与桌面 z 检查 (覆盖停靠漂移、P 0 终点、从收纳位起手)。
 * 另打印折叠位 raw 与是否越限, 便于现场判断要不要重做手推标定 / 调折叠位白名单。
 *
 * 安全性: 豁免只覆盖"贴着折叠位的一小段"(±3°, J3 上约 ±34 raw; 由当前
 * POSE_HOME_J 实测偏移 = 爪尖 13.0mm, 见 tools/pose_check.py 输出;
 * 该姿态实测离桌面 10mm, 且深的一侧被挡块顶死、物理上到不了), 路径中段照旧
 * 逐点严格检查 —— 09-01 那种中段下探 (z=-127mm) 仍拦得住; 立柱圆柱检查不豁免。
 * 模型 z 与真实余量的关系 (2026-09-16 澄清, 取代旧的"模型偏保守 6mm"说法):
 *   下限锚在 FK(POSE_HOME).z - 3mm, 路径检查也走同一套 FK ⇒ 模型绝对偏置在
 *   比较中自动抵消, 判据的真实含义恒为"爪尖不低于停靠位真实高度 - 3mm"。
 *   所以下限的安全性**不需要**模型 z 准; 反过来说, 也**不能**用模型 z 去反推
 *   桌面间隙。09-16 换 POSE_HOME_J 后模型 z 由 4.0 抬到 19.5mm, 下限随之
 *   0.98 -> 16.5mm。 */
#define TRAJ_Z_MIN_MM      2.0f    /* 兜底下限 (FK 求解失败时用) */
#define TRAJ_Z_MARGIN_MM   2.0f    /* 折叠位模型 z 之下留的余量 */
#define HOME_DOCK_TOL_DEG  3.0f    /* 折叠位白名单半径 (deg, 六轴需同时满足) */
static float traj_z_floor_mm = TRAJ_Z_MIN_MM;
/* 0924AS: 用户 zf 偏好 ("爪尖最低离桌 mm")。<0 = 从未设过, 下限走
 * update_home_dock 的折叠位推导。设过之后优先级高于推导值,
 * 且随 C 偏移一起持久化到扇区 11 区 F (geo_cfg.c)。 */
static float geo_zfloor_mm = -1.0f;
/* ⭐0924BV: 流控/遥操桌面限位启用开关 (zf on|off, 区F 持久化)。
 * 背景: 0924BS 直通模式把"桌面 z 检查"一并旁路了 ⇒ zf 形同虚设。
 * 拆出后: 直通仍旁路跳变/立柱/包线, 但桌面下限受本开关独立控制,
 * 默认 ON (安全第一) —— 想完全直通再 `zf off`。 */
static uint8_t geo_zf_on = 1U;
/* ⭐0924BZ: 区F v2 的 margin 字段保留仅为记录布局兼容, 固定存 UNSET ——
 * 边界安全带已按用户明令删除, 遥操只受限位约束。 */
static uint16_t geo_margin_raw = 0xFFFFU;

/* 0~360 环上的最短角差 (处理 0/360 包裹) */
static float ang_delta_deg(float a, float b) {
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

/* 采样点是否落在折叠位白名单内 (6 轴同时满足) */
static uint8_t fk_is_home_dock(const float q[6]) {
    for (uint8_t j = 0; j < 6; j++) {
        if (fabsf(ang_delta_deg(q[j], POSE_HOME[j])) > HOME_DOCK_TOL_DEG)
            return 0U;
    }
    return 1U;
}

static uint8_t raw_in_range(uint16_t raw, uint16_t mn, uint16_t mx);  /* 前向声明 */
void cmdStreamExit(const char* why);   /* 定义在本文件稍后 (主从直写章节) */
extern uint16_t JOINT_MIN_RAW[6];   /* 定义在本文件稍后 (限位章节) */
extern uint16_t JOINT_MAX_RAW[6];

static void update_home_dock(void) {
    /* 1) 桌面下限 */
    float rad[6];
    Kinematics_Pos p;
    for (uint8_t j = 0; j < 6; j++) rad[j] = servo_deg_to_rad(POSE_HOME[j], j);
    if (!FK_Solve(rad, &p)) {
        cprint("  Home dock: FK failed; desk floor stays %.1f mm\r\n",
               (double)traj_z_floor_mm);
        return;
    }
    float z0 = p.z - WORLD_OFFSET_Z;                /* 基座坐标 */
    if (geo_zfloor_mm >= 0.0f) {
        /* 0924AS: 用户设过 zf —— 偏好优先, C/zr/存零点重锚都不再覆盖 */
        traj_z_floor_mm = geo_zfloor_mm - WORLD_OFFSET_Z;
    } else {
        traj_z_floor_mm = z0 - TRAJ_Z_MARGIN_MM;
    }

    /* 2) 诊断: 折叠位 raw 与越限状态 (越限由上面的白名单兜住, 不报错) */
    char raw_txt[64];
    int  n = 0;
    uint8_t over = 0U;
    for (uint8_t j = 0; j < 6; j++) {
        uint16_t hr = deg_to_servo_raw(POSE_HOME[j]);
        n += snprintf(raw_txt + n, sizeof(raw_txt) - (size_t)n, "%u ", (unsigned)hr);
        if ((JOINT_MIN_RAW[j] || JOINT_MAX_RAW[j]) &&
            !raw_in_range(hr, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) {
            over = 1U;
        }
    }
    cprint("  Home dock: raw[%s] floor %.1f mm (fold model z=%.1f)%s%s\r\n",
           raw_txt, (double)traj_z_floor_mm, (double)z0,
           (geo_zfloor_mm >= 0.0f) ? "  [zf pref]" : "",
           over ? "  [over limit -> whitelisted]" : "");
}

/* ===== Joint safety limits (adjustable at runtime via 'L' command) ===== */
/* 默认值按真实机械结构 + 安全余量设定:
 * 'c' Phase2 校准 + 's' 保存后, apply_calib_limits() 会用实测极值覆盖。
 * 注意: J5 永不让 IK 进入 77~174 卡区, 任何情况下都硬覆盖为 180~340。 */
/* ⚠ 并非"display only": ArmApi_MoveJoints / IK 查询用它做 deg 空间越界判定,
 * GET_LIMITS(0x05) 回的也是它。权威钳制源仍是下方 RAW 限位, 这里必须同步。 */
float JOINT_MIN[6] = {   65,  75,  76,  71,  12,   144 };  /* min angle (deg), 功能性限位 */
float JOINT_MAX[6] = { 300, 284, 271, 284, 351, 264 };  /* max angle (deg), 功能性限位 */

/* ===== Joint calibration raw limits (servo 0..4095) =====
 * Filled by 'c' calibration mode, persisted in Flash via PID_FlashSave.
 * These are the AUTHORITATIVE limits used for clamping and IK, because raw
 * 0-4095 is a continuous circular space; 0-360 deg is discontinuous at 0/360.
 * JOINT_MIN/MAX 同步自 RAW, 但同样是功能性限位(deg 空间检查用), 不是纯显示。 */
uint16_t JOINT_MIN_RAW[6] = { 0, 0, 0, 0, 0, 0 };
uint16_t JOINT_MAX_RAW[6] = { 0, 0, 0, 0, 0, 0 };

static const char *JOINT_NAMES[6] = {
    "shoulder_pan ", "shoulder_lift", "elbow_flex   ",
    "wrist_flex   ", "wrist_roll    ", "gripper      "
};
static uint16_t calib_min_raw[6] = {4095, 4095, 4095, 4095, 4095, 4095};
static uint16_t calib_pos_raw[6] = {0};
static uint16_t calib_max_raw[6] = {0};
static uint16_t calib_zero_raw[6] = {0, 0, 0, 0, 0, 0}; /* 官方 Home 姿态各关节 raw, 0=未校准 */
/* ⭐主臂零点 (2026-11-04 方案1板内遥操): 主臂是 STS3215 虚拟舵机(只编码器无电机),
 * 不驱臂故**不存 flash/不释力矩/无限位**; 唯一用途 = `fi` 反解主臂角度时的零点基准。
 * 校 `c` Phase1 里按 m 切到主臂对象记录, fi 用它替代 SERVO_OFFSET 反解。 */
static uint16_t master_zero_raw[6] = {0, 0, 0, 0, 0, 0};
static uint8_t  master_zero_valid = 0U;   /* 0924BI: 6 轴零点全部非 0 即认为已校准(供状态显示) */
static volatile uint8_t calib_active = 0;   /* set by cmdCalibrate, cleared on exit */
static volatile uint8_t calib_selected = 0; /* currently focused joint (0..5) */

/* 前向声明: 定义在 pid_load_startup 之前, cmdCalibrate 保存后要调用 */
static void apply_calib_limits(void);

/* Try to read one raw byte from the console ring buffer (non-blocking).
 * Returns 1 if a byte was placed into *out, 0 if no byte available. */
static int calib_try_byte(uint8_t *out) {
    if (rx_head == rx_tail) return 0;
    *out = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUF_SIZE);
    return 1;
}

/* STS3215 中位定义:官方定义为 2048 (11位分辨率 0-4095,中心刻度)。
 * 实际驱动器/调试软件读出可能是 2047 或 2048 (整除向下取整),两者等价。
 * 用户可调整此常数以匹配 ESP32/上位机工具的中位校准值。 */
#define CALIB_MID_VALUE   2048U

/* Phase 0 = 设中位 (set POS to MID_VALUE at the user's chosen pose)
 * Phase 1 = 设限位 (穷极 MAX/MIN by rotating joint through range) */
static volatile uint8_t calib_phase = 0;

/* ⭐0924BH 校准对象 (方案1板内遥操): 0=从臂(默认,存flash)  1=主臂(虚拟舵机只记零点)
 * 主臂虚拟舵机无电机, 校准它不释力矩/无Phase2限位, 只记 master_zero_raw 供 fi 反解。
 * ⭐0924BI: 主臂零点现在也存 flash (区 H), 重启自动恢复, 不用再每次重标。
 * 'c' 校准向导里按 m 键切换, 按 d 键切双臂对照(实时显示两边raw差)。 */
static volatile uint8_t calib_target = 0;   /* 0=slave  1=master */

/* 读主臂 J(0..5) 的 raw (Bus A ID 7+j); 失败返回 0xFFFF。
 * 与 fi_read_master_deg 同一条 DXL 读路径, 但返回 raw 供校准对照。 */
static uint16_t calib_read_master_raw(uint8_t j) {
    uint8_t data[2] = {0, 0};
    uint8_t id = (uint8_t)(7U + j);
    if (DXL_ReadDiag(BUS_A, id, STS_ADDR_PRESENT_POSITION, 2, data,
                     10U, NULL, NULL) != HAL_OK) return 0xFFFFU;
    return (uint16_t)(data[0] | (data[1] << 8));
}
static uint8_t calib_phase2_seen = 0;   /* 是否真正进入过 Phase 2 (否则保存限位会污染 Flash) */

/* Render the joint-calibration table.
 * NOTE: cursor row indicated by '>'. PHASE changes column meaning:
 *   phase 0 (set-home-zero): 摆官方 Home 姿态, 按 0 记录各关节 raw 为运动学零点
 *   phase 1 (set-limits):  MIN/MAX auto-expand, POS shows current */
static void calib_render(int highlight_row) {
    if (calib_phase == 0) {
        /* 0924BH: 标注当前校准对象 (从臂存flash / 主臂只记零点供fi) */
        cprint("\r\n  --- PHASE 1: SET HOME ZERO [%s] ---\r\n",
               calib_target == 1 ? "主臂(只记零点,不存flash)" : "从臂(存flash)");
        cprint("  把整臂摆成官方 Home (上臂竖直/前臂水平/工具朝前), 按 0 全记录.\r\n");
        /* 双臂对照: 同时显示主臂/从臂 raw 与差值, 方便判断两臂是否同一水平 */
        if (calib_target == 2) {
            cprint("  +-----+----------------+--------+--------+--------+\r\n");
            cprint("  |     | NAME           | 从臂POS| 主臂POS| Δ差值  |\r\n");
            cprint("  +-----+----------------+--------+--------+--------+\r\n");
            for (uint8_t j = 0; j < 6; j++) {
                uint16_t cs = SERVO_GetPositionRaw(j);
                uint16_t cm = calib_read_master_raw(j);
                if (cs == 0xFFFFU) cs = 0;
                if (cm == 0xFFFFU) cm = 0;
                int diff = (int)cm - (int)cs;
                cprint("  | %s J%d | %s | %6d | %6d | %+6d |\r\n",
                       (highlight_row >= 0 && j == (uint8_t)highlight_row) ? ">" : " ",
                       j + 1, JOINT_NAMES[j], cs, cm, diff);
            }
            cprint("  +-----+----------------+--------+--------+--------+\r\n");
            cprint("  Keys: m=切主臂/从臂  d=对照模式  s=保存(仅从臂)  q=quit\r\n");
            return;
        }
        cprint("  +-----+----------------+------+------+\r\n");
        cprint("  |     | NAME           | POS  | ZERO |\r\n");
        cprint("  +-----+----------------+------+------+\r\n");
        for (uint8_t j = 0; j < 6; j++) {
            /* 主臂对象: 读主臂 raw 显示; 从臂对象: 读从臂 raw */
            uint16_t cur = (calib_target == 1) ? calib_read_master_raw(j)
                                               : SERVO_GetPositionRaw(j);
            if (cur == 0xFFFFU) cur = 0;
            uint16_t zr = (calib_target == 1) ? master_zero_raw[j]
                                              : calib_zero_raw[j];
            cprint("  | %s J%d | %s | %4d | %4d |\r\n",
                   (highlight_row >= 0 && j == (uint8_t)highlight_row) ? ">" : " ",
                   j + 1, JOINT_NAMES[j], cur, zr);
        }
        cprint("  +-----+----------------+------+------+\r\n");
        cprint("  Keys: 0=记录全部零点  1-6=单关节  m=切主臂  d=对照  x/X=next(限位)  s=save  q=quit\r\n");
    } else {
        cprint("\r\n  --- PHASE 2: SET LIMITS (auto track MIN/MAX) ---\r\n");
        cprint("  Rotate each joint through full range. MIN/MAX auto-expand.\r\n");
        cprint("  +-----+----------------+------+------+------+\r\n");
        cprint("  |     | NAME           | MIN  | POS  | MAX  |\r\n");
        cprint("  +-----+----------------+------+------+------+\r\n");
        for (uint8_t j = 0; j < 6; j++) {
            uint16_t cur = SERVO_GetPositionRaw(j);
            if (cur == 0xFFFFU) cur = 0;
            cprint("  | %s J%d | %s | %4d | %4d | %4d |\r\n",
                   (highlight_row >= 0 && j == (uint8_t)highlight_row) ? ">" : " ",
                   j + 1, JOINT_NAMES[j],
                   calib_min_raw[j], cur, calib_max_raw[j]);
        }
        cprint("  +-----+----------------+------+------+------+\r\n");
        cprint("  Keys: 1-6 select  r=reset J%d  R=reset all  p/P=prev phase  s=save  q=quit\r\n",
               calib_selected + 1);
    }
}

/* K: 一键置中校准 (官网: 40 地址写 128 => 当前位置设为中位 2048)。
 * 适用于官方 L 形 Home 姿态下对全部/单个舵机执行 —— 置中后 L 形 raw≈2048,
 * Phase1 校准记录值整齐, SERVO_OFFSET≈180°。
 * 注意: 写入即时永久改变该舵机编码基准! 执行后必须重新完成 Phase1 (+2) 校准
 * 保存, 否则固件 SERVO_OFFSET 与舵机实际基准不一致。J3/J5 保护为"相对 zero
 * 的 raw 偏移"写法, 置中后自动适配, 无需改宏。 */
static void cmdCenterCalib(uint8_t all, uint8_t jsel) {
    uint8_t lo = all ? 0U : jsel, hi = all ? 5U : jsel;
    for (uint8_t j = lo; j <= hi; j++) {
        uint8_t bus = JOINT_BUS[j];
        uint8_t id  = JOINT_ID[j];
        SERVO_SetTorque(j, 0);               /* 释放扭矩, 自由状态置中 */
        HAL_Delay(20);
        cprint("  J%d (bus%d id%d): writing 40=128... ", j + 1, bus, id);
        uint8_t d[1];
        d[0] = 1; SERVO_WriteSimple(bus, id, 55, d, 1);   /* EEPROM 解锁 */
        HAL_Delay(10);
        d[0] = 128; SERVO_WriteSimple(bus, id, 40, d, 1); /* 一键置中 */
        HAL_Delay(50);
        d[0] = 1;  SERVO_WriteSimple(bus, id, 55, d, 1);  /* 锁定 */
        HAL_Delay(10);
        uint16_t pos = SERVO_GetPositionRaw(j);
        uint8_t ok = (pos != 0xFFFFU && pos >= 2035U && pos <= 2061U);
        cprint("pos=%u %s\r\n", pos, ok ? "OK (≈2048)" : "!unexpected");
        if (!ok) return;                     /* 异常即停, 防止继续错位 */
    }
    if (all) cprint("  All 6 centered. WARNING: re-run 'c' Phase1+2 and save now!\r\n");
}

/* cmdCalibrate: enter interactive two-phase joint calibration.
 * Phase 1: lock the physical middle — servo treats current raw as CALIB_MID_VALUE.
 * Phase 2: rotate each joint to extremes — MIN/MAX auto-track.
 * 's' saves to flash. 'q' exits. */
static void cmdCalibrate(void) {
    calib_target = 0;   /* 0924BH: 默认校准从臂(存flash); m切主臂 d切对照 */
    uint8_t slave_torque_released = 0;  /* 本次是否释放过从臂扭矩(退出时要恢复) */
    cprint("\r\n  === Joint Calibration (2-phase) [%s] ===\r\n",
           calib_target == 1 ? "主臂" : "从臂");
    cprint("  Phase 1: 摆官方 Home 姿态 (L形), 按 0 记录各关节零点.\r\n");
    cprint("  Phase 2 (可选): 按 x 进入后转动各关节到极限, 自动跟踪 MIN/MAX 限位.\r\n");
    cprint("  Press 's' after phase 1 (or 2) to save; 'q' to quit.\r\n");
    cprint("  m=切主臂/从臂  d=双臂对照(看是否同一水平)  (主臂=虚拟舵机只记零点)\r\n");

    /* 校准需手动摆位: 释放全部扭矩。⭐0924BH: 主臂是虚拟舵机(无电机)不需释力矩;
     * 对照模式(d)也不释(只读两边); 仅当校准从臂对象时释放从臂。 */
    if (calib_target == 0) {
        for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
        slave_torque_released = 1;
        cprint("  Slave torques released for hand positioning.\r\n");
    }

    calib_phase = 0;
    calib_phase2_seen = 0;

    /* Phase 1 init: nothing tracked; POS column reads live position. */
    /* Phase 2 init: reset trackers so live pos can shrink/grow them. */
    for (uint8_t j = 0; j < 6; j++) {
        if (calib_min_raw[j] == 0 || calib_min_raw[j] >= calib_max_raw[j]) {
            calib_min_raw[j] = 4095;
            calib_max_raw[j] = 0;
        }
    }

    calib_active = 1;
    calib_render((int)calib_selected);

    uint32_t last_paint = HAL_GetTick();
    for (;;) {
        uint8_t ch;
        if (calib_try_byte(&ch)) {
            /* ⭐0924BH: m=切主臂/从臂  d=双臂对照。任何阶段都能切。 */
            if (ch == 'm' || ch == 'M') {
                /* 对照(2) -> 从臂(0) -> 主臂(1) -> 对照(2) 循环 */
                calib_target = (calib_target == 2) ? 0 : (calib_target + 1);
                /* 切到从臂且非对照时才需释力矩; 主臂无电机/对照只读 都不释 */
                if (calib_target == 0) {
                    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
                    slave_torque_released = 1;
                    cprint("\r\n  [target] 从臂 (存flash; 扭矩已释放)\r\n");
                } else if (calib_target == 1) {
                    cprint("\r\n  [target] 主臂 (虚拟舵机只记零点master_zero_raw,不存flash)\r\n");
                } else {
                    cprint("\r\n  [target] 双臂对照 (实时显示两边raw差值,看是否同一水平)\r\n");
                }
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
            } else if (ch == 'd' || ch == 'D') {
                calib_target = (calib_target == 2) ? 0 : 2;   /* d 在对照/从臂间切 */
                cprint("\r\n  [target] %s\r\n",
                       calib_target == 2 ? "双臂对照 (Δ=主臂-从臂 raw)"
                                         : "从臂 (存flash; 扭矩已释放)");
                if (calib_target == 0) {
                    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
                    slave_torque_released = 1;
                }
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
            } else if (ch == '0') {
                if (calib_target == 1) {
                    /* 主臂对象: 记 master_zero_raw (不存flash, 供fi反解) */
                    uint8_t ok = 1;
                    for (uint8_t j = 0; j < 6; j++) {
                        uint16_t cur = calib_read_master_raw(j);
                        if (cur == 0xFFFFU) { ok = 0; cprint("\r\n  [M-ZERO] J%d read FAILED\r\n", j+1); }
                        else master_zero_raw[j] = cur;
                    }
                    if (ok) {
                        master_zero_valid = 1U;
                        cprint("\r\n  [M-ZERO] 主臂零点已记录: %d %d %d %d %d %d (fi反解用;按 s 存flash)\r\n",
                               master_zero_raw[0], master_zero_raw[1], master_zero_raw[2],
                               master_zero_raw[3], master_zero_raw[4], master_zero_raw[5]);
                    }
                    calib_render((int)calib_selected);
                    last_paint = HAL_GetTick();
                } else if (calib_target == 0) {
                /* Phase 1: 一键记录全部关节的官方 Home 零点 raw */
                uint8_t ok = 1;
                for (uint8_t j = 0; j < 6; j++) {
                    uint16_t cur = SERVO_GetPositionRaw(j);
                    if (cur == 0xFFFFU) { ok = 0; cprint("\r\n  [ZERO] J%d read FAILED (offline?)\r\n", j + 1); }
                    else calib_zero_raw[j] = cur;
                }
                if (ok) {
                    cprint("\r\n  [ZERO] Home 零点已记录: %d %d %d %d %d %d\r\n",
                           calib_zero_raw[0], calib_zero_raw[1], calib_zero_raw[2],
                           calib_zero_raw[3], calib_zero_raw[4], calib_zero_raw[5]);
                    cprint("  ...按 s 保存生效 (无需等 Phase 2).\r\n");
                }
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
                } else {
                    cprint("\r\n  对照模式只读, 按 m 或 d 切到从臂/主臂再记零点.\r\n");
                }
            } else if (ch >= '1' && ch <= '6') {
                calib_selected = (uint8_t)(ch - '1');
                if (calib_target == 1) {
                    /* 主臂对象: 记 master_zero_raw 单关节 (不存flash) */
                    uint16_t cur = calib_read_master_raw(calib_selected);
                    if (cur == 0xFFFFU) {
                        cprint("\r\n  [M-ZERO] J%d read FAILED\r\n", calib_selected + 1);
                    } else {
                        master_zero_raw[calib_selected] = cur;
                        master_zero_valid = 1U;
                        cprint("\r\n  [M-ZERO] J%d master zero=%d (fi反解用;按 s 存flash)\r\n",
                               calib_selected + 1, cur);
                    }
                    calib_render((int)calib_selected);
                    last_paint = HAL_GetTick();
                } else if (calib_target == 0) {
                if (calib_phase == 0) {
                    /* Phase 1: 把当前 raw 记录为该关节的 Home 零点 (单关节微调) */
                    uint16_t cur = SERVO_GetPositionRaw(calib_selected);
                    if (cur == 0xFFFFU) {
                        cprint("\r\n  [ZERO] J%d read FAILED (offline?)\r\n", calib_selected + 1);
                    } else {
                        calib_zero_raw[calib_selected] = cur;
                        calib_pos_raw[calib_selected] = cur;
                        cprint("\r\n  [ZERO] J%d zero=%d (官方 Home 需 6 关节都记录)\r\n",
                               calib_selected + 1, cur);
                    }
                } else {
                    cprint("\r\n  [select] J%d\r\n", calib_selected + 1);
                }
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
                } else {
                    cprint("\r\n  对照模式只读, 按 m 或 d 切到从臂/主臂再选关节.\r\n");
                }
            } else if (ch == 'x' || ch == 'X') {
                if (calib_phase == 0) {
                    calib_phase = 1;
                    calib_phase2_seen = 1;   /* 进入 Phase 2 才算采集过真实限位 */
                    cprint("\r\n  [phase] -> PHASE 2: SET LIMITS (auto track MIN/MAX)\r\n");
                    calib_render((int)calib_selected);
                }
                last_paint = HAL_GetTick();
            } else if (ch == 'p' || ch == 'P') {
                if (calib_phase == 1) {
                    calib_phase = 0;
                    cprint("\r\n  [phase] -> PHASE 1: SET HOME ZERO\r\n");
                    calib_render((int)calib_selected);
                }
                last_paint = HAL_GetTick();
            } else if (ch == 'r' && calib_phase == 1) {
                calib_min_raw[calib_selected] = 4095;
                calib_max_raw[calib_selected] = 0;
                cprint("\r\n  [reset] J%d limits cleared.\r\n", calib_selected + 1);
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
            } else if (ch == 'R' && calib_phase == 1) {
                for (uint8_t j = 0; j < 6; j++) {
                    calib_min_raw[j] = 4095;
                    calib_max_raw[j] = 0;
                }
                cprint("\r\n  [reset] All joints' limits cleared.\r\n");
                calib_render((int)calib_selected);
                last_paint = HAL_GetTick();
            } else if (ch == 's' || ch == 'S') {
                /* ⭐0924BI: 主臂零点现在也存 flash (区 H, 跨复位保留, fi 上电自动恢复)。
                 * 从臂走 PID_FlashSave; 对照模式只读。 */
                if (calib_target == 1) {
                    uint8_t okm = 1U;
                    for (uint8_t j = 0; j < 6; j++) {
                        if (master_zero_raw[j] == 0U || master_zero_raw[j] == 0xFFFFU) {
                            okm = 0U;
                            cprint("\r\n  [save] J%d 主臂零点未记录 (先按 0 记录全部零点)\r\n", j + 1);
                        }
                    }
                    if (okm) {
                        if (MstZeroCfg_Save(master_zero_raw) == CFG_FLASH_OK) {
                            master_zero_valid = 1U;
                            cprint("\r\n  [save] ✔ 主臂零点已存 flash (区H,重启保留;fi 遥操自动用)\r\n");
                        } else {
                            cprint("\r\n  [save] ✗ 主臂零点写 flash 失败\r\n");
                        }
                    }
                    last_paint = HAL_GetTick();
                    continue;
                }
                if (calib_target == 2) {
                    cprint("\r\n  [save] 对照模式只读。切回从臂(m/d)再保存。\r\n");
                    last_paint = HAL_GetTick();
                    continue;
                }
                /* Persist via PID_FlashSave (keeps PID/correction/speed). */
                uint8_t corr_pct[6];
                for (uint8_t j = 0; j < 6; j++) {
                    int pct = (int)(corr_gain_init[j] * 100.0f + 0.5f);
                    corr_pct[j] = (uint8_t)(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
                }
                float params[PID_FLASH_ARM_PARAM_MAX];
                uint8_t param_count = ArmParams_Snapshot(params, PID_FLASH_ARM_PARAM_MAX);
                PID_JointLimits_t limits;
                for (uint8_t j = 0; j < 6; j++) {
                    if (calib_phase2_seen) {
                        limits.min[j] = calib_min_raw[j];
                        limits.max[j] = calib_max_raw[j];
                    } else {
                        /* 没做 Phase 2: 存哨兵, 重启时 apply_calib_limits 跳过,
                         * 保持编译默认 (官方 URDF 行程限位), 避免旧的假极值死锁 IK */
                        limits.min[j] = 4095;
                        limits.max[j] = 0;
                    }
                    limits.pos[j] = calib_pos_raw[j];
                    limits.zero[j] = calib_zero_raw[j];
                }
                uint8_t st = PID_FlashSave(pid_p, pid_i, pid_d, corr_pct,
                                           (uint16_t)(corr_threshold * 100.0f + 0.5f),
                                           corr_max_rounds, correction_enabled,
                                           all_servo_moving_speed, params, param_count,
                                           &limits);
                if (st == PID_FLASH_OK) {
                    cprint("\r\n  [save] OK. PID+corr+speed+limits+zero written to Flash.\r\n");
                    /* 顺序必须: 先重锚 Home 零点(改 SERVO_OFFSET) 再算限位, 见 pid_load_startup */
                    Kinematics_SetZeroRaw(calib_zero_raw);  /* Home 零点立即生效 */
                    if (calib_phase2_seen) {
                        apply_calib_limits();          /* 限位立即生效(按新零位) */
                    } else {
                        /* 未采集 Phase2: 保持编译默认 (官方 URDF 行程限位)。
                         * 不要改 JOINT_MIN/MAX —— 覆盖成 0~360 会让 IK 解出机械上
                         * 不可能的角 (曾把 J2 顶死). Flash 已存哨兵, 重启后同样
                         * 保持官方行程默认. */
                        apply_calib_limits();          /* 让 J3/J5 保护带随新 zero 平移 */
                        cprint("  No Phase-2 captured: joint limits = official URDF ranges.\r\n");
                    }
                    refresh_pose_servo();                  /* 预设位姿随新 zero 重算 */
                    update_home_dock();                    /* 折叠位白名单/下限随 zero 重算 */
                    cprint("  Home zero applied: FK/IK 现在以官方 Home 为 0°.\r\n");
                } else {
                    cprint("\r\n  [save] FAILED.\r\n");
                }
                last_paint = HAL_GetTick();
            } else if (ch == 'q' || ch == 'Q') {
                cprint("\r\n  Exiting calibration mode.\r\n");
                break;
            } else if (ch == 'h' || ch == 'H' || ch == '?') {
                if (calib_phase == 0) {
                    cprint("\r\n  Phase 1: 0=记全部零点  1-6=单关节  m=切主臂/从臂  d=双臂对照  x=next(限位)  s=save(仅从臂)  q=quit\r\n");
                    cprint("  [当前: %s]\r\n",
                           calib_target == 1 ? "主臂(只记零点master_zero_raw,不存flash)"
                            : calib_target == 2 ? "双臂对照(实时看两边raw差)"
                            : "从臂(存flash)");
                } else {
                    cprint("\r\n  Phase 2: 1-6 select  r=reset J  R=reset all  p=prev  s=save(仅从臂)  q=quit\r\n");
                }
            } else {
                cprint("\r\n  Unknown key '%c'. Press 'h' for help.\r\n", ch);
                last_paint = HAL_GetTick();
            }
        }

        /* Periodic poll (200 ms) — auto-track MIN/MAX only in Phase 2.
         * ⭐0924BH: 仅"从臂"对象做 MIN/MAX 跟踪; 主臂无限位(虚拟舵机不驱臂),
         * 对照模式只读不跟踪。 */
        if ((HAL_GetTick() - last_paint) >= 200U) {
            if (calib_phase == 1 && calib_target == 0) {
                int expanded = 0;
                for (uint8_t j = 0; j < 6; j++) {
                    uint16_t cur = SERVO_GetPositionRaw(j);
                    if (cur != 0xFFFFU) {
                        calib_pos_raw[j] = cur;
                        if (cur < calib_min_raw[j]) { calib_min_raw[j] = cur; expanded = 1; }
                        if (cur > calib_max_raw[j]) { calib_max_raw[j] = cur; expanded = 1; }
                    }
                }
                if (expanded) calib_render((int)calib_selected);
            } else if (calib_target == 2) {
                /* 对照模式: 周期重绘, 实时刷新两边 raw 差值 */
                calib_render((int)calib_selected);
            }
            last_paint = HAL_GetTick();
        }
    }

    calib_active = 0;

    /* 校准结束恢复扭矩保持当前位置, 防止臂失力垂落砸桌。
     * ⭐0924BH: 用 slave_torque_released 标志 —— 只要本次进过从臂对象(含中途
     * 切走过)就恢复; 主臂对象全程没碰从臂扭矩则不重复 SetTorque(1)。 */
    if (slave_torque_released) {
        for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1);
        cprint("  Torque re-enabled (holding current pose).\r\n");
    }
}

/* ===== Raw (0-4095) wrap-around helpers for joint limits =====
 * These work in the continuous circular raw space, so ranges that cross
 * 0/360 (e.g. min=350 deg, max=10 deg) are handled correctly. */

/* Forward distance from 'from' to 'to' along increasing raw, modulo 4096. */
/* ⭐0924BZ: 边界安全带 (0924BF~BS 的 stream_clamp_margin / `fi margin`) 已整体
 * 移除 —— 用户明令 "遥操只受限位约束"。历史教训仍在: Phase2 手推极值含齿隙,
 * 目标钳在边界raw上会位置环饱和顶死 (J3 冒烟) —— 靠 [STALL] 告警 + 用户自己
 * 控制输入速度兜底, 限位值本身由 Phase2 校准/`L` 命令管理。 */

static inline uint16_t raw_dist_fwd(uint16_t from, uint16_t to) {
    return (uint16_t)((to + 4096u - from) % 4096u);
}

/* Return 1 if raw is inside the calibrated [mn, mx] range (forward arc). */
static uint8_t raw_in_range(uint16_t raw, uint16_t mn, uint16_t mx) {
    uint16_t span = raw_dist_fwd(mn, mx);
    if (span == 0) return 0;
    return raw_dist_fwd(mn, raw) <= span;
}

/* Clamp a raw value to the nearest boundary of [mn, mx] (circular nearest).
 * 2026-09-02 修正: 旧的 d<=span/2 判据对跨 0/360 的包裹弧会选错侧
 * (如 J3 禁入点 133° 会被夹到 10° 而非更近的 177°)。
 * 2026-11-02 再修正 (⭐遥操 J3 反向突变根因): 09-02 版只比**正向**距离 ——
 * 对不跨 0 的普通弧, 目标只要**超出 max 一丁点**, 到 max 的正向距离要绕
 * 整圈(~4090), 到 min 反而只有半圈 ⇒ 被夹到**对面的 min**!
 *   现场: 主臂 J3 顶死限位挤出 0.4° 超限 (271.4°) → 从臂目标被钳到 76°
 *   (对面挡块), 表现为"反向突变打满全程 195°"。0924N 旧案"越限轴自己
 *   大幅跑 194°"同源; J5 (180~340) 目标 341° 同样会被打到 180°。
 * 改为双向圆周最短距离 min(fwd, rev): 越界点永远夹到圆周上更近的边界。
 * 对 09-02 关心的"禁入区中点选近侧"场景结果不变 (该点到两侧边界必有一侧
 * 正向更近, 双向化不会翻转)。 */
static uint16_t clamp_raw(uint16_t raw, uint16_t mn, uint16_t mx) {
    if (raw_in_range(raw, mn, mx)) return raw;
    uint16_t dm  = raw_dist_fwd(raw, mn), dm_r = raw_dist_fwd(mn, raw);
    uint16_t dx  = raw_dist_fwd(raw, mx), dx_r = raw_dist_fwd(mx, raw);
    uint16_t dmc = (dm <= dm_r) ? dm : dm_r;
    uint16_t dxc = (dx <= dx_r) ? dx : dx_r;
    return (dmc <= dxc) ? mn : mx;
}

/* Clamp angle to joint's safe range using raw limits (handles wrap-around).
 * Returns clamped value, prints warning if clamped. */
static float clamp_angle(uint8_t joint, float deg) {
    if (joint >= 6) return deg;
    uint16_t mn = JOINT_MIN_RAW[joint];
    uint16_t mx = JOINT_MAX_RAW[joint];
    if (mn == 0 && mx == 0) return deg;        /* uncalibrated: fall back to no clamp */
    uint16_t raw = deg_to_servo_raw(deg);
    if (raw_in_range(raw, mn, mx)) return deg;
    uint16_t new_raw = clamp_raw(raw, mn, mx);
    float new_deg = servo_raw_to_deg(new_raw);
    cprint("  J%d: %d -> %d deg (raw %d -> %d limit)\r\n",
           joint+1, (int)deg, (int)new_deg, (int)raw, (int)new_raw);
    return new_deg;
}

/* Check 6-joint array, clamp all using raw limits, return number of clamped joints */
static uint8_t clamp_all_angles(float target[6]) {
    uint8_t clamped = 0;
    for (uint8_t j = 0; j < 6; j++) {
        uint16_t mn = JOINT_MIN_RAW[j];
        uint16_t mx = JOINT_MAX_RAW[j];
        if (mn == 0 && mx == 0) continue;    /* uncalibrated */
        uint16_t raw = deg_to_servo_raw(target[j]);
        if (!raw_in_range(raw, mn, mx)) {
            uint16_t new_raw = clamp_raw(raw, mn, mx);
            target[j] = servo_raw_to_deg(new_raw);
            clamped++;
        }
    }
    return clamped;
}

/* 注: 旧 J2 重力前馈 (gravity_comp_j2 + gc.k) 已整体移除 ——
 * 重力补偿只以"悬浮示教 (Torque Limit 托举)"形态存在于 teach 流程,
 * 不参与任何轨迹/QL/QLC/B/回放等运动算法。gc.k 参数保留为占位(见 'G')。 */

/* ===== Persistent PID configuration in STM32 internal Flash ===== */
static void set_pid_gain(uint8_t joint, uint16_t addr, uint8_t value);
/* 校准限位 -> 真正生效: 把 Phase 2 自动跟踪到的 raw MIN/MAX 直接保存为
 * JOINT_MIN_RAW/MAX_RAW, 并在 raw 空间做环绕判断; 角度版 JOINT_MIN/MAX
 * 仅用于显示。
 * 仅当数据合法时应用: 未校准/已 reset 时是哨兵值 (min=4095, max=0, 即 min>=max),
 * 行程过短(<20 raw ~1.8°) 视为噪声, 都跳过以免把限位搞成全开或反转。
 * J5 (wrist_roll) 例外: 该臂 J5 在 77°~174° 存在机械卡点 (驱动即堵转红灯),
 * 校准极值 (手转 14~357) 含卡区, 故 J5 强制锁安全区 180~340 (Home 271 两侧),
 * 任何情况下都覆盖校准数据与 L 命令, 永不让 IK 指挥 J5 进入卡区。
 * 2026-09-02 修正: J3 不再强制 >=90, 使用实测机械限位 10~177; 算法负责让
 * 所有规划路点和 IK 解都落在该范围内, 而不是人为收紧限位。 */
/* 2026-09-02 深夜最终修正 (P 0 第二次卡死复盘):
 *   限位定义与检查一律使用"行程量"(raw 0-4095), 严禁退回角度/关节角推导!
 *   J3 的真实结构 (用户实测): 挡块在 伺服 10 和 177, 合法运动区 =
 *   servo [177 -> 360 -> 0 -> 10] 这条跨越 0/360 的包裹弧 (正向 span 193 deg)。
 *   伺服 (10,177) 之间是不可达禁入区 —— 用角度表达必然跨 0 重复、辨不了方向,
 *   这正是 P 0 目标 J3=133° 混进禁入区、运动途中撞后挡块 (175°) 卡死的根因。
 *   J3 限位直接写死挡块行程量 (不随 Phase1 zero / Phase2 极值变化,
 *   Phase2 极值 23~4083 是整圈绕行途中的记录, 不是挡块位置, 不能用于 J3)。
 *   J2/J4 同样只用 Phase2 实测行程量; J1/J6 全行程; J5 固定 180~340。 */
/* J3/J5 保护限位一律用"相对本关节 zero 的 RAW 偏移" (raw 差, 仍是行程量):
 *   2026-09-10 换 J2/J3 舵机后重实测 (新 zero raw=2048, f 读数反推确认):
 *   后挡块 servo 77.80° = raw 885 = 2048-1163;
 *   前挡块 servo 271.30° = raw 3086 = 2048+1038。
 *   合法弧 = raw 正向 [885 -> 3086] (span 193.5°, 不跨 0/360)。
 *   2026-09-11 修正前挡块: 用户实测收纳位 J3 servo 273.58° = raw 3112, 已超出
 *   手推实测的 3086 有 26 raw (2.3°) —— 手推顶死测得的挡块偏浅 (金属舵盘齿隙
 *   + 手推力不足)。把前偏移放宽到 +1090 (raw 3138), 把该姿态连同 26 raw 余量
 *   一起包进合法弧。后挡块不动。
 *   ⚠ 偏移量隐含装配前提: 新舵机装盘时 zero(2048) 对准官方 Home 姿态。
 *   今后重新拆装舵机 / 重锚 Home 零点后, 挡块相对 zero 的偏移会变,
 *   必须重测两个挡块 (断电解力轻推顶死, f 读 servo 度换 raw) 并同步改宏。
 *   (旧舵机实测 后 2013 = 3221-1208 / 前 114 = 3221+989 已作废)
 *   ⚠ J5 (-1033/+787) 复核 (2026-09-15, 用户确认 + 实测):
 *     1) **J5 舵机从未更换过** ⇒ 这对偏移是为当前这只 J5 量的, 不是"旧舵机值",
 *        不要拿它当嫌疑人 (旧措辞"旧舵机 zero=3081 时代实测"曾误导一轮排查)。
 *     2) 真问题是**窗口跨度只有 160°**, 而 J5 物理行程实测 241.2°
 *        (固件窗口 servo [89.23,249.23]; 主臂 ma 0 人力拖动可到 [17.94,259.14])
 *        ⇒ 从臂被静默 clamp 在窗口边界: clamp_raw() 夹住后照常下发、返回成功、
 *        不打印 ⇒ 控制台全干净, 历轮被误判成"J5 舵机慢"。
 *     3) 处置 (2026-09-16, 用户实机验证"手推完全 OK"后定案):
 *        **J3/J5 与其余四轴统一 —— 一律采用 Phase 2 手推实测的 calib_min_raw /
 *        calib_max_raw。** 两对内置偏移宏与 hm 开关均已**删除**, 限位来源只剩
 *        一条路径, 不再有"两条互斥分支"这种极易读反的结构。
 *        留档 (仅备查, 代码中不再出现; 推算 -1843/+899 那条也一并作废):
 *          J3 后挡块 zero-1163 / 前挡块 zero+1090 → zero=2048 时 raw 885~3138 = servo 77.8~275.8
 *          J5 下限   zero-1033 / 上限   zero+787  → zero=2048 时 raw 1015~2835 = servo 89.2~249.2
 *        回退办法: 烧 outputs/link_pc/ 里的 .prev 固件 (主 d0a1d1ad / 从 96e0e838),
 *        详见 docs/J3J5限位试验方案_2026-09-16.md。
 * ⚠ 改用实测值后必须记住的代价:
 *   J3 手推弧几乎占满全行程 (23~4083), J5 手推记录 (14~357) 把 77~174 卡区整个包进去
 *   ⇒ 这两轴的**禁入区 / 卡区保护已不再存在**。今后一旦换舵机、重锚 Home 零点或
 *   机械上重新压过挡块, **必须重做一遍 Phase 2 手推标定**(走 `c` 校准并 `s` 保存),
 *   否则限位会静默沿用旧的错误行程。 */

static void apply_calib_limits(void) {
    uint8_t applied = 0;
    for (uint8_t j = 0; j < 6; j++) {
        /* 六轴一视同仁: 直接用 Phase 2 实测行程量 MIN/MAX (raw 空间, 不跨 0 不包裹)。
         * 2026-09-16 起 J3/J5 不再有内置偏移分支 —— 用户实机验证手推值可用。 */
        uint16_t mn = calib_min_raw[j];
        uint16_t mx = calib_max_raw[j];
        uint16_t span = raw_dist_fwd(mn, mx);
        if (span == 0 || span < 20U) continue;   /* 未校准/行程过短: 跳过, 不写限位 */
        JOINT_MIN_RAW[j] = mn;
        JOINT_MAX_RAW[j] = mx;
        JOINT_MIN[j] = servo_raw_to_deg(mn);   /* 同步 deg 镜像 (MoveJoints/IK 判越界也用) */
        JOINT_MAX[j] = servo_raw_to_deg(mx);
        applied++;
    }
    IK_SetJointLimitsRaw(JOINT_MIN_RAW, JOINT_MAX_RAW, SERVO_OFFSET);
    cprint("  Work ranges applied (%d jnt): all axes = Phase2 hand-pushed travel.\r\n",
           applied);
}

/* R1 (RC 末端控制) 的限位来源: **RAM 里的运行时限位** (apply_calib_limits 装载,
 * `L` 显示的同一份数据)。⚠ 刻意不用 ArmApi_GetLimits() —— 那条路读扇区 11 的
 * flash PID 记录, 本板无有效记录 (PID_FlashLoad rc=1), 而校准限位只存在于 RAM。
 * 返回装载轴数 (6 = 全装载); 未装载轴写 0 (调用方据此拒绝武装)。 */
static float servo_to_joint_deg(float sdeg, uint8_t j);   /* 定义在 1769 行附近 */

uint8_t ArmMain_GetJointLimitsDeg(float mn[6], float mx[6]) {
    uint8_t ok = 0;
    for (uint8_t j = 0; j < 6; j++) {
        if (JOINT_MAX_RAW[j] > JOINT_MIN_RAW[j] &&
            raw_dist_fwd(JOINT_MIN_RAW[j], JOINT_MAX_RAW[j]) >= 20U) {
            mn[j] = servo_raw_to_deg(JOINT_MIN_RAW[j]);
            mx[j] = servo_raw_to_deg(JOINT_MAX_RAW[j]);
            ok++;
        } else {
            mn[j] = 0.0f;
            mx[j] = 0.0f;
        }
    }
    return ok;
}

/* ⭐ 域转换 L1(servo 度 0~360) -> L2(关节度 (-180,180]), 与 servo_to_joint_deg
 *   同式。R1 专用: cmdSolveIKPoint / ArmApi_IkQuery / ArmApi_GetStatus 的关节
 *   输出全是 L1, 而 ArmApi_StreamJoints 的输入是 L2 (内部做 joint_to_servo_deg)
 *   —— 2026-09-23 "乱走"事故根因就是 L1 直接喂 L2 入口 (双重映射, 每轴错
 *   ~180°)。域转换只许走这里, 不许调用方手搓。 */
float ArmMain_ServoToJoint1(float sdeg, uint8_t j) {
    return servo_to_joint_deg(sdeg, j);
}

/* ===== 0924BY: L 手动限位持久化 (扇区 11 区 I @ CFG_OFF_JLIM 0x8000) =====
 * 背景: J3 校准限位 76~271 (Phase 2 手推行程, 存区 A) 压不住遥操下压深度 ——
 * `L 2 min max` 可运行时扩, 但旧实现 RAM-only 断电丢。
 * now: L 设值即写区 I; 上电在校准限位装载**之后**最后覆盖 (手动值优先)。
 * `L clear` 撤销: 回校准限位(有校准)/编译默认, 区 I 写无效记录。 */
#define JLIM_MAGIC 0x4A4C4931U   /* "JLI1" */
typedef struct {
    uint32_t magic;
    uint16_t mn[6];
    uint16_t mx[6];
    uint32_t sum;                /* 前 28B 按 u32 求和取反 */
} JlimRec;                       /* 32B, 字段手工对齐无隐式 padding */

static const float JLIM_DEF_MIN[6] = {  65,  75,  76,  71,  12,  144 };
static const float JLIM_DEF_MAX[6] = { 300, 284, 271, 284, 351, 264 };

static uint32_t jlim_sum(const JlimRec* r) {
    const uint32_t* w = (const uint32_t*)(const void*)r;
    uint32_t s = 0U;
    for (uint8_t i = 0; i < 7U; i++) s += w[i];   /* magic+mn+mx = 28B = 7 word */
    return ~s;
}

static uint8_t Jlim_Save(void) {
    JlimRec r;
    r.magic = JLIM_MAGIC;
    for (uint8_t j = 0; j < 6U; j++) {
        r.mn[j] = JOINT_MIN_RAW[j];
        r.mx[j] = JOINT_MAX_RAW[j];
    }
    r.sum = jlim_sum(&r);
    return CfgFlash_WriteRegion(CFG_OFF_JLIM, &r, (uint32_t)sizeof(r));
}

static void Jlim_ApplyRecord(const JlimRec* r) {
    for (uint8_t j = 0; j < 6U; j++) {
        if (r->mx[j] <= r->mn[j]) return;   /* 坏记录: 整体不应用 (回校准/默认) */
    }
    for (uint8_t j = 0; j < 6U; j++) {
        JOINT_MIN_RAW[j] = r->mn[j];
        JOINT_MAX_RAW[j] = r->mx[j];
        JOINT_MIN[j] = servo_raw_to_deg(r->mn[j]);   /* deg 镜像同步 */
        JOINT_MAX[j] = servo_raw_to_deg(r->mx[j]);
    }
    IK_SetJointLimitsRaw(JOINT_MIN_RAW, JOINT_MAX_RAW, SERVO_OFFSET);
}

static void Jlim_LoadApply(void) {
    JlimRec r;
    if (CfgFlash_ReadRegion(CFG_OFF_JLIM, &r, (uint32_t)sizeof(r)) != CFG_FLASH_OK) return;
    if (r.magic != JLIM_MAGIC || r.sum != jlim_sum(&r)) return;
    Jlim_ApplyRecord(&r);
    cprint("  Joint limits: L 手动值已从 flash 恢复 (区 I)\r\n");
}

static void cmdLimitsClear(void) {
    for (uint8_t j = 0; j < 6U; j++) {
        JOINT_MIN[j]     = JLIM_DEF_MIN[j];
        JOINT_MAX[j]     = JLIM_DEF_MAX[j];
        JOINT_MIN_RAW[j] = deg_to_servo_raw(JLIM_DEF_MIN[j]);
        JOINT_MAX_RAW[j] = deg_to_servo_raw(JLIM_DEF_MAX[j]);
    }
    apply_calib_limits();        /* 有 Phase2 校准则校准优先 (原始行为) */
    {
        JlimRec r;
        memset(&r, 0, sizeof(r));   /* magic=0 = 无效记录 */
        (void)CfgFlash_WriteRegion(CFG_OFF_JLIM, &r, (uint32_t)sizeof(r));
    }
    cprint("  L 手动限位已清除: 回校准限位/编译默认, flash 区 I 已作废\r\n");
}

static void pid_load_startup(void) {
    uint8_t p[6], i[6], d[6], corr_pct[6];
    float saved_params[PID_FLASH_ARM_PARAM_MAX];
    PID_JointLimits_t saved_limits;
    uint16_t threshold_cdeg = 0;
    uint16_t moving_speed = STS_DEFAULT_GOAL_SPEED;
    uint8_t max_rounds = 0, enabled = 0, params_loaded = 0, limits_loaded = 0;
    if (PID_FlashLoad(p, i, d, corr_pct, &threshold_cdeg, &max_rounds, &enabled,
                      &moving_speed, saved_params, PID_FLASH_ARM_PARAM_MAX,
                      &params_loaded, &saved_limits, &limits_loaded) == PID_FLASH_OK &&
        threshold_cdeg >= 50U && threshold_cdeg <= 1000U &&
        max_rounds >= 1U && max_rounds <= 20U && moving_speed <= 2047U) {
        memcpy(pid_p, p, 6);
        memcpy(pid_i, i, 6);
        memcpy(pid_d, d, 6);
        for (uint8_t j = 0; j < 6; j++) {
            corr_gain_init[j] = (corr_pct[j] > 100U ? 100U : corr_pct[j]) / 100.0f;
            corr_gain[j] = corr_gain_init[j];
        }
        corr_threshold = threshold_cdeg / 100.0f;
        corr_max_rounds = max_rounds;
        correction_enabled = enabled ? 1U : 0U;
        all_servo_moving_speed = moving_speed;
        if (limits_loaded) {
            for (uint8_t j = 0; j < 6; j++) {
                calib_min_raw[j] = saved_limits.min[j];
                calib_pos_raw[j] = saved_limits.pos[j];
                calib_max_raw[j] = saved_limits.max[j];
                calib_zero_raw[j] = saved_limits.zero[j];
            }
            /* 顺序必须: 先重锚 Home 零点(改写 SERVO_OFFSET), 再算限位 ——
             * IK_SetJointLimitsRaw() 内部用 SERVO_OFFSET 把 raw 限位折到关节角,
             * 若它先跑, jlim 会用旧 offset(180°) 算, 与 rad_to_servo_deg 的输出
             * 错位, 解析解会被错误拒绝 -> 静默降级 CCD (位置对、pitch 自由)。
             * 2026-09-15 修正 (原为 apply 在前)。 */
            Kinematics_SetZeroRaw(calib_zero_raw);  /* Home 零点开机即生效 */
            apply_calib_limits();                   /* 再让已校准限位按新零位生效 */
        }
        /* 0924DA: 两条 loaded 合并成一行 (⚠ ArmParams_Restore 是功能调用, 必须保留) */
        uint8_t ap_restored = (params_loaded)
                              ? ArmParams_Restore(saved_params, ArmParams_Count()) : 0U;
        if (params_loaded && limits_loaded)
            cprint("  Flash config: PID/corr/speed%s + calib limits loaded.\r\n",
                   ap_restored ? "/algorithm" : "");
        else if (params_loaded)
            cprint("  Flash config: PID/corr/speed%s loaded.\r\n",
                   ap_restored ? "/algorithm" : "");
        else
            cprint("  Flash config: PID/corr/speed loaded.\r\n");
    } else {
        cprint("  No valid PID/correction/speed config; using firmware defaults.\r\n");
    }
    /* ⭐0924BY: L 手动限位 (区 I) 最后覆盖 —— 不论校准记录在不在, 手动值优先 */
    Jlim_LoadApply();
    SERVO_SetInitPID(pid_p, pid_i, pid_d);
    SERVO_SetInitSpeed(all_servo_moving_speed);
    /* zero (无论 Flash 恢复还是默认 180) 已定, 预设位姿换算成舵机度 */
    /* 0924AS: 区F 几何偏好上电恢复 —— C 世界偏移直接套用, zf 偏好交给
     * 下面的 update_home_dock() 生效 (偏好优先于折叠位推导)。
     * ⭐0924CG: 换 Load3 —— v3 记录多带 `P here` 折叠位标定 (relhome
     * 关节角)。恢复放 refresh_pose_servo() 之前: pose_home_j_rt 先填充,
     * POSE_HOME 舵机度 / 折叠位白名单 / 桌面下限随后全按标定值重算。
     * 旧 v1/v2 记录兼容读出 (fold_valid=0 走编译默认)。 */
    {
        float gx, gy, gz, gz_mm;
        uint8_t gz_vld = 0U, gz_on = 1U;
        uint16_t gz_mrg = 0xFFFFU;
        float   fold_j[6];
        uint8_t fold_vld = 0U;
        if (GeoCfg_Load3(&gx, &gy, &gz, &gz_mm, &gz_vld, &gz_on, &gz_mrg,
                         fold_j, &fold_vld)) {
            Kinematics_SetOffset(gx, gy, gz);
            if (gz_vld) geo_zfloor_mm = gz_mm;
            geo_zf_on = gz_on ? 1U : 0U;
            /* gz_mrg 读出即弃 (0924BZ: 边界安全带已删, 不再恢复) */
            if (fold_vld) {
                memcpy(pose_home_j_rt, fold_j, sizeof(pose_home_j_rt));
                pose_home_cal = 1U;
                pose_home_rt_init = 1U;   /* 已填充, 免被懒初始化覆盖 */
            }
            /* 0924DA: 三条 Geo pref 合并成两行 (C 一行, zf+折叠位一行) */
            cprint("  Geo pref: C %.1f %.1f %.1f\r\n",
                   (double)gx, (double)gy, (double)gz);
            if (gz_vld) {
                cprint("  Geo pref: zf %.1f mm (%s)%s\r\n",
                       (double)gz_mm, geo_zf_on ? "ON" : "OFF",
                       pose_home_cal ? " | 折叠位(P here)已恢复" : "");
            } else if (pose_home_cal) {
                cprint("  Geo pref: 折叠位(P here)已恢复\r\n");
            }
            if (pose_home_cal) {
                cprint("    relhome %.1f %.1f %.1f %.1f %.1f %.1f\r\n",
                       (double)pose_home_j_rt[0], (double)pose_home_j_rt[1],
                       (double)pose_home_j_rt[2], (double)pose_home_j_rt[3],
                       (double)pose_home_j_rt[4], (double)pose_home_j_rt[5]);
            }
        }
    }
    refresh_pose_servo();
    update_home_dock();                     /* 折叠位白名单 + 桌面下限 */
}

static void pid_save_current(void) {
    uint8_t corr_pct[6];
    for (uint8_t j = 0; j < 6; j++) {
        int pct = (int)(corr_gain_init[j] * 100.0f + 0.5f);
        corr_pct[j] = (uint8_t)(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    }
    float params[PID_FLASH_ARM_PARAM_MAX];
    uint8_t param_count = ArmParams_Snapshot(params, PID_FLASH_ARM_PARAM_MAX);
    PID_JointLimits_t limits;
    for (uint8_t j = 0; j < 6; j++) {
        limits.min[j] = calib_min_raw[j];
        limits.pos[j] = calib_pos_raw[j];
        limits.max[j] = calib_max_raw[j];
        limits.zero[j] = calib_zero_raw[j];
    }
    uint8_t st = PID_FlashSave(pid_p, pid_i, pid_d, corr_pct,
                               (uint16_t)(corr_threshold * 100.0f + 0.5f),
                               corr_max_rounds, correction_enabled,
                               all_servo_moving_speed, params, param_count,
                               &limits);
    cprint(st == PID_FLASH_OK ? "  PID/correction/speed/algorithm config saved to Flash.\r\n"
                               : "  Flash save FAILED.\r\n");
}

static void pid_reset_defaults(void) {
    static const uint8_t p[6] = {16, 32, 16, 16, 16, 16};
    static const uint8_t i[6] = {4, 4, 4, 4, 4, 4};
    static const uint8_t d[6] = {12, 12, 12, 12, 12, 12};
    memcpy(pid_p, p, 6); memcpy(pid_i, i, 6); memcpy(pid_d, d, 6);
    SERVO_SetInitPID(pid_p, pid_i, pid_d);
    for (uint8_t j = 0; j < 6; j++) {
        set_pid_gain(j, STS_ADDR_P_GAIN, pid_p[j]);
        set_pid_gain(j, STS_ADDR_I_GAIN, pid_i[j]);
        set_pid_gain(j, STS_ADDR_D_GAIN, pid_d[j]);
    }
    cprint("  PID defaults applied (use F to save).\r\n");
}

/* ===== Set PID gain for one joint (writes to 舵机 RAM + updates cache) ===== */
static void set_pid_gain(uint8_t joint, uint16_t addr, uint8_t value) {
    if (joint >= 6) return;
    SERVO_WriteSimple(JOINT_BUS[joint], JOINT_ID[joint], addr, &value, 1);
    if (addr == STS_ADDR_P_GAIN)      pid_p[joint] = value;
    else if (addr == STS_ADDR_I_GAIN) pid_i[joint] = value;
    else if (addr == STS_ADDR_D_GAIN) pid_d[joint] = value;
}

/* Start a coordinated move: read current positions, plan trajectory, enable torque */
/* 读取当前位置并启动一条六关节同步轨迹。
 * 导出：arm_control_api MoveJoints/MoveCartesian 调用（arm_main_exports.h） */
/* ⭐0924AD: 恢复"运动入口先重写 GOAL_SPEED"—— 0924AP 曾把本函数清空, 前提是
 * "舵机 RAM 里的 GoalSpeed 不会被别的路径写低"。这个前提被 RC 打破:
 * pace_speeds()/松手刹车帧会给 J1~J5 写各自的低 GoalSpeed (松手瞬间 ≈1 step/s;
 * 反馈钳制把指令钉住时 ≈ 舵机自身速率, 曾实测钉在 ~50 step/s ≈ 4.4°/s),
 * 而 ArmRcCtrl_Disarm 不清寄存器。之后任何轨迹类命令 (m/P 0/TRI 中位/z a)
 * 只 SyncWrite GoalPosition、从不重写 GoalSpeed ⇒ 每根轴被"上次 RC 留下的
 * 速度"各自封顶 —— 2026-09-29 实测 P 0: J4 ~39°/s 2s 到位, J2/J3 ~4.4°/s
 * 爬行数分钟, "Curve complete"+settle 5s 超时后舵机还在爬; 同步 S 曲线
 * 完全作废, 折叠->中位时 J4 先回直、J2/J3 还在低位 ⇒ 爪尖扫桌面。
 * 修法: 任何运动入口 (m/a/z) 先给六轴写 GoalSpeed=0 (0=全速; 曲线形状由
 * 50Hz 微步目标流保证, 舵机位置环只负责跟目标)。一次 6 写 ~几 ms, 非热路径。 */
static void TrajSpeedEnsureNormal(void) {
    for (uint8_t j = 0; j < 6U; j++) (void)SERVO_SetSpeed(j, 0U);
}

/* 起点姿态读取(多次采样多数表决, 读失败重试, 绝不用目标值兜底) —— 定义见文件后部 */
static uint8_t read_start_pose_checked(float out[6]);

#if ARM_MASTER_BUILD
/* 主臂"主动模式" (`ma 1`)。定义在 cmdMove() 之前, 因为它的入口要判这个标志。
 *   0 (默认) = 只读手拖: 拒绝一切运动命令, 六轴每秒重申 Torque OFF
 *   1        = 主动: 上力矩、接受 g/m/P 等运动命令并保持位姿。转发照旧,
 *              所以从臂会跟着主臂一起动 —— 此时人手不得停留在臂上。 */
static uint8_t master_hold_en = 0U;
#endif

void cmdMove(float target[6], float v_max, float accel) {
#if ARM_MASTER_BUILD
    /* 主臂默认只读: 误发一条轨迹会把正在被人力拖拽的主臂突然上力矩/
     * 甩动 —— 主臂是被人扶着的, 那样很危险。只有显式 `ma 1` 才放行。 */
    if (!master_hold_en) {
        (void)target; (void)v_max; (void)accel;
        cprint("  MASTER role: read-only (ma 0). Send 'ma 1' to allow motion.\r\n");
        return;
    }
#endif
    /* Cancel swing mode and pending correction if active */
    swing_active = 0;
    correction_pending = 0;

    /* 主从直写目标与规划轨迹不能并存: 流模式下流目标与插补目标会互相
     * 抢总线, 机械上表现为抖动/顿挫。任何轨迹入口先退出流模式。 */
    cmdStreamExit("trajectory move");

    /* 若上一段轨迹收尾降速残留低速,先恢复,否则新轨迹会从低速跑 */
    TrajSpeedEnsureNormal();

    /* Check E-Stop */
    if (IsEStop()) {
        cprint("  E-STOP active! Cannot move.\r\n");
        return;
    }

    /* Clamp target angles to safe limits */
    cprint("  PRE-CLAMP target: %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
           target[0], target[1], target[2], target[3], target[4], target[5]);
    uint8_t clamped = clamp_all_angles(target);
    cprint("  POST-CLAMP target: %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
           target[0], target[1], target[2], target[3], target[4], target[5]);
    if (clamped > 0) {
        cprint("  WARNING: %d joint(s) clamped to safe limits!\r\n", clamped);
    }

    /* Read a valid start point for every joint before planning. Never replace a
     * failed feedback read with 0 deg or the target pose: that produces a false
     * trajectory and can make the J2 follow guard hold forever. Use the same
     * robust reader as 'g'/'P' (multi-sample majority vote + retry); if it still
     * cannot get a trustworthy pose, reject this move safely. */
    float start[6];
    if (!read_start_pose_checked(start)) {
        cprint("  Move aborted: current pose unreadable (see above).\r\n");
        cprint("  Check bus/ID, then retry; no trajectory was started.\r\n> ");
        return;
    }

    /* Enable torque on all joints */
    for (uint8_t j = 0; j < 6; j++) {
        SERVO_SetTorque(j, 1);
    }

    /* 0924AP: 不再切换加速度, 单一 sts_accel_value 全局生效。 */

    /* Plan trajectory */
    Trajectory_Plan(start, target, v_max, accel);

    traj_tick = 0;
    traj_settle_pending = 0U;
    traj_settle_joint = 0U;
    traj_settle_mask = 0U;
    traj_settle_pass = 1U;
    traj_settle_stable_since = 0U;
    traj_settle_adaptive = 0U;

    /* Save targets for post-move gravity correction (J2/J3/J4) */
    for (uint8_t j = 0; j < 6; j++) {
        correction_target[j] = target[j];
        correction_offset[j] = 0.0f;
    }

    cprint("  Move started: v=%d deg/s a=%d deg/s2\r\n", (int)v_max, (int)accel);
    cprint("  From: %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
           start[0], start[1], start[2], start[3], start[4], start[5]);
    cprint("  To:   %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
           target[0], target[1], target[2], target[3], target[4], target[5]);
}

/* ===== 安全轨迹序列 (2026-09-02 修正): 防止爪尖在关节插补路径上扫过桌面 =====
 * 关节空间直线在笛卡尔空间是弧线: 从收拢位(POSE_HOME)直达远目标的
 * FK 采样路径最低 z=-127mm (砸穿桌面)。本模块把 g/p 的大范围运动
 * 拆成逐段经 FK 校验的安全路点序列, 任何一段校验失败即整条取消。
 *
 * 2026-09-02 修正:
 *   - J3 真实机械硬限位 10~177 (用户确认), 不再强制 J3>=90。
 *   - 所有硬编码路点 J3 均在 10~170 之间, 并受 JOINT_MIN/MAX 再次约束。
 *   - 展开链顺序: 抬肩 -> 展肘同时翻腕让工具朝上 -> 继续展肘 -> 调腕到高举。
 *   - 高举位 (z≈500mm) 做 J1 水平转位绝对安全; 下探段逐点校验。
 * 顺序规则: 抬升/展臂阶段 J1 不动, J1 只在高举位转, 下探阶段 J1 不动。 */

/* 桌面下限 = 运行期 traj_z_floor_mm (见 update_home_dock, 定义在
 * refresh_pose_servo 之后): = 折叠位模型爪尖 z - TRAJ_Z_MARGIN_MM。
 * 2026-09-11 从编译期常量 (曾 15/8) 改为跟随 POSE_HOME_J —— 旧写法每换一次
 * 折叠位就得再手工放宽一次 (15 让 P 0 双向被拒, 8 又被 J3=+93.54 击穿), 根因是
 * 模型在这个姿态偏保守 ~6mm。以后只改 POSE_HOME_J。
 * 立柱圆柱 (rho/z) 与逐点 raw 限位检查不变。 */
#define TRAJ_COL_R_MM   25.0f    /* J1->J2 立柱圆柱半径 (绕 J1 轴) */
#define TRAJ_COL_Z_MM   120.0f   /* 立柱圆柱顶高度: 桌面 -> J2 轴心水平线 */
/* 高举过渡姿态 (关节角, 相对 Home 零点): J2=-40/J3=-10/J4=-60 -> 工具竖直
 * 朝上, 爪尖 z≈486mm, J1 在此高度水平转位绝对安全。
 * 2026-09-02 触地事故重构: 旧常量是舵机度 (随 zero 漂移), 改为关节角定义,
 * 经 joint_to_servo_deg() 换算后恒落在 J3 合法弧/其余关节 raw 限位内。 */
#define RISE_J2_DEG   (-40.0f)
#define RISE_J3_DEG   (-10.0f)
#define RISE_J4_DEG   (-60.0f)
#define SAFE_SEQ_MAX    10

/* ===== 角度体系三层概念 (2026-09-02 重构, 务必分清) =====
 *   L0 RAW (0-4095): 限位权威 + 舵机协议, 唯一不发生"跨 0/360 重复"的表示。
 *                    所有限位判断/运动 clamp/路径检查都在此层 (raw_in_range)。
 *   L1 SERVO 度 (0~360): 协议标尺, 用户 a/L/p 等命令的输入单位, 显示用。
 *   L2 关节角 (相对 Home 零点, (-180,180]): IK/FK 工作空间 + 状态显示。
 *                    对 J3 合法弧 (-106.2~+86.8) 全程唯一单调, 无 0/360 跳变
 *                    (f/p 命令的 rel-home 列就是它)。 */
/* 关节角 <-> 舵机度 (0~360 wrap), 与 kinematics.c 同基准 (SERVO_OFFSET) */
static float joint_to_servo_deg(float jdeg, uint8_t j) {
    float s = SERVO_OFFSET[j] + JOINT_DIR[j] * jdeg;
    s = fmodf(s, 360.0f);
    if (s < 0.0f) s += 360.0f;
    return s;
}
/* 舵机度 -> 相对 Home 零点的带符号关节角 (-180,180]: 显示用, 消除 0/360 跳变
 * (J3 合法弧内唯一: 350° 显示 -10°, 10° 显示 +10°, 不出现 350->360->3)。 */
static float servo_to_joint_deg(float sdeg, uint8_t j) {
    float d = JOINT_DIR[j] * (sdeg - SERVO_OFFSET[j]);
    d = fmodf(d + 180.0f, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d - 180.0f;   /* 归一到 (-180, 180] */
}

static float   safe_seq_wp[SAFE_SEQ_MAX][6];
static uint8_t safe_seq_n = 0;
static uint8_t safe_seq_idx = 0;
static uint8_t safe_seq_active = 0;

/* 臂平面内关键点相对 J1 轴的水平半径 (J1 旋转只改变方位角, 不改变半径):
 *   J3 轴心: rho = J2_OFF_Y + L_UPPER*sin(psiA),  z = BASE_H+J2_OFF_Z+L_UPPER*cos(psiA)
 *   J4 轴心:                       + L_FORE*sin(psiB),            + L_FORE*cos(psiB)
 * 立柱圆柱体 (J1 轴, 半径 TRAJ_COL_R_MM, 高 TRAJ_COL_Z_MM):
 *   rho < R 且 z < 立柱顶 -> 连杆/机身碰撞。高于柱顶 (z>=顶) 或柱外 (rho>=R) 均安全。
 * 2026-09-02 实测教训: 展开链 J3 单段 1->240 在 J3≈25-95° 区间前臂回转圆弧
 *   横穿该圆柱 (J4 轴心 z≈90-100 且 rho<25), 爪撞底座立柱 -> 舵机堵转卡死。 */
static uint8_t fk_path_pt_ok(float rho, float z) {
    if (z < traj_z_floor_mm) return 0;            /* 桌面 (折叠位推导下限) */
    /* 2026-09-02 修正: 立柱在 J1 轴两侧都有, 负 rho (臂后折) 同样会撞,
     * 必须 |rho| 判定 —— 旧代码只判 rho<25, 后折路径全部漏检。 */
    if (fabsf(rho) < TRAJ_COL_R_MM && z < TRAJ_COL_Z_MM) return 0;  /* 立柱圆柱 */
    return 1;
}

/* ===== 0924AU: J2 壳体包线 (colj2) =====
 * 现场两次碰撞 (2026-09-29): 夹爪初端 (1) 下探姿态顶到 J2 顶部外壳,
 * (2) 收拢姿态顶到 J2 前部外壳。用户实测结构尺寸:
 *   J2 舵机顶部外壳离桌垂直 131.38mm;
 *   J1 轴心 -> J2 前部外壳水平 51.05mm。
 * 夹爪 = 等腰三角形箭头, 中位姿态立着指向前方 ⇒ J5 roll 转到任何角度,
 * 横向半宽都存在 —— 用"沿工具轴采样 + 每点 ±W 轴向/面内垂直偏移"覆盖。
 * 包线 (臂平面内): 0 <= rho < COLJ2_RHO_MAX 且 z < COLJ2_Z_TOP。
 * 默认值由 tools/colj2_check.py 离线标定: 两个碰撞姿态必拦, 折叠/中位/
 * 待命/挥动及其插补路径全部不受影响。 */
#define COLJ2_RHO_MAX   65.0f    /* 前部外壳 51.05 + ~14 余量 (含模型误差) */
#define COLJ2_Z_TOP     143.5f   /* 顶部外壳 131.38 + ~12 余量 */
#define COLJ2_W         18.0f    /* 夹爪横向半宽 (等腰三角箭头半高) */
static uint8_t colj2_en = 1U;    /* RAM 开关: colj2 on|off (不存 flash) */

/* 单点 (rho,z) 是否在 J2 壳体包线内 */
static uint8_t colj2_pt_in(float rho, float z) {
    return (uint8_t)(rho >= 0.0f && rho < COLJ2_RHO_MAX && z < COLJ2_Z_TOP);
}

/* 夹爪采样点检查: tcp_rho/tcp_z = FK 爪尖 (基座坐标), yaw_rad = 工具俯仰(rad)。
 * 返回 0=有采样点进包线 (调用方拒绝该姿态), 1=安全。纯浮点运算, 零总线开销。 */
static uint8_t colj2_gripper_ok(float tcp_rho, float tcp_z, float yaw_rad) {
    if (!colj2_en) return 1U;
    const float dx = sinf(yaw_rad), dz = cosf(yaw_rad);   /* 工具方向 (臂平面) */
    const float px = cosf(yaw_rad), pz = -sinf(yaw_rad);  /* 面内垂直方向 */
    static const float d_samp[4] = { 23.58f, 50.0f, 80.0f, ROBOT_L_TOOL };
    for (uint8_t i = 0; i < 4U; i++) {
        float bx = tcp_rho - (ROBOT_L_TOOL - d_samp[i]) * dx;
        float bz = tcp_z  - (ROBOT_L_TOOL - d_samp[i]) * dz;
        if (colj2_pt_in(bx, bz))                              return 0U;
        if (colj2_pt_in(bx + COLJ2_W * dx, bz + COLJ2_W * dz)) return 0U;
        if (colj2_pt_in(bx - COLJ2_W * dx, bz - COLJ2_W * dz)) return 0U;
        if (colj2_pt_in(bx + COLJ2_W * px, bz + COLJ2_W * pz)) return 0U;
        if (colj2_pt_in(bx - COLJ2_W * px, bz - COLJ2_W * pz)) return 0U;
    }
    return 1U;
}

/* 路径校验失败原因 (由 fk_path_z_ok 写入, 供 safe_seq_plan 打印)。
 * 注意: 裸机单线程, 调用 check_path 前清空, 失败后读取。 */
static char last_fk_path_fail[80];

/* a->b 关节直线插补路径的 FK 采样校验:
 *   (1) 爪尖 z >= 桌面阈值;  (2) J3/J4 轴心、爪尖不进入 J1 立柱圆柱。
 * 2026-09-03: 加入失败原因诊断, 便于定位 "NO safe segment plan"。 */
static uint8_t fk_path_z_ok(const float a[6], const float b[6]) {
    Kinematics_Pos p;
    float q[6], rad[6];
    const uint8_t N = 24;
    for (uint8_t i = 0; i <= N; i++) {
        float r = (float)i / (float)N;
        for (uint8_t j = 0; j < 6; j++)
            q[j] = a[j] + (b[j] - a[j]) * r;
        for (uint8_t j = 0; j < 6; j++)
            rad[j] = servo_deg_to_rad(q[j], j);
        if (!FK_Solve(rad, &p)) {
            snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                     "FK solve failed at i=%u", (unsigned)i);
            return 0;
        }
        /* ===== 折叠位白名单 (2026-09-11) =====
         * 采样点 6 轴全落在折叠位 ± HOME_DOCK_TOL_DEG 内 -> 该点属于"已知安全
         * 停靠位", 跳过 raw 限位与桌面 z 检查。用户明确要求折叠位无条件可达:
         * 它是贴死 J3 前挡块的收纳姿态, 天然踩在挡块边界上, 且模型爪尖 z
         * (3.98mm) 比实测 (10mm) 保守 ~6mm —— 不放行就会让 P 0 起点/终点双向
         * 被拒 (规划死锁, 只能 `a J` 单轴救援)。
         * 豁免半径只有 2°(爪尖 ±7mm), 路径中段照旧严格检查, 09-01 那种中段
         * 下探 (z=-127mm) 仍拦得住; 立柱圆柱检查也不豁免。 */
        if (fk_is_home_dock(q)) continue;
        /* 2026-09-02 深夜: 每个插补点必须逐关节通过 raw 限位检查。
         * 端点合法不代表路径合法 —— P 0 (J3 310°→133°) 端点都在限位内,
         * 但插补途中 J3 raw 穿过禁入区 (10,177)° 伺服角度带, 真机撞后挡块。
         * raw_in_range 对包裹弧 (J3: [后挡块 raw .. 前挡块 raw] 跨 0) 天然正确。 */
        for (uint8_t j = 0; j < 6; j++) {
            uint16_t raw = deg_to_servo_raw(q[j]);
            if (!raw_in_range(raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) {
                snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                         "J%u raw %u out of range [%u,%u] at i=%u",
                         (unsigned)j + 1U, (unsigned)raw,
                         (unsigned)JOINT_MIN_RAW[j], (unsigned)JOINT_MAX_RAW[j],
                         (unsigned)i);
                return 0;
            }
        }
        /* FK_Solve 输出叠加了 WORLD_OFFSET (C 命令), 桌面/立柱判定必须
         * 用基座坐标系 —— 2026-09-02 修正, 否则世界偏移会平移安全阈值。 */
        if (p.z - WORLD_OFFSET_Z < traj_z_floor_mm) {
            snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                     "TCP z=%.1f < floor %.1f at i=%u",
                     (double)(p.z - WORLD_OFFSET_Z), (double)traj_z_floor_mm,
                     (unsigned)i);
            return 0;
        }
        /* J3/J4 轴心圆柱校验 (臂平面内半径不随 J1 变) */
        float psiA = ROBOT_PSI_UPPER0 + rad[1];
        float psiB = ROBOT_PSI_FORE0  + rad[1] + rad[2];
        float rhoJ3 = ROBOT_J2_OFF_Y + ROBOT_L_UPPER * sinf(psiA);
        float zJ3   = ROBOT_BASE_H + ROBOT_J2_OFF_Z + ROBOT_L_UPPER * cosf(psiA);
        float rhoJ4 = rhoJ3 + ROBOT_L_FORE * sinf(psiB);
        float zJ4   = zJ3   + ROBOT_L_FORE * cosf(psiB);
        if (!fk_path_pt_ok(rhoJ3, zJ3)) {
            snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                     "J3 axis rho=%.0f z=%.0f in column at i=%u",
                     (double)rhoJ3, (double)zJ3, (unsigned)i);
            return 0;
        }
        if (!fk_path_pt_ok(rhoJ4, zJ4)) {
            snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                     "J4 axis rho=%.0f z=%.0f in column at i=%u",
                     (double)rhoJ4, (double)zJ4, (unsigned)i);
            return 0;
        }
        /* 0924AU: J2 壳体包线 —— 夹爪采样点 (J6/中段/TCP ± 横向半宽)
         * 不得进入 J2 舵机壳体区 (前 51.05 / 顶 131.38, 用户实测)。 */
        {
            float wrho = hypotf(p.x - WORLD_OFFSET_X, p.y - WORLD_OFFSET_Y);
            if (!colj2_gripper_ok(wrho, p.z - WORLD_OFFSET_Z, p.yaw)) {
                snprintf(last_fk_path_fail, sizeof(last_fk_path_fail),
                         "gripper in J2 housing zone at i=%u", (unsigned)i);
                return 0;
            }
        }
    }
    return 1;
}

/* ============================================================
 * 主从直写模式 (2026-09-11, 实时遥操作跟随)
 *
 * 背景: 50Hz 级别的连续跟随不能走 cmdMove() —— 它每次都要读起点(多数
 * 表决) + 逐插补点限位/桌面/立柱检查 + S 曲线规划 + settle(500ms 稳定
 * 窗口), 单次几十~几百 ms, 且前提是"臂已停稳"。本模式直接把主臂的
 * 逻辑关节角写进六轴目标寄存器: 不插补、不读起点、不 settle。
 *
 * 三层安全 (缺一不可):
 *   ① 逐轴 clamp 到 JOINT_MIN/MAX_RAW —— 与轨迹模式共用同一套硬限位;
 *   ② 单帧跳变限制 —— 主臂丢帧/异常值不会让从臂猛冲;
 *   ③ 桌面 z + J3/J4 立柱圆柱单点检查 —— 不满足则【拒绝整帧】(保持上
 *      一点), 而不是钳制 (钳制会造出用户没要求的姿态)。
 * 另加看门狗: 断流 STREAM_TIMEOUT_MS 后退出流模式; 位置模式下舵机
 * 自锁保持最后目标, 不会失力砸臂。
 *
 * 角度语义: 入参是【相对本臂 Home 的逻辑关节角】(deg)。两端零点各自
 * 校准 (SERVO_OFFSET 不同), 故协议上必须传逻辑角而非舵机度。
 * ============================================================ */

#define STREAM_TIMEOUT_MS    500U    /* 断流保护 (ms) */
#define STREAM_MAX_JUMP_DEG  25.0f   /* 单帧单轴最大变化 (逻辑角 deg) */

static uint8_t  stream_active  = 0U;   /* 1=流模式已激活 */
static uint8_t  stream_prev_ok = 0U;   /* stream_prev 是否可信 */
static uint32_t stream_last_ms = 0U;   /* 最近一次成功下发时刻 */
static float    stream_prev[6];        /* 上一帧实际下发的逻辑关节角 */
static uint16_t stream_reject  = 0U;   /* 连续拒绝计数 (诊断/限流打印) */
static uint32_t stream_reject_max = 0U; /* 历史最长连续拒绝帧数 (判是否自锁) */
static uint32_t stream_wd_count   = 0U; /* 看门狗(断流 500ms)触发次数 */
/* `sf 0` 暂停跟随: 进来的 0x13 帧一律忽略。刻意返回 OK 而非 NACK ——
 * 链路只在 NACK 时计数, 暂停期间不该把非法的"拒绝"记进 reject 统计里
 * (ACK 本身被 nack_only 抑制, 所以线上一片安静)。 */
static uint8_t  stream_follow_en = 1U;
/* ⭐0924BS 误差比例跟随: 1 = 流控帧把"本帧要走距离÷时间"算成 GoalSpeed 与
 * GoalPos 一起 sync write 下发 (主臂快⇒从臂快, 直传速度); 0 = 旧行为
 * (从臂以 i 初始化写入的固定 GoalSpeed≈200 step/s 匀速走向目标)。
 * 仅 fi 1 置 1; `fi spd 0/1` 可手动切换。 */
static uint8_t stream_prop_speed = 0U;
/* ⭐0924BS 跟随轴加速度 (STS addr41, 0~254)。i 初始化默认写 50 —— 主臂
 * 突然加速时从臂被加速度曲线拖住, 表现为"永远慢半拍、恒定滞后"。
 * fi 1 时把 6 个跟随轴写满 254 (≈无加速度限制), `fi acc <0-254>` 可调。 */
static uint8_t fi_follower_accel = 254U;
/* ⭐0924BS 遥测节流: [FIT] 行最快 1 行/s */
static uint32_t stream_stat_last = 0U;
/* ⭐0924BS 遥操直通闸门: 1 = 流控里跳变限制(STREAM_MAX_JUMP_DEG)、桌面/立柱
 * 位姿检查、"大走距压慢速"三道闸门全部旁路 —— 遥操只受**硬限位 clamp** 约束
 * (用户 2026-11-05 明令: 遥操只被限位限制, 其他算法机制全不要)。
 * fi 1 自动置 1, fi 0/2 清 0 (RC/网口中继路径不受影响, 仍走全闸门)。 */
static uint8_t stream_free_gates = 0U;

/* ⭐0924BT fi 诊断缓存: cmdStreamJoints 每帧把实际下发的夹后目标/误差/
 * 速度存这里, `fi diag` 一次性快照读出 (力矩/主臂/目标/实际/误差/速度
 * 一屏判别 "帧到没到 / 舵机执行没执行 / 目标被夹到哪")。
 * 声明必须在 cmdStreamJoints (≈2515) 之前 —— fi 命令解析在 2700 后,
 * 同文件 static 前向可见即可。 */
static uint16_t fi_dbg_tgt_raw[6];   /* 本帧实际下发目标 raw (夹后) */
static uint16_t fi_dbg_err[6];       /* 从臂实际→本帧目标 圆周距 raw */
static uint16_t fi_dbg_spd[6];       /* 本帧 GoalSpeed */
static uint32_t fi_dbg_dt_ms = 0U;   /* 本帧 dt */
static uint32_t fi_dbg_valid = 0U;   /* 缓存被写过至少一帧 */
static uint32_t fi_rej = 0U;         /* 流控拒帧数 (0924BT: 原 fi_frames 把
                                        cmdStreamJoints==1(拒帧) 当成功计数,
                                        数字反了, 修正后成功/拒帧分开记) */

/* ===== 运行状态位图 (2026-09-17) =====
 * CAN 周期状态帧(0x280+node)的数据源。集中在这里是因为这些 static 都住在
 * main.c, 而协议层不该反过来 include main.c 的内部变量。
 * ⚠ 只读变量, 绝不在这里读舵机总线 —— 状态帧是周期发的, 每帧读 6 轴
 *   会把 1Mbps 以下的半双工总线(Bus A 上还挂着 3 颗舵机)拖出丢帧。 */
uint8_t ArmMain_Flags(void) {
    uint8_t f = 0U;
    if (IsEStop())                       f |= ARM_MAIN_FLAG_ESTOP;
    if (Trajectory_IsActive())           f |= ARM_MAIN_FLAG_TRAJ;
    if (stream_active)                   f |= ARM_MAIN_FLAG_STREAM;
    if (correction_pending)              f |= ARM_MAIN_FLAG_CORR;
    if (teach_mode_active || teach_record_only) f |= ARM_MAIN_FLAG_TEACH_REC;
    if (teach_play_state != TEACH_PLAY_NONE)    f |= ARM_MAIN_FLAG_TEACH_PLAY;
    return f;
}
static uint32_t stream_ignored   = 0U; /* 暂停期间被忽略的流帧数 */

/* 逐轴"静默夹住"计数 (2026-09-15 现场: "J5 跟随始终比其他关节慢很多")。
 * clamp_raw() 把越界目标夹回限位边界后**照常返回成功, 一条日志都不打** ——
 * 于是"某一轴跟着跟着就不动了 / 总比别的轴慢半拍"最容易的真因(限位夹住)
 * 在现有全部诊断里是**不可见**的。这里逐轴记:
 *   cnt[j]      = 被夹住过多少帧
 *   max10[j]    = 单帧被夹掉的最大角度 (0.1 deg, 精简 newlib 没有 %f)
 *   frames      = 至少有一轴被夹的帧数
 * `lk` 打印。判据: 若某轴 cnt 明显高于其它轴 ⇒ 那一轴是被限位夹住, 不是舵机慢。 */
static uint32_t stream_clamp_cnt[6]   = {0U, 0U, 0U, 0U, 0U, 0U};
static uint32_t stream_clamp_max10[6] = {0U, 0U, 0U, 0U, 0U, 0U};
static uint32_t stream_clamp_frames   = 0U;

/* ⭐0924BF 持续顶限位告警 (2026-11-03 J3 冒烟事故).
 * 只靠"被夹帧数"看不出危险: 顶一下就松手是安全的, **连续几百帧都被夹**
 * 才是"舵机正被命令顶死挡块", 那是在烧线圈。
 * streak = 连续被夹的帧数(无夹帧即清零); 到 WARN 帧打印一次, 每翻倍再打。
 * 40Hz 下 200 帧 = 5s 连续顶死, 足够及时发现。 */
static uint32_t stream_clamp_streak   = 0U;
static uint32_t stream_clamp_streak_next = 0U;
#define STREAM_CLAMP_STREAK_WARN  200U

/* PC 中转计数 (`#P` 行接收端, 2026-09-15): 主臂吐行 -> PC 转发 -> 从臂这里。
 * 30~60 Hz 下**任何**成功打印都会淹掉控制台, 所以成功一律静默, 只在解不
 * 出 6 个数或从臂拒帧时限流报一次。 */
static uint32_t pc_relay_rx  = 0U;   /* 收到的合法 #P 行数 */
static uint32_t pc_relay_err = 0U;   /* 被 StreamJoints 拒绝的次数 */
static uint32_t pc_relay_bad = 0U;   /* 字段不足的行数 */

/* 单点姿态安全检查 (判据与 fk_path_z_ok 完全一致, 只是不做插补遍历)。
 * ⭐0924AJ 恢复规则 (z_ref): 绝对判据有个死角 —— 实际位姿已经在桌下
 *   (被 m/中位/手扳/断电下沉/重标几何带进去, 或换 C 偏移后 floor 相对抬高)
 *   时, 任何"仍留在桌下"的目标帧全部被拒 ⇒ RC/主从整帧 all-or-nothing,
 *   六轴全部冻死, 用户感受就是"碰了限位后摇杆按键全没反应"。
 *   恢复规则: 若参考位姿 (实际/上次接受) 本身已低于 floor, 则只要求
 *   **目标不低于参考** —— J1 水平转、抬升都能走, 继续下压仍被拒。
 *   z_ref 传 +1e9 (或参考安全) 时退化为绝对判据, 行为与旧版一致。 */
static uint8_t stream_pose_ok_ref(const float q[6], float z_ref) {
    Kinematics_Pos p;
    float rad[6];
    if (fk_is_home_dock(q)) return 1U;          /* 折叠位白名单 */
    for (uint8_t j = 0; j < 6; j++) rad[j] = servo_deg_to_rad(q[j], j);
    if (!FK_Solve(rad, &p)) return 0U;
    float zmin = traj_z_floor_mm;
    if (z_ref < zmin) zmin = z_ref;             /* 已在违例区: 不许比参考更深 */
    if (p.z - WORLD_OFFSET_Z < zmin) return 0U;          /* 桌面 */
    float psiA = ROBOT_PSI_UPPER0 + rad[1];
    float psiB = ROBOT_PSI_FORE0  + rad[1] + rad[2];
    float rhoJ3 = ROBOT_J2_OFF_Y + ROBOT_L_UPPER * sinf(psiA);
    float zJ3   = ROBOT_BASE_H + ROBOT_J2_OFF_Z + ROBOT_L_UPPER * cosf(psiA);
    float rhoJ4 = rhoJ3 + ROBOT_L_FORE * sinf(psiB);
    float zJ4   = zJ3   + ROBOT_L_FORE * cosf(psiB);
    if (!fk_path_pt_ok(rhoJ3, zJ3)) return 0U;
    if (!fk_path_pt_ok(rhoJ4, zJ4)) return 0U;
    /* 0924AU: J2 壳体包线 —— RC/主从帧级拒绝, 与轨迹路径同一判据 */
    {
        float wrho = hypotf(p.x - WORLD_OFFSET_X, p.y - WORLD_OFFSET_Y);
        if (!colj2_gripper_ok(wrho, p.z - WORLD_OFFSET_Z, p.yaw)) return 0U;
    }
    return 1U;
}

/* ⭐0924BV: 只查桌面下限的精简判据 (直通模式专用)。
 * 用户 2026-11-05: 遥操只受限位约束 —— 但 zf 桌面下限是防撞桌的物理保护,
 * 不是"算法闸门", 单独开关 (zf on|off, 区F 持久化) 独立于 stream_free_gates。
 * 带 0924AJ 恢复规则: 参考位姿已在桌下时只要求不低于参考 (抬升/平转能走)。 */
static uint8_t stream_floor_ok(const float q[6], float z_ref) {
    Kinematics_Pos p;
    float rad[6];
    if (fk_is_home_dock(q)) return 1U;          /* 折叠位白名单 */
    for (uint8_t j = 0; j < 6; j++) rad[j] = servo_deg_to_rad(q[j], j);
    if (!FK_Solve(rad, &p)) return 0U;
    float zmin = traj_z_floor_mm;
    if (z_ref < zmin) zmin = z_ref;             /* 已在违例区: 不许比参考更深 */
    return (p.z - WORLD_OFFSET_Z >= zmin) ? 1U : 0U;
}

static uint8_t stream_pose_ok(const float q[6]) {
    return stream_pose_ok_ref(q, 1.0e9f);
}

/* 流层姿态安全判据的只读导出 (2026-09-24, RC 遥控"顶到包线软停"用):
 * 与 cmdStreamJoints 内部的 stream_pose_ok 完全同一判据 (桌面 z + 立柱圆柱)。
 * ⚠ 域契约: 输入 L2 关节度, 内部先 joint_to_servo_deg 转 L1 再判 ——
 *   stream_pose_ok 吃的是流层 raw 反算的 sdeg (L1), 0924A 首版直接把 L2
 *   塞进去 ⇒ 白名单/FK 全部失配, RC 在中位一帧都发不出 ("怎么动都动不了",
 *   09-24 实机踩到, 0924B 修复)。 */
uint8_t ArmMain_StreamPoseOk(const float jdeg[6]) {
    float sdeg[6];
    for (uint8_t j = 0; j < 6U; j++) sdeg[j] = joint_to_servo_deg(jdeg[j], j);
    return stream_pose_ok(sdeg);
}

/* ⭐0924AJ 带实际位姿参考的判据 (RC 用): act_sdeg = 当前实际舵机度 (L1),
 * RC 会话本来就持有 act_l1, 零总线开销。实际已在桌下时启用恢复规则,
 * 目标只需不低于实际 —— 修"触限位后六轴全冻" (0924-09-29 实机)。 */
uint8_t ArmMain_StreamPoseOkAct(const float jdeg[6], const float act_sdeg[6]) {
    float sdeg[6];
    for (uint8_t j = 0; j < 6U; j++) sdeg[j] = joint_to_servo_deg(jdeg[j], j);
    float zref = 1.0e9f;
    float arad[6];
    Kinematics_Pos ap;
    for (uint8_t j = 0; j < 6U; j++) arad[j] = servo_deg_to_rad(act_sdeg[j], j);
    if (FK_Solve(arad, &ap)) zref = ap.z - WORLD_OFFSET_Z;
    return stream_pose_ok_ref(sdeg, zref);
}

void cmdStreamExit(const char* why) {
    /* RC 遥控让位: 任何轨迹类入口(m/示教/回放/多点)都会走到这里 ⇒ RC 解除。
     * 未武装时是静默 no-op (RC方案复核 P0-2)。 */
    ArmRcCtrl_Disarm("运动命令接管");
    if (!stream_active) return;
    stream_active = 0U;
    cprint("  Stream mode OFF (%s)\r\n", why ? why : "?");
}

/* sf 跟随开关读写导出 (arm_rc_ctrl 每拍查: sf 0 期间 RC 必须解除,
 * 否则流帧被静默忽略而积分器照常前进 ⇒ sf 恢复后臂突进)。
 * 0924U: 武装时自动置 1, 避免旧 sf=0 把刚武装的 RC 立即锁死。 */
uint8_t ArmMain_StreamFollowEn(void) { return stream_follow_en; }
void    ArmMain_SetStreamFollowEn(uint8_t en) { stream_follow_en = en ? 1U : 0U; }

/* ===== ⭐0924CI: GET_STATUS 遥测缓存通道 =====
 * ArmApi_GetStatus(0x02 GET_STATUS) 改走 FeedbackSampleService 的轮转缓存
 * (10ms/轴), 不再逐轴直读总线 —— fi 直通时 GET_STATUS 不再等舵机总线
 * 临界区, p95 从 ~68ms 降到 <1ms。缓存过期(>max_age_ms)/丢读轴回落
 * 直读(调用方兜底), 读_ok 语义不变。
 * 返回 1=取到缓存值 / 0=该轴缓存不可用(调用方须自行直读)。 */
static uint32_t g_stcache_hit = 0U, g_stcache_miss = 0U;
uint8_t ArmMain_GetCachedJointDeg(uint8_t j, float* out_deg, uint32_t max_age_ms) {
    if (j >= 6U || out_deg == NULL) return 0U;
    uint32_t now = HAL_GetTick();
    if (!telem_ok(j, now)) { g_stcache_miss++; return 0U; }
    if ((uint32_t)(now - telem_stamp[j]) > max_age_ms) { g_stcache_miss++; return 0U; }
    *out_deg = servo_raw_to_deg(telem_cache[j].pos);
    g_stcache_hit++;
    return 1U;
}

    /* 进入流模式: 结束一切在跑的轨迹类运动, 确保扭矩已开。
     * 0924AP: 不再限制 GoalSpeed, 也不再切换加速度, 让流控直接跑。 */
static void stream_enter(void) {
    Trajectory_Stop();
    multi_point_legs_left = 0U;
    qlc_active   = 0U;
    swing_active = 0U;
    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1U);
}

uint8_t cmdStreamJoints(const float jdeg[6]) {
    float sdeg[6];
    uint16_t raw[6];

#if ARM_MASTER_BUILD
    /* 主臂角色: 只读转发, 不接受任何直写目标 (会 Torque ON 并驱动) */
    (void)jdeg;
    return 3U;
#endif

    /* ⭐ P0-2 互斥 (RC方案复核 §P0-2): 收到**非 RC 遥控**来源的流帧
     *   (网口 STREAM_JOINTS / 主从 #P 中转) ⇒ RC 遥控立即解除。
     *   网口永远赢, RC 不自动抢回; 要再动必须重新 `rc go on`。
     *   (RC 自己的帧把 arm_rc_ctrl_in_frame 置 1, 不会进到这里) */
    if (!arm_rc_ctrl_in_frame) ArmRcCtrl_Disarm("外部流帧接管 (网口/主从)");

    /* `sf 0` 暂停跟随: 忽略流帧、立刻退出流模式 (从臂停在当前位置自锁)。
     * 主臂可以继续照发, 不会因此攒 reject 计数。 */
    if (!stream_follow_en) {
        stream_ignored++;
        if (stream_active) cmdStreamExit("sf 0");
        return 0U;
    }

    if (IsEStop()) return 2U;

    if (!stream_active) {
        stream_enter();
        stream_active  = 1U;
        stream_reject  = 0U;
        stream_prev_ok = 0U;
        /* 以从臂当前实际位置为跳变基准 (读失败 -> 本帧跳过跳变判据) */
        uint8_t ok = 1U;
        float cur[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        for (uint8_t j = 0; j < 6; j++) {
            float d = -1.0f;
            for (uint8_t t = 0; t < 3U; t++) {
                d = SERVO_GetPosition(j);
                if (d >= 0.0f) break;
                HAL_Delay(2);
            }
            if (d < 0.0f) { ok = 0U; break; }
            cur[j] = servo_to_joint_deg(d, j);
        }
        if (ok) {
            for (uint8_t j = 0; j < 6; j++) stream_prev[j] = cur[j];
            stream_prev_ok = 1U;
        }
        stream_last_ms = HAL_GetTick();
        cprint("  Stream mode ON (direct write, no planning) base=%s\r\n",
               ok ? "live" : "none");
    }

    /* 逻辑角 -> 舵机度 -> raw, 逐轴硬限位 clamp, 再反算实际下发角度 */
    uint8_t clamped_any = 0U;
    for (uint8_t j = 0; j < 6; j++) {
        float sd = joint_to_servo_deg(jdeg[j], j);
        uint16_t want = deg_to_servo_raw(sd);
        uint16_t mn = JOINT_MIN_RAW[j];
        uint16_t mx = JOINT_MAX_RAW[j];
        /* ⭐未校准保护 (0924BF): 限位未标定时 mn==mx==0, raw_in_range() 里
         * span==0 直接返回 0 ⇒ clamp_raw() 恒返回 mn=0 ⇒ **六轴目标全变 raw 0
         * (0°)** ⇒ 每轴全速冲向 0° 撞挡块堵转。clamp_all_angles() 一直有
         * `mn==0&&mx==0 -> continue` 这层保护, 但流控这里以前漏了。 */
        if (mn == 0U && mx == 0U) {
            raw[j] = want;
        } else {
            raw[j] = clamp_raw(want, mn, mx);
            if (raw[j] != want) {
                /* 被限位夹住: 记数与"被夹掉多少度" (取到所夹边界的**圆周最短**
                 * 距离, 2026-11-02 与 clamp_raw 同步修正 —— 旧的正向距离判据在
                 * "略超 max"场景会把 lost 算成大半圈)。整帧照旧下发, 不报错。 */
                uint16_t d1 = raw_dist_fwd(want, raw[j]);
                uint16_t d2 = raw_dist_fwd(raw[j], want);
                uint16_t lost = (d1 <= d2) ? d1 : d2;
                uint32_t lost10 = ((uint32_t)lost * 3600U) / 4095U;   /* 0.1 deg */
                stream_clamp_cnt[j]++;
                if (lost10 > stream_clamp_max10[j]) stream_clamp_max10[j] = lost10;
                clamped_any = 1U;
            }
            /* ⭐0924BZ: 边界安全带已删 (用户明令) —— 限位钳到边界即止,
             * 不再往弧内推。顶死风险由 [STALL] 告警覆盖。 */
        }
        sdeg[j] = servo_raw_to_deg(raw[j]);
    }
    if (clamped_any) {
        stream_clamp_frames++;
        /* 连续顶限位计时: 到 200 帧(≈5s@40Hz)报一次, 之后每翻倍再报。
         * 这是"舵机正在被命令顶死挡块"的唯一早期信号 —— 不报错、不停机,
         * 只提醒操作者松手, 避免 J3 那种手一碰就冒烟的事故。 */
        if (stream_clamp_streak < 0xFFFFFFFFU) stream_clamp_streak++;
        if (stream_clamp_streak >= STREAM_CLAMP_STREAK_WARN &&
            stream_clamp_streak >= stream_clamp_streak_next) {
            stream_clamp_streak_next = stream_clamp_streak * 2U;
            cprint("  [STALL] limit held %lu frames -- release stick (servo may overheat)\r\n",
                   (unsigned long)stream_clamp_streak);
        }
    } else {
        stream_clamp_streak = 0U;
        stream_clamp_streak_next = 0U;
    }

    /* ② 跳变限制: 用"实际下发角度"而非请求值判, 避免被 clamp 的帧误判。
     * ⭐0924BS: 遥操直通模式 (stream_free_gates=1) 旁路 —— 总线丢读使实际
     * 帧率只有 ~10-15Hz, 快甩主臂时单帧差轻松超 25° ⇒ 帧帧被拒、从臂冻住,
     * 这正是"超过某个速度就被限制住"的元凶。限位 clamp 仍在①里兜底。 */
    if (stream_prev_ok && !stream_free_gates) {
        for (uint8_t j = 0; j < 6; j++) {
            float dj = servo_to_joint_deg(sdeg[j], j);
            if (fabsf(dj - stream_prev[j]) > STREAM_MAX_JUMP_DEG) {
                stream_reject++;
                if ((stream_reject & 0x3FU) == 1U) {
                    cprint("  Stream reject: J%u jump %.0f deg (> %.0f)\r\n",
                           (unsigned)j + 1U, (double)fabsf(dj - stream_prev[j]),
                           (double)STREAM_MAX_JUMP_DEG);
                }
                return 1U;
            }
        }
    }

    /* ③ 桌面 z + 立柱圆柱单点检查
     * ⭐0924AJ: 以上次接受位姿为参考 —— 从臂已被带进桌下时 (floor 相对抬高/
     *   惯性冲过), 绝对判据会把所有帧全拒 ⇒ 主从链路六轴冻死。
     *   恢复规则: 只要求目标不低于上次接受位姿, 抬升/水平转都能走。
     * ⭐0924BS: 遥操直通模式旁路 (只保留硬限位)。 */
    if (!stream_free_gates) {
        uint8_t pok;
        if (stream_prev_ok) {
            float ps[6], prad[6];
            Kinematics_Pos pp;
            float zref = 1.0e9f;
            for (uint8_t j = 0; j < 6U; j++) ps[j] = joint_to_servo_deg(stream_prev[j], j);
            for (uint8_t j = 0; j < 6U; j++) prad[j] = servo_deg_to_rad(ps[j], j);
            if (FK_Solve(prad, &pp)) zref = pp.z - WORLD_OFFSET_Z;
            pok = stream_pose_ok_ref(sdeg, zref);
        } else {
            pok = stream_pose_ok(sdeg);
        }
        if (!pok) {
            stream_reject++;
            if ((stream_reject & 0x3FU) == 1U) cprint("  Stream reject: pose unsafe\r\n");
            return 1U;
        }
    } else if (geo_zf_on) {
        /* ⭐0924BV: 直通模式下桌面下限独立生效 (zf on, 区F 持久化) ——
         * 立柱/包线/跳变仍旁路, 只拦"爪尖低于 zf"。
         * 0924BS 时整段被旁路 ⇒ zf 形同虚设, 用户实测"zf 不起作用"即此。 */
        uint8_t pok;
        if (stream_prev_ok) {
            float ps[6], prad[6];
            Kinematics_Pos pp;
            float zref = 1.0e9f;
            for (uint8_t j = 0; j < 6U; j++) ps[j] = joint_to_servo_deg(stream_prev[j], j);
            for (uint8_t j = 0; j < 6U; j++) prad[j] = servo_deg_to_rad(ps[j], j);
            if (FK_Solve(prad, &pp)) zref = pp.z - WORLD_OFFSET_Z;
            pok = stream_floor_ok(sdeg, zref);
        } else {
            pok = stream_floor_ok(sdeg, 1.0e9f);
        }
        if (!pok) {
            stream_reject++;
            if ((stream_reject & 0x3FU) == 1U)
                cprint("  Stream reject: below zf floor (%.1f mm)  (zf off 关闭)\r\n",
                       (double)(traj_z_floor_mm + WORLD_OFFSET_Z));
            return 1U;
        }
    }

    if (stream_prop_speed) {
        /* ⭐0924BT 误差驱动速度 (修正 0924BP~BS 的语义缺陷):
         * BP~BS 用 stream_prev(上一帧目标)→本帧目标的距离算速度 —— 那是
         * **主臂帧间增量**, 不是从臂误差。从臂一旦落后(总线丢读/执行慢),
         * 误差只增不减; 主臂一停手 dist→0, v 被压到保底 150 step/s
         * (≈13°/s) ⇒ 从臂在半路爬行, 现场表象即"被限住/不动"。
         * 修: 每帧短读从臂实际 raw (2 试 × 4ms, 失败轴回退上一帧目标,
         * 不阻塞热路径), v = 圆周距(从臂实际→本帧目标)×1250/dt。
         * 从臂没到 → 全速追; 到位 → err→0 落到 150 微调。
         * GoalPos 仍是本帧目标: 位置伺服下 GoalSpeed 只是速度上限,
         * 到位即停, ×1.25 裕量不存在过冲风险。 */
        uint32_t dt = HAL_GetTick() - stream_last_ms;
        if (stream_free_gates) {
            if (dt < 10U) dt = 10U;    /* 只防除零, 不设上限: dt 是真实帧距 */
        } else {
            if (dt < 25U)  dt = 25U;
            if (dt > 400U) dt = 400U;
        }
        uint16_t spd[6];
        uint16_t errr[6];
        for (uint8_t j = 0; j < 6; j++) {
            errr[j] = 0U;
            if (!stream_prev_ok) { spd[j] = 1200U; continue; } /* 首帧: 慢奔赴 */
            /* 从臂实际 raw: ⭐0924CJ 先走遥测缓存 (FeedbackSampleService 10ms
             * 轮转, 0 总线事务) —— 取最近一次成功采样, 120ms 窗内都算可用
             * (不判 telem_err: 丢读总线上最新一拍失败≠数据无效, 只是不够新;
             * 速度估算容忍 ≤120ms 旧值, 目标是位置伺服, 收敛不受影响)。
             * 缓存出窗才直读 2 试×4ms。丢读总线下旧版每帧最多 12 次阻塞读
             * (最坏 ~48ms) 全砸在下发热路径上, 打乱节拍 —— 卡顿第二根源。 */
            uint16_t araw = 0xFFFFU;
            {
                uint32_t tnow = HAL_GetTick();
                if (telem_stamp[j] != 0U &&
                    (uint32_t)(tnow - telem_stamp[j]) < 120U) {
                    araw = telem_cache[j].pos;
                }
            }
            if (araw == 0xFFFFU) {
                for (uint8_t t = 0; t < 2U; t++) {
                    uint16_t r = SERVO_GetPositionRawTO(j, 4U);
                    if (r != 0xFFFFU) { araw = r; break; }
                }
            }
            if (araw == 0xFFFFU) {
                float sp = joint_to_servo_deg(stream_prev[j], j);
                araw = deg_to_servo_raw(sp);
            }
            uint16_t e1 = raw_dist_fwd(araw, raw[j]);
            uint16_t e2 = raw_dist_fwd(raw[j], araw);
            uint16_t err = (e1 <= e2) ? e1 : e2;
            errr[j] = err;
            uint32_t v = ((uint32_t)err * 1250U) / dt;   /* err*1000/dt ×1.25 */
            if (v < 150U)   v = 150U;      /* ⚠ GoalSpeed=0=全速(语义反向), 保底 */
            if (v > 4095U)  v = 4095U;     /* 舵机硬件满速, 无软件限速 */
            spd[j] = (uint16_t)v;
        }
        /* ⭐0924BT 遥测: 从臂最大误差 >60raw(≈5.3°) 才打 (1 行/s) —— 主臂
         * 静止/慢动时 BP 版的 dist 阈值 150 永远不触发, [FIT] 一片空白,
         * 没有信息。改打 err: "err 大 spd 也大而臂不动" = 力矩/供电/执行
         * 问题(一锤定音); "err 小" = 已跟齐。 */
        {
            uint16_t maxe = 0U; uint16_t maxs = 0U; uint8_t maxj = 0U;
            for (uint8_t j = 0; j < 6U; j++) {
                if (errr[j] > maxe) { maxe = errr[j]; maxj = j; }
                if (spd[j] > maxs)  maxs = spd[j];
            }
            if (maxe > 60U && (uint32_t)(HAL_GetTick() - stream_stat_last) > 1000U) {
                stream_stat_last = HAL_GetTick();
                cprint("  [FIT] dt=%lums J%u err=%u(%.1fdeg) spd=%u\r\n",
                       (unsigned long)dt, (unsigned)maxj + 1U,
                       (unsigned)maxe, (double)((float)maxe * 360.0f / 4095.0f),
                       (unsigned)maxs);
            }
        }
        /* 诊断缓存 (0924BT, `fi diag` 读) */
        for (uint8_t j = 0; j < 6U; j++) {
            fi_dbg_tgt_raw[j] = raw[j];
            fi_dbg_err[j]     = errr[j];
            fi_dbg_spd[j]     = spd[j];
        }
        fi_dbg_dt_ms  = dt;
        fi_dbg_valid  = 1U;
        if (SERVO_SetPositionsRawSpeedSync(raw, spd) != HAL_OK) {
            stream_reject++;
            return 1U;
        }
    } else if (SERVO_SetPositionsRawSync(raw) != HAL_OK) {
        stream_reject++;
        return 1U;
    }

    for (uint8_t j = 0; j < 6; j++) stream_prev[j] = servo_to_joint_deg(sdeg[j], j);
    stream_prev_ok = 1U;
    stream_reject  = 0U;
    stream_last_ms = HAL_GetTick();
    return 0U;
}

/* 看门狗: 主循环每拍调用。断流 -> 退出流模式并冻结 (位置模式自锁保持) */
void cmdStreamService(void) {
    /* 记历史最长连续拒绝帧数: 偶发单帧拒(≈1)和"自锁冻结"(数十~上千)是两种
     * 完全不同的病, 单看累计百分比分不出来。`lk` 会打出这个峰值。 */
    if (stream_reject > stream_reject_max) stream_reject_max = stream_reject;
    if (!stream_active) return;
    if ((uint32_t)(HAL_GetTick() - stream_last_ms) > STREAM_TIMEOUT_MS) {
        stream_active = 0U;
        stream_wd_count++;
        cprint("  Stream watchdog: no cmd for %u ms -> hold (total %lu)\r\n",
               (unsigned)STREAM_TIMEOUT_MS, (unsigned long)stream_wd_count);
    }
}

/* ===== 板内遥操 (2026-11-04, 方案1: 同板双总线, 省主臂板) =====
 * 拓扑: 主臂 6 舵机 -> 本板 Bus A (ID 7~12), 从臂 6 舵机 -> 本板 Bus B (ID 1~6)。
 * 主臂板整块退役; 本板周期读 Bus A 主臂位置 -> 转逻辑角 -> cmdStreamJoints
 * 下发 Bus B 从臂, 板内直转零网络延迟。复用 0924BF 防顶死/软跟随保护。
 * 前提: 主臂舵机 ID 已改为 7~12 (避开从臂 1~6), 否则 AutoMap 判"ID 撞车"。 */
#define FI_INPUT_BUS    BUS_A
#define FI_INPUT_ID0    7U       /* 主臂 J1 舵机 ID */
/* ⭐0924CJ: 25->20ms (50Hz)。切片+部分帧降级下发后目标流连续, 延迟降 20%;
 * GET_STATUS 走遥测缓存不占总线, 节拍提速不影响状态查询帧率 (A 需求单)。 */
#define FI_PERIOD_MS    20U      /* 50Hz 读主臂节拍 */
/* ⭐0924BS: 10→4ms。HAL 语义 = 首字节等满 timeout, 后续字节静默 5ms 判停
 * (servo_bus_hal.c receive_common)。成功应答首字节正常 <1ms 到, 4ms 裕量足;
 * 丢读 60% 时失败尝试烧满超时 —— 10ms 下每帧期望 ~100-150ms(掉到 ~7-10Hz,
 * 用户实测"遥操好延迟"), 4ms 砍半到 ~50ms(~15-20Hz)。根治靠上拉电阻压丢读。 */
#define FI_READ_TO_MS   4U       /* 单次读超时 (热路径, 不拖主循环) */
static uint8_t  fi_en         = 0U;
static uint8_t  fi_mon        = 0U;   /* fi 2: 双臂角度监视 (只读不驱动) */
static uint32_t fi_mon_last   = 0U;
static uint32_t fi_last_ms    = 0U;
static uint32_t fi_frames     = 0U;   /* 成功跟随帧 */
static uint32_t fi_lost       = 0U;   /* 主臂丢读帧 (跳过本拍) */

/* 读主臂 Bus A 上 ID (7+j) 的舵机度; 失败返回 -1。
 * raw->角度换算与 servo_driver.c 的 RawToAngle() 同一公式 (4095=360°),
 * 但该函数是 servo_driver.c 内部 static, 这里就地内联一份。 */
static float fi_read_master_deg(uint8_t j) {
    uint8_t data[2] = {0, 0};
    uint8_t id = (uint8_t)(FI_INPUT_ID0 + j);
    HAL_StatusTypeDef st = DXL_ReadDiag(FI_INPUT_BUS, id,
                                        STS_ADDR_PRESENT_POSITION, 2, data,
                                        FI_READ_TO_MS, NULL, NULL);
    if (st != HAL_OK) return -1.0f;
    uint16_t raw = (uint16_t)(data[0] | (data[1] << 8));
    return ((float)raw * 360.0f) / 4095.0f;
}

/* ===== 映射模式 (0924BU 引入, 0924BV 默认回退 abs) =====
 * fi_map_hold=0 (默认): abs 绝对映射 —— 与 0924BT 实测行为一致
 *   (用户 2026-11-05 实测 BT 稳定跟上, hold 不要)。
 * fi_map_hold=1 (hold 增量跟随, `fi map 1` 切换; 参考 ROS arm_teleop):
 *   开启瞬间锁参考 fi_lead_ref(主臂逻辑角)/fi_foll_ref(从臂当前关节角),
 *   每帧目标 = foll_ref + (主臂当前 - lead_ref), wrap 到 (-180,180]。
 *   开启时目标==从臂当前位置 ⇒ 零跳变、不奔赴; 主从零点差被参考吸收。
 * `fi zero` 随时重锁参考 (把主臂摆到与从臂同姿态后敲)。 */
static uint8_t fi_map_hold    = 0U;
static uint8_t fi_ref_locked  = 0U;
static float   fi_lead_ref[6];
static float   fi_foll_ref[6];

/* ⭐0924CI: 主臂舵机度 -> 逻辑关节角换算 (原内联在 fi_read_master_jdeg,
 * 切片状态机每读成功 1 轴也要做同样换算, 提出来共用防两处漂移)。 */
static void fi_conv_master_jdeg(uint8_t j, float sd, float* out) {
    /* ⭐0924BH: 主臂零点独立反解 (abs 模式用; hold 模式下参考吸收
     * 零点差, 此换算结果只作增量基准, 不需要校准) */
    if (master_zero_raw[j] != 0U) {
        float moff = ((float)master_zero_raw[j] * 360.0f) / 4095.0f;
        float d = JOINT_DIR[j] * (sd - moff);
        d = fmodf(d + 180.0f, 360.0f);
        if (d < 0.0f) d += 360.0f;
        *out = d - 180.0f;               /* 归一到 (-180,180] */
    } else {
        *out = servo_to_joint_deg(sd, j);   /* 未校主臂: 沿用从臂零点 */
    }
}

/* 读主臂 6 轴 (Bus A ID 7~12) -> 逻辑关节角 jdeg; 全轴成功返 1。
 * 启动锁参考与每帧跟随共用 (0924BU, 原内联在 service 里)。
 * ⭐0924CI: 只剩 fi_lock_ref 用 (跟随热路径已改切片状态机)。 */
static uint8_t fi_read_master_jdeg(float jdeg[6]) {
    for (uint8_t j = 0; j < 6; j++) {
        float sd = -1.0f;
        /* ⭐2026-11-05 用户指令: 读失败立即重试, 最多 10 次 —— 60% 丢读下
         * 单轴成功率 84%→99.4%。代价: 载荷重时每帧 ~100-150ms; 上拉电阻
         * 修复总线后自动回到 40Hz。(锁参考是一次性动作, 阻塞无妨) */
        for (uint8_t t = 0; t < 10U; t++) {
            sd = fi_read_master_deg(j);
            if (sd >= 0.0f) break;
        }
        if (sd < 0.0f) return 0U;
        fi_conv_master_jdeg(j, sd, &jdeg[j]);
    }
    return 1U;
}

/* 锁定增量参考: 主臂当前逻辑角 + 从臂当前关节角 (0924BU)。 */
static void fi_lock_ref(void) {
    float mj[6], sv[6];
    uint8_t ok = 1U;
    if (!fi_read_master_jdeg(mj)) {
        cprint("  ⚠ 主臂读失败, 参考未锁 (再试)\r\n");
        fi_ref_locked = 0U;
        return;
    }
    for (uint8_t j = 0; j < 6; j++) {
        float d = SERVO_GetPosition(j);      /* 10 连试版, 启动时一次 */
        if (d < 0.0f) { ok = 0U; break; }
        sv[j] = d;
    }
    if (!ok) {
        cprint("  ⚠ 从臂读失败, 参考未锁 (再试)\r\n");
        fi_ref_locked = 0U;
        return;
    }
    for (uint8_t j = 0; j < 6; j++) {
        fi_lead_ref[j] = mj[j];
        fi_foll_ref[j] = sv[j];
    }
    fi_ref_locked = 1U;
}

/* ===== ⭐0924CI: fi 跟随帧切片状态机 (分时, A 需求单第 1/2 条) =====
 * 旧实现: 每 25ms 一帧, 一帧内连续做完 6 轴主臂读(每轴最多 10 连试,
 * 失败 1 次烧满 4ms 超时) + SyncWrite 下发 —— 丢读时单帧最长 ~240ms,
 * 主循环整段被占, GET_STATUS(网口轮询)只能排在帧后面 = p50 36.5ms/p95 68ms。
 *
 * 切片后: 每次调用只做【1 次总线事务】(读 1 轴的 1 次尝试), 立即 return
 * 把 CPU 让回主循环(网口/CAN/控制台轮询照常跑)。
 *
 * ⭐0924CJ: 部分帧降级下发 (卡顿根治) —— CI 版帧是"全有全无": 帧截止或
 * 单轴 10 试全败就弃整帧 ⇒ 丢读总线(实测 25~70% NO-RESP)上 40Hz 目标流
 * 断成突发, BT 误差驱动速度"err 大→spd 猛追→到位急停"往复 = 从臂一顿一顿。
 * 现在: 单轴 10 试全败 → 该轴沿用上次成功值, 继续下一轴; 帧截止 → 已读轴
 * 用本帧新值、未读轴沿用旧值, 照常下发。目标流保持恒定节拍(默认 50Hz),
 * BT 速度按真实 dt 自动补偿, 丢读只造成单轴单拍滞后, 不再弃帧。
 * 种子期(fi 1 后首轮 6 轴读齐前)仍全有全无 —— 保证首帧位姿完整不跳变。
 * 健康总线: 6 次成功尝试 ~几 ms 内连完; 丢读总线: GET_STATUS 最长只等
 * 1 次 4ms 读超时, 不受影响。fi_partial 计数 = 总线丢读健康度指标。 */
#define FI_FRAME_DEADLINE_MS  14U   /* 一帧从首读起的最长墙时间 (CJ: 超时改降级下发, 不再弃帧) */
#define FI_AXIS_MAX_TRIES     10U   /* 每轴最大尝试次数 (跨调用分摊) */
static uint8_t  fi_st       = 0U;   /* 0=idle 1=读主臂中 2=待下发 */
static uint8_t  fi_axis     = 0U;   /* 当前读的轴 0~5 */
static uint8_t  fi_tries    = 0U;   /* 当前轴已尝试次数 */
static uint32_t fi_frame_t0 = 0U;   /* 本帧起始时刻 */
/* ⭐0924CJ: fi_frame_jdeg 语义 = 每轴最近一次成功读到的主臂逻辑角
 * (跨帧保留), 丢读轴沿用旧值下发, 不再弃整帧。 */
static float    fi_frame_jdeg[6];
static uint8_t  fi_seed_done = 0U;  /* fi 1 后 6 轴至少各成功读一次才允许下发 */
static uint32_t fi_partial   = 0U;  /* 含丢读轴的降级帧计数 (总线健康度指标) */
static uint32_t fi_period_ms = FI_PERIOD_MS;  /* ⭐0924CI: `fi hz` 可调 (默认 20ms=50Hz, 0924CJ) */

/* 主循环每拍调用: fi 开启时按节拍读主臂并下发从臂 (切片版, 0924CI)。 */
void cmdFollowInternalService(void) {
    /* ---- fi 2 监视模式: 双臂角度只读显示, 不驱动任何舵机 ----
     * 主臂 = Bus A ID 7~12 (fi_read_master_deg, 10 连试, 舵机原始度);
     * 从臂 = 路由表 J1~J6 (SERVO_GetPosition, 内部已 10 连试, 关节度)。
     * 丢读显示 "--"。300ms 一行; 总线丢读 60% 时每行 ~0.5s, 上拉修复后变快。
     * (诊断用, 阻塞无妨; 不在遥操热路径上) */
    if (fi_mon) {
        if ((uint32_t)(HAL_GetTick() - fi_mon_last) < 300U) return;
        fi_mon_last = HAL_GetTick();
        cprint("  M:");
        for (uint8_t j = 0; j < 6; j++) {
            float sd = -1.0f;
            for (uint8_t t = 0; t < 10U; t++) {
                sd = fi_read_master_deg(j);
                if (sd >= 0.0f) break;
            }
            if (sd < 0.0f) cprint(" J%u=--", j + 1U);
            else           cprint(" J%u=%.1f", j + 1U, sd);
        }
        cprint("\r\n  S:");
        for (uint8_t j = 0; j < 6; j++) {
            float d = SERVO_GetPosition(j);
            if (d < 0.0f) cprint(" J%u=--", j + 1U);
            else          cprint(" J%u=%.1f", j + 1U, d);
        }
        cprint("\r\n");
        return;
    }
    if (!fi_en) return;

    if (fi_st == 0U) {
        /* 帧调度: 距上一帧起始 >= period 才起新帧 */
        if ((uint32_t)(HAL_GetTick() - fi_last_ms) < fi_period_ms) return;
        fi_last_ms = HAL_GetTick();
        fi_axis = 0U;
        fi_tries = 0U;
        fi_frame_t0 = HAL_GetTick();
        fi_st = 1U;
        /* 直接落进读状态, 本拍就做第 1 次尝试 */
    }
    if (fi_st == 1U) {
        /* 帧截止: ⭐0924CJ 改降级下发 —— 已读轴用本帧新值, 未读轴沿用
         * 上次成功值, 照常下发。目标流恒定节拍, 不再弃帧。
         * 种子期照旧弃帧: 首帧必须 6 轴读齐, 保证位姿完整不跳变。 */
        if ((uint32_t)(HAL_GetTick() - fi_frame_t0) > FI_FRAME_DEADLINE_MS) {
            if (!fi_seed_done) {
                fi_lost++;
                fi_st = 0U;
                return;
            }
            if (fi_axis < 6U) fi_partial++;
            fi_st = 2U;   /* 落到下方下发 */
        } else {
            /* 每次 1 次尝试: 成功 ~0.7ms, 失败最多烧满 FI_READ_TO_MS=4ms */
            float sd = fi_read_master_deg(fi_axis);
            if (sd >= 0.0f) {
                fi_conv_master_jdeg(fi_axis, sd, &fi_frame_jdeg[fi_axis]);
                fi_axis++;
                fi_tries = 0U;
                if (fi_axis >= 6U) {
                    fi_st = 2U;          /* 6 轴读齐 -> 下发 */
                    fi_seed_done = 1U;
                }
            } else {
                fi_tries++;
                if (fi_tries >= FI_AXIS_MAX_TRIES) {
                    if (fi_seed_done) {
                        fi_partial++;    /* 该轴 10 试全败: 沿用旧值, 跳下一轴 */
                        fi_axis++;
                        fi_tries = 0U;
                        if (fi_axis >= 6U) fi_st = 2U;
                    } else {
                        fi_lost++;       /* 种子期: 不弃轴, 弃帧重来 */
                        fi_st = 0U;
                        return;
                    }
                }
            }
            return;   /* ⭐每次调用只做 1 次事务, 让回主循环 (分时的核心) */
        }
    }
    /* fi_st == 2U: 下发 (SyncWrite ~1ms 且不等应答, 单次调用可完成)
     * ⭐0924CJ: 到这里时 fi_frame_jdeg = 本帧新值 + 丢读轴旧值的混合,
     * 目标流保持恒定节拍 —— 旧值轴该拍保持不动, 下一帧读到新值补上。 */
    fi_st = 0U;

    /* ⭐0924BU hold 增量映射: 目标 = 从臂参考 + (主臂当前 - 主臂参考)。
     * 开启时目标==从臂当前位置, 之后主臂动多少从臂跟多少 —— ROS 端
     * arm_teleop 包实测流畅低延迟的同款架构。 */
    if (fi_map_hold) {
        if (!fi_ref_locked) { fi_lock_ref(); return; }
        for (uint8_t j = 0; j < 6; j++) {
            float t = fi_foll_ref[j] + (fi_frame_jdeg[j] - fi_lead_ref[j]);
            t = fmodf(t + 180.0f, 360.0f);
            if (t < 0.0f) t += 360.0f;
            fi_frame_jdeg[j] = t - 180.0f;
        }
    }

    /* 喂从臂流控 (内部含防顶死限位钳制 + 软跟随 + RC 互斥)
     * ⭐0924BT: 旧代码把返回值==1(拒帧)当成功计入 fi_frames —— 数字反了。
     * 修正: 0=成功下发, 1=流控拒帧, 2=ESTOP, 3=主臂角色。 */
    {
        uint8_t sret = cmdStreamJoints(fi_frame_jdeg);
        if (sret == 0U)      fi_frames++;
        else if (sret == 1U) fi_rej++;
    }
}

static void cmdFollowInternal(uint8_t ntok, char** tokens) {
    /* fi diag —— 一屏快照: 力矩/主臂/目标/实际/误差/速度/统计 (0924BT)
     * "从臂不动"时先敲这个: 力矩 OFF ⇒ 帧到了也没执行; 目标被夹 ⇒
     * 主从零点/限位域错; err 大 spd 大而实际不动 ⇒ 执行侧(供电/舵机)。 */
    if (ntok >= 2 && strcmp(tokens[1], "diag") == 0) {
        cprint("  [FI-DIAG]\r\n  力矩 :");
        for (uint8_t j = 0; j < 6; j++) {
            uint8_t tq = SERVO_GetTorqueEnabled(j);
            cprint(" J%u=%s", (unsigned)j + 1U,
                   (tq == 1U) ? "ON" : (tq == 0U) ? "OFF!" : "?");
        }
        cprint("   (OFF!=该轴不动)\r\n  主臂 :");
        for (uint8_t j = 0; j < 6; j++) {
            float sd = -1.0f;
            for (uint8_t t = 0; t < 10U; t++) {
                sd = fi_read_master_deg(j);
                if (sd >= 0.0f) break;
            }
            if (sd < 0.0f) cprint(" J%u=--", (unsigned)j + 1U);
            else           cprint(" J%u=%.1f", (unsigned)j + 1U, (double)sd);
        }
        cprint("   (舵机度)\r\n  目标 :");
        if (fi_dbg_valid) {
            for (uint8_t j = 0; j < 6; j++) {
                float td = servo_to_joint_deg(
                    servo_raw_to_deg(fi_dbg_tgt_raw[j]), j);
                cprint(" J%u=%.1f", (unsigned)j + 1U, (double)td);
            }
        } else {
            cprint(" --  (遥操还没跑过帧)");
        }
        cprint("\r\n  实际 :");
        for (uint8_t j = 0; j < 6; j++) {
            float d = SERVO_GetPosition(j);
            if (d < 0.0f) cprint(" J%u=--", (unsigned)j + 1U);
            else          cprint(" J%u=%.1f", (unsigned)j + 1U, (double)d);
        }
        cprint("   (从臂关节度)\r\n  err/spd:");
        if (fi_dbg_valid) {
            for (uint8_t j = 0; j < 6; j++) {
                cprint(" J%u=%u/%u", (unsigned)j + 1U,
                       (unsigned)fi_dbg_err[j], (unsigned)fi_dbg_spd[j]);
            }
            cprint("  dt=%lums", (unsigned long)fi_dbg_dt_ms);
        } else {
            cprint(" --");
        }
        cprint("\r\n  统计 : frames=%lu rej=%lu lost=%lu partial=%lu (CJ 降级帧)",
               (unsigned long)fi_frames, (unsigned long)fi_rej,
               (unsigned long)fi_lost, (unsigned long)fi_partial);
        cprint("  GET_STATUS缓存: hit=%lu miss=%lu (遥测采样 %ums/轴)",
               (unsigned long)g_stcache_hit, (unsigned long)g_stcache_miss,
               (unsigned)TELEM_SAMPLE_PERIOD_MS);
        cprint(" clamp[frames=%lu cnt:",
               (unsigned long)stream_clamp_frames);
        for (uint8_t j = 0; j < 6; j++)
            cprint(" %lu", (unsigned long)stream_clamp_cnt[j]);
        cprint("]\r\n  零点 : master raw");
        for (uint8_t j = 0; j < 6; j++)
            cprint(" J%u=%u", (unsigned)j + 1U, (unsigned)master_zero_raw[j]);
        cprint("  (0=未校, 用从臂 SERVO_OFFSET)\r\n");
        return;
    }
    /* fi map 0|1 —— 映射模式 (0924BU): 1=hold 增量(默认, ROS 同款) 0=abs 绝对 */
    if (ntok >= 2 && strcmp(tokens[1], "map") == 0) {
        if (ntok >= 3) {
            uint8_t v = (uint8_t)atoi(tokens[2]);
            if (v > 1U) {
                cprint("  Usage: fi map 0|1  (0=abs 绝对, 1=hold 增量跟随(默认))\r\n");
                return;
            }
            fi_map_hold = v;
            fi_ref_locked = 0U;   /* 换模式必须重锁 */
        }
        cprint("  映射模式 = %s\r\n", fi_map_hold
               ? "hold 增量跟随 (目标=从臂参考+主臂位移, 开启零跳变)"
               : "abs 绝对映射 (目标=主臂绝对姿态, 依赖 master 零点)");
        cprint("  hold 模式: `fi 1` 开启即锁参考; 把主臂摆到与从臂同姿态后 `fi zero` 重锁。\r\n");
        return;
    }
    /* fi zero —— 手动重锁增量参考 (0924BU) */
    if (ntok >= 2 && strcmp(tokens[1], "zero") == 0) {
        if (!fi_en) {
            cprint("  fi 未开启 (先 fi 1)\r\n");
            return;
        }
        if (!fi_map_hold) {
            cprint("  abs 模式不使用参考 (fi map 1 切 hold)\r\n");
            return;
        }
        fi_lock_ref();
        if (fi_ref_locked)
            cprint("  参考已重锁: 当下起从臂跟随主臂位移量 (从臂姿态不变)\r\n");
        return;
    }
    /* fi acc [0-254] —— 跟随轴加速度 (0924BS) */
    if (ntok >= 2 && strcmp(tokens[1], "acc") == 0) {
        if (ntok >= 3) {
            int a = atoi(tokens[2]);
            if (a < 0 || a > 254) {
                cprint("  ✗ acc 取值 0~254 (STS addr41, 100 step/s^2 每单位)\r\n");
                return;
            }
            fi_follower_accel = (uint8_t)a;
        }
        cprint("  跟随轴加速度 = %u  (RAM, 重启回默认 254)\r\n", (unsigned)fi_follower_accel);
        cprint("  `fi 1` 开启时写入 6 个跟随轴 addr41; `i` 重新初始化会写回全局值 50。\r\n");
        return;
    }
    /* fi spd 0|1 —— 比例跟随开关 (0924BS) */
    if (ntok >= 2 && strcmp(tokens[1], "spd") == 0) {
        if (ntok >= 3) {
            uint8_t v = (uint8_t)atoi(tokens[2]);
            if (v > 1U) {
                cprint("  Usage: fi spd 0|1  (0=固定速度(旧), 1=比例跟随(默认))\r\n");
                return;
            }
            stream_prop_speed = v;
        }
        cprint("  跟随速度模式 = %s\r\n", stream_prop_speed ? "比例跟随 (速度∝主臂, 0924BS)"
                                                          : "固定速度 (i 初始化写入的 GoalSpeed)");
        cprint("  比例跟随: 速度 = 本帧走距÷时间 ×1.25, 与位置同一帧 SyncWrite 下发;\r\n");
        cprint("            下限 150 (0=全速!), 上限 4095 = 舵机硬件满速, 不软件限速。\r\n");
        cprint("  直通模式(fi 1 自动开): 跳变/位姿/慢速阈值全旁路, 只留硬限位 (0924BS)。\r\n");
        return;
    }
    /* ⭐0924BZ: `fi margin` 已删除 (用户明令"把安全带删掉") —— 遥操只受限位
     * 约束。老 flash 区 F 里的 margin 字段不再读用, 存盘固定写 UNSET。 */
    /* fi hz [5~80] —— 跟随节拍 (0924CI 引入; ⭐0924CJ 默认 25->20ms=50Hz)。
     * 部分帧降级下发后目标流连续, 提速只降延迟不伤流畅; GET_STATUS 走
     * 遥测缓存不占总线, 节拍提速不影响状态查询帧率。 */
    if (ntok >= 2 && strcmp(tokens[1], "hz") == 0) {
        if (ntok >= 3) {
            int hz = atoi(tokens[2]);
            if (hz < 5 || hz > 80) {
                cprint("  Usage: fi hz <5~80>  (当前 %ums = %lu Hz)\r\n",
                       (unsigned)fi_period_ms,
                       (unsigned long)(1000U / fi_period_ms));
                return;
            }
            fi_period_ms = (uint32_t)(1000U / (uint32_t)hz);
            cprint("  fi 跟随节拍 -> %ums (%d Hz)  (RAM, 重启回默认 20ms=50Hz)\r\n",
                   (unsigned)fi_period_ms, hz);
        } else {
            cprint("  fi 跟随节拍 = %ums (%lu Hz)  用法: fi hz <5~80>\r\n",
                   (unsigned)fi_period_ms,
                   (unsigned long)(1000U / fi_period_ms));
        }
        return;
    }
    if (ntok >= 2) {
        uint8_t v = (uint8_t)atoi(tokens[1]);
        if (v > 2U) {
            cprint("  Usage: fi 0|1|2  (0=off, 1=follow, 2=双臂角度监视 只读)\r\n");
            return;
        }
        if (v == 0U && fi_en == 0U && fi_mon == 0U) {
            cprint("  fi already OFF\r\n");
            return;
        }
        if (v == 2U && fi_mon) {
            cprint("  fi monitor already ON\r\n");
            return;
        }
        /* 模式互斥: 先全关 */
        fi_en  = 0U;
        fi_mon = 0U;
        fi_st  = 0U;              /* ⭐0924CI: 切片状态机一并复位 */
        fi_axis = 0U;             /* ⭐0924CJ: 种子/降级状态一并复位 */
        fi_tries = 0U;
        fi_seed_done = 0U;        /* 重新等种子帧 (首轮 6 轴读齐才下发) */
        fi_ref_locked = 0U;       /* ⭐0924BU: 参考随模式复位 */
        stream_free_gates = 0U;   /* ⭐0924BS: 闸门随模式复位 */
        if (stream_active) cmdStreamExit("fi mode switch");
        if (v == 1U) {
            fi_en = 1U;
            fi_last_ms = 0U;   /* 立刻起第一拍 */
            stream_prop_speed = 1U;   /* ⭐0924BS: 遥操默认比例跟随 */
            stream_free_gates = 1U;   /* ⭐0924BS: 遥操直通 — 只留硬限位 */
            /* ⭐0924BQ: 跟随轴加速度写满 (addr41), 否则默认 50 的加速度曲线
             * 会在主臂快加速时把从臂拖出恒定滞后 ("跟不上"的另一半根因)。
             * ⭐0924BS: 每轴 10 试 —— 3 试时丢读 60% 下每轴 ~22% 概率没写进,
             * 失败轴的加速度曲线拖整条臂; 10 试后失败率降到 0.6%。
             * 失败轴下帧速度照传, 只是加速能力弱一些, 无安全影响。 */
            for (uint8_t j = 0; j < 6; j++) {
                for (uint8_t t = 0; t < 10U; t++) {
                    if (SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                          STS_ADDR_ACCELERATION,
                                          &fi_follower_accel, 1U) == HAL_OK) break;
                }
            }
            /* ⭐0924BT 力矩校验: 烧录/RESET 后六轴力矩全 OFF, 且 DXL_Write
             * 对超时按 "Return Level=1 不回包" 返回 OK —— 写丢了也报成功。
             * stream_enter 的 SetTorque 单发无回读, 一旦丢失: 指令帧每帧
             * "成功发出"而舵机不执行, 从臂纹丝不动 (2026-11-05 现场实测:
             * fi 2 的 S 行 40 行冻结 0 抖动)。这里读回逐轴校验, OFF 的
             * 补写 3 轮, 仍 OFF 直接警告 —— 一眼分辨"帧没到"还是"别的"。
             * 正常情况(力矩已 ON)只花 6 次短读, 不拖启动。 */
            for (uint8_t j = 0; j < 6; j++) {
                uint8_t tq = SERVO_GetTorqueEnabled(j);
                for (uint8_t t = 0; t < 3U && tq != 1U; t++) {
                    SERVO_SetTorque(j, 1U);
                    tq = SERVO_GetTorqueEnabled(j);
                }
                if (tq != 1U) {
                    cprint("       ⚠ J%u 力矩 OFF 补写失败 — 从臂该轴不会动, 查总线!\r\n",
                           (unsigned)j + 1U);
                }
            }
            /* ⭐0924BU: hold 增量映射下启动即锁参考 —— 目标=从臂当前位置,
             * 开启瞬间零跳变 (ROS arm_teleop 同款行为)。 */
            if (fi_map_hold) fi_lock_ref();
            cprint("  [FI] 板内遥操 ON: 读 Bus A 主臂 ID %u~%u -> 下发 Bus B 从臂\r\n",
                   (unsigned)FI_INPUT_ID0, (unsigned)(FI_INPUT_ID0 + 5U));
            cprint("       映射: %s (0924BU)\r\n",
                   fi_map_hold ? "hold 增量跟随 — 开启不跳变, 主臂动多少从臂跟多少"
                               : "abs 绝对映射 — 依赖 master 零点校准 (`fi map 1` 切回)");
            cprint("       前提: 主臂舵机已改 ID %u~%u 并接 Bus A\r\n",
                   (unsigned)FI_INPUT_ID0, (unsigned)(FI_INPUT_ID0 + 5U));
            cprint("       速度: 比例跟随无上限 (主臂多快从臂多快; `fi spd 0` 切回)\r\n");
            cprint("       加速度: 跟随轴 addr41=%u 已写 (`fi acc` 查看/修改)\r\n",
                   (unsigned)fi_follower_accel);
            cprint("       闸门: 直通 (跳变/位姿/慢速阈值全旁路, 只留硬限位, 0924BS)\r\n");
            cprint("       桌面: %s (下限模型z %.1f; `zf here` 标定, `zf on|off` 开关)\r\n",
                   geo_zf_on ? "拦" : "不拦",
                   (double)((geo_zfloor_mm >= 0.0f) ? geo_zfloor_mm
                            : (traj_z_floor_mm + WORLD_OFFSET_Z)));
            cprint("       %s\r\n",
                   stream_follow_en ? "(sf=1, follow OK)"
                                    : "⚠ sf=0 跟随被暂停, 先 `sf 1`");
        } else if (v == 2U) {
            fi_mon = 1U;
            fi_mon_last = 0U;
            cprint("  [FI-MON] 双臂角度监视 ON (300ms/行, 只读不动): \r\n");
            cprint("       M = 主臂 Bus A ID 7~12 (舵机原始度)\r\n");
            cprint("       S = 从臂 Bus B J1~J6 (关节度)\r\n");
            cprint("       停: fi 0\r\n");
        } else {
            cprint("  [FI] OFF\r\n");
        }
        return;
    }
    /* 查询 */
    cprint("  [FI] 板内遥操: %s   监视: %s   速度: %s   闸门: %s   映射: %s%s\r\n",
           fi_en ? "ON" : "OFF", fi_mon ? "ON" : "OFF",
           stream_prop_speed ? "比例" : "固定",
           stream_free_gates ? "直通(只限位)" : "全闸门",
           fi_map_hold ? "hold增量" : "abs绝对",
           fi_ref_locked ? "/已锁" : "");
    cprint("       frames=%lu rej=%lu lost=%lu partial=%lu (降级帧=丢读健康度)\r\n",
           (unsigned long)fi_frames, (unsigned long)fi_rej,
           (unsigned long)fi_lost, (unsigned long)fi_partial);
    cprint("       节拍=%lums  GET_STATUS缓存 hit=%lu miss=%lu (0924CJ: 部分帧降级+50Hz)\r\n",
           (unsigned long)fi_period_ms,
           (unsigned long)g_stcache_hit, (unsigned long)g_stcache_miss);
}

/* 带诊断打印的包装: 调用前清空, 失败后输出原因。 */
static uint8_t check_path(const float a[6], const float b[6]) {
    last_fk_path_fail[0] = '\0';
    uint8_t ok = fk_path_z_ok(a, b);
    if (!ok) {
        cprint("    path check fail: %s\r\n", last_fk_path_fail);
    }
    return ok;
}

/* 逐段校验一条链: from -> pts[0] -> ... -> pts[n-1] */
static uint8_t safe_chain_ok(const float from[6], const float pts[][6],
                             uint8_t n) {
    float prev[6];
    memcpy(prev, from, sizeof(float) * 6);
    for (uint8_t i = 0; i < n; i++) {
        if (!check_path(prev, pts[i])) return 0;
        memcpy(prev, pts[i], sizeof(float) * 6);
    }
    return 1;
}

/* 构造并校验 cur -> tgt 的安全路点序列。
 * 成功: 填充 safe_seq_wp/n 并返回 1; 失败: 打印原因返回 0 (不产生任何运动)。
 * 2026-09-02 触地事故重构: 路点全部改为"相对 Home 零点的关节角"定义
 * (joint_to_servo_deg 换算), 高举位 RISE_*_DEG 恒在 J3 合法弧内;
 * 折叠判定用 servo_to_joint_deg 关节角, 不再用随 zero 漂移的舵机度阈值。
 * 每一段仍经 fk_path_z_ok 逐点 FK 采样校验 (桌面 + 立柱圆柱含后方 + raw 限位)。 */
static uint8_t safe_seq_plan(const float cur[6], const float tgt[6],
                             float tx, float ty, float tz) {
    float rise[6];
    for (uint8_t j = 0; j < 6; j++) rise[j] = cur[j];
    rise[0] = tgt[0];
    rise[1] = joint_to_servo_deg(RISE_J2_DEG, 1);
    rise[2] = joint_to_servo_deg(RISE_J3_DEG, 2);
    rise[3] = joint_to_servo_deg(RISE_J4_DEG, 3);
    clamp_all_angles(rise);   /* 确保高举位在限位内 */

    safe_seq_n = 0;
    safe_seq_idx = 0;

    /* 1) 直达路径校验通过 -> 单段 */
    if (check_path(cur, tgt)) {
        memcpy(safe_seq_wp[0], tgt, sizeof(float) * 6);
        safe_seq_n = 1;
        cprint("  Safe path: direct (FK z-check OK).\r\n");
        return 1;
    }

    /* 2) 两段经高举: cur -> rise(工具竖直朝上) -> tgt。
     * Python 验证: 折叠位->高举 单段路径爪尖最低 z≈65mm, 高举->典型工作目标
     * 路径爪尖最低 z≈325mm, 均安全。 */
    if (fk_path_z_ok(cur, rise) && fk_path_z_ok(rise, tgt)) {
        memcpy(safe_seq_wp[0], rise, sizeof(float) * 6);
        memcpy(safe_seq_wp[1], tgt, sizeof(float) * 6);
        safe_seq_n = 2;
        cprint("  Safe path: via rise (tool-up, z~486), 2 segments.\r\n");
        return 1;
    }

    /* 3) 折叠展开链: 先抬肩(保持肘/腕构型) -> 高举 -> 目标。
     * 深折叠位直接展肘会把爪尖扫向桌面/立柱, 抬肩先行可抬高整条路径。 */
    {
        float lift[6];
        memcpy(lift, cur, sizeof(float) * 6);
        lift[1] = joint_to_servo_deg(RISE_J2_DEG, 1);
        clamp_all_angles(lift);
        float c1[3][6];
        memcpy(c1[0], lift, sizeof(float) * 6);
        memcpy(c1[1], rise, sizeof(float) * 6);
        memcpy(c1[2], tgt, sizeof(float) * 6);
        if (safe_chain_ok(cur, c1, 3)) {
            for (uint8_t i = 0; i < 3; i++)
                memcpy(safe_seq_wp[i], c1[i], sizeof(float) * 6);
            safe_seq_n = 3;
            cprint("  Safe path: lift-shoulder->rise->target, 3 segments.\r\n");
            return 1;
        }
    }

    /* 4) 折叠收拢链 (目标是深折叠): 高举 -> 半折叠(工具保持朝上) ->
     *    肘/腕到目标构型(肩仍高举) -> 降肩到目标。
     * 折叠判定用关节角 (servo_to_joint_deg), 旧 'tgt[2]<60 舵机度' 判据
     * 在 Phase1 校准 (zero 漂移 ~100°) 后完全失真。 */
    {
        float t_j3 = servo_to_joint_deg(tgt[2], 2);
        if (t_j3 < -60.0f) {
            float mid1[6], mid2[6];
            memcpy(mid1, tgt, sizeof(float) * 6);
            mid1[1] = joint_to_servo_deg(RISE_J2_DEG, 1);
            mid1[2] = joint_to_servo_deg(t_j3 * 0.5f, 2);   /* 半折叠 */
            mid1[3] = joint_to_servo_deg(RISE_J4_DEG, 3);   /* 工具仍朝上 */
            memcpy(mid2, tgt, sizeof(float) * 6);
            mid2[1] = joint_to_servo_deg(RISE_J2_DEG, 1);   /* 肩保持高举 */
            clamp_all_angles(mid1);
            clamp_all_angles(mid2);
            float c2[4][6];
            memcpy(c2[0], rise, sizeof(float) * 6);
            memcpy(c2[1], mid1, sizeof(float) * 6);
            memcpy(c2[2], mid2, sizeof(float) * 6);
            memcpy(c2[3], tgt, sizeof(float) * 6);
            if (safe_chain_ok(cur, c2, 4)) {
                for (uint8_t i = 0; i < 4; i++)
                    memcpy(safe_seq_wp[i], c2[i], sizeof(float) * 6);
                safe_seq_n = 4;
                cprint("  Safe path: rise->half-fold->shoulder-high->target, 4 segments.\r\n");
                return 1;
            }
        }
    }

    /* 5) 悬停点方案: 先到 (目标方位, x/y 减半, z=260, 工具水平) 的高处,
     *    再垂直下探 (同 xy 直落, 路径天然安全)。tx/ty/tz 无效(全0)时跳过。 */
    if (tx != 0.0f || ty != 0.0f) {
        Kinematics_Pos h;
        float hrad[6];
        h.x = tx * 0.5f; h.y = ty * 0.5f; h.z = 260.0f;
        h.yaw = 90.0f * 3.14159265f / 180.0f;
        float q_prev_hover[6];
        for (uint8_t j = 0; j < 6; j++) q_prev_hover[j] = servo_deg_to_rad(cur[j], j);
        if (IK_Solve(&h, hrad)) {
            float hover[6];
            for (uint8_t j = 0; j < 6; j++)
                hover[j] = rad_to_servo_deg(hrad[j], j);
            hover[4] = cur[4]; hover[5] = cur[5];   /* J5/J6 保持 */
            float c3[3][6];
            memcpy(c3[0], hover, sizeof(float) * 6);
            memcpy(c3[1], rise, sizeof(float) * 6);
            memcpy(c3[2], tgt, sizeof(float) * 6);
            if (safe_chain_ok(cur, c3, 3)) {
                for (uint8_t i = 0; i < 3; i++)
                    memcpy(safe_seq_wp[i], c3[i], sizeof(float) * 6);
                safe_seq_n = 3;
                cprint("  Safe path: hover(%.0f,%.0f,260)->rise->target, 3 segments.\r\n",
                       h.x, h.y);
                return 1;
            }
        }
    }

    cprint("  Safe path: NO safe segment plan (all candidates failed FK check).\r\n");
    cprint("  Move cancelled. Suggest 'P 2' (READY), or manual 'a J' moves.\r\n");
    return 0;
}

/* 起点软拉回容差: 当前位置因校准/反馈噪声略微越界时, 只拉到最近边界,
 * 避免 1 raw 误差导致所有路径规划失败。 */
#define START_SOFT_CLAMP_TOL_RAW  5

/* ===== 起点姿态读取 + 坏读校验 (2026-09-10) =====
 * 背景 A (本 bug 的真正根因): 旧版读失败时用【目标值】兜底(fallback=tgt),
 *   语义是"读不到就假装它已经在目标位"。若某轴偶发丢帧, 两次读都失败 ->
 *   两次都回退成同一个目标值 -> 两次"一致" -> 校验通过 -> 该轴用的是目标值,
 *   与其余轴的实际值拼成一个物理上不存在的"混合姿态" -> FK 算出假越界。
 *   实测: 臂停在折叠位(J2 servo 77), 发 'g 260 0 280 90'(目标 J2=135),
 *   J2 两次丢帧均回退成目标 135 -> 姿态 = J2 伸展 + J3/J4 折叠
 *   -> FK 得 TCP z=-23mm < 8mm -> 首次必被拒("path check fail ... at i=0"),
 *   重发一次即过。z=-23.2 与固件打印的 -23 完全吻合。
 *   对策: **绝不用目标值兜底**。读不到就重试, 重试仍失败则拒绝运动。
 * 背景 B: 半双工 74HC126 总线偶发把残帧/噪声当响应(帧头/ID/校验和合法但值
 *   陈旧), 单帧无法识别 -> 用"多次采样多数表决"过滤。
 * 策略: 每轴采样 START_READ_SAMPLES 次(每次读失败再原地重试 START_READ_TRIES
 *   次), 取 ≥2 个互相接近(<= TOL)的样本作多数; 任一轴无多数 -> 整组重试,
 *   最多 START_READ_ATTEMPTS 轮; 仍失败 -> 拒绝运动(安全优先: 坏读既会
 *   "假越界"误拒, 也可能让危险路径"假安全")。
 * 正常情况(总线干净)每轴前 2 个采样即达成多数, 读数量与旧版相当。 */
#define START_READ_TOL_DEG   4.0f
#define START_READ_SAMPLES   3      /* 每轴采样次数(多数表决) */
#define START_READ_TRIES     2      /* 单次采样读失败时的原地重试次数 */
#define START_READ_ATTEMPTS  3      /* 整组重试轮数 */

/* 最近一次"读不到"的原因 (SERVO_GetPositionDiag 输出), 供报错信息使用。
 * 三者现场处置完全不同: 无应答查线缆, 帧损坏查总线噪声, 告警查负载/散热。 */
static uint8_t pose_read_diag = 0;   /* DXL_RD_* */
static uint8_t pose_read_err  = 0;   /* 状态帧 Error 字节原值 */

/* ===== 2026-09-17: 带超时的版本 =====
 * 给"轨迹结束后的等待期"里的高频判定用(多点到位判定)。旧版本固定 100ms 超时,
 * 一轴读不到就要烧 3 次 × (100ms + 2ms) = 300ms 以上, 主循环(控制台/遥测/
 * E-STOP)会被整段拖住。热路径统一 10ms —— 与 SERVO_GetPositionRawTO /
 * SERVO_ReadTelemetry 一致, 单次失败只损失一个节拍。
 * (原 read_pos_retry() 固定 100ms 的裸版本已无调用者, 2026-09-17 删除:
 *  所有读位置入口现在都走 read_pos_vote()/_to(), 统一有多数表决。) */
static float read_pos_retry_to(uint8_t j, uint32_t to_ms) {
    float v = -1.0f;
    for (uint8_t t = 0; t < START_READ_TRIES; t++) {
        uint8_t dg = DXL_RD_OK, eb = 0;
        v = SERVO_GetPositionDiagTO(j, to_ms, &dg, &eb);
        if (v >= 0.0f) return v;
        pose_read_diag = dg;
        pose_read_err  = eb;
        HAL_Delay(2);
    }
    return -1.0f;
}

/* 单轴读取 + 多数表决。逐个采样, 一旦出现 2 个互相接近的样本即定稿
 * (正常总线每轴只需 2 次读)。
 * 返回: 1 = 成功(值写入 *out)  /  0 = 读不到(有效样本 <2)  /  -1 = 帧跳变(无多数) */
static int8_t read_pos_vote_to(uint8_t j, float *out, uint32_t to_ms) {
    float s[START_READ_SAMPLES];
    uint8_t n = 0;
    for (uint8_t k = 0; k < START_READ_SAMPLES; k++) {
        float v = read_pos_retry_to(j, to_ms);
        if (v < 0.0f) continue;
        for (uint8_t i = 0; i < n; i++) {
            float d = s[i] - v;
            if (d < 0.0f) d = -d;
            if (d <= START_READ_TOL_DEG) { *out = v; return 1; }   /* 多数达成 */
        }
        s[n++] = v;
        if (k + 1U < START_READ_SAMPLES) HAL_Delay(1);
    }
    return (n < 2U) ? 0 : -1;
}

static int8_t read_pos_vote(uint8_t j, float *out) {
    return read_pos_vote_to(j, out, 100U);
}

/* 打印"某轴读不到"的**具体原因** (2026-09-11)。
 * 旧版只有一句 "Jx read error", 三种故障长得一模一样:
 *   r == 0 且 pose_read_diag == NO_REPLY   -> 线缆/接插/共地/供电
 *   r == 0 且 pose_read_diag == BAD_FRAME  -> 总线噪声/丢字节 (1Mbps 下极易)
 *   r == 0 且 pose_read_diag == ERROR_BITS -> 舵机告警 (过载/过温/过压)
 *   r == -1                                -> 两次采样不一致 (帧跳变/抖动)
 * tag 用来区分是哪个入口 (FK / Start pose)。 */
static void print_pose_read_fail(const char* tag, uint8_t j, int8_t r) {
    if (r == -1) {
        cprint("  %s UNSTABLE: J%d two samples differ > %d deg (bus noise).\r\n",
               tag, (unsigned)j + 1U, (int)START_READ_TOL_DEG);
    } else if (pose_read_diag == DXL_RD_ERROR_BITS) {
        cprint("  %s J%d READ FAILED: servo ALERT err=0x%02X [%s] ",
               tag, (unsigned)j + 1U, (unsigned)pose_read_err,
               SERVO_ErrorString(pose_read_err));
        cprint("(check load/temperature, not wiring)\r\n");
    } else if (pose_read_diag == DXL_RD_BAD_FRAME) {
        cprint("  %s J%d READ FAILED: corrupt frame (bus noise/dropped byte).\r\n",
               tag, (unsigned)j + 1U);
    } else {
        cprint("  %s J%d READ FAILED: no reply (cable/connector/ground).\r\n",
               tag, (unsigned)j + 1U);
    }
}

/* 读失败原因 -> 一行人话 (2026-09-17 加, 给多点到位判定的逐轴报告复用)。 */
static const char* pose_read_diag_str(uint8_t dg) {
    if (dg == DXL_RD_ERROR_BITS) return "servo ALERT (overload/over-temp/voltage)";
    if (dg == DXL_RD_BAD_FRAME)  return "corrupt frame (bus noise/dropped byte)";
    return "no reply (cable/connector/ground)";
}

/* 打印一个带符号、一位小数的角度值 (精简版 newlib 没有 %f) */
static void cprint_deg1(float v) {
    char sign = (v < 0.0f) ? '-' : '+';
    if (v < 0.0f) v = -v;
    uint32_t s = (uint32_t)(v * 10.0f + 0.5f);
    cprint("%c%lu.%lu", sign, (unsigned long)(s / 10U), (unsigned long)(s % 10U));
}

/* 打印 6 轴"相对各自 Home 的逻辑关节角" —— 与主臂 0x13 帧里传的量完全同一语义,
 * 也就是从臂 `p J` 里的 `rel-home`。
 * 两块板各敲一次 `ms`、把臂摆成同一物理姿态, 两组数字应逐轴吻合 ——
 * 这就是主/从校准一致性最直接的判据 (不一致 = 某一端的 zero 没测对)。 */
static void print_joint_angles(const char* tag) {
    cprint("  %s joint angles (rel-home, the quantity the 0x13 frame carries):\r\n",
           tag);
    for (uint8_t j = 0; j < 6; j++) {
        float v;
        int8_t r = read_pos_vote(j, &v);
        if (r != 1) {
            cprint("    J%u: READ FAIL\r\n", (unsigned)j + 1U);
        } else {
            cprint("    J%u: %+.1f deg\r\n",
                   (unsigned)j + 1U, (double)servo_to_joint_deg(v, j));
        }
    }
}

#if ARM_MASTER_BUILD
/* ============================================================
 * 主臂角色 (2026-09-11, `make ROLE=master` 构建)
 *
 * 主臂只做两件事: 读本臂六轴位置 -> 换算成"相对本臂 Home 的逻辑关节角"
 * -> 组 0x13 STREAM_JOINTS 帧经 UART4 (PA0/PA1) 打给从臂。
 *
 * 为什么不传舵机 raw / 舵机度: 两端 SERVO_OFFSET(校准零点) 是各自测的,
 * 传舵机度必然错位。逻辑角是"相对各自 Home"的量, 两端语义同一, 从臂侧
 * ArmApi_StreamJoints() 会用自己的校准反算回 raw。
 *
 * 不跑轨迹/IK, 不写 Flash; 六轴 Torque OFF -> 人力可自由拖动。
 * ============================================================ */
#define MASTER_TX_PERIOD_MS   33U    /* 33ms ≈ 30Hz (20=50Hz, 50=20Hz) */
#define MASTER_FAIL_QUIET     30U    /* 连续读失败多少次才打印一次, 防刷屏 */

static uint8_t  master_seq      = 0;
static uint32_t master_next_tx  = 0;
static uint32_t master_tx_ok    = 0;
static uint32_t master_tx_skip  = 0;   /* 因读失败而丢弃的帧数 */
static uint8_t  master_fail_run = 0;
static uint8_t  master_fwd_en   = 1U;  /* `mf 0` 暂停转发 (上电默认发) */

/* ===== 控制台位置流 (`sp <Hz>`, 2026-09-15): 官网"PC 中转"路线的输出端 =====
 * 官网 LeRobot 的做法是主臂经 USB 串口把位置交给电脑、电脑再写给从臂
 * (两臂之间一根线都没有)。我们这里就是那个"主臂交给电脑"的半边:
 * 按节拍往控制台吐一行 `#P <6 个关节角×10>`, PC 侧 tools/pc_relay.py
 * 原样转给从臂; 从臂认同一行 ⇒ PC 脚本退化成一根纯转发线。
 * 与 `mf`(0x13 直连转发) 完全独立, 两者可同时开, 也可只开一个。 */
static uint32_t master_sp_period_ms = 0U;   /* 0 = 停 */
static uint32_t master_sp_next      = 0U;
static uint32_t master_sp_lines     = 0U;   /* 已吐行数 */
static uint32_t master_sp_skip      = 0U;   /* 因读失败被抑制的拍数 */

static void master_release_all(void);   /* 前向声明 (定义在本块末尾) */

#if ARM_MASTER_BUILD
/* ===== `ml [N]` 自环自检 (主臂专用) =====
 * 现场卡在"NE≈100% 到底是地弹, 还是线压根没接上"。本命令把【发帧】与
 * 【读舵机】彻底解耦: 不看任何舵机, 直接按 33 ms 发 N 帧标准 0x13 帧,
 * 于是**舵机整机断电时 PA0 照样有输出** —— 才能真正分离噪声源。
 *
 * 用法: 主臂把 PA0 短接到 PA1 (自环), 敲 `ml` (默认 200 帧 ≈ 6.6 s), 结束后
 * 自动给判决。**判据用 CRC 正确的回环帧数 (stream frames), 不用字节数** ——
 * 字节数会被浮空脚/环境噪声蒙混过去, CRC 不会。 */
static uint16_t master_st_n        = 0U;
static uint16_t master_st_total    = 0U;
static uint32_t master_st_exp_b    = 0U;   /* 实际发到线上的字节数 */
static uint32_t master_st_snap_b   = 0U;
static uint32_t master_st_snap_ne  = 0U;
static uint32_t master_st_snap_fe  = 0U;
static uint32_t master_st_snap_stm = 0U;
static uint32_t master_st_snap_tx  = 0U;
static uint32_t master_st_quiet_at = 0U;
static uint8_t  master_st_phase    = 0U;   /* 0=空闲 1=发帧中 2=等接收静默 */

static void master_selftest_report(void) {
    LinkDiag d;
    LinkProto_GetDiag(&d);
    uint32_t db   = d.bytes_rx - master_st_snap_b;
    uint32_t dne  = d.ne       - master_st_snap_ne;
    uint32_t dfe  = d.fe       - master_st_snap_fe;
    uint32_t dstr = LinkProto_StreamCount() - master_st_snap_stm;
    uint32_t dtx  = master_tx_ok - master_st_snap_tx;
    uint32_t exp  = master_st_exp_b;

    cprint("\r\n  ===== ml self-test result =====\r\n");
    cprint("  expected on wire : %u frames = %lu bytes\r\n",
           (unsigned)master_st_total, (unsigned long)exp);
    cprint("  actually sent    : %lu frames (HAL_UART_Transmit OK)\r\n",
           (unsigned long)dtx);
    cprint("  rx bytes delta   : %lu  (%lu%% of expected)\r\n",
           (unsigned long)db, (unsigned long)(exp ? (db * 100UL) / exp : 0UL));
    cprint("  rx NE / FE delta : %lu / %lu\r\n",
           (unsigned long)dne, (unsigned long)dfe);
    cprint("  CRC-OK loopback  : %lu frames\r\n", (unsigned long)dstr);

    if (master_st_total != 0U && dstr >= (uint32_t)master_st_total * 8UL / 10UL) {
        cprint("  -> [A 通路完好] 80%% 以上的帧原样回来了, CRC 全对。\r\n");
        cprint("     即: PA0/PA1 与 115200 都没问题 —— 故障在【到从臂那一段】。\r\n");
    } else if (db < exp / 5UL) {
        cprint("  -> [B 断路] 收到的字节不到发出的 20%%: PA1 基本没收到 PA0 的东西。\r\n");
        cprint("     查: 自环线/杜邦头没插实、脚位错、或 PA1 被别的东西占住。\r\n");
    } else if (db > exp * 3UL / 2UL) {
        cprint("  -> [C 额外流量] 收到的字节远超发出的: PA1 上有【独立信号源】。\r\n");
        cprint("     查: PA1 是否浮空(内部上拉被关过) / 是否与 1Mbps 舵机总线并走。\r\n");
    } else if (dne >= db * 9UL / 10UL) {
        cprint("  -> [D 通路在·位级被毁] 字节数对得上, 但每个字节都带噪声标志。\r\n");
        cprint("     这是真正的位错: 波形/参考地/串扰 —— 地弹嫌疑成立, 可以开始整改。\r\n");
    } else {
        cprint("  -> [E 部分受损] 字节数大致对, 噪声占 %lu%%: 边界情况。\r\n",
               (unsigned long)(db ? (dne * 100UL) / db : 0UL));
        cprint("     再跑 `ml 500` 看比例是否稳定; 同时留意 FE(帧错) 高不高。\r\n");
    }
    cprint("  ===============================\r\n\r\n");
}

static void master_selftest_tick(uint32_t now) {
    if (master_st_phase == 2U) {
        if ((int32_t)(now - master_st_quiet_at) < 0) return;
        master_st_phase = 0U;
        master_selftest_report();
        return;
    }
    if ((int32_t)(now - master_next_tx) < 0) return;
    master_next_tx = now + MASTER_TX_PERIOD_MS;

    /* payload: 6 轴逻辑角*10 都填"已发帧序号", 落回来看 raw 一眼能认出是哪一帧 */
    int16_t a10 = Aproto_EncDeg10((float)(master_st_total - master_st_n));
    uint8_t pl[12];
    for (uint8_t j = 0U; j < 6U; j++) {
        pl[j * 2U]      = (uint8_t)((uint16_t)a10 & 0xFFU);
        pl[j * 2U + 1U] = (uint8_t)((uint16_t)a10 >> 8);
    }
    uint8_t frame[APROTO_MAX_FRAME];
    uint8_t n = Aproto_BuildFrame(master_seq++, APROTO_CMD_STREAM_JOINTS,
                                  pl, 12U, frame, sizeof(frame));
    if (n > 0U && HAL_UART_Transmit(&huart4, frame, n, 20) == HAL_OK) {
        master_tx_ok++;
        master_st_exp_b += n;          /* 按真实帧长累计, 不硬写 19 */
    }
    if (master_st_n > 0U) master_st_n--;
    if (master_st_n == 0U) {
        master_st_phase    = 2U;
        master_st_quiet_at = now + 300U;
        cprint("  [ml] %u frames sent, waiting 300 ms for the RX path to settle...\r\n",
               (unsigned)master_st_total);
    }
}

static void master_selftest_start(uint16_t n) {
    LinkDiag d;
    LinkProto_GetDiag(&d);
    master_st_snap_b   = d.bytes_rx;
    master_st_snap_ne  = d.ne;
    master_st_snap_fe  = d.fe;
    master_st_snap_stm = LinkProto_StreamCount();
    master_st_snap_tx  = master_tx_ok;
    master_st_total    = n;
    master_st_n        = n;
    master_st_exp_b    = 0U;
    master_st_phase    = 1U;
    master_next_tx     = 0U;           /* 下一拍立即开始 */
    cprint("  [ml] self-test start: %u frames, 33 ms apart (~%lu ms)\r\n",
           (unsigned)n, (unsigned long)((uint32_t)n * MASTER_TX_PERIOD_MS));
    cprint("  [ml] 请确认 PA0 已短接到 PA1 (自环)。快照: bytes=%lu NE=%lu\r\n",
           (unsigned long)d.bytes_rx, (unsigned long)d.ne);
    cprint("  [ml] 期间不读舵机 —— 舵机断电也能跑; 结束后自动出判决。\r\n");
}

/* ===== `mg` GPIO 级自环自检 (主臂专用, 2026-09-14, v2) =====
 * 动机: `ml`/`lk` 仍受 UART 采样与解析影响, 无法回答最底层的问题 ——
 * "PA0 的高低电平, PA1 到底跟不跟?" 本命令**完全不经过 UART**:
 *   把 PA0 当普通推挽输出, PA1 当输入, 直接读 PA1 电平; 再反向做一遍。
 *
 * ⚠ v1 的判据是错的, 必须记下来(2026-09-14):
 *   v1 让 sense 脚配"输入 + 内部下拉", 判据是"两个方向都跟得住 = 通路完好"。
 *   实测却给出**自相矛盾**的结果 —— 向1(PA0 驱动)读到 1 不跟, 向2(PA1 驱动)
 *   读到 0 跟。**单根纯导线 + 同一颗 MCU + 同样推挽阻抗, 这种不对称物理上
 *   不可能发生** ⇒ 说明"下拉"这个判据本身不硬: 读到 1 时, 你分不清是
 *   "对端在拉高" 还是 "压根没连上"(悬空时自己的下拉本来是 0, 但只要有任何
 *   外部上拉就会变 1)。判据有歧义, 结论就必然是错的。
 *
 * v2 的修正: **sense 脚改用"输入 + 内部上拉"**。
 *   上拉输入读到 0 ⇒ 只可能是被外部**主动拉低**了 ⇒ **必然连通**。硬判据。
 *   于是三种情况可以干净分开:
 *     两向都能把对方拉低        -> 通路完好
 *     两向都拉不动(读到 1)      -> 断路(跳线没落在 MCU 引脚上)
 *     只有一向拉得动            -> 那一向的**输出低边驱动**损坏
 * 并且每一步都 dump 整字 `GPIOA->IDR` —— **输出脚的 IDR 仍反映引脚真实电平**,
 * 所以能顺带看出"驱动脚自己有没有被更强的源顶回来"。
 * 结束后自动把 PA0/PA1 恢复成 UART4 的 AF8 配置。 */

/* ===== 采样助手 (2026-09-14, mg/mp v3) =====
 * 对同一只脚连采 8 次, 打包成位图 (bit0 = 第 1 次)。
 *
 * 为什么必须连采: v2 实测出现"HAL_GPIO_ReadPin 读到 0, 但紧接着的 IDR 同一位是 1"
 * —— 同一个寄存器、同一个 bit、前后只差一条指令。已核实 HAL_GPIO_ReadPin 是干净的
 * 0/1 实现(不是把掩码值直接强转), 所以这不是打印 bug, 而是**电平真的在变**。
 * 一只挂了 40 kΩ 内建上拉的脚绝不可能在 100 ns 内自己翻转 ⇒ 线上有**在跳的信号源**。
 * 在那种节点上, 任何"单次采样"的判据都会随机, 前后自相矛盾。
 *
 *   00000000 -> 稳定低
 *   11111111 -> 稳定高
 *   其它     -> 节点不稳(有活动信号), 此时单点结论一律不可信
 *
 * 采样间隔约 40 us 空转 @168MHz, 8 次总耗时 < 1 ms。 */
static uint8_t master_sample8(GPIO_TypeDef *port, uint16_t pin) {
    uint8_t pat = 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
            pat |= (uint8_t)(1U << i);
        }
        for (volatile uint32_t k = 0U; k < 700U; k++) { /* ~40 us 空转 */ }
    }
    return pat;
}

/* 位图 -> "10110001" 串 (左=第1次采样)。buf 至少 9 字节 */
static const char *master_pat_str(uint8_t pat, char *buf) {
    for (uint32_t i = 0U; i < 8U; i++) {
        buf[i] = (char)(((pat >> i) & 1U) ? '1' : '0');
    }
    buf[8] = '\0';
    return buf;
}

static void master_gpio_loopback_test(void) {
    GPIO_InitTypeDef g = {0};
    uint8_t base_pa0, base_pa1;
    uint8_t p1lo_pa0, p1lo_pa1, p1hi_pa0, p1hi_pa1;
    uint8_t p2lo_pa0, p2lo_pa1, p2hi_pa0, p2hi_pa1;
    uint8_t i, unstable = 0U;
    char s0[10], s1[10];

    /* --- 基线: PA0/PA1 都配成"输入 + 下拉", 看没人驱动时各读到什么 --- */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1; g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_Delay(3);
    base_pa0 = master_sample8(GPIOA, GPIO_PIN_0);
    base_pa1 = master_sample8(GPIOA, GPIO_PIN_1);

    /* --- 相 1: PA0 推挽输出, PA1 输入(**上拉**) ---
     * sense 用"上拉": 读到稳定 0 ⇒ 只可能是被外部**主动拉低** ⇒ 必然连通(硬判据)。 */
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_1; g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(3);
    p1lo_pa1 = master_sample8(GPIOA, GPIO_PIN_1);
    p1lo_pa0 = master_sample8(GPIOA, GPIO_PIN_0);   /* 输出脚的 IDR 仍反映引脚真实电平 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(3);
    p1hi_pa1 = master_sample8(GPIOA, GPIO_PIN_1);
    p1hi_pa0 = master_sample8(GPIOA, GPIO_PIN_0);

    /* --- 相 2: PA1 推挽输出, PA0 输入(**上拉**) --- */
    g.Pin = GPIO_PIN_1; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(3);
    p2lo_pa0 = master_sample8(GPIOA, GPIO_PIN_0);
    p2lo_pa1 = master_sample8(GPIOA, GPIO_PIN_1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(3);
    p2hi_pa0 = master_sample8(GPIOA, GPIO_PIN_0);
    p2hi_pa1 = master_sample8(GPIOA, GPIO_PIN_1);

    /* --- 恢复 UART4 AF8 (与 HAL_UART_MspInit 保持一致) --- */
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &g);

    cprint("\r\n  ===== mg v3 GPIO loopback (不经过 UART) =====\r\n");
    cprint("  采样=同一只脚连采 8 次, 左=第1次。00000000=稳定低 11111111=稳定高, 混合=节点不稳\r\n");
    cprint("  baseline(都输入+下拉)      PA0=%s  PA1=%s\r\n",
           master_pat_str(base_pa0, s0), master_pat_str(base_pa1, s1));
    cprint("  [相1] PA0 输出低 -> PA0=%s(自己)  PA1=%s\r\n",
           master_pat_str(p1lo_pa0, s0), master_pat_str(p1lo_pa1, s1));
    cprint("        PA0 输出高 -> PA0=%s(自己)  PA1=%s\r\n",
           master_pat_str(p1hi_pa0, s0), master_pat_str(p1hi_pa1, s1));
    cprint("  [相2] PA1 输出低 -> PA0=%s  PA1=%s(自己)\r\n",
           master_pat_str(p2lo_pa0, s0), master_pat_str(p2lo_pa1, s1));
    cprint("        PA1 输出高 -> PA0=%s  PA1=%s(自己)\r\n",
           master_pat_str(p2hi_pa0, s0), master_pat_str(p2hi_pa1, s1));

    {
        const uint8_t pats[10] = { base_pa0, base_pa1,
                                   p1lo_pa0, p1lo_pa1, p1hi_pa0, p1hi_pa1,
                                   p2lo_pa0, p2lo_pa1, p2hi_pa0, p2hi_pa1 };
        for (i = 0U; i < 10U; i++) {
            if (pats[i] != 0x00U && pats[i] != 0xFFU) unstable = 1U;
        }

        /* 判据全部来自同一次连采, 不再混用两个通道 */
        uint8_t d1   = (p1lo_pa1 == 0x00U);   /* PA0 能把 PA1 拉到稳定低 */
        uint8_t d2   = (p2lo_pa0 == 0x00U);   /* PA1 能把 PA0 拉到稳定低 */
        uint8_t drv0 = (p1lo_pa0 == 0x00U);   /* PA0 自己拉低得动吗 */
        uint8_t drv1 = (p2lo_pa1 == 0x00U);   /* PA1 自己拉低得动吗 */

        if (unstable) {
            cprint("  -> [节点不稳] 有采样出现 0/1 混合 ⇒ 该节点上挂着**在跳的活动信号源**。\r\n");
            cprint("     40k 内建上拉绝不可能在 100ns 内自己翻转, 所以这**不是\"悬空\"能解释的**。\r\n");
            cprint("     单次读数在这种节点上必然随机 —— 上一版 mg 的矛盾就是它造成的。\r\n");
            cprint("     先查: 这两根排针/线上是否并了别的信号(舵机总线? 晶振? 另一路 TX? USB 桥?)\r\n");
        } else if (d1 && d2) {
            cprint("  -> [通路完好] 两向都能把对方拉到稳定低 ⇒ PA0<->PA1 确实是通导线。\r\n");
            cprint("     引脚与自环线都没问题 ⇒ NE 全错只剩 UART 波形/采样一路可查。\r\n");
        } else if (!d1 && !d2) {
            cprint("  -> [断路] 两向都拉不动对方(读到的都是自己的上拉)。\r\n");
            cprint("     ⇒ 自环跳线【没有落在 MCU 的 PA0/PA1 上】: 杜邦没通 / 插错排针 /\r\n");
            cprint("       这两根排针压根没连到 MCU 引脚本体。→ 用 `mo` + 万用表找脚。\r\n");
        } else if (d1) {
            cprint("  -> [单向] PA0 拉得低 PA1, 但 PA1 拉不低 PA0。\r\n");
        } else {
            cprint("  -> [单向] PA1 拉得低 PA0, 但 PA0 拉不低 PA1。\r\n");
        }

        if (!drv0) {
            cprint("     ⚠ 相1: PA0 拉低后**连自己都读不回 0** ⇒ PA0 低边驱动异常 / 被外部顶住。\r\n");
        }
        if (!drv1) {
            cprint("     ⚠ 相2: PA1 拉低后**连自己都读不回 0** ⇒ PA1 低边驱动异常 / 被外部顶住。\r\n");
        }
        if (drv0 && drv1 && (!d1 || !d2)) {
            cprint("     自检: 两只脚**自己的推挽都正常** ⇒ 不是驱动级坏, 而是**两只脚没连在一起**。\r\n");
            cprint("     ⇒ 问题在跳线/排针(或这两根排针不是 PA0/PA1), 不在 MCU。\r\n");
        }
    }
    cprint("  ================================================\r\n\r\n");
}

/* ===== `mp` 引脚直流环境探测 (主臂专用, 2026-09-14) =====
 * 动机: `mg` 需要跳线, 而跳线一旦插上就把两个节点并成一个, 反而看不出
 * "是哪一只脚本身有问题"。本命令**不需要任何跳线**, 对每只脚各做一次
 * "内部上拉读 → 内部下拉读"。内部上下拉约 40 kΩ, 任何外部器件只要不是
 * 高阻, 都会把它拽偏, 于是能一眼看出引脚上挂着什么:
 *
 *   上拉=1 下拉=0  ->  [浮空] 外部什么都没有。引脚干净。
 *   上拉=1 下拉=1  ->  [被顶高] 外部有驱动源/强上拉把引脚拉高。
 *                       ⚠ 万用表量"∞"是**断电**下量的; 板上芯片不工作时
 *                         它的输出是高阻, 一通电就可能主动驱动这只脚 ——
 *                         本命令测的正是"通电后的真实情况"。
 *   上拉=0 下拉=0  ->  [被拉低] 外部对地短路/强下拉。
 *   上拉=0 下拉=1  ->  [不稳]   容性耦合/振荡, 采不到稳定电平。
 *
 * 默认探测 PA0 / PA1, 外加 PC10 / PC11 作为**对照脚**
 * (UART4 在 F407 上还可以走 PC10/PC11 = AF8, 若它俩是"浮空"而 PA0/PA1
 *  是"被顶高", 那最省事的整改就是把链路挪到 PC10/PC11 —— 同一套代码)。
 * 结束后恢复 PA0/PA1 的 UART4 AF8 配置。 */
static void master_pin_probe_one(const char *nm, GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    uint8_t hi_pat, lo_pat, o_lo_pat, o_hi_pat;
    char b[4][10];
    const char *vin, *vout;

    /* --- 第 1 步: 输入态。各配一次内部上拉/下拉, **连采 8 次**看外部把它拽到哪 ---
     * 2026-09-14 v3: 单次采样在"节点上有活动信号"时会给出随机值(实测出现
     * ReadPin 与紧随其后的 IDR 位不一致), 连采能把"稳定"和"不稳"分开,
     * 详见 master_sample8() 注释。 */
    g.Pin = pin; g.Mode = GPIO_MODE_INPUT; g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(port, &g);
    HAL_Delay(3);
    hi_pat = master_sample8(port, pin);

    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(port, &g);
    HAL_Delay(3);
    lo_pat = master_sample8(port, pin);

    /* --- 第 2 步: 输出驱动自检 (2026-09-14 补) ---
     * 第 1 步只测"脚上挂着什么", **完全测不出这只脚的输出驱动级还能不能工作**。
     * 这是之前从未被怀疑过的一条(以前只怀疑线材/地弹, 从没怀疑过 PA0 自己)。
     * 做法: 把脚配成推挽输出, 直接读它自己的 IDR —— **输出脚的 IDR 仍反映
     * 引脚真实电平**。拉低应读到 0, 拉高应读到 1。
     * 读不回 = 该脚输出级损坏(UART TX 推不动 = 线上永远只有浮空/上拉)。 */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    o_lo_pat = master_sample8(port, pin);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(2);
    o_hi_pat = master_sample8(port, pin);

    /* 收尾放到"输入 + 无上下拉"这个最中性态, 由调用者最后统一恢复 */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &g);

    /* --- 判据: 任一状态出现 0/1 混合 ⇒ 节点上有活动信号, 单点结论不可信 --- */
    if ((hi_pat != 0x00U && hi_pat != 0xFFU) ||
        (lo_pat != 0x00U && lo_pat != 0xFFU)) {
        vin = "不稳!有活动信号";
    } else if (hi_pat == 0xFFU && lo_pat == 0x00U) {
        vin = "浮空(干净)";
    } else if (hi_pat == 0xFFU && lo_pat == 0xFFU) {
        vin = "被外部顶高";
    } else if (hi_pat == 0x00U && lo_pat == 0x00U) {
        vin = "被外部拉低";
    } else {
        vin = "矛盾(上下拉读数冲突)";
    }

    if ((o_lo_pat != 0x00U && o_lo_pat != 0xFFU) ||
        (o_hi_pat != 0x00U && o_hi_pat != 0xFFU)) {
        vout = "驱动级不稳";
    } else if (o_lo_pat == 0x00U && o_hi_pat == 0xFFU) {
        vout = "驱动级正常";
    } else if (o_lo_pat == 0xFFU && o_hi_pat == 0xFFU) {
        vout = "拉不低!低边坏/被顶住";
    } else if (o_lo_pat == 0x00U && o_hi_pat == 0x00U) {
        vout = "推不高!高边坏/被压住";
    } else {
        vout = "驱动级异常";
    }

    cprint("  %-4s in up=%s dn=%s [%s]  out lo->%s hi->%s [%s]\r\n",
           nm,
           master_pat_str(hi_pat, b[0]), master_pat_str(lo_pat, b[1]), vin,
           master_pat_str(o_lo_pat, b[2]), master_pat_str(o_hi_pat, b[3]), vout);
}

static void master_pin_probe_mini(void) {

    cprint("\r\n  ===== mp v3 pin DC probe (不需要跳线, 请拔掉 PA0<->PA1 跳线) =====\r\n");
    cprint("  in : 内部上拉读/下拉读(连采8次) -> 浮空(干净) / 被顶高 / 被拉低 / 不稳\r\n");
    cprint("  out: 配成推挽输出后读自己 -> 正常 / 拉不低 / 推不高\r\n");
    cprint("       串=8 次采样, 左=第1次。00000000=稳定低 11111111=稳定高, 混合=有活动信号\r\n");
    cprint("  ⚠ 本命令测的是**引脚本体**; 分辨不出跳线插没插(两脚各测一次, 单根\r\n");
    cprint("     上下拉时另一脚悬空, 读数与没插线一样)。判跳线用 `mg`。\r\n");

    master_pin_probe_one("PA0", GPIOA, GPIO_PIN_0);
    master_pin_probe_one("PA1", GPIOA, GPIO_PIN_1);
    cprint("  ---- 参考脚 (PC10/PC11 同时也是 SDIO_D2/D3, 本板有外部上拉) ----\r\n");
    master_pin_probe_one("PC10", GPIOC, GPIO_PIN_10);
    master_pin_probe_one("PC11", GPIOC, GPIO_PIN_11);

    /* --- 恢复: PA0/PA1 -> UART4 AF8; PC10/PC11 本就没用, 放回模拟输入 --- */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_AF_PP;
        g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
        g.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_1;
        HAL_GPIO_Init(GPIOA, &g);

        g.Pin = GPIO_PIN_10|GPIO_PIN_11; g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &g);
    }
    cprint("  ==========================================================\r\n\r\n");
}

/* 允许 `mp A 1` 形式探测任意一只脚 (端口字母 + 引脚号 0~15) */
static void master_pin_probe_arg(const char *port_letter, int pin_no) {
    GPIO_TypeDef *port = NULL;
    char nm[8];

    if (pin_no < 0 || pin_no > 15) {
        cprint("  [mp] 引脚号必须是 0~15。\r\n");
        return;
    }
    switch (port_letter[0]) {
    case 'A': case 'a': port = GPIOA; break;
    case 'B': case 'b': port = GPIOB; break;
    case 'C': case 'c': port = GPIOC; break;
    case 'D': case 'd': port = GPIOD; break;
    case 'E': case 'e': port = GPIOE; break;
    case 'F': case 'f': port = GPIOF; break;
    case 'G': case 'g': port = GPIOG; break;
    default:
        cprint("  [mp] 端口只支持 A~G。\r\n");
        return;
    }
    nm[0] = (char)(port_letter[0] & ~0x20U);   /* 转大写 */
    nm[1] = (char)('0' + (pin_no / 10));
    nm[2] = (char)('0' + (pin_no % 10));
    if (pin_no < 10) { nm[1] = (char)('0' + pin_no); nm[2] = '\0'; }
    else             { nm[3] = '\0'; }

    cprint("\r\n  ===== mp %s =====\r\n", nm);
    master_pin_probe_one(nm, port, (uint16_t)(1U << pin_no));
    cprint("\r\n");
}

/* ===== `mo` 输出保持 —— 给"万用表找脚"用 (主臂专用, 2026-09-14) =====
 * 用途: 没有原理图时, 这是唯一能把"排针 ↔ MCU 引脚"钉死的办法:
 *   把某只脚配成推挽输出并**保持在指定电平**, 然后拿万用表在排针上量 ——
 *   哪根排针跟着变, 哪根就是这个 MCU 引脚。`mg` 判出"断路"之后就用它找脚。
 *   用法:  mo A 1 0   -> PA1 输出低并保持
 *          mo A 1 1   -> PA1 输出高并保持
 *          mo r       -> 释放(PA0/PA1 恢复 UART4 AF8)
 * ⚠ 保持期间 UART4 的 TX/RX 被从 AF 上摘下来(链路暂停); `mo r` 或复位恢复。
 *   若目标脚上恰好有外部强驱动源, 会形成对顶(GPIO 电流被限在 ~25 mA, 短暂)。 */
static GPIO_TypeDef *master_port_of(char c) {
    switch (c) {
    case 'A': case 'a': return GPIOA;
    case 'B': case 'b': return GPIOB;
    case 'C': case 'c': return GPIOC;
    case 'D': case 'd': return GPIOD;
    case 'E': case 'e': return GPIOE;
    case 'F': case 'f': return GPIOF;
    case 'G': case 'g': return GPIOG;
    default:            return NULL;
    }
}

static void master_pin_hold_release(void) {
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW; g.Alternate = GPIO_AF8_UART4;
    g.Pin = GPIO_PIN_0; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOA, &g);
    cprint("  [mo] 已释放: PA0/PA1 恢复 UART4 AF8, 链路继续。\r\n");
}

static void master_pin_hold(const char *port_letter, int pin_no, int level) {
    GPIO_TypeDef *port = master_port_of(port_letter[0]);
    GPIO_InitTypeDef g = {0};

    if (port == NULL)              { cprint("  [mo] 端口只支持 A~G。\r\n"); return; }
    if (pin_no < 0 || pin_no > 15) { cprint("  [mo] 引脚号必须是 0~15。\r\n"); return; }

    g.Pin = (uint16_t)(1U << pin_no);
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, g.Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);

    cprint("  [mo] %c%d 现在输出 %s 并保持 —— 用万用表去排针上找哪根跟着变。\r\n",
           (char)(port_letter[0] & ~0x20U), pin_no, level ? "高(3.3V)" : "低(0V)");
    cprint("       测完敲 `mo r` 释放。\r\n");
}
#endif /* ARM_MASTER_BUILD */

static void master_service(void) {
    uint32_t now = HAL_GetTick();

    /* 每秒重申一次"六轴无扭矩"。主臂的角色契约是只读 —— 任何命令
     * (`i`/`t J 1`/`z`/teach) 都不该让手持中的主臂突然锁死或自己动。
     * 6 次 RAM 写/秒, 占用可忽略。 */
    static uint32_t release_next = 0;
    /* `ma 1` (主动模式) 下不再周期性释放 —— 否则力矩每秒被清一次, 轨迹
     * 跑两下就断, g/m/P 全废。`ma 0` 时照旧每秒重申"六轴无扭矩"。 */
    if (!master_hold_en && (int32_t)(now - release_next) >= 0) {
        release_next = now + 1000U;
        master_release_all();
    }

#if ARM_MASTER_BUILD
    /* `ml` 自检优先: 这一段时间内不读舵机, 只发固定帧 (舵机断电也能跑) */
    if (master_st_phase != 0U) {
        master_selftest_tick(now);
        return;
    }
#endif

    /* 两路输出共用同一拍读法 (2026-09-15):
     *   do_fwd = 组 0x13 帧发给从臂 (UART4/CAN 直连 —— 自有链路)
     *   do_sp  = 吐 `#P` 行给控制台  (PC 中转 —— 官网路线)
     * 两路都到期时**只读一次**舵机再分别输出: 读一轴是一次总线事务,
     * 翻倍会明显拖慢主循环。 */
    uint8_t do_fwd = 0U, do_sp = 0U;
    if (master_fwd_en && (int32_t)(now - master_next_tx) >= 0) {
        master_next_tx = now + MASTER_TX_PERIOD_MS;
        do_fwd = 1U;
    }
    if (master_sp_period_ms != 0U && (int32_t)(now - master_sp_next) >= 0) {
        master_sp_next = now + master_sp_period_ms;
        do_sp = 1U;
    }
    /* 两路都关: 不读不发 (主臂仍可自由拖动) */
    if (!do_fwd && !do_sp) return;

    float jd[6];
    for (uint8_t j = 0; j < 6; j++) {
        float v;
        int8_t r = read_pos_vote(j, &v);
        if (r != 1) {
            /* 读不到/不可信 -> 这一拍整拍不发 (绝不用旧值兜底, 否则从臂会
             * 收到凭空捏造的姿态)。连续失败才打印, 打印原因区分线缆/噪声/告警。 */
            if (do_fwd) master_tx_skip++;
            if (do_sp)  master_sp_skip++;
            if (++master_fail_run >= MASTER_FAIL_QUIET) {
                master_fail_run = 0;
                print_pose_read_fail("MASTER", j, r);
            }
            return;
        }
        master_fail_run = 0;
        jd[j] = servo_to_joint_deg(v, j);      /* 舵机度 -> 相对 Home 逻辑角 */
    }

    if (do_sp) {
        /* 一行 6 个"关节角×10"定点整数, 用与 0x13 帧**同一个**编码器,
         * 从臂侧用同一个解码器解回来 ⇒ 两端语义严格同一, 不可能错位。
         * ⚠ 整行一次 cprint: 中间不能插别的打印, 否则 PC 侧会收到半行。 */
        int16_t e[6];
        for (uint8_t j = 0; j < 6; j++) e[j] = Aproto_EncDeg10(jd[j]);
        cprint("#P %d %d %d %d %d %d\r\n",
               (int)e[0], (int)e[1], (int)e[2], (int)e[3], (int)e[4], (int)e[5]);
        master_sp_lines++;
    }

    if (!do_fwd) return;

    uint8_t pl[12];
    for (uint8_t j = 0; j < 6; j++) {
        int16_t a10 = Aproto_EncDeg10(jd[j]);
        pl[j * 2]     = (uint8_t)((uint16_t)a10 & 0xFFU);
        pl[j * 2 + 1] = (uint8_t)((uint16_t)a10 >> 8);
    }
#if ARM_LINK_CAN
    /* CAN 承载 (2026-09-14): 直接把逻辑负载交给 CAN 层做 8 字节分包,
     * 不再需要 Aproto_BuildFrame 的 AA55...+CRC16 串行帧。
     * 用 SendCmd 而不是 Send: 主从两端共用同一个 node_id, 从臂的接收
     * ID 段是 0x100+node, 所以主臂必须走"命令方向"。 */
    if (CanProto_SendCmd(master_seq++, APROTO_CMD_STREAM_JOINTS, pl, 12U)) {
        master_tx_ok++;
    }
#else
    uint8_t frame[APROTO_MAX_FRAME];
    uint8_t n = Aproto_BuildFrame(master_seq++, APROTO_CMD_STREAM_JOINTS,
                                  pl, 12, frame, sizeof(frame));
    if (n > 0U && HAL_UART_Transmit(&huart4, frame, n, 20) == HAL_OK) {
        master_tx_ok++;
    }
#endif
}

/* 主臂开机后置位: 六轴 Torque OFF, 人力拖得动 */
static void master_release_all(void) {
    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
}

/* 主臂发送状态 (`ms` 命令)。
 * 主臂是纯发送端, 链路上没有回执 —— 联调时"帧到底出去没有"只能靠这个:
 * tx_ok 在涨 = 本臂读 6 轴成功且 UART4 发送成功; tx_skip 在涨 = 丢帧被抑制。 */
static void master_print_status(void) {
    cprint("  Master mode: %s\r\n",
           master_hold_en
             ? "ACTIVE (ma 1) - torque ON, motion commands accepted"
             : "READ-ONLY (ma 0) - torque OFF, drag by hand");
    cprint("  Master forwarding: %s\r\n",
           master_fwd_en ? "ON" : "OFF (mf 0 / mf 1 to resume)");
    cprint("  Console stream: %s", master_sp_period_ms ? "ON" : "OFF");
    if (master_sp_period_ms)
        cprint(" (%lu ms, %lu Hz)", (unsigned long)master_sp_period_ms,
               (unsigned long)(1000U / master_sp_period_ms));
    cprint("  lines=%lu skipped=%lu\r\n",
           (unsigned long)master_sp_lines, (unsigned long)master_sp_skip);
    cprint("  Master TX: %lu frames sent, %lu suppressed (read fail)\r\n",
           (unsigned long)master_tx_ok, (unsigned long)master_tx_skip);
#if ARM_LINK_CAN
    cprint("    link %s %s @%uk -> slave, period %u ms\r\n",
           ArmCanDrv_IfName(), ArmCanDrv_PinDesc(),
           (unsigned)ArmCanDrv_GetBitrate(), (unsigned)MASTER_TX_PERIOD_MS);
    {
        uint8_t tec = 0U, rec = 0U, lec = 0U;
        uint8_t ok = ArmCanDrv_ErrorState(&tec, &rec, &lec);
        cprint("  Master RX: %lu frames from slave (CAN, 0x180+node)\r\n",
               (unsigned long)CanProto_PeerFrames());
        cprint("    CAN hw=%s  tec=%u rec=%u lec=%u   (LEC: 0=无错 3=ACK(无人应答) 6=CRC)\r\n",
               ok ? "ok" : "INIT-FAIL", (unsigned)tec, (unsigned)rec, (unsigned)lec);
        cprint("    (ct = 链路帧计数自检, cn = CAN 详情)\r\n");
    }
#else
    cprint("    link UART4 (PA0/PA1) @115200 -> slave, period %u ms\r\n",
           (unsigned)MASTER_TX_PERIOD_MS);
    /* 从臂回包: 主臂是纯发送端, 链路上唯一的"从臂是否收到/是否拒绝"
     * 线索 —— 从臂拒帧时会回 NACK(带 ARM_API_ERR_* 错误码), 这里直接
     * 显示出来, 不用跑到从臂那边敲 lk。 */
    {
        LinkDiag d;
        LinkProto_GetDiag(&d);
        cprint("  Master RX: %lu frames from slave (%lu responses, last NACK err=%u)\r\n",
               (unsigned long)d.frames, (unsigned long)d.rsp_rx, (unsigned)d.rsp_err);
        if (d.frames > 0U)
            cprint("    last from slave: cmd=0x%02X seq=%u len=%u CRC=%s\r\n",
                   (unsigned)d.cmd, (unsigned)d.seq, (unsigned)d.len,
                   d.crc_ok ? "OK" : "BAD");
    }
#endif
    if (master_fwd_en && master_tx_ok == 0U && master_tx_skip == 0U)
        cprint("    no TX yet -> check master arm read path\r\n");
    cprint("    (any READ FAIL below suppresses that whole frame)\r\n");
    print_joint_angles("Master");
}
#endif /* ARM_MASTER_BUILD */

static uint8_t read_start_pose_checked(float out[6]) {
    uint8_t bad_j = 0;        /* 读失败的轴 */
    uint8_t unstable_j = 0;   /* 采样无多数的轴 */

    for (uint8_t attempt = 0; attempt < START_READ_ATTEMPTS; attempt++) {
        uint8_t bad = 0;
        bad_j = 0; unstable_j = 0;

        for (uint8_t j = 0; j < 6; j++) {
            float v;
            int8_t r = read_pos_vote(j, &v);
            if (r == 1) { out[j] = v; continue; }
            bad = 1;
            if (r == 0) bad_j = j + 1U; else unstable_j = j + 1U;
            break;                     /* 该轴不可信 -> 整组重试 */
        }

        if (!bad) {
            if (attempt > 0)
                cprint("  Start pose read OK after retry %u (bus glitch)\r\n",
                       (unsigned)attempt);
            return 1;
        }
        HAL_Delay(4);
    }

    if (bad_j)
        print_pose_read_fail("Start pose", (uint8_t)(bad_j - 1U), 0);
    else
        print_pose_read_fail("Start pose", (uint8_t)(unstable_j - 1U), -1);
    cprint("  Move cancelled - cannot trust current pose (no target fallback).\r\n");
    return 0;
}

/* g / P(预设位姿) 命令的安全运动入口: 读当前实际角度 -> 规划校验序列 -> 启动第一段。
 * 返回 1=已规划, 0=无安全路径被取消 (W 命令依赖该返回值)。 */
static uint8_t cmdGotoSafePlan(const float tgt[6], float tx, float ty, float tz) {
    float cur[6];
    if (!read_start_pose_checked(cur)) return 0;   /* 起点读取(多数表决, 无目标兜底) */
    for (uint8_t j = 0; j < 6; j++) {
        /* 起点软拉回: 若当前位置仅在边界外 1~5 raw, 拉回最近边界。
         * 不处理大幅越界 (那会由后续 safe_seq_plan 拒绝)。 */
        uint16_t mn = JOINT_MIN_RAW[j];
        uint16_t mx = JOINT_MAX_RAW[j];
        if (mn == 0 && mx == 0) continue;     /* 未校准, 无约束 */
        uint16_t raw = deg_to_servo_raw(cur[j]);
        if (!raw_in_range(raw, mn, mx)) {
            int d_mn = (int)raw - (int)mn; if (d_mn < 0) d_mn = -d_mn;
            int d_mx = (int)raw - (int)mx; if (d_mx < 0) d_mx = -d_mx;
            if (d_mn <= START_SOFT_CLAMP_TOL_RAW || d_mx <= START_SOFT_CLAMP_TOL_RAW) {
                uint16_t new_raw = (d_mn <= d_mx) ? mn : mx;
                cur[j] = servo_raw_to_deg(new_raw);
                cprint("  Start J%d soft-clamped raw %d -> %d\r\n",
                       j + 1U, (int)raw, (int)new_raw);
            }
        }
    }
    if (!safe_seq_plan(cur, tgt, tx, ty, tz)) return 0;
    safe_seq_active = 1;
    safe_seq_idx = 0;
    cmdMove(safe_seq_wp[0], ARM_SC_DEF_VMAX, ARM_SC_DEF_ACCEL);
    return 1;
}

/* settle 稳定确认完成后由 cmdTrajectorySettleService 调用:
 * 推进安全序列下一段; 返回 1 表示已规划下一段 (跳过本轮 adaptive correction),
 * 返回 0 表示序列结束 (走正常 correction 流程)。 */
static uint8_t safe_seq_service(void) {
    if (!safe_seq_active) return 0;
    safe_seq_idx++;
    if (safe_seq_idx < safe_seq_n) {
        cprint("  Safe path segment %d/%d\r\n", safe_seq_idx + 1U, safe_seq_n);
        cmdMove(safe_seq_wp[safe_seq_idx], ARM_SC_DEF_VMAX, ARM_SC_DEF_ACCEL);
        return 1;
    }
    safe_seq_active = 0;
    /* W 命令的挥动准备段完成: 现在真正开始挥动 */
    if (swing_resume_pending) {
        swing_resume_pending = 0;
        swing_active = 1;
        cprint("  Swing start (approach done).\r\n");
    }
    return 0;
}

/* Predefined pose command */
static void cmdPointLoopStart(const float a[6], const float b[6],
                              uint16_t rounds, float speed) {
    memcpy(point_loop_a, a, sizeof(point_loop_a));
    memcpy(point_loop_b, b, sizeof(point_loop_b));
    point_loop_speed = speed;
    point_loop_phase = 0;
    point_loop_active = 1;
    /* Initial move to A is preparation; then execute rounds of A->B->A. */
    point_loop_legs_left = (rounds == 0) ? 0xFFFFU : (uint16_t)(rounds * 2U + 1U);
    cprint("  Point loop: A->B->A, rounds=%s, speed=%d deg/s\r\n",
           rounds == 0 ? "infinite" : "finite", (int)speed);
    /* Move to A first; completion then starts A->B. */
    cmdMove(point_loop_a, speed, speed * 2.0f);
}

static void cmdMultiPointStart(uint8_t count, uint16_t rounds, float speed) {
    multi_point_count = count;
    multi_point_index = 0;
    multi_point_speed = speed;
    for (uint8_t i = 0; i < count; i++) {
        if (multi_point_segment_speed[i] < 5.0f)
            multi_point_segment_speed[i] = 5.0f;
        if (multi_point_segment_speed[i] > 300.0f)
            multi_point_segment_speed[i] = 300.0f;
    }
    multi_point_legs_left = (rounds == 0U) ? 0xFFFFU
                                          : (uint16_t)(rounds * count);
    multi_point_started = 1;
    multi_point_waiting = 0;
    multi_point_wait_until = 0;
    multi_point_leg_reset();          /* 逐轴读失败计数/缓存/判位节流清零 */
    multi_point_poll_next = 0U;
    for (uint8_t p = 0U; p < count; p++) {
        multi_point_err_samples[p] = 0U;
        for (uint8_t j = 0U; j < 6U; j++) {
            multi_point_err_sum[p][j] = 0.0f;
            multi_point_err_min[p][j] = 0.0f;
            multi_point_err_max[p][j] = 0.0f;
        }
    }
    point_loop_active = 0;
    swing_active = 0;
    correction_pending = 0;
    cprint("  Multi-point loop: %d points, rounds=%s, speed=%d deg/s\r\n",
           count, rounds == 0U ? "infinite" : "finite", (int)speed);
    cmdMove(multi_points[0], multi_point_segment_speed[0],
            multi_point_segment_speed[0] * 2.0f);
}

/* 到位判定 (2026-09-10 改为三态 + 丢帧重试)
 * 返回: 1 = 六轴全部在容差内
 *       0 = 至少一轴读数有效但超出容差 (确实还没到)
 *       2 = 至少一轴读不到 / 帧异常 (无法判定)
 *
 * 为什么区分 2 和 0: 旧版把两者都返回 0, 调用处一律清零稳定计时器。
 * 本函数在 50Hz 服务里被调用, 每次读 6 轴 → 300 次读/秒, 而 Bus A
 * (J1/J2/J3) 已知偶发丢帧(实测日志有 "J3 read error")。只要一次失败就
 * 把 TRAJ_SETTLE_STABLE_MS(500ms) 的连续稳定窗口清零 → 窗口几乎永远
 * 攒不满 → QL 多点循环卡死在"等待到位确认"(2026-09-10 实测)。
 * 现在: 读失败(2)保持计时器, 只有真超差(0)才清零。另加一次原地重试,
 * 因为总线丢帧多为单次瞬态。 */
static uint8_t cmdMultiPointTargetReached(float actual[6]) {
    const float* target = multi_points[multi_point_index];
    uint8_t read_fail = 0U;
    uint32_t now = HAL_GetTick();
    for (uint8_t j = 0; j < 6; j++) {
        /* ===== 2026-09-17: 改为多数表决读 (read_pos_vote_to) =====
         * 旧写法 = 单次读 + 一次立即重试, 是全固件最后一处没有表决的读;
         * 而 `f`/`P 0`/`g` 早就统一用 read_pos_vote() 治 1Mbps 偶发丢字节
         * (见 readActualEndPose 的注释: "裸读时任何一次丢字节就让整条失败")。
         * 判位要连续六轴全过, 裸读下这就是必然卡死。
         * 超时用 10ms: 3 采样 × 2 重试全失败也只烧 ~70ms, 不会把主循环按住。 */
        float v = 0.0f;
        int8_t r = read_pos_vote_to(j, &v, USED_POS_READ_TIMEOUT_MS);
        if (r == 1) {
            mp_lastgood[j]    = v;
            mp_lastgood_ms[j] = now;
            actual[j] = v;
        } else {
            /* 偶发丢帧: 400ms 内的最近有效值可先用(只影响判位, 不影响安全路径) */
            if (mp_lastgood_ms[j] != 0U &&
                (int32_t)(now - mp_lastgood_ms[j]) <=
                (int32_t)MULTI_POINT_LASTGOOD_MS) {
                actual[j] = mp_lastgood[j];
            } else {
                actual[j] = -1.0f;
                read_fail = 1U;
                mp_fail_joint = j;
                mp_fail_diag  = pose_read_diag;
                mp_fail_err   = pose_read_err;
                mp_fail_dg[j] = pose_read_diag;
                mp_fail_eb[j] = pose_read_err;
                if (mp_fail_cnt[j] != 255U) mp_fail_cnt[j]++;
                continue;
            }
        }
        if (fabsf(actual[j] - target[j]) > MULTI_POINT_REACHED_TOL_DEG)
            return 0U;
    }
    return read_fail ? 2U : 1U;
}

/* 一段(两点之间)结束/开始时复位逐轴统计与缓存 —— 上一段的数据不许跨段用。 */
static void multi_point_leg_reset(void) {
    multi_point_stable_since  = 0U;
    multi_point_leg_since     = 0U;
    multi_point_readfail_cnt  = 0U;
    mp_fail_joint = 0U;
    mp_fail_diag  = DXL_RD_OK;
    mp_fail_err   = 0U;
    for (uint8_t j = 0; j < 6U; j++) {
        mp_fail_cnt[j]  = 0U;
        mp_fail_dg[j]   = 0U;
        mp_fail_eb[j]   = 0U;
        mp_lastgood[j]  = 0.0f;
        mp_lastgood_ms[j] = 0U;
    }
}

/* ===== 判位失败时的"逐轴真相" 报告 (2026-09-17 新增) =====
 * 现场只看到一句 "J6 corrupt frame" 是没法处置的 —— 必须回答:
 *   ① 哪一轴差多少(目标/实测/误差)   ② 哪几轴读不到、各失败几次、原因是什么
 *   ③ 够不到的那一轴是不是**没发力矩**(轨迹通道从不动 Torque, 被 `t J 0` 松过力
 *      或被旧版 `z` 关过力, 那一轴就会纹丝不动, 而所有命令都返回"成功")。
 * why: 触发原因文字("feedback unreadable x25" / "not confirmed in 8000 ms")。 */
static void cmdMultiPointReportTruth(const float actual[6], const char* why) {
    const float* target = multi_points[multi_point_index];
    cprint("\r\n  ===== QL point %d: %s =====\r\n",
           (unsigned)multi_point_index + 1U, why);
    cprint("    tol=%.1f deg   read-fail counts:",
           (double)MULTI_POINT_REACHED_TOL_DEG);
    for (uint8_t j = 0; j < 6U; j++) cprint(" J%d=%u", (unsigned)j + 1U,
                                            (unsigned)mp_fail_cnt[j]);
    cprint("\r\n");
    for (uint8_t j = 0; j < 6U; j++) {
        float err = (actual[j] >= 0.0f) ? (actual[j] - target[j]) : 0.0f;
        cprint("    J%d  tgt=", (unsigned)j + 1U);  cprint_deg1(target[j]);
        if (actual[j] < 0.0f) {
            cprint("  act=--      ");
        } else {
            cprint("  act="); cprint_deg1(actual[j]);
            cprint("  err="); cprint_deg1(err);
        }
        if (actual[j] < 0.0f) {
            cprint("  READ FAIL\r\n");
        } else if (fabsf(err) > MULTI_POINT_REACHED_TOL_DEG) {
            cprint("  OUT OF TOL");
            uint8_t tq = SERVO_GetTorqueEnabled(j);
            if (tq == 0xFF)   cprint("  torque=? (read failed)\r\n");
            else if (tq == 0U) cprint("  torque=OFF <<< 该轴没发力矩, 轨迹命令不会被执行; "
                                      "用 `t %u 1` 上力\r\n", (unsigned)j);
            else              cprint("  torque=ON (被负载卡住/限位挡块/够不到)\r\n");
        } else {
            cprint("  ok\r\n");
        }
        if (mp_fail_cnt[j] != 0U) {
            cprint("        read-fail reason: %s  err=0x%02X [%s]\r\n",
                   pose_read_diag_str(mp_fail_dg[j]), (unsigned)mp_fail_eb[j],
                   SERVO_ErrorString(mp_fail_eb[j]));
        }
    }
    cprint("  ===== continuing to next point =====\r\n\r\n> ");
}

static void cmdMultiPointRecordError(const float actual[6]) {
    uint8_t point = multi_point_index;
    const float* target = multi_points[point];
    uint32_t sample = multi_point_err_samples[point] + 1U;

    cprint("  QL point %d sample %lu error (actual-target deg):\r\n",
           point + 1U, (unsigned long)sample);
    for (uint8_t j = 0; j < 6U; j++) {
        float signed_err = actual[j] - target[j];
        if (multi_point_err_samples[point] == 0U ||
            signed_err < multi_point_err_min[point][j])
            multi_point_err_min[point][j] = signed_err;
        if (multi_point_err_samples[point] == 0U ||
            signed_err > multi_point_err_max[point][j])
            multi_point_err_max[point][j] = signed_err;
        multi_point_err_sum[point][j] += signed_err;
        cprint("    J%d: target=%d.%d actual=%d.%d err=%c%d.%d\r\n",
               j + 1U,
               (int)target[j], (int)(fabsf(target[j]) * 10.0f) % 10,
               (int)actual[j], (int)(fabsf(actual[j]) * 10.0f) % 10,
               signed_err < 0.0f ? '-' : '+', (int)fabsf(signed_err),
               (int)(fabsf(signed_err) * 10.0f) % 10);
    }
    multi_point_err_samples[point] = sample;
}

static void cmdMultiPointPrintErrorSummary(void) {
    cprint("\r\n  ===== QL Repeatability Error Summary =====\r\n");
    cprint("  Error = actual - target, unit: deg\r\n");
    cprint("  Statistics are independent for each saved point.\r\n");

    for (uint8_t point = 0U; point < multi_point_count; point++) {
        cprint("\r\n  Point %d samples: %lu (every actual-reached visit)\r\n",
               point + 1U, (unsigned long)multi_point_err_samples[point]);
        cprint("  |Joint| Min err | Max err | Avg err |\r\n");
        cprint("  +-----+---------+---------+---------+\r\n");
        for (uint8_t j = 0U; j < 6U; j++) {
            float average = (multi_point_err_samples[point] > 0U) ?
                            multi_point_err_sum[point][j] /
                            (float)multi_point_err_samples[point] : 0.0f;
            cprint("  | J%d  | %c%3d.%d  | %c%3d.%d  | %c%3d.%d  |\r\n",
                   j + 1U,
                   multi_point_err_min[point][j] < 0.0f ? '-' : '+',
                   (int)fabsf(multi_point_err_min[point][j]),
                   (int)(fabsf(multi_point_err_min[point][j]) * 10.0f) % 10,
                   multi_point_err_max[point][j] < 0.0f ? '-' : '+',
                   (int)fabsf(multi_point_err_max[point][j]),
                   (int)(fabsf(multi_point_err_max[point][j]) * 10.0f) % 10,
                   average < 0.0f ? '-' : '+', (int)fabsf(average),
                   (int)(fabsf(average) * 10.0f) % 10);
        }
        cprint("  +-----+---------+---------+---------+\r\n");
    }
}

static void cmdMultiPointAdvance(void) {
    if (multi_point_started) {
        multi_point_started = 0;
    } else if (multi_point_legs_left != 0xFFFFU) {
        multi_point_legs_left--;
    }
    if (multi_point_legs_left == 0U) {
        cprint("  Multi-point loop done (returned to point 1).\r\n");
        cmdMultiPointPrintErrorSummary();
        multi_point_count = 0;
        cprint("> ");
        return;
    }
    multi_point_index = (uint8_t)((multi_point_index + 1U) % multi_point_count);
    const float* next = multi_points[multi_point_index];
    uint8_t segment = (multi_point_index == 0U) ?
                      (uint8_t)(multi_point_count - 1U) :
                      (uint8_t)(multi_point_index - 1U);
    const float* prev = multi_points[segment];   /* 上一点: 读失败的规划起点兜底 */
    float cur[6];
    for (uint8_t j = 0; j < 6; j++) {
        cur[j] = SERVO_GetPosition(j);
        if (cur[j] < 0.0f) cur[j] = SERVO_GetPosition(j);          /* 重试一次 */
        if (cur[j] < 0.0f) cur[j] = prev[j];
        /* 2026-09-08 修复: 旧代码读失败时 cur=next(新目标), 若 J4 等偶发读失败,
         * 规划起点=新目标 → 舵机立刻收到远端目标全速冲刺 (QL ~1/50 换向爆冲)。
         * settle 刚确认臂在上一点, 用上一点当起点才是物理正确的兜底。 */
    }
    float speed = multi_point_segment_speed[segment];
    multi_point_leg_reset();     /* 新段: 逐轴读失败计数/最近有效值缓存清零 */
    Trajectory_Plan(cur, (float*)next, speed, speed * 2.0f);
    traj_tick = 0;
}

/* This must run continuously while the trajectory is idle. A loaded servo can
 * reach its target after the S-curve calculation has ended. */
static void cmdMultiPointService(void) {
    if (multi_point_legs_left == 0U || Trajectory_IsActive()) return;
    uint32_t now = HAL_GetTick();

    if (!multi_point_waiting) {
        /* ===== 判位节流 (2026-09-17): 不再每趟主循环都读总线 =====
         * 旧版每趟读 6 轴, 而且本服务在同一趟主循环里被调用两次(轨迹刚结束
         * 时) ⇒ 最多 12 次总线事务/趟; 主循环空转时就是对 1Mbps 半双工总线的
         * 连续风暴读。1Mbps 下每字节只有 10us 的硬实时窗口, 读得越密丢字节
         * 越多, 而判位要求"六轴一次全过"。稳定窗口本来就是 500ms,
         * 60ms 判一次足够, 总线占用降到约 1/50, 判位成功率反而更高。 */
        if ((int32_t)(now - multi_point_poll_next) < 0) return;
        multi_point_poll_next = now + MULTI_POINT_POLL_PERIOD_MS;
        if (multi_point_leg_since == 0U) multi_point_leg_since = now;  /* 本段等待起点 */

        float actual[6];
        /* 到位判定必须包含"稳定保持": 若此刻在容差内, 记下首次稳定时刻;
         * 一旦检测到偏离容差则清零(视为还在抖动/收尾)。只有持续稳定超过
         * TRAJ_SETTLE_STABLE_MS 才真正认为到位并采样, 避免 approach slow
         * 收尾阶段误差偶然进入容差就提前测量+切点。 */
        uint8_t rc = cmdMultiPointTargetReached(actual);
        if (rc == 0U || rc == 2U) {
            if (rc == 0U) {
                multi_point_stable_since = 0U;   /* 真超差, 重来 */
            } else if (++multi_point_readfail_cnt == 25U) {
                /* 读失败: 暂停稳定计时(不清零), 使偶发丢帧不再把 500ms 窗口打回原点。
                 * 计数到 25 先打一次**逐轴**真相 —— 旧版只打"最后一个失败轴",
                 * 会把早早就在偶发失败的其它轴全藏掉(现场那句 "J6 corrupt frame"
                 * 就是这么来的); 而且"读不到"分三类, 处置完全不同:
                 *   NO_REPLY   -> 线缆/接插/共地/供电
                 *   BAD_FRAME  -> 总线噪声/丢字节
                 *   ERROR_BITS -> 舵机告警(过载/过温/过压), 与线缆无关 */
                cmdMultiPointReportTruth(actual, "feedback unreadable (x25)");
            }
            /* ===== 单点确认超时 (2026-09-17 新增): 绝不再无限等 =====
             * 旧版没有任何上限: 只要有一轴够不到 2.0deg(例如那一轴被 `t J 0`
             * 松过力, 根本不执行轨迹命令), 或某轴持续读不到, 循环就永久卡在
             * 这一个点上, 而且只打一句误导性的 "corrupt frame" —— 现场完全
             * 看不出是哪一轴、差多少、为什么。超时后打印逐轴真相(含力矩开关
             * 状态), 然后放行进入下一点, 循环能自己跑完。 */
            if ((int32_t)(now - multi_point_leg_since) >=
                (int32_t)MULTI_POINT_CONFIRM_TIMEOUT_MS) {
                cmdMultiPointReportTruth(actual, "NOT confirmed in 8000 ms");
                multi_point_leg_reset();
                cmdMultiPointAdvance();
            }
            return;
        }
        multi_point_readfail_cnt = 0U;
        if (multi_point_stable_since == 0U) {
            multi_point_stable_since = now;   /* 进入容差起点 */
            return;
        }
        if ((int32_t)(HAL_GetTick() - multi_point_stable_since) <
            (int32_t)TRAJ_SETTLE_STABLE_MS)
            return;   /* 还不够稳, 继续等 */

        /* 此处已稳定保持超过窗口, 才记录误差 */
        multi_point_stable_since = 0U;
        /* 注意: 精简版 newlib 未链接 %f, 稳定窗口 ms 用整数格式输出 */
        cprint("  Point %d stable settled (%lu ms window). Sampling.\r\n",
               multi_point_index + 1U,
               (unsigned long)TRAJ_SETTLE_STABLE_MS);
        cmdMultiPointRecordError(actual);
        multi_point_waiting = 1;
        multi_point_wait_until = HAL_GetTick() +
            multi_point_dwell_ms[multi_point_index];
        cprint("  Point %d reached; dwell %lu ms.\r\n",
               multi_point_index + 1U,
               (unsigned long)multi_point_dwell_ms[multi_point_index]);
        return;
    }

    if ((int32_t)(HAL_GetTick() - multi_point_wait_until) < 0) return;
    multi_point_waiting = 0;
    cmdMultiPointAdvance();
}

/* ===== QLC: 连续流水线多点循环 =====
 * 与 QL 不同: 段间不做 0.5s 稳定判定、不采样、不驻留——每段 S 曲线一结束
 * 立刻从当前实际位置规划下一段, 形成连贯的"跑点"循环, 不出现到点停顿。
 * 关闭方式: S 停止 / 段数跑完 / 直接发其它运动命令(会被本循环接管,同旧 B 循环行为)。
 * 段速按点的 qv 设置(可选 speed 参数会一次性写满全部用到的段)。 */
static void qlc_finish_stop(void) {
    qlc_active = 0;
    qlc_started = 0;
    qlc_index = 0;
    qlc_count = 0;
    qlc_legs_left = 0;
    cprint("  QLC loop done (returned to point 1).\r\n> ");
}

static void cmdQLCService(void) {
    if (!qlc_active || Trajectory_IsActive()) return;

    if (qlc_started) {
        qlc_started = 0;                       /* 首段(去点1)不计入段数 */
    } else if (qlc_legs_left != 0xFFFFFFFFU) {
        if (qlc_legs_left == 0U) { qlc_finish_stop(); return; }
        qlc_legs_left--;
        if (qlc_legs_left == 0U) { qlc_finish_stop(); return; }
    }

    qlc_index = (uint8_t)((qlc_index + 1U) % qlc_count);
    const float *next = multi_points[qlc_index];
    uint8_t seg = (qlc_index == 0U) ? (uint8_t)(qlc_count - 1U)
                                    : (uint8_t)(qlc_index - 1U);
    const float *prev = multi_points[seg];       /* 读失败兜底: 上一点 */
    float cur[6];
    for (uint8_t j = 0; j < 6; j++) {
        cur[j] = SERVO_GetPosition(j);
        if (cur[j] < 0.0f) cur[j] = SERVO_GetPosition(j);          /* 重试一次 */
        if (cur[j] < 0.0f) cur[j] = prev[j];
        /* 同 cmdMultiPointAdvance 修复: 读失败不许拿 next 当起点(防换向爆冲) */
    }
    float speed = multi_point_segment_speed[seg];
    if (speed < 5.0f) speed = 5.0f;
    Trajectory_Plan(cur, (float *)next, speed, speed * 2.0f);
    traj_tick = 0;
}

/* 让上面的 service 在段结束后被调用: 由主循环轨迹完成链触发。 */

static void cmdQLCStart(uint8_t count, uint16_t rounds, float speed) {
    for (uint8_t p = 0; p < count; p++) multi_point_segment_speed[p] = speed;
    qlc_count = count;
    qlc_index = 0;
    qlc_legs_left = (rounds == 0U) ? 0xFFFFFFFFU : (uint32_t)rounds * (uint32_t)count;
    qlc_started = 1U;
    qlc_active = 1U;
    multi_point_legs_left = 0;
    point_loop_active = 0;
    swing_active = 0;
    swing_resume_pending = 0;
    correction_pending = 0;
    cprint("  QLC continuous loop: %d points, rounds=%s, speed=%d deg/s (no stops).\r\n",
           (int)count, rounds == 0U ? "infinite" : "finite", (int)speed);
    cmdMove(multi_points[0], speed, speed * 2.0f);
}

/* Normal m/g/P moves enter this state after their time curve ends. A single
 * blocking Present Position read is spread over each 80 ms poll interval, so
 * the 200 Hz write path remains responsive. */
static void cmdTrajectorySettleService(void) {
    if (!traj_settle_pending || Trajectory_IsActive()) return;

    uint32_t now = HAL_GetTick();

    /* 0924AP: 不再根据滞后量改写 GoalSpeed, 让舵机按当前速度自然到位。 */
    traj_settle_adaptive = 1U;

    /* ⭐0924AH: settle 超时兜底。某些姿态/参数下舵机可能无法在 1°/500ms
     * 窗口内稳定（如机械间隙、PID 振荡、读数跳变），导致 traj_settle_pending
     * 永远为 1，自动重武装永远等不到。超时后强制认为到位并进入 correction。 */
    if ((int32_t)(now - traj_settle_deadline) >= 0) {
        traj_settle_pending = 0U;
        cprint("  [WARN] settle timeout (5s): forcing feedback settled.\r\n");
        if (safe_seq_active) {
            if (safe_seq_service()) return;
        }
        if (correction_enabled) {
            correction_pending = 1U;
            correction_timer = now;
            correction_count = 0U;
            for (uint8_t j = 0; j < 6; j++) {
                corr_gain[j] = corr_gain_init[j];
                prev_error[j] = 0.0f;
                oscillation_count[j] = 0U;
                stall_count[j] = 0U;
                correction_offset[j] = 0.0f;
            }
            cprint("  Move reached (timeout). Adaptive correction starting...\r\n> ");
        } else {
            cprint("  Move reached (timeout): feedback settled. (correction disabled)\r\n> ");
        }
        return;
    }

    if ((int32_t)(now - traj_settle_next_poll) < 0) return;
    traj_settle_next_poll = now + TRAJ_SETTLE_POLL_MS;

    float actual = SERVO_GetPosition(traj_settle_joint);
    if (actual < 0.0f) actual = SERVO_GetPosition(traj_settle_joint);  /* 丢帧重试一次 */
    if (actual < 0.0f || fabsf(actual - correction_target[traj_settle_joint]) >
                         TRAJ_SETTLE_TOL_DEG) {
        traj_settle_pass = 0U;
    }
    traj_settle_mask |= (uint8_t)(1U << traj_settle_joint);
    traj_settle_joint++;

    if (traj_settle_joint < 6U) return;

    traj_settle_joint = 0U;
    if (traj_settle_mask != 0x3FU) return;
    traj_settle_mask = 0U;

    if (!traj_settle_pass) {
        traj_settle_pass = 1U;
        traj_settle_stable_since = 0U;
        return;
    }

    if (traj_settle_stable_since == 0U) {
        traj_settle_stable_since = now;
        return;
    }
    if (now - traj_settle_stable_since < TRAJ_SETTLE_STABLE_MS) return;

    traj_settle_pending = 0U;
    /* 0924AP: 到位后不再恢复 GOAL_SPEED / 切换加速度, 保持简单直接。 */
    /* 安全序列推进: 有下一段则直接规划 (中间段不做 adaptive correction,
     * 只在最后一段完成后校正一次)。返回 0 = 序列结束, 走正常校正。 */
    if (safe_seq_active) {
        if (safe_seq_service()) return;
    }
    if (correction_enabled) {
        correction_pending = 1U;
        correction_timer = now;
        correction_count = 0U;
        for (uint8_t j = 0; j < 6; j++) {
            corr_gain[j] = corr_gain_init[j];
            prev_error[j] = 0.0f;
            oscillation_count[j] = 0U;
            stall_count[j] = 0U;
            correction_offset[j] = 0.0f;
        }
        cprint("  Move reached. Adaptive correction starting...\r\n> ");
    } else {
        cprint("  Move reached: actual feedback settled. (correction disabled)\r\n> ");
    }
}

/* 导出：arm_main_exports.h（供调试/备用；arm_control_api 自带细粒度错误码版本） */
/* 工具俯仰核对 (2026-09-03): IK 解析解 t4 超 J4 限位时静默降级 CCD 位置兜底,
 * tool_pitch 被丢弃 (位置对、方向错)。用 FK 输出 yaw 与请求 pitch 比较,
 * 返回 wrap 到 [-180,180) 的角度差; |差|>10° 即视为俯仰未达成。 */
static float wrap_pitch_err(float got_pitch_deg, float pitch_deg) {
    float d = got_pitch_deg - pitch_deg;
    return fmodf(d + 540.0f, 360.0f) - 180.0f;
}

static uint8_t readActualEndPose(Kinematics_Pos *pos, float deg[6]);

int cmdSolveIKPoint(float x, float y, float z, float pitch_deg,
                           float deg[6]) {
    /* 2026-09-07: "已在其位"捷径 —— 当前实际位姿已满足目标 (位置误差<=FK_VERIFY
     * 且 pitch 误差<=10° 且 6 轴 raw 在限位内) 时, 直接采用当前实际关节角。
     * 背景: HOME 等 pose 常贴着 J3 连续限位边缘 (距 +限位仅 ~0.4°), 解析 IK
     * 的限位检查被零点几度的模型/实测误差判越限 -> 降级 CCD(俯仰自由) ->
     * pitch 校验失败误报 "J4 limit?"。物理上明明可达 (FK 同点完全一致)。 */
    {
        float cur[6];
        Kinematics_Pos curpos;
        if (readActualEndPose(&curpos, cur)) {
            float cerr = sqrtf((curpos.x - x) * (curpos.x - x) +
                               (curpos.y - y) * (curpos.y - y) +
                               (curpos.z - z) * (curpos.z - z));
            float cpitch = curpos.yaw * 180.0f / 3.14159265f;
            if (cpitch < 0.0f) cpitch += 360.0f;
            if (cerr <= KIN_FK_VERIFY_MM &&
                fabsf(wrap_pitch_err(cpitch, pitch_deg)) <= 10.0f) {
                uint8_t in_lim = 1;
                for (uint8_t j = 0; j < 6; j++) {
                    uint16_t raw = deg_to_servo_raw(cur[j]);
                    if (!raw_in_range(raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) {
                        in_lim = 0;
                        break;
                    }
                }
                if (in_lim) {
                    for (uint8_t j = 0; j < 6; j++) deg[j] = cur[j];
                    return 1;
                }
            }
        }
    }

    Kinematics_Pos target;
    target.x = x; target.y = y; target.z = z;
    target.yaw = pitch_deg * 3.14159265f / 180.0f;

    float rad[6];
    if (!IK_Solve(&target, rad)) return 0;
    for (uint8_t j = 0; j < 6; j++) {
        deg[j] = rad_to_servo_deg(rad[j], j);
    }
    /* J5(腕自转)/J6(夹爪)不改变末端位置: 保持当前角度 */
    float cur_j5 = SERVO_GetPosition(4);
    float cur_j6 = SERVO_GetPosition(5);
    if (cur_j5 >= 0.0f) deg[4] = cur_j5;
    if (cur_j6 >= 0.0f) deg[5] = cur_j6;
    for (uint8_t j = 0; j < 6; j++) {
        uint16_t raw = deg_to_servo_raw(deg[j]);
        if (!raw_in_range(raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) return 0;
    }
    Kinematics_Pos verify;
    FK_Solve(rad, &verify);
    float err = sqrtf((verify.x - x) * (verify.x - x) +
                      (verify.y - y) * (verify.y - y) +
                      (verify.z - z) * (verify.z - z));
    if (err > KIN_FK_VERIFY_MM) return 0;
    /* 俯仰降级拦截 (与 g 命令一致): 位置对但工具方向错的多点目标不可用 */
    float got_pitch = verify.yaw * 180.0f / 3.14159265f;
    if (got_pitch < 0.0f) got_pitch += 360.0f;
    if (fabsf(wrap_pitch_err(got_pitch, pitch_deg)) > 10.0f) {
        cprint("  IK point rejected: tool pitch NOT met (req %.0f got %.0f, J4 limit?)\r\n",
               pitch_deg, got_pitch);
        return 0;
    }
    return err <= KIN_FK_VERIFY_MM;
}

static void cmdPose(uint8_t pose_id) {
    const float* pose;
    const char* name;
    switch (pose_id) {
        case 0: pose = POSE_HOME;  name = "HOME";  break;
        case 1: pose = POSE_FOLD;  name = "FOLD";  break;
        case 2: pose = POSE_READY; name = "READY"; break;
        case 3: pose = POSE_WAVE1; name = "WAVE1"; break;
        case 4: pose = POSE_WAVE2; name = "WAVE2"; break;
        default:
            cprint("  Unknown pose %d. Available: 0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2\r\n", pose_id);
            return;
    }
    cprint("  Pose %d (%s): %d %d %d %d %d %d\r\n",
           pose_id, name, (int)pose[0], (int)pose[1], (int)pose[2],
           (int)pose[3], (int)pose[4], (int)pose[5]);
    cmdGotoSafePlan((float*)pose, 0.0f, 0.0f, 0.0f);
}

/* ===== 0924CG: 折叠位(休息位)标定 —— P here / P clear =====
 * `P here`: 以**当前实际位姿**为新的 P 0 目标 (relhome 关节角),
 *   区F v3 持久化, 重启保留。与 zf here 同思路: 不关心零点/几何模型
 *   偏差, 手摆/遥操到想要的休息位后一键记录。
 *   生效范围: P 0 目标、RC CIR 折叠键、折叠位白名单 fk_is_home_dock
 *   (±3°)、桌面下限推导 (未设 zf 时) —— refresh_pose_servo() +
 *   update_home_dock() 一并重算。
 * `P clear`: 撤销标定, 回编译默认 POSE_HOME_J。
 * 注意: 标定存的是 relhome 关节角 (相对零点), 之后再做 M a 重定中位/
 * c 重校零点, P 0 目标会随新零点自动重算 —— 存关节角而非舵机度正
 * 是为此。 */

/* 把 relhome 关节角刷进运行时折叠位并重算全部派生量; 返回 flash 结果 */
static uint8_t readActualEndPose(Kinematics_Pos *pos, float deg[6]);  /* 前向声明 (定义在 FK 章节) */
static uint8_t pose_home_apply(const float rel[6], uint8_t valid) {
    memcpy(pose_home_j_rt, rel, sizeof(pose_home_j_rt));
    pose_home_cal = valid;
    refresh_pose_servo();
    update_home_dock();
    uint8_t zfv = (geo_zfloor_mm >= 0.0f) ? 1U : 0U;
    return GeoCfg_Save3(WORLD_OFFSET_X, WORLD_OFFSET_Y, WORLD_OFFSET_Z,
                        zfv ? geo_zfloor_mm : 0.0f, zfv,
                        geo_zf_on, geo_margin_raw,
                        pose_home_j_rt, pose_home_cal);
}

static void cmdPoseCalHere(void) {
    float deg[6];
    Kinematics_Pos pos;
    if (!readActualEndPose(&pos, deg)) return;   /* 读失败已在内部打印原因 */

    float rel[6];
    for (uint8_t j = 0; j < 6U; j++)
        rel[j] = servo_to_joint_deg(deg[j], j);

    /* 合理性检查: relhome 超过 ±200° 基本就是零点/读数异常, 拒存 */
    for (uint8_t j = 0; j < 6U; j++) {
        if (fabsf(rel[j]) > 200.0f) {
            cprint("  ⚠ J%d relhome=%+.1f° 超出 ±200°, 疑似零点异常, 拒绝标定\r\n",
                   j + 1, (double)rel[j]);
            cprint("     先做 c 零点校准或检查读数, 再重试 P here\r\n");
            return;
        }
    }

    /* ⭐0924CK: 标定限位校验 —— 手动折叠(力矩 OFF)时软件限位不生效,
     * 当前位姿可能已超出标定限位。存进去后 P 0 的安全路径规划要求每个
     * 路点都在限位弧内, 目标本身超限 ⇒ 永远 "NO safe segment plan"
     * (2026-11-18 现场: J6 手折到 142.3°=raw1618 < min 1638, P 0 必挂,
     * 规划器钳边路点还差 1 raw 舍入报 1637)。未校准轴 (mn==mx==0) 跳过。 */
    for (uint8_t j = 0; j < 6U; j++) {
        uint16_t mn = JOINT_MIN_RAW[j];
        uint16_t mx = JOINT_MAX_RAW[j];
        if (mn == 0U && mx == 0U) continue;
        uint16_t raw = deg_to_servo_raw(deg[j]);
        if (raw < mn || raw > mx) {
            cprint("  ⚠ J%d 当前 %.1f° (raw %u) 超出标定限位 [%u, %u], 拒绝标定\r\n",
                   j + 1, (double)deg[j], (unsigned)raw,
                   (unsigned)mn, (unsigned)mx);
            cprint("     两种处理: ① 把该轴摆回限位弧内再 P here;\r\n");
            cprint("               ② 折叠位本来就需要这个行程 → 重新校准该轴限位\r\n");
            cprint("               (c 向导 → x 进 Phase 2 → 手动带着该轴走完折叠行程 → s 存)\r\n");
            return;
        }
    }

    uint8_t rc = pose_home_apply(rel, 1U);
    cprint("  折叠位已标定为当前位姿 (FK z=%.1f mm):\r\n", (double)pos.z);
    cprint("    relhome: J1=%+.2f J2=%+.2f J3=%+.2f J4=%+.2f J5=%+.2f J6=%+.2f\r\n",
           (double)rel[0], (double)rel[1], (double)rel[2],
           (double)rel[3], (double)rel[4], (double)rel[5]);
    cprint("    servo  : J1=%.1f J2=%.1f J3=%.1f J4=%.1f J5=%.1f J6=%.1f\r\n",
           (double)POSE_HOME[0], (double)POSE_HOME[1], (double)POSE_HOME[2],
           (double)POSE_HOME[3], (double)POSE_HOME[4], (double)POSE_HOME[5]);
    if (rc == CFG_FLASH_OK) {
        cprint("  P 0 / 折叠位白名单(±%.0f°)/桌面下限 已跟随更新, 已存 flash (重启保留)\r\n",
               (double)HOME_DOCK_TOL_DEG);
        cprint("  建议立即敲 `P 0` 验证: 臂应原位小动或不动\r\n");
    } else {
        cprint("  ⚠ flash 写入失败: 仅本次上电有效 (重启回旧折叠位)\r\n");
    }
}

static void cmdPoseCalClear(void) {
    uint8_t had = pose_home_cal;
    pose_home_apply(POSE_HOME_J, 0U);
    if (had) {
        cprint("  折叠位标定已撤销, 回编译默认 (P 0 恢复出厂折叠位), 已写 flash\r\n");
    } else {
        cprint("  本来就没有折叠位标定, 维持编译默认\r\n");
    }
}


/* ===== FK/IK command handlers ===== */

/* Read actual joint angles, then calculate the current end-effector pose by FK.
 * Return 1 only when all six servo reads succeed.
 *
 * 2026-09-11: 改用 read_pos_vote() (读失败原地重试 + 两次采样互校), 与
 * P 0/g 的起点读取同一套容错。此前这里是**裸读 SERVO_GetPosition** ——
 * 6 次事务里任何一次丢字节就让整条 f 命令失败, 而总线 A (J1/J2/J3) 在
 * 1Mbps 下偶发丢帧是已知事实, 表现为"连发 3 次 f 才成功一次"。
 * f 是用户用来标定折叠位的命令, 结果必须可靠, 不能靠碰运气。 */
static uint8_t readActualEndPose(Kinematics_Pos *pos, float deg[6]) {
    float rad[6];
    for (uint8_t j = 0; j < 6; j++) {
        float v;
        int8_t r = read_pos_vote(j, &v);
        if (r != 1) {                       /* 读不到 / 帧跳变: 报明原因再退出 */
            print_pose_read_fail("FK feedback", j, r);
            return 0;
        }
        deg[j] = v;
        rad[j] = servo_deg_to_rad(deg[j], j);
    }
    return (FK_Solve(rad, pos) == 1) ? 1 : 0;
}

/* f: Forward Kinematics - show current end effector position */
static void cmdFK(void) {
    float deg[6];
    Kinematics_Pos pos;
    if (!readActualEndPose(&pos, deg)) return;

    int pitch_deg = (int)(pos.yaw * 180.0f / 3.14159265f);

    cprint("  FK: x=%d y=%d z=%d mm  pitch=%d deg\r\n",
           (int)pos.x, (int)pos.y, (int)pos.z, pitch_deg);
    cprint("  servo  : J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f deg\r\n",
           deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);
    cprint("  relhome: J1=%+.2f J2=%+.2f J3=%+.2f J4=%+.2f J5=%+.2f J6=%+.2f deg\r\n",
           servo_to_joint_deg(deg[0], 0), servo_to_joint_deg(deg[1], 1),
           servo_to_joint_deg(deg[2], 2), servo_to_joint_deg(deg[3], 3),
           servo_to_joint_deg(deg[4], 4), servo_to_joint_deg(deg[5], 5));
}

/* g: Inverse Kinematics - move to target xyz position */
static void cmdGoto(float x, float y, float z, float pitch_deg) {
    Kinematics_Pos target;
    target.x = x;
    target.y = y;
    target.z = z;
    target.yaw = pitch_deg * 3.14159265f / 180.0f;

    float rad[6];
    int ik_ok = IK_Solve(&target, rad);
    if (!ik_ok) {
        cprint("  IK: unreachable! (x=%d y=%d z=%d)\r\n",
               (int)x, (int)y, (int)z);
        RLOG_W("IK", "unreachable: x=%d y=%d z=%d p=%d",
               (int)x, (int)y, (int)z, (int)pitch_deg);
        return;
    }

    /* Convert radians to servo degrees (0-360) */
    float deg[6];
    for (uint8_t j = 0; j < 6; j++)
        deg[j] = rad_to_servo_deg(rad[j], j);

    /* J5(腕自转)/J6(夹爪)不改变末端位置: IK 已复制当前角度，再次确保 */
    float cur_j5 = SERVO_GetPosition(4);
    float cur_j6 = SERVO_GetPosition(5);
    deg[4] = (cur_j5 >= 0.0f) ? cur_j5 : 180.0f;
    deg[5] = (cur_j6 >= 0.0f) ? cur_j6 : 180.0f;

    /* Check if IK solution is within joint limits (raw space, wrap-around aware) */
    uint8_t out_of_range = 0;
    for (uint8_t j = 0; j < 6; j++) {
        uint16_t raw = deg_to_servo_raw(deg[j]);
        if (!raw_in_range(raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) {
            out_of_range = 1;
            break;
        }
    }
    if (out_of_range) {
        cprint("  IK: solution out of joint limits!\r\n");
        cprint("  J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
               (int)deg[0], (int)deg[1], (int)deg[2],
               (int)deg[3], (int)deg[4], (int)deg[5]);
        cprint("  Limits raw: min=[%d %d %d %d %d %d] max=[%d %d %d %d %d %d]\r\n",
               (int)JOINT_MIN_RAW[0], (int)JOINT_MIN_RAW[1], (int)JOINT_MIN_RAW[2],
               (int)JOINT_MIN_RAW[3], (int)JOINT_MIN_RAW[4], (int)JOINT_MIN_RAW[5],
               (int)JOINT_MAX_RAW[0], (int)JOINT_MAX_RAW[1], (int)JOINT_MAX_RAW[2],
               (int)JOINT_MAX_RAW[3], (int)JOINT_MAX_RAW[4], (int)JOINT_MAX_RAW[5]);
        cprint("  Move cancelled (safety).\r\n");
        return;
    }

    cprint("  IK: x=%d y=%d z=%d -> J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
           (int)x, (int)y, (int)z,
           (int)deg[0], (int)deg[1], (int)deg[2],
           (int)deg[3], (int)deg[4], (int)deg[5]);

    /* Verify with FK */
    Kinematics_Pos verify;
    FK_Solve(rad, &verify);
    float fk_err = sqrtf((verify.x - x) * (verify.x - x) +
                         (verify.y - y) * (verify.y - y) +
                         (verify.z - z) * (verify.z - z));
    cprint("  Verify FK: x=%d y=%d z=%d (err: %d mm)\r\n",
           (int)verify.x, (int)verify.y, (int)verify.z, (int)fk_err);

    /* Cancel if IK solution error is too large (容差 = AP_IK_FK_VERIFY_MM) */
    if (fk_err > KIN_FK_VERIFY_MM) {
        cprint("  IK error too large (%d mm), move cancelled.\r\n", (int)fk_err);
        RLOG_W("IK", "FK verify err=%d mm, cancelled", (int)fk_err);
        return;
    }

    /* 2026-09-03 (g 300 300 220 20 实测): IK 解析解求 t4 时若超出 J4 限位会
     * 静默降级到 CCD 位置兜底 (忽略 tool_pitch), 输出与不同 pitch 完全相同。
     * 这里核对实际工具俯仰, 超容差即取消, 不再静默给错姿态。 */
    {
        float verify_pitch = verify.yaw * 180.0f / 3.14159265f;
        if (verify_pitch < 0.0f) verify_pitch += 360.0f;
        float d = wrap_pitch_err(verify_pitch, pitch_deg);
        cprint("  Pitch: req=%d got=%.0f (err %+.0f deg)\r\n",
               (int)pitch_deg, verify_pitch, d);
        if (fabsf(d) > 10.0f) {
            cprint("  IK: tool pitch NOT met (J4 limit?) - move cancelled.\r\n");
            RLOG_W("IK", "pitch degraded: req=%d got=%.0f, cancelled",
                   (int)pitch_deg, verify_pitch);
            return;
        }
    }

    /* Execute move via safe-path planner (FK 路径校验, 防爪尖扫桌) */
    cmdGotoSafePlan(deg, x, y, z);
}

/* ===== Command handlers ===== */

static void cmdScan(void) {
    cprint("\r\nScanning buses...\r\n");
    ScanBothBuses();

#if ARM_ONE_BUS_BUILD
    /* ⚠ 必须读 ONE_BUS_FOUND(即主臂的 found_a), 不能写死 found_b ——
     * 否则 USART3 扫到的 6 颗会被判成全 MISSING(纯显示 bug, 已在 11-02 修)。 */
    cprint("  Bus %s (%s, single-wire), expect ID 1/2/3/4/5/6:\r\n",
           ONE_BUS_TAG, ONE_BUS_PORT);
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t ok = 0;
        for (uint8_t k = 0; k < ONE_BUS_CNT; k++) {
            if (ONE_BUS_FOUND[k] == EXPECT_ALL[i]) { ok = 1; break; }
        }
        cprint("    ID=%d : %s\r\n", EXPECT_ALL[i], ok ? "OK" : "MISSING");
    }
#else
    /* 0924BD: 按当前路由分组打印, 不再写死 "Bus A: ID1-3 / Bus B: ID4-6"。
     * 六颗全挂 B 段时, 旧输出会把 J1~J3 判成 MISSING(其实它们就在 B 段上)。 */
    for (uint8_t b = 0U; b < 2U; b++) {
        uint8_t        bus = b ? BUS_B : BUS_A;
        uint8_t        cnt = b ? count_b : count_a;
        const uint8_t *fnd = b ? found_b : found_a;
        uint8_t        any = 0U;
        for (uint8_t j = 0U; j < 6U; j++) if (JOINT_BUS[j] == bus) { any = 1U; break; }
        if (!any) continue;
        cprint("  Bus %s (%s), mapped joints:\r\n", b ? "B" : "A", b ? "UART5" : "USART3");
        for (uint8_t j = 0U; j < 6U; j++) {
            if (JOINT_BUS[j] != bus) continue;
            uint8_t ok = 0U;
            for (uint8_t k = 0U; k < cnt; k++) {
                if (fnd[k] == JOINT_ID[j]) { ok = 1U; break; }
            }
            cprint("    ID=%d (J%d) : %s\r\n", (int)JOINT_ID[j], (int)(j + 1U), ok ? "OK" : "MISSING");
        }
    }
#endif

    cprint("  Total: %d / 6 servos online\r\n", count_a + count_b);
}

/* ===== 全量 ID 扫描(诊断用): 逐 ID 1..253 Ping 两条总线 =====
 * 用于找出"定点扫描 MISSING"的舵机实际挂在哪个 ID/哪条总线上
 * (例: BusA 只认出 1 颗 -> 可能其余舵机 ID 没改成 2/3)。
 * 每个空 ID 等 20ms, 空总线约 5s, 两条总线约 10s。 */
static void cmdFullScan(void) {
#if ARM_ONE_BUS_BUILD
    /* 单总线形态: 只扫"那条线"(ONE_BUS)。
     *   主臂   -> BUS_A/USART3, 已拆 126, 扫 A 安全;
     *   集成板 -> BUS_B/UART5; **绝不扫 Bus A** —— BUS_ONE 下一次
     *             DXL_ScanBus(BUS_A,...) 就会拉高 DIR_A, 门1 使能与门3 抢总线。 */
    static uint8_t ids_one[16];

    cprint("\r\nFull ID scan (1..253) on %s single-wire, ~5s...\r\n", ONE_BUS_PORT);
    cprint("  Bus %s (%s):  ", ONE_BUS_TAG, ONE_BUS_PORT);
    uint8_t nb = DXL_ScanBus(ONE_BUS, ids_one, 16, 20);
    if (nb == 0) {
        cprint("no servo\r\n");
    } else {
        for (uint8_t i = 0; i < nb; i++) {
            cprint("%sID=%d", (i ? ", " : ""), ids_one[i]);
        }
        cprint("  (%d)\r\n", nb);
    }

    cprint("  Expect: ID={1,2,3,4,5,6} all on the same wire\r\n");
    for (uint8_t id = 1; id <= 6; id++) {
        uint8_t ok = 0;
        for (uint8_t k = 0; k < nb; k++) if (ids_one[k] == id) { ok = 1; break; }
        cprint("    ID=%d : %s\r\n", id, ok ? "OK" : "NOT FOUND");
    }
#else
    static uint8_t ids_a[16], ids_b[16];

    cprint("\r\nFull ID scan (1..253), ~10s on empty buses...\r\n");
    cprint("  Bus A (USART3): ");
    uint8_t na = DXL_ScanBus(BUS_A, ids_a, 16, 20);
    if (na == 0) {
        cprint("no servo\r\n");
    } else {
        for (uint8_t i = 0; i < na; i++) {
            cprint("%sID=%d", (i ? ", " : ""), ids_a[i]);
        }
        cprint("  (%d)\r\n", na);
    }

    cprint("  Bus B (UART5):  ");
    uint8_t nb = DXL_ScanBus(BUS_B, ids_b, 16, 20);
    if (nb == 0) {
        cprint("no servo\r\n");
    } else {
        for (uint8_t i = 0; i < nb; i++) {
            cprint("%sID=%d", (i ? ", " : ""), ids_b[i]);
        }
        cprint("  (%d)\r\n", nb);
    }

    /* 与期望布局对照 */
    cprint("  Expect: BusA={1,2,3} BusB={4,5,6}\r\n");
    for (uint8_t id = 1; id <= 6; id++) {
        const uint8_t* ids = (id <= 3) ? ids_a : ids_b;
        uint8_t n = (id <= 3) ? na : nb;
        uint8_t onbus = 0, other = 0;
        for (uint8_t k = 0; k < n; k++) if (ids[k] == id) { onbus = 1; break; }
        const uint8_t* oids = (id <= 3) ? ids_b : ids_a;
        uint8_t on = (id <= 3) ? nb : na;
        for (uint8_t k = 0; k < on; k++) if (oids[k] == id) { other = 1; break; }
        if (onbus)       cprint("    ID=%d : OK (correct bus)\r\n", id);
        else if (other)  cprint("    ID=%d : !! on WRONG bus !!\r\n", id);
        else             cprint("    ID=%d : NOT FOUND on either bus\r\n", id);
    }
#endif
}

static void cmdInit(void) {
    cprint("\r\nInitializing all servos...\r\n");
    SERVO_InitAll();
    g_soft_stop = 0U;                      /* ⭐0924CZ: 重新上力矩 = 解除软件急停 */
    cprint("  Done. All servos set to joint mode.\r\n");
    cprint("  Done. All servos set to joint mode, torque ON (holding).\r\n");
    cprint("  Use 't J 0' to release a joint (e.g. before hand positioning).\r\n");
}

static void cmdInitOne(uint8_t joint) {
    if (joint >= 6) { cprint("  Joint must be 0-5\r\n"); return; }
    cprint("  Init J%d (ID=%d)... ", joint+1, JOINT_ID[joint]);
    SERVO_InitJoint(joint);
    cprint("done\r\n");
}

static void cmdRealtimeFeedback(void) {
    /* 走遥测缓存(FeedbackSampleService 按 20ms 节拍轮转刷新), 不直接占总线 ——
     * pv 的打印频率因此与总线负载解耦: pv 50 不会再打断运动。 */
    uint32_t now = HAL_GetTick();
    cprint("  Feedback:");
    for (uint8_t j = 0; j < 6; j++) {
        if (telem_ok(j, now))
            cprint(" J%d=%.2f", j + 1, servo_raw_to_deg(telem_cache[j].pos));
        else
            cprint(" J%d=--", j + 1);
    }
    cprint("\r\n");
}

/* pf: periodic end-effector pose feedback, calculated from actual servo angles. */
static void cmdRealtimePoseFeedback(void) {
    float deg[6];
    Kinematics_Pos pos;
    if (!readActualEndPose(&pos, deg)) return;

    cprint("  Pose: X=%.2f Y=%.2f Z=%.2f mm  Pitch=%.2f deg\r\n",
           pos.x, pos.y, pos.z, pos.yaw * 180.0f / 3.14159265f);
}

static void cmdStatus(void) {
    cprint("\r\n  +----+----+-------+--------+------+\r\n");
    cprint("  |Jnt | ID | Angle | Moving | Temp |\r\n");
    cprint("  +----+----+-------+--------+------+\r\n");
    for (uint8_t j = 0; j < 6; j++) {
        float   pos  = SERVO_GetPosition(j);
        uint8_t mov  = SERVO_IsMoving(j);
        uint8_t temp = SERVO_GetTemperature(j);
        int deg_int = (int)pos;
        int deg_dec = (int)(pos * 10.0f) % 10;
        if (pos < 0) { deg_int = 0; deg_dec = 0; }
        cprint("  | J%d | %d  | %3d.%d |   %s    | %3dC |\r\n",
               j+1, JOINT_ID[j], deg_int, deg_dec,
               mov == 1 ? "Yes" : (mov == 0 ? "No " : "Err"),
               temp);
    }
    cprint("  +----+----+-------+--------+------+\r\n");
}

static void cmdTorque(uint8_t joint, uint8_t en) {
    if (joint >= 6) { cprint("  Joint must be 0-5\r\n"); return; }
    if (IsEStop()) { cprint("  E-STOP active! Cannot enable torque.\r\n"); return; }
    HAL_StatusTypeDef st = SERVO_SetTorque(joint, en);
    if (en) g_soft_stop = 0U;              /* ⭐0924CZ: 上力矩 = 解除软件急停 */
    cprint("  J%d Torque %s: %s\r\n",
           joint+1, en ? "ON" : "OFF", st == HAL_OK ? "OK" : "FAIL");
    if (en) BuzzerBeep(20);
}

static void cmdAngle(uint8_t joint, float deg) {
    if (joint >= 6) { cprint("  Joint must be 0-5\r\n"); return; }
    /* 单关节运动(不走轨迹)前也确保速度已恢复 */
    TrajSpeedEnsureNormal();
    /* ⭐关节限位生效 (2026-11-13 澄清, 删除两条过时注释"限位开关保护已关闭/
     * 0~300 全开" —— 那是限位机制接入前的旧话, 与实际行为相反):
     * clamp_angle() 把目标折成 raw 与 JOINT_MIN_RAW/MAX_RAW (c 校准 Phase2 /
     * `L` 手动值) 比对, 越界**夹到最近边界**并打印 "J%d: X -> Y deg (raw -> limit)"。
     * 注意两点: ① 未校准 (mn==mx==0) 时不夹, 裸值直达舵机 —— 先做 c 校准;
     *           ② 越界是夹取不是拒绝 (与 `z` 的拒绝语义不同); 且 `a` 不查 E-STOP,
     *              紧停状态下仍可单轴动 —— 现场排障有用, 但要知道这点。 */
    deg = clamp_angle(joint, deg);
    HAL_StatusTypeDef st = SERVO_SetPosition(joint, deg);
    cprint("  J%d -> %.2f deg: %s\r\n", joint+1, deg, st == HAL_OK ? "OK" : "FAIL");
}

/* ===== `z J` / `z a` —— 回"校准零点"位 =====
 * 契约 (sdk_package/03_INTERFACE 2.4): "单轴 / 全部回中位（**用校准零点**）"；
 * arm_config.json 里 `z` 的 notes 也是"用校准 zero"。
 * 目标 raw = calib_zero_raw[j]；未校准 (全 0) 时回落舵机中位 STS_RAW_CENTER。
 * 2026-09-17 修：旧实现 SERVO_Home() 把目标**写死 2048**、绕过限位与 E-STOP、
 * 结束还关力矩、每轴阻塞 ≤15s —— 详见 servo_driver.c: SERVO_SetGoalRaw() 留档。
 * ⚠ 与 `P 0` 的区别（两个不同姿态，按钮文案别混）：
 *   `z`   = 各关节 rel-home 角为 0 的官方 L 形中位（servo = SERVO_OFFSET[j]）；
 *   `P 0` = 折叠收纳位 POSE_HOME（J2 关节角 -103.65° 等，实测舵机度 178.8 76.2 270.3 250.6 182.7 140.4）。
 * 本命令非阻塞：发完即返回，到位请查 `p J` / `f`（或轮询 0xA1）。 */
static uint16_t home_target_raw(uint8_t j) {
    uint16_t z = calib_zero_raw[j];
    return (z == 0U || z >= 4096U) ? (uint16_t)STS_RAW_CENTER : z;
}

/* 目标合法性：与其它运动入口同一条判据（未校准 mn==mx==0 ⇒ 放行）。
 * 越界一律**拒绝**而不是夹取 —— 夹取会变成"GUI 以为在 0 位、实物停在别处"，
 * 正是本工程反复清理的静默类 bug。返回 1 = 合法。 */
static uint8_t home_check_target(uint8_t j, uint16_t tgt) {
    uint16_t mn = JOINT_MIN_RAW[j], mx = JOINT_MAX_RAW[j];
    if (mn == 0U && mx == 0U) return 1U;
    if (raw_in_range(tgt, mn, mx)) return 1U;
    cprint("  J%d: 0-point raw=%d OUT of work range [%d,%d] -> refused\r\n",
           j + 1, tgt, mn, mx);
    return 0U;
}

static void cmdGoHome(uint8_t joint) {
    if (joint >= 6) { cprint("  Joint must be 0-5\r\n"); return; }
#if ARM_MASTER_BUILD
    /* 主臂默认只读（人扶着），回零会上力矩跟人手较劲 —— 和 cmdMove() 同一道门闩 */
    if (!master_hold_en) {
        cprint("  MASTER role: read-only (ma 0). Send 'ma 1' to allow motion.\r\n");
        return;
    }
#endif
    if (IsEStop()) { cprint("  E-STOP active! Cannot home.\r\n"); return; }

    uint16_t tgt = home_target_raw(joint);
    if (!home_check_target(joint, tgt)) return;

    TrajSpeedEnsureNormal();          /* 收尾低速残留时先恢复，否则归位慢吞吞 */
    SERVO_SetTorque(joint, 1U);
    HAL_StatusTypeDef st = SERVO_SetGoalRaw(joint, tgt);
    cprint("  J%d -> 0-point raw=%d (%.1f deg, %s): %s   [verify: p %d / f]\r\n",
           joint + 1, tgt, servo_raw_to_deg(tgt),
           (calib_zero_raw[joint] == 0U || calib_zero_raw[joint] >= 4096U)
               ? "uncalibrated -> default center" : "calib zero",
           st == HAL_OK ? "OK" : "FAIL", joint);
}

static void cmdGoHomeAll(void) {
#if ARM_MASTER_BUILD
    if (!master_hold_en) {
        cprint("  MASTER role: read-only (ma 0). Send 'ma 1' to allow motion.\r\n");
        return;
    }
#endif
    if (IsEStop()) { cprint("  E-STOP active! Cannot home.\r\n"); return; }

    /* 先全部判合法再动：半成功（三轴走了三轴没走）比全不动更难收拾 */
    uint16_t tgt[6];
    uint8_t bad = 0U, uncal = 0U;
    for (uint8_t j = 0; j < 6; j++) {
        tgt[j] = home_target_raw(j);
        if (calib_zero_raw[j] == 0U || calib_zero_raw[j] >= 4096U) uncal++;
        if (!home_check_target(j, tgt[j])) bad++;
    }
    if (bad) {
        cprint("  %d joint(s) refused -> no joint moved (all-or-nothing).\r\n", bad);
        return;
    }

    TrajSpeedEnsureNormal();
    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1U);
    /* 0924AR: 改走轨迹规划器 —— 原先 SyncWrite 直达 + GoalSpeed=0(全速) 会让
     * 六轴以舵机满速冲到位, 与 P 0/m 的 S 曲线速度完全脱节(爪尖甩动)。
     * 现在与 P 0 同一条 cmdMove 通道、同一组 ARM_SC_DEF_VMAX/ACCEL(运行时
     * 被改过也天然同速)。cmdMove 域 = 舵机度, raw→deg 一一对应。 */
    float tgt_deg[6];
    for (uint8_t j = 0; j < 6; j++) tgt_deg[j] = servo_raw_to_deg(tgt[j]);
    cprint("\r\n  ===== Home ALL -> 0-point =====\r\n");
    cprint("  target raw: %d %d %d %d %d %d\r\n",
           tgt[0], tgt[1], tgt[2], tgt[3], tgt[4], tgt[5]);
    cprint("  source: %s\r\n", (uncal == 6U) ? "uncalibrated -> default center 2048"
                            : (uncal ? "mixed (uncalibrated axes -> 2048)"
                                     : "calib_zero_raw"));
    cprint("  trajectory: cmdMove v=%.0f deg/s a=%.0f deg/s2   [verify: f / 0x21]\r\n",
           ARM_SC_DEF_VMAX, ARM_SC_DEF_ACCEL);
    cmdMove(tgt_deg, ARM_SC_DEF_VMAX, ARM_SC_DEF_ACCEL);
}

static void cmdSweep(uint8_t joint) {
    if (joint >= 6) { cprint("  Joint must be 0-5\r\n"); return; }

    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];
    uint8_t data[2], byte;

    /* Enable torque (0924CN: 改走 SERVO_SetTorque, 让 PE5 力矩状态灯计数同步) */
    byte = 1;
    (void)SERVO_SetTorque(joint, 1U);

    /* Set speed */
    data[0] = 80; data[1] = 0x00;
    SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_SPEED, data, 2);

    /* Torque limit */
    data[0] = 0xFF; data[1] = 0x03;
    SERVO_WriteSimple(bus, id, STS_ADDR_TORQUE_LIMIT, data, 2);

    cprint("  Sweep J%d (safe range): ", joint+1);

    /* Sweep within safe limits only */
    uint16_t pos_min = JOINT_MIN_RAW[joint];
    uint16_t pos_max = JOINT_MAX_RAW[joint];
    if (pos_min == 0 && pos_max == 0) { pos_min = 0; pos_max = 4095; }
    uint16_t pos_mid = 2048;  /* 180 deg center (STS3215) */
    uint16_t positions[] = {pos_min, pos_mid, pos_max, pos_mid};
    const char* labels[] = {"min", "mid", "max", "mid"};

    for (int i = 0; i < 4; i++) {
        cprint("%s ", labels[i]);
        data[0] = positions[i] & 0xFF;
        data[1] = (positions[i] >> 8) & 0xFF;
        SERVO_WriteSimple(bus, id, STS_ADDR_GOAL_POSITION, data, 2);
        HAL_Delay(1500);  /* wait for move */
    }

    /* Torque OFF (0924CN: 同步力矩状态灯计数) */
    byte = 0;
    (void)SERVO_SetTorque(joint, 0U);

    cprint("done\r\n");
}

/* L: Show or set joint limits */
static void cmdLimits(uint8_t joint, float min_val, float max_val) {
    if (joint >= 6) {
        /* Show all limits */
        cprint("\r\n  Joint Safety Limits:\r\n");
        cprint("  +----+------+------+------+\r\n");
        cprint("  | J  | MIN  | MAX  | CUR  |\r\n");
        cprint("  +----+------+------+------+\r\n");
        for (uint8_t j = 0; j < 6; j++) {
            float cur = SERVO_GetPosition(j);
            if (cur < 0) {
                /* ⭐0924BG: 总线丢读(SERVO_GetPosition 返回 -1)时如实显示 Err。
                 * 旧代码 if(cur<0) cur=0 把错误码洗成 0°, 用户会误以为关节真的
                 * 在 0° —— 丢读是随机瞬时的(承重轴实测 ~25%), 显示 0 纯属误导。 */
                cprint("  | J%d | %4d | %4d |  Err |\r\n",
                       j+1, (int)JOINT_MIN[j], (int)JOINT_MAX[j]);
            } else {
                cprint("  | J%d | %4d | %4d | %4d |\r\n",
                       j+1, (int)JOINT_MIN[j], (int)JOINT_MAX[j], (int)cur);
            }
        }
        cprint("  +----+------+------+------+\r\n");
        cprint("  Usage: L <J> <min> <max>  (e.g. L 0 80 220; 设值存 flash, 0924BY)\r\n");
        cprint("         L clear  撤销手动限位 (回校准/编译默认)\r\n");
        return;
    }
    /* Set limits for one joint (STS3215: 0-360 deg, 旧 STS3215 的 300 上限已废)。
     * 这里只允许 min<max 的常规区间; 跨越 0/360 的行程请用 'c' 校准。 */
    if (min_val < 0) min_val = 0;
    if (max_val > 360) max_val = 360;
    if (min_val >= max_val) {
        cprint("  Error: min must be < max (for wrap-around range use 'c' calibration)\r\n");
        return;
    }
    JOINT_MIN[joint] = min_val;
    JOINT_MAX[joint] = max_val;
    JOINT_MIN_RAW[joint] = deg_to_servo_raw(min_val);
    JOINT_MAX_RAW[joint] = deg_to_servo_raw(max_val);
    IK_SetJointLimitsRaw(JOINT_MIN_RAW, JOINT_MAX_RAW, SERVO_OFFSET);  /* sync to IK solver */
    cprint("  J%d limits set: %d ~ %d deg (raw %d ~ %d)\r\n",
           joint+1, (int)min_val, (int)max_val,
           (int)JOINT_MIN_RAW[joint], (int)JOINT_MAX_RAW[joint]);
    /* ⭐0924BY: 设值即持久化 (区 I), 重启自动恢复, 校准限位之上生效 */
    if (Jlim_Save() == CFG_FLASH_OK) {
        cprint("  已写入 flash (区 I), 重启自动恢复\r\n");
    } else {
        cprint("  ⚠ flash 写入失败: 限位仅本次上电有效\r\n");
    }
}

/* ===== Hand-teaching: continuous record + real-time replay =====
 * Flow:
 *   teach        -> torque OFF, auto-samples all six joints @50 Hz while dragging
 *   teach exit   -> stop sampling, torque ON (keeps the recording)
 *   play [loop]  -> torque ON, S-curve back to the recorded start, then replay
 *                   every sample at the recorded 50 Hz pace (J1..J6 together,
 *                   gripper J6 timing included). No per-point confirmation.
 * Safety note: replay does NOT re-run the FK/table-collision filter; it
 * reproduces exactly the physical motion that was taught. */

static void teach_servo_speed_all(uint16_t speed) {
    for (uint8_t j = 0; j < 6; j++) SERVO_SetSpeed(j, speed);
}

static void teach_play_finish(void) {
    teach_play_state = TEACH_PLAY_NONE;
    teach_play_idx = 0U;
    teach_play_loop = 0U;
    teach_servo_speed_all(all_servo_moving_speed);
    if (teach_play_from_file) {
        cprint("  Teach playback done: '%s' (%lu samples, ~%lu ms).\r\n> ",
               teach_play_file_name,
               (unsigned long)teach_play_src_cnt,
               (unsigned long)(teach_play_src_cnt * TEACH_REC_MS));
    } else {
        cprint("  Teach playback done (%lu samples, ~%lu ms).\r\n> ",
               (unsigned long)teach_play_src_cnt,
               (unsigned long)(teach_play_src_cnt * TEACH_REC_MS));
    }
    teach_play_from_file = 0U;
    teach_play_file_name[0] = 0;
}

static void teach_play_abort(uint8_t announce) {
    if (teach_play_state == TEACH_PLAY_NONE) return;
    teach_play_state = TEACH_PLAY_NONE;
    teach_play_idx = 0U;
    teach_play_loop = 0U;
    teach_servo_speed_all(all_servo_moving_speed);
    teach_play_from_file = 0U;
    teach_play_file_name[0] = 0;
    if (announce) cprint("  Teach playback aborted.\r\n");
}

/* 进入示教前的公共准备: 校验 + 取消所有运动 + 清录音。0=被拒绝 */
static uint8_t teach_mode_prepare(void) {
    if (IsEStop()) {
        cprint("  E-STOP active! Cannot enter teach mode.\r\n");
        return 0U;
    }
    if (teach_play_state != TEACH_PLAY_NONE) {
        cprint("  Stop playback first ('S').\r\n");
        return 0U;
    }
    if (teach_mode_active) {
        cprint("  Already recording (%lu samples). Drag on; 'teach exit' or 'play' to finish.\r\n",
               (unsigned long)teach_rec_cnt);
        return 0U;
    }
    /* Cancel any motion that may still be running. */
    Trajectory_Stop();
    safe_seq_active = 0;
    swing_active = 0;
    swing_resume_pending = 0;
    point_loop_active = 0;
    point_loop_legs_left = 0;
    qlc_active = 0;
    qlc_started = 0;
    qlc_legs_left = 0;
    multi_point_legs_left = 0;
    correction_pending = 0;
    traj_settle_pending = 0U;
    /* 2026-09-17: 清缓冲前**必须报出来**。旧版是静默 teach_rec_cnt=0U:
     * 现场在"上一轮录音没存盘就退出/断电前"再敲一次 `teach rec`, 会以为
     * 在录新的一段, 实际把上一条(最多 80 s)整段抹掉 —— 而唯一的证据是
     * 提示语从 "Already recording" 变成了 "RECORD-ONLY started",
     * 极容易被忽略(2026-09-17 现场就这么丢掉一条 4000 点录音)。
     * ⚠ 注意: 本函数只在"未 active"时才会走到这里 —— `teach exit` 之后
     *   buffer 仍在 RAM 里, 所以"退出过但没存盘"是最危险的窗口。 */
    if (teach_rec_cnt > 0U) {
        cprint("  ⚠ Discarding UNSAVED recording: %lu samples, ~%lu s "
               "(was it saved? 'list' shows stored files)\r\n",
               (unsigned long)teach_rec_cnt,
               (unsigned long)((teach_rec_cnt * TEACH_REC_MS) / 1000U));
    }
    teach_rec_cnt = 0U;
    teach_rec_full = 0U;
    /* 2026-09-16 录音完整性仪表清零 */
    teach_tick_lost = 0U;
    teach_smp_held = 0U;
    teach_smp_dropped = 0U;
    for (uint8_t j = 0U; j < 6U; j++) teach_read_fail[j] = 0U;
    /* 默认传统手拖示教; 只有 `teach rec` 在 prepare 成功后才把它置 1 */
    teach_record_only = 0U;
    teach_mode_active = 1U;
    return 1U;
}

/* 悬浮示教(默认): 扭矩 ON + 按姿态限矩托举, 边拖边录 */
static void cmdTeachEnter(void) {
    if (!teach_mode_prepare()) return;

    teach_assist_active = 1U;
    teach_pwm_active = 0U;

    /* 2026-09-08 B方案: 位置环导纳软跟随。
     * 保持 Mode 0, 把 P/I/D 降到极低, GoalSpeed/Acceleration 提到极高,
     * 让舵机以"零位置误差"方式跟随人手, 而不是用力把臂拉回到固定点。 */
    uint8_t acc = TEACH_FOLLOW_ACCEL;
    uint8_t speed_l = (uint8_t)(TEACH_FOLLOW_SPEED & 0xFFU);
    uint8_t speed_h = (uint8_t)(TEACH_FOLLOW_SPEED >> 8);
    for (uint8_t j = 0; j < 6; j++) {
        (void)SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                STS_ADDR_ACCELERATION, &acc, 1);
        uint8_t d[2] = {speed_l, speed_h};
        (void)SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                STS_ADDR_GOAL_SPEED, d, 2);
        (void)SERVO_SetTorque(j, 1);

        uint16_t r = SERVO_GetPositionRaw(j);
        if (r == 0xFFFFU) r = 2048U;
        teach_assist_goal_raw[j] = r;
        teach_assist_last_raw[j] = r;
        teach_off_raw[j] = 0;              /* 配重偏移清零, 重新收敛 */
    }

    /* 在线降增益: P 极小→软; I=0→无积分恢复力; D=0→不抗拒速度。
     * PUNCH 同步压零: P=0 时 punch 脉冲仍会产生启动推力, 必须一起关。 */
    for (uint8_t j = 0; j < 6; j++) {
        if (!SERVO_ReadReg(j, STS_ADDR_PUNCH, &teach_punch_backup[j]))
            teach_punch_backup[j] = 32U;   /* 读失败按典型默认值恢复 */
        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_PUNCH,
                          &teach_soft_punch, 1);
        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_P_GAIN,
                          &teach_soft_p, 1);
        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_I_GAIN,
                          &teach_soft_i, 1);
        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_D_GAIN,
                          &teach_soft_d, 1);
    }

    cprint("  Teach FLOAT started (admittance follow).\r\n");
    cprint("  Mode=0, P/I/D=%d/%d/%d, speed=%d, accel=%d, deadband=%d raw.\r\n",
           teach_soft_p, teach_soft_i, teach_soft_d,
           TEACH_FOLLOW_SPEED, TEACH_FOLLOW_ACCEL, TEACH_FOLLOW_DEADBAND);
    cprint("  Drag the arm gently. exit='teach exit'  free='teach free'  pwm='teach pwm'.\r\n");
    cprint("  Tune: 'teach p <0~254>' (softness); 'teach i <0~20>' (gravity balance).\r\n");
}

/* 全松示教(旧方式): 扭矩 OFF 自由拖 */
static void cmdTeachEnterFree(void) {
    /* 2026-09-09: 若从 PWM 场直接进 free, 先把 J2/J3/J4 切回 Mode0,
     * 避免之后 exit 时模式状态错乱。 */
    if (teach_pwm_active) {
        for (uint8_t j = 1U; j <= 3U; j++) (void)SERVO_SetOperatingMode(j, 0U);
        teach_pwm_active = 0U;
    }
    if (!teach_mode_prepare()) return;
    teach_assist_active = 0U;
    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
    cprint("  Teach FREE started: all torques OFF. Drag the arm (gripper J6 included).\r\n");
    cprint("  Auto-sampling every 20 ms, max ~%lu s. 'teach exit' = stop+hold, 'save <name>' = store.\r\n",
           (unsigned long)((TEACH_REC_MAX * TEACH_REC_MS) / 1000U));
}

/* 跟随录制(2026-09-16): 遥操/流式跟随下的"纯观察者"示教。
 *
 * 与上面三种手拖示教的**根本区别**: 一个舵机寄存器都不写 —— 不改 P/I/D、
 * 不改 PUNCH、不动扭矩、不改 GOAL_SPEED、不切运行模式。只是把
 * teach_mode_active 置 1, 让 50Hz 节拍去跑 teach_record_tick() 的"纯读"分支。
 *
 * 为什么需要它: 主臂经 PC 中转驱动从臂时(`sp` → pc_relay.py → `#P` → `sf 1`),
 * 从臂是**被外部目标驱动的**。此时若用 cmdTeachEnter(FLOAT), 它会去写
 * P/I/D 与 GOAL_SPEED、并把自己的 goal 拉回 present —— 那就跟 `#P` 流
 * 直接打架, 臂会变成"两套目标互相抢"。`teach rec` 让从臂保持正常伺服,
 * 一边跟流一边录, 录到的是**从臂实际走出来的轨迹**(已过限位 clamp 与跳变守卫),
 * 存盘后可脱离主臂单独 `play`。
 *
 * ⚠ 录制期间命令白名单(ProcessCommand)必须放行 `#P`/`sf`/`ec`, 否则第一条
 *   `#P` 行就会被"非示教命令自动退出示教"的逻辑顶掉, 录不到任何东西。 */
static void cmdTeachEnterRec(void) {
    if (!teach_mode_prepare()) return;

    teach_assist_active = 0U;
    teach_pwm_active    = 0U;
    teach_record_only   = 1U;

    cprint("  Teach RECORD-ONLY started: servos untouched (torque/PID/speed unchanged).\r\n");
    cprint("  Sampling every %u ms, max ~%lu s. Stream follow (#P / sf 1) keeps working.\r\n",
           (unsigned)TEACH_REC_MS, (unsigned long)((TEACH_REC_MAX * TEACH_REC_MS) / 1000U));
    cprint("  Stop: 'teach exit'  -> then 'save <name>' to store, 'play <name>' to replay.\r\n");
}

static void cmdTeachExit(void) {
    if (!teach_mode_active) {
        cprint("  Not in teach mode.\r\n");
        return;
    }
    teach_assist_active = 0U;
    /* 2026-09-09 混合模式: 只有 J2/J3/J4 进过 PWM (Mode2), 只需把它们
     * 切回位置伺服; J1/J5/J6 模式没变, 仅 Torque OFF, 下面统一 Torque ON。 */
    if (teach_pwm_active) {
        cprint("  Switching J2/J3/J4 back to position mode (support the arm)...\r\n");
        for (uint8_t j = 1U; j <= 3U; j++) {
            uint8_t rc = SERVO_SetOperatingMode(j, 0U);
            if (rc != 1U)
                cprint("  J%d mode restore FAILED (code %u)\r\n", j + 1, rc);
        }
        teach_pwm_active = 0U;
    }
    /* ⚠ 跟随录制(teach rec)模式下**必须整段跳过**下面所有"恢复"写入:
     *   · PUNCH 恢复用的是 teach_punch_backup[] —— 那个数组只在
     *     cmdTeachEnter() 里被读过; `teach rec` 从未进入过悬浮分支,
     *     数组还是初值 0 ⇒ 照写就是把六轴 PUNCH 全清成 0。
     *   · 恢复 GOAL_SPEED 会调 teach_servo_speed_all(all_servo_moving_speed),
     *     而流式跟随在 stream_enter() 里特意把 GOAL_SPEED 写成 0(=最大)。
     *     这里再写一次"有限速度"⇒ 之后继续遥操时从臂会莫名变慢。
     *   record_only 模式下舵机本来就一直是正常状态, 什么都不需要恢复。 */
    if (!teach_record_only) {
        /* 2026-09-08 B方案: 恢复缓存 P/I/D + PUNCH, 让臂恢复强保持。 */
        for (uint8_t j = 0; j < 6; j++) {
            SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_PUNCH,
                              &teach_punch_backup[j], 1);
            SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_P_GAIN,
                              &pid_p[j], 1);
            SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_I_GAIN,
                              &pid_i[j], 1);
            SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j], STS_ADDR_D_GAIN,
                              &pid_d[j], 1);
        }
        /* 恢复全扭矩限、正常速度与加速度, 使臂强保持 */
        for (uint8_t j = 0; j < 6; j++) {
            uint8_t acc = sts_accel_value;
            (void)SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                    STS_ADDR_ACCELERATION, &acc, 1);
            uint8_t d[2] = {0xFF, 0x0F};   /* 4095 full torque */
            (void)SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                    STS_ADDR_TORQUE_LIMIT, d, 2);
        }
        for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1);
        teach_servo_speed_all(all_servo_moving_speed);  /* clear any approach-low speed */
    }
    uint8_t was_rec_only = teach_record_only;
    teach_record_only = 0U;
    teach_mode_active = 0U;
    cprint("  Recording stopped: %lu samples, ~%lu ms.%s\r\n",
           (unsigned long)teach_rec_cnt,
           (unsigned long)(teach_rec_cnt * TEACH_REC_MS),
           was_rec_only ? " Servos untouched (was record-only)."
                        : " Torques ON full (holding).");
    teach_report_integrity();
    cprint("  'save <name>' = keep this motion; 'play' = replay it now; 'rec clear' = discard.\r\n");
}

static void cmdTeachClear(void) {
    teach_rec_cnt = 0U;
    teach_rec_full = 0U;
    cprint("  Unsaved recording cleared (saved files unaffected, use 'del <name>').\r\n");
}

static void cmdTeachList(void) {
    cprint("  Unsaved recording: ");
    if (teach_rec_cnt == 0U) {
        cprint("(none). Use 'teach' then drag the arm.\r\n");
    } else {
        cprint("%lu samples, ~%lu ms. 'save <name>' to store it.\r\n",
               (unsigned long)teach_rec_cnt,
               (unsigned long)(teach_rec_cnt * TEACH_REC_MS));
    }
    if (TeachStore_Count() > 0U) {
        cprint("  Saved files: %lu. Use 'list' to show, 'play <name>' to replay.\r\n",
               (unsigned long)TeachStore_Count());
    } else {
        cprint("  No saved files yet.\r\n");
    }
}

/* ===== 命名示教文件: 保存/列表/删除/整理 ===== */
static void cmdTeachSave(const char *name) {
    if (teach_mode_active) cmdTeachExit();
    if (teach_rec_cnt < 2U) {
        cprint("  Nothing to save: enter 'teach', drag the arm, then 'save <name>'.\r\n");
        return;
    }
    char auto_nm[TSTORE_NAME_MAX + 1U];
    if (!name || !*name) {
        uint32_t i = 1U;
        while (i < 1000U) {
            snprintf(auto_nm, sizeof(auto_nm), "rec%lu", (unsigned long)i);
            if (TeachStore_Resolve(auto_nm) == 0xFFFFFFFFU) break;
            i++;
        }
        if (i >= 1000U) { cprint("  No free auto name. Give a name: 'save <name>'.\r\n"); return; }
        name = auto_nm;
    } else if (strlen(name) > TSTORE_NAME_MAX) {
        cprint("  Name too long (max %d chars, no spaces).\r\n", (int)TSTORE_NAME_MAX);
        return;
    } else if (strcmp(name, "loop") == 0) {
        cprint("  Reserved name. Use another (e.g. loop1).\r\n");
        return;
    }

    cprint("  Saving '%s': %lu samples, ~%lu ms to internal flash...\r\n",
           name, (unsigned long)teach_rec_cnt,
           (unsigned long)(teach_rec_cnt * TEACH_REC_MS));
    uint8_t st = TeachStore_Save(name, (const uint16_t (*)[6])teach_rec, teach_rec_cnt);
    if (st == TSTORE_OK) {
        cprint("  Saved. Files=%lu, free ~%lu KB. Recording buffer cleared; 'teach' for next one.\r\n",
               (unsigned long)TeachStore_Count(),
               (unsigned long)(TeachStore_FreeBytes() / 1024U));
        teach_rec_cnt = 0U;
        teach_rec_full = 0U;
    } else if (st == TSTORE_ERR_NAME) {
        cprint("  Bad name: 1..%d chars of A-Z a-z 0-9 _ - (no spaces).\r\n", (int)TSTORE_NAME_MAX);
    } else if (st == TSTORE_ERR_FULL) {
        cprint("  Flash store full. Delete files ('del') or run 'compact' first.\r\n");
    } else if (st == TSTORE_ERR_LIMIT) {
        cprint("  Too many files (max %d). Delete some first.\r\n", (int)TSTORE_MAX_FILES);
    } else {
        cprint("  Flash write error (%d). Please retry.\r\n", (int)st);
    }
}

static void cmdTeachDelete(const char *name) {
    if (!name || !*name) { cprint("  Usage: del <name|#>\r\n"); return; }
    uint8_t st = TeachStore_Delete(name);
    if (st == TSTORE_OK)
        cprint("  Deleted '%s'. Free ~%lu KB ('compact' reclaims space).\r\n",
               name, (unsigned long)(TeachStore_FreeBytes() / 1024U));
    else if (st == TSTORE_ERR_NFOUND)
        cprint("  No such file: '%s'. 'list' shows saved files.\r\n", name);
    else
        cprint("  Delete failed (%d).\r\n", (int)st);
}

static void cmdTeachCompact(void) {
    if (TeachStore_Count() == 0U) { cprint("  Store empty, nothing to compact.\r\n"); return; }
    cprint("  Compacting flash store (~%lu KB)... may take a few seconds.\r\n",
           (unsigned long)(TeachStore_FreeBytes() / 1024U));
    uint8_t st = TeachStore_Compact();
    if (st == TSTORE_OK)
        cprint("  Compacted. Files=%lu, free ~%lu KB.\r\n",
               (unsigned long)TeachStore_Count(),
               (unsigned long)(TeachStore_FreeBytes() / 1024U));
    else
        cprint("  Compact failed (%d).\r\n", (int)st);
}

static void cmdStoreList(void) {
    uint32_t n = TeachStore_Count();
    cprint("  Saved teach files (%lu):\r\n", (unsigned long)n);
    cprint("  +----+------------+--------+--------+------+\r\n");
    cprint("  | #  | Name       |Samples |~Dur(ms)| Type |\r\n");
    cprint("  +----+------------+--------+--------+------+\r\n");
    for (uint32_t i = 0; i < n; i++) {
        const char *nm = TeachStore_FileName(i);
        uint32_t smp = TeachStore_FileSamples(i);
        cprint("  | %2lu | %-10s | %6lu | %6lu | save |\r\n",
               (unsigned long)(i + 1U), nm ? nm : "?",
               (unsigned long)smp, (unsigned long)(smp * TEACH_REC_MS));
    }
    cprint("  +----+------------+--------+--------+------+\r\n");
    cprint("  Total %lu samples (~%lu s), free ~%lu KB. 'play <name|#>' 'del <name|#>' 'compact'.\r\n",
           (unsigned long)TeachStore_TotalSamples(),
           (unsigned long)(TeachStore_TotalSamples() * TEACH_REC_MS / 1000U),
           (unsigned long)(TeachStore_FreeBytes() / 1024U));
}

/* After the ramp, sample 0 has already been commanded as the S-curve target. */
static void teach_ramp_done(void) {
    if (teach_play_src_cnt < 2U) return;   /* not reachable (checked in play) */
    teach_play_state = TEACH_PLAY_STREAM;
    teach_play_idx = 1U;
    teach_servo_speed_all(TEACH_STREAM_SPEED);
    cprint("  At recorded start; streaming %lu samples...\r\n",
           (unsigned long)(teach_play_src_cnt - 1U));
}

/* 共用回放启动: teach_play_src/cnt 已由调用者设置 */
static void teach_play_start(uint8_t loop) {
    if (teach_play_state != TEACH_PLAY_NONE) {
        cprint("  Playback already running. Use 'S' to stop.\r\n");
        return;
    }
    if (teach_play_src_cnt < 2U) return;
    if (Trajectory_IsActive() || point_loop_active || swing_active ||
        swing_resume_pending || qlc_active || safe_seq_active) {
        cprint("  Another motion is active. Use 'S' first.\r\n");
        return;
    }
    /* 回放期间必须关掉流式跟随: 回放每 20ms 直接 SyncWrite goal, 而流帧也写 goal,
     * 两路目标会互相抢(现象: 回放被"拖住"或来回抖)。关掉后主臂就算还在 `sp`,
     * 那些 `#P` 也只会被记进 stream_ignored 然后丢掉, 不会打断回放。
     * 回放结束要恢复遥操: 从臂敲 `sf 1`。 */
    if (stream_follow_en) {
        stream_follow_en = 0U;
        if (stream_active) cmdStreamExit("teach playback");
        cprint("  (stream follow OFF for playback — from-arm: 'sf 1' to resume teleop)\r\n");
    }
    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1);

    /* First leg: S-curve from the current pose back to the recorded start. */
    float start[6], target[6];
    for (uint8_t j = 0; j < 6; j++) {
        start[j] = SERVO_GetPosition(j);
        if (start[j] < 0.0f) start[j] = servo_raw_to_deg(teach_play_src[0][j]);
        target[j] = servo_raw_to_deg(teach_play_src[0][j]);
    }
    teach_servo_speed_all(TEACH_STREAM_SPEED);
    traj_settle_pending = 0U;
    correction_pending = 0U;
    Trajectory_Plan(start, target, 30.0f, 60.0f);
    traj_tick = 0;
    teach_play_state = TEACH_PLAY_RAMP;
    teach_play_loop = loop;
    teach_play_idx = 0U;
    if (teach_play_from_file) {
        cprint("  Replay file '%s': %lu samples (~%lu ms), %s. Returning to start...\r\n",
               teach_play_file_name,
               (unsigned long)teach_play_src_cnt,
               (unsigned long)(teach_play_src_cnt * TEACH_REC_MS),
               loop ? "loop" : "once");
    } else {
        cprint("  Replay %lu samples (~%lu ms), %s. Returning to start...\r\n",
               (unsigned long)teach_play_src_cnt,
               (unsigned long)(teach_play_src_cnt * TEACH_REC_MS),
               loop ? "loop" : "once");
    }
    if (!Trajectory_IsActive()) teach_ramp_done();   /* already at the start pose */
}

static void cmdTeachPlay(uint8_t loop) {
    if (teach_mode_active) cmdTeachExit();      /* stop sampling; torque ON */
    if (teach_rec_cnt < 2U) {
        if (TeachStore_Count() > 0U)
            cprint("  No unsaved recording. Use 'play <name|#>' for a saved file ('list' to view).\r\n");
        else
            cprint("  Nothing to replay yet: enter 'teach', drag the arm, then 'play' or 'save'.\r\n");
        return;
    }
    teach_play_src = (const uint16_t (*)[6])teach_rec;
    teach_play_src_cnt = teach_rec_cnt;
    teach_play_from_file = 0U;
    teach_play_file_name[0] = 0;
    teach_play_start(loop);
}

static void cmdTeachPlayFile(const char *name, uint8_t loop) {
    if (!name || !*name) { cmdTeachPlay(loop); return; }
    if (teach_mode_active) cmdTeachExit();
    uint32_t idx = TeachStore_Resolve(name);
    if (idx == 0xFFFFFFFFU) {
        cprint("  No such saved file: '%s'. 'list' shows all.\r\n", name);
        return;
    }
    const uint16_t (*p)[6] = TeachStore_FileData(idx);
    uint32_t n = TeachStore_FileSamples(idx);
    if (!p || n < 2U) { cprint("  File '%s' has no playable data.\r\n", name); return; }
    teach_play_src = p;
    teach_play_src_cnt = n;
    teach_play_from_file = 1U;
    snprintf(teach_play_file_name, sizeof(teach_play_file_name), "%s",
             TeachStore_FileName(idx));
    teach_play_start(loop);
}

/* ================================================================
 * PS2 按键 -> 功能分发 (0924H, rc_button_service)
 * 每 50Hz 拍调一次 (ArmRc_Poll 之后、ArmRcCtrl_Tick 之前)。
 *   电平型: 夹爪开/合 —— 在 arm_rc.c 合成层处理 (按住扫描/松开保持)。
 *   边沿型: 中位 / 折叠位 / 示教录制 / 回放 —— 这里消费 ArmRc_BtnEdges。
 * 安全链: 触发动作前检查 E-Stop/轨迹/回放; 动作类(中位/折叠/回放)先
 *   解除 RC (ArmRcCtrl_SetArmed(0)) —— 网口/文本永远赢, 要再动重 rc go on。
 *   录制(跟随录制)与 RC 驱臂**兼容** —— 录的就是 RC 实际走出来的轨迹。
 * ================================================================ */
static uint32_t rc_keysw_until = 0U;   /* rc keysw 监视窗口截止时刻 (0=关) */
static uint16_t rc_keysw_prev  = 0U;

/* 0924Q: 中位/折叠/回放按键触发后自动恢复 RC 武装 */
static uint32_t rc_auto_rearm_until = 0U;
static uint8_t  rc_auto_rearm_req   = 0U;
static uint8_t  rc_auto_rearm_sf    = 1U;

/* 0924AS: 重武装触发从"定时 8s 窗口"改为"机械臂实际静止 2s"。
 * 每 200ms 实读六轴(走 SERVO_GetPosition 总线实读, 非缓存), 任一轴
 * 位移 > 0.5° 就清零累计; 静止累计满 2s 才重武装。只在挂起期采样
 * (此时 RC 未武装、不发流帧, 总线空闲), 不给常态总线加负担。 */
#define RC_STILL_SAMPLE_MS   200U
#define RC_STILL_NEED_MS     2000U
#define RC_STILL_TOL_DEG     0.5f
static uint32_t rc_still_last_ms  = 0U;
static float    rc_still_prev[6]  = {0};
static uint8_t  rc_still_prev_vld = 0U;
static uint32_t rc_still_accum_ms = 0U;

/* 0924R: 双击 START 武装 / 双击 SELECT 锁定。
 * 用户显式锁定后, 禁用 0924Q 的自动重武装, 直到下次双击 START 或 rc go on。 */
static uint8_t  rc_user_locked      = 0U;
#define RC_DBL_START_CODE   4U     /* START */
#define RC_DBL_SELECT_CODE  1U     /* SELECT */
#define RC_DBL_CLICK_MS     400U   /* 两次按下间隔阈值 */
static uint32_t rc_dbl_start_t      = 0U;
static uint8_t  rc_dbl_start_cnt    = 0U;
static uint32_t rc_dbl_select_t     = 0U;
static uint8_t  rc_dbl_select_cnt   = 0U;

static uint16_t rc_btn_bit(uint8_t code) {
    return (code >= 1U && code <= ARM_RC_NBTN)
               ? (uint16_t)(1U << (uint8_t)(code - 1U)) : 0U;
}

static void rc_button_service(void) {
    uint32_t now  = HAL_GetTick();
    uint16_t cur  = ArmRc_Buttons();

    /* ---- 0924Q: home/fold/play 后自动恢复 RC 武装 ----
     * 这些动作类按键会主动 disarm RC; 运动完成后若 RC 仍有效且外部轨迹全部
     * 结束, 自动 re-arm, 用户不用再次手动 rc go on。
     * ⭐0924AS 重写触发条件: 用户要求"不是等 5/8 秒定时, 而是臂静止 2s 就
     * 重武装"。改为: 运动生产者(轨迹/回放/点位/摆动/校正)活跃时每拍顺延
     * 死线; 运动链一结束就以"六轴实读静止累计 2s"为触发。settle(等反馈)/
     * correction 已开时的 correction_pending 仍算忙 —— 它们可能补发运动。
     * 死线 8s 保留为兜底(摇杆没回中时每拍重试, 真超时才放弃)。 */
    if (rc_auto_rearm_until != 0U && !rc_user_locked) {
        uint8_t motion_busy = Trajectory_IsActive() || teach_play_state != TEACH_PLAY_NONE ||
                              point_loop_active || swing_active || swing_resume_pending ||
                              qlc_active || safe_seq_active ||
                              (correction_enabled && correction_pending);
        if (motion_busy) {
            rc_auto_rearm_until = now + 8000U;   /* 运动链未结束, 每拍顺延 */
            rc_still_prev_vld = 0U;              /* 运动中: 静止计时作废 */
            rc_still_accum_ms = 0U;
            /* ⭐0924AH: 调试日志，每秒打印一次当前阻塞自动重武装的原因 */
            static uint32_t last_rearm_block_log = 0U;
            if ((int32_t)(now - last_rearm_block_log) >= 1000) {
                last_rearm_block_log = now;
                RLOG_W("RCCTL", "rearm blocked: traj=%d teach=%d point=%d swing=%d "
                       "qlc=%d safe=%d corr=%d",
                       (int)Trajectory_IsActive(),
                       (int)(teach_play_state != TEACH_PLAY_NONE),
                       (int)point_loop_active, (int)swing_active,
                       (int)qlc_active, (int)safe_seq_active,
                       (int)(correction_enabled && correction_pending));
            }
        } else {
            /* 0924AS: 静止检测 —— 200ms 采样一次, 全轴位移 <=0.5° 计入累计 */
            uint32_t still_dt = now - rc_still_last_ms;
            if (!rc_still_prev_vld) {
                rc_still_prev_vld = 1U;
                rc_still_accum_ms = 0U;
                for (uint8_t j = 0U; j < 6U; j++) {
                    rc_still_prev[j] = SERVO_GetPosition(j);
                }
                rc_still_last_ms = now;
            } else if (still_dt >= RC_STILL_SAMPLE_MS) {
                float moved = 0.0f;
                for (uint8_t j = 0U; j < 6U; j++) {
                    float v = SERVO_GetPosition(j);
                    if (v >= 0.0f) {          /* 丢读不参与比较, 不当"动了" */
                        float d = fabsf(v - rc_still_prev[j]);
                        if (d > moved) moved = d;
                        rc_still_prev[j] = v;
                    }
                }
                rc_still_last_ms = now;
                rc_still_accum_ms = (moved <= RC_STILL_TOL_DEG)
                                        ? (rc_still_accum_ms + still_dt) : 0U;
            }
            if (rc_still_prev_vld && rc_still_accum_ms >= RC_STILL_NEED_MS &&
                rc_auto_rearm_req && !IsEStop()) {
                stream_follow_en = 1U;   /* 0924S: 保证 re-arm 不被 sf=0 挡住 */
                ArmRcCtrl_SetArmed(1U);
                if (ArmRcCtrl_IsArmed()) {
                    rc_auto_rearm_until = 0U;
                    rc_auto_rearm_req   = 0U;
                    rc_still_prev_vld   = 0U;
                    cprint("  [keys] 臂静止 2s, RC 自动重武装\r\n");
                }
            } else if ((int32_t)(now - rc_auto_rearm_until) >= 0) {
                rc_auto_rearm_until = 0U;   /* 运动结束后 8s 内没等到静止 2s, 放弃 */
                rc_auto_rearm_req   = 0U;
                rc_still_prev_vld   = 0U;
                cprint("  [keys] 自动重武装超时: 请双击 START 手动武装\r\n");
            }
        }
    }

    /* ---- keysw 可视化: 键位图变化即打印 (非阻塞, 窗口到期自动关) ---- */
    if (rc_keysw_until != 0U) {
        if ((int32_t)(now - rc_keysw_until) >= 0) {
            rc_keysw_until = 0U;
            cprint("  [keys] 监视窗口结束 (再开: rc keysw [秒])\r\n");
        } else if (cur != rc_keysw_prev) {
            uint16_t chg = (uint16_t)(cur ^ rc_keysw_prev);
            cprint("  [keys]");
            for (uint8_t b = 0U; b < ARM_RC_NBTN; b++) {
                if (chg & (1U << b)) {
                    cprint(" %s=%u", ArmRc_BtnName((uint8_t)(b + 1U)),
                           (unsigned)((cur >> b) & 1U));
                }
            }
            cprint("\r\n");
            rc_keysw_prev = cur;
        }
    }

    /* ---- 0924R: 双击 START 武装 / 双击 SELECT 锁定 ---- */
    uint16_t edges = ArmRc_BtnEdges();

    if (edges & rc_btn_bit(RC_DBL_START_CODE)) {
        if (rc_dbl_start_cnt == 0U ||
            (uint32_t)(now - rc_dbl_start_t) > RC_DBL_CLICK_MS) {
            rc_dbl_start_cnt = 1U;
        } else {
            rc_dbl_start_cnt++;
        }
        rc_dbl_start_t = now;
        if (rc_dbl_start_cnt >= 2U && !ArmRcCtrl_IsArmed()) {
            ArmRcCtrl_SetArmed(1U);
            if (ArmRcCtrl_IsArmed()) {
                rc_user_locked      = 0U;
                rc_dbl_start_cnt    = 0U;
                cprint("  [keys] 双击 START: RC 武装, 摇杆/按键生效\r\n");
            }
        }
    }

    if (edges & rc_btn_bit(RC_DBL_SELECT_CODE)) {
        if (rc_dbl_select_cnt == 0U ||
            (uint32_t)(now - rc_dbl_select_t) > RC_DBL_CLICK_MS) {
            rc_dbl_select_cnt = 1U;
        } else {
            rc_dbl_select_cnt++;
        }
        rc_dbl_select_t = now;
        if (rc_dbl_select_cnt >= 2U && ArmRcCtrl_IsArmed()) {
            ArmRcCtrl_SetArmed(0U);
            rc_user_locked       = 1U;
            rc_dbl_select_cnt    = 0U;
            cprint("  [keys] 双击 SELECT: RC 锁定, 摇杆/按键全部失效\r\n");
        }
    }

    /* ---- 未武装时, 除 START/SELECT 双击外, 其余按键/摇杆全部无效 ---- */
    if (!ArmRcCtrl_IsArmed()) return;

    const uint8_t* mp = ArmRc_MapGet();

    /* 中位: 六轴回 180° (S 曲线 v=15, 与 `m 180 ... 15` 同一条安全链) */
    if (mp[ARM_RC_FN_HOME] != 0U && (edges & rc_btn_bit(mp[ARM_RC_FN_HOME]))) {
        if (IsEStop()) {
            cprint("  [keys] 中位: E-STOP 激活, 拒绝\r\n");
        } else if (Trajectory_IsActive() || teach_play_state != TEACH_PLAY_NONE) {
            cprint("  [keys] 中位: 有运动/回放进行中, 拒绝\r\n");
        } else {
            /* ⭐0924AG: req 无条件设置 —— RC 未 armed 时按键(如刚上电)也要能
             * 在运动完成后自动恢复; 双击 SELECT 锁定的用户由 rc_user_locked 挡住 */
            rc_auto_rearm_req   = 1U;
            rc_auto_rearm_sf    = stream_follow_en;
            rc_auto_rearm_until = now + 8000U;
            if (ArmRcCtrl_IsArmed()) ArmRcCtrl_SetArmed(0U);
            float mid[6] = { 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f };
            if (ArmApi_MoveJoints(mid, 15.0f, 0.0f) == ARM_API_OK) {
                cprint("  [keys] 中位: 六轴回 180 (v=15)\r\n");
            } else {
                cprint("  [keys] 中位: 移动被拒 (看上一行原因)\r\n");
            }
        }
    }

    /* 折叠位: POSE_HOME (与 `pose 0` 同一条路径) */
    if (mp[ARM_RC_FN_FOLD] != 0U && (edges & rc_btn_bit(mp[ARM_RC_FN_FOLD]))) {
        if (IsEStop()) {
            cprint("  [keys] 折叠位: E-STOP 激活, 拒绝\r\n");
        } else if (Trajectory_IsActive() || teach_play_state != TEACH_PLAY_NONE) {
            cprint("  [keys] 折叠位: 有运动/回放进行中, 拒绝\r\n");
        } else {
            /* ⭐0924AG: req 无条件设置 (同中位) */
            rc_auto_rearm_req   = 1U;
            rc_auto_rearm_sf    = stream_follow_en;
            rc_auto_rearm_until = now + 8000U;
            if (ArmRcCtrl_IsArmed()) ArmRcCtrl_SetArmed(0U);
            cprint("  [keys] 折叠位:\r\n");
            cmdPose(0U);
        }
    }

    /* 示教录制: 开/停切换。与 RC 驱臂兼容 (record-only 不写任何舵机寄存器,
     * 录的是 RC 实际走出来的轨迹); 停止后可用回放键直接回放,
     * 或 `teach save <名>` 存盘 (掉电保留)。 */
    if (mp[ARM_RC_FN_TEACH_REC] != 0U &&
        (edges & rc_btn_bit(mp[ARM_RC_FN_TEACH_REC]))) {
        if (teach_play_state != TEACH_PLAY_NONE) {
            cprint("  [keys] 回放进行中, 不能录 (先按回放键停止)\r\n");
        } else if (teach_mode_active) {
            cmdTeachExit();
            cprint("  [keys] 录制停止 (%lu 样本, ~%lu s)。回放键=回放; "
                   "teach save <名>=存盘\r\n",
                   (unsigned long)teach_rec_cnt,
                   (unsigned long)((teach_rec_cnt * TEACH_REC_MS) / 1000U));
        } else if (IsEStop()) {
            cprint("  [keys] 录制: E-STOP 激活, 拒绝\r\n");
        } else {
            cmdTeachEnterRec();
            cprint("  [keys] 示教录制开启 —— 用摇杆遥控走轨迹, 再按录制键停\r\n");
        }
    }

    /* 回放: 停录制 + 解除 RC + 回放刚录的轨迹; 回放中再按 = 停止回放 */
    if (mp[ARM_RC_FN_TEACH_PLAY] != 0U &&
        (edges & rc_btn_bit(mp[ARM_RC_FN_TEACH_PLAY]))) {
        if (teach_play_state != TEACH_PLAY_NONE) {
            teach_play_abort(1U);
        } else if (teach_rec_cnt < 2U) {
            cprint("  [keys] 没有可回放的录音 (先按录制键, 摇杆走一段)\r\n");
        } else if (IsEStop()) {
            cprint("  [keys] 回放: E-STOP 激活, 拒绝\r\n");
        } else {
            /* ⭐0924AG: req 无条件设置 (同中位) */
            rc_auto_rearm_req   = 1U;
            rc_auto_rearm_sf    = stream_follow_en;
            rc_auto_rearm_until = now + 8000U;
            if (ArmRcCtrl_IsArmed()) ArmRcCtrl_SetArmed(0U);
            cmdTeachPlay(0U);
        }
    }
}

/* ===== B方案: 位置环导纳软跟随 (零力示教) =====
 * 保持 Mode 0 位置伺服, 但让 GoalPosition 每 20ms 紧跟 PresentPosition。
 * 位置误差≈0 → 舵机不会主动输出力矩去"拉回"某个固定点,
 * 人手只感受到齿轮摩擦和惯性, 从而能轻松拖动。
 * 同时用 teach_off_raw 对慢速漂移积分, 自动建立重力配重。 */
static void teach_assist_write(const uint16_t raw[6]) {
    if (!teach_assist_active) return;

    uint16_t goal[6];

    for (uint8_t j = 0; j < 6; j++) {
        /* 1) 速度计算 & 自动重力配重:
         *    vel = present - last; |vel| 很小(1~5 raw/tick)说明臂在重力下缓慢漂移,
         *    此时给 virtual_goal 一个反向偏移来抵消重力。
         *    |vel| 较大说明人在主动拖动, 冻结配重防止"推背感"。 */
        int vel = (int)raw[j] - (int)teach_assist_last_raw[j];
        int mv = vel; if (mv < 0) mv = -mv;
        teach_assist_last_raw[j] = raw[j];

        if (teach_int_gain > 0U) {
            int o = (int)teach_off_raw[j];
            if (mv >= 1 && mv < 6) {
                /* 慢速漂移: 积分建立配重 (符号与 vel 相反) */
                o -= vel * (int)teach_int_gain;
            } else if (mv >= 6) {
                /* 主动拖动: 缓慢回收偏移, 避免反向推力 */
                if (o > 0) o--; else if (o < 0) o++;
            }
            if (o >  (int)TEACH_FOLLOW_OFF_MAX) o =  (int)TEACH_FOLLOW_OFF_MAX;
            if (o < -(int)TEACH_FOLLOW_OFF_MAX) o = -(int)TEACH_FOLLOW_OFF_MAX;
            teach_off_raw[j] = (int16_t)o;
        }

        /* 2) virtual_goal 紧跟 present (死区 + 限速):
         *    计算 present+配重 与当前 virtual_goal 的偏差。
         *    偏差很小(≤死区)时不更新 goal, 抑制反馈噪声导致的微振。
         *    偏差较大时按限速步进跟随, 避免一次通信/读数突跳导致急加速。 */
        int target = (int)raw[j] + (int)teach_off_raw[j];
        int err = target - (int)teach_assist_goal_raw[j];
        if (err < 0) err = -err;

        if (err <= (int)TEACH_FOLLOW_DEADBAND) {
            goal[j] = teach_assist_goal_raw[j];
        } else {
            int step = target - (int)teach_assist_goal_raw[j];
            if (step >  (int)TEACH_FOLLOW_STEP_MAX) step =  (int)TEACH_FOLLOW_STEP_MAX;
            if (step < -(int)TEACH_FOLLOW_STEP_MAX) step = -(int)TEACH_FOLLOW_STEP_MAX;
            int g = (int)teach_assist_goal_raw[j] + step;
            if (g < 0) g = 0;
            if (g > 4095) g = 4095;
            goal[j] = (uint16_t)g;
            teach_assist_goal_raw[j] = goal[j];
        }
    }

    SERVO_SetPositionsRawSync(goal);
}

/* ===== PWM 模式重力补偿写入 (每 20ms) =====
 * 2026-09-08 修正版: 占空比写 addr44 (GoalTime) + bit10 方向 (官方 SDK 编码)。
 * duty = flip × DIR × gain × (重力杠杆, 带符号) - damp × joint 速度
 * 重力前馈抵消自重; 阻尼抑制摆动; 拖动只对抗摩擦 (无位置环短路制动)。
 * 2026-09-09 模型升级 (CSDN 重力补偿文章思路): 受力分析与雅可比/虚功法
 * 数学等价 (τ_j = Σ m_i·g·∂x_ci/∂q_j), 精度差别在参数——旧"末端杠杆"
 * 假设全部质量在各杆末端且等质量; 正确形态 = 质量加权质心杠杆:
 *   τ_J2 ∝ m_A·xc_A + m_B·xc_B + m_C·xc_C   (xc = 各段质心水平坐标)
 *   τ_J3 ∝ m_B·(xc_B-x_J3) + m_C·(xc_C-x_J3)
 *   τ_J4 ∝ m_C·(xc_C-x_J4)
 * 段质量取 SO-101 官方 URDF (我们构型同 SO-101, 同为 STS3215): */
#define TC_M_UPPER   0.203f   /* 肩壳 0.100 + 上臂 0.103 kg */
#define TC_M_FORE    0.104f   /* 前臂 0.104 kg */
#define TC_M_WRIST   0.178f   /* 腕 0.079 + 夹爪 0.087+0.012 kg */
#define TC_CG_UPPER  0.50f    /* 各段质心系数 (×段长, 中点假设, 可辨识微调) */
#define TC_CG_FORE   0.50f
#define TC_CG_WRIST  0.45f    /* 腕重夹爪轻 → 等效质心略偏近端 */
#define TC_MTOT      (TC_M_UPPER + TC_M_FORE + TC_M_WRIST)
static uint8_t teach_pwm_model = 1U;   /* 1=质量加权质心杠杆(默认) 0=旧末端杠杆 */
static void teach_pwm_write(const uint16_t raw[6]) {
    float deg[6], rad[6];
    for (uint8_t j = 0; j < 6; j++) {
        deg[j] = servo_raw_to_deg(raw[j]);
        rad[j] = servo_deg_to_rad(deg[j], j);
    }
    float sB = sinf(ROBOT_PSI_FORE0 + rad[1] + rad[2]);
    float sC = sinf(rad[1] + rad[2] + rad[3]);
    float sA = sinf(ROBOT_PSI_UPPER0 + rad[1]);
    /* 带符号重力杠杆 (J1 绕竖直轴, 无重力矩)。
     * model=1: 质量加权质心杠杆 (受力分析/雅可比正确形态), 输出归一化
     *          到"等效 mm", gain 语义不变 (×0.1 duty/mm);
     * model=0: 旧末端杠杆 (保留便于对比)。 */
    float ff[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    if (teach_pwm_model != 0U) {
        float x3 = ROBOT_L_UPPER * sA;
        float x4 = x3 + ROBOT_L_FORE * sB;
        float cA = TC_CG_UPPER * ROBOT_L_UPPER * sA;         /* 肩+上臂段质心 */
        float cB = x3 + TC_CG_FORE * ROBOT_L_FORE * sB;      /* 前臂段质心 */
        float cC = x4 + TC_CG_WRIST * ROBOT_L_DISTAL * sC;   /* 腕+夹爪段质心 */
        ff[1] = (TC_M_UPPER * (cA - 0.0f) + TC_M_FORE * (cB - 0.0f) +
                 TC_M_WRIST * (cC - 0.0f)) / TC_MTOT;
        ff[2] = (TC_M_FORE * (cB - x3) + TC_M_WRIST * (cC - x3)) / TC_MTOT;
        ff[3] = (TC_M_WRIST * (cC - x4)) / TC_MTOT;
    } else {
        ff[1] = ROBOT_L_UPPER * sA + ROBOT_L_FORE * sB + ROBOT_L_DISTAL * sC;
        ff[2] = ROBOT_L_FORE * sB + ROBOT_L_DISTAL * sC;
        ff[3] = ROBOT_L_DISTAL * sC;
    }

    for (uint8_t j = 0; j < 6; j++) {
        /* J1/J5/J6 在混合模式下 Torque OFF (真 free), 不写 duty */
        if (j == 0U || j == 4U || j == 5U) continue;

        int vel = (int)raw[j] - (int)teach_assist_last_raw[j];
        teach_assist_last_raw[j] = raw[j];

        /* 单轴标定模式 (teach test J): 非目标轴 duty 归 0 (走 ±2 dither,
         * 等效 free), 只标定一个轴, 排除参数互相干扰。 */
        if (teach_test_joint != 0 && teach_test_joint != (int)(j + 1)) {
            (void)SERVO_SetPwmDuty(j, 0);
            continue;
        }

        int duty = (int)(teach_pwm_flip[j] * JOINT_DIR[j] *
                         teach_pwm_gain[j] * ff[j] / 10.0f);
        duty += teach_pwm_off[j];                      /* 恒定偏置 (J5/J6 偏心力矩) */
        duty -= vel * teach_pwm_damp[j];               /* 速度阻尼(原始空间) */
        /* 库伦摩擦补偿 (零力控制第二半): 顺运动方向补 Fc 抵消齿轮摩擦 */
        if (teach_fc_duty[j] > 0 && (vel > 0 || vel < 0))
            duty += (vel > 0) ? teach_fc_duty[j] : -teach_fc_duty[j];
        /* 交变预紧 dither (论文: 静摩擦启动难解法): ±d 方波破静摩擦 */
        if (teach_dither[j] > 0) {
            duty += (((teach_dith_ph++ >> 2) & 1U) != 0U) ? teach_dither[j]
                                                          : -teach_dither[j];
        }

        /* Per-joint duty 限幅 (2026-09-09 可调): J3 默认 ±40 防损伤共振,
         * 但大负载姿态下会托不住 → teach cap 3 80 可放宽, 共振与下坠间权衡。 */
        int cap = teach_pwm_cap[j];
        if (duty >  cap) duty =  cap;
        if (duty < -cap) duty = -cap;

        /* 限位边缘保护: 任何关节靠近限位边界(<40 raw)时进一步压到 ±30 */
        int edge_margin = 40;
        int dist_low  = (int)raw[j] - (int)JOINT_MIN_RAW[j];
        int dist_high = (int)JOINT_MAX_RAW[j] - (int)raw[j];
        if (dist_low < 0) dist_low = -dist_low;
        if (dist_high < 0) dist_high = -dist_high;
        uint8_t near_edge = (dist_low < edge_margin || dist_high < edge_margin) ? 1U : 0U;
        if (near_edge) {
            if (duty >  30) duty =  30;
            if (duty < -30) duty = -30;
        }

        /* 死区避开 (2026-09-08): PWM 驱动芯片在 |duty|<deadband 时进入死区,
         * 外力拖动产生自激振荡(疯狂甩动)。推到 deadband 外可消除。
         * duty=0 时给一个极小的交替 dither(平均力矩≈0)保持芯片活跃。 */
        if (duty == 0) {
            duty = (((teach_dith_ph >> 1) & 1U) != 0U) ? 2 : -2;
        } else if (duty > 0 && duty < teach_pwm_db) {
            duty = teach_pwm_db;
        } else if (duty < 0 && duty > -teach_pwm_db) {
            duty = -teach_pwm_db;
        }
        (void)SERVO_SetPwmDuty(j, (int16_t)duty);
    }
}

/* 50 Hz tick while teach mode is active: append one snapshot sample.
 *
 * 2026-09-16 改版 (/bug: 回放"中后段没复刻出来"):
 *  旧版任何一轴读失败 -> 整个 return, 那一拍样本**不记**, 而且旧超时是
 *  100ms (一次失败=5 个节拍消失) ⇒ 录音里被挖出一个"时间空洞", 回放时
 *  那一段就变成一条直线抄近路, 后面照常 1:1 重放所以尾段看着是对的。
 *  现在: 短超时 + 单轴失败沿用上一拍的值, 保证**每个节拍都记一个样本**,
 *  时间轴不再被打断 (20ms 的微保持肉眼不可见)。 */
static void teach_record_tick(void) {
    if (!teach_mode_active) return;

    uint16_t raw[6];
    for (uint8_t j = 0; j < 6; j++) {
        raw[j] = SERVO_GetPositionRawTO(j, TEACH_READ_TO_MS);
        if (raw[j] != 0xFFFFU) continue;
        teach_read_fail[j]++;
        if (teach_rec_cnt == 0U) {
            /* 首拍没有"上一拍"可沿用: 整拍放弃。
             * ⚠ 绝不能塞假值(如 2048) —— 它会被同时喂给悬浮控制(把 goal 拉到中位,
             * 舵机会真的往那儿走)和录音(变成回放起点)。宁可这一拍不记。 */
            teach_smp_dropped++;
            return;
        }
        raw[j] = teach_rec[teach_rec_cnt - 1U][j];   /* 沿用上一拍, 不整拍丢 */
        teach_smp_held++;
    }

    /* 悬浮/PWM 控制每 tick 都要跑, 否则臂失去托举 —— 放在写满判定【之前】:
     * 旧版写满(4000点/80s)时先 return, 悬浮托举会**突然消失**。
     * (例外: 上面"首拍读失败"那条 return 会连带跳过本拍控制, 只发生在起手,
     *  此时 goal 还等于 present, 舵机原地保持, 安全。) */
    if (teach_assist_active)   teach_assist_write(raw);
    else if (teach_pwm_active) teach_pwm_write(raw);

    if (teach_rec_cnt >= TEACH_REC_MAX) {
        if (!teach_rec_full) {
            teach_rec_full = 1U;
            cprint("  Teach buffer full (~%lu s). 'teach exit' to stop.\r\n",
                   (unsigned long)((TEACH_REC_MAX * TEACH_REC_MS) / 1000U));
        }
        return;
    }
    for (uint8_t j = 0; j < 6; j++) teach_rec[teach_rec_cnt][j] = raw[j];
    teach_rec_cnt++;
}

/* 录音完整性报告: 只在有异常时打, 免得刷屏 */
static void teach_report_integrity(void) {
    if (teach_tick_lost == 0U && teach_smp_held == 0U && teach_smp_dropped == 0U) return;
    cprint("  [rec-integrity] lost_ticks=%lu held_axes=%lu dropped=%lu  read-fail J1..J6=%u %u %u %u %u %u\r\n",
           (unsigned long)teach_tick_lost, (unsigned long)teach_smp_held,
           (unsigned long)teach_smp_dropped,
           teach_read_fail[0], teach_read_fail[1], teach_read_fail[2],
           teach_read_fail[3], teach_read_fail[4], teach_read_fail[5]);
    if (teach_tick_lost > 0U)
        cprint("    (lost_ticks>0 = 主循环有 >20ms 的卡顿, 时间轴被压缩 %lums)\r\n",
               (unsigned long)(teach_tick_lost * TEACH_REC_MS));
    if (teach_smp_held > 0U)
        cprint("    (held_axes>0 = 有轴读到 0xFFFF, 已用上一拍值顶替)\r\n");
    if (teach_smp_dropped > 0U)
        cprint("    (dropped>0 = 起手就没读到, 那几拍没记)\r\n");
}

/* 50 Hz tick while a replay is streaming: send one recorded sample.
 * Source is either the RAM scratch (teach_rec) or a saved flash file. */
static void teach_stream_tick(void) {
    if (teach_play_state != TEACH_PLAY_STREAM) return;
    if (teach_play_idx < teach_play_src_cnt) {
        SERVO_SetPositionsRawSync(teach_play_src[teach_play_idx]);
        teach_play_idx++;
        if (teach_play_idx >= teach_play_src_cnt) {
            if (teach_play_loop) {
                cprint("  Loop restart.\r\n");
                teach_play_idx = 0U;   /* next tick re-commands sample 0 */
            } else {
                teach_play_finish();
            }
        }
    }
}

/* ===== link: 链路偏好 (2026-09-22) ==================================
 *   link              显示当前偏好与物理约束
 *   link can|net|off  改偏好(只改 RAM)
 *   link save         落盘(Flash 扇区 11 的区 C, 保区写入)
 * ⚠ 切换**必须重启才生效**: 引脚 AF(PB13 / PC4 / PC5) 与外设都是启动时一次
 *   配好的, 运行期改一个变量挪不动它们。本命令只负责"改偏好 + 落盘 + 说清
 *   楚什么时候生效", 绝不给"已经切过去了"的假象。 */
static void cmdLink(int ntok, char** tokens)
{
#if ARM_LINK_MULTI
    const ArmLinkCfg* c = LinkCfg_Get();

    if (ntok == 1) {
        cprint("\r\n  ===== 链路偏好 (link) =====\r\n");
        cprint("  本次生效 : %s   (串口恒通)%s\r\n", LinkCfg_NameOf(LinkCfg_ActiveMode()),
               LinkCfg_AutoEnabled() ? "  [auto 动态: 静默=off]" : "");
        cprint("  下次开机 : %s\r\n", LinkCfg_NameOf(c->mode));
        cprint("  配置来源 : %s\r\n",
               LinkCfg_LoadedFromFlash() ? "Flash 记录(扇区11 区C)"
                                         : "**未配置 -> 默认 auto**(0924CF 起开机即双监听)");
        cprint("  待生效   : %s\r\n",
               LinkCfg_IsSaved() ? "无改动" : "**有未保存的改动** -> 用 `link save`");
        cprint("  串口     : USART6 PC6/PC7 @115200 -- **永远常通**\r\n");
        cprint("  硬件现状 : CAN(PB12/13) 与网口(RMII PG组+PC4/5)**无引脚冲突**;\r\n");
        cprint("             PC4/PC5 让位后 LED 不可用。auto 模式两条同时初始化(0924CE):\r\n");
        cprint("             谁发合法包谁 active, %us 无包自动静默(仍监听)。\r\n",
               (unsigned)(LINK_AUTO_TIMEOUT_MS / 1000U));
        cprint("  用法     : `link can|net|off|auto` 改偏好 -> `link save` 落盘 -> `rb` 生效\r\n");
        cprint("  ============================\r\n\r\n");
        return;
    }

    if (strcmp(tokens[1], "save") == 0) {
        cprint("\r\n  [LINK] 写入 Flash(扇区 11 区 C; 保区写入, 不动 PID/CAN 两区) ...\r\n");
        if (LinkCfg_Save() != 0) {
            cprint("  [LINK] **写入失败** -- Flash 未改变, 偏好仍是 %s。\r\n\r\n",
                   LinkCfg_NameOf(c->mode));
            return;
        }
        cprint("  [LINK] 已保存: %s\r\n", LinkCfg_NameOf(c->mode));
        cprint("  ⚠ **下次开机生效** -- 现在敲 `rb`(软复位, 0924CF) 或重新上电。\r\n\r\n");
        return;
    }

    {
        uint8_t m;
        if (LinkCfg_Parse(tokens[1], &m) != 0) {
            cprint("\r\n  [LINK] 无法识别 '%s'。用法: link [can|net|off|auto|save]\r\n\r\n",
                   tokens[1]);
            return;
        }
        (void)LinkCfg_Set(m);
        cprint("\r\n  [LINK] 偏好已改为 %s (仅 RAM, **未落盘**, 也**不会立即生效**)。\r\n",
               LinkCfg_NameOf(m));
        if (m == LINK_MODE_AUTO) {
            cprint("         auto = 开机后 CAN+网口都初始化、都在监听;\r\n");
            cprint("         谁发合法包谁 active, %us 无包自动静默。未配置默认=auto,\r\n",
                   (unsigned)(LINK_AUTO_TIMEOUT_MS / 1000U));
            cprint("         auto 必须显式 `link save` 才会启用。\r\n");
        }
        cprint("         执行 `link save` 落盘, 再 `rb` 软复位才生效。\r\n\r\n");
    }
#else
    /* 单链路构建: 明说"不能切" —— 别让用户以为敲了会有用。
     * (这是"静默无操作"的反面, 与红线 66 同源: 功能没开就要说出来。) */
    (void)ntok; (void)tokens;
    cprint("\r\n  [LINK] 本固件是**单链路构建**, 不支持运行期切换。\r\n");
#if ARM_LINK_NET
    cprint("         编译期链路 = **net** (LINK=net)\r\n");
#elif ARM_LINK_CAN
    cprint("         编译期链路 = **can** (LINK=can)\r\n");
#else
    cprint("         编译期链路 = **uart** (主从链路, 无 CAN / 无网口)\r\n");
#endif
    cprint("         要能运行期切换, 请烧 LINK=both 固件。\r\n\r\n");
#endif
}

static void cmdHelp(void) {
    /* ⚠ 首行是**固件指纹**, 给上位机脚本做"烧的是不是新版"前置检查用
     *   (见 tools/can_pin_hunt.py 的 preflight)。改动它等于改版本号, 别顺手删。 */
#if ARM_MASTER_BUILD
    /* 主臂构建 (ROLE=master, 2026-10-30 / 11-02 改线): 只读六轴 + 经 UART4 转发
     * 0x13 流帧给从臂, 不跑轨迹/不禁拖拽。⚠ 舵机总线已从 UART5(PC12/PD2) 改接
     * **USART3(PB10/PB11)**, 6 颗舵机现全部并联在这一条开漏单总线上; UART5 旁路。
     * ⚠ 主臂默认 LINK=uart, 会掉进下面 CAN 那个兜底分支打出 ARM-CAN-0918H
     *   ⇒ 现场误判"烧错固件"。所以主臂必须有自己的指纹。 */
    cprint("\r\n  fw-tag ARM-MASTER-1030E build=ROLE=master  uart4=115200  "
           "usart6=console(always-on)\r\n");
    cprint("  role=master: 6 servo on USART3 (PB10/PB11 open-drain single bus), "
           "read-only + forward 0x13 to slave\r\n");
    cprint("  src=common tree (ARM_MASTER_BUILD=1)  dirlink=UART4 PA0/PA1 (must cross)\r\n");
#elif ARM_LINK_MULTI
    /* both 构建(2026-09-22): 首行打**本构建专属指纹** + 运行期链路状态,
     * 然后**复用下面的通用帮助正文** —— 所以通用分支的条件写成
     * `ARM_LINK_NET && !ARM_LINK_MULTI`, 把 NET 那 20 行探针帮助挡掉。
     * ⚠ 别把 NET 专属帮助塞进 both: 它是 LINK=net 专属的, 在 CAN/OFF 模式下
     *   `nx/np/nm/nt` 那些命令根本不存在, 打出来只会误导排查。
     * ⚠ 同样受红线 5 约束: 每条 cprint ≤255 字节, 超了会被静默截断。 */
    cprint("\r\n  fw-tag ARM-BOTH-0924DE build=LINK=both  usart6=console(always-on)"
#if defined(BUS_ONE) && BUS_ONE
           "  bus=PP+1Y3Y(1-wire)  map=fixed(B)"
#elif defined(BUS_PP) && BUS_PP
           "  bus=PP(74HC126)  map=auto"
#else
           "  bus=OD(1k->3V3)  map=auto"
#endif
           "\r\n");
    cprint("  link active=%s  pref=%s%s\r\n",
           LinkCfg_NameOf(LinkCfg_ActiveMode()),
           LinkCfg_NameOf(LinkCfg_Get()->mode),
           LinkCfg_LoadedFromFlash() ? "" : " (no flash cfg -> OFF)");
    cprint("  'link' show status | 'link can|net|off|auto' set & save | 'rb' to apply\r\n");
    cprint("  fix=both-link,link-cfg-flash,multizone-guard,pc45-runmode,led-runmode,"
           "default-auto,cr-lit-fix,ph-cal,rc-cfg-raw,cu-glitch-id,msb-stick-filt,phase-auto,mode-gate\r\n");
#endif

    /* ↓ 通用帮助正文; 只有 LINK=net **单链路**构建才走 NET 专属那一支
     *   (both 构建里 ARM_LINK_NET 也是 1, 必须再与 !ARM_LINK_MULTI 相与,
     *    否则会把 NET 那 20 行探针帮助打进 CAN/OFF 模式里)。 */
#if ARM_LINK_NET && !ARM_LINK_MULTI
    /* ⚠ 这一段必须拆成**每条 cprint 都 ≤255 字节**: cprint 用 vsnprintf 钳位,
     *   整行拼接超过 255 会被**截断**(且不报错)。0920P 的 fix 清单加到 20 项后
     *   整行 368 B ⇒ 现场 `h` 只能看到 `...np-text-fix,ne` 就断了,
     *   连"到底烧进去没有"都判不出来(2026-09-21 实机踩到)。
     *   以后加 token 请照抄这个"每行 ≤255"的拆法。 */
    cprint("\r\n  fw-tag ARM-NET-0924DE phy=LAN8720A(RMII)  udp=192.168.7.120:5000"
           "  build=LINK=net  txpins=PG11/PG13/PG14"
#if defined(BUS_ONE) && BUS_ONE
           "  bus=PP+1Y3Y(1-wire)  map=fixed(B)"
#elif defined(BUS_PP) && BUS_PP
           "  bus=PP(74HC126)  map=auto"
#else
           "  bus=OD(1k->3V3)  map=auto"
#endif
           "\r\n");
    cprint("  fix=net-udp,nocancmds,txpins-pg,mdc-div102,miim-probe32,clkprobe4,selftest,apb1-84m,"
           "init-fail-explain,pc45-led-guard,pa1-pull-probe,np-text-fix,net-master-cmd-block,\r\n");
    cprint("      itr-fix-tim5-itr1,tim4-witness,path-calib,ts-macro-noshift,pe2-nrst,"
           "icmp-cksum,icmp-buf-size,banner-255fix,icmp-diag,txpath-probe,cic-force-bypass,"
           "rc-tim4-capture,rc6ch-t1p167-exti-pg10,ccer-exp-bbbb,\r\n");
    cprint("      rc-r1-ctrl,lim-ram-src,rc-domain-fix,rc-spd-flash,rc-envelope-softstop,"
           "poseok-l2fix,rc-fb-pace,rc-goal-pace,rc-openloop,rc-spd-unit50x,rc-brake,\r\n");
    cprint("      rc-ps2-input,rc-keymap,rc-teach-btn,rc-pitch-btn,rc-diag,rc-pin-probe,"
           "rc-diag-3speed,rc-hwspi,rc-spi-cpha,rc-keymap-l2r2,rc-swap-xy,rc-j5inv,\r\n");
    cprint("      rc-claw-slow,rc-claw-stop,rc-vxy-limit,rc-auto-rearm,rc-dblstart-arm,"
           "rc-intent-fix,rc-j5-max350,rc-j4-btn,rc-arm-sf1,rc-j4-inv,"
           "rc-j4-owner,rc-claw-anchor,rc-act-rescue,\r\n");
    cprint("      rc-rearm-wait,rc-off-cancel,rc-joint-ctrl,rc-j5-speed,rc-j5-pace,"
           "rc-brake-exact,rc-lead-restore,rc-claw-fullspeed,kinematic-cal,link-calib-0928,\r\n");
    cprint("      rc-claw-sym18,rc-rearm-settle,rc-rearm-fix2,rc-settle-timeout,"
           "c-offset-reanchor,rc-floor-recover,traj-follow-rm,traj-settle-smooth,"
           "servo-accel80,traj-accel-sync,servo-accel-split,traj-approach-safe,"
           "rc-stream-hold,traj-simple-run,rc-no-hold\r\n");
    cprint("      servo-accel-single,rc-disarm-spd,m-spd-restore,z-a-traj,"
           "rearm-still,geo-pref,boot-quiet,sid-tool,busmap-persist,\r\n");  /* 0924DA: 拆行, 原 288B 超 255 上限会截断 */
    cprint("      net-an-quiet,busscan-per-route,clamp-near-fix,uncal-guard,clamp-band-fix,lim-cur-err\r\n");
    cprint("      sid-bus-locate,follow-internal,mstzero-flash,fi-monitor,fi-margin,fi-prop-spd,fi-free-gates,fi-stat,err-speed,torque-verify,\r\n");  /* 0924DB: 拆行 */
    cprint("      fi-diag,fi-hold-map,zf-gate,geo-v2,zf-here,zf-num-rm,jlim-flash,margin-rm,t-all,rb-reboot,ph-cal,rc-cfg-raw,cu-glitch-id,msb-stick-filt,phase-auto,mode-gate\r\n");
    cprint("  ⚠ NET 构建: CAN 从站未启用。RMII 发送脚 = PG11(TX_EN)/PG13(TXD0)/PG14(TXD1),\r\n");
    cprint("    PC4/PC5(原 LED)=RXD0/RXD1, PA1/PA2/PA7/PC1 = REF_CLK/MDIO/CRS_DV/MDC。\r\n");
    cprint("    cx/ps/can/cp/cy/cz/cw 已被**显式拦掉**: 它们会把 PB12/PB13 重配成\r\n");
    cprint("    AF9_CAN2 并拉起 CAN 外设 —— 与本构建无关, 却会白白混淆排查\r\n");
    cprint("    (而且首字母兜底会让 `cx` 掉进校准向导并松力矩), 所以直接挡住并说明。\r\n");
    cprint("    排查用 `nx`(状态) / `nx scan`(重扫 PHY) / `np`(引脚取证)\r\n");
    cprint("    / `nm`(软 MIIM+MDIO 线体检) / `nt`(REF_CLK 时钟体检 v4fix: TS-macro-noshift+TI2FP2+双路径\r\n");
    cprint("    两条独立路径 + 已知 100kHz 自检 + 引脚→TI2 通路标定; 自检不过则整段输出作废)。\r\n");
    cprint("  RC 遥控 (PS2 手柄, 硬件 SPI1: DAT=PB4 CMD=PB5 CLK=PB3 CS=PB1, Mode3):\r\n");
    cprint("    `rc` 总览+寄存器回读 | `rc raw` 单行 | `rc watch [n]` 连看 | `rc mid` 记中位\r\n");
    cprint("    | `rc reset` 清统计 | `rc reinit` 抢回 TIM4(敲过 `nt` 后必须敲)\r\n");
    cprint("    | ⚠ 已非只读: `rc go on` 武装后摇杆驱动关节, 详见 `rc go` / `rc map`\r\n");
#else
    /* ⚠ 与 NET 横幅同规矩: 每条 cprint ≤255 字节, 超了会被 vsnprintf 静默截断。
     *   `rc-tim4-capture` 是本 token: 用来判"手里这份 CAN 包是不是 0922 重编的那一版"
     *   (0922 之前导出的 02_slave_CANslave.bin 没有它 —— 三链路延迟对比要求两轮固件同代次)。 */
    cprint("\r\n  fw-tag ARM-CAN-0918H  pins=CAN1 %s  iface=cx-switchable+cxpa"
           "  defif=CAN2(PB12/PB13)"
           "  fix=cprint-overread,frameseq,tec-latch,defcan2,tecwin500,rc-tim4-capture"
           "  cmds=cx,cp,cy,cz,cw,ps\r\n",
           ARM_CAN_PIN_DESC);
#endif
    /* ⚠ 下面这块是 5406 字节: 必须走 cprint_long, 用 cprint 只会打出前 255 字节
     *   再把栈读出去(2026-09-18 事故, 见 cprint 上方注释)。 */
    cprint_long("\r\n"
           "  +---------------------------------------+\r\n"
           "  |   6-Axis Arm Controller (STS3215)      |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Scan & Config                         |\r\n"
           "  | [n]        Scan buses (A:1-3 B:4-6)   |\r\n"
           "  | [Z]        Full ID scan 1..253 (diag) |\r\n"
           "  | [b 0/1/2]  Bus mode (0=norm 1=allA 2=allB) |\r\n"
           "  | [i]        Init all servos            |\r\n"
           "  | [i J]      Init one joint (J=0~5)     |\r\n"
           "  | [s]        Status (all 6 joints)      |\r\n"
           "  | [c]        Calibrate joint limits     |\r\n"
           "  | [M a]      ⚠⚠ 中心标定(非中位!): 当前姿态写成新零点  |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Single Joint (J=0~5 = J1~J6)         |\r\n"
           "  | [t J 0/1]  Torque off/on              |\r\n"
           "  | [a J deg]  Angle (0~300 deg)          |\r\n"
           "  | [r J 0-4095] Raw position             |\r\n"
           "  | [p J]      Read position              |\r\n"
           "  | [pv ms]    Live angles; pv 0 stops     |\r\n"
           "  | [pf ms]    Live XYZ pose; pf 0 stops   |\r\n"
           "  | [z J]      Go to 0 (calib zero)      |\r\n"
           "  | [z a]      Go to 0, all six          |\r\n"
           "  | [w J]      Sweep test                 |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Coordinated Motion                    |\r\n"
           "  | [m a1..a6] 6-joint move (deg)         |\r\n"
           "  |   opt: [m ... v] with speed (deg/s)   |\r\n"
           "  | [B ...]     Joint A/B loop; rounds=0 inf|\r\n"
           "  | [BG ...]    XYZ A/B loop; rounds=0 inf  |\r\n"
           "  | [q n j1..j6] Save joint point 1~8     |\r\n"
           "  | [qg n x y z p] Save XYZ/Pitch point 1~8|\r\n"
           "  | [qv n v]     Set segment n speed       |\r\n"
           "  | [qd n ms]    Set point n dwell time    |\r\n"
           "  | [QL n r v]   Loop + point/error stats; r=0 inf|\r\n"
           "  | [QLC n r v]  Continuous loop, no stops      |\r\n"
           "  | [P 0~4]    Predefined pose            |\r\n"
           "  |   0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2|\r\n"
           "  | [P here]   Calibrate HOME/fold pose to |\r\n"
           "  |            current (saved in flash)    |\r\n"
           "  | [P clear]  Reset fold pose to default  |\r\n"
           "  | [W n]      Swing n times (default=3)   |\r\n"
           "  | [S]        Stop trajectory            |\r\n"
           "  | Hand Teaching                         |\r\n"
           "  | [teach]      Float teach (auto-record, 托举)\r\n"
           "  | [teach free] Teach with torques OFF        |\r\n"
           "  | [teach rec]  Record-only (servos untouched,\r\n"
           "  |              for PC-relay teleop teaching)  |\r\n"
           "  | [teach exit] Stop record, torque ON         |\r\n"
           "  | [teach k]    Show float gains              |\r\n"
           "  | [teach k k2 k3 k4] Set float gains (x1000) |\r\n"
           "  | [teach s pct] Scale float torque 1~300%%    |\r\n"
           "  | [save <nm>]  Store recording in flash        |\r\n"
           "  | [list]       List saved recordings           |\r\n"
           "  | [play]       Replay unsaved recording        |\r\n"
           "  | [play <nm|#>] Replay a saved file [loop]     |\r\n"
           "  | [del <nm|#>] Delete a saved file             |\r\n"
           "  | [compact]    Reclaim deleted file space      |\r\n"
           "  | [rec clear]  Discard unsaved recording       |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Monitoring                            |\r\n"
           "  | [T]        Show all temperatures      |\r\n"
           "  | [T J]      Show single joint temp     |\r\n"
           "  | [O]        Show all joint loads       |\r\n"
           "  | [O J]      Show single joint load(+/-)|\r\n"
           "  | [E]        Show all hw error status   |\r\n"
           "  | [E J]      Show single joint error    |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Kinematics (FK/IK)                    |\r\n"
           "  | [f]        Show end position (FK)     |\r\n"
           "  | [g x y z]  Move to position (IK, mm)  |\r\n"
           "  |   opt: [g x y z pitch] (deg)          |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Safety                               |\r\n"
           "  | [L]        Show joint limits          |\r\n"
           "  | [L J min max] Set joint limit (deg)   |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Calibration                          |\r\n"
           "  | [C]        Show world offset          |\r\n"
           "  | [C dx dy dz] Set offset (actual-cmd)  |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Tuning                               |\r\n"
           "  | [G]        Show adaptive corr & PID  |\r\n"
           "  | [G k]      Set feedforward K (deg/mm)|\r\n"
           "  | [K]        Show all P-Gains (Kp)     |\r\n"
           "  | [K J val]  Set P-Gain (J=1~6,0~254)  |\r\n"
           "  | [I]        Show all I-Gains (Ki)     |\r\n"
           "  | [I J val]  Set I-Gain (J=1~6,0~254)  |\r\n"
           "  | [D]        Show all D-Gains (Kd)     |\r\n"
           "  | [D J val]  Set D-Gain (J=1~6,0~254)  |\r\n"
           "  | [N J P I D] Set all PID for joint J  |\r\n"
           "  | [V]        Show all-servo speed       |\r\n"
           "  | [V value]  Set all 6 speed (0~4095)   |\r\n"
           "  | [F]        Save PID/correction/speed  |\r\n"
           "  | [X]        Apply firmware default PID |\r\n"
           "  | [A]        Show adaptive config      |\r\n"
           "  | [A e 0/1]  Enable/disable correction |\r\n"
           "  | [A t val]  Set threshold (deg)       |\r\n"
           "  | [A r val]  Set max rounds            |\r\n"
           "  | [A g J v]  Set init gain J (0~100%)  |\r\n"
           "  | [R]        Reset & re-trigger corr   |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Master / PC-relay (official route)   |\r\n"
           "  | [ma 0/1]  Passive/Active             |\r\n"
           "  | [mf 0/1]  Arm link off/on            |\r\n"
           "  | [sp Hz]   Console pos stream         |\r\n"
           "  | [ms]      Master + stream cnt        |\r\n"
           "  | [#P ...]  Slave: relay pos line      |\r\n"
           "  | [sf 0/1]  Slave: follow on/off       |\r\n"
           "  | [ec 0/1]  Console echo off/on        |\r\n"
           "  +---------------------------------------+\r\n"
           "  | Params & RunLog (v2.1)              |\r\n"
           "  | [LP]       List all algo params      |\r\n"
           "  | [SP id v]  Set param (see LP ids)    |\r\n"
           "  | [LL]       Show runlog level         |\r\n"
           "  | [LL 0~4]   Set level 0=off..4=debug  |\r\n"
           "  | Binary proto: 0xAA 0x55 frame on     |\r\n"
           "  |   this port (see docs/UPGRADE*.md)   |\r\n"
           "  +---------------------------------------+\r\n"
           "  | [d]        Diagnostics (Bus B test)   |\r\n"
           "  | [h]        This help                  |\r\n"
           "  +---------------------------------------+\r\n");
}

/* ===== 多字符命令白名单 (2026-09-18 新增, 见红线 26) =====
 * 下面 ProcessCommand 的末尾有一个「首字母兜底 switch」: 任何"精确匹配全没命中"
 * 的 token 都会按 tokens[0][0] 落进对应 case。这个设计对**单键命令**(s/h/n/i…)
 * 很方便, 但对**打错的多字符命令**是致命的 —— 实测踩过的:
 *
 *   旧固件敲 `cx`  → case 'c' → 进入阻塞式校准向导 → **松全部力矩**(整臂塌下来)
 *   敲 `ps`        → case 'p' → "Usage: p <J>"      (侥幸无害)
 *   敲 `ze`        → case 'z' → 回零点(**真的动**)
 *   敲 `ge`        → case 'g' → IK 运动(**真的动**)
 *   敲 `tq`        → case 't' → 力矩开关(可能松力矩)
 *
 * ⇒ 凡「长度 > 1 且不在白名单里」的 token, 一律当**未知命令**挡在 switch 之前。
 *   `#P` 是 PC 中转的位置流帧, 不是人敲的命令, 放行。
 * ⚠ 以后**新增任何多字符命令**(以及新增 `tokens[0][1]` 变体, 如 `qv`), 必须同步
 *   往下面这张表里加名字, 否则新命令会被这条守卫挡掉, 现场表现为"命令没反应"。 */
static uint8_t cmd_is_known_multichar(const char* t)
{
    static const char* const names[] = {
        /* ── 在 switch 之前就精确匹配并 return 的 ── */
        "teach", "rec", "play", "save", "list", "files", "del", "delete",
        "compact", "tm", "lk", "ec", "sf", "mf", "sp",
        "rb",                                      /* 0924CC: 软复位(无 RESET 键) */
        "ml", "mg", "mp", "mo", "ma", "ms",
        "ct", "cn", "can",                        /* CAN 状态/配置   */
        "cx", "cp", "cy", "cz", "cw", "ps",       /* CAN 取证/接口   */
        "nx", "np", "nm", "nt",                  /* 网口: 状态/引脚/MIIM/时钟体检 */
        "rc",                                      /* RC 遥控输入层: PS2 手柄(只读) */
        "zi", "zr", "zf",                         /* 零点检查/复位; 桌面下限(离桌 mm) */
        "colj2",                                   /* 0924AU: J2 壳体包线开关 */
        /* ── 靠 case 内 `tokens[0][1]` 分发的变体 ── */
        "pv", "pV", "pf", "pF",                   /* case 'p' 实时角/位姿 */
        "qv", "qV", "qd", "qD", "qg", "qG",       /* case 'q' 点位参数 */
        "QV", "Qv", "QD", "Qd", "QL", "QLC", "QLc",   /* case 'Q' 循环回放 */
        "BG",                                     /* case 'B' XYZ 循环 */
        "SP", "Sp", "LP", "Lp", "LL", "Ll",       /* case 'S'/'L' 参数/日志 */
    };
    for (uint32_t i = 0U; i < sizeof(names) / sizeof(names[0]); i++) {
        if (strcmp(t, names[i]) == 0) return 1U;
    }
    return 0U;
}

/* ===== Command parser ===== */
/* 0924AT: 前置声明 —— 定义在 main() 附近, 但 bootlog 命令这里就要用 */
static uint8_t boot_verbose;

/* ==================================================================
 * `sid` 工具辅助 — STS3215 改 ID 维护 (2026-10-23, 移植自 ESP32
 * sts3215_id_tool.ino; 走总线 A 半双工, 复用 scs_protocol 原语)。
 * 出厂舵机 ID=1 / 波特率 1Mbps; 改 ID 须: TorqueOFF -> EEPROM 解锁
 * -> 写 ID(烧写等待) -> RESET 生效 -> 重新上锁 -> 验证。
 * ================================================================== */
#define SID_PING_TO   50U     /* Ping/读超时 ms (舵机应答 <2ms)            */
#define SID_WR_TO     100U    /* 写超时 ms                                 */

#if ARM_ONE_BUS_BUILD
/* 单总线形态: `sid` 必须走"那条线"(ONE_BUS)。
 *   主臂   -> BUS_A/USART3 (UART5 已旁路, 走 B 什么也扫不到);
 *   集成板 -> BUS_B/UART5 (BUS_ONE 下走 A 会把 DIR_A 拉高, 门1 与门3 抢总线)。 */
#define SID_BUS       ONE_BUS
#define SID_BUS_NAME  ONE_BUS_PORT " (single-wire)"

static uint8_t sid_bus_of(uint8_t id) { (void)id; return SID_BUS; }
static const char* sid_bus_name(uint8_t bus) { (void)bus; return SID_BUS_NAME; }
#else
/* ⭐0924BG: PP/双总线形态下 `sid` 不再写死 Bus A。
 * 用户现状 = 逻辑单总线(六颗全接 Bus B, A 座拔空), 旧代码把 SID_BUS 写死
 * BUS_A ⇒ `sid scan` 永远扫 A 的空段, "没扫到任何舵机"纯属扫错了地方。
 * 改为运行时定位 ID 所在总线, 探测顺序:
 *   ① 上次定位缓存(连续 set 流程省时) ② JOINT_ID 路由表(关节舵机按槽位)
 *   ③ Bus B ④ Bus A —— 命中即缓存, 0xFF = 两条总线都 ping 不通。
 * 每次探测 50ms, 控制台命令可接受。BUS_ONE 形态不受影响(上方分支)。 */
static uint8_t s_sid_bus = 0xFFU;    /* 0xFF = 尚未定位 */

static const char* sid_bus_name(uint8_t bus) {
    switch (bus) {
    case BUS_A: return "USART3 (Bus A)";
    case BUS_B: return "UART5 (Bus B)";
    default:    return "A+B(无应答)";
    }
}

static uint8_t sid_bus_of(uint8_t id)
{
    uint8_t tryb[4];
    uint8_t n = 0U;
    if (s_sid_bus != 0xFFU) tryb[n++] = s_sid_bus;
    for (uint8_t j = 0; j < 6U; j++) {
        if (JOINT_ID[j] != id) continue;
        uint8_t b = JOINT_BUS[j], dup = 0U;
        for (uint8_t k = 0; k < n; k++) if (tryb[k] == b) dup = 1U;
        if (!dup && n < (uint8_t)(sizeof(tryb) / sizeof(tryb[0]))) tryb[n++] = b;
    }
    if (n < (uint8_t)(sizeof(tryb) / sizeof(tryb[0]))) tryb[n++] = BUS_B;
    if (n < (uint8_t)(sizeof(tryb) / sizeof(tryb[0]))) tryb[n++] = BUS_A;
    for (uint8_t k = 0; k < n; k++) {
        if (DXL_Ping(tryb[k], id, SID_PING_TO) == HAL_OK) {
            s_sid_bus = tryb[k];
            return tryb[k];
        }
    }
    return 0xFFU;
}
#endif

static uint8_t sid_ping(uint8_t id) {
    return (sid_bus_of(id) != 0xFFU) ? 1U : 0U;
}

static int sid_read1(uint8_t id, uint16_t addr) {
    uint8_t bus = sid_bus_of(id);
    uint8_t v = 0U;
    if (bus == 0xFFU) return -1;
    /* ⭐2026-11-05 用户指令: 读失败立即重试最多 10 次(SID_PING_TO/次), 成功才输出 */
    for (uint8_t t = 0; t < 10U; t++) {
        if (DXL_Read(bus, id, addr, 1U, &v, SID_PING_TO) == HAL_OK) return (int)v;
    }
    return -1;
}

static uint8_t sid_write1(uint8_t id, uint16_t addr, uint8_t v) {
    uint8_t bus = sid_bus_of(id);
    uint8_t d = v;
    if (bus == 0xFFU) return 0U;
    return (DXL_Write(bus, id, addr, &d, 1U, SID_WR_TO) == HAL_OK)
               ? 1U : 0U;
}

/* INST_RESET: scs_protocol 未包高层函数, 组帧直发。
 * RESET 后舵机内部重启, 应答可能迟到 —— 超时不判失败, 由上层 Ping 复核。 */
static void sid_reset(uint8_t id) {
    uint8_t bus = sid_bus_of(id);
    uint8_t  frame[8], rx[16];
    uint16_t rx_got = 0U;
    if (bus == 0xFFU) return;
    uint16_t n = DXL_BuildPacket(id, INST_RESET, NULL, 0U, frame);
    (void)DXL_TransmitReceive(bus, frame, n, rx, (uint16_t)sizeof(rx),
                              &rx_got, 100U);
}

/* ===== 0924BJ: 舵机总线波特率 (1M/500k 双挡 + 上电自适配) =====
 * 背景: 方案1 后每条总线 6 只舵机 (原 3 只, 容性负载翻倍), 1Mbps 边沿
 * 裕量不足 —— `d p` 实测 NO-RESP 50~70% 且 BAD-FRAME=0 (发端边沿拖缓,
 * 舵机听不清 ping, 而不是答坏了)。固件支持双挡:
 *   - 上电先按 1M 探测, 全静默再切 500k 再试 (BusBaudAutodetect)
 *   - `sid baud 0|1` 逐只写舵机 EEPROM 并回读验证, 全过才切固件挡 */
static uint32_t s_bus_baud = 1000000U;   /* 当前挡位 (自适配/sid baud 维护) */

static void BusBaudAutodetect(void) {
    /* 0924BK 加固: 每挡探 4 次(两个代表点×2轮), ≥2 中才认定"该挡有舵机"。
     * 旧版单次探测在 1M 高丢读(50~70%)下会被偶发 2 连失骗去降挡,
     * 而舵机明明全在 1M ⇒ 降挡后彻底全静默。降挡必须 1M 4探0中且 500k ≥2中。 */
    uint8_t hit1m = 0U;
    for (uint8_t r = 0U; r < 2U; r++) {
        if (DXL_Ping(JOINT_BUS[0], JOINT_ID[0], 30U) == HAL_OK) hit1m++;
        if (DXL_Ping(JOINT_BUS[5], JOINT_ID[5], 30U) == HAL_OK) hit1m++;
    }
    if (hit1m >= 2U) { s_bus_baud = 1000000U; return; }  /* 上电 BRR 已是 1M */
    ServoBus_SetBaudBoth(500000U);
    HAL_Delay(30);
    uint8_t hit5 = 0U;
    for (uint8_t r = 0U; r < 2U; r++) {
        if (DXL_Ping(JOINT_BUS[0], JOINT_ID[0], 30U) == HAL_OK) hit5++;
        if (DXL_Ping(JOINT_BUS[5], JOINT_ID[5], 30U) == HAL_OK) hit5++;
    }
    if (hit5 >= 2U) {
        s_bus_baud = 500000U;
        cprint("  [BAUD] 1M 4探0中, 500k 探到 %u/4 -> 总线按 500k 运行。\r\n", hit5);
        return;
    }
    /* 两挡都不达标: 回 1M (舵机断电/未接时扫描会如实报 0) */
    ServoBus_SetBaudBoth(1000000U);
    s_bus_baud = 1000000U;
}

static void ProcessCommand(char* line, int len) {
    /* 0924CM: 通信灯标记 —— 任一链路(控制台/网口/CAN 文本)的有效指令都会汇到
     * 这里, 记下时刻给 Led_Service() 的 1s 亮灯窗用 */
    Led_CommMark();

    /* Tokenize: split by spaces */
    char* tokens[16];
    int ntok = 0;
    char* tok = strtok(line, " \t");
    while (tok && ntok < 16) {
        tokens[ntok++] = tok;
        tok = strtok(NULL, " \t");
    }
    if (ntok == 0) return;

#if ARM_LINK_NET
    /* ── LINK=net 构建: CAN 相关命令必须**显式拦掉**(2026-09-20) ──────────
     * 这不是"命令没实现"那么轻。`cx`(换接口) 与 `ps`(引脚扫描) 都会经
     * HAL_CAN_MspInit 把 **PB12/PB13 重配成 AF9_CAN2** —— 而这两只脚正是
     * 本构建的 **ETH_RXD1/ETH_TXD1** ⇒ 敲一条命令就把网口掐死, 现象是
     * "网口突然不通", 而且从网络侧怎么查都查不出原因。
     * (0924CB 曾按"ETH 已用 PG 脚位"放行过, 用户决定不并行, 回退拦态。)
     *
     * 另一个必须在这里 return 的理由: 若把这些分支删掉, 它们会落进
     * **首字母兜底**(case 'c' → 阻塞式校准向导 → **松全部力矩**),
     * 整臂会当场塌下来(工程红线 3/4)。⇒ 正确做法是"接住 + 说清楚"。
     *
     * 网口自己的状态命令是 `nx`(见上面的 #if ARM_LINK_NET 块), 不受影响。 */
    if (NET_BLOCK_CAN_CMDS() &&
        (strcmp(tokens[0], "cx") == 0 || strcmp(tokens[0], "cp") == 0 ||
         strcmp(tokens[0], "cy") == 0 || strcmp(tokens[0], "cz") == 0 ||
         strcmp(tokens[0], "cw") == 0 || strcmp(tokens[0], "ps") == 0 ||
         strcmp(tokens[0], "can") == 0)) {
        cprint("\r\n  [NET] '%s' 在当前链路(网口)下**已停用** —— 它会启用 CAN。\r\n",
               tokens[0]);
        cprint("  原因(2026-11-10 修正表述): RMII 已在 PG 组+PC4/5, PB12/13 是\r\n");
        cprint("        CAN2 专用并无引脚冲突; 拦截是保守策略 —— cx/can 会重配 CAN\r\n");
        cprint("        外设与过滤器, 避免网口主人在场时 CAN 侧改动造成状态混乱。\r\n");
#if ARM_LINK_MULTI
        cprint("  要用 CAN: auto 模式下等 %us 静默后直接从 CAN 口发即可;\r\n",
               (unsigned)(LINK_AUTO_TIMEOUT_MS / 1000U));
        cprint("            手动模式则 `link can` → `link save` → `rb` 后生效。\r\n\r\n");
#else
        cprint("  网口状态用 `nx`; 要回 CAN 请重烧 outputs/can_api/02_slave_CANslave.bin。\r\n\r\n");
#endif
        return;
    }

    /* ── 第二批必须挡的 (2026-09-20 代码审计发现: 目前只是"恰好没编进来") ──
     * `mg`/`mp`/`mo`(以及 `ml` 走的 UART4 自检) 的收尾**无条件**把
     * PA0/PA1 还原成 `GPIO_AF8_UART4` —— 而在 net 构建里 PA1 是
     * **ETH_RMII_REF_CLK**:
     *     mg → master_gpio_loopback_test()  收尾 PA0/PA1 = AF8_UART4
     *     mp → master_pin_probe_mini()      收尾 PA0/PA1 = AF8_UART4
     *     mo r → master_pin_hold_release()  PA0/PA1 = AF8_UART4
     * 它们现在被 `#if ARM_MASTER_BUILD` 挡着(本构建 ROLE=slave, 已用
     * `nm build_net/*.elf` 确认没编进来), 但 **`make ROLE=master LINK=net`
     * 一组合就全部生效** ⇒ 敲一次 `mg`/`mp`/`mo r` 就能把 REF_CLK 掐死,
     * 而且现象与"PHY 坏了"**一模一样**, 极难反查。
     * ⭐ 通用形态: 埋着的雷要在**离它最近的构建分支**上拆, 别指望另一个宏
     *   恰好配成你想要的值(见 docs/工程红线速查 第 49 条)。 */
    if (NET_BLOCK_CAN_CMDS() &&
        (strcmp(tokens[0], "mg") == 0 || strcmp(tokens[0], "mp") == 0 ||
         strcmp(tokens[0], "mo") == 0 || strcmp(tokens[0], "ml") == 0)) {
        cprint("\r\n  [NET] '%s' 在当前链路(网口)下**已停用** —— 它会重配 PA0/PA1,\r\n", tokens[0]);
        cprint("        而 net 构建的 PA1 = ETH_RMII_REF_CLK ⇒ 敲一次就把网口掐死\r\n");
        cprint("        (它们的收尾固定写 GPIO_AF8_UART4)。\r\n");
        cprint("  引脚/时钟取证请用 `np`(含 PA1 悬空判据) / `nm` / `nt`。\r\n\r\n");
        return;
    }
#endif

    /* 等宽字体容错 (2026-09-11): 控制台字体里 l / 1 / i / I 形状几乎一样,
     * 现场敲 'lk' 极易敲成 '1k' 或 'ik' —— 而 'i' 恰好是"初始化舵机"命令,
     * 误敲会 Torque ON 抱闸, 有实打实的副作用。这里把「首字符 ∈ {1,i,I}
     * 且余下恰好是 'k'」的 token 纠正成 'lk' 再进分发。
     * 单独输入 'i' 时余下为空, 不会被改写, 'i' / 'i J' 行为不变。 */
    if ((tokens[0][0] == '1' || tokens[0][0] == 'i' || tokens[0][0] == 'I') &&
        tokens[0][1] == 'k' && tokens[0][2] == '\0') {
        tokens[0][0] = 'l';
    }

    /* While teach-recording (torque OFF) only read-only / teach commands make
     * sense: auto-exit for anything else so a motion command cannot fire while
     * the arm is limp. While a replay is streaming, any command other than
     * stop/status cancels the replay first. */
    /* ⚠ tokens[0][0] != '#' : `#P` 是 PC 中转的位置流帧, **不是人敲的命令**。
     *   少了这一条, teach rec 一开录, 收到的第一条 `#P` 就会落进下面的
     *   "非示教命令自动退出示教"分支 ⇒ 录制当场结束, 现场表现为
     *   "teach rec 之后永远 0 samples"。 */
    if (teach_mode_active && tokens[0][0] != '#' &&
        strcmp(tokens[0], "teach") && strcmp(tokens[0], "rec") &&
        strcmp(tokens[0], "play") && strcmp(tokens[0], "save") &&
        strcmp(tokens[0], "list") && strcmp(tokens[0], "files") &&
        strcmp(tokens[0], "del") && strcmp(tokens[0], "delete") &&
        strcmp(tokens[0], "compact") &&
        /* 2026-09-16: `sf`(开始/暂停跟随) 与 `ec`(控制台回显开关) 放行 ——
         * 跟随录制期间要能开关跟随, 而不是一敲就把录制顶掉。 */
        strcmp(tokens[0], "sf") && strcmp(tokens[0], "ec") &&
        strcmp(tokens[0], "s") && strcmp(tokens[0], "h") &&
        strcmp(tokens[0], "S") &&
        strcmp(tokens[0], "U") && strcmp(tokens[0], "pv") &&
        strcmp(tokens[0], "pf") && strcmp(tokens[0], "tm")) {
        /* 2026-09-08: 'U'(寄存器读/写诊断) 与 'pv'/'pf'(只读反馈) 放行,
         * 不再顶退 teach —— 否则无法在 teach 进行中采样 Torque Limit。
         * 2026-09-10: 'tm'(只读遥测) 同样放行。 */
        cmdTeachExit();
        cprint("  (auto-exited teach mode for '%s')\r\n", tokens[0]);
    }
    /* 回放期间: 除停止/状态外任何**人敲的**命令都先中止回放。
     * ⚠ tokens[0][0] != '#' : `#P` 流帧不算(主臂还在转发时, 回放不该被它
     *   一句话打断 —— 回放启动时已把流式跟随关掉, 那些帧会被计进
     *   stream_ignored 后丢弃, 见 cmdStreamJoints 的 !stream_follow_en 分支)。 */
    if (teach_play_state != TEACH_PLAY_NONE && tokens[0][0] != '#' &&
        strcmp(tokens[0], "S") && strcmp(tokens[0], "s") &&
        strcmp(tokens[0], "h")) {
        teach_play_abort(1);
    }

    /* ===== 遥测 (tm): 单总线"边发边收" =====
     * 采样子系统在后台按 20ms 节拍轮转(每拍只读 1 个关节), 打印只读缓存,
     * 因此打印频率与总线负载解耦 —— 运动中也能持续回传而不打断轨迹下发。 */
    if (strcmp(tokens[0], "tm") == 0) {
        if (ntok < 2) {
            cprint("  Telemetry: %s", telem_period_ms ? "ON" : "OFF");
            if (telem_period_ms) cprint(", period=%lu ms", (unsigned long)telem_period_ms);
            cprint("\r\n  fields:");
            if (telem_fields & TELEM_F_POS)  cprint(" p(角度)");
            if (telem_fields & TELEM_F_TEMP) cprint(" t(温度)");
            if (telem_fields & TELEM_F_CUR)  cprint(" i(电流)");
            if (telem_fields & TELEM_F_VOLT) cprint(" v(电压)");
            if (telem_fields & TELEM_F_SPD)  cprint(" s(速度)");
            if (telem_fields & TELEM_F_LOAD) cprint(" l(负载)");
            cprint("\r\n  Usage: tm <ms> [ptivsl]   (0=stop; 运动中建议 >=100ms)\r\n");
            return;
        }
        uint32_t period = (uint32_t)atol(tokens[1]);
        if (period == 0U) {
            telem_period_ms = 0U;
            cprint("  Telemetry stopped.\r\n");
            return;
        }
        if (period < 20U)    period = 20U;
        if (period > 60000U) period = 60000U;

        uint8_t fields = 0U;
        if (ntok >= 3) {
            for (const char* pc = tokens[2]; *pc != '\0'; pc++) {
                char ch = *pc;
                if (ch >= 'A' && ch <= 'Z') ch = (char)(ch - 'A' + 'a');
                switch (ch) {
                case 'p': fields |= TELEM_F_POS;  break;
                case 't': fields |= TELEM_F_TEMP; break;
                case 'i': case 'c': fields |= TELEM_F_CUR;  break;
                case 'v': case 'u': fields |= TELEM_F_VOLT; break;
                case 's': fields |= TELEM_F_SPD;  break;
                case 'l': fields |= TELEM_F_LOAD; break;
                case 'a': case '*': fields |= TELEM_F_ALL;  break;
                default: break;
                }
            }
        }
        if (fields == 0U) fields = (TELEM_F_POS | TELEM_F_TEMP | TELEM_F_CUR);

        telem_fields      = fields;
        telem_period_ms   = period;
        telem_sample_next = 0U;                /* 立即开始采样 */
        telem_print_next  = HAL_GetTick();     /* 立即打印一次 */

        cprint("  Telemetry ON: print every %lu ms; sample 1 joint per %u ms"
               " (full sweep %u ms).\r\n",
               (unsigned long)telem_period_ms,
               (unsigned)TELEM_SAMPLE_PERIOD_MS,
               (unsigned)(TELEM_SAMPLE_PERIOD_MS * 6U));
        cprint("  Fields:");
        if (fields & TELEM_F_POS)  cprint(" p(角度)");
        if (fields & TELEM_F_TEMP) cprint(" t(温度)");
        if (fields & TELEM_F_CUR)  cprint(" i(电流,约6.5mA/LSB)");
        if (fields & TELEM_F_VOLT) cprint(" v(电压)");
        if (fields & TELEM_F_SPD)  cprint(" s(速度)");
        if (fields & TELEM_F_LOAD) cprint(" l(负载)");
        cprint("\r\n");
        if (telem_period_ms < 100U)
            cprint("  Note: 打印会阻塞控制台 UART(115200 下每字符约87us),"
                   "运动中建议 <=10Hz(>=100ms)。\r\n");
        return;
    }

    /* ===== 主从链路口状态 (lk) =====
     * 主臂板经 UART4 (PA0/PA1) 打 0x13 流帧进来。这里只读计数, 用来判断
     * “线通了没有 / 从臂收到多少帧 / 被拒多少次”。
     * 现场开关: 从臂 `sf 0|1` 暂停/恢复跟随; 主臂 `ms` 看发送计数、`mf 0|1` 暂停转发。 */
    /* ===== link: 链路偏好 (2026-09-22) =====
     * ⚠ 与 `lk`(主从链路诊断) 是两个不同命令, strcmp 精确匹配不会混。
     *   但**必须**放在下面那个"首字母兜底 switch"之前 —— 否则 'l' 开头的
     *   未匹配 token 会落进兜底分支。 */
    if (strcmp(tokens[0], "rb") == 0) {
        /* ⭐0924CC: 软复位 —— 本板**没有 RESET 按键**, link 切换(link save 后)/
         * 参数验证后的唯一重启口。走 ArmMain_RequestReset 延迟通路(与网口
         * SYS_RESET 同一条): 只登记时间戳, 主循环 ServicePendingReset 在
         * 300ms 后真正 NVIC_SystemReset —— 延迟让本行回显先发完。 */
        cprint("\r\n  [SYS] reboot in 300 ms ... (link 偏好等 RESET 后生效)\r\n");
        ArmMain_RequestReset(300U);
        return;
    }

    if (strcmp(tokens[0], "link") == 0) {
        cmdLink(ntok, tokens);
        return;
    }

    if (strcmp(tokens[0], "lk") == 0) {
        LinkDiag d;
        LinkProto_GetDiag(&d);
        cprint("  Link UART4 (PA0/PA1 @115200): %s\r\n",
               stream_active ? "STREAMING" : "idle");
#if !ARM_MASTER_BUILD
        if (stream_follow_en)
            cprint("    follow: ON (sf 0 to pause)\r\n");
        else
            cprint("    follow: OFF (sf 1 to resume), ignored %lu frames\r\n",
                   (unsigned long)stream_ignored);
#else
        cprint("    (master build: this port is TX to the slave)\r\n");
#endif
        cprint("    in %lu bytes -> %lu frames (CRC err %lu), responses %lu\r\n",
               (unsigned long)d.bytes_rx, (unsigned long)d.frames,
               (unsigned long)d.crc_errs, (unsigned long)d.rsp_rx);

        /* 硬件错误 + 实测字节率 —— 把"CRC 全错"拆成可判定的物理原因。
         * 期望值: 主臂 19 字节/帧 @33ms ≈ 576 B/s。 */
        cprint("    uart err: FE(frame)=%lu NE(noise)=%lu ORE(overrun)=%lu\r\n",
               (unsigned long)d.fe, (unsigned long)d.ne, (unsigned long)d.ore);

        /* 控制台自身健康度 (2026-09-15 补): PC 中转时**从臂**的收帧瓶颈全在
         * 这里 —— 环溢出会丢字节并把两行粘成垃圾(⇒ 整帧拒 ⇒ 自锁冻结),
         * 逐字符回显会白占发送线并挤掉收帧时间。 */
        cprint("    console RX: ring drop %lu B, echo %s\r\n",
               (unsigned long)rx_ovf, console_echo_en ? "ON" : "OFF(quiet)");
#if !ARM_MASTER_BUILD
        if ((pc_relay_rx | pc_relay_err | pc_relay_bad) != 0U) {
            /* 千分比再除 10 —— 精简 newlib 没有浮点格式化, 不用 %f */
            uint32_t perm = pc_relay_rx ? ((pc_relay_err * 1000U) / pc_relay_rx) : 0U;
            cprint("    PC relay(#P): ok %lu, reject %lu (%lu.%lu%%), bad %lu\r\n",
                   (unsigned long)pc_relay_rx, (unsigned long)pc_relay_err,
                   (unsigned long)(perm / 10U), (unsigned long)(perm % 10U),
                   (unsigned long)pc_relay_bad);
            cprint("      reject streak now %u, max %lu, watchdog(500ms) %lu\r\n",
                   (unsigned)stream_reject, (unsigned long)stream_reject_max,
                   (unsigned long)stream_wd_count);
            cprint("      (%s)\r\n",
                   (stream_reject_max >= 20U)
                       ? "max 数十以上 = 自锁冻结, 不是偶发坏行"
                       : "max 很小 = 偶发单帧坏行");
        }

        /* 逐轴"静默夹住"报告 (2026-09-15, 见 stream_clamp_cnt 声明处注释)。
         * 这是判"某轴跟随总是比别的轴慢"到底是【限位夹住】还是【舵机真慢】
         * 的**唯一直接证据** —— 夹住时 clamp_raw() 照常返回成功、不打日志,
         * 所以没有这一栏就只能靠猜。 */
        {
            uint8_t any = 0U;
            for (uint8_t j = 0; j < 6; j++) {
                if (stream_clamp_cnt[j] != 0U) { any = 1U; break; }
            }
            cprint("    clamp(limit): frames %lu%s\r\n",
                   (unsigned long)stream_clamp_frames,
                   any ? "" : "  (no axis was clamped)");
            /* J5 限位 (2026-09-16 起): 与其余五轴完全相同 —— 直接来自 Phase 2 手推实测的
             * calib_min_raw / calib_max_raw。内置偏移分支与 hm 开关均已删除。
             * ⚠ 历史 (留档, 防重复排查): 2026-09-15 确认**旧内置窗口只有 160°**
             *   (servo [89.23,249.23]) ⊂ J5 物理行程 241.2° ⇒ 从臂被 clamp_raw()
             *   **静默**夹在下限 (钉死 servo 89.67 = raw 1015), 历轮误判成"J5 舵机慢"。
             *   当天那两条"限位分支错位"的假设已被两臂 `zi` 实测否掉, 别再提。
             *   ⇒ 判读: 本行 `clamp(limit)` 某轴计数明显高于其它轴 + 该轴在窗口内跟得准
             *     = 限位夹住, 不是舵机慢。修法: 重做 Phase 2 手推标定
             *     (`c` → 逐轴手推全程 → `s`), 而不是去改固件。
             *   详见 docs/J5跟随慢_限位排查_2026-09-15.md §0 / §0.3。 */
            {
                uint32_t lo10 = ((uint32_t)JOINT_MIN_RAW[4] * 3600U) / 4095U;
                uint32_t hi10 = ((uint32_t)JOINT_MAX_RAW[4] * 3600U) / 4095U;
                cprint("      J5 window raw [%u, %u] = servo %lu.%lu..%lu.%lu deg,"
                       " calib_zero_raw=%u\r\n",
                       (unsigned)JOINT_MIN_RAW[4], (unsigned)JOINT_MAX_RAW[4],
                       (unsigned long)(lo10 / 10U), (unsigned long)(lo10 % 10U),
                       (unsigned long)(hi10 / 10U), (unsigned long)(hi10 % 10U),
                       (unsigned)calib_zero_raw[4]);
            }
            if (any) {
                cprint("      clamped frames  :");
                for (uint8_t j = 0; j < 6; j++)
                    cprint(" J%u=%lu", (unsigned)j + 1U, (unsigned long)stream_clamp_cnt[j]);
                cprint("\r\n      max lost/frame(0.1deg):");
                for (uint8_t j = 0; j < 6; j++)
                    cprint(" J%u=%lu", (unsigned)j + 1U,
                           (unsigned long)stream_clamp_max10[j]);
                cprint("\r\n      (某轴远高于其它轴 = 那一轴被限位夹住 ⇒ 「跟得慢/停住」是\n");
                cprint("       限位问题, 不是舵机慢。六轴限位现均取自 Phase 2 手推实测值,\n");
                cprint("       换舵机或重锚零点后必须重做手推标定, 否则会「物理可达却被限位拒」。)\r\n");
            }
        }
#endif
        {
            /* 自上次 `lk` 起算, 比"开机至今"的均值更能反映当下状况 */
            static uint32_t prev_bytes = 0, prev_ms = 0;
            uint32_t now = HAL_GetTick();
            uint32_t db = d.bytes_rx - prev_bytes;
            uint32_t dm = now - prev_ms;
            if (prev_ms != 0U && dm >= 200U && db > 0U) {
                uint32_t bps = (db * 1000U) / dm;
                cprint("    rate: %lu B/s (delta %lu B / %lu ms), expect ~576 B/s\r\n",
                       (unsigned long)bps, (unsigned long)db, (unsigned long)dm);
                if (bps < 400U)
                    cprint("      -> 偏慢: 线上在丢字节(ORE/干扰) 或 主臂发得比 30Hz 慢\r\n");
                else if (bps > 750U)
                    cprint("      -> 偏快: 两端波特率不成 1:1 (晶振不同?) 或线上有额外流量\r\n");
            }
            prev_bytes = d.bytes_rx;
            prev_ms = now;
        }
        if (d.raw_len > 0U) {
            cprint("    last frame: cmd=0x%02X seq=%u len=%u CRC=%s\r\n",
                   (unsigned)d.cmd, (unsigned)d.seq, (unsigned)d.len,
                   d.crc_ok ? "OK" : "BAD");
            cprint("    raw:");
            for (uint8_t i = 0; i < d.raw_len; i++) cprint(" %02X", (unsigned)d.raw[i]);
            cprint("\r\n");
        }
        cprint("    stream frames: %lu, rejects(NACK): %lu, overwritten: %u\r\n",
               (unsigned long)LinkProto_StreamCount(),
               (unsigned long)LinkProto_RejectCount(),
               (unsigned)LinkProto_DroppedCount());
        cprint("    NACK sent err=%u, NACK received err=%u\r\n",
               (unsigned)d.nack_err, (unsigned)d.rsp_err);
        if (LinkProto_StreamCount() == 0U) {
            if (d.bytes_rx == 0U)
                cprint("    -> 线上 0 字节: 主臂没在发 / 没接线 / 脚接错 (先确认主臂 ms 显示 PA0/PA1)\r\n");
            else if (d.frames == 0U)
                cprint("    -> 有字节但组不成帧: 帧头/长度不对, 查波特率·接线·串阻\r\n");
            else if (d.crc_errs >= d.frames)
                cprint("    -> 帧结构成立但 CRC 全错: 位错(波特率/共地/串扰) 或收到自己的回声\r\n");
            else
                cprint("    -> 收到的是非 0x13 帧(见上面 cmd): 两端固件不一致或链路回环\r\n");
        }
        /* 硬件错误标志指向性最强, 单独给结论 */
        if (d.fe > 0U || d.ne > 0U)
            cprint("    -> UART 已报帧/噪声错: 信号电气质量有问题 —— 依次查 共地 / 串扰 / 波特率\r\n");
        else if (d.ore > 0U)
            cprint("    -> UART 溢出丢字节: 字节流被打断, 后续整段错位(本端取数不及时/ISR 被关太久)\r\n");
        else if (d.bytes_rx > 0U && d.crc_errs > 0U)
            cprint("    -> UART 无硬件错误但 CRC 全错: 字节是'干净但错'的 —— 优先怀疑波特率不成 1:1(两端晶振不同)\r\n");
        return;
    }

    /* ===== 板内遥操 (fi 0|1, 2026-11-04 方案1: 同板双总线省主臂板) =====
     * fi 1: 本板周期读 Bus A 主臂舵机(ID 7~12) -> 下发 Bus B 从臂(ID 1~6),
     *        板内直转零网络延迟, 主臂板整块退役。
     * 前提: 主臂舵机已改 ID 7~12 并接本板 Bus A, 从臂舵机 ID 1~6 在 Bus B。 */
    if (strcmp(tokens[0], "fi") == 0) {
#if !ARM_MASTER_BUILD
        cmdFollowInternal(ntok, tokens);
#else
        cprint("  fi: 仅从臂固件可用 (主臂构建是只读转发角色)\r\n");
#endif
        return;
    }

    /* ===== 从臂: 暂停/恢复跟随 (sf 0|1) =====
     * `sf 0` 立刻退出流模式并忽略后续流帧 —— 现场"停住不动"的软开关
     * (硬手段仍是 PE11 急停)。注意: 判据用第一段 token 的首字符, 与
     * 本文件其它带参命令一致。 */
    /* ===== PC 中转: 位置行接收 (#P a1..a6), 2026-09-15 =====
     * 官网 LeRobot 的做法是"PC 当中转"(主臂→USB→PC→USB→从臂, 两臂之间
     * 没有任何连线)。从臂这里直接认主臂吐出的**同一行** `#P`, 于是 PC 侧
     * 脚本可以退化成一根纯转发线 (tools/pc_relay.py)。
     * 数值 = 关节角(rel-home)×10 的定点整数, 与 0x13 帧同编码 ⇒ 语义同一。
     * `sf 0` 时整帧照旧被忽略(stream_ignored++), 与直连链路行为完全一致。 */
    if (tokens[0][0] == '#' && tokens[0][1] == 'P' && tokens[0][2] == '\0') {
        if (ntok < 7) {
            if ((++pc_relay_bad & 0x3FU) == 1U)
                cprint("  [#P] bad line (need 6 values): ignored %lu lines\r\n",
                       (unsigned long)pc_relay_bad);
            return;
        }
        float deg[6];
        for (uint8_t j = 0; j < 6; j++)
            deg[j] = Aproto_DecDeg10((int16_t)atoi(tokens[j + 1]));
        int rc = ArmApi_StreamJoints(deg);
        pc_relay_rx++;
        if (rc != ARM_API_OK) {
            if ((++pc_relay_err & 0x3FU) == 1U)
                cprint("  [#P] rejected rc=%d (limit/estop), %lu of %lu\r\n",
                       rc, (unsigned long)pc_relay_err, (unsigned long)pc_relay_rx);
        }
        return;
    }

    /* ===== 控制台回显开关 (ec 0|1), 2026-09-15 =====
     * PC 中转时从臂会把收到的每一行 `#P` **逐字符回显**回 PC —— 那是纯粹
     * 的浪费: 35 次阻塞发送/帧 + 白占发送线, 而且 cprint 是阻塞的, 回显会
     * 反过来拖慢从臂收帧 (现场表现: 偶发 500 ms 断流 + 整片拒帧)。
     * 只影响"收到什么就回显什么", 不影响读命令/发诊断 ⇒ 关掉后敲 `ec 1`
     * 依然有效(只是看不见自己敲的字)。默认 ON, 不敲就是旧行为。 */
    if (strcmp(tokens[0], "ec") == 0) {
        if (ntok >= 2 && tokens[1][0] == '0') {
            console_echo_en = 0U;
            cprint("  Console echo OFF (quiet). Type `ec 1` to restore.\r\n");
        } else {
            console_echo_en = 1U;
            cprint("  Console echo ON.\r\n");
        }
        return;
    }

    if (strcmp(tokens[0], "sf") == 0) {
#if ARM_MASTER_BUILD
        cprint("  'sf' is a SLAVE-role command; this is the MASTER build.\r\n");
#else
        if (ntok >= 2 && tokens[1][0] == '0') {
            stream_follow_en = 0U;
            cmdStreamExit("sf 0");
            cprint("  Stream follow OFF -> incoming 0x13 frames ignored.\r\n");
        } else {
            stream_follow_en = 1U;
            cprint("  Stream follow ON.\r\n");
        }
#endif
        return;
    }

    /* ===== 主臂: 暂停/恢复转发 (mf 0|1) =====
     * 主臂上电即开始 30Hz 外发, 现场想暂停只能拔线。`mf 0` 让链路静默
     * (主臂仍可自由拖动、`ms` 仍能读数), `mf 1` 立即恢复发送。 */
    if (strcmp(tokens[0], "mf") == 0) {
#if ARM_MASTER_BUILD
        if (ntok >= 2 && tokens[1][0] == '0') {
            master_fwd_en = 0U;
            cprint("  Master forwarding OFF (link silent).\r\n");
        } else {
            master_fwd_en = 1U;
            master_next_tx = 0U;      /* 下一拍立即恢复, 不等 33ms 周期 */
            cprint("  Master forwarding ON.\r\n");
        }
#else
        cprint("  'mf' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 主臂: 控制台位置流 (sp <Hz>), 2026-09-15 =====
     * 官网"PC 中转"路线的输出端。把 6 关节角按节拍打成本工程自定义的一行
     * `#P <6× deg*10>`, PC 侧 tools/pc_relay.py 原样转发给从臂; 从臂认这
     * 一行 (#P 命令) ⇒ 两臂之间**一根线都不用接**, PC 就是那根线。
     * 与 `mf` 独立: `mf 0` 静默的是自有的 0x13 直连链路, 与 `sp` 无关。 */
    if (strcmp(tokens[0], "sp") == 0) {
#if ARM_MASTER_BUILD
        if (ntok >= 2) {
            long hz = atol(tokens[1]);
            if (hz <= 0) {
                master_sp_period_ms = 0U;
                cprint("  Console position stream OFF (%lu lines, %lu suppressed).\r\n",
                       (unsigned long)master_sp_lines, (unsigned long)master_sp_skip);
                return;
            }
            if (hz > 100) hz = 100;                     /* 控制台带宽上限保护 */
            master_sp_period_ms = (uint32_t)(1000L / hz);
            if (master_sp_period_ms < 5U) master_sp_period_ms = 5U;
            master_sp_next = 0U;                        /* 下一拍立即开始 */
            cprint("  Console position stream ON: %ld Hz (%lu ms/line)\r\n",
                   hz, (unsigned long)master_sp_period_ms);
            cprint("    line: \"#P <6× joint_deg*10>\"  (与 0x13 帧同编码)\r\n");
            cprint("    ⚠ 建议先敲 `mf 0` 静默直连链路, 否则两路各读一遍舵机 (总线负载翻倍)。\r\n");
            cprint("    PC 侧: python tools/pc_relay.py --master <本板COM> --slave <从臂COM>\r\n");
        } else {
            cprint("  Console position stream: %s", master_sp_period_ms ? "ON" : "OFF");
            if (master_sp_period_ms)
                cprint(" (%lu ms, %lu Hz)", (unsigned long)master_sp_period_ms,
                       (unsigned long)(1000U / master_sp_period_ms));
            cprint("\r\n  lines=%lu suppressed=%lu\r\n",
                   (unsigned long)master_sp_lines, (unsigned long)master_sp_skip);
            cprint("  Usage: sp <Hz>  (1~100, 0=stop)   配合 tools/pc_relay.py 使用\r\n");
        }
#else
        cprint("  'sp' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 链路自检 ct [N] / CAN 详情 cn (两种角色都能用, 2026-09-14) =====
     * CAN 承载下的"帧计数证明": A 板 `ct 50` 发 50 帧到 0x7E0, B 板 `ct`
     * 看自己收到几帧 —— CAN 有硬件 CRC 与自动 ACK, 判据极简, 且**不可能被
     * 浮空脚蒙混**。这正是 UART4 那一轮一直缺的那一步。 */
    if (strcmp(tokens[0], "ct") == 0) {
        uint16_t n = 50U;
        if (ntok >= 2) {
            int v = atoi(tokens[1]);
            if (v > 0 && v <= 500) n = (uint16_t)v;
        }
        uint32_t rx_before = ArmCanDrv_TestRxCount();
        uint32_t tx_before = ArmCanDrv_TestTxCount();
        uint8_t  tec = 0U, rec = 0U, lec = 0U;
        uint16_t ok;

        cprint("\r\n  ===== ct CAN link test (id 0x%03X) =====\r\n",
               (unsigned)ARM_CAN_TEST_ID);
        cprint("  本板 自检帧 已收 %lu / 已发 %lu\r\n",
               (unsigned long)rx_before, (unsigned long)tx_before);
        cprint("  正在发送 %u 帧 (每帧 2ms) ...\r\n", (unsigned)n);
        ok = ArmCanDrv_SendTestFrames(n);
        (void)ArmCanDrv_ErrorState(&tec, &rec, &lec);
        cprint("  入发送邮箱 %u/%u 帧   tec=%u rec=%u lec=%u\r\n",
               (unsigned)ok, (unsigned)n, (unsigned)tec, (unsigned)rec, (unsigned)lec);
        cprint("  -> 去对面那块板敲 `ct`, 看它的\"已收\"增加了多少:\r\n");
        cprint("       增加 ~%u  ⇒ 物理链路通(收发器/双绞线/120R/共地全 OK)\r\n",
               (unsigned)n);
        cprint("       不增加   ⇒ 查收发器供电 / CANH-CANL 接反 / 120R 终端 / 共地\r\n");
        cprint("       lec=3    ⇒ 无人应答(对端没上电 / 线断 / 两端波特率不一致)\r\n");
        cprint("  ==========================================\r\n\r\n");
        return;
    }

    /* ===== CAN 状态详情 (cn) ===== */
    if (strcmp(tokens[0], "cn") == 0) {
        uint8_t tec = 0U, rec = 0U, lec = 0U;
        uint8_t ok = ArmCanDrv_ErrorState(&tec, &rec, &lec);
        cprint("\r\n  ===== cn CAN status =====\r\n");
        cprint("  接口    : %s %s   (切另一个口: cx 1 / cx 2)\r\n",
               ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        cprint("  hw init : %s   (FAILED = HAL_CAN_Init/ConfigFilter/Start 失败)\r\n",
               ok ? "ok" : "FAILED");
        cprint("  bitrate : 500 kbit/s (APB1 42MHz, PS=6 BS1=11TQ BS2=2TQ), 自动重传=关\r\n");
        cprint("  id 段   : 0x100+node 命令 | 0x180+node 响应 | 0x7E0 自检 | node=1\r\n");
        cprint("  test tx : %lu 帧入邮箱     test rx : %lu 帧收到\r\n",
               (unsigned long)ArmCanDrv_TestTxCount(),
               (unsigned long)ArmCanDrv_TestRxCount());
        cprint("  peer rx : %lu 帧 (0x180+node = 从臂回包)\r\n",
               (unsigned long)CanProto_PeerFrames());
        cprint("  合法好帧: %lu 帧 (CRC 通过并已执行 —— 从臂看这个数以 ~30/s 涨就说明流帧到了)\r\n",
               (unsigned long)CanProto_RxOkCount());
        cprint("  err     : tec=%u rec=%u lec=%u\r\n",
               (unsigned)tec, (unsigned)rec, (unsigned)lec);
        cprint("  判读: tec 快速增长 = 对端不应答; lec 3=ACK 错(无人应答) 6=CRC 错(线上干扰)\r\n");
        cprint("        hw=FAILED ⇒ 先敲 e 做内部回环, 再查收发器供电\r\n");
        /* ---- 原始寄存器 (2026-09-14 加) ----
         * 只报 tec/rec/lec 不够, 还必须回答两件事:
         *   (a) 控制器是不是真在正常模式跑  -> MCR.INRQ / MCR.SLEEP / MSR.INAK
         *   (b) CAN_RX 引脚此刻的实际电平    -> MSR.RX 位
         *       CAN 空闲 = 隐性 = 高; 恒低 = 总线被卡在显性。
         *       MSR.RX 是 bxCAN 对该引脚的实时采样, 不用切 GPIO、不用万用表
         *       就能分清"总线空闲"和"总线被拉死"。
         * F4 bxCAN 位: MSR bit11=RX bit10=SAMP bit1=SLAK bit0=INAK
         *               MCR bit6=ABOM bit4=NART bit1=SLEEP bit0=INRQ */
        {
            uint32_t mcr = ArmCanDrv_RegMCR(), msr = ArmCanDrv_RegMSR();
            uint32_t esr = ArmCanDrv_RegESR(), tsr = ArmCanDrv_RegTSR();
            uint8_t  bus_rx = (uint8_t)((msr >> 11) & 1UL);

            cprint("  reg     : MCR=%08lX MSR=%08lX ESR=%08lX TSR=%08lX\r\n",
                   (unsigned long)mcr, (unsigned long)msr,
                   (unsigned long)esr, (unsigned long)tsr);
            cprint("  mode    : INRQ=%lu SLEEP=%lu NART=%lu ABOM=%lu\r\n",
                   (unsigned long)((mcr >> 0) & 1UL), (unsigned long)((mcr >> 1) & 1UL),
                   (unsigned long)((mcr >> 4) & 1UL), (unsigned long)((mcr >> 6) & 1UL));
            cprint("  state   : INAK=%lu SLAK=%lu SAMP=%lu\r\n",
                   (unsigned long)((msr >> 0) & 1UL), (unsigned long)((msr >> 1) & 1UL),
                   (unsigned long)((msr >> 10) & 1UL));
            cprint("  bus pin : MSR.RX=%u (%s)\r\n", (unsigned)bus_rx,
                   bus_rx ? "隐性/空闲 ⇒ 收发器在输出高, 说明它上了电"
                          : "显性/被拉低 ⇒ 总线被拉着(对端在发 / 短路 / 模块没供电)");
            cprint("  esr     : EWGF=%lu EPVF=%lu BOFF=%lu\r\n",
                   (unsigned long)(esr & 1UL), (unsigned long)((esr >> 1) & 1UL),
                   (unsigned long)((esr >> 2) & 1UL));
            /* ⚠ 语义修正(2026-09-17 现场):
             * bxCAN 的 MSR.INAK=1 表示"硬件确实在初始化模式", MCR.INRQ=1 才是"请求进入"。
             * INRQ=0 且 INAK=0 = **正常模式、已上总线** —— 这是健康态(与上面 "hw init: ok" 一致)。
             * 真正该报警的是 INAK=1: 请求了初始化但没等到 11 位隐性 ⇒ 没上总线。
             * 旧代码写成 (INAK==0) 报警, 对健康态报"CAN 根本没在收发", 会把排查带偏。 */
            if ((msr & 1UL) != 0UL) {
                cprint("  ⚠ INAK=1 ⇒ 控制器仍在初始化模式(没等到 11 位隐性) ⇒ CAN 没上总线\r\n");
            } else if (((mcr >> 0) & 1UL) == 0UL) {
                cprint("  state ok: INRQ=0 INAK=0 ⇒ 已上总线(正常模式, 可收可发)\r\n");
            }
            if ((mcr >> 1) & 1UL) {
                cprint("  ⚠ SLEEP=1 ⇒ 控制器在睡眠(bus-off 后没唤醒), 收发全停\r\n");
            }
            if ((esr >> 2) & 1UL) {
                cprint("  ⚠ BOFF=1 ⇒ 已 bus-off (TEC>255), CAN 已被硬件摘下线\r\n");
            }
        }
        cprint("  ========================\r\n\r\n");
        return;
    }

    /* ===== CAN 从站配置 (can ...) 2026-09-17 =====
     * 机械臂作 CAN 从站时的寻址与节拍。上位机靠 `0x100+node` 发命令、
     * `0x180+node` 收响应、`0x280+node` 收周期状态与事件。
     * 改完必须 `can save` 才掉电保留(存 Flash 扇区 11 的 CAN 区, 与
     * PID/校准记录同扇区互不覆盖 —— 见 flash_cfg.h)。 */
    if (strcmp(tokens[0], "can") == 0) {
        const ArmCanCfg* cfg = CanCfg_Get();
        uint8_t tec = 0U, rec = 0U, lec = 0U;

        /* can node <1~15> */
        if (ntok >= 2 && strcmp(tokens[1], "node") == 0) {
            int v = (ntok >= 3) ? atoi(tokens[2]) : -1;
            if (CanCfg_SetNode((uint8_t)v) != 0) {
                cprint("  ✗ node 必须 1~15。用法: can node <1~15>\r\n");
                return;
            }
            CanProto_SetNodeId((uint8_t)v);
            cprint("  ✓ node = %u  →  cmd=0x%03X  rsp=0x%03X  evt=0x%03X\r\n",
                   (unsigned)cfg->node_id,
                   (unsigned)(CAN_PROTO_ID_CMD_BASE + cfg->node_id),
                   (unsigned)(CAN_PROTO_ID_RSP_BASE + cfg->node_id),
                   (unsigned)(CAN_PROTO_ID_EVT_BASE + cfg->node_id));
            cprint("  ⚠ 上位机也必须发到新 ID, 否则收不到响应\r\n");
            if (!CanCfg_IsSaved()) cprint("  ⚠ 未保存: 'can save' 落盘, 否则重启回默认\r\n");
            return;
        }

        /* can baud <100|125|250|500|1000>  (kbit/s) */
        if (ntok >= 2 && strcmp(tokens[1], "baud") == 0) {
            int v = (ntok >= 3) ? atoi(tokens[2]) : -1;
            if (CanCfg_SetBaud((uint16_t)v) != 0) {
                cprint("  ✗ 只支持 100 / 125 / 250 / 500 / 1000 kbit/s。"
                       "用法: can baud 500\r\n");
                return;
            }
            if (!ArmCanDrv_SetBitrate((uint16_t)v)) {
                cprint("  ✗ 驱动拒绝了 %d kbit/s (本板 PCLK1 下无法整除)\r\n", v);
                return;
            }
            /* 之前 CAN 没起来(hw init 失败)时, 借这次重配重试一次初始化 */
            if (!ArmCanDrv_ErrorState(&tec, &rec, &lec)) {
                CanProto_Init(cfg->node_id);
            }
            (void)ArmCanDrv_ErrorState(&tec, &rec, &lec);
            cprint("  ✓ baud = %u kbit/s, CAN 控制器: %s\r\n",
                   (unsigned)cfg->baud_kbps,
                   ArmCanDrv_ErrorState(&tec, &rec, &lec) ? "ok" : "FAILED");
            cprint("  ⚠ 另一端的波特率必须同步改, 否则总线上立刻全是错误帧\r\n");
            if (!CanCfg_IsSaved()) cprint("  ⚠ 未保存: 'can save' 落盘\r\n");
            return;
        }

        /* can hb <ms>  周期状态帧; 0=关心跳 */
        if (ntok >= 2 && strcmp(tokens[1], "hb") == 0) {
            int v = (ntok >= 3) ? atoi(tokens[2]) : -1;
            if (CanCfg_SetHb((uint16_t)v) != 0) {
                cprint("  ✗ 心跳周期只能 0(关) 或 100~10000 ms。用法: can hb 1000\r\n");
                return;
            }
            cprint("  ✓ 周期状态帧(hb) = %u ms %s\r\n",
                   (unsigned)cfg->hb_ms, cfg->hb_ms ? "" : "(已关闭)");
            if (!CanCfg_IsSaved()) cprint("  ⚠ 未保存: 'can save' 落盘\r\n");
            return;
        }

        /* can save / can reset / can stat clr */
        if (ntok >= 2 && strcmp(tokens[1], "save") == 0) {
            if (CanCfg_Save() == 0) {
                cprint("  ✓ 已写入 Flash (iface=%s node=%u baud=%u hb=%u), 重启后仍生效\r\n",
                       ArmCanDrv_IfName(), (unsigned)cfg->node_id,
                       (unsigned)cfg->baud_kbps, (unsigned)cfg->hb_ms);
                CanProto_SendEvent(ARM_CAN_EVENT_CFG_SAVED, (uint8_t)cfg->node_id);
            } else {
                cprint("  ✗ Flash 写入失败 (扇区 11 CAN 区)。配置仍在 RAM 里生效\r\n");
            }
            return;
        }
        if (ntok >= 2 && strcmp(tokens[1], "reset") == 0) {
            CanCfg_Reset();
            (void)ArmCanDrv_SetInterface(ARM_CAN_IF_CAN1);
            (void)ArmCanDrv_SetBitrate(cfg->baud_kbps);
            CanProto_SetNodeId(cfg->node_id);
            (void)CanCfg_Save();
            cprint("  ✓ 已恢复默认并落盘: iface=CAN1 node=%u baud=%u kbit/s hb=%u ms\r\n",
                   (unsigned)cfg->node_id, (unsigned)cfg->baud_kbps,
                   (unsigned)cfg->hb_ms);
            return;
        }
        if (ntok >= 3 && strcmp(tokens[1], "stat") == 0 &&
            strcmp(tokens[2], "clr") == 0) {
            CanProto_ResetStats();
            cprint("  ✓ CAN 统计已清零\r\n");
            return;
        }
        if (ntok >= 2 && strcmp(tokens[1], "stat") != 0 &&
            strcmp(tokens[1], "show") != 0) {
            cprint("  Usage: can [node <1~15>] [baud <100|125|250|500|1000>]\r\n");
            cprint("              [hb <ms>] [save] [reset] [stat clr]\r\n");
            cprint("         CAN1/CAN2 接口切换用另一条命令: cx 1 | cx 2 | cx save\r\n");
            return;
        }

        /* 无参 / can show / can stat: 全量显示 */
        {
            CanProtoStats st;
            CanProto_GetStats(&st);
            uint8_t hw = ArmCanDrv_ErrorState(&tec, &rec, &lec);
            cprint("\r\n  ===== can  CAN 从站配置 =====\r\n");
            cprint("  接口    : %s %s   (切另一个口: cx 1 / cx 2)\r\n",
                   ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
            cprint("  node    : %u      波特率: %u kbit/s %s\r\n",
                   (unsigned)cfg->node_id, (unsigned)cfg->baud_kbps,
                   CanCfg_LoadedFromFlash() ? "[Flash]" : "[默认值, 未保存过]");
            cprint("  id 段   : cmd=0x%03X (收)  rsp=0x%03X (发)  evt=0x%03X (状态/事件)\r\n",
                   (unsigned)(CAN_PROTO_ID_CMD_BASE + cfg->node_id),
                   (unsigned)(CAN_PROTO_ID_RSP_BASE + cfg->node_id),
                   (unsigned)(CAN_PROTO_ID_EVT_BASE + cfg->node_id));
            cprint("  周期状态: hb=%u ms %s\r\n", (unsigned)cfg->hb_ms,
                   cfg->hb_ms ? "" : "(关)");
            if (cfg->hb_ms != 0U && !CanProto_PeerSeen()) {
                cprint("  ⚠ 状态帧停发中: 还没收到过任何发往本节点的帧(0x%03X)。\r\n",
                       (unsigned)(CAN_PROTO_ID_CMD_BASE + cfg->node_id));
                cprint("   这是刻意的 —— 没对端时每秒白发一帧会让 bxCAN 的 TEC 每帧 +8,\r\n");
                cprint("   十几秒就 error-passive / bus-off。上位机 PING 一次即自动恢复。\r\n");
            } else if (cfg->hb_ms != 0U) {
                cprint("  (已探测到对端, 状态帧正常发送)\r\n");
            }
            if (tec >= 128U) {
                cprint("  ⚠ tec=%u ⇒ 已 error-passive, 状态帧降为每 %u ms 探一帧\r\n",
                       (unsigned)tec, (unsigned)ARM_CAN_TEC_PAUSE_MS);
                cprint("    (0918G 起不再永久停发 —— TEC 只能靠发送成功递减,\r\n");
                cprint("     永久停发就是永久自锁; 收到对端命令帧会立刻解除。\r\n");
                cprint("     恢复延迟 ≈ (tec-127) × %u ms ⇒ 当前约 %u s)\r\n",
                       (unsigned)ARM_CAN_TEC_PAUSE_MS,
                       (unsigned)(((tec > 127U ? (tec - 127U) : 0U) *
                                   ARM_CAN_TEC_PAUSE_MS + 999U) / 1000U));
            }
            cprint("  硬件    : %s   hw bitrate=%u kbit/s\r\n",
                   hw ? "ok" : "FAILED", (unsigned)ArmCanDrv_GetBitrate());
            cprint("  收      : 命令帧 %lu  对端响应帧 %lu  CRC错 %lu  重组丢弃 %lu\r\n",
                   (unsigned long)st.cmd_frames, (unsigned long)st.rsp_frames,
                   (unsigned long)st.crc_err, (unsigned long)st.reasm_err);
            cprint("  发      : CAN 帧 %lu (状态帧 %lu / 事件帧 %lu)\r\n",
                   (unsigned long)st.tx_frames, (unsigned long)st.status_tx,
                   (unsigned long)st.event_tx);
            cprint("  队列    : 驱动收丢 %lu  事件丢 %lu\r\n",
                   (unsigned long)st.rx_dropped, (unsigned long)CanProto_EventDropped());
            cprint("  最近返回码: %u (%s)   错误计数: tec=%u rec=%u lec=%u\r\n",
                   (unsigned)st.last_err,
                   st.last_err == 0U ? "OK" : "见 ARM_API_ERR_*",
                   (unsigned)tec, (unsigned)rec, (unsigned)lec);
            cprint("  配置    : %s\r\n",
                   CanCfg_IsSaved() ? "已保存" : "⚠ RAM 中有未保存的改动");
            cprint("  用法    : can node <1~15> | can baud <kbps> | can hb <ms>\r\n");
            cprint("            can save | can reset | can stat clr\r\n");
            cprint("  ==============================\r\n\r\n");
        }
        return;
    }

    /* ===== Home 零点 / IK 限位 状态与复位 (zi / zr [save]) =====
     * 2026-09-15 现场事故复盘: `c` 示教模式里任何输入流中的 '0'(记录全部 Home
     * 零点) 都会被当成按键 —— 一旦被误触并按了 's', 错误姿态就被当成官方
     * Home 落盘, 整臂 SERVO_OFFSET 被改写 (实测 J3 偏 93°、J2 偏 101°)。
     * 后果: IK 解出的舵机度全体错位 -> 解析解被 jlim/raw 检查拒 -> 静默降级
     * CCD (位置对、pitch 自由) -> 现场表现为 "以前能到、现在报 (J4 limit?)"。
     * zi = 只读打印, zr = 恢复出厂中位(全 0), zr save = 同时落盘。 */
    if (strcmp(tokens[0], "zi") == 0 || strcmp(tokens[0], "zr") == 0) {
        uint8_t do_reset = (tokens[0][1] == 'r');
        uint8_t do_save = 0U;
        float jmin[6], jmax[6];
        uint8_t bad = 0U, j;

        if (do_reset && tokens[1] != NULL && strcmp(tokens[1], "save") == 0) do_save = 1U;

        if (do_reset) {
            for (j = 0; j < 6U; j++) calib_zero_raw[j] = 0U;
            /* 顺序必须: 先重锚 Home 零点(改 SERVO_OFFSET), 再算限位(jlim 用新 offset) */
            Kinematics_SetZeroRaw(calib_zero_raw);
            apply_calib_limits();
            refresh_pose_servo();
            update_home_dock();
            if (do_save) pid_save_current();
            cprint("\r\n  ===== zr: Home 零点已复位为出厂中位 (SERVO_OFFSET=180°) =====\r\n");
            cprint("  %s\r\n", do_save ? "已写入 Flash (重启保持)。" : "仅本次运行有效 — 重启会恢复 Flash 里的旧值; 要固化请敲 `zr save`。");
        } else {
            cprint("\r\n  ===== zi: Home 零点 / IK 限位 状态 =====\r\n");
        }

        cprint("  calib_zero_raw (官方 Home 姿态 raw; 0=未校准 ⇒ 出厂中位 180°)\r\n");
        cprint("     [%u %u %u %u %u %u]\r\n",
               (unsigned)calib_zero_raw[0], (unsigned)calib_zero_raw[1],
               (unsigned)calib_zero_raw[2], (unsigned)calib_zero_raw[3],
               (unsigned)calib_zero_raw[4], (unsigned)calib_zero_raw[5]);
        cprint("  SERVO_OFFSET (IK 关节 0 位, 度)\r\n     [%.1f %.1f %.1f %.1f %.1f %.1f]\r\n",
               (double)SERVO_OFFSET[0], (double)SERVO_OFFSET[1], (double)SERVO_OFFSET[2],
               (double)SERVO_OFFSET[3], (double)SERVO_OFFSET[4], (double)SERVO_OFFSET[5]);
        cprint("  JOINT_MIN_RAW [%u %u %u %u %u %u]\r\n",
               (unsigned)JOINT_MIN_RAW[0], (unsigned)JOINT_MIN_RAW[1],
               (unsigned)JOINT_MIN_RAW[2], (unsigned)JOINT_MIN_RAW[3],
               (unsigned)JOINT_MIN_RAW[4], (unsigned)JOINT_MIN_RAW[5]);
        cprint("  JOINT_MAX_RAW [%u %u %u %u %u %u]\r\n",
               (unsigned)JOINT_MAX_RAW[0], (unsigned)JOINT_MAX_RAW[1],
               (unsigned)JOINT_MAX_RAW[2], (unsigned)JOINT_MAX_RAW[3],
               (unsigned)JOINT_MAX_RAW[4], (unsigned)JOINT_MAX_RAW[5]);
        IK_GetJointLimitsDeg(jmin, jmax);
        for (j = 0U; j < 6U; j++) {
            if (JOINT_MIN_RAW[j] == 0U && JOINT_MAX_RAW[j] == 0U) bad = 1U;
        }
        cprint("  IK jlim (关节角空间, 度)  = 由 JOINT_*_RAW + SERVO_OFFSET 算出:\r\n");
        cprint("     J2: [%.2f, %.2f]\r\n", (double)jmin[1], (double)jmax[1]);
        cprint("     J3: [%.2f, %.2f]\r\n", (double)jmin[2], (double)jmax[2]);
        cprint("     J4: [%.2f, %.2f]\r\n", (double)jmin[3], (double)jmax[3]);
        cprint("  ⚠ 自检: 若上面 SERVO_OFFSET 与 calib_zero_raw 不匹配 (例如 zero 是\r\n");
        cprint("     [2014 896 3106 2331 2022 1567] 这种参差的读数), 说明 Home 零点被\r\n");
        cprint("     某次示教误触 '0' 记录成当时姿态 —— IK 限位会随之错位。\r\n");
        cprint("     健康值: 未校准应全 0; 已校准应接近 [2048 ×6]。\r\n");
        cprint("     修复: `zr` (本次生效) 或 `zr save` (写 Flash 固化)。\r\n");
        if (bad) cprint("     (注: 有 JOINT_*_RAW 仍为 0 ⇒ 该关节未校准, IK 保持 ±150° 默认)\r\n");
        cprint("  ========================================\r\n\r\n");
        return;
    }

    /* ===== cx CAN 接口切换 (2026-09-18) =====
     * 把"用哪个 CAN 控制器 + 哪一对脚"做成运行时可切。
     * 动机: 现场反复出现"帧发得出去、谁也收不到、RX 脚一动不动", 而其中一个
     * 很有力的解释是 —— **手里的 USB-CAN 插在板子的另一个 CAN 口上**(F407 有
     * CAN1 与 CAN2, 各一对脚, 症状一模一样)。过去接口是编译期定死的, 验一次
     * 这个假设要拉 BOOT0 重烧一次固件; 现在一条命令就切。
     *   cx          —— 打印当前接口与可选项
     *   cx 1        —— 切到 CAN1 (引脚用下面 pa/pb 选的那对)
     *   cx pa/cx pb —— 换 CAN1 的那一对脚(PA11/PA12 或 PB8/PB9), 运行期, 不落盘
     *   cx 2        —— 切到 CAN2 (PB12/PB13, AF9)
     *   cx save     —— 把当前选择写进 Flash, 重启后仍生效 (cx reset 回 CAN1)
     * ⚠ 切换会停一下总线并重配过滤器, 对端可能记一次错误, 属正常。 */
#if ARM_LINK_NET
    if (strcmp(tokens[0], "nx") == 0) {
#if ARM_LINK_MULTI
        /* both 构建: 本次启动 NET 外设没初始化时, NetProto 结构体是未初始化内存
         * ⇒ 读出来会像一张正常的表 ⇒ 必须明确拒绝, 不能假装能看(红线 66)。
         * 0924CE: 判据改 LinkCfg_NetHwInited() —— auto 模式下网口已初始化,
         * 静默期(LINK_IS_NET()==0) nx 照样有意义。 */
        if (!LinkCfg_NetHwInited()) {
            cprint("\r\n  [LINK] 本次启动未初始化网口(active=%s) ⇒ `nx` 无意义。\r\n",
                   LinkCfg_NameOf(LinkCfg_ActiveMode()));
            cprint("  切到网口: `link net` 或 `link auto` → `link save` → `rb` 后生效。\r\n\r\n");
            return;
        }
#endif
        NetProtoStatus st;
        NetProto_GetStatus(&st);
        static const char* NS[] = { "MAC-wait", "PHY-scan", "AN/link", "RUN" };
        cprint("\r\n  ===== nx NET status =====\r\n");
        cprint("  state    : %s (phy %s addr %u)\r\n",
               NS[st.state & 3U],
               st.phy_found ? "found" : "NOT found",
               st.phy_found ? (unsigned)st.phy_addr : 0U);
        cprint("  link     : %s, %sM %s-duplex%s\r\n",
               st.link ? "UP" : "down",
               st.speed100 ? "100" : "10",
               st.fulldup ? "full" : "half",
               st.forced ? "  (forced, autoneg timed out)" : "");
        cprint("  addr     : udp://192.168.7.120:5000  (MAC 02:4E:45:54:00:01)\r\n");
        cprint("  rx/tx    : %lu/%lu 帧   udp cmd=%lu  crc err=%lu  tx err=%lu\r\n",
               (unsigned long)st.rx_pkts, (unsigned long)st.tx_pkts,
               (unsigned long)st.udp_cmd, (unsigned long)st.crc_err,
               (unsigned long)st.tx_err);
        cprint("  peer     : arp=%lu  icmp(ping)=%lu\r\n",
               (unsigned long)st.arp_req, (unsigned long)st.icmp_req);
        /* ICMP 校验和自检: 把"算得的"和"帧里实际的"分开报, 免得再对着 0x0000 猜。
         *   一致 ⇒ 通路没问题(包还被丢就是 PC 侧); 不一致 ⇒ 算了没写进帧。 */
        cprint("  icmp自检 : ilen=%u  算得=%04X  帧里=%04X  %s\r\n",
               (unsigned)st.icmp_ilen, (unsigned)st.icmp_calc, (unsigned)st.icmp_frame,
               st.icmp_req == 0UL ? "(还没收到 ping)"
                                  : (st.icmp_calc == st.icmp_frame ? "一致 ✔"
                                                                   : "不一致 ✗ ⇒ 存写/拷贝环节"));
        /* ---- 0920R 发送路径体检: 把 "f2 → 池 → DMA/MAC" 拆开看 ----
         * 判读:
         *   池[36..37] == 帧里 ⇒ 帧在内存里是好的 ⇒ 0x0000 是 MAC 发的时候改的(HW 校验和插入)
         *   TXD0 bit23:22(CIC) 非 0 ⇒ 就是在插
         *   RXD2 落进 0x2000D950~0x2000F150 ⇒ RX DMA 别名到 TX 池 */
        cprint("  dbg-MACCR: %08lX  (bit10 IPCO=%lu, 该位使能 IPv4 校验和卸载)\r\n",
               (unsigned long)st.maccr, (unsigned long)((st.maccr >> 10) & 1U));
        cprint("  dbg-TXD0 : %08lX %08lX %08lX %08lX   (CIC=bit23:22)\r\n",
               (unsigned long)st.txd0[0], (unsigned long)st.txd0[1],
               (unsigned long)st.txd0[2], (unsigned long)st.txd0[3]);
        cprint("  dbg-TXD2 : %08lX %08lX %08lX %08lX\r\n",
               (unsigned long)st.txd2[0], (unsigned long)st.txd2[1],
               (unsigned long)st.txd2[2], (unsigned long)st.txd2[3]);
        cprint("  dbg-RXD2 : %08lX %08lX %08lX %08lX\r\n",
               (unsigned long)st.rxd2[0], (unsigned long)st.rxd2[1],
               (unsigned long)st.rxd2[2], (unsigned long)st.rxd2[3]);
        cprint("  dbg-池回读: slot=%u [24..25]=%04X [36..37]=%04X [38]=%02X  %s\r\n",
               (unsigned)st.pool_slot, (unsigned)st.pool24, (unsigned)st.pool36,
               (unsigned)st.pool38,
               st.icmp_frame == 0U ? "(还没发过 ICMP)"
                                   : (st.pool36 == st.icmp_frame
                                      ? "= 帧里 ⇒ 池没被写, 是 MAC 改的"
                                      : "★≠ 帧里 ⇒ 拷贝/内存被写★"));
        {
            uint8_t dbg = NetProto_DebugCtl(0xFFU, 0U);   /* 只读 */
            cprint("  dbg-开关 : cic-fix=%u  sentinel=%u  hwck=%u   (nx cic|sent|hwck 0/1)\r\n",
                   (unsigned)(dbg & 1U), (unsigned)((dbg >> 1) & 1U), (unsigned)((dbg >> 2) & 1U));
        }
        if (strcmp(tokens[0], "nx") == 0 && ntok >= 3 && strcmp(tokens[1], "cic") == 0) {
            (void)NetProto_DebugCtl(0U, (uint8_t)(tokens[2][0] == '1'));
            cprint("  → cic-fix 置 %c (1=显式清 CIC 不插校验和, 0=让 HAL 保持原样)\r\n", tokens[2][0]);
        } else if (strcmp(tokens[0], "nx") == 0 && ntok >= 3 && strcmp(tokens[1], "sent") == 0) {
            (void)NetProto_DebugCtl(1U, (uint8_t)(tokens[2][0] == '1'));
            cprint("  → sentinel 置 %c (1=ICMP 校验和故意写 0xAABB)\r\n", tokens[2][0]);
        } else if (strcmp(tokens[0], "nx") == 0 && ntok >= 3 && strcmp(tokens[1], "hwck") == 0) {
            (void)NetProto_DebugCtl(2U, (uint8_t)(tokens[2][0] == '1'));
            cprint("  → hwck 置 %c (1=让 MAC 自己插校验和 / 0=软件写)\r\n", tokens[2][0]);
        } else if (strcmp(tokens[0], "nx") == 0 && ntok >= 2 && strcmp(tokens[1], "scan") == 0) {
            NetProto_Rescan();
            cprint("  → 已重新扫描 PHY\r\n");
        } else {
            cprint("  nx scan  -> 重新扫描 PHY / 重跑链路\r\n");
            cprint("  nx cic  0/1 -> 显式清 CIC(关硬件校验和插入)\r\n");
            cprint("  nx sent 0/1 -> ICMP 校验和哨兵(故意写 0xAABB)\r\n");
            cprint("  nx hwck 0/1 -> 反过来让 MAC 自己插校验和\r\n");
            cprint("  np       -> 引脚级取证 (AFR/REF_CLK/MDIO 原值)\r\n");
            cprint("  nm       -> 软 MIIM + MDIO 线体检 (分清线没接/被拉高/PHY 不应答)\r\n");
            cprint("  nt       -> REF_CLK 时钟体检 v3 (复位模式+外时钟模式+定窗自检\r\n");
            cprint("              + ITR 100kHz 自检 + 引脚→TI2 通路标定; 任一不过则结论不可信)\r\n");
        }
        cprint("  ==========================\r\n\r\n");
        return;
    }

    /* `np` = 引脚级取证: 一次把 AFR / REF_CLK 跳不跳 / MDIO 原始读值全打出来,
     * 用来把"网口不通"劈成"固件配错"和"硬件没到"两半。只读, 不改任何配置。 */
    if (strcmp(tokens[0], "np") == 0) {
        NetProtoProbe pb;
        NetProto_ProbeSample(&pb);
        cprint("\r\n  ===== np NET pin probe (只读取证) =====\r\n");
        if (!pb.valid) {
            cprint("  MAC 还没初始化完, 等一两秒再敲 (或用 nx 看状态)\r\n\r\n");
            return;
        }
        cprint("  RMII_sel=%u  MACMIIAR=%08X  CR=%u (168MHz 应为 4=Div102)\r\n",
               (unsigned)pb.rmii_sel, (unsigned)pb.macmiiar,
               (unsigned)((pb.macmiiar & 0x1CU) >> 2U));
        cprint("  AFR (期望全 11=AF11)\r\n");
        cprint("    A: PA1=%u PA2=%u PA7=%u   C: PC1=%u PC4=%u PC5=%u\r\n",
               (unsigned)pb.afr[0], (unsigned)pb.afr[1], (unsigned)pb.afr[2],
               (unsigned)pb.afr[3], (unsigned)pb.afr[4], (unsigned)pb.afr[5]);
        cprint("    G: PG11=%u PG13=%u PG14=%u  (TX_EN/TXD0/TXD1)\r\n",
               (unsigned)pb.afr[6], (unsigned)pb.afr[7], (unsigned)pb.afr[8]);
        cprint("  PA1(REF_CLK) 采样 %u 次: 高 %u 低 %u\r\n",
               (unsigned)pb.refclk_n, (unsigned)pb.refclk_hi,
               (unsigned)(pb.refclk_n - pb.refclk_hi));
        cprint("    ⚠ 这个比例**判不了**有没有时钟: 50MHz 真时钟下伪随机采样必然 ~50%,\r\n");
        cprint("      而**悬空脚捡耦合噪声也是 ~50%** ⇒ 两种情况读数一模一样(红线 44)。\r\n");
        cprint("      \"有没有 50MHz\"只看 `nt` 的硬件边沿计数 + TIM3_TRGO 自检。\r\n");
        /* —— 这三态才能把"悬空"与"被驱动"分开, 且不用万用表 —— */
        cprint("  PA1 静态电平对照 (每态 %lu 次采样, 记高电平次数):\r\n",
               (unsigned long)pb.pa1_ln);
        cprint("    内部不拉=%lu  上拉=%lu  下拉=%lu\r\n",
               (unsigned long)pb.pa1_np, (unsigned long)pb.pa1_pu,
               (unsigned long)pb.pa1_pd);
        {
            uint32_t hi_th = pb.pa1_ln - pb.pa1_ln / 10U;   /* ≥90% 算"稳定高" */
            uint32_t lo_th = pb.pa1_ln / 10U;               /* ≤10% 算"稳定低" */
            uint8_t np_hi = (uint8_t)(pb.pa1_np >= hi_th);
            uint8_t np_lo = (uint8_t)(pb.pa1_np <= lo_th);
            uint8_t pd_hi = (uint8_t)(pb.pa1_pd >= hi_th);
            uint8_t pd_lo = (uint8_t)(pb.pa1_pd <= lo_th);
            if (np_hi && pd_hi) {
                cprint("    判读: 三种状态都读到高 ⇒ 外部**推挽拉高**(内部下拉压不动)\r\n");
                cprint("      ⇒ 这根脚上**有源在驱动**, 不是悬空。但若是 25MHz 无源晶振方案,\r\n");
                cprint("        PHY 的 pin14 本应输出 REFCLKO 方波, 它却是个**静态高**\r\n");
                cprint("        ⇒ 重点查 nINTSEL(2脚): 锁成 1 时 pin14 变成开漏 nINT,\r\n");
                cprint("          只会被上拉钉在高、不输出时钟 —— 且 PHY 本身也不会跑。\r\n");
            } else if (np_lo && pd_lo) {
                cprint("    判读: 三种状态都读到低 ⇒ 外部**主动拉低**(或强下拉) ⇒ 不是悬空\r\n");
                cprint("      ⇒ 查这条网络是否短路 / 被别的推挽输出占(对原理图)。\r\n");
            } else if (np_hi && pd_lo) {
                cprint("    判读: 电平完全跟着内部上下拉走 ⇒ **PA1 悬空**(没有外部源在驱动)\r\n");
                cprint("      ⇒ 三个可能, 按顺序查:\r\n");
                cprint("        ① B1 实际没接通/触点氧化 —— **断电**用蜂鸣档量 B1 两端通断;\r\n");
                cprint("        ② PA1 到 B1 或 B1 到 PHY 的走线断/虚焊;\r\n");
                cprint("        ③ PHY 的 REFCLKO(pin14) 处于高阻(没工作)。\r\n");
            } else {
                cprint("    判读: 不稳定 ⇒ 这个节点有活动信号, 静态判据不适用\r\n");
                cprint("      ⇒ 一切以 `nt`(硬件边沿计数 + 自检) 的结论为准。\r\n");
            }
        }
        cprint("  IDR: PA2(MDIO)=%u 期望1 | PA7=%u | PC1(MDC)=%u | PC4=%u PC5=%u\r\n",
               (unsigned)pb.idr_pa2, (unsigned)pb.idr_pa7, (unsigned)pb.idr_pc1,
               (unsigned)pb.idr_pc4, (unsigned)pb.idr_pc5);
        cprint("  MDIO 原始 IDR1 a0..a7:\r\n   ");
        for (uint32_t a = 0U; a < 8U; a++) {
            cprint(" %u:%04X%c", (unsigned)a, (unsigned)pb.mdio_raw[a],
                   pb.mdio_ok[a] ? 'R' : 'T');
        }
        cprint("\r\n   全 FFFF+R = 上拉在、PHY 不应答 => 查 PHY 供电 / nRST / PHYAD0\r\n");
        cprint("   全 T      = MDIO 忙位超时       => 查 MDC-MDIO 走线 / CR 分频\r\n");
        cprint("   全 0000   = 线被拉死           => 查短路 / 收发器占脚\r\n");
        cprint("  ======================================\r\n\r\n");
        return;
    }

    /* `nm` = 软 MIIM(bit-bang) + MDIO 线体检。
     * `np` 只能说"读回 FFFF"; `nm` 把 FFFF 拆成三种**确定**原因:
     *   线没接上 / 线被硬拉高 / 线是好的但 PHY 不应答。
     * 只动 PA2(开漏) 与 PC1(推挽), 退出前还给 ETH。 */
    if (strcmp(tokens[0], "nm") == 0) {
        NetProbeMIIM mb;
        NetProto_MiimProbe(&mb);
        cprint("\r\n  ===== nm MIIM soft probe (软 MIIM + MDIO 线体检) =====\r\n");
        if (!mb.valid) {
            cprint("  MAC 还没初始化完, 等一两秒再敲 (或 nx 看状态)\r\n\r\n");
            return;
        }

        cprint("  [1] MDIO(PA2) 线体检 (开漏: 松手读真实电平)\r\n");
        cprint("      松手(idle)电平   = %u   期望 1\r\n", (unsigned)mb.mdio_idle);
        cprint("      主动写0 -> 读回  = %u   期望 0(跟手) => 没被外部硬拉高\r\n",
               (unsigned)mb.mdio_driven0);
        cprint("      松手 -> 自己回高  = %s", mb.rel_ok ? "能" : "**不能**");
        if (mb.rel_ok && mb.dwt_ok) {
            cprint(", 用了 %lu 周期 ≈ %lu.%02lu us @168MHz",
                   (unsigned long)mb.rel_cycles,
                   (unsigned long)(mb.rel_cycles / 168U),
                   (unsigned long)(((mb.rel_cycles % 168U) * 100U) / 168U));
        } else if (mb.rel_ok) {
            cprint(", 循环 %lu 次 (DWT 不可用)", (unsigned long)mb.rel_loops);
        } else {
            cprint("(循环 %lu 次未回高)", (unsigned long)mb.rel_loops);
        }
        cprint("\r\n");
        if (!mb.rel_ok || (mb.dwt_ok && mb.rel_cycles > 1680U)) {
            cprint("      ⇒ **回高很慢/回不来** = 这条线上没有上拉电阻\n\r");
            cprint("        ⇒ PA2 很可能**根本没接到 PHY**(悬空), 或那侧上拉没装\n\r");
        } else if (mb.mdio_idle == 1U) {
            cprint("      ⇒ 松手稳定在 1 ⇒ MDIO 网上**有外部上拉**(规范设计) ✔\n\r");
            cprint("        ⚠ 这只证明**线是设计出来的** —— 上拉是颗电阻, 跟 PHY 有没有\n\r");
            cprint("          上电/复位**毫无关系**。PHY 活没活, 要看 `nt`。\n\r");
        } else {
            cprint("      ⇒ 能自己回高但稳定电平仍读 0 ⇒ 弱上拉或读数受扰, 结合 `nt` 看\n\r");
        }

        cprint("  [2] MDC(PC1) 驱动自检: 写0->%u 写1->%u   (期望 0 / 1)\r\n",
               (unsigned)mb.mdc_drv0, (unsigned)mb.mdc_drv1);

        cprint("  [3] 软 MIIM 读 reg2(IDR1) @MDC≈180kHz, 全 32 地址, 第3列=该帧 MDIO 被拉低的 bit 数:\r\n");
        for (uint32_t a = 0U; a < 32U; a += 4U) {
            for (uint32_t k = 0U; k < 4U; k++) {
                cprint("  a%02u:%04X/%ub", (unsigned)(a + k),
                       (unsigned)mb.id1[a + k], (unsigned)mb.lows[a + k]);
            }
            cprint("\r\n");
        }
        if (mb.phy_hit != 0xFFU) {
            cprint("  ✔ 软 MIIM 认到 LAN8720A: PHY 地址 = %u\r\n", (unsigned)mb.phy_hit);
        } else if (mb.any_hit) {
            cprint("  ⚠ 没认到 0007(LAN8720A), 但 addr %u 回了 %04X ⇒ **MDIO 通, PHY 在应答**\r\n",
                   (unsigned)mb.any_addr, (unsigned)mb.id1[mb.any_addr]);
            cprint("    ⇒ 只是这颗不是 LAN8720A(或地址非 0/1) ⇒ 按它的 ID 改判据即可\r\n");
        } else {
            cprint("  ✗ 32 个地址全是 FFFF/0000 ⇒ **MDIO 上没有任何芯片在应答**\r\n");
        }
        cprint("  判据: 低0bit  = 这 18 拍里 MDIO **一次都没被拉低** => PHY 完全没驱动\r\n");
        cprint("        低>0bit = PHY 在驱动这根线, 那 16bit 才是真实 ID 值\r\n");
        cprint("  =====================================================\r\n\r\n");
        return;
    }

    /* `nt` = REF_CLK 时钟体检 (2026-09-20 第 2 版)
     * 第 1 版的 [3] 其实在量"自己的轮询周期", 会给出与 [1] 矛盾的结论 ⇒
     * 现在改成 **两条独立硬件路径 + 一条自检**; 自检不过就明说"整段作废",
     * 不再输出任何自信的结论(宁可说不出话, 也不给错方向)。 */
    if (strcmp(tokens[0], "nt") == 0) {
        NetProtoClock ck;
        NetProto_ClockProbe(&ck);
        cprint("\r\n  ===== nt REF_CLK 时钟体检 v4 (TI2FP2 + TIM5 计数 + DWT 定窗 + 通路自检) =====\r\n");
        /* ⭐0924CL: [0] 时钟寄存器直读 —— HSERDY + HCLK (不依赖 MAC 初始化, 最先打印) */
        {
            uint32_t rcc_cr = RCC->CR;
            cprint("  [0] HSE: HSEON=%lu HSERDY=%lu | HSI: HSION=%lu HSIRDY=%lu | PLL: PLLON=%lu PLLRDY=%lu\r\n",
                   (rcc_cr >> 16) & 1U, (rcc_cr >> 17) & 1U,
                   rcc_cr & 1U, (rcc_cr >> 1) & 1U,
                   (rcc_cr >> 24) & 1U, (rcc_cr >> 25) & 1U);
            cprint("      SYSCLK=%lu Hz | HCLK=%lu Hz | APB1(PCLK1)=%lu Hz | APB2(PCLK2)=%lu Hz\r\n",
                   (unsigned long)HAL_RCC_GetSysClockFreq(),
                   (unsigned long)HAL_RCC_GetHCLKFreq(),
                   (unsigned long)HAL_RCC_GetPCLK1Freq(),
                   (unsigned long)HAL_RCC_GetPCLK2Freq());
            cprint("      判读: HSERDY=1 ⇒ 外部晶振起振; HCLK=168000000 ⇒ SYSCLK 满速;\r\n");
            cprint("            HSERDY=0 且 SYSCLK=168MHz ⇒ 时钟来自 HSI-PLL, 板上晶振没被用/没振\r\n");
        }
        if (!ck.valid) {
            cprint("  MAC 还没初始化完, 等一两秒再敲 (或 nx 看状态)\r\n\r\n");
            return;
        }

        cprint("  TIM5 实际时钟 = %lu Hz (APB1 定时器时钟; 不是 168MHz) ; DWT=%s\r\n",
               (unsigned long)ck.tim_hz, ck.dwt_ok ? "可用" : "不可用");
        cprint("  量测期间 PA1 的 AFR 回读 = %u (期望 2=AF2_TIM5) ; 量测前 PA1 瞬时电平 = %u\r\n",
               (unsigned)ck.af_ref, (unsigned)ck.pa1_level);

        /* 拍数 → 边沿率上限: 「最长 N 拍没被清」⇒ N+1 拍里一个沿都没有 */
        uint32_t ub_np = ck.tim_hz / (ck.np_max + 1U);
        uint32_t ub_pd = ck.tim_hz / (ck.pd_max + 1U);
        cprint("  [1] 复位模式 NOPULL   : CNT 最大 %lu 拍 ⇒ 沿率 ≤ %lu Hz ; 小样本 %lu/%lu\r\n",
               (unsigned long)ck.np_max, (unsigned long)ub_np,
               (unsigned long)ck.np_low, (unsigned long)ck.np_n);
        cprint("  [2] 复位模式 PULLDOWN : CNT 最大 %lu 拍 ⇒ 沿率 ≤ %lu Hz ; 小样本 %lu/%lu\r\n",
               (unsigned long)ck.pd_max, (unsigned long)ub_pd,
               (unsigned long)ck.pd_low, (unsigned long)ck.pd_n);
        cprint("      读法: 复位模式每个沿清 CNT ⇒ 「最大 N 拍」就是**最长那一段没沿的时间**。\r\n");
        cprint("            N≈3 万(≈0.4ms) ⇒ 连几 kHz 都没有 ⇒ 这条线上没有时钟。\r\n");

        uint32_t f_np = (ck.ec_np_us > 0U)
                      ? (uint32_t)(((uint64_t)ck.ec_np_edge * 1000000ULL) / ck.ec_np_us) : 0U;
        uint32_t f_pd = (ck.ec_pd_us > 0U)
                      ? (uint32_t)(((uint64_t)ck.ec_pd_edge * 1000000ULL) / ck.ec_pd_us) : 0U;
        cprint("  [3] 外时钟1 NOPULL   : %lu 个沿 / %lu µs ⇒ ≈ %lu.%03lu MHz\r\n",
               (unsigned long)ck.ec_np_edge, (unsigned long)ck.ec_np_us,
               (unsigned long)(f_np / 1000000U), (unsigned long)((f_np % 1000000U) / 1000U));
        cprint("  [4] 外时钟1 PULLDOWN : %lu 个沿 / %lu µs ⇒ ≈ %lu.%03lu MHz\r\n",
               (unsigned long)ck.ec_pd_edge, (unsigned long)ck.ec_pd_us,
               (unsigned long)(f_pd / 1000000U), (unsigned long)((f_pd % 1000000U) / 1000U));
        cprint("      读法: 外部时钟模式1 每个沿给 CNT +1, 窗口由 DWT 卡死 ⇒ 不吃轮询快慢的亏。\r\n");

        /* ---- [5] 两条自检: 定窗+计数器 / ITR 触发链路 ---- */
        uint32_t st_f   = (ck.st_us > 0U)
                        ? (uint32_t)(((uint64_t)ck.st_edge * 1000000ULL) / ck.st_us) : 0U;
        uint32_t fr_exp = (uint32_t)(((uint64_t)ck.tim_hz * (uint64_t)ck.fr_us) / 1000000ULL);
        uint8_t  lb_follow_ok = (uint8_t)(ck.lb_toggle > 0U &&
                                         (ck.lb_follow * 10U) >= (ck.lb_toggle * 8U));
        uint8_t  lb_ti2_ok    = (uint8_t)(ck.lb_toggle > 0U &&
                                         (ck.lb_ti2 * 2U) >= ck.lb_toggle);

        cprint("  [5] 自检A 定窗+计数器(自由跑) : 数到 %lu / 期望 %lu (84MHz×%luµs)  %s\r\n",
               (unsigned long)ck.fr_cnt, (unsigned long)fr_exp, (unsigned long)ck.fr_us,
               ck.fr_ok ? "✔ 通过" : "✗ 不过");
        cprint("  [5] 自检B 已知100kHz(TIM5-ITR1): %lu 个沿 / %lu µs ⇒ ≈ %lu.%03lu kHz  %s\r\n",
               (unsigned long)ck.st_edge, (unsigned long)ck.st_us,
               (unsigned long)(st_f / 1000U), (unsigned long)(st_f % 1000U),
               ck.st_ok ? "✔ 通过" : "✗ 不过");
        cprint("      旁证: TIM4(ITR2) 同一激励数到 %lu 个沿 ; TIM3 自身 = %s\r\n",
               (unsigned long)ck.st_w4, ck.st_t3_run ? "在跑" : "**没在跑**");
        cprint("      ⚠ 触发源必须查**本从定时器那一行**: TIM5 看 ITR1=TIM3、TIM4 看 ITR2=TIM3\r\n");
        cprint("        (v2 把 TIM5 写成 ITR2=TIM4 ⇒ 自检必然不过; 那是**工具的 bug**, 不是板子)\r\n");

        /* ---- [6] 通路标定: 前面四条全都要经过 "PA1 → TI2FP2" ---- */
        cprint("  [6] 通路标定(引脚→TI2): 上下拉脉冲 %lu 次, 电平跟着走 %lu 次, TI2 数到 %lu 个沿\r\n",
               (unsigned long)ck.lb_toggle, (unsigned long)ck.lb_follow, (unsigned long)ck.lb_ti2);

        cprint("  -----------------------------------------------------------\r\n");

        if (!ck.fr_ok) {
            cprint("  结论: ✗ **自检A 不过 ⇒ 定窗/计数器本身有问题 ⇒ 整段输出作废**\r\n");
            cprint("    (自由跑都数不准 ⇒ 后面所有窗口读数都没意义) 先 RESET 重跑一次。\r\n");
        } else if (!ck.st_ok) {
            cprint("  结论: ✗ **自检B 不过 ⇒ 触发链路有问题 ⇒ 整段输出作废**\r\n");
            if (!ck.st_t3_run) {
                cprint("    TIM3 自身没在跑 ⇒ 查 PSC/ARR 与 TIM3 的 APB1 时钟\r\n");
            } else if (ck.st_w4 > 0U) {
                cprint("    TIM4 数到了、TIM5 没数到 ⇒ **TIM5 那条 ITR 通路坏** ⇒ 改代码\r\n");
            } else {
                cprint("    TIM3 在跑但两条都没数到 ⇒ TRGO 没出去(查 TIM3->CR2 的 MMS)\r\n");
            }
            cprint("    ⚠ 此时只能说\"尺子坏了\", **不能**说板子/PHY 有问题(红线 53)。\r\n");
        } else if (!lb_follow_ok) {
            cprint("  结论: ⚠ **节点搬不动 ⇒ PA1 上有外部驱动器(不是悬空)**\r\n");
            cprint("    内部 40k 上下拉压不动它 ⇒ 有低阻源在驱动这根脚。\r\n");
            cprint("    这一步同时也**验证不了** TI2 通路(激励加不进去)。请:\r\n");
            cprint("      ① 断电 → 把 B1 拨到 OFF → 上电 → 再敲一次 `nt` (**不用重烧**);\r\n");
            cprint("      ② 那时节点变被动 ⇒ [6] 就有阳性对照了。\r\n");
            cprint("    \"搬不动\" 与 `np` 报的\"三态都是高\"是**同一件事的两种测法**。\r\n");
        } else if (!lb_ti2_ok) {
            cprint("  结论: ✗✗ **找到 bug: 电平跟着走(%lu/%lu)但 TI2 一个边沿都没有**\r\n",
                   (unsigned long)ck.lb_follow, (unsigned long)ck.lb_toggle);
            cprint("    ⇒ **PA1 → TI2FP2 这段通路是坏的** ⇒ [1]~[4] 的\"没有时钟\"**不是结论**。\r\n");
            cprint("    查: ① AFR 回读应为 2 (实测 %u) ② CC2S/CCER 是否被别处改掉\r\n",
                   (unsigned)ck.af_ref);
            cprint("       ③ 是否有别的配置点把这只脚抢回去(红线 49)\r\n");
        } else {
            /* 三条自检 + 通路标定全过 ⇒ [1]~[4] 才允许被采信 */
            uint32_t hot = ck.tim_hz / 3U;                  /* 量程内(≈28MHz) */
            uint8_t  nz_np = (uint8_t)(ck.ec_np_edge > 0U);
            uint8_t  nz_pd = (uint8_t)(ck.ec_pd_edge > 0U);
            uint8_t  rs_np = (uint8_t)(ck.np_max <= (ck.tim_hz / 100000U));   /* 复位模式也看到了沿 */
            uint8_t  rs_pd = (uint8_t)(ck.pd_max <= (ck.tim_hz / 100000U));

            if (nz_np && !nz_pd) {
                cprint("  结论: ✗ **NOPULL 有沿、PULLDOWN 沿没了 ⇒ 悬空脚噪声**, 不是时钟\r\n");
                cprint("    ⇒ 这条网络没被驱动。查: ① B1 是否真接通 ② 走线断/虚焊 ③ PHY 供电\r\n");
            } else if (!nz_np && !nz_pd && !rs_np && !rs_pd) {
                cprint("  结论: ✗✗ **PA1 上没有任何时钟** —— 四条独立证据一致\r\n");
                cprint("    而且**尺子已自证**: 定窗/计数器 ✔、ITR 触发 ✔、引脚→TI2 通路 ✔\r\n");
                cprint("    ⇒ 这条线上的\"静止\"是**可信的测量结果**, 不是工具故障。\r\n");
                cprint("    ⚠ 但它**不指向某个器件**: 静态电平也可能是 `nINTSEL` 锁错\r\n");
                cprint("      (pin2=1 ⇒ pin14 变开漏 nINT, 静态拉高、不输出任何时钟)。按顺序查:\r\n");
                cprint("      ① 断电量 B1 通断 ② PHY 供电(VDDCR≈1.2V / RBIAS 12.1k)\r\n");
                cprint("      ③ `nINTSEL`(pin2) 复位后必须为低 ④ 25MHz 晶振 ⑤ nRST\r\n");
            } else if (nz_np || nz_pd) {
                cprint("  结论: ✔ **PA1 上有真实时钟** (%lu.%03lu MHz) ⇒ PHY 在跑、时钟正常\r\n",
                       (unsigned long)(f_np / 1000000U), (unsigned long)((f_np % 1000000U) / 1000U));
                if (f_np > hot) {
                    cprint("    ⚠ 但 %lu MHz 已超过 84MHz 定时器量程的一半 ⇒ **频率读数不可信**,\r\n",
                           (unsigned long)(f_np / 1000000U));
                    cprint("      只能取\"有快速信号\"这一点 (50MHz 本该 ~1000000 个沿/20ms,\r\n");
                    cprint("      但已超出量程, 数出来必然偏少 ⇒ 别当频率用)。\r\n");
                }
                cprint("    ⇒ 那 MDIO 全 32 地址无人应答, 只剩两个方向:\r\n");
                cprint("      ① MDC(PC1)/MDIO(PA2) 到 PHY 的走线虚焊或断线 ② 断电冷启后再看。\r\n");
            } else {
                cprint("  结论: ✗ 两条路径**互相矛盾**([1][2] 说无边沿, [3][4] 说数到沿)\r\n");
                cprint("    ⇒ 工具还没修干净, 别下结论。请把这段原样发回。\r\n");
            }
        }
        /* `nt`(时钟体检) 曾经把 TIM4 拿去当"旁证计数器"(R9DS 时代)。
         * PS2 输入层不用 TIM4 了 —— 收尾的 ArmRc_Reinit 只重配 GPIO+手柄, 无害保留。 */
        ArmRc_Reinit();
        cprint("  ⚠ 量测期间 REF_CLK 断过, 跑完 `nt` 建议 RESET 一次再做业务。\r\n");
        cprint("     (PS2 输入层不受影响; 若手柄无响应可敲 `rc reinit` 重配)\r\n");
        cprint("  ===================================================\r\n\r\n");
        return;
    }
#endif

    /* ==================================================================
     * `sid` — STS3215 改 ID 维护工具 (2026-10-23, 移植自 ESP32 版并优化)
     * 走总线 A; 一次只接一个舵机(出厂 ID=1), 改完断电换下一个。
     * 优化 vs ESP32 版: 防呆(范围/同ID/E-STOP/运动互斥)、六步带确认打印、
     * EEPROM 烧写等待 500ms、验证 = Ping + EEPROM 回读双确认、
     * 失败按"回滚/双不通"给精确分支诊断。
     * ================================================================== */
    if (strcmp(tokens[0], "sid") == 0) {
        const char* sub = (ntok >= 2) ? tokens[1] : "";

        if (strcmp(sub, "set") == 0 && ntok >= 4) {
            int old = atoi(tokens[2]);
            int nw  = atoi(tokens[3]);
            cprint("\r\n  ===== sid set: 改 ID %d -> %d (自动定位所在总线) =====\r\n", old, nw);
            if (old < 1 || old > 253 || nw < 1 || nw > 253) {
                cprint("  ✗ ID 必须在 1~253 (254=广播保留)\r\n\r\n");
                return;
            }
            if (old == nw) {
                cprint("  ✗ 新旧 ID 相同, 无事可做\r\n\r\n");
                return;
            }
            if (IsEStop()) {
                cprint("  ✗ E-STOP 激活, 先旋起急停\r\n\r\n");
                return;
            }
            if (Trajectory_IsActive() || teach_play_state != TEACH_PLAY_NONE) {
                cprint("  ✗ 有轨迹/回放进行中, 先 `S` 停掉再改 (总线会打架)\r\n\r\n");
                return;
            }
            if (!sid_ping((uint8_t)old)) {
                uint8_t lb = sid_bus_of((uint8_t)old);
                cprint("  ✗ ID=%d 无应答 (探测: %s)。自查:\r\n", old, sid_bus_name(lb));
                cprint("    ① 舵机接在哪一路? `sid scan` 双总线探测\r\n");
                cprint("    ② 12V 已上电? 与 STM32 共地?\r\n");
                cprint("    ③ ID 可能不是 %d —— 先 `sid scan` 找它\r\n\r\n", old);
                return;
            }
            cprint("  [1/6] Ping %d ✔  (总线上多于一只会互相干扰, 确认只接了一只)\r\n",
                   old);
            (void)sid_write1((uint8_t)old, STS_ADDR_TORQUE_ENABLE, 0U);
            HAL_Delay(20);
            cprint("  [2/6] Torque OFF ✔\r\n");
            (void)sid_write1((uint8_t)old, STS_ADDR_LOCK, 0U);
            HAL_Delay(20);
            cprint("  [3/6] EEPROM 解锁 ✔\r\n");
            (void)sid_write1((uint8_t)old, STS_ADDR_ID, (uint8_t)nw);
            HAL_Delay(500);                /* EEPROM 烧写需时, 50ms 不够 */
            cprint("  [4/6] 新 ID 写入 EEPROM ✔ (烧写等待 500ms)\r\n");
            sid_reset((uint8_t)old);
            HAL_Delay(600);
            cprint("  [5/6] RESET 生效 ✔ (舵机内部重启)\r\n");
            (void)sid_write1((uint8_t)nw, STS_ADDR_LOCK, 1U);
            HAL_Delay(20);
            cprint("  [6/6] EEPROM 重新上锁 ✔\r\n");
            uint8_t ok  = sid_ping((uint8_t)nw);
            int     rid = ok ? sid_read1((uint8_t)nw, STS_ADDR_ID) : -1;
            if (ok && rid == nw) {
                cprint("  ✔ 验证通过: Ping %d 通 + EEPROM 回读 ID=%d\r\n", nw, rid);
                cprint("  断电 -> 换下一只 -> `sid scan` 复查\r\n");
            } else if (ok) {
                cprint("  ⚠ Ping %d 通但回读 ID=%d (≠%d), 断电重启后 `sid info %d`\r\n",
                       nw, rid, nw, nw);
            } else if (sid_ping((uint8_t)old)) {
                cprint("  ⚠ 新 ID 不通、旧 ID %d 还在 ⇒ RESET 回滚了写入, 重试一次;\r\n",
                       old);
                cprint("    连续同败则 `sid lock %d 0` 手动解锁后再 set\r\n", old);
            } else {
                cprint("  ⚠ 两个 ID 都不通 ⇒ 舵机重启中/供电跌落, 5s 后 `sid scan`\r\n");
            }
            cprint("  ⚠ 若这是机械臂关节舵机: 固件 JOINT_ID 表({1..6})要同步改,\r\n");
            cprint("    否则该关节'失联' —— 在线关节改 ID 前想清楚!\r\n\r\n");
            return;
        }

        if (strcmp(sub, "scan") == 0) {
            uint8_t found[16];
#if ARM_ONE_BUS_BUILD
            cprint("\r\n  ===== sid scan: %s 全扫描 (约 4s) =====\r\n", SID_BUS_NAME);
            uint8_t n = DXL_ScanBus(SID_BUS, found,
                                    (uint8_t)(sizeof(found) / sizeof(found[0])),
                                    15U);
            if (n == 0U) {
                cprint("  ✗ 没扫到任何舵机: ①接的是 %s? ②12V 供电? ③共地?\r\n\r\n",
                       SID_BUS_NAME);
            } else {
                cprint("  在线 %u 只:", (unsigned)n);
                for (uint8_t i = 0; i < n; i++) {
                    cprint(" %u", (unsigned)found[i]);
                }
                cprint("\r\n  ⚠ 多于 1 只时改 ID 会互相干扰, 一次只接一只!\r\n\r\n");
            }
#else
            /* ⭐0924BG: 双总线各扫一遍 (A 座空置时 A 段秒回, 不增加等待) */
            cprint("\r\n  ===== sid scan: 双总线探测 Bus A + Bus B (约 4s) =====\r\n");
            uint8_t nA = DXL_ScanBus(BUS_A, found, 8U, 15U);
            uint8_t nB = DXL_ScanBus(BUS_B, found + nA,
                                     (uint8_t)((sizeof(found) / sizeof(found[0])) - nA),
                                     15U);
            uint8_t n = (uint8_t)(nA + nB);
            if (n == 0U) {
                cprint("  ✗ A/B 两路都没扫到: ①12V 供电? ②共地? ③舵机确实接上了?\r\n\r\n");
            } else {
                cprint("  在线 %u 只:", (unsigned)n);
                for (uint8_t i = 0; i < nA; i++) cprint(" %u(A)", (unsigned)found[i]);
                for (uint8_t i = nA; i < n;  i++) cprint(" %u(B)", (unsigned)found[i]);
                cprint("\r\n  ⚠ 多于 1 只时改 ID 会互相干扰, 一次只接一只!\r\n\r\n");
            }
#endif
            return;
        }

        /* ⭐0924BJ: sid baud —— 舵机总线降速过渡 (6只/总线后 1M 裕量不足)。
         * sid baud       : 显示当前固件挡位
         * sid baud 0|1   : 0=1Mbps 1=0.5Mbps。逐只写 EEPROM+回读验证(每只≤10次,
         *   抗 50% 丢读), **全部通过才**切固件 BRR; 有失败保持原挡, 重跑续传。
         * ⚠ 两路 12 只必须同挡: 固件换挡是 A/B 一起切的 (ServoBus_SetBaudBoth)。 */
        if (strcmp(sub, "baud") == 0) {
            if (ntok < 3) {
                cprint("  固件当前总线波特率: %lu bps\r\n", (unsigned long)s_bus_baud);
                cprint("  用法: sid baud 0|1  (0=1Mbps, 1=0.5Mbps)\r\n");
                cprint("  背景: 每路 6 只后 1M 丢读 50~70%%, 降 500k 换裕量;\r\n");
                cprint("        重启后固件会自动适配已写入的挡位, 无需重烧。\r\n\r\n");
                return;
            }
            /* sid baud sw 0|1: 仅切固件挡, 不写 EEPROM (救援错挡舵机用) */
            if (strcmp(tokens[2], "sw") == 0 && ntok >= 4) {
                int sv = atoi(tokens[3]);
                if (sv != 0 && sv != 1) {
                    cprint("  ✗ sw 挡位只能是 0 (1M) 或 1 (500k)\r\n\r\n");
                    return;
                }
                uint32_t tb = (sv != 0U) ? 500000U : 1000000U;
                ServoBus_SetBaudBoth(tb);
                HAL_Delay(50);
                s_bus_baud = tb;
                cprint("  ✔ 固件已切到 %lu bps (未动舵机 EEPROM)\r\n\r\n",
                       (unsigned long)tb);
                return;
            }
            int vv = atoi(tokens[2]);
            if (vv != 0 && vv != 1) {
                cprint("  ✗ 挡位只能是 0 (1M) 或 1 (500k)\r\n\r\n");
                return;
            }
            if (IsEStop()) { cprint("  ✗ E-STOP 激活, 先旋起急停\r\n\r\n"); return; }
            if (Trajectory_IsActive() || teach_play_state != TEACH_PLAY_NONE) {
                cprint("  ✗ 有轨迹/回放进行中, 先 `S` 停掉 (总线会打架)\r\n\r\n");
                return;
            }
            uint8_t want = (uint8_t)vv;
            uint32_t target = (want != 0U) ? 500000U : 1000000U;
            /* 1) 双总线全扫描, 收集在线 ID 及其所在总线 */
            uint8_t ids[24], idbus[24], tmp[16];
            uint8_t n = 0U;
            uint8_t na = DXL_ScanBus(BUS_A, tmp, 16U, 20U);
            for (uint8_t i = 0U; i < na && n < 24U; i++) {
                ids[n] = tmp[i];   idbus[n] = BUS_A;   n++;
            }
            uint8_t nb = DXL_ScanBus(BUS_B, tmp, 16U, 20U);
            for (uint8_t i = 0U; i < nb && n < 24U; i++) {
                ids[n] = tmp[i];   idbus[n] = BUS_B;   n++;
            }
            if (n == 0U) {
                cprint("  ✗ A/B 都没扫到舵机 —— 无从写起 (12V/接线先查)\r\n\r\n");
                return;
            }
            cprint("\r\n  ===== sid baud -> %lu bps: 在线 %u 只, 逐只写 EEPROM =====\r\n",
                   (unsigned long)target, (unsigned)n);
            /* 2) 逐只: 解锁 -> 写 -> 烧写等待 -> 回读, 失败重试 ≤10 (抗丢读) */
            uint8_t fail = 0U;
            for (uint8_t i = 0U; i < n; i++) {
                uint8_t id = ids[i], bus = idbus[i];
                uint8_t done = 0U;
                for (uint8_t t = 0U; t < 10U && !done; t++) {
                    (void)DXL_Write(bus, id, STS_ADDR_LOCK, 0U, 1U, SID_WR_TO);
                    HAL_Delay(20);
                    uint8_t d = want;
                    (void)DXL_Write(bus, id, STS_ADDR_BAUD_RATE, &d, 1U, SID_WR_TO);
                    HAL_Delay(500);            /* EEPROM 烧写需时 (同 sid set) */
                    uint8_t rb = 0xFFU;
                    if (DXL_Read(bus, id, STS_ADDR_BAUD_RATE, 1U, &rb, SID_PING_TO)
                        == HAL_OK && rb == want) {
                        done = 1U;
                    }
                }
                if (done) {
                    uint8_t lk = 1U;
                    (void)DXL_Write(bus, id, STS_ADDR_LOCK, &lk, 1U, SID_WR_TO);
                    HAL_Delay(20);
                    cprint("  [%2u] ID=%-3u (%s) ✔ 回读 BaudCode=%u\r\n",
                           (unsigned)(i + 1U), (unsigned)id,
                           (bus == BUS_A) ? "A" : "B", (unsigned)want);
                } else {
                    fail++;
                    cprint("  [%2u] ID=%-3u (%s) ✗ 10 次写+回读均未确认\r\n",
                           (unsigned)(i + 1U), (unsigned)id,
                           (bus == BUS_A) ? "A" : "B");
                }
            }
            if (fail != 0U) {
                /* 兜底: 广播写 (ID 254) 重发 6 次 —— 已写过的重写无害,
                 * 漏网的原挡舵机有机会被补上 (单帧丢 50% ⇒ 6 发漏网 ~1.5%) */
                cprint("  %u 只逐写未确认, 广播补写 6 次...\r\n", (unsigned)fail);
                uint8_t p[1]; p[0] = want;
                uint8_t bc[16];
                uint16_t bl = DXL_BuildPacket(254U, INST_WRITE, p, 1U, bc);
                (void)DXL_Write(BUS_A, 254U, STS_ADDR_LOCK, 0U, 1U, SID_WR_TO);
                (void)DXL_Write(BUS_B, 254U, STS_ADDR_LOCK, 0U, 1U, SID_WR_TO);
                HAL_Delay(20);
                for (uint8_t r = 0U; r < 6U; r++) {
                    uint8_t rx[16]; uint16_t got = 0U;
                    (void)DXL_TransmitReceive(BUS_A, bc, bl, rx,
                                              (uint16_t)sizeof(rx), &got, 5U);
                    (void)DXL_TransmitReceive(BUS_B, bc, bl, rx,
                                              (uint16_t)sizeof(rx), &got, 5U);
                    HAL_Delay(10);
                }
            }
            /* 无条件切挡: 逐写已验证的 + 广播补到的都在新挡;
             * 仍错挡的用 `sid baud sw <旧挡>` 临时切回救援。 */
            ServoBus_SetBaudBoth(target);
            HAL_Delay(50);
            s_bus_baud = target;
            uint8_t okn = 0U;
            for (uint8_t i = 0U; i < n; i++) {
                if (DXL_Ping(idbus[i], ids[i], SID_PING_TO) == HAL_OK) okn++;
            }
            if (okn == n) {
                cprint("  ✔ 全部 %u 只在新挡 ping 通, 固件已切 %lu bps。\r\n",
                       (unsigned)n, (unsigned long)target);
            } else {
                cprint("  ⚠ 新挡 ping %u/%u 通; 缺的还在旧挡。救援三步:\r\n",
                       (unsigned)okn, (unsigned)n);
                cprint("    1) sid baud sw %u   (固件切回旧挡)\r\n",
                       (unsigned)(1U - want));
                cprint("    2) sid baud %u      (重写缺的那几只)\r\n",
                       (unsigned)want);
                cprint("    3) sid baud sw %u   (切回新挡)\r\n", (unsigned)want);
            }
            cprint("  重启无需重烧: 上电自适配自动跟上挡位。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "ping") == 0 && ntok >= 3) {
            uint8_t id = (uint8_t)atoi(tokens[2]);
            uint8_t lb = sid_bus_of(id);
            cprint("\r\n  sid ping ID=%u -> %s : %s\r\n\r\n", (unsigned)id,
                   sid_bus_name(lb),
                   (lb != 0xFFU) ? "✔ 应答" : "✗ A/B 两路均无应答");
            return;
        }

        if (strcmp(sub, "info") == 0 && ntok >= 3) {
            uint8_t id = (uint8_t)atoi(tokens[2]);
            uint8_t lb = sid_bus_of(id);
            cprint("\r\n  ===== sid info: %s ID=%u =====\r\n", sid_bus_name(lb),
                   (unsigned)id);
            if (lb == 0xFFU) {
                cprint("  ✗ A/B 两路都无应答 (接线/供电/ID 核对)\r\n\r\n");
                return;
            }
            int rid  = sid_read1(id, STS_ADDR_ID);
            int baud = sid_read1(id, STS_ADDR_BAUD_RATE);
            int lock = sid_read1(id, STS_ADDR_LOCK);
            int trq  = sid_read1(id, STS_ADDR_TORQUE_ENABLE);
            int mode = sid_read1(id, STS_ADDR_MODE);
            cprint("  ID=%d  BaudCode=%d (0=1Mbps)  Lock=%d  Torque=%d  Mode=%d\r\n",
                   rid, baud, lock, trq, mode);
            cprint("  (Lock=1 时 EEPROM 写入被拒; sid set 会自动解锁)\r\n\r\n");
            return;
        }

        if (strcmp(sub, "lock") == 0 && ntok >= 4) {
            uint8_t id  = (uint8_t)atoi(tokens[2]);
            uint8_t val = (tokens[3][0] == '1') ? 1U : 0U;
            uint8_t lb  = sid_bus_of(id);
            uint8_t okr = (lb == 0xFFU) ? 0U : sid_write1(id, STS_ADDR_LOCK, val);
            cprint("\r\n  sid lock ID=%u (%s) -> %u : %s\r\n\r\n",
                   (unsigned)id, sid_bus_name(lb), (unsigned)val,
                   okr ? "✔" : "✗ 写失败/无应答");
            return;
        }

        if (strcmp(sub, "reset") == 0 && ntok >= 3) {
            uint8_t id = (uint8_t)atoi(tokens[2]);
            uint8_t lb = sid_bus_of(id);
            sid_reset(id);
            HAL_Delay(600);
            cprint("\r\n  sid reset ID=%u (%s) 已发 (清错误/内部重启), 稍后 `sid ping %u`\r\n\r\n",
                   (unsigned)id, sid_bus_name(lb), (unsigned)id);
            return;
        }

        cprint("\r\n  ===== sid: STS3215 改 ID 工具 (自动定位总线, 一次只接一只) =====\r\n");
        cprint("  sid scan            全总线扫描找舵机 (约4s)\r\n");
        cprint("  sid ping <id>       Ping 指定 ID\r\n");
        cprint("  sid info <id>       读 ID/波特率/Lock/力矩/模式\r\n");
        cprint("  sid set <old> <new> 改 ID (自动解锁/写EEPROM/RESET/上锁/验证)\r\n");
        cprint("  sid lock <id> 0|1   手动解锁/上锁 EEPROM\r\n");
        cprint("  sid reset <id>      RESET 舵机 (清错误)\r\n");
        cprint("  ⚠ 改的是物理舵机 ID; 若是机械臂关节舵机, JOINT_ID 表要同步!\r\n\r\n");
        return;
    }

    /* ===== RC 遥控输入层 (只读, 2026-09-24 起: PS2 手柄, R9DS 废除) =====
     * 硬件 SPI1 (0924M): DAT=PB4(MISO) CMD=PB5(MOSI) CLK=PB3(SCK) CS=PB1,
     * 5V/GND 按接收器标签接。ACK 不存在。无运动代码, 仅合成通道脉宽。
     * 映射(方向吸收在采集层, ctrl 层 RC_SIGN_* 不动):
     *   CH1 X=右杆RX  CH2 Y=右杆RY  CH3 Z=左杆LY  CH4 J5=左杆LX
     *   CH5 夹爪=R1/L1 合成绝对量(R1=张开 L1=闭合, 松开保持)
     *   CH6 心跳=链路状态(PS2 主动轮询, 没响应=没帧, fresh 恢复本义)
     * R9DS 时代三个待标定问题(中位/行程/丢控语义)在 PS2 上全部消失:
     *   中位恒 1500(开机自动标)、行程合成 1000~2000、丢控=轮询失败。
     * ⚠ 这里不发生任何运动。映射与驱动在 arm_rc_ctrl.c, 见
     *   docs/RC方案复核_2026-09-23.md §11。 */
    if (strcmp(tokens[0], "rc") == 0) {
        const char* sub = (ntok >= 2) ? tokens[1] : "";

        if (strcmp(sub, "reset") == 0) {
            ArmRc_ResetStats();
            cprint("\r\n  rc reset: 已清零 min/max/帧数/边沿/失控次数。"
                   "中位保留(要重标敲 `rc mid`)。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "go") == 0) {
            /* R1 武装开关。⭐ 默认 off: 上电后 RC 不可能自己开始驱臂。
             * 0924R: 增加手柄双击 START/SELECT 等价于 rc go on/off。 */
            const char* sub2 = (ntok >= 3) ? tokens[2] : "";
            if (strcmp(sub2, "on") == 0) {
                ArmRcCtrl_SetArmed(1U);
                if (ArmRcCtrl_IsArmed()) {
                    rc_user_locked = 0U;
                    cprint("  rc go on: RC 已武装 (双击 SELECT 可立即锁定)。\r\n");
                }
            } else if (strcmp(sub2, "off") == 0) {
                ArmRcCtrl_SetArmed(0U);
                rc_user_locked = 1U;
                rc_auto_rearm_req = 0U;   /* 0924W: 显式解除 = 撤销挂起的自动重武装 */
                rc_auto_rearm_until = 0U;
                cprint("  rc go off: RC 遥控已解除。\r\n");
            } else {
                cprint("\r\n  RC 遥控(末端速度流): %s\r\n",
                       ArmRcCtrl_IsArmed() ? "**ARMED** (驱臂中)" : "off (默认/已锁定)");
                cprint("  用法: rc go on|off\r\n");
                cprint("  手柄: 双击 START = 武装; 双击 SELECT = 锁定 (上电后必须先双击 START)\r\n");
                cprint("  通路(0924X 关节空间): 左杆左右=J1 左杆上下=J2 右杆上下=J3 右杆左右=J4\r\n");
                cprint("        J5=L1/R1(默认)  夹爪=L2/R2(默认)  TRI=中位 CIR=折叠 CRS=录 SQR=放\r\n");
                cprint("  按键: rc keysw 30 找键 -> rc map <功能> <键> -> rc map save 存 flash\r\n");
                cprint("        中位/折叠/示教录/回放都可用手柄键触发 (默认 L1/R1, 见 rc map)\r\n");
                cprint("  前提: PS2 手柄已配对(接收器灯常亮); 限位已装载(L 命令可查)\r\n");
                cprint("  ⚠ 双击 SELECT 会显式锁定并禁用自动重武装; 下次必须双击 START 或 rc go on。\r\n");
                cprint("  互斥: 网口/文本任何运动都会立即解除 RC(要重开 rc go on)\r\n");
                cprint("  日志: 会话起止/解除原因看 runlog (tag=RCCTL)\r\n\r\n");
            }
            return;
        }

        if (strcmp(sub, "spd") == 0) {
            /* 速度设置 (09-24, 0924E 起夹爪不限速): rc spd           查看当前
             *               rc spd <vxy> <vz> <vj5>   设运行时速度
             *               rc spd save               存 flash(区 D), 掉电不丢 */
            float v[4];
            ArmRcCtrl_SpdGet(v);
            if (ntok >= 3 && strcmp(tokens[2], "save") == 0) {
                cprint("\r\n  rc spd save: 写扇区 11 区 D ... (阻塞 ~1s, 勿在运动中敲)\r\n");
                uint8_t ok = ArmRcCtrl_SpdSave();
                cprint("  %s: X/Y=%.0f mm/s  Z=%.0f mm/s  J5=%.0f deg/s\r\n\r\n",
                       ok ? "✔ 已存 flash, 掉电不丢" : "✗ 写入失败(再看 `lk`/flash 状态)",
                       (double)v[0], (double)v[1], (double)v[2]);
                return;
            }
            if (ntok >= 5) {
                /* atoi 不认小数 —— 用 strtof */
                float a = strtof(tokens[2], NULL);
                float b = strtof(tokens[3], NULL);
                float c = strtof(tokens[4], NULL);
                float d = (ntok >= 6) ? strtof(tokens[5], NULL) : v[3];
                if (!ArmRcCtrl_SpdSet(a, b, c, d)) {
                    cprint("  ✗ 出界。合法域: vxy 1~120 mm/s, vz 1~80 mm/s,"
                           " vj5 5~300 deg/s (j6 参数已废弃)\r\n\r\n");
                    return;
                }
                ArmRcCtrl_SpdGet(v);
                cprint("\r\n  ✔ 速度已更新(运行时): X/Y=%.0f  Z=%.0f  J5=%.0f\r\n",
                       (double)v[0], (double)v[1], (double)v[2]);
                cprint("  ⚠ 仅 RAM —— 掉电丢失! 要长期生效再敲: rc spd save\r\n\r\n");
                return;
            }
            cprint("\r\n  当前 RC 速度: X/Y=%.0f mm/s  Z=%.0f mm/s  J5=%.0f deg/s\r\n",
                   (double)v[0], (double)v[1], (double)v[2]);
            cprint("  用法: rc spd <vxy> <vz> <vj5>     设运行时速度(小数可以)\r\n");
            cprint("        rc spd save                  存 flash(区 D), 掉电不丢\r\n");
            cprint("  说明: 这里调的是\"杆推满时的最大速度\", 实时速度=舵量成比例。\r\n");
            cprint("        夹爪已改滑杆绝对位置全速实时跟随, 不再限速 (0924E)。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "reinit") == 0) {
            /* PS2 下 TIM4 不再归 RC —— `nt` 之后 GPIO 也不会被动。
             * 此命令保留为"救急": 重配 GPIO + 重跑手柄配置序列。 */
            ArmRc_Reinit();
            const ArmRcState* r = ArmRc_Get();
            cprint("\r\n  rc reinit: PS2 GPIO 已重配, 手柄配置序列(红灯模拟+锁存)已重跑。\r\n");
            cprint("    手柄 ID=0x%02X %s  (0x73=红灯模拟 ✔  0x41=绿灯 ✗  0x20/0x79=非标无线 ✔)\r\n\r\n",
                   (unsigned)r->id,
                   ARM_RC_ID_ANALOG_OK(r->id) ? "模拟模式(摇杆有效)" :
                   ((r->id == 0x41U) ? "绿灯(数字)" : "无响应"));
            return;
        }

        if (strcmp(sub, "mid") == 0) {
            ArmRc_SetMid();
            const ArmRcState* m = ArmRc_Get();
            cprint("\r\n  rc mid: PS2 摇杆中位恒 1500(开机已自动标), 此命令仅保留兼容。\r\n");
            for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
                cprint("    CH%u = %u µs\r\n", (unsigned)(i + 1U),
                       (unsigned)m->mid[i]);
            }
            cprint("  ⚠ 夹爪(CH5)是按键合成的**绝对量**, 中位 1500 = 半开, 不是\"回中\"概念。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "raw") == 0) {
            const ArmRcState* w = ArmRc_Get();
            uint32_t age = (w->last_frame_ms != 0U) ? (w->now_ms - w->last_frame_ms) : 0U;
            cprint("  rc: live=%u age=%lums lost=%lu\r\n",
                   (unsigned)w->live, (unsigned long)age, (unsigned long)w->lost_events);
            for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
                cprint("    CH%u=%5u fr=%-7lu%s\r\n", (unsigned)(i + 1U),
                       (unsigned)w->ch[i].width_us, (unsigned long)w->ch[i].frames,
                       w->ch[i].fresh ? "" : " (stale)");
            }
            return;
        }

        if (strcmp(sub, "watch") == 0) {
            uint32_t n = (ntok >= 3) ? (uint32_t)atol(tokens[2]) : 20U;
            if (n == 0U || n > 600U) n = 20U;
            cprint("\r\n  rc watch %lu 行 @10Hz (约 %lu s)。⚠ **阻塞**: 臂在动时别敲。\r\n",
                   (unsigned long)n, (unsigned long)(n / 10U));
            cprint("    #    CH1    CH2    CH3    CH4    CH5    CH6   age(ms)  live\r\n");
            for (uint32_t k = 0U; k < n; k++) {
                const ArmRcState* w = ArmRc_Get();
                uint32_t age = (w->last_frame_ms != 0U) ? (w->now_ms - w->last_frame_ms) : 9999U;
                cprint("  %3lu", (unsigned long)k);
                for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
                    cprint("  %5u", (unsigned)w->ch[i].width_us);
                }
                cprint("    %5lu     %u\r\n", (unsigned long)age, (unsigned)w->live);
                HAL_Delay(100U);
            }
            cprint("  (watch 结束)\r\n\r\n");
            return;
        }

        if (strcmp(sub, "keys") == 0) {
            /* 按键可视化: 立即打印 16 键按下状态 (按下=1)。
             * 要"按一下跳一行"的实时监视用 rc keysw。 */
            uint16_t bm = ArmRc_Buttons();
            const ArmRcState* k = ArmRc_Get();
            cprint("\r\n  ===== rc keys 按键即时态 (按下=1) =====\r\n");
            for (uint32_t r = 0U; r < 2U; r++) {
                cprint("  ");
                for (uint8_t b = 0U; b < 8U; b++) {
                    uint8_t code = (uint8_t)(r * 8U + b + 1U);
                    cprint(" %s=%u", ArmRc_BtnName(code),
                           (unsigned)((bm >> (code - 1U)) & 1U));
                }
                cprint("\r\n");
            }
            cprint("  摇杆 raw: RX=%u RY=%u LX=%u LY=%u (中位 128)\r\n",
                   (unsigned)k->stick_rx, (unsigned)k->stick_ry,
                   (unsigned)k->stick_lx, (unsigned)k->stick_ly);
            cprint("  实时监视(变化即打印): rc keysw [秒]   键位映射: rc map\r\n\r\n");
            return;
        }

        if (strcmp(sub, "keysw") == 0) {
            /* 按键实时监视: 在 50Hz 拍里非阻塞打印键位变化, 窗口到期自动关。
             * ⭐ 不像 rc watch 那样阻塞 —— 臂可以照常遥控。 */
            uint32_t sec = (ntok >= 3) ? (uint32_t)atol(tokens[2]) : 30U;
            if (sec == 0U || sec > 600U) sec = 30U;
            rc_keysw_until = HAL_GetTick() + sec * 1000U;
            rc_keysw_prev  = ArmRc_Buttons();
            cprint("\r\n  rc keysw: %lu s 内按键变化即打印 (按 Ctrl+C 停串口刷屏; "
                   "窗口自动关)\r\n  现在按手柄上的键试试 —— 按了哪个, 哪个 =1。\r\n\r\n",
                   (unsigned long)sec);
            return;
        }

        if (strcmp(sub, "diag") == 0) {
            /* 接线诊断 (0924J, 0924L 扩三帧): 原始轮询 DI 逐字节打印 + 判读。
             * 手柄/接收器灯都亮但 rc 收不到帧时, 先敲这个。 */
            uint8_t  raw[27];
            uint8_t  idle = 0U;
            cprint("\r\n  ===== rc diag: PS2 原始帧诊断 =====\r\n");
            if (!ArmRc_DiagRaw(raw, &idle)) {
                cprint("  ✗ 采集层未初始化 (rc 显示 GPIO ✗?), 先查启动日志。\r\n\r\n");
                return;
            }
            cprint("  DI 空闲电平: %u  (期望 1 —— 内部上拉, 无人驱动=高)\r\n",
                   (unsigned)idle);
            cprint("  当前 SPI 模式: %s (CPOL=1, CPHA=%u, bit-bang); 可用 `rc cpha 0|1` 切\r\n",
                   ArmRc_GetCpha() ? "Mode3" : "Mode2",
                   (unsigned)(ArmRc_GetCpha() ? 1U : 0U));
            cprint("  采样相位  : %s (%s) —— 不对齐用 `rc phase 0|1|a` 切\r\n",
                   ArmRc_GetPhaseEarly() ? "提前一槽 (旧克隆型)" : "标准 Mode3",
                   ArmRc_GetPhaseAuto() ? "自动判型 ON" : "手动固定");
            cprint("  帧1 (字节隔 16µs, 现役):");
            for (uint32_t i = 0U; i < 9U; i++) {
                cprint(" %02X", raw[i]);
            }
            cprint("\r\n  帧2 (字节隔 32µs):");
            for (uint32_t i = 9U; i < 18U; i++) {
                cprint(" %02X", raw[i]);
            }
            cprint("\r\n  帧3 (官方字节隔 50µs):");
            for (uint32_t i = 18U; i < 27U; i++) {
                cprint(" %02X", raw[i]);
            }
            cprint("\r\n  期望(健康手柄): FF 73 5A 7F FF 80 80 80 80"
                   " (首字节=空闲高, ID=0x73 红灯; 非标无线套装可为 0x20/0x79, 0x5A 在位即有效)\r\n\r\n");

            uint8_t all00 = 1U, allFF = 1U;
            for (uint32_t i = 0U; i < 27U; i++) {
                if (raw[i] != 0x00U) all00 = 0U;
                if (raw[i] != 0xFFU) allFF = 0U;
            }
            uint8_t f1_id = raw[1];
            /* 0924CO: 非标 2.4G 接收器 ID=0x20 / 无线双震 0x79 也算有效帧 */
            uint8_t f1_ok = ((f1_id == 0x73U || f1_id == 0x41U ||
                              f1_id == 0x20U || f1_id == 0x79U)) && raw[2] == 0x5AU;
            uint8_t f2_ok = ((raw[10] == 0x73U || raw[10] == 0x41U ||
                              raw[10] == 0x20U || raw[10] == 0x79U)) && raw[11] == 0x5AU;
            uint8_t f3_ok = ((raw[19] == 0x73U || raw[19] == 0x41U ||
                              raw[19] == 0x20U || raw[19] == 0x79U)) && raw[20] == 0x5AU;

            cprint("  判读:\r\n");
            if (idle == 0U) {
                cprint("  ✗ DAT(DI) 空闲就是低电平 ⇒ 这根线被拉死在地:\r\n");
                cprint("    最常见 = DAT 接到了 GND 脚 / 与 GND 接反 / 接到了黑线。\r\n");
                cprint("    拔掉 DAT 再敲 rc diag: 回 1 = DAT 线与地短路或接错; 仍 0 = PB4 坏(罕见)。\r\n");
            } else if (all00) {
                cprint("  ✗ 全 0x00 = DAT 在通信期间被拉低 ⇒ DAT 接错脚(接到 GND/别的低电平脚)。\r\n");
            } else if (f1_ok) {
                cprint("  ✔ 帧1 ID=0x%02X —— 通信通! 回复我日志, 查为什么 Poll 判失败。\r\n",
                       (unsigned)f1_id);
            } else if (f3_ok) {
                cprint("  ⚠ 帧3(官方 50µs 隔)能通、帧1 不通 ⇒ 把 ARM_RC_PS2_BYTE_GAP_US 改 50 重编。\r\n");
            } else if (f2_ok) {
                cprint("  ⚠ 帧2(32µs 隔)能通、帧1 不通 ⇒ 把 ARM_RC_PS2_BYTE_GAP_US 改 32 重编。\r\n");
            } else if (allFF) {
                cprint("  ✗ 三帧全 0xFF = DAT 空闲高、三档字节隔都无人应答。接线 rc pin 已验证\r\n");
                cprint("    连通的前提下, 剩两种可能:\r\n");
                cprint("    ① **孔序数反了(镜像)** —— ⭐ 判别: rc pin 结束后空闲态量 6 号孔,\r\n");
                cprint("       3.3V=序号方向对; ≈0.5~1V 且 PB1(CS) 发烫 = 6 号孔其实是 GND(数反了),\r\n");
                cprint("       插头转 180° 或从另一端重数再接。\r\n");
                cprint("    ② 无线套装没对频 —— 先给接收器上电, 再开手柄开关; 仍不行按套装\r\n");
                cprint("       说明对频(通常手柄长按 MODE/START 数秒看灯闪)。\r\n");
            } else if (f1_id == 0x00U) {
                cprint("  ⚠ ID=0x%02X: 有响应但 ID 非法 —— 半通状态:\r\n", (unsigned)f1_id);
                cprint("    ① DAT/CMD/ATT/CLK 四线有一根接触不良(杜邦线最常见)\r\n");
                cprint("    ② 接收器供电不足 —— VCC 换一路 3.3V / 补粗 GND 线\r\n");
                cprint("    ③ 杂牌手柄变体 —— 看帧2/帧3 是否变好, 变好就按上面提示改字节隔\r\n");
            } else {
                cprint("  ⚠ 响应非标准(见上原始字节) ⇒ 对照下表逐脚核对:\r\n");
                if (f1_id == 0xF3U && raw[2] == 0x5AU) {
                    cprint("  ⭐ ID=0xF3 = 0x73(红灯)+末位毛刺 或 配置模式卡死:\r\n");
                    cprint("    0924CZ: 摇杆字节全离轨值(00/FF) = 配置模式卡死 ⇒ 自动补发退出帧;\r\n");
                    cprint("    摇杆 ~80 离轨值之外 = 真红灯模拟, 已按模拟放行, 直接可用。\r\n");
                    if (ArmRc_GetCfgEn()) {
                        cprint("    当前 rc cfg 1(配置模式开): 持续 F3 请 `rc cfg 0` 关配置。\r\n");
                    } else {
                        cprint("    纯轮询默认不发配置命令 —— F3 卡死时⭐接收器断电重上电清状态。\r\n");
                    }
                } else if ((f1_id & 0x7FU) == 0x41U && raw[2] == 0x5AU) {
                    cprint("  ⭐ ID=0x%02X = 绿灯数字模式+末位毛刺 —— 按键有效(0924CZ 已放行),\r\n",
                           (unsigned)f1_id);
                    cprint("    摇杆无模拟量(恒 FF)。试 `rc cfg a` 直发 0x44 模拟命令开摇杆。\r\n");
                }
            }
            cprint("  ---------- PS2 插头引脚表 (母头开口朝上, 左1右9) ----------\r\n");
            cprint("    1 DAT(棕)   2 CMD(橙)   3 NC(7.4V振动电源, 不接!)\r\n");
            cprint("    4 GND(黑)   5 VCC 3.3V(蓝)   6 ATT/CS   7 CLK   8 NC   9 ACK(悬空)\r\n");
            cprint("    接法(0924M 硬件SPI): 1 DAT→PB4  2 CMD→PB5  4→GND  5→3.3V\r\n");
            cprint("                         6 ATT→PB1  7 CLK→PB3\r\n");
            cprint("    ⚠ 线色各厂不同, 以引脚序号为准! 3 号脚接 3.3V 会发热/异常。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "pin") == 0) {
            /* 接线定位 (0924K): 分阶段拉低 CS/CLK/CMD 各 4 s, 万用表直流
             * 电压档在插头孔上找"哪个孔变 0V" —— 不靠猜, 直接定位每根线
             * 实际接在哪。探测期 Poll 挂起, 不会打架。 */
            static const struct { uint8_t ph; const char* pin; const char* hole; } PH[3] = {
                {1U, "PB1 (CS/ATT)",   "6 号孔 ATT"},
                {2U, "PB3 (SPI1 SCK)", "7 号孔 CLK"},
                {3U, "PB5 (SPI1 MOSI)","2 号孔 CMD"},
            };
            cprint("\r\n  ===== rc pin: 接线定位 (约 12 s, 需万用表) =====\r\n");
            cprint("  方法: 万用表打直流电压档, 黑表笔接板子 GND,\r\n");
            cprint("  红表笔依次戳插头的孔。某个阶段读到 0V 的孔 = 正被拉低的那根线;\r\n");
            cprint("  若阶段里预期孔始终 3.3V ⇒ 那根线没接在这个孔上(接错位/虚接)。\r\n");
            cprint("  DAT(PB4/SPI MISO) 是输入脚拉不低 —— 2/6/7 号孔都定位后, 剩下的信号孔就是 1 号 DAT。\r\n");
            for (uint32_t i = 0U; i < 3U; i++) {
                cprint("  阶段 %u/3: 正在拉低 %s —— 快戳 %s (4 s)\r\n",
                       (unsigned)(i + 1U), PH[i].pin, PH[i].hole);
                ArmRc_PinDrive(PH[i].ph);
                HAL_Delay(4000U);
            }
            ArmRc_PinDrive(0U);
            cprint("  ✔ 探测结束, 四脚已回空闲高, 遥控轮询恢复。\r\n");
            cprint("  把每个孔的实测结果回给我: 哪个孔在哪个阶段变 0V / 全程 3.3V。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "map") == 0) {
            /* 键位映射: rc map                查看
             *           rc map <功能> <按键>  设 (功能: open close home fold rec play j5ccw j5cw)
             *           rc map save           存 flash 区 E, 掉电不丢
             *           rc map def            恢复默认 (不落盘) */
            static const char* const FN_NAMES[ARM_RC_FN_COUNT] = {
                "open", "close", "home", "fold", "rec", "play", "j5ccw", "j5cw"
            };
            static const char* const FN_DESC[ARM_RC_FN_COUNT] = {
                "夹爪张开(长按)", "夹爪闭合(长按)", "中位(六轴回180)",
                "折叠位(POSE_HOME)", "示教录制 开/停", "回放/停止回放",
                "J5逆时针(长按)", "J5顺时针(长按)"
            };
            const uint8_t* mp0 = ArmRc_MapGet();

            if (ntok >= 3 && strcmp(tokens[2], "save") == 0) {
                cprint("\r\n  rc map save: 写扇区 11 区 E ... (阻塞 ~1s, 勿在运动中敲)\r\n");
                cprint("  %s\r\n\r\n", ArmRc_MapSave()
                       ? "✔ 已存 flash, 掉电不丢" : "✗ 写入失败(看 `lk`/flash 状态)");
                return;
            }
            if (ntok >= 3 && strcmp(tokens[2], "def") == 0) {
                ArmRc_MapDefaults();
                cprint("\r\n  ✔ 已恢复默认键位 (仅 RAM, 要固化再 rc map save):\r\n");
                for (uint32_t i = 0U; i < ARM_RC_FN_COUNT; i++) {
                    cprint("    %-5s -> %s\r\n", FN_NAMES[i],
                           mp0[i] ? ArmRc_BtnName(mp0[i]) : "(未绑定)");
                }
                cprint("\r\n");
                return;
            }
            if (ntok >= 4) {
                uint8_t fn = 0xFFU;
                for (uint32_t i = 0U; i < ARM_RC_FN_COUNT; i++) {
                    if (strcmp(tokens[2], FN_NAMES[i]) == 0) { fn = (uint8_t)i; break; }
                }
                if (fn == 0xFFU) {
                    cprint("  ✗ 未知功能 '%s'。可用: open close home fold rec play j5ccw j5cw\r\n\r\n",
                           tokens[2]);
                    return;
                }
                uint8_t code = ArmRc_BtnFromName(tokens[3]);
                if (tokens[3][0] != 0U && code == 0U && strcmp(tokens[3], "0") != 0) {
                    cprint("  ✗ 未知按键 '%s'。可用: SELECT L3 R3 START UP RIGHT DOWN"
                           " LEFT L2 R2 L1 R1 TRI CIR CRS SQR (或 1~16 / none)\r\n\r\n",
                           tokens[3]);
                    return;
                }
                uint8_t mr = ArmRc_MapSet(fn, code);
                if (mr == 1U) { cprint("  ✗ 参数越界\r\n\r\n"); return; }
                if (mr == 2U) {
                    cprint("  ✗ '%s' 已被别的功能占用 (一个键只干一件事)。"
                           "先把它处改成 none 再绑。\r\n\r\n", tokens[3]);
                    return;
                }
                cprint("\r\n  ✔ %s(%s) -> %s   (仅 RAM! 要掉电不丢再敲: rc map save)\r\n\r\n",
                       FN_NAMES[fn], FN_DESC[fn],
                       code ? ArmRc_BtnName(code) : "(未绑定)");
                return;
            }
            cprint("\r\n  ===== rc map 键位映射 =====\r\n");
            cprint("  功能          按键        说明\r\n");
            for (uint32_t i = 0U; i < ARM_RC_FN_COUNT; i++) {
                cprint("  %-6s(%-4s) -> %-10s %s\r\n", FN_NAMES[i], FN_DESC[i],
                       mp0[i] ? ArmRc_BtnName(mp0[i]) : "(未绑定)", "");
            }
            cprint("  可用按键: SELECT L3 R3 START UP RIGHT DOWN LEFT\r\n");
            cprint("            L2 R2 L1 R1 TRI CIR CRS SQR (或编号 1~16)\r\n");
            cprint("  设映射 : rc map <功能> <按键>   例: rc map play SQR\r\n");
            cprint("  存 flash: rc map save (区 E, 掉电不丢)\r\n");
            cprint("  恢复默认: rc map def\r\n");
            cprint("  ⭐ 夹爪(open/close)是**长按**语义: 按住动, 两个都松 = 停;\r\n");
            cprint("     其余是**边沿**语义: 按下瞬间触发一次。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "cpha") == 0) {
            /* 运行时切换 SPI 时钟相位, 用来验证 PS2 手柄到底需要 Mode2 还是 Mode3 */
            const char* sub2 = (ntok >= 3) ? tokens[2] : "";
            if (strcmp(sub2, "0") == 0 || strcmp(sub2, "mode2") == 0) {
                ArmRc_SetCpha(0U);
                cprint("\r\n  rc cpha 0: SPI 已切 Mode2 (CPOL=1, CPHA=0),"
                       "手柄配置序列已重跑。\r\n");
            } else if (strcmp(sub2, "1") == 0 || strcmp(sub2, "mode3") == 0) {
                ArmRc_SetCpha(1U);
                cprint("\r\n  rc cpha 1: SPI 已切 Mode3 (CPOL=1, CPHA=1),"
                       "手柄配置序列已重跑。\r\n");
            } else {
                cprint("\r\n  用法: rc cpha 0 | rc cpha 1\r\n");
                cprint("  0 = Mode2 (CPOL=1, CPHA=0); 1 = Mode3 (CPOL=1, CPHA=1, 默认)\r\n");
                cprint("  切完后立即生效并自动重跑配置序列, 不用重新烧录。\r\n\r\n");
                return;
            }
            cprint("  请立即敲 `rc diag` 看三帧, 哪帧出现 FF 73/41 5A ... 即当前模式可用。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "phase") == 0) {   /* 0924CZ: 采样相位 (换手柄用) */
            const char* sub2 = (ntok >= 3) ? tokens[2] : "";
            if (*sub2) {
                if (strcmp(sub2, "0") == 0) {
                    ArmRc_SetPhase(0U);
                    cprint("\r\n  rc phase 0: 已固定【提前一槽】时序 (0924CZ, 旧克隆手柄)。\r\n");
                } else if (strcmp(sub2, "1") == 0) {
                    ArmRc_SetPhase(1U);
                    cprint("\r\n  rc phase 1: 已固定【标准 Mode3】时序 (新接收机)。\r\n");
                } else if (strcmp(sub2, "a") == 0 || strcmp(sub2, "auto") == 0) {
                    ArmRc_SetPhase(2U);
                    cprint("\r\n  rc phase a: 自动判型已开 —— 按 mark(0x5A) 移位特征每帧自校,\r\n");
                    cprint("  新旧手柄通吃, 连续 3 帧移位才切换 (~60ms), 不用重烧。\r\n");
                } else {
                    cprint("\r\n  用法: rc phase 0 | rc phase 1 | rc phase a\r\n");
                    cprint("  0=提前一槽(旧克隆) 1=标准Mode3(新接收机) a=自动判型(默认)\r\n\r\n");
                    return;
                }
            } else {
                cprint("\r\n  采样相位: %s (%s)\r\n  用法: rc phase 0 | rc phase 1 | rc phase a\r\n\r\n",
                       ArmRc_GetPhaseEarly() ? "提前一槽 (CWb, 旧克隆)" : "标准 Mode3",
                       ArmRc_GetPhaseAuto() ? "自动判型 ON" : "手动固定");
                return;
            }
            cprint("  请立即敲 `rc diag` 看三帧, ID/mark 对齐即当前时序可用。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "gate") == 0) {    /* 0924CZb: 模式闸门 (红灯模拟族=工作默认) */
            const char* sub2 = (ntok >= 3) ? tokens[2] : "";
            if (*sub2) {
                if (strcmp(sub2, "g") == 0 || strcmp(sub2, "green") == 0) {
                    ArmRc_SetGate(1U);
                    cprint("\r\n  rc gate g: 【绿灯(数字族) = 工作】红灯(模拟族)帧全部禁行\r\n");
                    cprint("  (按键/摇杆清零回中, 等效未武装; 开机也不再自动拉模拟)。\r\n");
                } else if (strcmp(sub2, "r") == 0 || strcmp(sub2, "red") == 0) {
                    ArmRc_SetGate(0U);
                    cprint("\r\n  rc gate r: 【红灯(模拟族) = 工作】绿灯(数字族)禁行 (传统行为)。\r\n");
                    cprint("  如需拉模拟: 敲 `rc reinit` (会自动直发 0x44 模拟命令)。\r\n");
                } else {
                    cprint("\r\n  用法: rc gate g | rc gate r\r\n");
                    cprint("  r=红灯(模拟)工作/绿灯全禁 (默认)   g=绿灯工作/红灯禁行\r\n\r\n");
                    return;
                }
            } else {
                cprint("\r\n  模式闸门: %s   当前禁行: %s\r\n", 
                       ArmRc_GetGate() ? "绿灯(数字族)工作 / 红灯(模拟族)全禁" : "红灯(模拟族)工作 / 绿灯禁行",
                       ArmRc_GetGateBlock() ? "本帧被禁行 (绿灯族)" : "无 (工作模式在放行)");
                cprint("  用法: rc gate g | rc gate r\r\n\r\n");
                return;
            }
            cprint("  立即敲 `rc` 看闸门状态行, 推杆/按键验证当前模式是否放行。\r\n\r\n");
            return;
        }

        if (strcmp(sub, "cfg") == 0) {
            /* 0924CZ: 手柄配置序列开关 —— 默认纯轮询(适配 2.4G 克隆接收器),
             * 原装手柄需要拉回红灯模拟时打开。 */
            const char* sub2 = (ntok >= 3) ? tokens[2] : "";
            if (strcmp(sub2, "0") == 0 || strcmp(sub2, "off") == 0) {
                ArmRc_SetCfgEn(0U);
                cprint("\r\n  rc cfg 0: 已切【纯轮询】—— 从不发任何配置命令(0x43/0x44),\r\n");
                cprint("  只发 0x42 轮询帧。这是 2.4G 克隆接收器的正确姿势 (ESP32 直测\r\n");
                cprint("  证据: 裸轮询回 ID=0x20 且按键/摇杆正常)。掉线后不再自动重配。\r\n");
                cprint("  ⭐ 若手柄当前卡 F3(配置模式): 请给接收器断电重上电清状态。\r\n\r\n");
            } else if (strcmp(sub2, "1") == 0 || strcmp(sub2, "on") == 0) {
                ArmRc_SetCfgEn(1U);
                cprint("\r\n  rc cfg 1: 已切【配置序列】—— 上电/掉线重配时发完整\r\n");
                cprint("  SetInit(进配置→红灯模拟锁存→退出), 供真·原装手柄用。\r\n");
                cprint("  ⚠ 2.4G 克隆接收器会被推进 F3 且退不出来 —— 别对它用!\r\n\r\n");
            } else if (strcmp(sub2, "a") == 0 || strcmp(sub2, "direct") == 0) {
                /* 0924CZ: 不进配置模式直发 0x44 模拟命令 —— 克隆手柄旁路试探 */
                ArmRc_CfgAnalogDirect();
                cprint("\r\n  rc cfg a: 已直发 0x44 模拟命令(不进/退配置模式):\r\n");
                cprint("  3×轮询 → 5ms → 0x44(模拟+锁存) → 5ms → 3×轮询。\r\n");
                cprint("  请立即 `rc diag` 看帧: ID 变 0x73(或 F3 且摇杆~80) = 模拟已开;\r\n");
                cprint("  仍 C1 且摇杆全 FF = 克隆手柄不理会, 试 `rc cfg 1`(零尾退出版)。\r\n\r\n");
            } else {
                cprint("\r\n  当前配置序列: %s\r\n", ArmRc_GetCfgEn() ? "开 (cfg 1)" : "关 (纯轮询, 默认)");
                cprint("  用法: rc cfg 0 | rc cfg 1\r\n");
                cprint("  0 = 纯轮询 (0924CZ 默认, 克隆接收器/非标无线用这个)\r\n");
                cprint("  1 = 跑 SetInit 配置序列 (原装手柄绿灯时拉回红灯用)\r\n");
                cprint("  即时生效, 不用重烧; 掉线自动重配只在 cfg 1 下才跑。\r\n\r\n");
            }
            return;
        }

        /* ---- 默认: 总览 (2026-09-24 起: PS2 手柄) ---- */
        const ArmRcState* s = ArmRc_Get();
        uint32_t age = (s->last_frame_ms != 0U) ? (s->now_ms - s->last_frame_ms) : 0U;

        cprint("\r\n  ===== rc PS2 手柄输入 (硬件 SPI1: DAT=PB4 CMD=PB5 CLK=PB3 CS=PB1) =====\r\n");
        cprint("  驱动     : %s    配置序列: %s\r\n",
               s->inited ? "GPIO ✔" : "**失败** ✗",
               ArmRc_GetCfgEn() ? "开 (rc cfg 1)" : "关 = 纯轮询 (0924CZ 默认)");
        cprint("  链路     : %s", s->live ? "live ✔ (手柄在通信)" : "**无信号 ✗**");
        if (s->last_frame_ms != 0U) {
            cprint(", 最近有效帧距今 %lu ms", (unsigned long)age);
        } else {
            cprint(", 从没收到过有效帧");
        }
        {
            uint8_t id7 = (uint8_t)(s->id & 0x7FU);
            const char* idtxt;
            if (ARM_RC_ID_ANALOG_OK(s->id) || s->id == 0x73U) {
                idtxt = "红灯模拟 ✔ (0x20/0x79=非标无线)";
            } else if (s->id == 0xF3U) {
                idtxt = "0x73+末位毛刺: 摇杆合理=模拟✔ / 离轨=配置模式卡死";
            } else if (id7 == 0x41U) {
                idtxt = "绿灯数字 (摇杆无效, 只按键; 0924CZ 按键已放行)";
            } else {
                idtxt = "无响应";
            }
            cprint("\r\n  手柄 ID  : 0x%02X %s   帧统计: ok=%lu fail=%lu (连续失败 %lu)\r\n",
                   (unsigned)s->id, idtxt,
                   (unsigned long)s->poll_ok, (unsigned long)s->poll_fail,
                   (unsigned long)s->fail_run);
        }
        cprint("  模式闸门 : %s%s\r\n",
               ArmRc_GetGate() ? "绿灯(数字族)=工作, 红灯(模拟族)=全禁 [rc gate 切换]"
                               : "红灯(模拟族)=工作, 绿灯(数字族)=全禁 [rc gate 切换]",
               ArmRc_GetGateBlock() ? "  ★ 当前帧禁行中" : "");
        cprint("  按键字节 : D3=%02X D4=%02X (按下=0; D4: L2 R2 L1 R1 △ ○ ╳ □)\r\n",
               (unsigned)s->btn3, (unsigned)s->btn4);
        cprint("  失控计数 : %lu 次 (有信号→无信号; PS2 下恢复本义: 手柄关机/离场即 +1)\r\n",
               (unsigned long)s->lost_events);

        cprint("  ---------- 逐通道 ----------\r\n");
        cprint("  通道       CH1     CH2     CH3     CH4     CH5     CH6\r\n");
        cprint("  来源       RX      RY      LY      LX    R1/L1    链路\r\n");
        cprint("  用途        X       Y       Z      J5     夹爪    心跳\r\n");
        cprint("  脉宽(µs)");
        for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
            cprint("  %5u", (unsigned)s->ch[i].width_us);
        }
        cprint("\r\n");

        cprint("  中位(µs)");
        if (s->mid_valid) {
            for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
                if ((s->mid_mask >> i) & 1U) cprint("  %5u", (unsigned)s->mid[i]);
                else                         cprint("   ----");
            }
        } else {
            cprint(" 未标定");
        }
        cprint("\r\n");

        cprint("  行程(µs)");
        for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
            if (s->ch[i].frames > 0U) {
                cprint(" %3u~%-3u", (unsigned)s->ch[i].w_min, (unsigned)s->ch[i].w_max);
            } else {
                cprint("   ----");
            }
        }
        cprint("\r\n");

        cprint("  帧数    ");
        for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
            cprint("  %5lu", (unsigned long)s->ch[i].frames);
        }
        cprint("\r\n  新鲜    ");
        for (uint32_t i = 0U; i < ARM_RC_NCH; i++) {
            cprint("  %4s", s->ch[i].fresh ? "✔" : "✗");
        }
        cprint("\r\n");

        cprint("  ---------- 结论 ----------\r\n");
        if (!s->inited) {
            cprint("  ✗ GPIO 初始化失败 ⇒ 先看 HAL 返回值。\r\n");
        } else if (s->poll_ok == 0U && s->poll_fail == 0U) {
            cprint("  (还没有轮询过 —— 等一拍再看)\r\n");
        } else if (s->poll_ok == 0U) {
            cprint("  ✗ 一帧都没收到过 ⇒ 按可能性查:\r\n");
            cprint("    ① 手柄没开机/没电(手柄灯闪=在搜接收器, 常亮=已配对)\r\n");
            cprint("    ② 接收器电源没上(VDD=3.3V)/GND 断\r\n");
            cprint("    ③ 接线错: DAT=PB4 CMD=PB5 CLK=PB3 CS=PB1, 5V/GND 别反;\r\n");
            cprint("       慢闪=等待配对; 快闪后变慢闪=收到时钟但通信没通, 试 `rc cpha 0` 切 Mode2。\r\n");
        } else if (!s->live) {
            cprint("  ⚠ 历史上有帧(%lu)但**当前无信号** ⇒ 手柄已关/超出无线距离。\r\n",
                   (unsigned long)s->poll_ok);
        } else if (!s->analog_ok) {
            cprint("  ⚠ 手柄在绿灯(数字)模式 ⇒ 摇杆无模拟量, 按键已放行可用 (0924CZ)。\r\n");
            cprint("    先试 `rc cfg a`(直发 0x44 模拟命令) 再 `rc diag` 看摇杆是否出 ~80;\r\n");
            cprint("    仍数字 = 这套手柄只能按键控 (方向键映射需另做)。\r\n");
        } else {
            cprint("  ✔ 通信正常, 红灯模拟模式。快速自检:\r\n");
            cprint("    ① 摇杆回中 ⇒ CH1~CH4 应 =1500; 推杆 ⇒ 对应通道 1000~2000 变化\r\n");
            cprint("    ② 按住夹爪张开键(默认 R1) ⇒ CH5 递增到 2000; 闭合键(默认 L1) ⇒ 递减\r\n");
            cprint("    ③ `rc keysw 30` 后随便按键 ⇒ 变化的键即时打印 (找到想要的键)\r\n");
            cprint("    ④ `rc map <功能> <键>` 改键位; `rc map save` 存 flash\r\n");
            cprint("    ⑤ 方向反了改 arm_rc_ctrl.c 的 RC_SIGN_*(别改采集层映射)\r\n");
            cprint("    ⑥ 然后 `rc go on` 驱臂 (人离开扫过区!)\r\n");
        }
        cprint("  用法: rc | rc raw | rc watch [n] | rc keys | rc keysw [s] | rc diag | rc pin | rc map | rc cpha | rc phase | rc gate | rc cfg\r\n");
        cprint("        rc cfg 0|1 | rc mid | rc spd | rc go | rc reset | rc reinit\r\n\r\n");
        return;
    }

    if (strcmp(tokens[0], "cx") == 0) {
        uint8_t cur = ArmCanDrv_GetInterface();

        if (ntok < 2) {
            cprint("\r\n  ===== cx CAN interface =====\r\n");
            cprint("  当前正在用 : %s %s\r\n", ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
            cprint("  cx 1       -> CAN1 (控制器换回 CAN1, 引脚用下面 pa/pb 选的那对)\r\n");
            cprint("  cx 2       -> CAN2  %s           (复用 AF9_CAN2)\r\n",
                   ARM_CAN2_PIN_DESC);
            cprint("  cx pa      -> CAN1 引脚换 PA11(RX)/PA12(TX)   [当前 CAN1 用 %s]\r\n",
                   ArmCanDrv_Can1PinDesc(ArmCanDrv_GetCan1Pins()));
            cprint("  cx pb      -> CAN1 引脚换 PB8(RX)/PB9(TX)\r\n");
            cprint("  cx save    -> 把**控制器**选择写进 Flash(CAN 配置区), 重启后仍生效\r\n");
            cprint("  cx reset   -> 控制器选择回 CAN1 并落盘\r\n");
            cprint("  ⚠ 引脚对(pa/pb)不落盘: 复位后回到编译期 CANPIN 的默认值\r\n");
            cprint("  ⚠ 波特率/节点号/心跳不受影响(它们由 `can` 命令管)\r\n");
            cprint("  ⚠ 现场提示: ps 的 ★ 落在 PB12/PB13 ⇒ cx 2; 落在 PA11/PA12 ⇒ cx pa\r\n");
            cprint("  ============================\r\n\r\n");
            return;
        }

        if (strcmp(tokens[1], "save") == 0) {
            if (CanCfg_SetIface(cur) == 0 && CanCfg_Save() == 0) {
                cprint("  ✓ 已写入 Flash: 接口 = %s %s, 重启后仍生效\r\n",
                       ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
            } else {
                cprint("  ✗ Flash 写入失败 (扇区 11 CAN 区)。本次运行仍是 %s\r\n",
                       ArmCanDrv_IfName());
            }
            return;
        }
        if (strcmp(tokens[1], "reset") == 0) {
            (void)ArmCanDrv_SetInterface(ARM_CAN_IF_CAN1);
            (void)CanCfg_SetIface(ARM_CAN_IF_CAN1);
            (void)CanCfg_Save();
            cprint("  ✓ 接口已回到 %s %s 并落盘\r\n",
                   ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
            return;
        }

        /* ---- cx pa / cx pb: 换 CAN1 的那一对脚 (2026-09-18) ---- */
        if (strcmp(tokens[1], "pa") == 0 || strcmp(tokens[1], "pb") == 0) {
            uint8_t want = (tokens[1][1] == 'a') ? ARM_CAN1_PINS_PA : ARM_CAN1_PINS_PB;
            if (want == ArmCanDrv_GetCan1Pins()) {
                cprint("  CAN1 引脚已经是 %s 了。\r\n",
                       ArmCanDrv_Can1PinDesc(want));
                return;
            }
            cprint("\r\n  切换 CAN1 引脚 %s -> %s ...\r\n",
                   ArmCanDrv_Can1PinDesc(ArmCanDrv_GetCan1Pins()),
                   ArmCanDrv_Can1PinDesc(want));
            if (ArmCanDrv_SetCan1Pins(want)) {
                cprint("  ✓ 现在是 %s %s\r\n",
                       ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
                if (ArmCanDrv_GetInterface() == ARM_CAN_IF_CAN2) {
                    cprint("  ⚠ 当前控制器是 CAN2 ⇒ 这个选择已记住, 敲 cx 1 切回 CAN1 时生效\r\n");
                } else {
                    cprint("  下一步: cn 看状态 -> cw 验外环 -> (上位机灌帧) ps 看 ★ 落哪只脚\r\n");
                }
            } else {
                cprint("  ✗ 切换失败 (HAL_CAN_Init/ConfigFilter/Start 没过) —— 敲 cn 看详情\r\n");
                cprint("     现在处于未初始化态; 再敲一次 cx pa/pb 可换回去\r\n");
            }
            return;
        }

        {
            int v = atoi(tokens[1]);
            if (v != (int)ARM_CAN_IF_CAN1 && v != (int)ARM_CAN_IF_CAN2) {
                cprint("  ✗ 只支持 cx 1 / cx 2 / cx pa / cx pb / cx save / cx reset\r\n");
                return;
            }
            if ((uint8_t)v == cur) {
                cprint("  已经在 %s %s 上了。\r\n",
                       ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
                return;
            }
            cprint("\r\n  切换 %s -> CAN%d ...\r\n", ArmCanDrv_IfName(), v);
            if (ArmCanDrv_SetInterface((uint8_t)v)) {
                cprint("  ✓ 现在是 %s %s, hw init ok\r\n",
                       ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
                cprint("  下一步: cn 看状态 -> cp 量脚 -> cw 验外环 -> cz 验发侧\r\n");
                cprint("          -> cy 2500 看对端发的时候有没有电平 -> ct 对拍帧数\r\n");
                cprint("  想重启后仍生效: cx save\r\n");
            } else {
                cprint("  ✗ 切换失败 (HAL_CAN_Init/ConfigFilter/Start 没过) —— 敲 cn 看详情\r\n");
                cprint("     旧接口已停掉, 现在处于未初始化态; 可再敲 cx 换回去\r\n");
            }
            return;
        }
    }

    /* ===== CAN 引脚直流探测 (cp) =====
     * UART4 那一轮最大的时间黑洞是"分不清引脚没接、还是线不对"。CAN 这边
     * 同样会踩, 所以直接量脚: 收发器的 RXD 是推挽输出、空闲(隐性)为高,
     * 给它加内部下拉还能读到 1 ⇒ 外面真的有驱动 = 收发器已上电且接上了。 */
    if (strcmp(tokens[0], "cp") == 0) {
        uint8_t rx_pd = 0U, rx_pu = 0U, tx_lo = 0U, tx_hi = 0U;
        uint8_t ok;
        char bb[2][9];

        cprint("\r\n  ===== cp CAN pin DC probe (%s %s) =====\r\n",
               ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        cprint("  位图 = 连采 8 次, 左=第1次。00000000=稳定低 11111111=稳定高 混合=有活动信号\r\n");
        cprint("  ⚠ 探测会把 TX 脚拉低几微秒(总线被拉一下), 对端可能记一个错误, 属正常\r\n");
        /* 切 GPIO 之前先读一次 bxCAN 对 CAN_RX 的采样 —— 这是最快的
         * "模块有没有上电"判据: CAN 空闲=隐性=高, 一根线都不用查。 */
        {
            uint8_t bus_rx = ArmCanDrv_BusRxLevel();
            cprint("  探测前总线: MSR.RX=%u (%s)\r\n", (unsigned)bus_rx,
                   bus_rx ? "隐性/空闲 ⇒ 收发器已上电且在输出高"
                          : "显性/被拉低 ⇒ 模块没供电 或 总线被卡死");
        }
        ok = ArmCanDrv_PinProbe(&rx_pd, &rx_pu, &tx_lo, &tx_hi);
        cprint("  RX 脚 (%s, 应接模块的 RX/RXD 输出)\r\n", ArmCanDrv_PinDesc());
        cprint("     内部下拉 -> %s   内部上拉 -> %s\r\n",
               bin8(rx_pd, bb[0]), bin8(rx_pu, bb[1]));
        if (rx_pd == 0xFFU && rx_pu == 0xFFU) {
            cprint("     -> [被外部驱高] 收发器已接且已上电(RXD 空闲=隐性=高) ✔\r\n");
        } else if (rx_pd == 0x00U && rx_pu == 0xFFU) {
            cprint("     -> [悬空] 外面没有驱动 ⇒ 模块 RX 这根线没接到该脚, 或模块没上电 ✘\r\n");
        } else if (rx_pd == 0x00U && rx_pu == 0x00U) {
            cprint("     -> [被外部驱低] 总线卡在显性(对端正在发/短路)或该脚被别的信号占用\r\n");
        } else {
            cprint("     -> [有活动信号] 该节点上有 0/1 变化 ⇒ 总线在跑或该脚串了别的信号\r\n");
        }
        cprint("  TX 脚 (%s, 应接模块的 TX/TXD 输入)\r\n", ArmCanDrv_PinDesc());
        cprint("     推挽输出低 -> %s   推挽输出高 -> %s\r\n",
               bin8(tx_lo, bb[0]), bin8(tx_hi, bb[1]));
        if (tx_lo == 0x00U && tx_hi == 0xFFU) {
            cprint("     -> [驱动级正常]\r\n");
        } else {
            cprint("     -> [驱动级异常] 该脚可能被短死/被外部顶住 —— 换另一组脚再试\r\n");
            cprint("        cx 2 (CAN2 PB12/PB13) 或 cx pa (CAN1 PA11/PA12) —— 都是运行时切, 不用重烧\r\n");
        }
        cprint("  CAN 恢复: %s\r\n", ok ? "ok" : "FAILED (敲 cn 看 hw init)");
        cprint("  ==========================================\r\n\r\n");
        return;
    }

    /* ===== CAN 总线电平监听 (cy [窗口ms]) =====
     * 单向判据: 只要对端在发帧, 总线电平就一定跳变 —— 不需要对端回报,
     * 也不需要我方发送。它把"链路通不通"从"要双向 ACK"降级成"看一根线的
     * 活动", 所以在 lec=3(无人应答) 死局里是唯一还能取证的手段。
     * 采样源 = bxCAN 对 CAN_RX 引脚的实时电平 (MSR.RX, 硬件内部), 因此
     * 与引脚被配成 AF 无关, 也不用切 GPIO。
     * ⚠ 主臂测之前必须先 `mf 0` 暂停转发, 否则测到的是自己的帧。 */
    if (strcmp(tokens[0], "cy") == 0) {
        uint32_t win = 500U, i, hi = 0U, lo = 0U, edges = 0U;
        uint8_t  prev = 0xFFU;
        if (ntok >= 2) {
            int v = atoi(tokens[1]);
            if (v >= 50 && v <= 10000) win = (uint32_t)v;
        }
        cprint("\r\n  ===== cy CAN bus level watch (%lu ms, 每 1ms 采 1 次) =====\r\n",
               (unsigned long)win);
        cprint("  采样 bxCAN 的 CAN_RX 引脚实时电平 (MSR.RX), 不要求对端应答\r\n");
        cprint("  当前接口 = %s %s\r\n", ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        cprint("  隐性(总线空闲)=高   显性(有帧在跑)=低\r\n");
        for (i = 0U; i < win; i++) {
            uint8_t lv = ArmCanDrv_BusRxLevel();
            if (lv) { hi++; } else { lo++; }
            if (prev != 0xFFU && prev != lv) { edges++; }
            prev = lv;
            HAL_Delay(1);
        }
        cprint("  高 %lu 次   低 %lu 次   跳变 %lu 次\r\n",
               (unsigned long)hi, (unsigned long)lo, (unsigned long)edges);
        if (lo == 0U) {
            cprint("  -> [总线恒定隐性] 总线上一点活动都没有\r\n");
            cprint("     对端没在发 / 链路断 / 对端收发器没上电 / 对端模块压根没接\r\n");
        } else if (hi == 0U) {
            cprint("  -> [总线恒定显性] 被拉死了(短路 / 对端卡在错误帧 / 两端都没终端电阻)\r\n");
        } else {
            cprint("  -> [总线在活动] 有跳变 ⇒ CAN_RX 方向通了, 对面确实在发帧 ✔\r\n");
            cprint("     若此时 ct 仍 lec=3 ⇒ 是**我方发不出去**, 敲 cz 继续定位\r\n");
        }
        cprint("  ==========================================\r\n\r\n");
        return;
    }

    /* ===== CAN 发侧通路探测 (cz) =====
     * 回答"我方把 CAN1_TX 拉低, 到底有没有变成总线上的显性位"。
     * 发 1 帧, 同时以微秒级高频采样 MSR.RX: 模块真把 TXD 送上了总线 ⇒
     * 发送期间总线必出现显性(低) ⇒ lo>0。
     * lo 全程 = 0 而 cp 里 TX 驱动自检正常 ⇒ 问题在模块侧(没供电 / TXD 没
     * 接到 MCU 的 CAN1_TX / 电平跳线错 / 器件与供电电压不匹配)。 */
    if (strcmp(tokens[0], "cz") == 0) {
        /* 载荷全 0 = 显性位最多 ⇒ "总线被拉低"的特征最明显, 采样最容易撞上。
         * ⚠ 2026-09-17 修正: 旧版**只发 1 帧**再采样, 而 500k/8字节的帧只有 ~190us,
         *   采样循环也只有 ~120us ⇒ 两个窗口几乎不重叠。实测出现过自相矛盾的输出:
         *     lo=0(说"总线没动") 却 tec+8/lec=3(证明帧确实发出去了、且位监视吻合)
         *   ⇒ 会把"采样没撞上"误判成"模块没驱动总线", 把人往错误方向带。
         *   现在改成**连发 N 帧 + 每帧之间全程采样**, 必然重叠。 */
        static const uint8_t pl[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint8_t  tec = 0U, rec = 0U, lec = 0U;
        uint32_t lo = 0U, hi = 0U, edges = 0U, i, k;
        uint8_t  prev = 0xFFU;
        uint16_t n = 20U, ok = 0U;

        if (ntok >= 2) {
            int v = atoi(tokens[1]);
            if (v > 0 && v <= 200) n = (uint16_t)v;
        }
        cprint("\r\n  ===== cz CAN TX-path probe (连发 %u 帧) =====\r\n", (unsigned)n);
        cprint("  接口 %s %s\r\n", ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        cprint("  发 0x%03X (8 字节全 0 = 显性位最多), 发的同时全程采样 MSR.RX\r\n",
               (unsigned)ARM_CAN_TEST_ID);
        for (i = 0U; i < (uint32_t)n; i++) {
            if (ArmCanDrv_Tx((uint16_t)ARM_CAN_TEST_ID, pl, 8U)) {
                ok++;
            }
            for (k = 0U; k < 2000U; k++) {       /* 每帧 ~120us 的采样窗口 */
                uint8_t lv = ArmCanDrv_BusRxLevel();
                if (lv) { hi++; } else { lo++; }
                if (prev != 0xFFU && prev != lv) { edges++; }
                prev = lv;
            }
            HAL_Delay(2);                        /* 让发送邮箱与总线都静下来 */
        }
        (void)ArmCanDrv_ErrorState(&tec, &rec, &lec);
        cprint("  入邮箱 %u/%u 帧   高 %lu   低 %lu   跳变 %lu   tec=%u lec=%u\r\n",
               (unsigned)ok, (unsigned)n, (unsigned long)hi, (unsigned long)lo,
               (unsigned)edges, (unsigned)tec, (unsigned)lec);
        if (lo > 0U) {
            cprint("  -> [TX 上总线了] 发送期间 CAN_RX 出现显性 ⇒ 模块确实在驱动差分对 ✔\r\n");
            cprint("     模块供电/TXD 接线/电平都已排除; 对端仍收不到 ⇒ 查线/极性/共地/终端\r\n");
        } else if (lec == 3U) {
            cprint("  -> [本步不作判据] lo=0 但 lec=3(ACK 错): 两条证据互相矛盾\r\n");
            cprint("     lec=3 说明帧**确实发出去了**(位监视吻合, 否则会是 lec=4/5 位错误)\r\n");
            cprint("     ⇒ 是采样窗口没撞上发送窗口, **不能**据此判'模块没驱动总线'。\r\n");
            cprint("       把 N 调大再试(cz 100); 仍然如此就换 cy 看对端在发时有没有电平。\r\n");
        } else {
            cprint("  -> [总线没动] 连发 %u 帧, CAN_RX 全程隐性 且 lec=%u(位错误)\r\n",
                   (unsigned)n, (unsigned)lec);
            cprint("     ⇒ 模块没把信号送上去: 查模块供电 / TXD 接线 / 电平跳线 / 器件型号\r\n");
            cprint("       (cp 里 TX 驱动自检若正常, 即可排除 MCU 侧)\r\n");
        }
        cprint("  ==========================================\r\n\r\n");
        return;
    }

    /* ===== cw 外环测试 (2026-09-17) =====
     * cp 的盲区补丁: cp 读的是"引脚自己"(RX 被外部驱高吗 / TX 能推挽吗),
     * **从没验过** TX -> 收发器 TXD -> 收发器驱动总线 -> 收发器 RXD -> RX
     * 这条外环。TXD 那根线断了, cp 依然全绿, 症状却是"帧发出去了没人收到"。
     *   cw        —— 只做外环判定
     *   cw 5000   —— 先按住显性 5 秒(供万用表在卡端量 H/L), 再做外环判定 */
    if (strcmp(tokens[0], "cw") == 0) {
        uint8_t dom = 0U, rec = 0U;
        uint8_t ok;
        char bb[2][9];
        uint16_t hold_ms = 0U;

        if (ntok >= 2) {
            int v = atoi(tokens[1]);
            if (v > 0 && v <= 30000) hold_ms = (uint16_t)v;
        }
        cprint("\r\n  ===== cw CAN wire-loop (TX -> 收发器 -> 总线 -> 收发器 -> RX) =====\r\n");
        cprint("  接口 %s %s\r\n", ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        cprint("  cp 验的是'RX 被外部驱高'与'TX 自己能推挽'(都读引脚自己);\r\n");
        cprint("  cw 把 TX 脚按住当 TXD 用并直接读 RX —— 验的是**整条外环**。\r\n");
        if (hold_ms) {
            cprint("  先持续驱动显性 %u ms —— 现在拿万用表量**卡那端**的 H/L:\r\n",
                   (unsigned)hold_ms);
            cprint("     CANH≈3.5V / CANL≈1.5V  ⇒ 两根线都通到卡了\r\n");
            cprint("     仍 ≈2.5V / 2.5V       ⇒ 卡没接到臂的这一对端子上\r\n");
            (void)ArmCanDrv_DriveDominant(hold_ms);
        }
        ok = ArmCanDrv_WireLoopTest(&dom, &rec);
        cprint("  TX 驱动显性(低) 时 RX 读到 -> %s\r\n", bin8(dom, bb[0]));
        cprint("  TX 驱动隐性(高) 时 RX 读到 -> %s\r\n", bin8(rec, bb[1]));
        if (dom == 0x00U && rec == 0xFFU) {
            cprint("  -> [外环通] TX→收发器→总线→收发器→RX 整条链正常 ✔\r\n");
            cprint("     ⇒ MCU 侧与收发器都排除。对端仍收不到 ⇒ 断点只在\r\n");
            cprint("       **卡 ↔ 模块 CANH/CANL** 这一段: 没接到位 / 接错端子 / 极性 / 共地\r\n");
        } else if (dom == 0xFFU && rec == 0xFFU) {
            cprint("  -> [外环断] TX 已拉低, RX 仍是高(隐性) ⇒ 收发器没把 TXD 送到总线\r\n");
            cprint("     三种可能, 按顺序查(这是 cp 查不出来的那一类):\r\n");
            cprint("       1) TX 那根线**没接到模块的 TX/TXD 输入**\r\n");
            cprint("       2) 模块 TX/RX 丝印与实际相反 ⇒ 把这两根**对调**再跑一次 \r\n");
            cprint("       3) 模块 VCC 跳线选错(TJA1050/SIT1050 必须 5V) 或器件已坏\r\n");
        } else if (dom == 0x00U && rec == 0x00U) {
            cprint("  -> [总线被卡死] 驱动隐性时 RX 仍低 ⇒ 被外部强拉显性 / H-L 短路\r\n");
        } else {
            cprint("  -> [有活动信号] RX 上出现 0/1 变化 ⇒ 总线上还有别的节点在发\r\n");
        }
        cprint("  CAN 恢复: %s\r\n", ok ? "ok" : "FAILED (敲 cn 看 hw init)");
        cprint("  ==========================================\r\n\r\n");
        return;
    }

    /* ===== ps: CAN 引脚嗅探 (2026-09-18) =====
     * cp 与 cw 都只认"当前这一组 CAN 脚"(默认 PB8/PB9)。它们回答不了更前面的
     * 那个问题: **板子上是不是还有第二个 CAN 口? 手里的 USB-CAN 到底挂在哪只脚上?**
     * 这两种情况的现场症状一模一样(帧发得出去、没人应答、RX 脚一动不动)。
     * ps 把**所有**与 CAN 有关的候选脚逐个切成 GPIO 输入高速采样, 有信号的那只
     * 会出现大量低电平与跳变 —— 也就是"卡的数据落在哪只脚上"。
     *   ps        —— 扫一遍
     * ⚠ 必须**在卡那一端持续灌帧的同时**跑(tools/can_flood.py), 否则没有输入源。 */
    if (strcmp(tokens[0], "ps") == 0) {
        static ArmCanPinStat_t st[ARM_CAN_PINSCAN_N];
        uint8_t n = ArmCanDrv_PinScanCount();
        uint8_t ok;

        cprint("\r\n  ===== ps CAN pin sniffer (CAN 数据落在哪只脚上) =====\r\n");
        cprint("  ⚠ 必须**在卡那一端持续灌帧的同时**跑; 没有输入源就什么都读不出\r\n");
        cprint("  ⚠ 扫描期间 bxCAN 停 ~10ms(对端可能记一次错), 扫完自动恢复\r\n");
        ok = ArmCanDrv_PinScan(st);
        cprint("  脚    无上拉低/总    边沿  下拉后低/总   判读\r\n");
        for (uint8_t i = 0U; i < n; i++) {
            const char* verdict;
            uint16_t low = st[i].low, edge = st[i].edge;
            uint16_t lpd = st[i].low_pd, tot = st[i].total;

            if (low == 0U && lpd == 0U) {
                verdict = "被外部驱高(悬空脚下拉会读低)";
            } else if (edge >= 8U) {
                verdict = "★有活动信号 ⇒ 对端 CAN 在这只脚";
            } else if (lpd >= (uint16_t)(tot - (tot / 16U))) {
                verdict = "悬空/无外部驱动";
            } else {
                verdict = "被外部恒定驱低";
            }
            cprint("  %-5s %5u/%-5u %5u  %5u/%-5u   %s\r\n",
                   ArmCanDrv_PinScanName(i),
                   (unsigned)low, (unsigned)tot, (unsigned)edge,
                   (unsigned)lpd, (unsigned)tot, verdict);
        }
        cprint("  CAN 恢复: %s\r\n", ok ? "ok" : "FAILED (敲 cn 看 hw init)");
        cprint("  --------------------------------------------------------\r\n");
        cprint("  本工程当前在用的 CAN 脚 = %s (%s)\r\n",
               ArmCanDrv_PinDesc(), ArmCanDrv_IfName());
        cprint("  ★ 落在 PB12/PB13 ⇒ 卡接的是板子的 CAN2 口 ⇒ **直接敲 cx 2 切过去再验**\r\n");
        cprint("  ★ 落在 PB8/PB9   ⇒ 卡确实在 CAN1 口上, 问题在收发器或那段线\r\n");
        cprint("  ★ 落在 PA11/PA12 ⇒ 卡接在 CAN1 的**另一组脚**上 ⇒ **直接敲 cx pa 切过去**\r\n");
        cprint("     (2026-09-18 起引脚组也运行期可切, 不用再拉 BOOT0 重烧)\r\n");
        cprint("  11 行全无 ★      ⇒ 卡的信号没进到 MCU 任何一只 CAN 脚上\r\n");
        cprint("  ========================================================\r\n\r\n");
        return;
    }

    /* ===== 主臂: 自环自检 (ml [N]) =====
     * 不看舵机, 直接发 N 帧标准 0x13; 用【CRC 正确的回环帧数】判定 PA0->PA1
     * 通路是否真通。用来回答"NE≈100% 是地弹还是线没接上"。默认 200 帧。 */
    if (strcmp(tokens[0], "ml") == 0) {
#if ARM_MASTER_BUILD
        uint16_t n = 200U;
        if (ntok >= 2) {
            int v = atoi(tokens[1]);
            if (v > 0 && v <= 3000) n = (uint16_t)v;
        }
        if (master_st_phase != 0U) {
            cprint("  [ml] 自检正在跑, 等它出判决再敲。\r\n");
        } else {
            master_selftest_start(n);
        }
#else
        cprint("  'ml' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 主臂: GPIO 级自环测试 (mg) =====
     * 不经过 UART, 直接问最底层的问题: PA0 的电平 PA1 跟不跟。
     * 见 master_gpio_loopback_test() 注释。 */
    if (strcmp(tokens[0], "mg") == 0) {
#if ARM_MASTER_BUILD
        master_gpio_loopback_test();
#else
        cprint("  'mg' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 主臂: 引脚直流环境探测 (mp [端口 引脚号]) =====
     * 不需要跳线, 用内部上/下拉看每只脚外部挂着什么, 并做输出驱动自检。
     * 见 master_pin_probe_* 注释。 */
    if (strcmp(tokens[0], "mp") == 0) {
#if ARM_MASTER_BUILD
        if (ntok >= 3) {
            master_pin_probe_arg(tokens[1], atoi(tokens[2]));
        } else {
            master_pin_probe_mini();
        }
#else
        cprint("  'mp' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 主臂: 输出保持, 配合万用表找脚 (mo A 1 0 / mo r) =====
     * 见 master_pin_hold*() 注释。 */
    if (strcmp(tokens[0], "mo") == 0) {
#if ARM_MASTER_BUILD
        if (ntok >= 2 && (tokens[1][0] == 'r' || tokens[1][0] == 'R')) {
            master_pin_hold_release();
        } else if (ntok >= 4) {
            master_pin_hold(tokens[1], atoi(tokens[2]), atoi(tokens[3]));
        } else {
            cprint("  用法: mo A 1 0 (PA1 输出低并保持) / mo A 1 1 / mo r (释放)\r\n");
        }
#else
        cprint("  'mo' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 主臂: 主动模式 (ma 0|1) =====
     * 主臂默认只读 (Torque OFF 手拖、拒绝一切运动命令)。`ma 1` 上力矩并
     * 放行 g/m/P —— 用于把主臂精确摆位并保持, 或用命令驱动从臂。
     * 转发不受影响: 主臂自己动了, 从臂照旧跟着动。
     * ⚠ 上力矩前必须先把每轴 GoalPos 写成【当前位置】: 舵机上力矩会立刻
     *   朝"当前目标位"跑, 而主臂从没写过目标位 (可能还是上电置中的
     *   raw 2048), 直接上力矩臂会突然甩出去。 */
    if (strcmp(tokens[0], "ma") == 0) {
#if ARM_MASTER_BUILD
        if (ntok >= 2 && tokens[1][0] == '1') {
            uint8_t ok = 1U, bad = 0U;
            for (uint8_t j = 0U; j < 6U; j++) {
                float v;
                int8_t r = read_pos_vote(j, &v);
                if (r != 1) { ok = 0U; bad = j; break; }
                SERVO_SetPositionFast(j, v);   /* goal = 当前位 -> 原地保持 */
            }
            if (!ok) {
                cprint("  ma 1 aborted: J%u position unreadable (bus/ID?).\r\n",
                       (unsigned)bad + 1U);
                return;
            }
            for (uint8_t j = 0U; j < 6U; j++) SERVO_SetTorque(j, 1U);
            master_hold_en = 1U;
            cprint("  Master ACTIVE (ma 1): torque ON, motion commands accepted.\r\n");
            cprint("    Holds its current pose; g/m/P now drive it.\r\n");
            cprint("    WARNING: keep hands clear. Forwarding still ON -> slave follows.\r\n");
        } else {
            if (Trajectory_IsActive()) Trajectory_Stop();
            swing_active = 0U;
            correction_pending = 0U;
            master_hold_en = 0U;
            master_release_all();
            cprint("  Master PASSIVE (ma 0): stopped, torque OFF, drag by hand.\r\n");
        }
#else
        cprint("  'ma' is a MASTER-role command; this is the SLAVE build.\r\n");
#endif
        return;
    }

    /* ===== 关节角快照 (ms) — 两块板都能用 =====
     * 打印 6 轴"相对各自 Home 的逻辑关节角"。主臂构建额外带发送计数。
     * 联调用法: 主/从摆成同一姿态, 两边各敲一次 `ms`, 两组数字应逐轴吻合。 */
    if (strcmp(tokens[0], "ms") == 0) {
#if ARM_MASTER_BUILD
        master_print_status();
#else
        cprint("  Slave: stream follow %s\r\n",
               stream_follow_en ? "ON" : "OFF (sf 0)");
        print_joint_angles("Slave");
        cprint("  Calibration check: run 'ms' on the MASTER in the same pose and compare.\r\n");
#endif
        return;
    }

    /* Hand-teaching commands: full-word tokens to avoid collision with
     * single-letter 't', 'r', 'p' commands. */
    if (strcmp(tokens[0], "teach") == 0) {
        if (ntok >= 2 && strcmp(tokens[1], "exit") == 0) {
            cmdTeachExit();
        } else if (ntok >= 2 && strcmp(tokens[1], "free") == 0) {
            /* 2026-09-08: free→lock→free 连续示教支持。
             * 若已在 free/lock 状态(teach_mode_active && !assist && !pwm),
             * 直接释放扭矩即可, 无需再走 prepare(会拒绝)。 */
            if (teach_mode_active && !teach_assist_active && !teach_pwm_active) {
                for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 0);
                cprint("  Teach FREE resumed: torques OFF. Drag on.\r\n");
            } else {
                cmdTeachEnterFree();
            }
        } else if (ntok >= 2 && strcmp(tokens[1], "lock") == 0) {
            /* 2026-09-08 分步示教: free 摆位后一键锁定当前位置。
             * goal=当前实测 -> Torque ON 满刚度保持; 录制不中断。
             * 'teach free' 解锁继续拖。无重力补偿标定, 稳。 */
            if (!teach_mode_active) {
                cprint("  Not in teach mode (use 'teach free' first).\r\n");
            } else if (teach_pwm_active || teach_assist_active) {
                cprint("  Only for FREE teach (torque OFF); current mode already holds.\r\n");
            } else {
                uint16_t raw[6];
                uint8_t bad = 0U;
                for (uint8_t j = 0; j < 6; j++) {
                    raw[j] = SERVO_GetPositionRaw(j);
                    if (raw[j] == 0xFFFFU) { bad = 1U; break; }
                }
                if (!bad) {
                    SERVO_SetPositionsRawSync(raw);   /* goal = 当前位 */
                    HAL_Delay(3);
                    for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1);
                    for (uint8_t j = 0; j < 6; j++) teach_assist_last_raw[j] = raw[j];
                    cprint("  LOCKED at current pose (full stiffness, hands free).\r\n");
                    cprint("  'teach free' = release & keep dragging; recording continues.\r\n");
                } else {
                    cprint("  Position read failed, lock aborted.\r\n");
                }
            }
        } else if (ntok >= 2 && strcmp(tokens[1], "s") == 0) {
            if (ntok >= 3) {
                long pct = atol(tokens[2]);
                if (pct < 1L) pct = 1L;
                if (pct > 300L) pct = 300L;
                assist_scale_pct = (uint16_t)pct;
            }
            cprint("  Float scale = %d%%  (set: teach s <1~300>; 小=更松)\r\n",
                   (int)assist_scale_pct);
        } else if (ntok >= 2 && strcmp(tokens[1], "k") == 0) {
            if (ntok >= 5) {
                assist_k2 = (float)atof(tokens[2]);
                assist_k3 = (float)atof(tokens[3]);
                assist_k4 = (float)atof(tokens[4]);
                if (assist_k2 < 0.0f) assist_k2 = 0.0f;
                if (assist_k3 < 0.0f) assist_k3 = 0.0f;
                if (assist_k4 < 0.0f) assist_k4 = 0.0f;
                if (assist_k2 > 8.0f) assist_k2 = 8.0f;
                if (assist_k3 > 8.0f) assist_k3 = 8.0f;
                if (assist_k4 > 8.0f) assist_k4 = 8.0f;
            }
            cprint("  Float gains (x1000): k2=%d k3=%d k4=%d  (set: teach k <k2> <k3> <k4>)\r\n",
                   (int)(assist_k2 * 1000.0f), (int)(assist_k3 * 1000.0f),
                   (int)(assist_k4 * 1000.0f));
        } else if (ntok >= 2 && strcmp(tokens[1], "p") == 0) {
            /* 2026-09-08 B方案: 实时调节位置环软跟随的 P/I/D。
             * 参数: teach p <P> [I] [D]; 小=软/易塌 大=硬/稳。
             * 推荐: P=0~4, I=0, D=0; I>0 会产生积分恢复力, 重新变硬。 */
            if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 254) v = 254;
                teach_soft_p = (uint8_t)v;
                if (ntok >= 4) {
                    v = atoi(tokens[3]);
                    if (v < 0) v = 0;
                    if (v > 254) v = 254;
                    teach_soft_i = (uint8_t)v;
                }
                if (ntok >= 5) {
                    v = atoi(tokens[4]);
                    if (v < 0) v = 0;
                    if (v > 254) v = 254;
                    teach_soft_d = (uint8_t)v;
                }
                if (teach_mode_active && teach_assist_active) {
                    for (uint8_t j = 0; j < 6; j++) {
                        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                          STS_ADDR_P_GAIN, &teach_soft_p, 1);
                        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                          STS_ADDR_I_GAIN, &teach_soft_i, 1);
                        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                          STS_ADDR_D_GAIN, &teach_soft_d, 1);
                    }
                }
            }
            cprint("  Teach soft PID = P=%d I=%d D=%d  (set: teach p <P> [I] [D])\r\n",
                   teach_soft_p, teach_soft_i, teach_soft_d);
        } else if (ntok >= 2 && strcmp(tokens[1], "n") == 0) {
            /* PUNCH 实时调节: P=0 还硬时的第二个嫌疑, 启动脉冲力矩。
             * teach 中立即生效到 6 颗舵机。 */
            if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 254) v = 254;
                teach_soft_punch = (uint8_t)v;
                if (teach_mode_active && teach_assist_active) {
                    for (uint8_t j = 0; j < 6; j++)
                        SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                          STS_ADDR_PUNCH, &teach_soft_punch, 1);
                }
            }
            cprint("  Teach soft PUNCH = %d  (set: teach n <0~254>; P=0 仍硬时压这个)\r\n",
                   teach_soft_punch);
        } else if (ntok >= 2 && strcmp(tokens[1], "i") == 0) {
            /* 自动重力配重积分增益: 大=配重建立快但可能缓慢震荡 */
            if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 20) v = 20;
                teach_int_gain = (uint8_t)v;
            }
            cprint("  Teach gravity-balance I = %d  (set: teach i <0~20>; 0=关)\r\n",
                   teach_int_gain);
        } else if (ntok >= 2 && strcmp(tokens[1], "o") == 0) {
            /* 查看自动配重当前偏移 (raw): 调试托举是否建立 */
            cprint("  Gravity offsets (raw): ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d ", j+1, teach_off_raw[j]);
            cprint("\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "pwm") == 0) {
            /* ===== PWM 模式真·重力补偿 (Mode2 开环力控, 混合模式) =====
             * 2026-09-09 实测定稿: PWM 模式 Torque ON 时即使 duty=0 驱动电路
             * 仍在管理电机 → 零负载轴(J1/J5/J6)也有电磁制动, 手感≠free。
             * 定稿: 只切 J2/J3/J4 到 Mode2 力控; J1/J5/J6 保持 Mode0+Torque OFF
             * (真 free, 与 teach free 手感一致)。teach exit 时恢复。
             * !! 首次测试请扶住臂: 切换期 J2/J3/J4 会失力下垂 !! */
            if (teach_mode_active) cmdTeachExit();
            if (!teach_mode_prepare()) return;
            cprint("  ** SUPPORT THE ARM! J2/J3/J4 switching to PWM mode,\r\n");
            cprint("     each loses power ~1.5s during RESET (total ~8s). **\r\n");
            cprint("  J1/J5/J6: torque OFF (true free, no PWM needed).\r\n");
            uint8_t fails = 0U;
            for (uint8_t j = 1U; j <= 3U; j++) {   /* 只有 J2/J3/J4 切 PWM */
                uint8_t rc = SERVO_SetOperatingMode(j, 2U);
                /* 失败自动重试 2 次: 偶发 ping 丢包不应让整个模式切换回退 */
                for (uint8_t rt = 0; rc != 1U && rt < 2U; rt++) {
                    cprint("  J%d retry %u...\r\n", j + 1, rt + 1);
                    HAL_Delay(150);
                    rc = SERVO_SetOperatingMode(j, 2U);
                }
                if (rc != 1U) {
                    const char* why = (rc == 2U) ? "ping失败" :
                                      (rc == 3U) ? "写入后验证不符(免RESET)" :
                                      (rc == 4U) ? "RESET后仍不符" :
                                      (rc == 5U) ? "验证读取失败" : "未知";
                    cprint("  J%d PWM switch FAILED (code %u: %s)\r\n", j + 1, rc, why);
                    fails++;
                }
            }
            if (fails != 0U) {
                cprint("  %u joint(s) failed - switching back to position mode.\r\n", fails);
                for (uint8_t j = 1U; j <= 3U; j++) (void)SERVO_SetOperatingMode(j, 0U);
                for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, 1);
                teach_mode_active = 0U;
                cprint("> ");
                return;
            }
            /* J1/J5/J6 直接 Torque OFF = 真 free */
            for (uint8_t j = 0; j < 6; j++) {
                if (j == 1U || j == 2U || j == 3U) continue;
                SERVO_SetTorque(j, 0);
            }
            teach_assist_active = 0U;
            teach_pwm_active = 1U;
            /* 默认参数: 只有 J2/J3/J4 有效 (J1/J5/J6 已 torque off, 参数无效)。 */
            const int def_gain[6] = {0, 6, 6, 6, 0, 0};
            const int def_damp[6] = {0, 3, 3, 3, 0, 0};
            const int def_cap[6]  = {100, 100, 40, 100, 100, 100};
            for (uint8_t j = 0; j < 6; j++) {
                teach_pwm_gain[j] = def_gain[j];
                teach_pwm_damp[j] = def_damp[j];
                teach_pwm_flip[j] = 1;
                teach_fc_duty[j] = 0;
                teach_dither[j] = 0;
                teach_pwm_off[j] = 0;
                teach_pwm_cap[j] = def_cap[j];
                uint16_t r = SERVO_GetPositionRaw(j);
                teach_assist_last_raw[j] = (r == 0xFFFFU) ? 2048U : r;
                if (j == 1U || j == 2U || j == 3U) {
                    uint8_t z[2] = {0, 0};
                    (void)SERVO_WriteSimple(JOINT_BUS[j], JOINT_ID[j],
                                            STS_ADDR_GOAL_TIME, z, 2);
                }
            }
            teach_test_joint = 0;
            teach_rec_cnt = 0U;
            teach_rec_full = 0U;
            cprint("  Teach PWM ON: J2/J3/J4 force-control, J1/J5/J6 free.\r\n");
            cprint("  Gravity model: mass-weighted CG (SO-101 URDF masses).\r\n");
            cprint("  Manual per-joint tuning (J2/J3/J4 only): g/v/f/c/d/o/cap J val.\r\n");
            cprint("  Sag: raise 'teach g J' or 'teach o J val'. Stiff: lower 'teach v J'.\r\n");
            cprint("  Oscillating: 'teach v J 8'. Pushing DOWN: 'teach f J'.\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "dump") == 0) {
            /* 一键输出当前全部 PWM 零力参数 (标定完成后汇报固化用) */
            cprint("  ===== Teach PWM params (for firmware default bake-in) =====\r\n");
            cprint("  gain (g):  J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_gain[0], teach_pwm_gain[1], teach_pwm_gain[2],
                   teach_pwm_gain[3], teach_pwm_gain[4], teach_pwm_gain[5]);
            cprint("  damp (v):  J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_damp[0], teach_pwm_damp[1], teach_pwm_damp[2],
                   teach_pwm_damp[3], teach_pwm_damp[4], teach_pwm_damp[5]);
            cprint("  flip (f):  J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_flip[0], teach_pwm_flip[1], teach_pwm_flip[2],
                   teach_pwm_flip[3], teach_pwm_flip[4], teach_pwm_flip[5]);
            cprint("  fric  (c): J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_fc_duty[0], teach_fc_duty[1], teach_fc_duty[2],
                   teach_fc_duty[3], teach_fc_duty[4], teach_fc_duty[5]);
            cprint("  dith  (d): J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_dither[0], teach_dither[1], teach_dither[2],
                   teach_dither[3], teach_dither[4], teach_dither[5]);
            cprint("  offs  (o): J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_off[0], teach_pwm_off[1], teach_pwm_off[2],
                   teach_pwm_off[3], teach_pwm_off[4], teach_pwm_off[5]);
            cprint("  cap:       J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_cap[0], teach_pwm_cap[1], teach_pwm_cap[2],
                   teach_pwm_cap[3], teach_pwm_cap[4], teach_pwm_cap[5]);
            cprint("  deadband (b) = %d\r\n", teach_pwm_db);
            cprint("  model (m) = %u\r\n", teach_pwm_model);
            cprint("  ============================================================\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "m") == 0) {
            /* 重力杠杆模型: 1=质量加权质心 (受力分析/雅可比正确形态, 默认)
             * 0=旧末端杠杆 (全部质量在杆末端假设, 保留对比) */
            if (ntok >= 3) {
                if (strcmp(tokens[2], "0") == 0) teach_pwm_model = 0U;
                else teach_pwm_model = 1U;
            }
            cprint("  PWM gravity model = %u (%s)\r\n", teach_pwm_model,
                   (teach_pwm_model != 0U) ? "mass-weighted CG (SO-101 URDF)"
                                           : "legacy end-of-link");
            cprint("  Set: teach m <0|1>\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "g") == 0) {
            /* teach g [J] <0~40>: J=1~6 单独设置; 省略 J=全局设置。 */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < 0) v = 0;
                    if (v > 100) v = 100;
                    teach_pwm_gain[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                for (uint8_t j = 0; j < 6; j++) teach_pwm_gain[j] = v;
            }
            cprint("  PWM gravity gain (x0.1 duty/mm): J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_gain[0], teach_pwm_gain[1], teach_pwm_gain[2],
                   teach_pwm_gain[3], teach_pwm_gain[4], teach_pwm_gain[5]);
            cprint("  Set: teach g <0~100> (all) or teach g J <0~100> (J=1~6)\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "v") == 0) {
            /* teach v [J] <0~20>: J=1~6 单独设置; 省略 J=全局设置。 */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < 0) v = 0;
                    if (v > 20) v = 20;
                    teach_pwm_damp[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 20) v = 20;
                for (uint8_t j = 0; j < 6; j++) teach_pwm_damp[j] = v;
            }
            cprint("  PWM velocity damping: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_damp[0], teach_pwm_damp[1], teach_pwm_damp[2],
                   teach_pwm_damp[3], teach_pwm_damp[4], teach_pwm_damp[5]);
            cprint("  Set: teach v <0~20> (all) or teach v J <0~20> (J=1~6)\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "f") == 0) {
            /* teach f [J]: 翻转重力方向; 省略 J=全部翻转。 */
            if (ntok >= 3) {
                int j = atoi(tokens[2]) - 1;
                if (j >= 0 && j < 6) {
                    teach_pwm_flip[j] = -teach_pwm_flip[j];
                    cprint("  J%d PWM gravity flip = %d\r\n", j + 1, teach_pwm_flip[j]);
                }
            } else {
                for (uint8_t j = 0; j < 6; j++) teach_pwm_flip[j] = -teach_pwm_flip[j];
                cprint("  ALL PWM gravity flip: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                       teach_pwm_flip[0], teach_pwm_flip[1], teach_pwm_flip[2],
                       teach_pwm_flip[3], teach_pwm_flip[4], teach_pwm_flip[5]);
            }
        } else if (ntok >= 2 && strcmp(tokens[1], "c") == 0) {
            /* 库伦摩擦补偿: teach c [J] <0~300> */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < 0) v = 0;
                    if (v > 300) v = 300;
                    teach_fc_duty[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 300) v = 300;
                for (uint8_t j = 0; j < 6; j++) teach_fc_duty[j] = v;
            }
            cprint("  PWM Coulomb friction comp: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_fc_duty[0], teach_fc_duty[1], teach_fc_duty[2],
                   teach_fc_duty[3], teach_fc_duty[4], teach_fc_duty[5]);
            cprint("  Set: teach c <0~300> (all) or teach c J <0~300> (J=1~6)\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "d") == 0) {
            /* 交变预紧 dither: teach d [J] <0~100> */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < 0) v = 0;
                    if (v > 100) v = 100;
                    teach_dither[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                for (uint8_t j = 0; j < 6; j++) teach_dither[j] = v;
            }
            cprint("  PWM stiction dither: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_dither[0], teach_dither[1], teach_dither[2],
                   teach_dither[3], teach_dither[4], teach_dither[5]);
            cprint("  Set: teach d <0~100> (all) or teach d J <0~100> (J=1~6)\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "o") == 0) {
            /* 恒定偏置 duty (2026-09-09): J5/J6 前馈杠杆恒 0 (绕 TCP 轴无
             * 重力杠杆), 但夹爪/偏心仍产生小力矩 → 用常数偏置托。
             * teach o [J] <-200~200>; 方向反了就取负值。 */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < -200) v = -200;
                    if (v > 200) v = 200;
                    teach_pwm_off[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < -200) v = -200;
                if (v > 200) v = 200;
                for (uint8_t j = 0; j < 6; j++) teach_pwm_off[j] = v;
            }
            cprint("  PWM const offset: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_off[0], teach_pwm_off[1], teach_pwm_off[2],
                   teach_pwm_off[3], teach_pwm_off[4], teach_pwm_off[5]);
            cprint("  Set: teach o J <-200~200> (J5/J6 sag: 从 ±20 往上加; 推反取负)\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "cap") == 0) {
            /* Per-joint duty 限幅 (2026-09-09 可调): 默认 J3=40 其他=100。
             * J3 下坠时放宽 (teach cap 3 80), 共振时收紧。 */
            if (ntok >= 4) {
                int j = atoi(tokens[2]) - 1;
                int v = atoi(tokens[3]);
                if (j >= 0 && j < 6) {
                    if (v < 10) v = 10;
                    if (v > 1000) v = 1000;
                    teach_pwm_cap[j] = v;
                }
            } else if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 10) v = 10;
                if (v > 1000) v = 1000;
                for (uint8_t j = 0; j < 6; j++) teach_pwm_cap[j] = v;
            }
            cprint("  PWM duty cap: J1=%d J2=%d J3=%d J4=%d J5=%d J6=%d\r\n",
                   teach_pwm_cap[0], teach_pwm_cap[1], teach_pwm_cap[2],
                   teach_pwm_cap[3], teach_pwm_cap[4], teach_pwm_cap[5]);
            cprint("  Set: teach cap J <10~1000>; J3 托不住=放宽, 共振=收紧\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "test") == 0) {
            /* 单轴标定模式 (2026-09-09): teach test J → 仅 J 轴有 PWM 输出,
             * 其他轴等效 free; teach test 0 恢复全轴。排除参数互相干扰。 */
            if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 6) v = 6;
                teach_test_joint = v;
            }
            if (teach_test_joint == 0)
                cprint("  PWM test mode OFF - all joints output.\r\n");
            else
                cprint("  PWM test mode: ONLY J%d outputs (others = free). Drag J%d only.\r\n",
                       teach_test_joint, teach_test_joint);
            cprint("  Set: teach test <0|1~6>\r\n");
        } else if (ntok >= 2 && strcmp(tokens[1], "b") == 0) {
            /* 死区避开值: 驱动芯片在 |duty|<db 时死区自激 → 甩动。
             * 加大 db 让 duty 离死区更远; 减小时阻力更小但可能再振荡。 */
            if (ntok >= 3) {
                int v = atoi(tokens[2]);
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                teach_pwm_db = v;
            }
            cprint("  PWM deadband avoid = %d duty  (set: teach b <0~100>; 甩动=加大)\r\n",
                   teach_pwm_db);
        } else if (ntok >= 2 && strcmp(tokens[1], "rec") == 0) {
            /* 跟随录制: 只读, 不碰舵机 —— 供"PC 中转遥操时把从臂轨迹录下来"用 */
            cmdTeachEnterRec();
        } else {
            cmdTeachEnter();
        }
        return;
    }
    if (strcmp(tokens[0], "rec") == 0) {
        if (ntok >= 2 && strcmp(tokens[1], "clear") == 0) cmdTeachClear();
        else if (ntok >= 2 && strcmp(tokens[1], "list") == 0) cmdTeachList();
        else cprint("  Auto-record: type 'teach' & drag. 'save <name>' keeps it; 'rec clear' discards.\r\n");
        return;
    }
    if (strcmp(tokens[0], "save") == 0) {
        cmdTeachSave((ntok >= 2) ? tokens[1] : NULL);
        return;
    }
    if (strcmp(tokens[0], "list") == 0 || strcmp(tokens[0], "files") == 0) {
        cmdStoreList();
        return;
    }
    if (strcmp(tokens[0], "del") == 0 || strcmp(tokens[0], "delete") == 0) {
        cmdTeachDelete((ntok >= 2) ? tokens[1] : NULL);
        return;
    }
    if (strcmp(tokens[0], "compact") == 0) {
        cmdTeachCompact();
        return;
    }
    if (strcmp(tokens[0], "play") == 0) {
        uint8_t loop = 0U;
        const char *nm = NULL;
        for (int i2 = 1; i2 < ntok; i2++) {
            if (strcmp(tokens[i2], "loop") == 0) loop = 1U;
            else nm = tokens[i2];   /* 名字/序号; 只支持一个目标 */
        }
        if (nm) cmdTeachPlayFile(nm, loop);
        else cmdTeachPlay(loop);
        return;
    }

    char cmd = tokens[0][0];

    /* ⚠ 地雷闸门 (2026-09-18, 红线 26): 长度 > 1 又不认识 ⇒ 绝不进首字母 switch。
     * 上面所有精确匹配的命令都已经 return 了, 走到这里的多字符 token 只可能是
     * 「打错的命令」或「白名单漏登记的变体」。前者会按首字母触发松力矩/回零点/IK
     * 运动这类**真动作**, 后者会静默失败 —— 两种情况都远不如明说"未知命令"。 */
    if (tokens[0][0] != '#' && tokens[0][1] != '\0' &&
        !cmd_is_known_multichar(tokens[0])) {
        cprint("  Unknown command '%s'.  'h' = help;\r\n", tokens[0]);
        cprint("  单独敲 'c'(只有这一个字符) 才是校准向导 —— 别把别的命令敲成 c 开头。\r\n");
        return;
    }

    switch (cmd) {
    case 'e': {
        /* CAN Loopback self-test */
        cprint("\r\n  ===== CAN Loopback Test =====\r\n");
        cprint("  Testing %s (%s) internal loopback...\r\n",
               ArmCanDrv_IfName(), ArmCanDrv_PinDesc());
        uint8_t r = ArmCanDrv_LoopbackTest();
        if (r == 1) {
            cprint("  RESULT: PASS - CAN controller OK\r\n");
            cprint("  -> CAN controller works. Problem is in transceiver/wiring.\r\n");
        } else if (r == 0xFF) {
            cprint("  RESULT: TX FAIL - cannot send frame\r\n");
        } else if (r == 0xFE) {
            cprint("  RESULT: INIT FAIL - CAN re-init failed\r\n");
        } else {
            cprint("  RESULT: RX TIMEOUT - no loopback frame received\r\n");
            cprint("  -> CAN controller may not be initialized properly.\r\n");
        }
        cprint("\r\n");
        break;
    }

    case 'h':
        cmdHelp();
        break;

    case 'n':
        cmdScan();
        break;

    case 'c':
        /* 0924AU: colj2 —— J2 壳体包线开关。必须放在 'c' 单字符校准向导守卫
         * 之前, 否则会被下面的"非单字符 c 一律未知命令"吞掉。 */
        if (tokens[0][1] == 'o' && tokens[0][2] == 'l' &&
            tokens[0][3] == 'j' && tokens[0][4] == '2' && tokens[0][5] == '\0') {
            if (ntok >= 2 && strcmp(tokens[1], "off") == 0) {
                colj2_en = 0U;
                cprint("  colj2 OFF (J2 壳体包线检查停用, 仅本次上电)\r\n");
            } else if (ntok >= 2 && strcmp(tokens[1], "on") == 0) {
                colj2_en = 1U;
                cprint("  colj2 ON\r\n");
            } else if (ntok == 1) {
                cprint("  colj2 %s  包线: 0<=rho<%.1f 且 z<%.1f mm (基座坐标)\r\n",
                       colj2_en ? "ON" : "OFF",
                       (double)COLJ2_RHO_MAX, (double)COLJ2_Z_TOP);
                cprint("  依据: J2 前壳水平 51.05 / 顶壳离桌 131.38 (现场实测) + 余量\r\n");
                cprint("  Usage: colj2 on|off   (RAM 开关, 不存 flash)\r\n");
            } else {
                cprint("  Usage: colj2 on|off\r\n");
            }
            break;
        }
        /* Interactive joint-limit calibration. Blocks until user presses 'q'.
         *
         * ⚠⚠ 2026-09-18 实测踩到过的地雷: 这个 switch 是**首字母兜底**分支 ——
         * 任何"精确匹配全部失败"的 token 都会按 tokens[0][0] 落到这里。
         * 于是旧固件上敲 `cw` / `cs` / `cx` 这种**不存在的命令**, 不会报"未知命令",
         * 而是**静默进入阻塞式校准向导**: 它先把全部力矩松掉(整臂会掉下来),
         * 并且在该模式下按 `s` 会**固化零点**(见红线)。同一晚踩了两次。
         * ⇒ 现在只在**恰好就是单个 'c'** 时才进校准; 其余一律按"未知命令"处理。
         *    (can/cn/cp/ct/cy/cz/cw/compact 都在上面的精确匹配里, 不受影响) */
        if (tokens[0][1] != '\0') {
            cprint("  Unknown command '%s'. Type 'h' for help.\r\n", tokens[0]);
            cprint("  (没进校准向导 —— 要校准请**只敲 c**; 校准中按 s 会固化零点)\r\n");
            break;
        }
        cmdCalibrate();
        break;

    case 'M':
        /* One-key center calibration: set current pose as servo middle (2048).
         * M <0-5> single joint; M a all (use in official L-form Home pose). */
        if (ntok >= 2 && tokens[1][0] == 'a') cmdCenterCalib(1, 0);
        else if (ntok >= 2) cmdCenterCalib(0, (uint8_t)atoi(tokens[1]));
        else cprint("  Usage: M <0-5> (single) or M a (all 6, in L-form Home)\r\n");
        break;

    case 'Z':
        cmdFullScan();
        break;

    case 'b':
        /* 0924AT: bootlog —— 开机日志降噪开关 (默认安静, RAM-only 不存 flash)。
         * 安静模式只打重要的: 横幅/[FAULT]/[STALL]/[BOOT]/扫描结果/警告/
         * 链路状态/偏好恢复; `bootlog on` 恢复全套 Init 过程行 (排查启动问题时用)。 */
        if (strcmp(tokens[0], "bootlog") == 0) {
            if (ntok >= 2 && (tokens[1][0] == 'o')) {
                if (tokens[1][1] == 'n')      boot_verbose = 1U;
                else if (tokens[1][1] == 'f') boot_verbose = 0U;
                else { cprint("  Usage: bootlog on|off\r\n"); break; }
            } else if (ntok >= 2) {
                cprint("  Usage: bootlog on|off\r\n");
                break;
            }
            cprint("  Boot log: %s (下次开机默认安静)\r\n",
                   boot_verbose ? "VERBOSE (全套过程行)" : "QUIET (只打重要信息)");
            break;
        }
        /* Bus mode: b          查看当前模式 + 六轴实际路由
         *          b 0        正常: J1-J3=BusA, J4-J6=BusB (重置为默认布局)
         *          b 1        全部走 Bus A
         *          b 2        全部走 Bus B (测试 Bus B 硬件)
         *          b auto     重新逐 ID 探测, 按实测应答重建路由(换过舵机/插错座子用) */
#if defined(BUS_ONE) && BUS_ONE
        /* 集成板(1Y+3Y 短接)守卫: 只允许 mode 2。
         * 切到 0/1 会让某次事务对 BUS_A 动作 -> DIR_A 拉高 -> 门1 使能,
         * 而门1 输出与门3 输出已经短接在同一条线上 = 两个推挽对打,
         * 轻则通信全乱, 重则烧 126 或舵机输入级。宁可拒绝, 不给机会。 */
        if (ntok >= 2 && atoi(tokens[1]) != 2) {
            cprint("  [DENY] BUS_ONE build: 1Y+3Y shorted, bus mode locked to 2.\r\n");
            cprint("         mode 0/1 would enable gate1 while gate3 drives the same\r\n");
            cprint("         wire (push-pull collision). Rebuild without BUS_ONE=1\r\n");
            cprint("         and un-short 1Y/3Y if you need the dual-bus layout.\r\n");
            break;
        }
#endif
        if (ntok >= 2 && (tokens[1][0] == 'a' || tokens[1][0] == 'A')) {
            /* 逐 ID 重探测: 换过舵机、改过 ID、插错座子之后不用重启 */
            uint8_t miss = 0U, dup = 0U, moved = 0U;
            uint8_t online = SERVO_AutoMapBuses(&miss, &dup, &moved);
            cprint("  [MAP] re-probe: online %u/6\r\n", (unsigned)online);
            for (uint8_t j = 0U; j < 6U; j++) {
                cprint("    J%d (ID %u) -> Bus %s%s%s\r\n", (int)(j + 1), (unsigned)JOINT_ID[j],
                       (JOINT_BUS[j] == BUS_A) ? "A" : "B",
                       (miss & (uint8_t)(1U << j)) ? "  MISSING" : "",
                       (dup  & (uint8_t)(1U << j)) ? "  DUP-ID" : "");
            }
            if (moved) {
                /* moved 是位掩码, 不能直接 %u —— 否则 J1/J2/J3 三位相加
                 * 会打成 "7 axis re-routed"(0924BA 的显示 bug)。 */
                uint8_t nmoved = 0U;
                for (uint8_t j = 0U; j < 6U; j++)
                    if (moved & (uint8_t)(1U << j)) nmoved++;
                cprint("  %u axis re-routed.\r\n", (unsigned)nmoved);
                cprint("  Route saved to flash (survives reset).\r\n");
            }
            cprint("  (run 'n' to rescan, then 'i' to init)\r\n");
            break;
        }
        if (ntok >= 2) {
#if ARM_MASTER_BUILD
            uint8_t m = SERVO_SetBusMode(1);   /* 舵机总线钉死 BUS_A(USART3), UART5 旁路 */
#else
            uint8_t m = SERVO_SetBusMode((uint8_t)atoi(tokens[1]));
#endif
            cprint("  Bus mode set to ");
            cprint(m == 2 ? "2 (ALL -> Bus B)" : (m == 1 ? "1 (ALL -> Bus A)" : "0 (NORMAL)"));
#if defined(BUS_ONE) && BUS_ONE
            cprint("\r\n  (Rescan with 'n', then 'i' to init)\r\n");
#else
            /* 0924BC: 显式钉死的路由必须跨复位保留 —— 否则重启又回编译期
             * 默认 A/A/A/B/B/B, 在"六颗全接 B"的板上等于白敲。 */
            if (SERVO_SaveRouteAll())
                cprint("\r\n  Route saved to flash (survives reset).\r\n"
                       "  (Rescan with 'n', then 'i' to init)\r\n");
            else
                cprint("\r\n  [WARN] route save FAILED (RAM only, lost on reset).\r\n"
                       "  (Rescan with 'n', then 'i' to init)\r\n");
#endif
        } else {
            uint8_t m = SERVO_GetBusMode();
            cprint("  Bus mode = ");
            cprint(m == 2 ? "2 (ALL -> Bus B)" : (m == 1 ? "1 (ALL -> Bus A)" : "0 (NORMAL)"));
            cprint("\r\n  Route  :");
            for (uint8_t j = 0U; j < 6U; j++)
                cprint(" J%d=%s", (int)(j + 1), (JOINT_BUS[j] == BUS_A) ? "A" : "B");
            cprint("\r\n  Usage: b <0|1|2|auto>   0=normal 1=all BusA 2=all BusB auto=probe\r\n");
        }
        break;

    case 'i':
        if (ntok >= 2) cmdInitOne((uint8_t)atoi(tokens[1]));
        else           cmdInit();
        break;

    case 's':
        cmdStatus();
        break;

    case 't':
        /* ⭐0924CA: t <J> <0|1>, J=0~5 (0-based)。新增 6=全部六轴 ——
         * 之前用户敲 `t J 0` (字母 J 被 atoi 成 0) 只松了 J1, 误当全松。 */
        if (ntok < 3) { cprint("  Usage: t <J 0~5|6=全部> <0|1>   (0=松 1=紧)\r\n"); break; }
        if ((uint8_t)atoi(tokens[1]) == 6U) {
            uint8_t en = (uint8_t)atoi(tokens[2]);
            for (uint8_t j = 0; j < 6U; j++) cmdTorque(j, en);
            break;
        }
        cmdTorque((uint8_t)atoi(tokens[1]), (uint8_t)atoi(tokens[2]));
        break;

    case 'a':
        if (ntok < 3) { cprint("  Usage: a <J> <deg>\r\n"); break; }
        cmdAngle((uint8_t)atoi(tokens[1]), (float)atof(tokens[2]));
        break;

    case 'r':
        /* Raw position write: r <J> <0-4095> */
        if (ntok < 3) {
            cprint("  Usage: r <J> <0-4095>   (raw position)\r\n");
            break;
        } {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            uint16_t raw = (uint16_t)atoi(tokens[2]);
            if (j >= 6) { cprint("  Joint must be 0-5\r\n"); break; }
            /* Clamp raw to safe limits (wrap-around aware) */
            uint16_t raw_min = JOINT_MIN_RAW[j];
            uint16_t raw_max = JOINT_MAX_RAW[j];
            if (raw_min == 0 && raw_max == 0) { raw_min = 0; raw_max = 4095; }
            if (!raw_in_range(raw, raw_min, raw_max)) {
                uint16_t clamped = clamp_raw(raw, raw_min, raw_max);
                cprint("  Clamped: %d -> %d (raw limit)\r\n", raw, clamped);
                raw = clamped;
            }
            uint8_t bus = JOINT_BUS[j];
            uint8_t id  = JOINT_ID[j];
            uint8_t data[2] = { raw & 0xFF, (raw >> 8) & 0xFF };
            HAL_StatusTypeDef st = DXL_Write(bus, id, STS_ADDR_GOAL_POSITION, data, 2, 100);
            cprint("  J%d -> raw=%d: %s\r\n", j+1, raw, st == HAL_OK ? "OK" : "FAIL");
        }
        break;

    case 'f':
        /* Forward Kinematics: show current end position */
        cmdFK();
        break;

    case 'g':
        /* Inverse Kinematics: g x y z [pitch] */
        if (ntok < 4) {
            cprint("  Usage: g <x> <y> <z> [pitch_deg]\r\n");
            cprint("  x,y,z in mm. pitch optional (default=0=up)\r\n");
            break;
        }
        {
            float gx = (float)atof(tokens[1]);
            float gy = (float)atof(tokens[2]);
            float gz = (float)atof(tokens[3]);
            float gpitch = (ntok >= 5) ? (float)atof(tokens[4]) : 0.0f;
            cmdGoto(gx, gy, gz, gpitch);
        }
        break;

    case 'p':
        /* pv <interval_ms>: periodic all-joint feedback; pv 0 stops.
         * pf <interval_ms>: periodic actual end-effector FK pose; pf 0 stops. */
        if (tokens[0][1] == 'v' || tokens[0][1] == 'V' ||
            tokens[0][1] == 'f' || tokens[0][1] == 'F') {
            uint8_t pose_mode = (tokens[0][1] == 'f' || tokens[0][1] == 'F');
            if (ntok < 2) {
                cprint("  Usage: p%c <interval_ms> (10~60000, 0=stop)\r\n",
                       pose_mode ? 'f' : 'v');
                break;
            }
            uint32_t period = (uint32_t)atol(tokens[1]);
            if (period == 0U) {
                if (pose_mode) {
                    pose_feedback_period_ms = 0U;
                    cprint("  Real-time XYZ pose feedback stopped.\r\n");
                } else {
                    feedback_period_ms = 0U;
                    cprint("  Real-time angle feedback stopped.\r\n");
                }
            } else {
                if (period < 10U) period = 10U;
                if (period > 60000U) period = 60000U;
                if (pose_mode) {
                    pose_feedback_period_ms = period;
                    pose_feedback_next_tick = HAL_GetTick();
                    cprint("  Real-time XYZ pose feedback: every %lu ms. Use pf 0 to stop.\r\n",
                           (unsigned long)pose_feedback_period_ms);
                } else {
                    feedback_period_ms = period;
                    feedback_next_tick = HAL_GetTick();
                    cprint("  Real-time angle feedback: every %lu ms. Use pv 0 to stop.\r\n",
                           (unsigned long)feedback_period_ms);
                }
            }
            break;
        }
        if (ntok < 2) { cprint("  Usage: p <J>\r\n"); break; } {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            float deg = SERVO_GetPosition(j);
            uint16_t raw = SERVO_GetPositionRaw(j);
            cprint("  J%d: raw=%d servo=%.2f deg  rel-home=%+.2f deg\r\n",
                   j+1, raw, deg, servo_to_joint_deg(deg, j));
        }
        break;

    case 'z':
        /* ⭐0924AJ→BX: zf 命令族 —— here(实测标定, 唯一设值口)/on|off(开关)/
         * clear(撤销)。数字版 <mm> 已移除 (0924BX, 用户明令): FK 模型系统性
         * 偏差 ~10mm, 数字值与实际离桌距离对不上。floor 与 C 偏移自洽:
         * floor = geo_zfloor_mm − WORLD_OFFSET_Z, check 两边同减抵消。 */
        if (tokens[0][1] == 'f') {
            /* ⭐0924BV: zf on|off —— 流控/遥操桌面限位开关 (区F 持久化)。
             * 0924BS 直通把桌面检查一并旁路 ⇒ zf 形同虚设; 拆出后
             * 直通只旁路跳变/立柱/包线, 桌面下限由本开关独立控制。 */
            if (ntok >= 2 && strcmp(tokens[1], "on") == 0) {
                geo_zf_on = 1U;
                uint8_t zfv = (geo_zfloor_mm >= 0.0f) ? 1U : 0U;
                if (GeoCfg_Save2(WORLD_OFFSET_X, WORLD_OFFSET_Y, WORLD_OFFSET_Z,
                                 zfv ? geo_zfloor_mm : 0.0f, zfv,
                                 geo_zf_on, geo_margin_raw) == CFG_FLASH_OK)
                    cprint("  桌面限位 ON (遥操/流控拦爪尖低于 %.1f mm), 已写 flash\r\n",
                           (double)((geo_zfloor_mm >= 0.0f) ? geo_zfloor_mm
                                    : (traj_z_floor_mm + WORLD_OFFSET_Z)));
                else
                    cprint("  桌面限位 ON (⚠ flash 写失败, 重启丢)\r\n");
                break;
            }
            if (ntok >= 2 && strcmp(tokens[1], "off") == 0) {
                geo_zf_on = 0U;
                uint8_t zfv = (geo_zfloor_mm >= 0.0f) ? 1U : 0U;
                if (GeoCfg_Save2(WORLD_OFFSET_X, WORLD_OFFSET_Y, WORLD_OFFSET_Z,
                                 zfv ? geo_zfloor_mm : 0.0f, zfv,
                                 geo_zf_on, geo_margin_raw) == CFG_FLASH_OK)
                    cprint("  桌面限位 OFF (直通模式完全无桌面拦截, 注意安全), 已写 flash\r\n");
                else
                    cprint("  桌面限位 OFF (⚠ flash 写失败, 重启丢)\r\n");
                break;
            }
            if (ntok >= 2 && tokens[1][0] == 'h' && tokens[1][1] == 'e') {
                /* 0924BW: zf here —— 实测标定。FK 模型在该臂形有 ~10mm 系统性
                 * 偏差 (用户实测: 设 5mm 实际停 15mm), 数字 zf 治不了 (负值被
                 * 钳 0)。here 把爪尖摆到想要的最小高度后直接以**当前实际位姿**
                 * 的 FK z 为下限 —— 偏差来源(零点/几何/TCP)不用管, 整体吸收。
                 * 存区F: geo_zfloor_mm 存世界系模型 z, 与上电恢复链
                 * (traj = geo_zfloor_mm - WORLD_OFFSET_Z) 自洽; 顺手打开限位。 */
                float q[6]; uint8_t ok = 1U;
                for (uint8_t j = 0; j < 6U; j++) {
                    q[j] = SERVO_GetPosition(j);
                    if (q[j] < 0.0f) { ok = 0U; break; }
                }
                if (!ok) {
                    cprint("  ⚠ 读从臂实际位置失败, 重试 (或先 `pv` 确认 6 轴可读)\r\n");
                } else {
                    Kinematics_Pos p; float rad[6];
                    for (uint8_t j = 0; j < 6U; j++) rad[j] = servo_deg_to_rad(q[j], j);
                    if (!FK_Solve(rad, &p)) {
                        cprint("  ⚠ FK 求解失败 (当前位姿异常?)\r\n");
                    } else {
                        traj_z_floor_mm = p.z - WORLD_OFFSET_Z;
                        geo_zfloor_mm   = p.z;   /* 世界系模型 z, 恢复链自洽 */
                        geo_zf_on = 1U;          /* 标定即要用 */
                        if (GeoCfg_Save2(WORLD_OFFSET_X, WORLD_OFFSET_Y, WORLD_OFFSET_Z,
                                         geo_zfloor_mm, 1U, geo_zf_on,
                                         geo_margin_raw) == CFG_FLASH_OK) {
                            cprint("  桌面下限 = 当前爪尖模型 z %.1f mm (floor=%.1f), 已存 flash\r\n",
                                   (double)geo_zfloor_mm, (double)traj_z_floor_mm);
                            cprint("  遥操限位 ON: 以后压到这个高度就停 (模型偏差已吸收)\r\n");
                        } else {
                            cprint("  ⚠ flash 写入失败: 仅本次上电有效\r\n");
                        }
                    }
                }
            } else if (ntok >= 2 && tokens[1][0] == 'c' &&
                (tokens[1][1] == 'l' || tokens[1][1] == 0)) {
                /* 0924AS: zf clear —— 清除偏好, 回到折叠位推导锚定 */
                geo_zfloor_mm = -1.0f;
                update_home_dock();
                (void)GeoCfg_Save2(WORLD_OFFSET_X, WORLD_OFFSET_Y, WORLD_OFFSET_Z,
                                   0.0f, 0U, geo_zf_on, geo_margin_raw);
                cprint("  zf 偏好已清除 (回折叠位推导), 已写 flash\r\n");
            } else if (ntok >= 2 && ((tokens[1][0] >= '0' && tokens[1][0] <= '9') ||
                                     tokens[1][0] == '-' || tokens[1][0] == '+')) {
                /* 0924BX: 数字版 zf <mm> 已移除 (用户明令)。FK 模型系统性偏差
                 * ~10mm, 数字值与实际离桌距离对不上; 标定一律 zf here。 */
                cprint("  ⚠ zf <数字> 已移除 —— FK 模型偏差 ~10mm, 数字值对不上实际离桌距离\r\n");
                cprint("     标定用 zf here: 爪尖摆到目标高度后敲 (存 flash, 重启保留)\r\n");
            } else {
                cprint("  当前桌面下限 floor=%.1f mm ≈ 离桌 %.1f mm (C偏移Z=%d)%s\r\n",
                       (double)traj_z_floor_mm,
                       (double)(traj_z_floor_mm + WORLD_OFFSET_Z),
                       (int)WORLD_OFFSET_Z,
                       (geo_zfloor_mm >= 0.0f) ? "  [zf pref 已存 flash]" : "");
                cprint("  遥操/流控桌面限位: %s%s\r\n",
                       geo_zf_on ? "ON" : "OFF",
                       geo_zf_on ? "" : "  (zf on 打开)");
                cprint("  Usage: zf here   实测标定: 爪尖摆到目标高度后敲 (推荐)\r\n");
                cprint("         zf on|off  遥操桌面限位开关 (存 flash)\r\n");
                cprint("         zf clear   清除标定 (回折叠位推导)\r\n");
            }
            break;
        }
        /* 2026-09-17: 参数显式校验。旧写法 `cmdGoHome(atoi(tokens[1]))` 对任何
         * 非数字参数 atoi 都得 0 ⇒ `z s` / `z x` 之类会**闷声把 J1 归位**。 */
        if (ntok >= 2 && tokens[1][0] == 'a') {
            cmdGoHomeAll();
        } else if (ntok >= 2 && tokens[1][0] >= '0' && tokens[1][0] <= '5' &&
                   (tokens[1][1] < '0' || tokens[1][1] > '9')) {
            cmdGoHome((uint8_t)(tokens[1][0] - '0'));
        } else {
            cprint("  Usage: z <J 0~5> | z a   (go to 0-point = calib zero)\r\n");
        }
        break;

    case 'w':
        if (ntok < 2) { cprint("  Usage: w <J>\r\n"); break; }
        cmdSweep((uint8_t)atoi(tokens[1]));
        break;

    case 'm': {
        /* m a1 a2 a3 a4 a5 a6 [v_max] */
        if (ntok < 7) {
            cprint("  Usage: m <a1> <a2> <a3> <a4> <a5> <a6> [speed]\r\n");
            cprint("  Angles in degrees (0~300), speed in deg/s (default=30)\r\n");
            break;
        }
        float target[6];
        for (int i = 0; i < 6; i++) {
            target[i] = (float)atof(tokens[1 + i]);
            if (target[i] < 0) target[i] = 0;
            if (target[i] > 300) target[i] = 300;
        }
        float v_max = (ntok >= 8) ? (float)atof(tokens[7]) : ARM_SC_DEF_VMAX;
        if (v_max < ARM_SC_V_MIN) v_max = ARM_SC_V_MIN;
        if (v_max > ARM_SC_V_MAX) v_max = ARM_SC_V_MAX;
        cmdMove(target, v_max, v_max * ARM_SC_ACCEL_FACTOR);
        break;
    }

    case 'q': {
        /* qv/qd are accepted in lowercase as well as uppercase. */
        if (tokens[0][1] == 'v' || tokens[0][1] == 'V') {
            if (ntok < 3) {
                cprint("  Usage: qv <segment 1~8> <speed deg/s>\r\n");
                break;
            }
            int idx = atoi(tokens[1]);
            float speed = (float)atof(tokens[2]);
            if (idx < 1 || idx > MULTI_POINT_MAX) {
                cprint("  Segment index must be 1~%d\r\n", MULTI_POINT_MAX);
                break;
            }
            if (speed < 5.0f) speed = 5.0f;
            if (speed > 300.0f) speed = 300.0f;
            multi_point_segment_speed[idx - 1] = speed;
            cprint("  Segment %d speed = %d deg/s\r\n", idx, (int)speed);
            break;
        }
        if (tokens[0][1] == 'd' || tokens[0][1] == 'D') {
            if (ntok < 3) {
                cprint("  Usage: qd <point 1~8> <dwell ms>\r\n");
                break;
            }
            int idx = atoi(tokens[1]);
            long dwell = atol(tokens[2]);
            if (idx < 1 || idx > MULTI_POINT_MAX) {
                cprint("  Point index must be 1~%d\r\n", MULTI_POINT_MAX);
                break;
            }
            if (dwell < 0) dwell = 0;
            if (dwell > 60000) dwell = 60000;
            multi_point_dwell_ms[idx - 1] = (uint32_t)dwell;
            cprint("  Point %d dwell = %d ms\r\n", idx, (int)dwell);
            break;
        }
        /* qg index x y z [pitch]: solve/validate Cartesian point now, then
         * store its six joint targets in the normal multi-point queue. */
        if (tokens[0][1] == 'g' || tokens[0][1] == 'G') {
            if (ntok < 5) {
                cprint("  Usage: qg <1~8> <x_mm> <y_mm> <z_mm> [pitch_deg]\r\n");
                break;
            }
            int idx = atoi(tokens[1]);
            if (idx < 1 || idx > MULTI_POINT_MAX) {
                cprint("  Point index must be 1~%d\r\n", MULTI_POINT_MAX);
                break;
            }
            float pitch = (ntok >= 6) ? (float)atof(tokens[5]) : 0.0f;
            if (!cmdSolveIKPoint((float)atof(tokens[2]), (float)atof(tokens[3]),
                                 (float)atof(tokens[4]), pitch,
                                 multi_points[idx - 1])) {
                cprint("  qg: IK failed, limits exceeded, or FK error > 5 mm.\r\n");
                break;
            }
            if ((uint8_t)idx > multi_point_count) multi_point_count = (uint8_t)idx;
            cprint("  XYZ point %d saved: X=%d Y=%d Z=%d mm, Pitch=%d deg\r\n",
                   idx, (int)atof(tokens[2]), (int)atof(tokens[3]),
                   (int)atof(tokens[4]), (int)pitch);
            cprint("  IK: J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f deg\r\n", 
                   multi_points[idx - 1][0], multi_points[idx - 1][1],
                   multi_points[idx - 1][2], multi_points[idx - 1][3],
                   multi_points[idx - 1][4], multi_points[idx - 1][5]);
            break;
        }
        /* q index j1..j6: save one joint-space point, index=1..8 */
        if (ntok < 8) {
            cprint("  Usage: q <1~8> <j1> <j2> <j3> <j4> <j5> <j6>\r\n");
            break;
        }
        int idx = atoi(tokens[1]);
        if (idx < 1 || idx > MULTI_POINT_MAX) {
            cprint("  Point index must be 1~%d\r\n", MULTI_POINT_MAX);
            break;
        }
        for (uint8_t j = 0; j < 6; j++)
            multi_points[idx - 1][j] = clamp_angle(j, (float)atof(tokens[2 + j]));
        if ((uint8_t)idx > multi_point_count) multi_point_count = (uint8_t)idx;
        cprint("  Point %d saved: %d %d %d %d %d %d\r\n", idx,
               (int)multi_points[idx - 1][0], (int)multi_points[idx - 1][1],
               (int)multi_points[idx - 1][2], (int)multi_points[idx - 1][3],
               (int)multi_points[idx - 1][4], (int)multi_points[idx - 1][5]);
        break;
    }

    case 'Q': {
        /* QL count rounds [speed]: loop saved points in order.
         * qv segment speed, qd point dwell time; both use 1-based index. */
        if (tokens[0][1] == 'V' || tokens[0][1] == 'v') {
            if (ntok < 3) {
                cprint("  Usage: qv <segment 1~8> <speed deg/s>\r\n");
                break;
            }
            int idx = atoi(tokens[1]);
            float speed = (float)atof(tokens[2]);
            if (idx < 1 || idx > MULTI_POINT_MAX) {
                cprint("  Segment index must be 1~%d\r\n", MULTI_POINT_MAX);
                break;
            }
            if (speed < 5.0f) speed = 5.0f;
            if (speed > 300.0f) speed = 300.0f;
            multi_point_segment_speed[idx - 1] = speed;
            cprint("  Segment %d speed = %d deg/s\r\n", idx, (int)speed);
            break;
        }
        if (tokens[0][1] == 'D' || tokens[0][1] == 'd') {
            if (ntok < 3) {
                cprint("  Usage: qd <point 1~8> <dwell ms>\r\n");
                break;
            }
            int idx = atoi(tokens[1]);
            long dwell = atol(tokens[2]);
            if (idx < 1 || idx > MULTI_POINT_MAX) {
                cprint("  Point index must be 1~%d\r\n", MULTI_POINT_MAX);
                break;
            }
            if (dwell < 0) dwell = 0;
            if (dwell > 60000) dwell = 60000;
            multi_point_dwell_ms[idx - 1] = (uint32_t)dwell;
            cprint("  Point %d dwell = %d ms\r\n", idx, (int)dwell);
            break;
        }
        if (tokens[0][1] == 'L' && tokens[0][2] == 'C') {
            /* QLC count rounds [speed]: 连续流水线循环, 段间不停顿 */
            if (ntok < 3) {
                cprint("  Usage: QLC <count 2~8> <rounds, 0=infinite> [speed]\r\n");
                break;
            }
            uint8_t count = (uint8_t)atoi(tokens[1]);
            uint16_t rounds = (uint16_t)atoi(tokens[2]);
            float speed = (ntok >= 4) ? (float)atof(tokens[3]) : 30.0f;
            if (count < 2 || count > MULTI_POINT_MAX || count > multi_point_count) {
                cprint("  Save points 1..%d first; count must be 2~%d\r\n",
                       multi_point_count, MULTI_POINT_MAX);
                break;
            }
            if (rounds > (uint16_t)(65534U / count)) rounds = (uint16_t)(65534U / count);
            if (speed < 5.0f) speed = 5.0f;
            if (speed > 300.0f) speed = 300.0f;
            if (qlc_active || point_loop_active || swing_active ||
                swing_resume_pending || multi_point_legs_left != 0U) {
                cprint("  Another loop is active. Use S first.\r\n");
                break;
            }
            cmdQLCStart(count, rounds, speed);
            break;
        }
        if (tokens[0][1] != 'L' || ntok < 3) {
            cprint("  Usage: QL <count 2~8> <rounds, 0=infinite> [speed]\r\n");
            break;
        }
        uint8_t count = (uint8_t)atoi(tokens[1]);
        uint16_t rounds = (uint16_t)atoi(tokens[2]);
        float speed = (ntok >= 4) ? (float)atof(tokens[3]) : 30.0f;
        if (count < 2 || count > MULTI_POINT_MAX || count > multi_point_count) {
            cprint("  Save points 1..%d first; count must be 2~%d\r\n",
                   multi_point_count, MULTI_POINT_MAX);
            break;
        }
        if (rounds > (uint16_t)(65534U / count)) rounds = (uint16_t)(65534U / count);
        if (speed < 5.0f) speed = 5.0f;
        if (speed > 300.0f) speed = 300.0f;
        if (point_loop_active || swing_active || swing_resume_pending || qlc_active || multi_point_count == 0) {
            cprint("  Another loop is active. Use S first.\r\n");
            break;
        }
        cmdMultiPointStart(count, rounds, speed);
        break;
    }

    case 'B': {
        float a[6], b[6];
        uint16_t rounds;
        float speed;
        if (tokens[0][1] == 'G') {
            /* BG x1 y1 z1 pitch1 x2 y2 z2 pitch2 rounds [speed]. */
            if (ntok < 10) {
                cprint("  Usage: BG x1 y1 z1 pitch1 x2 y2 z2 pitch2 rounds [speed]\r\n");
                break;
            }
            rounds = (uint16_t)atoi(tokens[9]);
            speed = (ntok >= 11) ? (float)atof(tokens[10]) : 30.0f;
            if (!cmdSolveIKPoint((float)atof(tokens[1]), (float)atof(tokens[2]),
                                 (float)atof(tokens[3]), (float)atof(tokens[4]), a) ||
                !cmdSolveIKPoint((float)atof(tokens[5]), (float)atof(tokens[6]),
                                 (float)atof(tokens[7]), (float)atof(tokens[8]), b)) {
                cprint("  BG: IK failed, limits exceeded, or FK error > 5 mm.\r\n");
                break;
            }
        } else {
            /* B a1..a6 b1..b6 rounds [speed]; rounds=0 means infinite. */
            if (ntok < 14) {
                cprint("  Usage: B a1 a2 a3 a4 a5 a6 b1 b2 b3 b4 b5 b6 rounds [speed]\r\n");
                break;
            }
            for (uint8_t j = 0; j < 6; j++) {
                a[j] = (float)atof(tokens[1 + j]);
                b[j] = (float)atof(tokens[7 + j]);
            }
            rounds = (uint16_t)atoi(tokens[13]);
            speed = (ntok >= 15) ? (float)atof(tokens[14]) : 30.0f;
            for (uint8_t j = 0; j < 6; j++) {
                a[j] = clamp_angle(j, a[j]);
                b[j] = clamp_angle(j, b[j]);
            }
        }
        if (rounds > 1000U) rounds = 1000U;
        if (speed < 5.0f) speed = 5.0f;
        if (speed > 300.0f) speed = 300.0f;
        if (point_loop_active || swing_active || swing_resume_pending || qlc_active) {
            cprint("  Another loop is active. Use S first.\r\n");
            break;
        }
        cmdPointLoopStart(a, b, rounds, speed);
        break;
    }

    case 'P': {
        /* Predefined pose: P 0~4; 0924CG: P here/clear 折叠位标定 */
        if (ntok < 2) {
            cprint("  Usage: P <0~4> | P here | P clear\r\n");
            cprint("  0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2\r\n");
            cprint("  here  = 把当前位姿标定为折叠位 (P 0 的新目标, 存 flash)\r\n");
            cprint("  clear = 撤销标定, 回编译默认折叠位\r\n");
            break;
        }
        /* ⚠ 必须在 atoi 之前拦截: atoi("here")=0 会误触发 P 0 运动 */
        if (strcmp(tokens[1], "here") == 0) {
            cmdPoseCalHere();
            break;
        }
        if (strcmp(tokens[1], "clear") == 0 || strcmp(tokens[1], "def") == 0) {
            cmdPoseCalClear();
            break;
        }
        cmdPose((uint8_t)atoi(tokens[1]));
        break;
    }

    case 'S':
        /* SP <id> <value>: 设置算法参数（arm_params，LP 查看 id 表） */
        if (tokens[0][1] == 'P' || tokens[0][1] == 'p') {
            if (ntok < 3) { cprint("  Usage: SP <id> <value>  (LP lists ids)\r\n"); break; }
            int pid = atoi(tokens[1]);
            float val = (float)atof(tokens[2]);
            if (ArmParams_Set((uint8_t)pid, val) != 0) {
                cprint("  Unknown param id %d (LP lists ids)\r\n", pid);
                break;
            }
            const ArmParamMeta* pm = ArmParams_Find((uint8_t)pid);
            float applied = 0.0f;
            ArmParams_Get((uint8_t)pid, &applied);
            cprint("  Param #%d %s = ", pid, pm ? pm->name : "?");
            cprint_fixed3(applied);
            cprint(" %s (applied)\r\n", pm ? pm->unit : "");
            RLOG_I("CFG", "param #%d = %d (x1000)", pid, (int)(applied * 1000.0f));
            break;
        }
        /* Stop current trajectory */
        Trajectory_Stop();
        g_soft_stop = 1U;                      /* ⭐0924CZ: 软件急停态, 报警灯常亮 */
        g_soft_stop_tick = HAL_GetTick();
        safe_seq_active = 0;   /* 同步取消安全序列, 防止停止后残留段继续执行 */
        swing_active = 0;
        swing_resume_pending = 0;  /* W 准备段也被取消, 不再触发挥动 */
        point_loop_active = 0;
        point_loop_legs_left = 0;
        qlc_active = 0;      /* QLC 连续循环一并停止 */
        qlc_started = 0;
        qlc_legs_left = 0;
        multi_point_legs_left = 0;
        multi_point_started = 0;
        multi_point_waiting = 0;
        multi_point_wait_until = 0;
        multi_point_count = 0;
        traj_settle_pending = 0U;
        correction_pending = 0;
        teach_play_abort(1);
        if (teach_mode_active) cmdTeachExit();
        RLOG_I("MOVE", "stop command: all motion states cleared");
        if (Trajectory_IsActive()) {
            cprint("  Trajectory stopped.\r\n");
        } else {
            cprint("  Stopped.\r\n");
        }
        break;

    case 'W': {
        /* Swing/wave: W [count] */
        if (swing_active) {
            cprint("  Swing already running. Use 'S' to stop.\r\n");
            break;
        }
        uint8_t count = (ntok >= 2) ? (uint8_t)atoi(tokens[1]) : 3;
        if (count == 0) count = 3;
        if (count > 20) count = 20;
        /* 准备段(->WAVE1)走安全序列 (FK 逐段校验), 序列完成后由
         * safe_seq_service 置 swing_active 开始真正挥动。
         * 旧代码先置 swing_active=1 再 cmdMove —— cmdMove 入口会把
         * swing_active 清零, 挥动从未执行过 (2026-09-02 修复)。 */
        swing_remaining = count;
        swing_phase = 0;  /* 首次挥动目标: WAVE2 (准备段已在 WAVE1) */
        cprint("  Swing: %d times (safe approach to WAVE1, then WAVE1<->WAVE2)\r\n", count);
        swing_resume_pending = 1;
        if (!cmdGotoSafePlan((float*)POSE_WAVE1, 0.0f, 0.0f, 0.0f)) {
            swing_resume_pending = 0;
            cprint("  Swing aborted: no safe path to WAVE1.\r\n");
        }
        break;
    }

    case 'T': {
        /* Temperature: T (all) or T J (single) */
        if (ntok >= 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 6) { cprint("  Joint must be 0-5\r\n"); break; }
            uint8_t temp = SERVO_GetTemperature(j);
            cprint("  J%d (ID=%d): %dC\r\n", j+1, JOINT_ID[j], temp);
        } else {
            cprint("\r\n  +----+----+------+\r\n");
            cprint("  |Jnt | ID | Temp |\r\n");
            cprint("  +----+----+------+\r\n");
            for (uint8_t j = 0; j < 6; j++) {
                uint8_t temp = SERVO_GetTemperature(j);
                cprint("  | J%d | %d  | %3dC |\r\n", j+1, JOINT_ID[j], temp);
            }
            cprint("  +----+----+------+\r\n");
        }
        break;
    }

    case 'O': {
        /* Load / torque readout: O (all) or O J (single)
         * 输出符号-幅度解码后的 signed 负载 (负=CW堵转, 正=CCW堵转, ~0=空载)
         * 源自飞特协议 PresentLoad(60) 位10=方向, 低10位=幅值。 */
        if (ntok >= 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 6) { cprint("  Joint must be 0-5\r\n"); break; }
            int16_t load = SERVO_GetPresentLoadSigned(j);
            cprint("  J%d (ID=%d): load=%+d\r\n", j+1, JOINT_ID[j], load);
        } else {
            cprint("\r\n  +----+----+----------+------+\r\n");
            cprint("  |Jnt | ID | Load(sgn)|Stat  |\r\n");
            cprint("  +----+----+----------+------+\r\n");
            for (uint8_t j = 0; j < 6; j++) {
                int16_t load = SERVO_GetPresentLoadSigned(j);
                uint8_t err = SERVO_GetHardwareError(j);
                /* 读取失败时 err=0xFF, 标记 n/a */
                cprint("  | J%d | %d  | %+8d | %s\r\n",
                       j+1, JOINT_ID[j], load,
                       err == 0xFF ? "n/a" : "    ");
            }
            cprint("  +----+----+----------+------+\r\n");
        }
        break;
    }

    case 'E': {
        /* Hardware error status: E (all) or E J (single)
         * 对照飞特/LeRobot 错误位: bit0过压 bit1欠压 bit2过温
         * bit3过载 bit4传感器 bit5电机。 */
        if (ntok >= 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 6) { cprint("  Joint must be 0-5\r\n"); break; }
            uint8_t err = SERVO_GetHardwareError(j);
            cprint("  J%d (ID=%d): err=0x%02X [%s]\r\n",
                   j+1, JOINT_ID[j], err, SERVO_ErrorString(err));
        } else {
            cprint("\r\n  ===== Servo Hardware Error =====\r\n");
            for (uint8_t j = 0; j < 6; j++) {
                uint8_t err = SERVO_GetHardwareError(j);
                cprint("  J%d (ID=%d): 0x%02X [%s]\r\n",
                       j+1, JOINT_ID[j], err, SERVO_ErrorString(err));
            }
        }
        break;
    }

    case 'd': {
        /* d t 1/0: toggle trajectory-plan diagnostic output
         * (d t 1 = log planned curve targets, d t 0 = off) */
        if (ntok >= 2 && (tokens[1][0] == 't' || tokens[1][0] == 'T')) {
            if (ntok >= 3 && tokens[2][0] == '1') {
                diag_plan_enable = 1U;
                cprint("  Plan diagnostic ENABLED (dt on)\r\n");
            } else {
                diag_plan_enable = 0U;
                cprint("  Plan diagnostic DISABLED\r\n");
            }
            break;
        }
        /* Diagnostic: test the active servo bus step by step.
         * 主臂舵机总线 = USART3 (BUS_A, PB10/PB11); 从臂 = UART5 (BUS_B, PC12/PD2)。
         * 用 DIAG_BUS / DIAG_HUART 统一引用, 不写死具体 UART。 */
#if ARM_ONE_BUS_BUILD
  #define DIAG_BUS    ONE_BUS
  #define DIAG_HUART  ((ONE_BUS == BUS_A) ? (&huart3) : (&huart5))
  #define DIAG_NAME   ONE_BUS_PORT
#else
  #define DIAG_BUS    BUS_B
  #define DIAG_HUART  (&huart5)
  #define DIAG_NAME   "UART5 / Bus B"
#endif
        cprint("\r\n  ===== Servo Bus Diagnostic (%s) =====\r\n", DIAG_NAME);

        cprint("  [1] Loopback test (TX self-echo through servo bus wire)...\r\n");

        /* 方向切换: 仅从臂经 74HC126 时需要(PA10 拉高使能门3);
         * 主臂开漏单线无 74HC126, 此写操作无效(引脚未接 126), 不影响回环。 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

        /* Clear stale flags */
        __HAL_UART_CLEAR_OREFLAG(DIAG_HUART);
        __HAL_UART_CLEAR_FEFLAG(DIAG_HUART);
        __HAL_UART_CLEAR_NEFLAG(DIAG_HUART);

        /* Ensure RX is ON (we want the echo!) */
        SET_BIT(DIAG_HUART->Instance->CR1, USART_CR1_RE);

        /* Send 3 bytes with known pattern: 0xAA 0x55 0xF0.
         * 逐字节发送并立即读回该字节的回声 —— 一次性发 3 字节再读,
         * 回声会在发送期间淤积 ORE, 前两字节被覆盖丢失导致误判。 */
        uint8_t test_bytes[3] = {0xAA, 0x55, 0xF0};
        uint8_t rx_buf[3] = {0};
        uint16_t match = 0;
        HAL_StatusTypeDef tx_st = HAL_OK;
        uint8_t echo_ok = 1;

        for (int i = 0; i < 3; i++) {
            HAL_StatusTypeDef s1 = HAL_UART_Transmit(DIAG_HUART, &test_bytes[i], 1, 100);
            if (s1 != HAL_OK) tx_st = s1;
            uint32_t t0 = HAL_GetTick();
            uint8_t got_echo = 0;
            while ((HAL_GetTick() - t0) < 5) {
                if (__HAL_UART_GET_FLAG(DIAG_HUART, UART_FLAG_ORE)) {
                    __HAL_UART_CLEAR_OREFLAG(DIAG_HUART);
                }
                if (__HAL_UART_GET_FLAG(DIAG_HUART, UART_FLAG_RXNE)) {
                    rx_buf[i] = (uint8_t)(DIAG_HUART->Instance->DR & 0xFF);
                    got_echo = 1;
                    break;
                }
            }
            if (!got_echo) { echo_ok = 0; break; }
        }
        cprint("      TX 3 bytes: %d (0=OK)\r\n", tx_st);

        if (echo_ok) {
            match = (rx_buf[0] == 0xAA && rx_buf[1] == 0x55 && rx_buf[2] == 0xF0) ? 1 : 0;
            cprint("      RX data: %02X %02X %02X  match=%d\r\n",
                   rx_buf[0], rx_buf[1], rx_buf[2], match);
        } else {
            cprint("      RX echo: TIMEOUT (no echo byte)\r\n");
        }

        /* Set back to RX direction */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

        /* Restore normal state: receiver stays enabled (RE 常开,
         * 回声由 DXL_Transmit 之后的 FlushErrors 排空) */
        SET_BIT(DIAG_HUART->Instance->CR1, USART_CR1_RE);

        if (!match) {
            cprint("      >>> LOOPBACK FAILED! Servo-bus path broken.\r\n");
#if ARM_MASTER_BUILD
            cprint("          Check: PB10(TX)->servo DATA wire->PB11(RX)\r\n");
            cprint("          Check: servo bus pull-up present, common GND\r\n");
#else
            cprint("          Check: PC12->Pin9, Pin8->DATA_B, DATA_B->PD2\r\n");
            cprint("          Check: PA10->Pin10, Pin14=3.3V, Pin7=GND\r\n");
#endif
            break;
        }
#if ARM_MASTER_BUILD
        cprint("      >>> LOOPBACK OK! PB10->servo DATA->PB11 path works.\r\n\r\n");
#else
        cprint("      >>> LOOPBACK OK! PC12->74HC126->PD2 path works.\r\n\r\n");
#endif

        /* 2) Dump the active servo-bus UART status register */
        cprint("  [2] %s SR: 0x%08lX\r\n", DIAG_NAME, DIAG_HUART->Instance->SR);

        /* 3) Ping ID=4 on active servo bus */
        cprint("  [3] Pinging ID=4 on servo bus...\r\n");
        {
            uint8_t tx[16];
            uint16_t tx_len = DXL_BuildPacket(4, 0x01 /*INST_PING*/, NULL, 0, tx);

            HAL_StatusTypeDef st = DXL_Transmit(DIAG_BUS, tx, tx_len);
            cprint("      TX status: %d (0=OK 1=ERROR 2=BUSY 3=TIMEOUT)\r\n", st);

            /* 舵机 PING 只回 6 字节: 逐字节收帧, 静默判停。
             * (旧版 HAL_UART_Receive 凑 14 字节必然超时, 结果失真) */
            uint8_t rx[32];
            uint16_t got = 0;
            memset(rx, 0, sizeof(rx));
            st = DXL_Receive(DIAG_BUS, rx, sizeof(rx), &got, 200);
            cprint("      RX status: %d, %u byte(s)\r\n", st, got);

            if (st == HAL_OK && got > 0) {
                cprint("      RX raw: ");
                for (uint16_t i = 0; i < got; i++) cprint("%02X ", rx[i]);
                cprint("\r\n");

                /* 帧头同步后解析 */
                uint16_t off = 0;
                while ((uint16_t)(off + 2) <= got &&
                       !(rx[off] == 0xFF && rx[off + 1] == 0xFF)) off++;
                if ((uint16_t)(off + 2) <= got) {
                    DXL_Packet_t pkt;
                    if (DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt))
                        cprint("      Parse OK! ID=%d err=0x%02X\r\n", pkt.id, pkt.error);
                    else
                        cprint("      Parse FAILED (checksum/header)\r\n");
                } else {
                    cprint("      No 0xFF 0xFF header in reply\r\n");
                }
            } else {
                cprint("      >> No servo response.\r\n");
                cprint("      >> Loopback OK + no reply = check servo power,\r\n");
                cprint("         servo DATA wiring, common GND.\r\n");
            }
        }

        /* 4) 连环 Ping 统计(可选, d p): 对舵机总线 ID4/5/6 各连续 Ping 30 次,
         *    统计 OK/无回应/坏帧 分类 —— 一次定位"收不到"还是"收到坏帧"。 */
        if (ntok >= 2 && (tokens[1][0] == 'p' || tokens[1][0] == 'P')) {
            static const uint8_t ids[3] = {4U, 5U, 6U};
            cprint("\r\n  === Servo bus stress test (d p): 30x ping each ID ===\r\n");
            for (uint8_t k = 0; k < 3; k++) {
                uint8_t id = ids[k];
                uint16_t ok = 0, noresp = 0, badframe = 0;
                for (uint16_t n = 0; n < 30; n++) {
                    /* 直接发 ping, 收原始字节, 分类统计 */
                    uint8_t tx[8];
                    uint16_t tx_len = DXL_BuildPacket(id, 0x01 /*INST_PING*/, NULL, 0, tx);
                    uint8_t rx[16];
                    uint16_t got = 0;
                    HAL_StatusTypeDef st = DXL_TransmitReceive(DIAG_BUS, tx, tx_len,
                                                               rx, sizeof(rx), &got, 60);
                    if (st != HAL_OK || got < 6) { noresp++; continue; }

                    /* 找帧头 */
                    uint16_t off = 0;
                    while ((uint16_t)(off + 2) <= got &&
                           !(rx[off] == 0xFF && rx[off + 1] == 0xFF)) off++;
                    if ((uint16_t)(off + 2) > got) { noresp++; continue; }

                    /* 解析 */
                    DXL_Packet_t pkt;
                    if (!DXL_ParsePacket(&rx[off], (uint16_t)(got - off), &pkt)) {
                        badframe++;
                        continue;
                    }
                    if (pkt.id != id) { badframe++; continue; }
                    ok++;
                }
                cprint("  ID=%u: OK=%u NO-RESP=%u BAD-FRAME=%u / 30\r\n",
                       id, ok, noresp, badframe);
                HAL_Delay(20);
            }
            cprint("  (OK 高 = 物理层稳; NO-RESP 高 = 收不到字节/舵机不应答; BAD-FRAME 高 = 帧错/校验失/串扰)\r\n");
        }
  #undef DIAG_BUS
  #undef DIAG_HUART
        break;
    }

    case 'L': {
        /* LP: 列出全部算法参数（arm_params 统一参数表） */
        if (tokens[0][1] == 'P' || tokens[0][1] == 'p') {
            const ArmParamMeta* t = ArmParams_Table();
            cprint("  === Algorithm Params (arm_params, %d entries) ===\r\n",
                   ArmParams_Count());
            cprint("  id  name             value       range             unit\r\n");
            for (uint8_t i = 0; i < ArmParams_Count(); i++) {
                float v = 0.0f;
                ArmParams_Get(t[i].id, &v);
                cprint("  #%2d %-16s ", t[i].id, t[i].name);
                cprint_fixed3(v);
                cprint(" [");
                cprint_fixed3(t[i].vmin);
                cprint(" ~ ");
                cprint_fixed3(t[i].vmax);
                cprint("] %s\r\n", t[i].unit);
            }
            cprint("  Set: SP <id> <value>\r\n");
            break;
        }
        /* LL [0~4]: 运行日志级别（0=关 1=错 2=警 3=信息 4=调试） */
        if (tokens[0][1] == 'L' || tokens[0][1] == 'l') {
            if (ntok >= 2) {
                RunLog_SetLevel((uint8_t)atoi(tokens[1]));
            }
            cprint("  RunLog level = %d (0=off 1=err 2=warn 3=info 4=debug)\r\n",
                   RunLog_GetLevel());
            break;
        }
        /* Joint limits: L (show all) or L J min max (set) */
        if (ntok == 2 && strcmp(tokens[1], "clear") == 0) {
            cmdLimitsClear();   /* 0924BY: 撤销手动限位, 回校准/默认 */
        } else if (ntok < 4) {
            cmdLimits(6, 0, 0);  /* 6 = show all */
        } else {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            float mn = (float)atof(tokens[2]);
            float mx = (float)atof(tokens[3]);
            cmdLimits(j, mn, mx);
        }
        break;
    }

    case 'C': {
        /* Calibration offset: C (show) or C dx dy dz (set) */
        if (ntok == 1) {
            cprint("  World offset (actual - command):\r\n");
            cprint("    X=%d Y=%d Z=%d mm\r\n",
                   (int)WORLD_OFFSET_X, (int)WORLD_OFFSET_Y, (int)WORLD_OFFSET_Z);
            cprint("  Usage: C <dx> <dy> <dz>\r\n");
        } else if (ntok == 4) {
            float dx = (float)atof(tokens[1]);
            float dy = (float)atof(tokens[2]);
            float dz = (float)atof(tokens[3]);
            Kinematics_SetOffset(dx, dy, dz);
            cprint("  Offset set: X=%d Y=%d Z=%d mm\r\n",
                   (int)dx, (int)dy, (int)dz);
            /* ⭐0924AI: 偏移改变会平移世界坐标, 桌面下限必须随之重锚 ——
             * traj_z_floor_mm = FK(POSE_HOME).z - WORLD_OFFSET_Z - margin,
             * 不重锚的话下限仍用旧偏移算出的值, 相对锚定被破坏,
             * 贴桌 g 目标会被多拒 |dz| mm (0924-09-29 实测 z 差 13mm 踩到)。 */
            update_home_dock();
            /* 0924AS: 设值即持久化 (区F)。zf 偏好存在就一并保留,
             * update_home_dock 里的 [zf pref] 分支保证下限不被重锚覆盖。 */
            uint8_t zfv = (geo_zfloor_mm >= 0.0f) ? 1U : 0U;
            /* 0924CG: 改 Save2 —— 保留当前 zf_on/margin/fold (旧 v1 包装
             * 会强写 zf_on=1 且抹掉 P here 折叠位标定) */
            if (GeoCfg_Save2(dx, dy, dz, zfv ? geo_zfloor_mm : 0.0f, zfv,
                             geo_zf_on, geo_margin_raw)
                    == CFG_FLASH_OK) {
                cprint("  已写入 flash (区F), 重启自动恢复%s\r\n",
                       zfv ? " (zf 偏好保留)" : "");
            } else {
                cprint("  ⚠ flash 写入失败: 偏移仅本次上电有效\r\n");
            }
        } else {
            cprint("  Usage: C <dx> <dy> <dz>\r\n");
            cprint("  dx = actual_x - command_x, etc.\r\n");
        }
        break;
    }

    case 'G': {
        /* G: 自适应校正/PID 状态; gc.k 为占位(重力补偿仅用于 teach 悬浮) */
        if (ntok == 1) {
            cprint("  === Adaptive Correction & PID Status ===\r\n");
            cprint("  gc.k (feedforward) = %.3f [占位: 不参与运动算法]\r\n", GRAVITY_COMP_K);
            cprint("  Correction = %s/%s (round %d/%d, threshold +/-%d deg)\r\n",
                   correction_enabled ? "ON" : "OFF",
                   correction_pending ? "ACTIVE" : "idle",
                   correction_count, corr_max_rounds, (int)corr_threshold);
            cprint("  Joint  P   I   D   | Gain  Osc Stall\r\n");
            cprint("  ----- --- --- --- | ----- --- -----\r\n");
            for (uint8_t j = 0; j < 6; j++) {
                uint8_t active = (j >= CORR_FIRST_JOINT && j <= CORR_LAST_JOINT);
                cprint("  J%d    %3d %3d %3d | %3d%%  %3d  %3d%s\r\n",
                       j+1, pid_p[j], pid_i[j], pid_d[j],
                       active ? (int)(corr_gain[j]*100) : 0,
                       oscillation_count[j], stall_count[j],
                       active ? "" : " (no corr)");
            }
        } else if (ntok == 2) {
            float k = (float)atof(tokens[1]);
            if (k < 0.0f) k = 0.0f;
            if (k > 0.5f) k = 0.5f;
            GRAVITY_COMP_K = k;
            cprint("  gc.k stored = %.3f (占位, 不影响任何运动)\r\n", k);
        } else {
            cprint("  Usage: G [k]  (k 仅占位存储)\r\n");
        }
        break;
    }

    case 'K': {
        /* P-Gain (Kp): K (show all) or K J (show one) or K J value (set) */
        if (ntok == 1) {
            cprint("  P-Gain: ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d ", j+1, pid_p[j]);
            cprint("\r\n");
        } else if (ntok == 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 1 && j <= 6)
                cprint("  J%d P-Gain = %d\r\n", j, pid_p[j-1]);
            else
                cprint("  Usage: K [J] [value]  J=1~6\r\n");
        } else if (ntok == 3) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            uint8_t val = (uint8_t)atoi(tokens[2]);
            if (j >= 1 && j <= 6 && val <= 254) {
                set_pid_gain(j-1, STS_ADDR_P_GAIN, val);
                cprint("  J%d P-Gain = %d\r\n", j, val);
                cprint("  [cache only] send 'Y %d' to burn into servo EEPROM\r\n", j);
            } else {
                cprint("  Usage: K J value  J=1~6, value=0~254\r\n");
            }
        }
        break;
    }

    case 'V': {
        /* V [value]: show or immediately set STS3215 Goal Speed for all six servos. */
        if (ntok == 1) {
            cprint("  All six STS3215 Goal Speed = %u (range 0~4095)\r\n",
                   (unsigned)all_servo_moving_speed);
        } else if (ntok == 2) {
            char *end = NULL;
            long value = strtol(tokens[1], &end, 10);
            if (*tokens[1] == '\0' || *end != '\0' || value < 0 || value > 4095) {
                cprint("  Usage: V <0~4095>\r\n");
                break;
            }
            all_servo_moving_speed = (uint16_t)value;
            SERVO_SetInitSpeed(all_servo_moving_speed);
            uint8_t failed = 0;
            for (uint8_t j = 0; j < 6; j++) {
                if (SERVO_SetSpeed(j, all_servo_moving_speed) != HAL_OK) failed++;
            }
            cprint("  All six STS3215 Goal Speed set to %u%s.\r\n",
                   (unsigned)all_servo_moving_speed,
                   failed ? " (some writes failed)" : "");
            cprint("  Use F to save this default after testing.\r\n");
        } else {
            cprint("  Usage: V [0~4095]\r\n");
        }
        break;
    }

    case 'N': {
        /* Set all PID for one joint: N J P I D */
        if (ntok == 5) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            uint8_t p = (uint8_t)atoi(tokens[2]);
            uint8_t i = (uint8_t)atoi(tokens[3]);
            uint8_t d = (uint8_t)atoi(tokens[4]);
            if (j >= 1 && j <= 6 && p <= 254 && i <= 254 && d <= 254) {
                set_pid_gain(j-1, STS_ADDR_P_GAIN, p);
                set_pid_gain(j-1, STS_ADDR_I_GAIN, i);
                set_pid_gain(j-1, STS_ADDR_D_GAIN, d);
                cprint("  J%d PID = P:%d I:%d D:%d\r\n", j, p, i, d);
                cprint("  [cache only] send 'Y %d' to burn into servo EEPROM\r\n", j);
            } else {
                cprint("  Usage: N J P I D  J=1~6, each 0~254\r\n");
            }
        } else if (ntok == 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 1 && j <= 6) {
                uint8_t k = (uint8_t)(j - 1);
                cprint("  J%d cache PID = P:%d I:%d D:%d\r\n",
                       j, pid_p[k], pid_i[k], pid_d[k]);
                uint8_t rp = 0, rd = 0, ri = 0;
                if (SERVO_ReadPidEeprom(k, &rp, &rd, &ri)) {
                    uint8_t match = (rp == pid_p[k] && rd == pid_d[k] && ri == pid_i[k]);
                    cprint("  J%d servo EEPROM = P:%d D:%d I:%d  %s\r\n",
                           j, rp, rd, ri, match ? "(一致)" : "(不一致! 需 Y 烧写)");
                } else {
                    cprint("  J%d servo EEPROM read FAILED (offline?)\r\n", j);
                }
            } else {
                cprint("  Usage: N J P I D  (show: N J)\r\n");
            }
        } else {
            cprint("  Usage: N J P I D  (set all PID for joint J)\r\n");
            cprint("         N J        (show PID for joint J)\r\n");
        }
        break;
    }

    case 'D': {
        /* D-Gain: D (show all) or D J (show one) or D J value (set) */
        if (ntok == 1) {
            cprint("  D-Gain: ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d ", j+1, pid_d[j]);
            cprint("\r\n");
        } else if (ntok == 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 1 && j <= 6)
                cprint("  J%d D-Gain = %d\r\n", j, pid_d[j-1]);
            else
                cprint("  Usage: D [J] [value]  J=1~6\r\n");
        } else if (ntok == 3) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            uint8_t val = (uint8_t)atoi(tokens[2]);
            if (j >= 1 && j <= 6 && val <= 254) {
                set_pid_gain(j-1, STS_ADDR_D_GAIN, val);
                cprint("  J%d D-Gain = %d\r\n", j, val);
                cprint("  [cache only] send 'Y %d' to burn into servo EEPROM\r\n", j);
            } else {
                cprint("  Usage: D J value  J=1~6, value=0~254\r\n");
            }
        }
        break;
    }

    case 'I': {
        /* I-Gain: I (show all) or I J (show one) or I J value (set) */
        if (ntok == 1) {
            cprint("  I-Gain: ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d ", j+1, pid_i[j]);
            cprint("\r\n");
        } else if (ntok == 2) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            if (j >= 1 && j <= 6)
                cprint("  J%d I-Gain = %d\r\n", j, pid_i[j-1]);
            else
                cprint("  Usage: I [J] [value]  J=1~6\r\n");
        } else if (ntok == 3) {
            uint8_t j = (uint8_t)atoi(tokens[1]);
            uint8_t val = (uint8_t)atoi(tokens[2]);
            if (j >= 1 && j <= 6 && val <= 254) {
                set_pid_gain(j-1, STS_ADDR_I_GAIN, val);
                cprint("  J%d I-Gain = %d\r\n", j, val);
                cprint("  [cache only] send 'Y %d' to burn into servo EEPROM\r\n", j);
            } else {
                cprint("  Usage: I J value  J=1~6, value=0~254\r\n");
            }
        }
        break;
    }

    case 'A': {
        /* Adaptive correction config:
         * A              = show config
         * A e 0/1        = enable/disable correction
         * A t val        = set threshold (deg)
         * A r val        = set max rounds
         * A g J val      = set initial gain for joint J (0~100 = 0.0~1.0) */
        if (ntok == 1) {
            cprint("  === Adaptive Correction Config ===\r\n");
            cprint("  Enabled:    %s\r\n", correction_enabled ? "YES" : "NO");
            cprint("  Threshold:  +/-%d deg\r\n", (int)corr_threshold);
            cprint("  Max rounds: %d\r\n", corr_max_rounds);
            cprint("  Joints:     J%d~J%d\r\n", CORR_FIRST_JOINT+1, CORR_LAST_JOINT+1);
            cprint("  Init gains: ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d%% ", j+1, (int)(corr_gain_init[j]*100));
            cprint("\r\n");
        } else if (ntok >= 2) {
            char sub = tokens[1][0];
            if (sub == 'e' && ntok == 3) {
                correction_enabled = (uint8_t)atoi(tokens[2]);
                cprint("  Correction %s\r\n", correction_enabled ? "ENABLED" : "DISABLED");
            } else if (sub == 't' && ntok == 3) {
                float t = (float)atof(tokens[2]);
                if (t < 0.5f) t = 0.5f;
                if (t > 10.0f) t = 10.0f;
                corr_threshold = t;
                cprint("  Threshold = +/-%d deg\r\n", (int)corr_threshold);
            } else if (sub == 'r' && ntok == 3) {
                uint8_t r = (uint8_t)atoi(tokens[2]);
                if (r < 1) r = 1;
                if (r > 20) r = 20;
                corr_max_rounds = r;
                cprint("  Max rounds = %d\r\n", corr_max_rounds);
            } else if (sub == 'g' && ntok == 4) {
                uint8_t j = (uint8_t)atoi(tokens[2]);
                int pct = atoi(tokens[3]);
                if (j >= 1 && j <= 6 && pct >= 0 && pct <= 100) {
                    corr_gain_init[j-1] = pct / 100.0f;
                    cprint("  J%d init gain = %d%%\r\n", j, pct);
                } else {
                    cprint("  Usage: A g J val  J=1~6, val=0~100\r\n");
                }
            } else {
                cprint("  Usage:\r\n");
                cprint("    A e 0/1     Enable/disable\r\n");
                cprint("    A t val     Threshold (deg)\r\n");
                cprint("    A r val     Max rounds\r\n");
                cprint("    A g J val   Init gain J (0~100%%)\r\n");
            }
        }
        break;
    }

    case 'F':
        pid_save_current();
        break;

    case 'X':
        pid_reset_defaults();
        break;

    case 'Y': {
        /* Y J = 把缓存 PID (K/D/I/N 所设) 真正烧入舵机 EEPROM (J=1~6)
         * Y 0 = 逐个烧写全部 6 颗。
         * 背景: PID 在 EEPROM 且默认上锁, K/D/I/N 只改了 STM32 缓存,
         * 裸写会被舵机静默丢弃 —— 必须 Y 才真正生效 (Torque OFF ~0.6s)。
         * CAUTION: 烧写期间该关节失力, 请用手扶住机械臂! */
        if (ntok == 2) {
            int j = atoi(tokens[1]);
            if (j < 0 || j > 6) {
                cprint("  Usage: Y J  J=1~6 (or 0 = all six)\r\n");
                break;
            }
            if (Trajectory_IsActive() || swing_active) {
                cprint("  Arm busy (trajectory/multi-point active). Stop first.\r\n");
                break;
            }
            if (j == 0) {
                cprint("  Burning PID to ALL servos (each Torque OFF ~0.6 s).\r\n");
                cprint("  ** SUPPORT THE ARM - it will go limp several times! **\r\n");
                for (uint8_t k = 0; k < 6; k++) {
                    uint8_t ok = SERVO_ApplyPidToEeprom(k, pid_p[k], pid_i[k], pid_d[k]);
                    uint8_t rp = 0, rd = 0, ri = 0;
                    /* 回读失败必须显式报告, 否则会把 0 当成舵机真实值打印 */
                    if (SERVO_ReadPidEeprom(k, &rp, &rd, &ri)) {
                        cprint("  J%d: %s  EEPROM=P:%d D:%d I:%d\r\n", k+1,
                               ok ? "OK" : "FAIL", rp, rd, ri);
                    } else {
                        uint8_t hw = SERVO_GetHardwareError(k);
                        cprint("  J%d: %s  EEPROM readback failed [%s] (check 'E %d')\r\n",
                               k+1, ok ? "burned" : "FAIL",
                               hw == 0xFF ? "no reply" : SERVO_ErrorString(hw), k+1);
                    }
                }
            } else {
                uint8_t k = (uint8_t)(j - 1);
                cprint("  Burning J%d PID P:%d I:%d D:%d -> EEPROM (Torque OFF ~0.6s)\r\n",
                       j, pid_p[k], pid_i[k], pid_d[k]);
                cprint("  ** SUPPORT THE ARM during this step! **\r\n");
                uint8_t ok = SERVO_ApplyPidToEeprom(k, pid_p[k], pid_i[k], pid_d[k]);
                uint8_t rp = 0, rd = 0, ri = 0;
                if (SERVO_ReadPidEeprom(k, &rp, &rd, &ri)) {
                    cprint("  J%d %s. EEPROM=P:%d D:%d I:%d\r\n", j,
                           ok ? "OK" : "MISMATCH", rp, rd, ri);
                } else {
                    /* 回读不通: 可能是真离线, 也可能是舵机处在告警态
                     * (DXL_Read 遇到任何告警位都返回错误) —— 读 HW_ERROR 区分 */
                    uint8_t hw = SERVO_GetHardwareError(k);
                    cprint("  J%d %s. EEPROM readback failed [%s]\r\n", j,
                           ok ? "burned(unverified)" : "FAILED",
                           hw == 0xFF ? "no reply" : SERVO_ErrorString(hw));
                    if (hw != 0xFF && hw != 0U)
                        cprint("    servo is ALERTING but reachable -> check 'E %d'\r\n", j);
                }
                if (ok) {
                    cprint("  Use 'F' to sync the STM32-side flash backup.\r\n");
                }
            }
        } else {
            cprint("  Usage: Y J  J=1~6 (or 0 = all six)\r\n");
        }
        break;
    }

    case 'U': {
        /* U J [A] [val]: 舵机寄存器读/写。
         * U J          = 回读 PUNCH(24); U J val = 写 PUNCH 并回读 (兼容旧用法)
         * U J A        = 回读任意 8bit 寄存器 A (如 U 3 48 看 Torque Limit)
         * U J A val    = 写寄存器 A = val 并回读验证
         * 用途: PUNCH(24) 对抗低速粘滑; 调试 teach 限矩(48)/Max Torque(16) 等。 */
        if (ntok >= 2 && ntok <= 4) {
            int j = atoi(tokens[1]);
            if (j >= 1 && j <= 6) {
                uint8_t k = (uint8_t)(j - 1);
                uint16_t addr = STS_ADDR_PUNCH;
                if (ntok >= 3) addr = (uint16_t)atoi(tokens[2]);
                if (ntok == 2 || ntok == 4) {
                    uint8_t val = (ntok == 4) ? (uint8_t)atoi(tokens[3])
                                              : (uint8_t)atoi(tokens[2]);
                    SERVO_WriteSimple(JOINT_BUS[k], JOINT_ID[k], addr, &val, 1);
                    cprint("  J%d write addr=%d val=%d\r\n", j, addr, val);
                }
                uint8_t rb = 0;
                if (SERVO_ReadReg(k, addr, &rb))
                    cprint("  J%d addr %d = %d\r\n", j, addr, rb);
                else
                    cprint("  J%d addr %d read FAILED (offline?)\r\n", j, addr);
            } else {
                cprint("  Usage: U J [A] [val]  J=1~6, A=register addr, val=0~254\r\n");
            }
        } else {
            cprint("  Usage: U J [A] [val]  (register read/write; A缺省=24 PUNCH)\r\n");
        }
        break;
    }

    case 'R': {
        /* `R 1` = 广播 reboot 两条总线上的所有舵机 (尝试恢复卡死/报错的舵机) */
        if (ntok >= 2 && tokens[1][0] == '1') {
            cprint("  Rebooting ALL servos (both buses) ...\r\n");
            SERVO_RebootAll();
            cprint("  Done. Rescan with 'n'.\r\n");
            break;
        }
        /* Reset adaptive state & re-trigger correction to last targets */
        correction_count = 0;
        for (uint8_t j = 0; j < 6; j++) {
            corr_gain[j] = corr_gain_init[j];
            prev_error[j] = 0.0f;
            oscillation_count[j] = 0;
            stall_count[j] = 0;
            correction_offset[j] = 0.0f;
        }
        if (!Trajectory_IsActive() && !swing_active) {
            correction_pending = 1;
            correction_timer = HAL_GetTick();
            cprint("  Adaptive state reset. Correction re-triggered.\r\n");
            cprint("  Targets: ");
            for (uint8_t j = 0; j < 6; j++)
                cprint("J%d=%d ", j+1, (int)correction_target[j]);
            cprint("\r\n");
        } else {
            cprint("  Adaptive state reset. (busy, will trigger after move)\r\n");
        }
        break;
    }

    default:
        cprint("  Unknown command '%c'. Type 'h' for help.\r\n", cmd);
        break;
    }
}

/* ============================================================
 * 网口文本命令透传 (2026-09-23)
 * ------------------------------------------------------------
 * 目标: 让 UDP 能下发 COM11 控制台上的**全部**命令, 而不是逐条加二进制
 *       opcode。理由有三:
 *   ① ProcessCommand 里有 124 条命令, 逐条加 opcode 永远追不上以后新增的;
 *      透传则"控制台加了什么, 网口立刻就能用什么"。
 *   ② 语义零漂移 —— 走的是**同一条** ProcessCommand, 连提示文本都一样。
 *   ③ 11 条抢 ETH 引脚的命令 (cx/cp/cy/cz/cw/ps/can/mg/mp/mo/ml) 的安全
 *      拦截**天然继承**: 那段 #if ARM_LINK_NET 判断就在 ProcessCommand 开头,
 *      不用再抄一份清单(抄两份 = 迟早有一份漂移)。
 * 代价: 输出上限 2048 B(超出截断并置标记); 交互式命令必须挡 (见 TextGate)。
 * ============================================================ */

/* 安全闸门。返回 0 = 放行; 1 = 交互/阻塞类(会永久卡死无看门狗的主循环);
 * 2 = 急停中只允许 s / h。
 * ⚠ 这里**故意**不重复列 11 条抢引脚命令, 理由见上面 ③。 */
uint8_t ArmMain_TextGate(const char* line) {
    if (line == NULL || line[0] == '\0' || line[0] == ' ') return 1U;
    if (line[0] == '#') return 1U;     /* `#P` 是主从位置流帧, 不是人敲的命令 */

    if (IsEStop()) {
        /* 与主循环里的急停白名单保持一致: 只有 s / h 放行 */
        if (!((line[0] == 's' || line[0] == 'h') && line[1] == '\0')) return 2U;
    }

    /* 交互式/阻塞命令黑名单。目前只有一条, 但它是**真会打死板子**的:
     * `c`(单独一个字符) 进 cmdCalibrate(), 里面是 `for(;;)` 等人在**本机串口**
     * 上敲 '0'/'s'/'q' —— 从网口下发它, 主循环就再也回不来了, 而本工程
     * **没有看门狗**(红线 5), 现象是"板子彻底失联但 USB 还在枚举"。
     * ⇒ 相位校准向导**设计上就是控制台专属**(它必须配合人手摆位)。 */
    if (line[0] == 'c' && line[1] == '\0') return 1U;

    return 0U;
}

uint16_t ArmMain_ExecText(char* line, int len, uint8_t* truncated) {
    g_text_len   = 0U;
    g_text_trunc = 0U;
    con_sink = con_capture;                       /* 回显改道到内存 */
    main_progress(ARM_PHASE_CMD, (uint32_t)(uint8_t)line[0]);
    ProcessCommand(line, len);
    main_progress(ARM_PHASE_IDLE, 0U);
    con_sink = con_write_uart;                    /* 立刻还回真串口 */
    if (truncated) *truncated = g_text_trunc;
    return g_text_len;
}

uint16_t ArmMain_TextLength(void) { return g_text_len; }

uint8_t ArmMain_TextRead(uint16_t off, uint8_t n, char* out) {
    if (out == NULL || n == 0U || off >= g_text_len) return 0U;
    uint16_t avail = (uint16_t)(g_text_len - off);
    uint16_t k = (n < avail) ? (uint16_t)n : avail;
    memcpy(out, &g_text_buf[off], k);
    return (uint8_t)k;
}

/* ---- 软复位 (网口 SYS_RESET) ----
 * 必须"先回包、后复位": 在 Dispatch 里直接 NVIC_SystemReset() 的话, ACK 还没
 * 发出去板子就重启了, 上位机只会看到超时, 分不清"复位成功"与"包丢了"。
 * 所以只登记时间戳, 由主循环在 150 ms 后执行 —— 那时 UDP 应答早已发完。
 * 典型用途: `link can` → `link save` → SYS_RESET, 远程切换链路。 */
static uint8_t  g_reset_armed = 0U;
static uint32_t g_reset_at_ms = 0U;

void ArmMain_RequestReset(uint16_t delay_ms) {
    g_reset_at_ms = HAL_GetTick() + (uint32_t)delay_ms;
    g_reset_armed = 1U;
}

void ArmMain_ServicePendingReset(void) {
    if (g_reset_armed && (int32_t)(HAL_GetTick() - g_reset_at_ms) >= 0) {
        g_reset_armed = 0U;
        cprint("\r\n  [SYS] soft reset requested over link -> rebooting now\r\n");
        HAL_Delay(20);
        NVIC_SystemReset();
    }
}

/* ===== Console ready flag (for Error_Handler) ===== */
static uint8_t console_ready = 0;
/* 0924AT: 开机日志降噪开关 (默认安静)。0=只打重要的(横幅/故障诊断/扫描结果/
 * 警告/链路状态/偏好恢复), 1=恢复全套 "Init ..." 过程行。运行期 `bootlog` 切换,
 * RAM-only 不存 flash —— 每次开机默认安静。 */
static uint8_t boot_verbose = 0U;

/* ===== Main ===== */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    /* 链路偏好: **必须早于 MX_GPIO_Init()** —— gpio.c 要按它决定 PC4/PC5
     * 是配成 LED 还是整段让位给 RMII 的 AF11。
     * 本调用只读 Flash, 不写、不打日志(RunLog_Init() 还没跑)。
     * 未配置 / 校验失败 ⇒ 默认 AUTO(0924CF 起, 用户 2026-11-10 明令, 覆盖 09-22 全关):
     * 开机即 CAN+网口双监听, 免人工切换; 显式关闭走 `link off`+`link save`。 */
    LinkCfg_Init();

    MX_GPIO_Init();

    /* 0924CM: 电源灯常亮 (gpio.c 已置高, 此处显式兜底) */
    LED_Power(1);

    /* Buzzer = "code is running" signal (no LEDs needed) */
    BuzzerBeep(50);
    HAL_Delay(50);
    BuzzerBeep(50);

    /* Console FIRST - so we can see messages even if later init fails */
    Console_Init();
    console_ready = 1;

    /* 0924CM: g_boot_tick 保留 (时序基准); 0924CZ: 开机卡死慢闪提示已按用户
     * 要求删除 —— 报警灯只报软件急停(S)与总线丢读。 */
    g_boot_tick = HAL_GetTick();

    cprint("\r\n");
    cprint("  +=====================================+\r\n");
    cprint("  |  6-Axis Arm Controller v2.0        |\r\n");
    cprint("  |  STM32F407 + STS3215 x6             |\r\n");
    cprint("  +=====================================+\r\n\r\n");

    if (boot_verbose) cprint("  Console OK (USART6 @ 115200)\r\n");

    /* ===== "上一次运行是怎么死的?" (2026-09-18) =====
     * 本工程**没有看门狗**, 所以「CPU 撞异常」「主循环卡在无界等待」「掉电」这
     * 三种死在现场的表象完全一样: 口还在枚举、0 字节输出、bootloader 也不应答。
     * 这两条记录放在 CCM RAM(跨复位保留、跨掉电消失), 于是三种结论互斥且完备:
     *   有 [FAULT]  ⇒ CPU 撞异常(被 arm_fault 的 while(1) 兜住)
     *   有 [STALL]  ⇒ 主循环卡死, phase/cmd 指出卡在哪
     *   两条都没有  ⇒ 上一次是掉电/断电
     * ⚠ 想拿到记录, 故障后要**按 RESET(不是断电)** —— 断电会把 CCM RAM 一起清掉。 */
    {
        ArmFaultRec fr;
        ArmStallRec sr;
        ArmFault_Read(&fr);
        ArmStall_Read(&sr);
        if (fr.code != ARM_FAULT_NONE) {
            char dec[128];
            ArmFault_DecodeCfsr(fr.cfsr, dec, sizeof(dec));
            cprint("  [FAULT] 上次运行死于 **%s**  IPSR=%lu EXC_RET=%08lX\r\n",
                   ArmFault_Name(fr.code), (unsigned long)fr.ipsr,
                   (unsigned long)fr.exc_return);
            cprint("          CFSR=%08lX(%s)  HFSR=%08lX\r\n",
                   (unsigned long)fr.cfsr, dec, (unsigned long)fr.hfsr);
            cprint("          BFAR=%08lX MMFAR=%08lX  => %s\r\n",
                   (unsigned long)fr.bfar, (unsigned long)fr.mmfar,
                   ArmFault_AddrHint(fr.bfar ? fr.bfar : fr.mmfar));
        }
        if (sr.magic == ARM_STALL_MAGIC) {
            if (sr.cmd != 0U) {
                cprint("  [STALL] 上次运行**主循环卡死** 至少 %lu ms: phase=%lu 命令首字符='%c'\r\n",
                       (unsigned long)sr.stall_ms, (unsigned long)sr.phase, (char)sr.cmd);
            } else {
                cprint("  [STALL] 上次运行**主循环卡死** 至少 %lu ms: phase=%lu\r\n",
                       (unsigned long)sr.stall_ms, (unsigned long)sr.phase);
            }
        }
        /* 0924DE: "上次运行无故障"提示行删除(用户定案) —— [FAULT]/[STALL] 有事才说话 */
        ArmFault_Clear();
        ArmStall_Clear();
    }


    /* v2.1: 运行日志（RUN-START 头，PC 工具据此归档每次运行） */
    RunLog_Init();

    if (boot_verbose) cprint("  Init TIM2...\r\n");
    MX_TIM2_Init();

    /* RC 遥控输入层 (2026-09-24 起: PS2 手柄, 硬件 SPI1: DAT=PB4 CMD=PB5
     * CLK=PB3 CS=PB1)。放在 TIM2 之后、舵机自举之前: 只配 GPIO+手柄配置
     * 序列, 不碰任何舵机总线。失败也不致命(只读功能): inited=0 用 `rc` 看。 */
    if (boot_verbose) cprint("  Init RC input (PS2, hardware SPI1 PB1/3/4/5)...\r\n");
    ArmRc_Init();

    if (boot_verbose) cprint("  Init USART3 (Bus A)...\r\n");
    MX_USART3_UART_Init();
#if ARM_MASTER_BUILD
    /* 主臂舵机总线已在 USART3, UART5 不再接舵机。仍然 init(句柄留给
     * servo_bus_hal 的 BUS_B 分支兜底, 避免误用未初始化句柄), 但明确标注 bypass。 */
    if (boot_verbose) cprint("  Init UART5 (Bus B, bypassed - servos on USART3)...\r\n");
#else
    if (boot_verbose) cprint("  Init UART5 (Bus B)...\r\n");
#endif
    MX_UART5_Init();
    /* 主从臂链路 (UART4: PA0=TX / PA1=RX @115200)。
     * 从臂侧这条口只收主臂板打来的 0x13 流帧, 纯二进制无文本混流,
     * 故 ISR 直接喂协议状态机, 不需要 USART6 那种 0xAA/文本分流。 */
#if ARM_LINK_MULTI
    /* both 构建: 由**开机引脚形态**决定, 逻辑与下面两个编译期分支逐字对应。
     * ⚠ 0924CE: 判据用 LinkCfg_BootNetPins() —— auto 下 PA1 = ETH_REF_CLK,
     *   UART4 必须跳过(与 NET 相同); 用 LINK_IS_NET() 判会在 auto 开机时
     *   (动态值=OFF) 误初始化 UART4 ⇒ PA0/PA1 被配成 AF8 掐死 ETH。
     * ⚠ 走到 else(CAN/OFF)时都是"初始化外设但**不使能 RX 中断**": PA1 在
     *   usart.c 里配的是 GPIO_NOPULL, 线一悬空就是"字节制造机", 那些垃圾字节
     *   会被 LinkProto_FeedByte() 组装起来污染诊断(LINK=can 构建踩过的坑)。 */
    if (LinkCfg_BootNetPins()) {
        if (boot_verbose) cprint("  Init UART4 (skipped - PA1 is ETH_REF_CLK in NET/AUTO mode)...\r\n");
    } else {
        if (boot_verbose) cprint("  Init UART4 (idle - link is %s)...\r\n",
               LinkCfg_NameOf(LinkCfg_Get()->mode));
        MX_UART4_Init();
        if (boot_verbose) cprint("  Init done.\r\n");
    }
#elif ARM_LINK_NET
    /* net 构建: PA1 = ETH_REF_CLK, UART4(PA0/PA1) 不能再占脚 ⇒ 整个跳过。
     * 主从链路在 net 构建下不可用(要恢复请用 LINK=uart / LINK=can 构建)。 */
    if (boot_verbose) cprint("  Init UART4 (skipped - PA1 is ETH_REF_CLK in NET build)...\r\n");
#elif ARM_LINK_CAN
    /* 链路改走 CAN: UART4 外设仍初始化(随时可切回), 但**不使能 RX 中断**。
     * 关键理由: PA1(UART4_RX) 在 usart.c 里配的是 GPIO_NOPULL, 一旦线悬空
     * 就是"字节制造机"; 那些垃圾字节会被 LinkProto_FeedByte() 组装, 与 CAN
     * 收到的帧混在一起, 让诊断再次变成污染测量。 */
    if (boot_verbose) cprint("  Init UART4 (idle - link is on CAN)...\r\n");
    MX_UART4_Init();
    if (boot_verbose) cprint("  Init done.\r\n");
#else
    if (boot_verbose) cprint("  Init UART4 (master-slave link)...\r\n");
    MX_UART4_Init();
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);   /* 与控制台同级, 低于 TIM2 */
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
    if (boot_verbose) cprint("  Init done.\r\n");
#endif

    HAL_TIM_Base_Start_IT(&htim2);

    /* Scan both buses.
     * 舵机上电自举需要时间: 12V 轨带 6 颗舵机 + 每颗 100µF 大电容,
     * 电压爬升慢, STS3215 内部复位完成后才应答 Ping。
     * 立即扫描 = "刚上电有时认得到有时认不到"。
     * 若首轮不全(<6), 等 500ms 再补扫一轮, 取结果较好的一次。 */
    HAL_Delay(500);
#if !ARM_ONE_BUS_BUILD
    /* ===== 0924BC: 先恢复上次实测确认的路由, 再扫描 =====
     * JOINT_BUS[] 是 RAM, 一复位就回到编译期默认 A/A/A/B/B/B。集成板
     * "六颗全接 B 段"时那个默认本身就是错的 —— 若不在这里灌回来,
     * ScanBothBuses 会拿 A 段去查 J1~J3(必然 0 命中), AutoMap 的 pass1
     * 也要重新赌一次当轮误码运气; 赌输的那一轴就停在空的 A 段, 表现成
     * "上次明明好了, 重启又缺轴", 而且固件不知道自己错过。
     * 恢复后 pass1 直接查正确的段, 命中即返回, 不再依赖运气。 */
    {
        uint8_t restored = SERVO_LoadSavedRoute();
        if (restored) {
            uint8_t nondef = 0U;
            for (uint8_t j = 0U; j < 6U; j++) {
                uint8_t def = (j < 3U) ? BUS_A : BUS_B;
                if (JOINT_BUS[j] != def) nondef = 1U;
            }
            /* 默认布局不吵(安静启动); 非默认布局必须打出来 —— 那是关键状态 */
            if (nondef || boot_verbose) {
                cprint("  [MAP] saved route restored:");
                for (uint8_t j = 0U; j < 6U; j++)
                    cprint(" J%d=%s", (int)(j + 1), (JOINT_BUS[j] == BUS_A) ? "A" : "B");
                cprint("\r\n");
            }
        }

        /* ===== 0924BI: 恢复主臂零点 (fi 遥操反解用, 区 H) =====
         * fi 用 master_zero_raw 反解主臂角度; 上电从 flash 灌回,
         * 免每次重标。记录无效(空区/坏数据)时 fi 回退用从臂零点。 */
        {
            uint16_t z[6];
            if (MstZeroCfg_Load(z)) {
                for (uint8_t j = 0U; j < 6U; j++) master_zero_raw[j] = z[j];
                master_zero_valid = 1U;
                if (boot_verbose) {
                    cprint("  [MAP] master zero restored:");
                    for (uint8_t j = 0U; j < 6U; j++)
                        cprint(" %u", (unsigned)master_zero_raw[j]);
                    cprint("\r\n");
                }
            }
        }
    }
#endif
#if ARM_MASTER_BUILD
    /* ===== 主臂: 舵机总线固定映射 (0924U: UART5 疑似失效, 改走 USART3) =====
     * 去掉 74HC126D 后, 6 颗舵机全部并联在一条"开漏单总线"上。
     * 原设计物理接在 UART5 (PC12/PD2) = Bus B; 现因 UART5 失效改接到
     * USART3 (PB10/PB11) = Bus A。UART4(PA0/PA1) 主从链路保持不动。
     * 注意: 硬件上需把 PB10/PB11 飞线到舵机 DATA 座子(连同原总线上的上拉)。 */
    SERVO_SetBusMode(1);
    cprint("  MASTER: servo bus = USART3 (PB10/PB11) single-wire, bus mode 1 fixed (UART5 bypassed).\r\n");
#elif defined(BUS_ONE) && BUS_ONE
    /* ===== 集成板: 74HC126 拆不掉, 改把 1Y 与 3Y 短接成一条总线 =====
     * 一片 126 用两路门: 门1(1OE=PA8) 驱动 Bus A, 门3(3OE=PA10) 驱动 Bus B,
     * 门2/门4 的 OE 与 A 都接地闲置。硬件把 1Y+3Y 短接后, 两路输出并成
     * 一条物理总线, 6 颗舵机全挂在上面。
     *
     * 电气前提(必须由固件保证, 否则两路推挽对撞):
     *   DIR_A 永久保持低电平 -> 门1 永久高阻 -> 只有门3 在驱动总线。
     *   本构建在扫描之前钉死 bus mode 2, 之后所有事务只走 BUS_B,
     *   即 DXL_SetDirection 只会动 DIR_B, DIR_A 停留在 gpio.c 的初值 0。
     * ⚠ 因此本固件**禁止**再执行 `b 0` / `b 1`(见 b 命令守卫),
     *   也**不能**换回双总线固件 —— 那样 DIR_A 会被拉高, 两路门同驱一条线。 */
    SERVO_SetBusMode(2);
    cprint("  BUS_ONE: 6 servos on UART5 (1Y+3Y shorted), DIR_A held LOW, mode 2 fixed.\r\n");
#endif
    /* 0924BJ: 上电波特率自适配 —— 1M 全静默则自动试 500k (sid baud 降速后
     * 重启免重烧; 舵机全断电时保持 1M 原状)。 */
    BusBaudAutodetect();
    if (boot_verbose) cprint("  Scanning buses...\r\n");
    ScanBothBuses();
    /* 最多 3 轮扫描，取最好结果。12V 带 6 颗舵机电压爬升慢，
     * STS3215 内部复位+初始化可能需要 >1s 才稳定应答。 */
    for (uint8_t retry = 1; retry <= 2 && (count_a + count_b) < 6; retry++) {
        uint8_t first_a = count_a, first_b = count_b;
        HAL_Delay(800);
        cprint("  Rescan (round %d, prev %d/6)...\r\n", retry, first_a + first_b);
        ScanBothBuses();
        if (count_a + count_b < (uint8_t)(first_a + first_b)) {
            count_a = first_a;
            count_b = first_b;
        }
    }
    /* 0924BD: 计数打印移到 AutoMap **之后** —— AutoMap 可能已经改了路由,
     * 先打印就会打出"改之前"的旧账(现场: 六颗全在 B, 却打 Bus B: 3)。 */

#if !ARM_ONE_BUS_BUILD
    /* ===== 逐 ID 自动路由 (2026-11-02, 0924BA) =====
     * 不改任何接线的前提下, 让 6 颗舵机插在哪个座子上都能用: 按实测应答
     * 重建"关节 -> 总线"映射, 取代编译期写死的 A/A/A/B/B/B。物理上仍是两条
     * 独立总线(门1/门3 各自驱动), 但软件把它们当成"一条按 ID 寻址的逻辑总线",
     * 两条 UART 继续并行 -> 性能与双总线版完全相同。
     *
     * 布局正确时只多 6 次 PING(pass1 直接返回), 启动时间不受影响;
     * 只有存在缺失/错插时才做完整的逐 ID 双总线探测。
     * 主臂/BUS_ONE 构建跳过: 那两种形态只有一条物理总线, 映射已钉死为 mode 2。 */
    {
        uint8_t miss = 0U, dup = 0U, moved = 0U;
        uint8_t online = SERVO_AutoMapBuses(&miss, &dup, &moved);

        if (moved || dup || (online < 6U)) {
            uint8_t reroute = 0U;
            cprint("  [MAP] probe: online %u/6 ->", (unsigned)online);
            for (uint8_t j = 0U; j < 6U; j++) {
                uint8_t def = (j < 3U) ? BUS_A : BUS_B;
                if (JOINT_BUS[j] != def) reroute++;
                cprint(" J%d:%s%s%s", (int)(j + 1),
                       (JOINT_BUS[j] == BUS_A) ? "A" : "B",
                       (miss & (uint8_t)(1U << j)) ? "!" : "",
                       (dup  & (uint8_t)(1U << j)) ? "?" : "");
            }
            cprint("\r\n         (! = not found on either bus, ? = same ID on BOTH buses)\r\n");
            if (reroute)
                cprint("  [MAP] %u axis re-routed by real reply (socket layout differs from default)\r\n",
                       (unsigned)reroute);
            if (dup)
                cprint("  [MAP] DUP ID: fix with 'sid set <old> <new>' (two servos share an ID)\r\n");
        } else if (boot_verbose) {
            cprint("  [MAP] layout OK (J1-J3=Bus A, J4-J6=Bus B), no re-route needed.\r\n");
        }

        /* 用实测结论重算"每段在线几颗": 不再拿编译期固定 ID 区间去数,
         * 而是按 AutoMap 定下的路由 + 实测离线位统计。 */
        count_a = 0U;
        count_b = 0U;
        for (uint8_t j = 0U; j < 6U; j++) {
            if (miss & (uint8_t)(1U << j)) continue;      /* 两条线都不应答 */
            if (JOINT_BUS[j] == BUS_A) count_a++;
            else                       count_b++;
        }
    }
#endif
#if ARM_ONE_BUS_BUILD
    /* 单总线: 另一条线没接舵机, 报它就等于给现场制造"还有一条总线"的错觉。
     * 主臂 -> Bus A (USART3); 集成板 -> Bus B (UART5)。 */
    cprint("  Bus %s (%s): %d servo(s)  (other bus not used)\r\n",
           ONE_BUS_TAG, ONE_BUS_PORT, ONE_BUS_CNT);
#else
    /* 0924DE: 本臂 6 轴全在 B 且主臂在线时, "Bus A: 0" 整行去掉 ——
     * 主臂行顶上(用户 2026-11-24 定案); 只在确有本臂关节走 A 段时才打 A/B 两行。 */
    if (count_a == 0U && count_ma > 0U) {
        /* 主臂行顶上: 恒列 ID 清单, 6/6 加 fi ready 标记 */
        cprint("  Bus A master (ID 7-12): %d online:", count_ma);
        for (uint8_t mk = 0U; mk < count_ma; mk++) cprint(" %d", found_ma[mk]);
        cprint("%s\r\n", (count_ma == 6U) ? "  <- fi ready" : "");
    } else {
        cprint("  Bus A: %d servo(s)  Bus B: %d servo(s)\r\n", count_a, count_b);
        /* 0924DA: 主臂在线单独一行 —— fi 板内遥操前提 (Bus A ID 7~12) */
        if (count_ma > 0U) {
            cprint("  Bus A master (ID 7-12): %d online:", count_ma);
            for (uint8_t mk = 0U; mk < count_ma; mk++) cprint(" %d", found_ma[mk]);
            cprint("%s\r\n", (count_ma == 6U) ? "  <- fi ready" : "");
        } else {
            cprint("  Bus A master (ID 7-12): 0 online  (master arm not detected)\r\n");
        }
    }
#endif

    /* Set algorithm defaults first, then let Flash configuration override them. */
    ArmParams_Init();

    /* Load persistent PID and algorithm configuration before servo initialization. */
    pid_load_startup();

    /* Init all found servos */
    if (boot_verbose) cprint("  Initializing servos...\r\n");
    SERVO_InitAll();

    /* 校验舵机模式: Mode 必须=0(位置伺服); Mode=1=恒速模式 -> 会持续自转 */
    {
        uint8_t all_ok = 1;
        for (uint8_t j = 0; j < 6; j++) {
            uint8_t m = SERVO_GetMode(j);
            if (m == 0) continue;
            all_ok = 0;
            if (m == 0xFF)
                cprint("  [WARN] J%d: mode read failed (offline?)\r\n", j + 1);
            else
                cprint("  [WARN] J%d Mode=%d (SPEED/WHEEL MODE! will spin) - EEPROM fix failed\r\n",
                       j + 1, m);
        }
        if (all_ok && boot_verbose) cprint("  All 6 servos in position mode (Mode=0).\r\n");
    }

#if ARM_MASTER_BUILD
    /* ===== 主臂角色: 释放六轴, 人力拖动 =====
     * SERVO_InitAll() 结尾会给每轴上力矩(抱闸), 主臂必须立刻放开, 否则拖不动。
     * 之后运行期不再上力矩 —— 主臂只读位置, 从不下发目标。 */
    cprint("\r\n  *** MASTER ROLE (ARM_MASTER_BUILD) ***\r\n");
    cprint("  Torque OFF x6 -> drag the master arm by hand.\r\n");
    master_release_all();
    cprint("  Servo bus: USART3 (PB10/PB11) single-wire, TX open-drain, no 74HC126 (UART5 bypass).\r\n");
    cprint("  Forwarding 0x13 @%u ms (~%u Hz) on UART4 (PA0/PA1).\r\n",
           (unsigned)MASTER_TX_PERIOD_MS,
           (unsigned)(1000U / MASTER_TX_PERIOD_MS));
#endif

    /* Sync IK joint limits from safety arrays (raw space, wrap-around aware) */
    for (uint8_t j = 0; j < 6; j++) {
        if (JOINT_MIN_RAW[j] == 0 && JOINT_MAX_RAW[j] == 0) {
            JOINT_MIN_RAW[j] = deg_to_servo_raw(JOINT_MIN[j]);
            JOINT_MAX_RAW[j] = deg_to_servo_raw(JOINT_MAX[j]);
        }
    }
    IK_SetJointLimitsRaw(JOINT_MIN_RAW, JOINT_MAX_RAW, SERVO_OFFSET);
    if (boot_verbose) cprint("  IK limits synced (raw).\r\n");

    /* v2.1: 统一参数表 + 二进制控制协议（串口 0xAA 帧 / CAN） */
    SerialProto_Init();
#if ARM_LINK_MULTI
    if (!LINK_IS_NET()) {
        LinkProto_Init();      /* 主从链路口 (UART4): 只回 NACK, 抑制流帧 ACK */
    }
#elif !ARM_LINK_NET
    LinkProto_Init();          /* 主从链路口 (UART4): 只回 NACK, 抑制流帧 ACK */
#endif
    /* CAN 从站 (2026-09-17): 顺序不能换 ——
     *   ① CanCfg_Load()               先把节点号/波特率/接口从 Flash 读出来
     *   ② ArmCanDrv_SetInterface()    选定 CAN1 还是 CAN2 (2026-09-18 起运行时可切)
     *   ③ ArmCanDrv_SetBitrate()      把波特率交给驱动(此时还没 init, 只记录)
     *   ④ CanProto_Init(node)         真正初始化 CAN 控制器与过滤器
     * 反过来的话 CanProto_Init 会先用默认 500k 起一次总线, 再被重配 ——
     * 总线上会出现几百 ms 的"错误波特率"窗口, 上位机那边看到的就是满屏错误帧。
     * ⚠ ②必须早于 ④: 接口定下来之后 MspInit 才会按对的那一对脚去配 GPIO/中断。
     *   接口还没 init 时 SetInterface 只改状态, 不碰硬件, 所以它放在这里安全。 */
#if ARM_LINK_MULTI
    /* both 构建: 由 Flash 里的**链路偏好**决定初始化哪一条。
     * 三条铁律:
     *   ① **只初始化该形态需要的链路**。手动 can/net 只开一条(旧语义);
     *      auto(0924CE) 两条都开 —— 硬件已无冲突(RMII 在 PG 组, PB12/13 是
     *      CAN2), 0924CB 三开构建实测可同时收发。
     *   ② `CanCfg_Load()` **无条件读**(它只读 Flash、不碰硬件), 让 `can` 与
     *      审计在任何模式下都能报出当前 CAN 配置。
     *   ③ **AUTO 是默认态**(2026-11-10 用户明令, 覆盖 09-22 全关): Flash 没写过
     *      ⇒ 双链路监听。显式关 = `link off` + `link save`。
     * ⚠ CAN 分支内 ①②③④ 的顺序不能动(理由见下面 #else 里那段原注释)。
     * ⚠ 判据必须是 LinkCfg_BootMode()(偏好) —— auto 下 ActiveMode() 开机
     *   返回 OFF(动态值), 拿它判会漏掉双初始化。 */
    CanCfg_Load();
    switch (LinkCfg_BootMode()) {
    case LINK_MODE_NET:
        NetProto_Init();
        LinkCfg_MarkNetInited();
        if (boot_verbose) cprint("  Binary proto v%d ready (0xAA 0x55 frame). 'LP' lists params.\r\n",
               (int)APROTO_VERSION);
        cprint("  NET slave: udp://192.168.7.120:5000, RMII+LAN8720A, PHY 探测在主循环进行 ('nx' 看状态)\r\n");
        RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, link=NET (CAN off)",
               count_a, count_b, (int)APROTO_VERSION);
        break;

    case LINK_MODE_CAN:
        (void)ArmCanDrv_SetInterface(CanCfg_IfaceOrDefault(CanCfg_Get()));
        (void)ArmCanDrv_SetBitrate(CanCfg_Get()->baud_kbps);
        CanProto_Init(CanCfg_Get()->node_id);
        LinkCfg_MarkCanInited();
        if (boot_verbose) cprint("  Binary proto v%d ready (0xAA 0x55 frame). 'LP' lists params.\r\n",
               (int)APROTO_VERSION);
        cprint("  CAN slave: %s %s, node=%u @%u kbit/s, ids cmd=0x%03X rsp=0x%03X evt=0x%03X, hb=%ums%s\r\n",
               ArmCanDrv_IfName(), ArmCanDrv_PinDesc(),
               (unsigned)CanCfg_Get()->node_id, (unsigned)CanCfg_Get()->baud_kbps,
               (unsigned)(CAN_PROTO_ID_CMD_BASE + CanCfg_Get()->node_id),
               (unsigned)(CAN_PROTO_ID_RSP_BASE + CanCfg_Get()->node_id),
               (unsigned)(CAN_PROTO_ID_EVT_BASE + CanCfg_Get()->node_id),
               (unsigned)CanCfg_Get()->hb_ms,
               CanCfg_LoadedFromFlash() ? "" : " (defaults)");
        cprint("  CAN cfg: 'can' shows all, 'can node/baud/hb/save' to change, 'cx 1|2' switches CAN1/CAN2.\r\n");
        RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, link=CAN node=%d %uk",
               count_a, count_b, (int)APROTO_VERSION,
               (int)CanCfg_Get()->node_id, (unsigned)CanCfg_Get()->baud_kbps);
        break;

    case LINK_MODE_AUTO:
        /* 0924CE 双链路自动门控: CAN 四步序 + NET 同时初始化。
         * 开机即静默监听(active=OFF), 谁先来合法包谁 active。 */
        (void)ArmCanDrv_SetInterface(CanCfg_IfaceOrDefault(CanCfg_Get()));
        (void)ArmCanDrv_SetBitrate(CanCfg_Get()->baud_kbps);
        CanProto_Init(CanCfg_Get()->node_id);
        LinkCfg_MarkCanInited();
        NetProto_Init();
        LinkCfg_MarkNetInited();
        if (boot_verbose) cprint("  Binary proto v%d ready (0xAA 0x55 frame). 'LP' lists params.\r\n",
               (int)APROTO_VERSION);
        /* 0924DA: 两行并一行, 开机文本瘦身 */
        cprint("  AUTO link: CAN(%s node=%u @%uk) + NET(udp://192.168.7.120:5000) 双监听;\r\n",
               ArmCanDrv_IfName(), (unsigned)CanCfg_Get()->node_id,
               (unsigned)CanCfg_Get()->baud_kbps);
        cprint("             谁发合法包谁 active, %us 无包静默。LED 让位 RMII。\r\n",
               (unsigned)(LINK_AUTO_TIMEOUT_MS / 1000U));
        RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, link=AUTO (dual listen)",
               count_a, count_b, (int)APROTO_VERSION);
        break;

    default:   /* LINK_MODE_OFF —— 只有 Flash 里**显式存过 off** 才会走到这 */
        cprint("  [LINK] 链路已显式关闭(Flash 区C 存过 off) —— 只留串口控制台。\r\n");
        cprint("         开 CAN : `link can`  -> `link save` -> `rb` 软复位\r\n");
        cprint("         开网口 : `link net`  -> `link save` -> `rb` 软复位\r\n");
        cprint("         双自动 : `link auto` -> `link save` -> `rb` (两条都监听, 谁来包开谁)\r\n");
        cprint("         (未配置时默认即 auto —— 见 0924CF)\r\n");
        RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, link=OFF (no CAN, no NET)",
               count_a, count_b, (int)APROTO_VERSION);
        break;
    }
#elif ARM_LINK_NET
    CanCfg_Load();                 /* 仍读节点号(打印/审计用), 但**不碰 CAN 硬件**:
                                      TXD1=PB13 与 CAN2_TX 冲突, LINK=net 里 CAN 必须让位 */
    NetProto_Init();
    if (boot_verbose) cprint("  Binary proto v%d ready (0xAA 0x55 frame). 'LP' lists params.\r\n",
           (int)APROTO_VERSION);
    cprint("  NET slave: udp://192.168.7.120:5000, RMII+LAN8720A, PHY 探测在主循环进行 ('nx' 看状态)\r\n");
    RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, NET build (CAN off)",
           count_a, count_b, (int)APROTO_VERSION);
#else
    /* CAN 从站 (2026-09-17): 顺序不能换 ——
     *   ① CanCfg_Load()               先把节点号/波特率/接口从 Flash 读出来
     *   ② ArmCanDrv_SetInterface()    选定 CAN1 还是 CAN2 (2026-09-18 起运行时可切)
     *   ③ ArmCanDrv_SetBitrate()      把波特率交给驱动(此时还没 init, 只记录)
     *   ④ CanProto_Init(node)         真正初始化 CAN 控制器与过滤器
     * 反过来的话 CanProto_Init 会先用默认 500k 起一次总线, 再被重配 ——
     * 总线上会出现几百 ms 的"错误波特率"窗口, 上位机那边看到的就是满屏错误帧。
     * ⚠ ②必须早于 ④: 接口定下来之后 MspInit 才会按对的那一对脚去配 GPIO/中断。
     *   接口还没 init 时 SetInterface 只改状态, 不碰硬件, 所以它放在这里安全。 */
    CanCfg_Load();
    (void)ArmCanDrv_SetInterface(CanCfg_IfaceOrDefault(CanCfg_Get()));
    (void)ArmCanDrv_SetBitrate(CanCfg_Get()->baud_kbps);
    CanProto_Init(CanCfg_Get()->node_id);
    if (boot_verbose) cprint("  Binary proto v%d ready (0xAA 0x55 frame). 'LP' lists params.\r\n",
           (int)APROTO_VERSION);
    cprint("  CAN slave: %s %s, node=%u @%u kbit/s, ids cmd=0x%03X rsp=0x%03X evt=0x%03X, hb=%ums%s\r\n",
           ArmCanDrv_IfName(), ArmCanDrv_PinDesc(),
           (unsigned)CanCfg_Get()->node_id, (unsigned)CanCfg_Get()->baud_kbps,
           (unsigned)(CAN_PROTO_ID_CMD_BASE + CanCfg_Get()->node_id),
           (unsigned)(CAN_PROTO_ID_RSP_BASE + CanCfg_Get()->node_id),
           (unsigned)(CAN_PROTO_ID_EVT_BASE + CanCfg_Get()->node_id),
           (unsigned)CanCfg_Get()->hb_ms,
           CanCfg_LoadedFromFlash() ? "" : " (defaults)");
    cprint("  CAN cfg: 'can' shows all, 'can node/baud/hb/save' to change, 'cx 1|2' switches CAN1/CAN2.\r\n");
    RLOG_I("SYS", "boot ok: busA=%d busB=%d servos, proto v%d, can node=%d %uk",
           count_a, count_b, (int)APROTO_VERSION,
           (int)CanCfg_Get()->node_id, (unsigned)CanCfg_Get()->baud_kbps);
#endif

    /* 命名示教文件存储（Flash） */
    (void)TeachStore_Init();
    if (boot_verbose) cprint("  Teach store: %lu saved file(s), %lu samples (~%lu s).\r\n",
           (unsigned long)TeachStore_Count(),
           (unsigned long)TeachStore_TotalSamples(),
           (unsigned long)(TeachStore_TotalSamples() * TEACH_REC_MS / 1000U));

    /* 指令速查清单(arm_commands.c)—— 引用 cmd_registry 让数组不被链接器删除 */
    if (boot_verbose) cprint("  Cmd registry: %u entries (first: \"%s\").\r\n",
           (unsigned)cmd_registry_count,
           cmd_registry[0].name ? cmd_registry[0].name : "?");

    if (boot_verbose) cprint("  Done. Enter commands ('h' = help).\r\n\r\n");

    /* Show current joint limits */
    if (boot_verbose) cprint("  Limits: J1[%d~%d] J2[%d~%d] J3[%d~%d] J4[%d~%d] J5[%d~%d] J6[%d~%d]\r\n",
           (int)JOINT_MIN[0],(int)JOINT_MAX[0], (int)JOINT_MIN[1],(int)JOINT_MAX[1],
           (int)JOINT_MIN[2],(int)JOINT_MAX[2], (int)JOINT_MIN[3],(int)JOINT_MAX[3],
           (int)JOINT_MIN[4],(int)JOINT_MAX[4], (int)JOINT_MIN[5],(int)JOINT_MAX[5]);

    /* Check E-Stop status before entering main loop */
    if (IsEStop()) {
        cprint("  *** WARNING: E-Stop active (PE11=LOW)! ***\r\n");
        cprint("  Commands blocked. Connect PE11 to 3.3V or disable E-Stop.\r\n\r\n");
    }

    /* Ready indicator */
    LED_G(1);
    BuzzerBeep(80);

    cprint("> ");

    /* Main command loop */
    while (1) {
        /* 每转一圈报一次进度 —— TIM2 中断据此判断主循环有没有卡死(见 arm_fault.h)。 */
        main_progress(ARM_PHASE_IDLE, 0U);

        /* ===== Trajectory batch update at 50Hz (200Hz S-curve timebase) ===== */
        if (traj_do_update) {
            traj_do_update = 0;
            main_progress(ARM_PHASE_TRAJ, 0U);

            /* 0924CM: 四灯服务 —— 运行灯 2Hz 翻转 / 报警灯 E-STOP / 通信灯 1s 窗。
             * 放在 50Hz 拍上: 主循环卡死时运行灯冻结(PE5), 与 TIM2 侧 PC4 心跳互补。 */
            Led_Service();

            /* RC 输入层的 PS2 轮询(只读采集 + 通道合成, ~0.9ms 阻塞)。
             * 挂在 50Hz 拍上: 一拍一次正好 50Hz 刷新。
             * ⚠ 只能放这里 —— 不能在 TIM2 ISR 里做, 因为 R1 起的控制
             *   要读舵机(阻塞式总线 I/O), 进 ISR 会把整个节拍拖死。 */
            ArmRc_Poll();

            /* PS2 按键 -> 功能分发 (keysw 可视化/中位/折叠/示教录/回放)。
             * ⚠ 必须在 Poll 之后(取新边沿)、Ctrl_Tick 之前(触发的动作
             *   会解除 RC / 规划轨迹, Tick 的互斥检查要看到最新状态)。 */
            rc_button_service();

            /* R1: RC -> 末端速度流。内部自带使能闸(`rc go on`)与全套
             * 安全闸(E-Stop/新鲜度/互斥/sf0/心跳/拒帧熔断), 未武装时
             * 是一次廉价的状态查询, 见 arm_rc_ctrl.c。 */
            ArmRcCtrl_Tick();

            /* Teach auto-record / replay stream advance once per 50 Hz tick. */
            if (teach_play_state == TEACH_PLAY_STREAM) {
                teach_stream_tick();
            } else if (teach_mode_active) {
                teach_record_tick();
            }
            if (Trajectory_IsActive()) {
                float angles[6];
                Trajectory_Interpolate(traj_tick, angles);

                /* 注: 重力前馈已按需求移除 —— 重力补偿只用于示教悬浮,
                 * 不参与任何轨迹/QL/QLC/B/回放等运动算法。仅保留 J2 限位
                 * 安全钳制。 */
                uint16_t raw1 = deg_to_servo_raw(angles[1]);
                if (!raw_in_range(raw1, JOINT_MIN_RAW[1], JOINT_MAX_RAW[1])) {
                    uint16_t clamped1 = clamp_raw(raw1, JOINT_MIN_RAW[1], JOINT_MAX_RAW[1]);
                    angles[1] = servo_raw_to_deg(clamped1);
                }

                /* 规划器按时间推进目标, 不等舵机实际位置: STS3215 扭矩余量
                 * 足够, 落后量由舵机内部位置环自行补齐, 曲线结束后由 settle
                 * 阶段确认到位 (跟踪保护已随 XL320 时代移除)。 */
                traj_tick += TRAJ_SEND_PERIOD_TICKS;

                /* 0924AP: 移除到位收尾降速与尾段交接, 直接按规划目标下发。
                 * 舵机内部加速度已提高到 50, 能跟上 25~35°/s 规划, 无需再限制。 */
                if (diag_plan_enable) {
                    cprint("PLAN %lu %d %d %d %d %d %d\r\n",
                           (unsigned long)traj_tick,
                           (int)angles[0], (int)angles[1], (int)angles[2],
                           (int)angles[3], (int)angles[4], (int)angles[5]);
                    cprint("  TICK%lu ANGLES: %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
                           (unsigned long)traj_tick,
                           angles[0], angles[1], angles[2], angles[3], angles[4],
                           angles[5]);
                }
                (void)SERVO_SetPositionsSync(angles);

                if (!Trajectory_IsActive()) {
                    traj_tick = 0;
                    /* 0924AP: 曲线完成后直接等 settle, 不再保持低速。 */

                    /* Multi-point transitions are serviced below every main-loop pass. */
                    if (multi_point_legs_left != 0U) {
                        cmdMultiPointService();
                    } else if (qlc_active) {
                        /* QLC 连续循环: 轨迹一结束立刻接下一段, 不停顿 */
                        cmdQLCService();
                    } else if (point_loop_active) {
                        if (point_loop_legs_left != 0xFFFFU) point_loop_legs_left--;
                        if (point_loop_legs_left == 0U) {
                            point_loop_active = 0;
                            cprint("  Point loop done (returned to A).\r\n> ");
                        } else {
                            point_loop_phase = !point_loop_phase;
                            const float* next = point_loop_phase ? point_loop_b : point_loop_a;
                            const float* prev = point_loop_phase ? point_loop_a : point_loop_b;
                            float cur[6];
                            for (uint8_t j = 0; j < 6; j++) {
                                cur[j] = SERVO_GetPosition(j);
                                if (cur[j] < 0.0f) cur[j] = SERVO_GetPosition(j);   /* 重试 */
                                if (cur[j] < 0.0f) cur[j] = prev[j];  /* 上一点兜底, 防爆冲 */
                            }
                            Trajectory_Plan(cur, (float*)next,
                                            point_loop_speed, point_loop_speed * 2.0f);
                            traj_tick = 0;
                        }
                    /* Swing mode: auto-plan next segment */
                    } else if (swing_active) {
                        if (swing_remaining > 0) {
                            swing_remaining--;
                            swing_phase = !swing_phase;
                            const float* next = swing_phase ? POSE_WAVE2 : POSE_WAVE1;
                            const float* prev = swing_phase ? POSE_WAVE1 : POSE_WAVE2;
                            float cur[6];
                            for (uint8_t j = 0; j < 6; j++) {
                                cur[j] = SERVO_GetPosition(j);
                                if (cur[j] < 0.0f) cur[j] = SERVO_GetPosition(j);   /* 重试 */
                                if (cur[j] < 0.0f) cur[j] = prev[j];  /* 上一个摆动位兜底 */
                            }
                            Trajectory_Plan(cur, (float*)next, 30.0f, 60.0f);
                            traj_tick = 0;
                        } else {
                            /* All swings done, return to READY */
                            swing_active = 0;
                            const float* last_pose = swing_phase ? POSE_WAVE2 : POSE_WAVE1;
                            float cur[6];
                            for (uint8_t j = 0; j < 6; j++) {
                                cur[j] = SERVO_GetPosition(j);
                                if (cur[j] < 0.0f) cur[j] = SERVO_GetPosition(j);   /* 重试 */
                                if (cur[j] < 0.0f) cur[j] = last_pose[j];
                            }
                            Trajectory_Plan(cur, (float*)POSE_READY, 30.0f, 60.0f);
                            traj_tick = 0;
                            cprint("  Swing done. Returning to READY.\r\n> ");
                        }
                    } else if (teach_play_state == TEACH_PLAY_RAMP) {
                        /* Teach replay: the return-to-start leg finished; begin
                         * streaming the recorded samples in real time. */
                        teach_ramp_done();
                    } else {
                        /* Time curve ended. Do not claim completion yet: wait for
                         * all Present Position values to settle at the final target. */
                        traj_settle_pending = 1U;
                        traj_settle_joint = 0U;
                        traj_settle_mask = 0U;
                        traj_settle_pass = 1U;
                        traj_settle_stable_since = 0U;
                        traj_settle_adaptive = 0U;
                        traj_settle_next_poll = HAL_GetTick();
                        traj_settle_deadline = HAL_GetTick() + 5000U;  /* ⭐0924AH: 5s 超时兜底 */
                        cprint("  Curve complete; waiting for actual feedback...\r\n");
                    }
                }
            }
        }

        /* ===== 单总线遥测采样: 必须放在轨迹批量下发【之后】 =====
         * 每个 20ms 节拍只读 1 个关节(轮转), 6 拍刷完 6 轴。
         * 放在这里而不是前面, 是为了保证 6 轴 GoalPos 同步写不被读事务
         * 劈成两半 —— 否则各轴起跑时刻不同, 机械上是歪的。 */
        FeedbackSampleService();

        /* Keep polling real servo feedback after an S-curve finishes.
         * This lets multi-point motion leave a loaded point only after it is reached. */
        cmdMultiPointService();
        cmdTrajectorySettleService();
        cmdStreamService();      /* 主从直写看门狗 (断流 -> 冻结退出) */
        cmdFollowInternalService(); /* 板内遥操 (fi 1: 同板 Bus A 读主臂 -> Bus B 驱从臂) */

        /* ===== Post-move adaptive gravity correction for J2/J3/J4 ===== */
        if (correction_pending && !Trajectory_IsActive() && !swing_active) {
            if (HAL_GetTick() - correction_timer > 500) {
                uint8_t any_corrected = 0;
                uint8_t all_in_tol = 1;

                for (uint8_t j = CORR_FIRST_JOINT; j <= CORR_LAST_JOINT; j++) {
                    float actual = SERVO_GetPosition(j);
                    if (actual < 0.0f) {
                        cprint("  J%d read failed, skip\r\n", j+1);
                        continue;
                    }
                    float error = correction_target[j] - actual;

                    /* Corrupted-read filter: a stuck/glitched bus read can produce
                     * a wildly wrong "actual", which would inject a huge offset and
                     * make the joint jump ("猛一下又回调"). If the error changes
                     * abruptly vs the previous round (>8 deg), treat it as a bad
                     * read and skip this round without accumulating offset. */
                    if (prev_error[j] != 0.0f &&
                        fabsf(error) > fabsf(prev_error[j]) + 8.0f) {
                        cprint("  J%d bad-read skip (err jump %d -> %d)\r\n",
                               j+1, (int)prev_error[j], (int)error);
                        continue;
                    }

                    /* Within tolerance? */
                    if (fabsf(error) <= corr_threshold) {
                        cprint("  J%d OK: actual=%d tgt=%d (err=%d)\r\n",
                               j+1, (int)actual, (int)correction_target[j], (int)error);
                        continue;
                    }
                    all_in_tol = 0;

                    /* Max rounds reached? */
                    if (correction_count >= corr_max_rounds) {
                        cprint("  J%d TIMEOUT: actual=%d tgt=%d (err=%d)\r\n",
                               j+1, (int)actual, (int)correction_target[j], (int)error);
                        continue;
                    }

                    /* --- Adaptive gain adjustment --- */
                    if (prev_error[j] != 0.0f) {
                        uint8_t sign_flip = ((error > 0) != (prev_error[j] > 0));
                        if (sign_flip) {
                            /* Overshoot: reduce gain */
                            oscillation_count[j]++;
                            corr_gain[j] *= 0.5f;
                            if (corr_gain[j] < 0.05f) corr_gain[j] = 0.05f;
                            cprint("  J%d OVERSHOOT: gain->%d%% (osc=%d)\r\n",
                                   j+1, (int)(corr_gain[j]*100), oscillation_count[j]);
                            /* Auto-increase D_Gain to damp oscillation */
                            if (oscillation_count[j] >= 2 && pid_d[j] < 32) {
                                pid_d[j] += 4;
                                set_pid_gain(j, STS_ADDR_D_GAIN, pid_d[j]);
                                cprint("  J%d auto-tune: D_Gain->%d (damp oscillation)\r\n",
                                       j+1, pid_d[j]);
                            }
                        } else {
                            /* Same direction: did we make progress? */
                            if (fabsf(error) < fabsf(prev_error[j]) - 0.5f) {
                                /* Progress but not enough: increase gain */
                                corr_gain[j] *= 1.25f;
                                if (corr_gain[j] > 0.85f) corr_gain[j] = 0.85f;
                            } else {
                                /* No progress: stall detected */
                                stall_count[j]++;
                                corr_gain[j] *= 1.4f;
                                if (corr_gain[j] > 0.85f) corr_gain[j] = 0.85f;
                                cprint("  J%d STALL: gain->%d%% (stall=%d)\r\n",
                                       j+1, (int)(corr_gain[j]*100), stall_count[j]);
                                /* Auto-increase P_Gain for more torque */
                                if (stall_count[j] >= 2 && pid_p[j] < 64) {
                                    pid_p[j] += 8;
                                    set_pid_gain(j, STS_ADDR_P_GAIN, pid_p[j]);
                                    cprint("  J%d auto-tune: P_Gain->%d (boost torque)\r\n",
                                           j+1, pid_p[j]);
                                }
                                /* Auto-add I_Gain for steady-state error */
                                if (stall_count[j] >= 3 && pid_i[j] < 16) {
                                    pid_i[j] += 2;
                                    set_pid_gain(j, STS_ADDR_I_GAIN, pid_i[j]);
                                    cprint("  J%d auto-tune: I_Gain->%d (steady error)\r\n",
                                           j+1, pid_i[j]);
                                }
                            }
                        }
                    }

                    /* Apply correction (accumulating offset so the command
                     * never retreats when the error shrinks). Limit the per-round
                     * offset step to avoid a large delayed jump when a big error
                     * appears late (prevents "猛一下又回调"). */
                    float step = error * corr_gain[j];
                    if (step > 3.0f) step = 3.0f;
                    if (step < -3.0f) step = -3.0f;
                    correction_offset[j] += step;
                    if (correction_offset[j] > 30.0f) correction_offset[j] = 30.0f;
                    if (correction_offset[j] < -30.0f) correction_offset[j] = -30.0f;
                    float corrected = correction_target[j] + correction_offset[j];
                    uint16_t c_raw = deg_to_servo_raw(corrected);
                    if (!raw_in_range(c_raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j])) {
                        c_raw = clamp_raw(c_raw, JOINT_MIN_RAW[j], JOINT_MAX_RAW[j]);
                        corrected = servo_raw_to_deg(c_raw);
                    }
                    SERVO_SetPosition(j, corrected);

                    cprint("  J%d corr #%d: actual=%d tgt=%d -> %d (err=%d, gain=%d%% off=%d)\r\n",
                           j+1, correction_count + 1, (int)actual, (int)correction_target[j],
                           (int)corrected, (int)error, (int)(corr_gain[j] * 100),
                           (int)correction_offset[j]);

                    prev_error[j] = error;
                    any_corrected = 1;
                }

                if (any_corrected) {
                    correction_count++;
                    correction_timer = HAL_GetTick();
                } else {
                    if (all_in_tol) {
                        cprint("  Adaptive correction DONE: all within +/-%d deg (%d rounds)\r\n",
                               (int)corr_threshold, correction_count);
                        RLOG_I("CORR", "done: %d rounds, thr=%d deg",
                               correction_count, (int)corr_threshold);
                    } else {
                        cprint("  Adaptive correction STOP: max %d rounds reached\r\n",
                               correction_count);
                        RLOG_W("CORR", "stop: max rounds %d reached (timeout)",
                               correction_count);
                    }
                    correction_pending = 0;
                }
            }
        }

        /* ===== Periodic real-time angle feedback ===== */
        if (feedback_period_ms != 0U &&
            (int32_t)(HAL_GetTick() - feedback_next_tick) >= 0) {
            feedback_next_tick = HAL_GetTick() + feedback_period_ms;
            cmdRealtimeFeedback();
        }

        /* ===== Periodic real-time end-effector pose feedback ===== */
        if (pose_feedback_period_ms != 0U &&
            (int32_t)(HAL_GetTick() - pose_feedback_next_tick) >= 0) {
            pose_feedback_next_tick = HAL_GetTick() + pose_feedback_period_ms;
            cmdRealtimePoseFeedback();
        }

        /* ===== 遥测打印 (tm): 只读缓存, 不占总线 ===== */
        TelemetryPrintService();

        /* ===== E-Stop check ===== */
        {
            /* CAN 上报用: 只报**边沿**(按下那一次), 不每拍刷屏。
             * 常态由周期状态帧的 flags 位表达, 事件帧只负责"刚刚发生了什么"。 */
            static uint8_t can_prev_estop = 0U;
            uint8_t estop_now = IsEStop();
            if (estop_now && !can_prev_estop) {
                CanProto_SendEvent(ARM_CAN_EVENT_ESTOP, 0U);
                RLOG_W("CAN", "event: E-STOP asserted");
            }
            can_prev_estop = estop_now;
        }
        if (IsEStop()) {
            LED_R(1);
            if (Trajectory_IsActive()) {
                Trajectory_Stop();
                cprint("\r\n  E-STOP! Trajectory aborted.\r\n> ");
                RLOG_W("SAFE", "E-STOP! trajectory aborted");
            }
            teach_play_abort(0);
            /* Only allow 's' status and 'h' help */
            int len = Console_TryReadLine();
            if (len > 0) {
                if (console_rx[0] != 's' && console_rx[0] != 'h') {
                    cprint("  E-STOP active! Only 's' and 'h' allowed.\r\n> ");
                    continue;
                }
                ProcessCommand((char*)console_rx, len);
                cprint("> ");
            }
        } else {
            LED_R(0);
        }

        /* ===== 轨迹结束事件 (CAN 上报) =====
         * 上位机发一条 MOVE_JOINTS 之后, ACK 只说明"运动已启动";
         * 真正"走完了"要靠这个事件或轮询 GET_STATUS。给事件后主机就能
         * 事件驱动地排下一条指令, 不必按固定周期去猜。 */
        {
            static uint8_t can_prev_traj = 0U;
            uint8_t traj_now = Trajectory_IsActive();
            if (can_prev_traj && !traj_now) {
                CanProto_SendEvent(ARM_CAN_EVENT_TRAJ_DONE,
                                   (uint8_t)(IsEStop() ? 0U : 1U));
            }
            can_prev_traj = traj_now;
        }

        /* ===== Binary protocol dispatch (主循环上下文，可安全总线通信) ===== */
        SerialProto_DispatchPending();   /* 串口 0xAA 帧执行 + 回包 */
#if ARM_LINK_MULTI
        /* both 构建: 轮询**本次启动实际初始化过**的外设(0924CE auto 下是两条)。
         * ⚠ 没被初始化的那条一次都不能 poll —— NetProto_Poll() 会驱动 PHY 状态机
         *   并操作 MAC 描述符, CanProto_Poll() 会读 CAN 寄存器; 而它们的外设
         *   在这次启动里**压根没初始化**, 轮询等于读未初始化内存 / 访问未使能
         *   的外设。OFF 时两条都不碰, 只留串口。
         * ⚠ 判据用 LinkCfg_*HwInited()(静态), **不能**用 ActiveMode():
         *   auto 下 active 是动态的, 静默期(动态=OFF)两条外设照样要 poll,
         *   否则就永远探测不到"有东西下发"了。 */
        if (LinkCfg_NetHwInited()) {
            NetProto_Poll();                 /* 网口(UDP): 收包 / PHY 状态机 */
        }
        if (LinkCfg_CanHwInited()) {
            LinkProto_DispatchPending();     /* 主从链路口 (UART4) 流帧执行 */
            CanProto_Poll();                 /* CAN 轮询（ARM_CAN_ENABLE=1 时有效） */
            /* 0x280 心跳/事件推送: 手动 CAN 模式恒推; auto 下链路静默期
             * 停推(总线上没人听就别喊), 有合法包(active=CAN)恢复。 */
            if (!LinkCfg_AutoEnabled() || LINK_IS_CAN()) {
                CanProto_Service();          /* CAN 主动上报: 周期状态帧 + 事件帧 */
            }
        }
        LinkCfg_Tick();                      /* auto: 5s 无合法包回收 active */
#else
#if !ARM_LINK_NET
        LinkProto_DispatchPending();     /* 主从链路口 (UART4) 流帧执行 */
#endif
#if ARM_LINK_NET
        NetProto_Poll();                 /* 网口(UDP) 轮询: 收包/PHY 状态机 */
#else
        CanProto_Poll();                 /* CAN 轮询（ARM_CAN_ENABLE=1 时有效） */
        CanProto_Service();              /* CAN 主动上报: 周期状态帧 + 事件帧 */
#endif
#endif

#if ARM_MASTER_BUILD
        master_service();                /* 主臂: 读六轴 -> 组 0x13 帧 -> UART4 */
#endif

        /* ===== Console (non-blocking) =====
         * 一次主循环把环里**已到**的字符尽量消化掉 (上限 64, 防饿死急停检查)。
         * 旧写法每拍只取 1 个字符 ⇒ 收一行 `#P`(35B) 要 35 拍主循环; 65 Hz 下
         * 需要主循环 ≥2.4 千拍/秒, 只要有一拍被拖长(总线读超时/诊断打印),
         * 128 B 的环(66Hz 下只有 ≈53 ms 余量)立刻溢出, 丢字节把两行粘成垃圾 ⇒ 整帧拒 ⇒
         * 自锁冻结。改成"按需吸干"后, 抖动只是让某几拍多做事, 不再丢字节。 */
        if (!IsEStop()) {
            for (uint8_t budget = 0U; budget < 64U; budget++) {
                int len = Console_TryReadLine();
                if (len <= 0) break;
                main_progress(ARM_PHASE_CMD, (uint32_t)(uint8_t)console_rx[0]);
                ProcessCommand((char*)console_rx, len);
                main_progress(ARM_PHASE_IDLE, 0U);
                if (console_echo_en) cprint("> ");
            }
        }

        /* 网口 SYS_RESET: 上面 NetProto_Poll() 里的 ACK 已经发完, 这时才真正
         * 复位 —— 顺序反了上位机只会看到超时。见 ArmMain_RequestReset 注释。 */
        ArmMain_ServicePendingReset();
    }
}

/* ===== TIM2 interrupt callback (2kHz) ===== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        /* 主循环停滞观察者 (2026-09-18): 本中断与主循环**完全独立**, 所以哪怕
         * 主循环卡死在某个无界等待里, 它也照常在跑 —— 正因如此它才适合当见证者。
         * 2 kHz 下每拍只做两次整数比较, 开销可忽略。见 arm_fault.h。 */
        ArmStall_Tick(g_main_progress_ms, g_main_phase, g_main_cmd);

        /* Heartbeat: toggle green LED at ~2Hz (net 构建下 PC4=ETH_RXD0, 只翻软件态不动脚) */
        static uint16_t heartbeat = 0;
        if (++heartbeat >= 1000) {
            heartbeat = 0;
#if ARM_LINK_MULTI
            /* both 构建: NET 外设初始化过(手动 NET 或 auto)时 PC4 = ETH_RXD0
             * (由 PHY 驱动), 只能翻软件态; CAN/OFF 形态下 PC4 是 LED, 照常翻转。
             * ⚠ 0924CE: 判据用 NetHwInited —— auto 静默期也不能把 RMII 脚当 LED。 */
            if (LinkCfg_NetHwInited()) {
                LED_G(0);
            } else {
                LED_G(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4));
            }
#elif !ARM_LINK_NET
            LED_G(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4));
#else
            LED_G(0);
#endif
        }

        /* Trajectory output: divide 2kHz -> 50Hz (every 40 ticks).
         * The S-curve still uses its 200Hz timeline; main loop advances four
         * interpolation ticks after each batch Sync Write. */
        if (++traj_tick_div >= 40) {
            traj_tick_div = 0;
            /* 2026-09-16: 节拍标志只有 1 bit。若上一拍还没被主循环消化, 这个
             * 新边沿只是把它重新置 1 —— 中间的节拍被**静默合并**掉。示教录音
             * 按节拍记样本, 被合并的节拍就是录音里的"时间空洞"。这里计数,
             * 由 teach_report_integrity() 在示教退出时打出来。 */
            if (teach_mode_active && traj_do_update) teach_tick_lost++;
            traj_do_update = 1;
        }
    }
}

/* ===== System Clock ===== */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void) {
    if (console_ready) {
        cprint("\r\n*** ERROR_HANDLER! Init failed. ***\r\n");
    }
    /* Buzzer long beep = error signal */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { while(1); }
#endif
