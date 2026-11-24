/* ============================================================
 * arm_rc.h — RC 遥控输入层 (只读采集)
 *
 * 硬件 (2026-09-24 0924M 起): **PS2 无线手柄 + 接收器**, **硬件 SPI1 外设驱动**
 *       (此前 GPIO 模拟时序三档速度全 0xFF, 换硬件 SPI 一锤定音; 官方例程
 *       `ps手柄SPI(1).zip` 同款; ⭐0924N 修正采样沿 = **Mode3: CPOL=1/CPHA=1**
 *       /LSB 先行 —— 首版误设 CPHA=0(Mode2) 收不到数据):
 *         CS/ATT  <- PB1  (输出推挽)     -> 接收器 **CS** 线
 *         SCK     <- PB3  (SPI1_SCK AF5) -> 接收器 **CLK** 线
 *         MISO    <- PB4  (SPI1_MISO AF5)-> 接收器 **DAT** 线 (手柄 -> 主机)
 *         MOSI    <- PB5  (SPI1_MOSI AF5)-> 接收器 **CMD** 线 (主机 -> 手柄)
 *       ⭐ 接收器是 **6 线带标签款** (DAT/CS/CMD/CLK/5V/GND) —— 按**标签**接线,
 *         早先那套"9 孔/孔序/镜像"判读**全部作废**;
 *         **5V 供电 (不是 3.3V!** 旧 3.3V 供电疑似欠压)**, GND 共地, ACK 悬空。
 *       SPI1 @ APB2 84MHz / 256 ≈ 328kHz (手册 250kHz 档, 官方例程 281kHz 同级)。
 *       ⚠ PB3/PB4 复位默认是 JTAG —— 本工程只用 SWD, 配成 AF5 即可。
 *       ⚠ PB5 = CAN2_RX —— net 构建下 CAN2 弃用, 编译期已拦 `cx/can` 命令。
 *
 * 协议摘要 (手册 V1.5, 一帧 9 字节, CS 低电平期间一次传完):
 *   DO: 0x01 0x42 0x00...        DI: idle ID 0x5A 按键1 按键2 RX RY LX LY
 *   ID: 0x73=红灯(模拟)模式 0x41=绿灯(数字)模式; 按键按下=0
 *   按键1(Data[3]): SELECT L3 R3 START UP RIGHT DOWN LEFT
 *   按键2(Data[4]): L2 R2 L1 R1 △ ○ ╳ □
 *   摇杆: 0x00~0xFF, 中位 ~128 (红灯模式下有效; SetInit 强制红灯并锁存)
 *
 * ⭐ 对上层 (arm_rc_ctrl.c) 的契约**不变**: 仍然是 6 路脉宽 (1000~2000µs,
 *   中位 1500) + fresh/live + w_min/w_max。摇杆模拟量在**本层**合成脉宽:
 *     CH1 X = RX (右=+)   CH2 Y = RY (上=+)   CH3 Z = LY (上=+)
 *     CH4 J5 = LX (右=+)  CH5 夹爪 = R1/L1 合成绝对量 (R1=张开→2000,
 *     L1=闭合→1000, 松开保持)   CH6 心跳 = 链路状态 (通信正常=1000)
 *   ⇒ 上/右 = >1500µs = 正向, 与 AT9S 杆感一致; 方向不符改 arm_rc_ctrl.c
 *   的 RC_SIGN_*(实测确认过的符号, 别顺手改)。
 *
 * ⭐ R9DS 时代的三条结构性教训在 PS2 上**天然消失**:
 *   ① R9DS 丢控=继续发帧只改值 ⇒ fresh/lost_events 结构性失效 —— PS2 是
 *      主动轮询, 手柄没响应就是没有帧, fresh/lost 恢复本义;
 *   ② 心跳通道要靠 F/S 反极性 —— PS2 链路状态就是心跳本身 (CH6);
 *   ③ `rc mid` 标中位 —— PS2 摇杆中位恒 1500(带死区), 开机自动标记。
 *
 * 历史 (2026-09-22~09-24): HotRC F-04A → R9DS+AT9S Pro (TIM4 四杆 + TIM1_CH4
 *   夹爪 + EXTI PG10 心跳), 已废除。寄存器回读字段(ccer/psc/t1_xx/hb_xx)保留
 *   在结构体里但恒 0 —— 供旧版 main.c 兼容, 新显示不再使用。
 * ============================================================ */
#ifndef ARM_RC_H
#define ARM_RC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 通道布局 (对上层不变) ---- */
#define ARM_RC_NCH           6U       /* CH1..4 杆 + CH5 夹爪 + CH6 心跳        */
#define ARM_RC_CH_CLAW       4U       /* CH5 = 夹爪 (R1/L1 合成绝对量)          */
#define ARM_RC_CH_HB         5U       /* CH6 = 心跳 (PS2 链路状态)              */

/* ---- PS2 硬件引脚 (0924M 硬件 SPI 定案, 见头注释) ---- */
#define ARM_RC_PS2_CS_PORT   GPIOB
#define ARM_RC_PS2_CS_PIN    GPIO_PIN_1    /* CS/ATT  -> 输出推挽 -> 插头6 */
#define ARM_RC_PS2_SCK_PIN   GPIO_PIN_3    /* SPI1_SCK  AF5 -> 插头7 CLK  */
#define ARM_RC_PS2_MISO_PIN  GPIO_PIN_4    /* SPI1_MISO AF5 <- 插头1 DAT  */
#define ARM_RC_PS2_MOSI_PIN  GPIO_PIN_5    /* SPI1_MOSI AF5 -> 插头2 CMD  */
#define ARM_RC_PS2_BYTE_GAP_US 16U         /* 字节间间隔 (官方例程 10~50µs) */
#define ARM_RC_PS2_BIT_HALF_US 5U          /* bit-bang 位半周期 5µs ≈ 50kHz (0924CW) */

/* ---- PS2 协议常量 ---- */
#define ARM_RC_PS2_ID_RED    0x73U         /* 红灯 = 模拟模式 (摇杆有效)         */
#define ARM_RC_PS2_ID_GREEN  0x41U         /* 绿灯 = 数字模式 (摇杆无效)         */
#define ARM_RC_PS2_ID_79     0x79U         /* 无线双震手柄 (部分 2.4G 套装)      */
#define ARM_RC_PS2_ID_WLESS  0x20U         /* 非标 2.4G 接收器 (0924CO, 新板实测:
                                            * 帧结构/0x5A/按键全正常, 仅 ID 非标) */
#define ARM_RC_PS2_ID_CFG    0xF3U         /* 配置模式 (0924CP): 手柄活着但卡在
                                            * config mode, 需只补发退出帧 */
#define ARM_RC_PS2_MARK      0x5AU         /* Data[2] = "数据来了"               */
/* 摇杆有效的 ID 判定: 红灯/无线双震/非标 2.4G 都按模拟处理 (0924CO) */
#define ARM_RC_ID_ANALOG_OK(id) \
    ((id) == ARM_RC_PS2_ID_RED   || (id) == ARM_RC_PS2_ID_79 || \
     (id) == ARM_RC_PS2_ID_WLESS)
#define ARM_RC_PS2_BTN_L1    0x04U         /* Data[4] bit2 (按下=0)              */
#define ARM_RC_PS2_BTN_R1    0x08U         /* Data[4] bit3                       */
#define ARM_RC_PS2_STICK_MID 128           /* 摇杆中位                           */
#define ARM_RC_PS2_STICK_DZ  3             /* 摇杆死区 (±count, 滤中心抖动)      */
#define ARM_RC_PS2_US_FULL   500.0f        /* 摇杆满行程 -> ±500µs               */

/* ---- 夹爪按键合成参数 ---- */
#define ARM_RC_CLAW_SWEEP_MS 2400U         /* 松/夹全程时间 (ms, 1000µs 行程), 0924P 减半速 */

/* ---- 按键位图与键位映射 (2026-09-25, 0924H) ----
 * 16 个按键: 代码 1~16 对应位图 bit0~15 (按下=1, 原始字节按下=0 已取反):
 *   btn3: SELECT L3 R3 START UP RIGHT DOWN LEFT
 *   btn4: L2 R2 L1 R1 TRI CIR CRS SQR
 * 功能 -> 按键的映射表存扇区 11 区 E (`rc map save` 落盘, 掉电不丢)。 */
#define ARM_RC_NBTN          16U

/* 可映射功能编号 (映射表下标) */
#define ARM_RC_FN_CLAW_OPEN  0U    /* 夹爪张开 (长按动, 松开停)              */
#define ARM_RC_FN_CLAW_CLOSE 1U    /* 夹爪闭合 (长按动, 松开停)              */
#define ARM_RC_FN_HOME       2U    /* 中位: 六轴回 180° (边沿触发)           */
#define ARM_RC_FN_FOLD       3U    /* 折叠位: POSE_HOME (边沿触发)           */
#define ARM_RC_FN_TEACH_REC  4U    /* 示教录制 开/停 (边沿触发)              */
#define ARM_RC_FN_TEACH_PLAY 5U    /* 回放刚录的轨迹 / 停止回放 (边沿触发)   */
#define ARM_RC_FN_J5_CCW     6U    /* J5 手腕旋转 逆时针 (长按连续调, 松开保持) */
#define ARM_RC_FN_J5_CW      7U    /* J5 手腕旋转 顺时针 (同上)                */
#define ARM_RC_FN_COUNT      8U

/* ---- 通道合成/新鲜度 ---- */
#define ARM_RC_TIMEOUT_MS    100U          /* 距今这么久没有效帧 ⇒ 该通道 stale */
#define ARM_RC_CFG_RETRY_MS  2000U         /* 非红灯模式时, 重跑 SetInit 的间隔  */
#define ARM_RC_FAIL_RECFG    50U           /* 连续失败这么多帧才怀疑模式/链路    */

/* 单通道快照 (与 R9DS 版一致, 上层无感) */
typedef struct {
    uint16_t width_us;   /* 最近一个合成脉宽 (µs); 0 = 还没有样本 */
    uint16_t w_min;      /* 观测到的最小脉宽 (0xFFFF = 无样本)    */
    uint16_t w_max;      /* 观测到的最大脉宽                      */
    uint32_t edges;      /* 兼容保留: = 有效帧数                  */
    uint32_t frames;     /* 收到的有效帧数                        */
    uint32_t last_ms;    /* 最近一个有效帧的 HAL_GetTick()        */
    uint8_t  fresh;      /* 1 = 距今 ≤ ARM_RC_TIMEOUT_MS          */
} ArmRcChan;

/* 整体快照 */
typedef struct {
    uint8_t  inited;         /* PS2 GPIO/时序初始化成功               */
    uint8_t  t1_inited;      /* ⭐新义: SetInit 配置手柄成功(红灯模拟) */
    uint8_t  exti_inited;    /* 弃用, 恒 1 (R9DS 时代 = EXTI 心跳)    */
    uint8_t  live;           /* 1 = 最近有有效帧 (手柄在通信)         */
    uint8_t  mid_valid;      /* 1 = 中位已标记 (开机自动 1500)        */
    uint8_t  mid_mask;       /* bit i = CH(i+1) 中位已标              */
    uint16_t mid[ARM_RC_NCH];
    uint32_t now_ms;
    uint32_t last_frame_ms;
    uint32_t lost_events;    /* "有信号→无信号"次数 (PS2 上恢复本义)  */
    /* ---- 以下为 R9DS 时代的寄存器回读字段, 保留但恒 0 (兼容) ---- */
    uint32_t ccer, ccmr1, ccmr2, psc, arr;
    uint32_t t1_ccer, t1_ccmr2, t1_psc;
    uint32_t hb_imr, hb_exticr;
    ArmRcChan ch[ARM_RC_NCH];
    /* ---- ⭐ PS2 新增 (追加在尾部, 不破坏前面偏移) ---- */
    uint8_t  id;             /* 最近有效帧的手柄 ID (0x73/0x41)       */
    uint8_t  analog_ok;      /* 1 = 红灯模拟模式                      */
    uint8_t  btn3, btn4;     /* 原始按键字节 (按下=0)                 */
    uint8_t  stick_rx, stick_ry, stick_lx, stick_ly;  /* 原始模拟量   */
    uint32_t poll_ok;        /* 有效帧累计                            */
    uint32_t poll_fail;      /* 无响应/坏帧累计                       */
    uint32_t fail_run;       /* 当前连续失败帧数                      */
} ArmRcState;

/* 开机调用一次。失败不致命: inited=0, 其余照常。 */
void ArmRc_Init(void);

/* 重配 GPIO + 重跑手柄配置 (`nt` 后或手柄抽风时救急; PS2 下不再抢 TIM4)。 */
void ArmRc_Reinit(void);

/* 主循环 50Hz 调一次(挂 traj_do_update 那拍)。⭐ 阻塞 ~0.9ms: 完成一帧
 * PS2 轮询 + 通道合成 + 新鲜度/失控计数。 */
void ArmRc_Poll(void);

/* 清 min/max/帧数/失控次数(不动中位)。 */
void ArmRc_ResetStats(void);

/* 把当前脉宽记为"中位"。PS2 下开机已自动标 1500, 此命令仅保留兼容。 */
void ArmRc_SetMid(void);

/* 取快照(指针指向模块内部静态, 只读)。 */
const ArmRcState* ArmRc_Get(void);

/* ---- 按键层 (0924H) ---- */

/* 当前按下的键位图: bit n = 代码 n+1 按下。手柄无链路时恒 0。 */
uint16_t ArmRc_Buttons(void);

/* 取走"自上次调用以来新按下"的边沿位 (0→1)。取走即清零; 上层每 50Hz 拍
 * 调一次 (rc_button_service)。漏调不丢边沿 (pending 累积)。 */
uint16_t ArmRc_BtnEdges(void);

/* 代码 1..16 是否正被按住 (电平查询, 夹爪合成层用)。 */
uint8_t  ArmRc_BtnDown(uint8_t code);

/* 功能 fn (ARM_RC_FN_*) 当前是否正被按住 = 查它映射的键的电平。
 * 未绑定(0)恒 0。俯仰双按键就是走这个 (长按语义, 不是边沿)。 */
uint8_t  ArmRc_FnDown(uint8_t fn);

/* 代码 <-> 名字 ("SELECT".."SQR")。BtnFromName 接受名字/数字 1~16/"none",
 * 未知返回 0。 */
const char* ArmRc_BtnName(uint8_t code);
uint8_t  ArmRc_BtnFromName(const char* name);

/* ---- 键位映射 (扇区 11 区 E) ---- */

/* 当前映射表 [ARM_RC_FN_COUNT], 值 = 按键代码 (0 = 未绑定)。只读指针。 */
const uint8_t* ArmRc_MapGet(void);

/* 设映射: fn < ARM_RC_FN_COUNT, code 0~16 (0=解绑)。重复绑定返回 2。 */
uint8_t  ArmRc_MapSet(uint8_t fn, uint8_t code);

/* 恢复默认 (L2=张开 R2=闭合 TRI=中位 CIR=折叠 SQR=录制 CRS=回放
 *           R1=J5逆时针 L1=J5顺时针 —— 见 arm_rc.c MAP_DEF)。 */
void     ArmRc_MapDefaults(void);

/* 写区 E (阻塞 ~1s, 只许控制台上下文调)。返回 1=OK。 */
uint8_t  ArmRc_MapSave(void);

/* 读区 E (Init 自动调): magic/版本/校验/条目任一不过 = 用默认, 不报错。 */
void     ArmRc_MapLoad(void);

/* ---- 接线诊断 (0924J, 0924L 扩三帧) ----
 * 跑三帧原始轮询: 帧1=常速(现役参数) 帧2=慢速 20µs 帧3=官方例程同款 50µs,
 * 各 9 字节 DI 原样返回 buf[0..8]/buf[9..17]/buf[18..26];
 * *di_idle = 采样前 DI 空闲电平。返回 0 = 采集层未初始化。
 * 不动任何内部状态 (不合成通道/不计数)。 */
uint8_t  ArmRc_DiagRaw(uint8_t buf[27], uint8_t* di_idle);

/* ---- 接线定位 (0924K) ----
 * 分阶段拉低单个输出脚, 供 main.c `rc pin` 配合万用表找"哪根线接哪个孔":
 * phase 0 = 四脚全回空闲高 + 解除探测保持; 1 = 只拉低 CS(PB4);
 * 2 = 只拉低 CLK(PB5); 3 = 只拉低 DO/CMD(PB3)。DI(PB1) 是输入不参与。
 * 探测期间 ArmRc_Poll 挂起 (内部 s_probe_hold), 不与 50Hz 轮询抢 GPIO。 */
void     ArmRc_PinDrive(uint8_t phase);

/* ---- SPI CPHA 调试 (0924N) ----
 * 运行时切换 SPI 时钟相位 (0=CPHA=0, 1=CPHA=1)。
 * 切完自动重跑手柄配置序列, 方便在线验证哪种模式能通信。
 * 上电默认 CPHA=1 (索尼 PS2 标准, SPI Mode3)。 */
void     ArmRc_SetCpha(uint8_t cpha1);
uint8_t  ArmRc_GetCpha(void);
void     ArmRc_SetPhase(uint8_t mode);   /* 0924CX: 0=提前(旧克隆) 1=标准 2=自动 */
uint8_t  ArmRc_GetPhaseEarly(void);
void     ArmRc_SetGate(uint8_t digital_work);  /* 0924CY: 1=绿灯族工作(默认) 0=红灯族工作 */
uint8_t  ArmRc_GetGate(void);
uint8_t  ArmRc_GetGateBlock(void);
uint8_t  ArmRc_GetPhaseAuto(void);

/* ---- 手柄配置序列开关 (0924CT) ----
 * en=0 (⭐默认): 纯轮询 —— 上电/reinit/掉线重配 **从不发** 任何配置命令
 *   (0x43/0x44), 只发 0x42 轮询帧。适配 2.4G 克隆接收器: ESP32 直测证明
 *   它对裸轮询回 ID=0x20 且按键/摇杆正常, 而"进配置→锁模拟→退出"序列
 *   会把它推进配置模式 (ID=0xF3) 且退出帧被吞 = 永久卡死。
 * en=1: 兼容模式 —— 恢复手册 SetInit 序列 (真·原装手柄绿灯时用)。
 * 运行时用 `rc cfg 0|1` 切; 若当前卡 F3, 补发退出帧的逃逸逻辑照常工作。 */
void     ArmRc_SetCfgEn(uint8_t en);
uint8_t  ArmRc_GetCfgEn(void);

/* ---- 模拟命令旁路 (0924CU) ----
 * 不进/退配置模式, 直发 0x44(模拟+0x03 锁存): 3×轮询→5ms→0x44→5ms→3×轮询。
 * 供 main.c `rc cfg a` 调; SetInit 必卡 F3 的克隆手柄的试探通道。 */
void     ArmRc_CfgAnalogDirect(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_RC_H */
