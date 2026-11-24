/* ============================================================
 * arm_link_cfg.h - 链路偏好（串口常通 + CAN/网口）的持久化与运行期门控
 *
 * 硬件现状(2026-11-10 修正): RMII 已在 PG11/PG13/PG14 + PC4/PC5(RXD0/1)
 * + PA1/PA2/PA7 + PC1, PB12/PB13 已腾给 CAN2 ⇒ **CAN 与网口无引脚冲突**,
 * 0924CB 三开构建实测可同时初始化/同时收发(旧"PB13 物理互斥"是错断言)。
 * PC4/PC5 让位后 LED 指示灯不可用(唯一硬件代价)。
 *
 * 三种运行形态:
 *   can / net —— 手动单链路: 开机只初始化被选中的那一条(冻结, 行为同旧版)。
 *   auto(0924CE) —— **双链路自动门控**: 开机 CAN+NET 都初始化、都在监听;
 *     哪条收到**合法包**(CRC 过 + CMD 合法)哪条获得 active 并刷新时间戳;
 *     5s 无合法包自动收回(静默监听, 0x280 心跳也停)。下一个合法包即敲门
 *     + 直接执行(不吞包 —— 首包吞掉会让上位机"第一次总超时", 不可靠)。
 *     ⓵ auto **绝不做默认值**(2026-09-22"上电默认全关"仍然有效), 只能
 *        `link auto` + `link save` 显式开启;
 *     ⓶ "断开"对无连接的 UDP/CAN 只能定义为超时, 不是真断电 —— 外设
 *        必须常听才能探测"有东西下发";
 *     ⓶ 手动 can/net 原样保留, 要确定性时用。
 *
 * ⚠ 手动切换仍是"**下次开机生效**": 引脚 AF 与外设是启动时一次配好的。
 *   (auto 模式两条都初始化, 不存在这个问题。)
 * ============================================================ */
#ifndef ARM_LINK_CFG_H
#define ARM_LINK_CFG_H

#include <stdint.h>

/* 链路模式。数值进 Flash, 不要随手改。 */
#define LINK_MODE_OFF   0U   /* 两条都不启用(默认) */
#define LINK_MODE_CAN   1U   /* CAN 从站 (CAN2 = PB12/PB13, AF9) */
#define LINK_MODE_NET   2U   /* 网口 UDP (RMII PG11/13/14 + PC4/5) */
#define LINK_MODE_AUTO  3U   /* 双链路自动门控: 都监听, 合法包授 active(0924CE) */

/* AUTO 模式的活动超时: 无合法包超过此时长 => active 回到静默监听态 */
#define LINK_AUTO_TIMEOUT_MS  5000U

/* 生效偏好(2 字节) */
typedef struct {
    uint8_t mode;          /* LINK_MODE_* */
    uint8_t reserved;      /* 对齐/将来扩展 */
} ArmLinkCfg;

/* 开机流程: **必须早于 MX_GPIO_Init()** —— 因为 gpio.c 要按它决定
 * PC4/PC5 是配成 LED 还是整段让位给 RMII 的 AF11。
 * ⚠ 本函数**只读 Flash**, 不做任何写操作, 也不打日志
 *   (调用点在 RunLog_Init() 之前, RLOG 还不可用)。 */
void LinkCfg_Init(void);

/* 当前的**偏好** —— 即"**下次开机**用哪条"(Flash 记录, 或 RAM 里待保存的改动)。
 * ⚠ 它**不是**"现在正在跑哪条"; 后者看 LinkCfg_ActiveMode()。
 *   两者只有在没有未保存改动时才相等。 */
const ArmLinkCfg* LinkCfg_Get(void);

/* ⭐ 运行期链路门控的唯一合法判据。
 * 手动 can/net 模式: 启动时从 Flash 读出后冻结, 运行期不变(语义同旧版)。
 * auto 模式: 返回**动态**值 —— 合法包授 NET/CAN, 5s 超时回 LINK_MODE_OFF
 * (静默监听)。⭐ 启动初始化分支不能用它判 AUTO(见 LinkCfg_BootMode)。
 * 运行期的一切判定(命令拦截 / 链路轮询 / 状态上报)用它;
 * **绝不能**用 LinkCfg_Get(): 否则用户敲一次 `link can` 就会让**正在跑的网口
 * 当场停摆**(NetProto_Poll 不再被调用), 并转而 poll 一个**从未初始化**的
 * CAN 外设 —— 访问未使能的外设比"不工作"更糟(可能读野地址/抢总线)。 */
uint8_t LinkCfg_ActiveMode(void);

/* ⭐ 开机初始化分支专用: 返回"本次启动该按哪个形态初始化"(含 AUTO)。
 * 与 LinkCfg_Get()->mode 等价, 但语义显式 —— ActiveMode() 在 auto 下
 * 开机返回 OFF(动态值), 不能拿去判"要不要双初始化"。 */
uint8_t LinkCfg_BootMode(void);

/* ⭐ 开机**引脚分配**判据: 1 = 本次启动 PC4/PC5 与 PA1 归 RMII(网口形态)。
 * NET 手动模式与 **AUTO 都返回 1** —— 引脚是启动时一次配死的, auto 下
 * PC4/PC5 必须整段让位给 RMII AF11(LED 不可用)、PA1 是 ETH_REF_CLK
 * (UART4 必须跳过)。gpio.c 的 LED 配置与 UART4 初始化判定用这个,
 * **绝不能**用 LINK_IS_NET()/ActiveMode(): auto 开机时动态值=OFF, 会把
 * RMII 引脚错配成 LED/UART4 ⇒ 网口直接废掉(0924CE 踩坑记录)。 */
uint8_t LinkCfg_BootNetPins(void);

/* ---- AUTO 双链路门控(0924CE) ---------------------------------------- */
/* 启动初始化完成后登记(boot 分支里紧挨 NetProto_Init/CanProto_Init 调用)。
 * LED/轮询等"外设是否真的活着"的判定用这两个, **不能**用 ActiveMode()
 * (auto 下 active 是动态的, 静默时 LED 若按 NET 判会把 RMII 引脚当 LED 翻)。 */
void LinkCfg_MarkNetInited(void);
void LinkCfg_MarkCanInited(void);
uint8_t LinkCfg_NetHwInited(void);
uint8_t LinkCfg_CanHwInited(void);

/* auto 开关查询: 1 = 本次启动是 auto 双链路形态 */
uint8_t LinkCfg_AutoEnabled(void);

/* auto 模式下收到**合法包**(CRC 过 + 即将执行)时调用 —— 授 active + 刷时间戳。
 * 非 auto 模式下是空操作, 各链路收包点可以无条件调用。 */
void LinkCfg_TouchNet(void);
void LinkCfg_TouchCan(void);

/* 主循环每拍调用: auto 下超过 LINK_AUTO_TIMEOUT_MS 无合法包 => active 回 OFF。
 * 非 auto 模式空操作。 */
void LinkCfg_Tick(void);

/* 只改 RAM(下次开机才生效), 不落盘。返回 0=成功, -1=非法。 */
int LinkCfg_Set(uint8_t mode);

/* 落盘。返回 0=成功, -1=失败。
 * ⚠ 阻塞: 128KB 扇区擦除典型 ~1s。只在控制台命令上下文调用。 */
int LinkCfg_Save(void);

/* 1 = RAM 与 Flash 一致; 0 = 有未保存改动。 */
uint8_t LinkCfg_IsSaved(void);

/* 1 = 本次启动的偏好来自 Flash; 0 = 未配置(magic/CRC 不符) ⇒ 全关。 */
uint8_t LinkCfg_LoadedFromFlash(void);

/* 名字: "off" / "can" / "net" / "auto"; 未知返回 "?"。 */
const char* LinkCfg_NameOf(uint8_t mode);

/* 解析 "off|none|0" / "can|1" / "net|eth|udp|2" / "auto|3"。
 * 返回 0=成功(*out_mode 有效), -1=无法识别。 */
int LinkCfg_Parse(const char* s, uint8_t* out_mode);

/* ---- 运行期判定便捷宏(both 构建专用) -------------------------------
 * 只在 ARM_LINK_MULTI 下存在。旧构建(uart/can/net 单链路)里这些宏
 * **故意不定义** —— 那些构建继续走编译期 #if, 运行时行为与改动前一致
 * (新加的代码被 #if ARM_LINK_MULTI 挡在外面; flash_cfg 的保区缓冲倒是
 *  从 4KB 扩到 8KB, 那是加第三个配置区的必然代价, 见 flash_cfg.h)。
 * ⚠ 判定基准是 **LinkCfg_ActiveMode()** 而不是偏好 —— 理由见上面那段。
 *   引用它们的地方必须包在 #if ARM_LINK_MULTI 里。 */
#if ARM_LINK_MULTI
#define LINK_IS_OFF()   (LinkCfg_ActiveMode() == LINK_MODE_OFF)
#define LINK_IS_CAN()   (LinkCfg_ActiveMode() == LINK_MODE_CAN)
#define LINK_IS_NET()   (LinkCfg_ActiveMode() == LINK_MODE_NET)
#endif

#endif /* ARM_LINK_CFG_H */
