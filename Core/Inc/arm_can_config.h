/* ============================================================
 * arm_can_config.h - CAN 通信配置（节点号 / 波特率 / 心跳周期）
 *
 * 为什么需要: 机械臂作为 CAN 从站, 上位机靠 `0x100+node` / `0x180+node`
 * 两个 ID 段寻址。过去 node 在 main.c 里写死 `CanProto_Init(1)`, 波特率
 * 在 arm_can_driver.c 里写死 PS=6(仅 500k) —— 换一套组网就要重编重烧。
 *
 * 现在: 运行时可改（`can node/baud/hb`）并落盘到 Flash 扇区 11 的
 * CAN 区（与 PID/校准记录同扇区但互不覆盖, 见 flash_cfg.h）。
 *
 * 默认值: node=1, 500 kbit/s, 心跳 1000 ms。
 * ============================================================ */
#ifndef ARM_CAN_CONFIG_H
#define ARM_CAN_CONFIG_H

#include <stdint.h>

/* 出厂默认 */
#define CAN_CFG_DEF_NODE     1U
#define CAN_CFG_DEF_BAUD     500U      /* kbit/s */
#define CAN_CFG_DEF_HB_MS    1000U     /* 周期状态帧周期; 0=关 */

/* 取值范围 */
#define CAN_CFG_NODE_MIN     1U
#define CAN_CFG_NODE_MAX     15U
#define CAN_CFG_HB_MIN       100U      /* 非 0 时的下限 */
#define CAN_CFG_HB_MAX       10000U

/* 支持的标准波特率(kbit/s)。都是 42MHz PCLK1 下用 14TQ / 采样点 85.7%
 * 精确整除得到的, 所以采样点在各档之间一致。 */
#define CAN_CFG_BAUD_COUNT   5U
extern const uint16_t can_cfg_baud_table[CAN_CFG_BAUD_COUNT];

/* 生效配置（8 字节, 便于整块存 Flash） */
typedef struct {
    uint8_t  node_id;     /* 1..15, 默认 1 */
    uint8_t  iface;       /* 1=CAN1(PB8/PB9 或 PA11/PA12), 2=CAN2(PB12/PB13);
                           * 0 视为 1 —— 兼容 2026-09-18 之前写下的旧记录
                           * (那时这个字节是 reserved0, 恒 0)。见 `cx` 命令。 */
    uint16_t baud_kbps;   /* 100/125/250/500/1000, 默认 500 */
    uint16_t hb_ms;       /* 0=关心跳; 100..10000, 默认 1000 */
    uint16_t reserved1;
} ArmCanCfg;

/* 接口取值（与 arm_proto_can.h 的 ARM_CAN_IF_* 一致; 这里不复用宏是为了让
 * arm_can_config 不依赖 HAL 头文件, 便于单测/移植) */
#define CAN_CFG_IF_CAN1      1U
#define CAN_CFG_IF_CAN2      2U

/* ---- 开机默认接口（2026-09-18 晚, 0918H）----------------------------------
 * ⭐ 默认值由 CAN1 改成 **CAN2**。三条理由，每条都是现场定案的：
 *   ① **本板卡的 CAN 收发器实际就接在 CAN2**（PB12/PB13）—— 用 `ps` 灌帧/不灌帧
 *      对照实验定案（红线 34），不是猜的。
 *   ② CAN1 的默认那对脚 PA11/PA12 在 F407 上**同时是 USB_DM/USB_DP**：板子的 USB
 *      座就在那里，走 CAN1 天然要跟 USB 打架（见 arm_proto_can.h 里那段警示）。
 *   ③ 以前每次烧录都要人工 `cx 2` 才能开始测 —— 因为整片擦除会把 CAN 配置区一起
 *      抹掉、回落到默认值 CAN1。漏一次就表现为"固件没问题、指纹也对、但上位机
 *      全部超时"，极易误判成"烧坏了/新固件没修好"（红线 37，现场真踩过）。
 *   ⇒ 把默认值改成**硬件实况**：烧完即用。要回到 CAN1 用 `cx 1`（或 `CANIF=c1` 重编）。
 * ⚠ 只是"开机默认值"：`cx 1|2` 运行期随时可切，`cx save` 可把它写进 flash 覆盖本默认。 */
#ifdef ARM_CAN_DEFAULT_IF_CAN1
#define CAN_CFG_DEF_IFACE    CAN_CFG_IF_CAN1
#else
#define CAN_CFG_DEF_IFACE    CAN_CFG_IF_CAN2
#endif

/* 启动流程: 先 CanCfg_Load(), 再把 baud 交给 ArmCanDrv_SetBitrate(),
 * 最后 CanProto_Init(cfg->node_id)。 */
void              CanCfg_Load(void);

/* 只读访问当前生效配置（永远非 NULL）。 */
const ArmCanCfg*  CanCfg_Get(void);

/* ---- 修改（只改 RAM, 立即生效的项由调用方去驱动层落实） ---- */
/* 返回 0=成功, 负=参数非法(-1)。成功不代表已落盘。 */
int  CanCfg_SetNode(uint8_t node);
int  CanCfg_SetBaud(uint16_t kbps);
int  CanCfg_SetHb(uint16_t ms);
/* 改 CAN 接口(1=CAN1 / 2=CAN2)。⚠ 只改 RAM 里的记录, **真正切换硬件**要调用
 * ArmCanDrv_SetInterface(); main.c 里先 SetIface 再(需要时)Save。 */
int  CanCfg_SetIface(uint8_t iface);

/* 规范化: 0/非法值一律当 CAN1 (兼容旧 Flash 记录)。 */
uint8_t CanCfg_IfaceOrDefault(const ArmCanCfg* c);

/* 恢复出厂默认（只改 RAM）。 */
void CanCfg_Reset(void);

/* ---- 落盘 ---- */
/* 写 Flash 扇区 11 的 CAN 区（保区写入, 不动 PID/校准记录）。
 * 返回 0=成功, -1=失败。 */
int  CanCfg_Save(void);

/* 1 = 当前 RAM 配置与 Flash 里的一致; 0 = 有未保存的改动。 */
uint8_t CanCfg_IsSaved(void);
/* 1 = 本次启动是从 Flash 载入的(而非默认值)。 */
uint8_t CanCfg_LoadedFromFlash(void);

/* 波特率是否受支持 */
uint8_t CanCfg_BaudSupported(uint16_t kbps);

#endif /* ARM_CAN_CONFIG_H */
