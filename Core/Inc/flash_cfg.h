/* ============================================================
 * flash_cfg.h - 配置扇区(sector 11)的分区读写
 *
 * 背景(为什么必须有这一层):
 *   STM32F4 的 Flash 只能**整扇区擦除**, 而本工程唯一留作数据的
 *   扇区 11 (0x080E0000, 128KB) 上要同时住三份互不相干的配置:
 *     区 A: PID/修正/速度 + 关节限位与 Home 零点  (pid_flash.c)
 *     区 B: CAN 通信配置 (节点号/波特率/心跳)     (arm_can_config.c)
 *     区 C: 链路偏好 (串口常通 + CAN/网口 二选一) (arm_link_cfg.c)
 *   各自直接 HAL_FLASHEx_Erase(sector 11) 的写法会造成:
 *     "存 CAN 配置 → 校准零点被擦掉", "F 存 PID → CAN 配置被擦掉"。
 *   对第一件事尤其致命: 校准零点丢了会让解析 IK 静默降级成 CCD,
 *   现场表现为 `g` 报 "tool pitch NOT met" —— 排查代价极高。
 *   所以统一走本模块的【保区写入】: 先读出"其余各区"到 RAM, 擦整
 *   扇区, 再把它们原样写回, 最后写本区。
 *
 * 分区布局(相对扇区基址 0x080E0000):
 *   0x0000 ~ 0x0FFF   区 A  PID/校准记录      (现记录 192B, 留 4KB 余量)
 *   0x1000 ~ 0x1FFF   区 B  CAN 通信配置      (现记录 32B, 留 4KB 余量)
 *   0x2000 ~ 0x2FFF   区 C  链路偏好          (现记录 8B,  留 4KB 余量)
 *   0x3000 ~ 0x3FFF   区 D  RC 遥控速度偏好   (现记录 ~32B, 留 4KB 余量; 09-24)
 *   0x4000 ~ 0x4FFF   区 E  RC 按键键位映射   (现记录 16B,  留 4KB 余量; 09-24)
 *   0x5000 ~ 0x5FFF   区 F  几何偏好         (C 世界偏移 + zf 桌面下限, 24B; 09-29)
 *   0x6000 ~ 0x6FFF   区 G  舵机总线路由     (JOINT_BUS[6] + mask, 16B; 11-02)
 *   0x7000 ~ 0x7FFF   区 H  主臂零点         (master_zero_raw[6], 32B; 11-04)
 *   0x8000 ~ 0x1FFFF  未使用(保持 0xFF)
 *   ⚠ 本扇区**只归本模块使用**: 示教文件在 S7~S10(见 teach_store.h),
 *     两者不冲突, 别把这里的"未使用"当成可以随便放的空地。
 *
 * 字节级保区: 本模块**不解释各区的结构**, 只按字节搬。所以
 * PID_Record_t / ArmCanCfg / ArmLinkCfg 将来加字段/改版本都不会
 * 影响这里(反向亦然)。
 *
 * 保区成本: 擦除前要同时持有"其余各区", 故需 (CFG_REGION_COUNT-1) 个
 *   4KB 缓冲 = 12KB RAM(static, 绝不进栈; 09-24 加区 D 后 3 个)。见
 *   flash_cfg.c 的 s_preserve。
 * ============================================================ */
#ifndef FLASH_CFG_H
#define FLASH_CFG_H

#include <stdint.h>

/* STM32F407 1MB: sector 11 = 0x080E0000 ~ 0x080FFFFF
 * (FLASH_SECTOR_11, 128KB)。填 11 而非直接用 HAL 宏, 是为了让本头
 * 文件不依赖 HAL, 便于上位机/PC 侧复用。 */
#define CFG_SECTOR_INDEX   11U
#define CFG_SECTOR_ADDR    0x080E0000U

/* 分区起点(相对扇区基址)与单区预留跨度 */
#define CFG_OFF_PID        0x0000U
#define CFG_OFF_CAN        0x1000U
#define CFG_OFF_LINK       0x2000U     /* 链路偏好: 串口常通 + CAN/网口 二选一 */
#define CFG_OFF_RCSPD      0x3000U     /* RC 遥控速度偏好 (arm_rc_ctrl.c, 09-24) */
#define CFG_OFF_RCMAP      0x4000U     /* RC 按键键位映射 (arm_rc.c, 09-24)      */
#define CFG_OFF_GEO        0x5000U     /* 几何偏好: C 偏移 + zf 桌面下限 (09-29) */
#define CFG_OFF_BUSMAP     0x6000U     /* 舵机总线路由: JOINT_BUS[6] + mask (11-02) */
#define CFG_OFF_MSTZERO    0x7000U     /* 主臂零点: master_zero_raw[6] (11-04, fi 遥操用) */
#define CFG_OFF_JLIM       0x8000U     /* L 手动关节限位: mn/mx raw x6 (0924BY) */
#define CFG_REGION_SPAN    0x1000U     /* 每区 4KB 预留 */
#define CFG_REGION_COUNT   8U          /* 已用区数 —— 保区缓冲数量由它推导 */

/* 返回码 */
#define CFG_FLASH_OK       0
#define CFG_FLASH_ERROR    1
#define CFG_FLASH_BADARG   2

/* 保区写入: 把 src 的 n 字节写到 (扇区基址 + off)。
 * 只允许 off 取 s_region_off 里的合法区起点之一 (A=PId/B=CAN/C=LINK/
 * D=RCSPD/E=RCMAP); n <= CFG_REGION_SPAN。其余各区**整块原样保住**。
 * 流程: 读其余各区 -> 擦整扇区 -> 回写其余各区 -> 写本区 -> 双向校验。
 * 返回 CFG_FLASH_OK / CFG_FLASH_ERROR / CFG_FLASH_BADARG。
 *
 * ⚠ 本调用是**阻塞**的: 128KB 扇区擦除典型 ~1s(最坏 4s), 加上回写各
 *   4KB。调用点必须不在实时路径上 —— 本项目只在**控制台命令上下文**调,
 *   那种场景下命令本来就慢, 可以接受。**绝不要**在 TIM2 回调/主循环
 *   高频段里调它。 */
uint8_t CfgFlash_WriteRegion(uint32_t off, const void* src, uint32_t n);

/* 只读: 把 (扇区基址 + off) 的 n 字节读到 dst。参数非法返回 BADARG。 */
uint8_t CfgFlash_ReadRegion(uint32_t off, void* dst, uint32_t n);

/* 整扇区擦除(所有区一起清, 用于"恢复出厂")。 */
uint8_t CfgFlash_EraseAll(void);

/* 本扇区是否已被访问过(HAL 状态自检用, 目前恒 1)。 */
uint8_t CfgFlash_Ready(void);

#endif /* FLASH_CFG_H */
