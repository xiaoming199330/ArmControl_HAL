/* ============================================================
 * busmap_cfg.h - 舵机总线路由持久化 (扇区 11 区 G @ CFG_OFF_BUSMAP 0x6000)
 *
 * 存什么: "关节 -> 总线" 的实测路由 (JOINT_BUS[6]), 只存**已被实测确认**
 * 过的轴 (mask 位)。没确认过的轴不写, 上电后仍按编译期默认走。
 *
 * 为什么必须持久化: 0924BA/BB 的自动路由 s_map_conf[] 与 JOINT_BUS[] 都是
 * RAM 变量, **一复位就回到编译期默认 A/A/A/B/B/B**。而集成板常是"六颗全
 * 在 B 段", 那个默认本身就是错的 —— 于是每次上电都要重新赌一次 AutoMap,
 * 赌输(某轴当轮误码)就把该轴留在空的 A 段, 表现成"上次明明好了, 重启又
 * 缺轴", 且固件自己不知道自己错过。
 *
 * 持久化后: 上电先灌入上次确认的路由 -> pass1 直接查正确的段 -> 命中即返回,
 * 不再依赖当轮误码运气。路由真变了(换座子/换舵机) 时 AutoMap 仍能自愈并
 * 回写, 所以不会锁死在旧值上。
 *
 * 结构 16B 自带 magic + 求和校验; 搬运用 CfgFlash 保区写,
 * A~F 区自动原样保住 (见 flash_cfg.c)。
 * ============================================================ */
#ifndef BUSMAP_CFG_H
#define BUSMAP_CFG_H

#include <stdint.h>
#include "flash_cfg.h"   /* 返回码 CFG_FLASH_OK/ERROR/BADARG 与区地址 */

/* 读出路由。返回 1 = 有有效记录 (bus/mask 全部有效);
 * 返回 0 = 空区/坏记录, 调用方保持编译期默认。
 * mask 的 bit j 置 1 表示 bus[j] 来自实测, 可以采信。 */
uint8_t BusMapCfg_Load(uint8_t bus[6], uint8_t* mask);

/* 写入路由 (保区写, 阻塞 ~1s, 只许控制台/启动上下文调用)。
 * mask=0 视为无有效内容, 直接返回 BADARG 不写。
 * 返回 CFG_FLASH_OK / CFG_FLASH_ERROR / CFG_FLASH_BADARG。 */
uint8_t BusMapCfg_Save(const uint8_t bus[6], uint8_t mask);

/* 清除记录 (抹掉 magic)。下一次上电回到编译期默认布局。 */
uint8_t BusMapCfg_Clear(void);

#endif /* BUSMAP_CFG_H */
