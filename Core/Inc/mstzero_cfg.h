/* ============================================================
 * mstzero_cfg.h - 主臂零点持久化 (区 H, 见 flash_cfg.h 分区表)
 *
 * 用途: 板内遥操 (fi 命令, 0924BH) 反解主臂角度时, 需要主臂各关节
 *   官方 Home 对应的 raw 零点 (master_zero_raw[6])。主臂是 STS3215
 *   虚拟舵机 (只编码器无电机), 零点一次标定后应跨复位保留 —— 否则
 *   每次上电都要重新 c -> m -> 0 记一次。
 *
 * 记录结构 32B 自带 magic + 求和校验, 走 CfgFlash 保区写,
 *   A~G 区自动原样保住。load 失败(空区/坏数据)调用方应回退
 *   用从臂 SERVO_OFFSET 反解 (fi 现有行为)。
 * ============================================================ */
#ifndef MSTZERO_CFG_H
#define MSTZERO_CFG_H

#include <stdint.h>

/* 保存主臂 6 轴零点 raw。z 为 6 元素数组; 任一为 0xFFFF(无效哨兵)
 * 或 z==NULL 返回 BADARG。成功返回 CFG_FLASH_OK。 */
uint8_t MstZeroCfg_Save(const uint16_t z[6]);

/* 读出主臂 6 轴零点。记录有效(空区/坏数据不算)返回 1 并填 z,
 * 否则返回 0 且 z 不被改动。 */
uint8_t MstZeroCfg_Load(uint16_t z[6]);

/* 清除记录 (写全 0, 下次 Load 判无效)。 */
uint8_t MstZeroCfg_Clear(void);

#endif /* MSTZERO_CFG_H */
