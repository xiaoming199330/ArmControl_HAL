/* ============================================================
 * geo_cfg.h - 几何偏好持久化 (扇区 11 区 F @ CFG_OFF_GEO 0x5000)
 *
 * 存什么: 用户现场标出来、不想每次上电重敲的几何/安全参数 ——
 *   1) `C dx dy dz`  世界坐标偏移 (kinematics.c WORLD_OFFSET_X/Y/Z);
 *   2) `zf <mm>`     桌面下限 "爪尖最低离桌 mm" (main.c geo_zfloor_mm);
 *   3) `zf on|off`   流控/遥操桌面限位启用开关 (0924BV, geo_zf_on);
 *   4) `fi margin`   遥操边界安全带 raw (0924BV, geo_margin_raw)。
 *
 * 为什么单独开一区: 扇区 11 整扇区擦除, A~E 区已被 PID/CAN/LINK/
 * RCSPD/RCMAP 占用; 塞进 A 区(PID 记录)要动版本化结构, 风险高
 * (校准零点在同区)。区 F 独立 4KB, 结构自带 magic+sum, 坏了就走默认值。
 *
 * v2 (0924BV): 记录 24B -> 28B, 新增 zf_on + margin。
 * Load 兼容旧 v1(24B) 记录: v2 校验失败回落 v1, 新字段走默认
 * (zf_on=1, margin=0xFFFF=未设) —— 老固件已存的 C/zf 不丢。
 *
 * v3 (0924CG): 记录 28B -> 60B, 新增 fold_j[6] + fold_valid ——
 *   `P here` 折叠位(休息位)实测标定, 存 relhome 关节角 (deg, 与
 *   main.c POSE_HOME_J 同坐标系), P 0 / 折叠位白名单 / 桌面下限
 *   全部跟随。布局: 前 28B 与 v2 完全一致 (sum 兼容), 新字段追加在
 *   28B 之后, 再带一个覆盖前 56B 的 sum2 —— 旧 v2 固件读 v3 记录仍能
 *   通过 v2 校验 (fold 被忽略), 双向兼容。Load 兼容 v1/v2 记录
 *   (fold_valid=0)。Save2/Save 等旧包装写区时自动带上最近一次
 *   Load3/Save3 读写的 fold 字段, 不会把标定抹掉。
 * ============================================================ */
#ifndef GEO_CFG_H
#define GEO_CFG_H

#include <stdint.h>

#define GEO_MARGIN_UNSET  0xFFFFU   /* margin 未设过, 上电用固件默认 */

/* 读出几何偏好。返回 1 = 有有效记录 (输出参数全部有效);
 * 返回 0 = 空区/坏记录, 调用方走固件默认。
 * zf_valid 输出 1 表示用户设过 zf, zf_mm 才有意义。
 * v2 兼容: 旧 v1 记录读出时 zf_on=1, margin=GEO_MARGIN_UNSET。 */
uint8_t GeoCfg_Load2(float* dx, float* dy, float* dz,
                     float* zf_mm, uint8_t* zf_valid,
                     uint8_t* zf_on, uint16_t* margin_raw);

/* 写入几何偏好 v2 (保区写, 阻塞 ~1s, 只许控制台上下文调用)。
 * 返回 CFG_FLASH_OK / CFG_FLASH_ERROR / CFG_FLASH_BADARG。 */
uint8_t GeoCfg_Save2(float dx, float dy, float dz,
                     float zf_mm, uint8_t zf_valid,
                     uint8_t zf_on, uint16_t margin_raw);

/* v1 兼容包装: 等价 Save2(..., zf_on=1, margin=GEO_MARGIN_UNSET) */
uint8_t GeoCfg_Save(float dx, float dy, float dz,
                    float zf_mm, uint8_t zf_valid);
uint8_t GeoCfg_Load(float* dx, float* dy, float* dz,
                    float* zf_mm, uint8_t* zf_valid);

/* v3 (0924CG): 全字段读写, 含折叠位标定。
 * fold_j 输出/输入 relhome 关节角 (deg, 6 轴); fold_valid=1 表示
 * 用户用 `P here` 标定过。fold_j 可传 NULL (只关心其余字段)。
 * Load3 兼容 v3/v2/v1 记录; 旧记录 fold_valid 输出 0。 */
uint8_t GeoCfg_Load3(float* dx, float* dy, float* dz,
                     float* zf_mm, uint8_t* zf_valid,
                     uint8_t* zf_on, uint16_t* margin_raw,
                     float* fold_j6, uint8_t* fold_valid);
uint8_t GeoCfg_Save3(float dx, float dy, float dz,
                     float zf_mm, uint8_t zf_valid,
                     uint8_t zf_on, uint16_t margin_raw,
                     const float* fold_j6, uint8_t fold_valid);

#endif /* GEO_CFG_H */
