#include "pid_flash.h"
#include "flash_cfg.h"      /* 配置扇区保区写入(与 CAN 配置共用 sector 11) */
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* STM32F407 1MB: sector 11 is 0x080E0000~0x080FFFFF, reserved for config.
 * v4: PID + arm params.
 * v5: v4 + joint raw-position limits (PID_JointLimits_t).
 * v6: v5 + Home-pose zero raw offsets (limits.zero).
 *
 * ⚠ 2026-09-17: 扇区 11 现在**不止住这一份记录** —— CAN 通信配置
 *   (arm_can_config.c) 住在同一个扇区的 0x1000 偏移。所以本文件的
 *   擦/写一律通过 CfgFlash_WriteRegion(CFG_OFF_PID, ...) 走保区路径,
 *   否则 `F` 存 PID 会把节点号/波特率一起擦没(反向亦然)。 */
#define PID_FLASH_ADDRESS   (CFG_SECTOR_ADDR + CFG_OFF_PID)
#define PID_FLASH_MAGIC    0x50494431U
#define PID_FLASH_VERSION  6U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint8_t p[6];
    uint8_t i[6];
    uint8_t d[6];
    uint8_t corr_pct[6];
    uint16_t moving_speed;
    uint16_t threshold_cdeg;
    uint8_t max_rounds;
    uint8_t enabled;
    uint8_t arm_param_count;
    uint8_t reserved[3];
    float arm_params[PID_FLASH_ARM_PARAM_MAX];
    PID_JointLimits_t joint_limits;
    uint32_t crc;
} PID_Record_t;

static uint32_t pid_crc(const PID_Record_t *r) {
    const uint8_t *b = (const uint8_t *)r;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t n = 0; n < offsetof(PID_Record_t, crc); n++) {
        crc ^= b[n];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1U)));
    }
    return ~crc;
}

uint8_t PID_FlashLoad(uint8_t p[6], uint8_t i[6], uint8_t d[6],
                      uint8_t corr_pct[6], uint16_t *threshold_cdeg,
                      uint8_t *max_rounds, uint8_t *enabled,
                      uint16_t *moving_speed, float arm_params[],
                      uint8_t arm_param_capacity, uint8_t *params_loaded,
                      PID_JointLimits_t *joint_limits,
                      uint8_t *limits_loaded) {
    const PID_Record_t *r = (const PID_Record_t *)PID_FLASH_ADDRESS;
    if (params_loaded) *params_loaded = 0U;
    if (limits_loaded) *limits_loaded = 0U;
    if (r->magic != PID_FLASH_MAGIC || r->version != PID_FLASH_VERSION ||
        r->length != sizeof(PID_Record_t) || r->crc != pid_crc(r))
        return PID_FLASH_INVALID;
    if (r->arm_param_count > PID_FLASH_ARM_PARAM_MAX ||
        r->arm_param_count > arm_param_capacity)
        return PID_FLASH_INVALID;
    memcpy(p, r->p, 6);
    memcpy(i, r->i, 6);
    memcpy(d, r->d, 6);
    memcpy(corr_pct, r->corr_pct, 6);
    *moving_speed = r->moving_speed;
    *threshold_cdeg = r->threshold_cdeg;
    *max_rounds = r->max_rounds;
    *enabled = r->enabled;
    if (r->arm_param_count > 0U && arm_params) {
        memcpy(arm_params, r->arm_params, r->arm_param_count * sizeof(float));
        if (params_loaded) *params_loaded = 1U;
    }
    if (joint_limits && limits_loaded) {
        memcpy(joint_limits, &r->joint_limits, sizeof(PID_JointLimits_t));
        *limits_loaded = 1U;
    }
    return PID_FLASH_OK;
}

uint8_t PID_FlashSave(const uint8_t p[6], const uint8_t i[6], const uint8_t d[6],
                      const uint8_t corr_pct[6], uint16_t threshold_cdeg,
                      uint8_t max_rounds, uint8_t enabled,
                      uint16_t moving_speed, const float arm_params[],
                      uint8_t arm_param_count,
                      const PID_JointLimits_t *joint_limits) {
    if (arm_param_count > PID_FLASH_ARM_PARAM_MAX ||
        (arm_param_count > 0U && !arm_params)) return PID_FLASH_ERROR;

    PID_Record_t r;
    memset(&r, 0, sizeof(r));
    r.magic = PID_FLASH_MAGIC;
    r.version = PID_FLASH_VERSION;
    r.length = sizeof(PID_Record_t);
    memcpy(r.p, p, 6);
    memcpy(r.i, i, 6);
    memcpy(r.d, d, 6);
    memcpy(r.corr_pct, corr_pct, 6);
    r.moving_speed = moving_speed;
    r.threshold_cdeg = threshold_cdeg;
    r.max_rounds = max_rounds;
    r.enabled = enabled ? 1U : 0U;
    r.arm_param_count = arm_param_count;
    if (arm_param_count > 0U) {
        memcpy(r.arm_params, arm_params, arm_param_count * sizeof(float));
    }
    if (joint_limits) {
        memcpy(&r.joint_limits, joint_limits, sizeof(PID_JointLimits_t));
    }
    r.crc = pid_crc(&r);

    /* 保区写入: 扇区 11 的另一区(CAN 通信配置)会先被搬到 RAM, 擦除后原样
     * 写回, 所以 `F` 不会顺带清掉节点号/波特率。 */
    if (CfgFlash_WriteRegion(CFG_OFF_PID, &r, sizeof(r)) != CFG_FLASH_OK) {
        return PID_FLASH_ERROR;
    }

    const PID_Record_t *saved = (const PID_Record_t *)PID_FLASH_ADDRESS;
    return (saved->magic == PID_FLASH_MAGIC &&
            saved->version == PID_FLASH_VERSION &&
            saved->length == sizeof(PID_Record_t) &&
            saved->crc == pid_crc(saved) &&
            memcmp(saved->p, p, 6) == 0 &&
            memcmp(saved->i, i, 6) == 0 &&
            memcmp(saved->d, d, 6) == 0 &&
            memcmp(saved->corr_pct, corr_pct, 6) == 0 &&
            saved->moving_speed == moving_speed &&
            saved->threshold_cdeg == threshold_cdeg &&
            saved->max_rounds == max_rounds &&
            saved->enabled == (enabled ? 1U : 0U) &&
            saved->arm_param_count == arm_param_count &&
            memcmp(saved->arm_params, arm_params,
                   arm_param_count * sizeof(float)) == 0 &&
            (!joint_limits ||
             memcmp(&saved->joint_limits, joint_limits,
                    sizeof(PID_JointLimits_t)) == 0))
           ? PID_FLASH_OK : PID_FLASH_ERROR;
}

uint8_t PID_FlashErase(void) {
    /* ⚠ 整扇区擦除: CAN 通信配置(同扇区另一区)也会一起没了。
     * 这是"恢复出厂"语义, 调用方需自行提示用户。 */
    return (CfgFlash_EraseAll() == CFG_FLASH_OK) ? PID_FLASH_OK : PID_FLASH_ERROR;
}