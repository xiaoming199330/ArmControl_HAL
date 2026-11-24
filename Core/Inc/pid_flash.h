#ifndef PID_FLASH_H
#define PID_FLASH_H

#include <stdint.h>

#define PID_FLASH_OK       0
#define PID_FLASH_INVALID  1
#define PID_FLASH_ERROR    2

/* Maximum number of runtime algorithm parameters stored with one record. */
#define PID_FLASH_ARM_PARAM_MAX  24U

/* Joint raw-position limits (0..4095) saved alongside PID/correction params.
 * Restored at boot as defaults for the interactive calibration tool. */
typedef struct {
    uint16_t min[6];   /* servo raw min edge */
    uint16_t pos[6];   /* servo raw nominal/center pose */
    uint16_t max[6];   /* servo raw max edge */
    uint16_t zero[6];  /* servo raw at official Home pose = kinematics zero offset.
                        * 0 = not calibrated (fall back to 180 deg / raw 2048). */
} PID_JointLimits_t;

/* Load PID, correction, Moving Speed, optional ArmParams, and joint limits.
 * joint_limits may be NULL if caller does not need limits.
 * limits_loaded=1 means joint_limits was restored; 0 means legacy record. */
uint8_t PID_FlashLoad(uint8_t p[6], uint8_t i[6], uint8_t d[6],
                      uint8_t corr_pct[6], uint16_t *threshold_cdeg,
                      uint8_t *max_rounds, uint8_t *enabled,
                      uint16_t *moving_speed, float arm_params[],
                      uint8_t arm_param_capacity, uint8_t *params_loaded,
                      PID_JointLimits_t *joint_limits,
                      uint8_t *limits_loaded);

/* Save PID, correction, Moving Speed, ArmParams and joint limits to Flash.
 * joint_limits may be NULL to skip writing the limits portion. */
uint8_t PID_FlashSave(const uint8_t p[6], const uint8_t i[6], const uint8_t d[6],
                      const uint8_t corr_pct[6], uint16_t threshold_cdeg,
                      uint8_t max_rounds, uint8_t enabled,
                      uint16_t moving_speed, const float arm_params[],
                      uint8_t arm_param_count,
                      const PID_JointLimits_t *joint_limits);

/* Erase saved gains; next boot falls back to firmware defaults. */
uint8_t PID_FlashErase(void);

#endif