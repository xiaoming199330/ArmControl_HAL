/* ============================================================
 * arm_params.c - 三大算法统一配置参数表（实现）
 *
 * 指针绑定：每个表项的 ptr 指向“实际生效变量”（定义在
 * main.c / kinematics.c / trajectory.c / 本文件），Set/Get
 * 直接读写生效点，全工程无双份拷贝。
 *
 * 生效变量分布:
 *   kinematics.c : KIN_CCD_CONVERGE_MM / KIN_CCD_MAX_ITER /
 *                  KIN_IK_FAIL_MM / KIN_FK_VERIFY_MM
 *   trajectory.c : TRAJ_SCURVE_TJ
 *   main.c       : GRAVITY_COMP_K / correction_enabled /
 *                  corr_threshold / corr_max_rounds / corr_gain_init[]
 *   本文件       : ARM_SC_* （S 曲线默认速度/加速度等）
 * ============================================================ */
#include "arm_params.h"
#include "arm_main_exports.h"
#include "kinematics.h"
#include "trajectory.h"
#include "servo_driver.h"

/* ---- S 曲线默认参数生效变量（main.c 运动命令默认值引用） ---- */
float ARM_SC_DEF_VMAX     = 40.0f;   /* deg/s  ⭐0924DA: 用户定案 SP 17 40 为默认 */
float ARM_SC_DEF_ACCEL    = 80.0f;   /* deg/s^2 */

float ARM_SC_V_MIN        = 5.0f;    /* deg/s  */
float ARM_SC_V_MAX        = 300.0f;  /* deg/s  */
float ARM_SC_ACCEL_FACTOR = 2.0f;    /* a = v * factor */

/* ---- 参数元数据表（id 升序；新增参数在分组内追加） ---- */
static const ArmParamMeta ap_table[] = {
    /* --- IK-CCD 运动学 --- */
    { AP_IK_CONVERGE_MM,  "ik.converge_mm",  AP_T_FLOAT, &KIN_CCD_CONVERGE_MM, 0.1f,  5.0f,  0.5f,  "mm"     },
    { AP_IK_MAX_ITER,     "ik.max_iter",     AP_T_INT,   &KIN_CCD_MAX_ITER,    10.0f, 500.0f, 100.0f,"次"     },
    { AP_IK_FAIL_MM,      "ik.fail_mm",      AP_T_FLOAT, &KIN_IK_FAIL_MM,      0.5f,  10.0f,  1.0f,  "mm"    },
    { AP_IK_FK_VERIFY_MM, "ik.fk_verify_mm", AP_T_FLOAT, &KIN_FK_VERIFY_MM,    1.0f,  20.0f,  5.0f,  "mm"    },

    /* --- 七段 S 曲线轨迹 --- */
    { AP_SC_TJ_S,         "sc.tj",           AP_T_FLOAT, &TRAJ_SCURVE_TJ,      0.02f, 0.50f,  0.15f, "s"     },
    { AP_SC_DEF_VMAX,     "sc.def_vmax",     AP_T_FLOAT, &ARM_SC_DEF_VMAX,     5.0f,  300.0f, 40.0f, "deg/s" },
    { AP_SC_DEF_ACCEL,    "sc.def_accel",    AP_T_FLOAT, &ARM_SC_DEF_ACCEL,    10.0f, 600.0f, 80.0f, "deg/s2"},
    { AP_SC_V_MIN,        "sc.v_min",        AP_T_FLOAT, &ARM_SC_V_MIN,        1.0f,  50.0f,  5.0f,  "deg/s" },
    { AP_SC_V_MAX,        "sc.v_max",        AP_T_FLOAT, &ARM_SC_V_MAX,        50.0f, 300.0f, 300.0f,"deg/s" },
    { AP_SC_ACCEL_FACTOR, "sc.accel_factor", AP_T_FLOAT, &ARM_SC_ACCEL_FACTOR, 1.0f,  4.0f,   2.0f,  "-"     },

    /* --- 重力补偿 + 自适应修正 --- */
    { AP_GC_K,            "gc.k",            AP_T_FLOAT, &GRAVITY_COMP_K,      0.0f,  0.5f,   0.0f,  "deg/mm"},
    { AP_CORR_ENABLED,    "corr.enabled",    AP_T_U8,    &correction_enabled,  0.0f,  1.0f,   0.0f,  "0/1"   },  /* ⭐0924DA: 默认 0 */
    { AP_CORR_THRESHOLD,  "corr.threshold",  AP_T_FLOAT, &corr_threshold,      0.5f,  10.0f,  2.0f,  "deg"   },
    { AP_CORR_MAX_ROUNDS, "corr.max_rounds", AP_T_U8,    &corr_max_rounds,     1.0f,  20.0f,  10.0f, "轮"    },
    { AP_CORR_GAIN_J1,    "corr.gain_j1",    AP_T_FLOAT, &corr_gain_init[0],   0.0f,  0.85f,  0.20f, "0~0.85"},
    { AP_CORR_GAIN_J2,    "corr.gain_j2",    AP_T_FLOAT, &corr_gain_init[1],   0.0f,  0.85f,  0.50f, "0~0.85"},
    { AP_CORR_GAIN_J3,    "corr.gain_j3",    AP_T_FLOAT, &corr_gain_init[2],   0.0f,  0.85f,  0.30f, "0~0.85"},
    { AP_CORR_GAIN_J4,    "corr.gain_j4",    AP_T_FLOAT, &corr_gain_init[3],   0.0f,  0.85f,  0.20f, "0~0.85"},
    { AP_CORR_GAIN_J5,    "corr.gain_j5",    AP_T_FLOAT, &corr_gain_init[4],   0.0f,  0.85f,  0.20f, "0~0.85"},
    { AP_CORR_GAIN_J6,    "corr.gain_j6",    AP_T_FLOAT, &corr_gain_init[5],   0.0f,  0.85f,  0.20f, "0~0.85"},

    /* --- 轨迹收尾降速（2026-09-07 运行时化, 0924AP 起保留参数但不再生效） ---
     * 0924AP: 移除收尾降速与尾段交接算法, 这些值仅保留给 LP/SP 兼容显示。 */
    { AP_TRAJ_APPROACH_SPEED, "traj.approach_speed", AP_T_FLOAT, &TRAJ_APPROACH_SPEED, 0.0f, 4095.0f, 5.0f, "raw"   },
    { AP_TRAJ_APPROACH_FRAC,  "traj.approach_frac",  AP_T_FLOAT, &TRAJ_APPROACH_FRAC,  0.0f, 0.25f,   0.10f, "比例"  },

    /* --- 舵机内部加速度（2026-09-07 运行时化, 0924AP 恢复单一值） ---
     * 默认 50: 兼顾轨迹跟随与流控负载轴稳定, 不再 split/切换。
     * SP 44 调全局加速度; 改后 'i' 重新下发或复位生效。 */
    { AP_SERVO_ACCEL, "servo.accel", AP_T_U8, &sts_accel_value, 0.0f, 254.0f, 50.0f, "步/s2" },

    /* --- 轨迹尾段交接（2026-09-07 新增, 2026-09-10 默认由 0 改为 4, 同日改回 0） ---
     * 所有关节距终点 < handoff_deg 时终点一次写入交舵机 profile 收尾。
     * 2026-09-10 结论: 回折叠位临近终点的剧烈颤抖根因是 **J3 PID 被改坏**
     * (P50/I2/D30, D 由 12 放大 2.5×), 恢复默认即消失 —— 非收尾机制所致。
     * 2026-09-10 晚: 交接段速度已由"常速(all_servo_moving_speed)"改为
     * **TRAJ_APPROACH_SPEED(#42)** —— 与斜坡末速衔接连续, 不再"满速冲最后
     * handoff_deg 度"; 要调交接速度就调 SP 42, 无需改 V。
     * 启用建议: `SP 45 4` + `SP 42 20~40`(≈88~176 °/s)。
     * 0=关闭(退回纯流式尾段)。 */
    { AP_TRAJ_HANDOFF_DEG, "traj.handoff_deg", AP_T_FLOAT, &TRAJ_HANDOFF_DEG, 0.0f, 15.0f, 0.0f, "deg" },
};

#define AP_TABLE_LEN  (sizeof(ap_table) / sizeof(ap_table[0]))

const ArmParamMeta* ArmParams_Table(void) { return ap_table; }
uint8_t             ArmParams_Count(void) { return (uint8_t)AP_TABLE_LEN; }

const ArmParamMeta* ArmParams_Find(uint8_t id) {
    for (uint8_t i = 0; i < AP_TABLE_LEN; i++) {
        if (ap_table[i].id == id) return &ap_table[i];
    }
    return 0;
}

/* 经指针写入生效点（按类型转换） */
static void ap_write(const ArmParamMeta* m, float value) {
    switch (m->type) {
        case AP_T_FLOAT: *(float*)m->ptr    = value;                 break;
        case AP_T_INT:   *(int*)m->ptr      = (int)(value + 0.5f);   break;
        case AP_T_U8:    *(uint8_t*)m->ptr  = (uint8_t)(value + 0.5f); break;
    }
}

static float ap_read(const ArmParamMeta* m) {
    switch (m->type) {
        case AP_T_FLOAT: return *(const float*)m->ptr;
        case AP_T_INT:   return (float)*(const int*)m->ptr;
        case AP_T_U8:    return (float)*(const uint8_t*)m->ptr;
    }
    return 0.0f;
}

int ArmParams_Set(uint8_t id, float value) {
    const ArmParamMeta* m = ArmParams_Find(id);
    if (!m) return -1;
    if (value < m->vmin) value = m->vmin;
    if (value > m->vmax) value = m->vmax;
    ap_write(m, value);
    return 0;
}

int ArmParams_Get(uint8_t id, float* out_value) {
    const ArmParamMeta* m = ArmParams_Find(id);
    if (!m || !out_value) return -1;
    *out_value = ap_read(m);
    return 0;
}

void ArmParams_Reset(void) {
    for (uint8_t i = 0; i < AP_TABLE_LEN; i++) {
        ap_write(&ap_table[i], ap_table[i].vdef);
    }
}

uint8_t ArmParams_Snapshot(float out_values[], uint8_t capacity) {
    if (!out_values || capacity < AP_TABLE_LEN) return 0U;
    for (uint8_t i = 0; i < AP_TABLE_LEN; i++) {
        out_values[i] = ap_read(&ap_table[i]);
    }
    return (uint8_t)AP_TABLE_LEN;
}

uint8_t ArmParams_Restore(const float values[], uint8_t count) {
    if (!values || count != AP_TABLE_LEN) return 0U;
    for (uint8_t i = 0; i < AP_TABLE_LEN; i++) {
        if (ArmParams_Set(ap_table[i].id, values[i]) != 0) return 0U;
    }
    return (uint8_t)AP_TABLE_LEN;
}

/* 只初始化“新增算法变量”（IK/S 曲线，ID<32），不触碰 main.c 中
 * 由 pid_flash 管理的修正参数（Flash 值优先，启动时序安全）。
 * 调用点：main() 中 pid_load_startup() 之前或之后均可。 */
void ArmParams_Init(void) {
    for (uint8_t i = 0; i < AP_TABLE_LEN; i++) {
        if (ap_table[i].id < AP_GC_K) {
            ap_write(&ap_table[i], ap_table[i].vdef);
        }
    }
}
