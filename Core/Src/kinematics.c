/* ============================================================
 * kinematics.c - FK/IK for SO-ARM101 style 6-DOF Arm
 *
 * 构型 (2026-08-31 实测):
 *   J1: Yaw   - 底座绕竖直 Z 轴水平旋转
 *   J2: Pitch - 肩 (竖直平面)
 *   J3: Pitch - 肘
 *   J4: Pitch - 腕俯仰
 *   J5: Roll  - 腕自转, 绕夹爪轴 (不改变末端位置)
 *   J6: 夹爪  (不改变末端位置)
 *
 * World frame: X=forward, Y=left, Z=up, 原点=桌面上的 J1 轴心
 *
 * 连杆方向角约定 psi (弧度, 臂竖直平面内):
 *   u(psi) = (sin psi, cos psi)  -- psi=0 竖直向上, psi>0 向前倾
 *   J2->J3 : psi = PSI_UPPER0 + t2
 *   J3->J4 : psi = PSI_FORE0  + t2 + t3
 *   J4->TCP: psi = PSI_FORE0  + t2 + t3 + t4  (J5 轴=前臂轴线, 工具沿前臂)
 *
 * 角度 0 = 中性位 (全部舵机 raw 2048 / 180°, = 官网 Home:
 *   上臂近竖直 +13.97°, 前臂水平 +87.79°)
 *
 * IK 策略:
 *   1) J1 = atan2(y, x) 方位角直接解
 *   2) J2/J3/J4: 3 关节平面 2D 目标 -> 1 个冗余自由度用于满足工具俯仰:
 *      先解析解 (俯仰精确满足, 肘上/肘下两种姿态), 限位不可行时
 *      CCD 多起点兜底 (只保位置, 俯仰自由)
 * ============================================================ */

#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================================
 * 运行时可调 IK-CCD 参数（arm_params 指针绑定的生效点）
 * ============================================================ */
float KIN_CCD_CONVERGE_MM = 0.5f;    /* CCD 收敛阈值 mm */
int   KIN_CCD_MAX_ITER    = 100;     /* CCD 单起点最大迭代次数 */
float KIN_IK_FAIL_MM      = 1.0f;    /* IK 失败判定阈值 mm */
float KIN_FK_VERIFY_MM    = 5.0f;    /* FK 回代验证容差 mm */

/* ============================================================
 * World frame calibration offset (mm)
 * ============================================================ */
float WORLD_OFFSET_X = 0.0f;
float WORLD_OFFSET_Y = 0.0f;
float WORLD_OFFSET_Z = 0.0f;

void Kinematics_SetOffset(float dx, float dy, float dz) {
    WORLD_OFFSET_X = dx;
    WORLD_OFFSET_Y = dy;
    WORLD_OFFSET_Z = dz;
}

/* ============================================================
 * Per-joint limits (radians, set from main.c via IK_SetJointLimits)
 * Default: full servo range
 * ============================================================ */
static float jlim_min[6] = {
    -150.0f*M_PI/180.0f, -150.0f*M_PI/180.0f, -150.0f*M_PI/180.0f,
    -150.0f*M_PI/180.0f, -150.0f*M_PI/180.0f, -150.0f*M_PI/180.0f
};
static float jlim_max[6] = {
     150.0f*M_PI/180.0f,  150.0f*M_PI/180.0f,  150.0f*M_PI/180.0f,
     150.0f*M_PI/180.0f,  150.0f*M_PI/180.0f,  150.0f*M_PI/180.0f
};

void IK_SetJointLimits(const float min_deg[6], const float max_deg[6]) {
    for (int i = 0; i < 6; i++) {
        /* DIR=-1 的关节, 限位换算后 min/max 顺序翻转, 取 min/max 归一 */
        float a = JOINT_DIR[i] * (min_deg[i] - SERVO_OFFSET[i]) * M_PI / 180.0f;
        float b = JOINT_DIR[i] * (max_deg[i] - SERVO_OFFSET[i]) * M_PI / 180.0f;
        jlim_min[i] = (a < b) ? a : b;
        jlim_max[i] = (a > b) ? a : b;
    }
}

/* Raw (0-4095) distance travelling forward (increasing raw) from 'from' to 'to',
 * modulo 4096.  Used to handle limits that wrap across 0/360. */
static inline uint16_t raw_dist_fwd(uint16_t from, uint16_t to) {
    return (uint16_t)((to + 4096u - from) % 4096u);
}

void IK_SetJointLimitsRaw(const uint16_t min_raw[6], const uint16_t max_raw[6],
                          const float zero_offset_deg[6]) {
    for (int i = 0; i < 6; i++) {
        uint16_t mn = min_raw[i];
        uint16_t mx = max_raw[i];
        uint16_t span = raw_dist_fwd(mn, mx);    /* forward distance mn -> mx */
        if (span == 0) continue;                 /* uncalibrated / single point */

        float off = zero_offset_deg[i];
        float span_deg = (float)span * 360.0f / 4095.0f;

        /* 沿 raw 正向弧连续展开: mx 的连续 servo 角 = mn 的 servo 角 + span_deg
         * (可 >360), 再统一折算到关节角, 保证区间连续、不分裂到错误分支。
         * 2026-09-02 修复: 旧实现直接用 servo_raw_to_deg(mx) 的 [0,360) 分支,
         * 对跨 0/360 的包裹弧 (如 J3: 合法带 = servo 10~177 之外的 193° 弧) 会
         * 掉进远离 IK 解的分支 (把 [-106,+87] 算成 [-466,-273]), 限位形同虚设。 */
        float deg_mn_servo = servo_raw_to_deg(mn);
        float deg_mx_servo = deg_mn_servo + span_deg;   /* 连续, 可 > 360 */
        float deg_a = JOINT_DIR[i] * (deg_mn_servo - off);
        float deg_b = JOINT_DIR[i] * (deg_mx_servo - off);

        jlim_min[i] = (deg_a < deg_b ? deg_a : deg_b) * M_PI / 180.0f;
        jlim_max[i] = (deg_a < deg_b ? deg_b : deg_a) * M_PI / 180.0f;
    }
}

/* ============================================================
 * Conversion utilities
 * ============================================================ */

/* 关节零位 (舵机度): 默认 180 (= raw 2048 / 官方出厂中位)。
 * 联轴装配与 2048 不对齐的臂通过 Kinematics_SetZeroRaw 以官方 Home 姿态重锚定。 */
float SERVO_OFFSET[6] = { 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f };

void Kinematics_SetZeroRaw(const uint16_t raw[6]) {
    uint8_t any = 0;
    for (int j = 0; j < 6; j++) {
        if (raw[j] >= 4096U) continue;      /* 非法值忽略 */
        if (raw[j] == 0U) continue;         /* 0 = 未校准占位 */
        SERVO_OFFSET[j] = raw[j] / 4095.0f * 360.0f;
        any = 1;
    }
    if (!any) {                             /* 全 0 -> 恢复默认中位 */
        for (int j = 0; j < 6; j++) SERVO_OFFSET[j] = 180.0f;
    }
}

/* Servo degrees (0-360) -> kinematics radians (0=center, per-joint offset + dir) */
float servo_deg_to_rad(float deg, int joint) {
    return JOINT_DIR[joint] * (deg - SERVO_OFFSET[joint]) * M_PI / 180.0f;
}

/* Kinematics radians (0=center) -> servo degrees (0-360, per-joint offset + dir) */
float rad_to_servo_deg(float rad, int joint) {
    float deg = SERVO_OFFSET[joint] + JOINT_DIR[joint] * rad * 180.0f / M_PI;
    if (deg < SERVO_MIN_DEG) deg = SERVO_MIN_DEG;
    if (deg > SERVO_MAX_DEG) deg = SERVO_MAX_DEG;
    return deg;
}

/* Servo degrees (0-360) -> STS3215 raw position (0-4095) */
uint16_t deg_to_servo_raw(float deg) {
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 360.0f) deg = 360.0f;
    return (uint16_t)(deg / 360.0f * 4095.0f + 0.5f);
}

/* STS3215 raw position (0-4095) -> servo degrees (0-360) */
float servo_raw_to_deg(uint16_t raw) {
    return (float)raw / 4095.0f * 360.0f;
}

/* ============================================================
 * 平面臂公共计算: 连杆长度/中性方向角 (J2 起 3 段)
 * link0 = J2->J3 (上臂), link1 = J3->J4 (前臂), link2 = J4->TCP (腕+工具)
 * ============================================================ */
static const float KIN_LEN[3]  = { ROBOT_L_UPPER, ROBOT_L_FORE, ROBOT_L_DISTAL };
static const float KIN_BASE[3] = { ROBOT_PSI_UPPER0, ROBOT_PSI_FORE0, ROBOT_PSI_DISTAL0 };

/* 平面末端坐标 (相对 J2 轴): (前向 y, 高度 z) */
static void planar_fk(const float ang[3], float *py, float *pz) {
    float s = 0.0f, y = 0.0f, z = 0.0f;
    for (int i = 0; i < 3; i++) {
        s += ang[i];                       /* 累计关节角 t2..t(i+1) */
        y += KIN_LEN[i] * sinf(KIN_BASE[i] + s);
        z += KIN_LEN[i] * cosf(KIN_BASE[i] + s);
    }
    *py = y;
    *pz = z;
}

/* ============================================================
 * Forward Kinematics
 *
 * joints[0] = J1 yaw (方位角, 0 = 朝 +X)
 * joints[1..3] = J2/J3/J4 pitch (0 = 中性位)
 * joints[4] = J5 roll (绕工具轴, 不改变位置)
 * joints[5] = J6 夹爪 (不改变位置)
 * ============================================================ */

int FK_Solve(const float joints[NUM_JOINTS], Kinematics_Pos *result) {
    float t1 = joints[0];
    float ang[3] = { joints[1], joints[2], joints[3] };

    float py, pz;
    planar_fk(ang, &py, &pz);

    /* J2 轴在臂平面内偏移 (J1 轴 -> J2 轴) */
    py += ROBOT_J2_OFF_Y;
    pz += ROBOT_BASE_H + ROBOT_J2_OFF_Z;

    /* 世界坐标 (含标定偏移) */
    result->x   = py * cosf(t1) + WORLD_OFFSET_X;
    result->y   = py * sinf(t1) + WORLD_OFFSET_Y;
    result->z   = pz + WORLD_OFFSET_Z;

    /* 工具轴方向 = 工具连杆方向 psi_C = PSI_FORE0 + t2+t3+t4
     * (J5 自转不改变方向; 0=竖直向上, 90°=水平向前, 180°=向下) */
    result->yaw = ROBOT_PSI_FORE0 + joints[1] + joints[2] + joints[3];

    return 1;
}

/* ============================================================
 * 解析法 IK (J2/J3/J4): 工具俯仰已知 -> J4 位置唯一 -> 2R 闭式解
 * ty/tz = 目标相对 J2 的平面坐标; tool_pitch = 工具轴方向角
 * 肘上/肘下两种姿态都试, 取限位内且 FK 误差最小者
 * 成功返回 1, ang = {t2, t3, t4}
 * ============================================================ */
static int ik_analytic(float ty, float tz, float tool_pitch, float ang[3]) {
    /* J4 轴位置 = 目标 - L_DISTAL * u(tool_pitch) */
    float j4y = ty - ROBOT_L_DISTAL * sinf(tool_pitch);
    float j4z = tz - ROBOT_L_DISTAL * cosf(tool_pitch);
    float d = sqrtf(j4y * j4y + j4z * j4z);

    if (d < 1e-3f) return 0;
    if (d > ROBOT_L_UPPER + ROBOT_L_FORE + 0.01f) return 0;
    if (d < fabsf(ROBOT_L_UPPER - ROBOT_L_FORE)) return 0;

    /* J2 处三角形内角 (J2->J4 与上臂夹角) */
    float cosb = (ROBOT_L_UPPER * ROBOT_L_UPPER + d * d - ROBOT_L_FORE * ROBOT_L_FORE)
               / (2.0f * ROBOT_L_UPPER * d);
    if (cosb >  1.0f) cosb =  1.0f;
    if (cosb < -1.0f) cosb = -1.0f;
    float beta  = acosf(cosb);
    float gamma = atan2f(j4y, j4z);      /* J2 -> J4 方向的 psi 角 */

    int found = 0;
    float best[3] = { 0, 0, 0 };
    float best_err = 1e10f;

    for (int side = 0; side <= 1; side++) {
        float psa = gamma + (side ? -beta : beta);          /* 上臂方向 */
        float j3y = ROBOT_L_UPPER * sinf(psa);
        float j3z = ROBOT_L_UPPER * cosf(psa);
        float psb = atan2f(j4y - j3y, j4z - j3z);           /* 前臂方向 */

        float t2 = psa - ROBOT_PSI_UPPER0;
        float t3 = (psb - ROBOT_PSI_FORE0) - t2;
        /* 工具方向 psi_C = PSI_FORE0 + t2+t3+t4 = tool_pitch */
        float t4 = tool_pitch - ROBOT_PSI_FORE0 - (t2 + t3);

        /* 限位检查 (J2/J3/J4 = jlim[1..3]) */
        if (t2 < jlim_min[1] || t2 > jlim_max[1]) continue;
        if (t3 < jlim_min[2] || t3 > jlim_max[2]) continue;
        if (t4 < jlim_min[3] || t4 > jlim_max[3]) continue;

        /* FK 误差校验 (解析解应为数值精度级) */
        float cand[3] = { t2, t3, t4 };
        float py, pz;
        planar_fk(cand, &py, &pz);
        float err = sqrtf((py - ty) * (py - ty) + (pz - tz) * (pz - tz));
        if (err > KIN_IK_FAIL_MM) continue;

        if (err < best_err) {
            best_err = err;
            best[0] = t2; best[1] = t3; best[2] = t4;
            found = 1;
        }
    }

    if (!found) return 0;
    ang[0] = best[0]; ang[1] = best[1]; ang[2] = best[2];
    return 1;
}


/* ============================================================
 * CCD: 平面 3 关节 (J2/J3/J4) 位置 IK, 多起点由调用方驱动
 * 返回最终位置误差 (mm)
 * ============================================================ */
static float ccd_run(float ang[3], float ty, float tz) {
    for (int iter = 0; iter < KIN_CCD_MAX_ITER; iter++) {
        /* 当前末端 */
        float ey, ez;
        planar_fk(ang, &ey, &ez);

        float dy = ty - ey, dz = tz - ez;
        if (sqrtf(dy * dy + dz * dz) < KIN_CCD_CONVERGE_MM)
            break;

        /* 从末端关节到根部逐个调整 */
        for (int j = 2; j >= 0; j--) {
            /* 关节 j 轴心位置 = 前 j 条连杆末端 */
            float qy = 0.0f, qz = 0.0f;
            {
                float s = 0.0f;
                for (int i = 0; i < j; i++) {
                    s += ang[i];
                    qy += KIN_LEN[i] * sinf(KIN_BASE[i] + s);
                    qz += KIN_LEN[i] * cosf(KIN_BASE[i] + s);
                }
            }

            /* 当前末端 (本轮回合内前面关节可能已调整) */
            float ey2, ez2;
            planar_fk(ang, &ey2, &ez2);

            /* 关节->末端 与 关节->目标 向量 */
            float ver = ey2 - qy, vez = ez2 - qz;
            float vtr = ty  - qy, vtz = tz  - qz;

            float ve_len = sqrtf(ver * ver + vez * vez);
            float vt_len = sqrtf(vtr * vtr + vtz * vtz);
            if (ve_len < 0.001f || vt_len < 0.001f)
                continue;

            float dot   = (ver * vtr + vez * vtz) / (ve_len * vt_len);
            float cross = (ver * vtz - vez * vtr) / (ve_len * vt_len);
            if (dot > 1.0f) dot = 1.0f;
            if (dot < -1.0f) dot = -1.0f;

            float delta = atan2f(cross, dot);
            ang[j] -= delta;   /* u(psi)=(sin,cos) 约定: -delta 转向目标 */

            /* 限位 (J2/J3/J4 = jlim[1..3]) */
            if (ang[j] < jlim_min[j + 1]) ang[j] = jlim_min[j + 1];
            if (ang[j] > jlim_max[j + 1]) ang[j] = jlim_max[j + 1];
        }
    }

    float ey, ez;
    planar_fk(ang, &ey, &ez);
    return sqrtf((ty - ey) * (ty - ey) + (tz - ez) * (tz - ez));
}

/* Smallest angular difference on a circle (radians), always in [0, pi]. */
static inline float circle_delta(float a, float b) {
    float d = fabsf(a - b);
    while (d > 2.0f * M_PI) d -= 2.0f * M_PI;
    if (d > M_PI) d = 2.0f * M_PI - d;
    return d;
}

/* 诊断: 导出当前 jlim (弧度 -> 度), 供 `zi` 核对 IK 限位与 SERVO_OFFSET 是否错位。 */
void IK_GetJointLimitsDeg(float min_deg[6], float max_deg[6]) {
    for (int i = 0; i < 6; i++) {
        min_deg[i] = jlim_min[i] * 180.0f / (float)M_PI;
        max_deg[i] = jlim_max[i] * 180.0f / (float)M_PI;
    }
}

/* ============================================================
 * Inverse Kinematics
 *
 * Step 1: J1 = atan2(y, x) (方位角直接解, 限位内)
 * Step 2: 平面目标 = 目标在臂平面投影 - J2 偏移
 * Step 3: 解析解 (俯仰优先) -> 失败则 CCD 多起点兜底 (俯仰自由)
 * J5/J6 输出 0 (中性); 调用方负责保持当前角度
 * ============================================================ */
int IK_Solve(const Kinematics_Pos *target, float joints[NUM_JOINTS]) {
    /* 去标定偏移 -> 内部模型目标 */
    float x = target->x - WORLD_OFFSET_X;
    float y = target->y - WORLD_OFFSET_Y;
    float z = target->z - WORLD_OFFSET_Z;
    float tool_pitch = target->yaw;

    /* --- Step 1: J1 方位角 --- */
    float theta1 = atan2f(y, x);
    if (theta1 < jlim_min[0]) theta1 = jlim_min[0];
    if (theta1 > jlim_max[0]) theta1 = jlim_max[0];
    joints[0] = theta1;

    /* --- Step 2: 平面目标 (相对 J2 轴) --- */
    float rho = sqrtf(x * x + y * y);            /* 目标到 J1 轴水平距离 */
    float ty = rho - ROBOT_J2_OFF_Y;
    float tz = z - (ROBOT_BASE_H + ROBOT_J2_OFF_Z);

    float planar_dist = sqrtf(ty * ty + tz * tz);
    if (planar_dist > ARM_MAX_REACH + 0.5f) {
        return 0;   /* 超出 J2 平面臂展 */
    }

    /* --- Step 3a: 解析解 (俯仰优先) --- */
    float ang[3];
    if (ik_analytic(ty, tz, tool_pitch, ang)) {
        joints[1] = ang[0];
        joints[2] = ang[1];
        joints[3] = ang[2];
        joints[4] = 0.0f;
        joints[5] = 0.0f;
        return 1;
    }

    /* --- Step 3b: CCD 多起点兜底 (位置优先, 俯仰自由) --- */
    float aim = atan2f(ty, tz);                  /* 目标方向 psi 角 */
    float fold = 0.0f;
    if (planar_dist < ARM_MAX_REACH && planar_dist > 0.1f) {
        float ratio = planar_dist / ARM_MAX_REACH;
        if (ratio > 1.0f) ratio = 1.0f;
        fold = acosf(ratio) * 0.5f;
    }

    float starts[4][3] = {
        { 0.0f,          0.0f,          0.0f          }, /* 中性竖直 */
        { aim * 0.5f,    0.0f,          0.0f          }, /* 朝目标倾 */
        { fold,          fold,          -fold         }, /* S 形折叠 */
        { aim * 0.3f,    aim * 0.3f,    aim * 0.4f    }  /* 均匀前倾 */
    };

    float best_ang[3] = { 0, 0, 0 };
    float best_err = 1e10f;

    for (int si = 0; si < 4; si++) {
        float a[3];
        for (int i = 0; i < 3; i++) {
            a[i] = starts[si][i];
            if (a[i] < jlim_min[i + 1]) a[i] = jlim_min[i + 1];
            if (a[i] > jlim_max[i + 1]) a[i] = jlim_max[i + 1];
        }

        float err = ccd_run(a, ty, tz);
        if (err < best_err) {
            best_err = err;
            best_ang[0] = a[0]; best_ang[1] = a[1]; best_ang[2] = a[2];
        }

        if (err < KIN_CCD_CONVERGE_MM)
            break;
    }

    joints[1] = best_ang[0];
    joints[2] = best_ang[1];
    joints[3] = best_ang[2];
    joints[4] = 0.0f;
    joints[5] = 0.0f;

    if (best_err > KIN_IK_FAIL_MM) {
        return 0;
    }
    return 1;
}
