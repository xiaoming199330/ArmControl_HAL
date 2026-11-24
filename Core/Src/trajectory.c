/*
 * 六关节同步 S 曲线速度轨迹规划器。
 *
 * 将原来的梯形速度规划升级为七段式 S 曲线（jerk-limited profile），
 * 加速度在过渡段平滑变化而非瞬间跳变，消除机械冲击和"顿挫感"。
 *
 * 七段结构（加速段3段 + 恒速段1段 + 减速段3段）：
 *   段1: jerk 上升  — 加速度 0→a_max，速度抛物线增长
 *   段2: 恒定加速度 — 加速度 = a_max，速度线性增长
 *   段3: jerk 下降  — 加速度 a_max→0，速度抛物线趋于 v_peak
 *   段4: 恒速段     — 加速度 = 0，速度 = v_peak
 *   段5~7: 减速段   — 加速段的镜像
 *
 * S 曲线时间基准固定 200Hz；目标包可按更低频率批量下发，六关节仍按比例同步到达。
 */
#include "trajectory.h"
#include <math.h>
#include <string.h>

/* S 曲线加加速度过渡时间(s)：加速度从 0 升到 a_max 所需的时间。
   值越大越柔和，值越小越接近梯形。0.15s 适合小型舵机机械臂。
   （编译期默认值；运行时可经 arm_params 修改 TRAJ_SCURVE_TJ） */
#define SCURVE_TJ  0.15f

/* 运行时可调 jerk 过渡时间生效点（arm_params 指针绑定）。
 * Trajectory_Plan 使用时钳制到 0.02~0.50s 安全范围。 */
float TRAJ_SCURVE_TJ = SCURVE_TJ;


/* 一条轨迹的全部参数。 */
typedef struct {
    float q0[6], q1[6];   /* 起点、终点角度 */
    float v_peak;          /* 实际峰值速度（短距离时可能 < v_max） */
    float a;               /* 最大加速度 (deg/s²) */
    float j;               /* 加加速度 jerk (deg/s³) = a / tj */
    float tj;              /* jerk 过渡时间 */
    float ta;              /* 恒定加速度段时长（可能为 0） */
    float tv;              /* 恒速段时长（可能为 0） */
    float t_acc;           /* 加速段总时长 = 2*tj + ta */
    float t_total;         /* 轨迹总时长 */
    float d_acc;           /* 加速段行驶距离 */
    float max_d;           /* 六关节中最大位移 */
    uint32_t total_ticks;  /* 总插值步数 (200Hz) */
    uint8_t active;
} Trajectory_t;

static Trajectory_t traj;

/* ===== 加速段位置函数 =====
 * 计算加速段内（t 从 0 到 t_acc）的位移。
 * 减速段通过镜像调用此函数实现：s_dec(t) = max_d - s_acc(t_total - t)。
 */
static float s_acc(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= traj.t_acc) return traj.d_acc;

    float tj = traj.tj;
    float a  = traj.a;
    float j  = traj.j;
    float ta = traj.ta;

    if (t < tj) {
        /* 段1: jerk 上升
         * a(t) = j*t,  v(t) = j*t²/2,  s(t) = j*t³/6 */
        return j * t * t * t / 6.0f;

    } else if (t < tj + ta) {
        /* 段2: 恒定加速度
         * a(t) = a,  v(t) = v1 + a*(t-tj),  s(t) = s1 + v1*t2 + 0.5*a*t2² */
        float t2 = t - tj;
        float v1 = 0.5f * a * tj;         /* 段1结束时的速度 */
        float s1 = a * tj * tj / 6.0f;    /* 段1总距离 */
        return s1 + v1 * t2 + 0.5f * a * t2 * t2;

    } else {
        /* 段3: jerk 下降
         * a(t) = a - j*t3,  v(t) = v2 + a*t3 - j*t3²/2
         * s(t) = s1 + s2 + v2*t3 + 0.5*a*t3² - j*t3³/6 */
        float t3 = t - tj - ta;
        float v1 = 0.5f * a * tj;
        float v2 = v1 + a * ta;           /* 段2结束时的速度 */
        float s1 = a * tj * tj / 6.0f;
        float s2 = v1 * ta + 0.5f * a * ta * ta;
        return s1 + s2 + v2 * t3 + 0.5f * a * t3 * t3 - j * t3 * t3 * t3 / 6.0f;
    }
}

/* ===== 规划一条六关节同步 S 曲线轨迹 =====
 * 以位移最大的关节为基准计算时间，其余关节按比例同步运动。
 * 自动处理三种距离情况：
 *   1. 长距离：完整七段（有恒定加速度段 + 恒速段）
 *   2. 中距离：五段（无恒速段，减小峰值速度）
 *   3. 短距离：三段（无恒定加速度段和恒速段，进一步减小 jerk 时间）
 */
void Trajectory_Plan(float start[6], float end[6], float v_max, float a) {
    memcpy(traj.q0, start, 6 * sizeof(float));
    memcpy(traj.q1, end,   6 * sizeof(float));
    traj.a = a;

    /* 找最大位移关节 */
    float max_d = 0;
    for (uint8_t i = 0; i < 6; i++) {
        float d = fabsf(end[i] - start[i]);
        if (d > max_d) max_d = d;
    }
    if (max_d < 0.01f) {
        traj.active = 0;
        traj.total_ticks = 0;
        return;
    }
    traj.max_d = max_d;

    /* jerk 过渡时间：运行时可调（AP_SC_TJ_S），钳制安全范围 */
    float tj = TRAJ_SCURVE_TJ;
    if (tj < 0.02f) tj = 0.02f;
    if (tj > 0.50f) tj = 0.50f;
    float j  = a / tj;

    /* 检查能否达到 v_max：需要 v_max/a >= tj（即 ta >= 0） */
    if (v_max / a >= tj) {
        /* 正常情况：可以达到 v_max */
        traj.tj     = tj;
        traj.j      = j;
        traj.ta     = v_max / a - tj;
        traj.v_peak = v_max;
    } else {
        /* v_max/a < tj：加速度太小或速度太大，无法在 tj 内加速到 v_max。
           使用三角 S 曲线，峰值速度 = a*tj */
        traj.tj     = tj;
        traj.j      = j;
        traj.ta     = 0;
        traj.v_peak = a * tj;
    }

    /* 计算加速段距离：d_acc = a*tj² + 0.5*a*ta*(3*tj + ta) */
    traj.d_acc  = a * tj * tj + 0.5f * a * traj.ta * (3.0f * tj + traj.ta);
    traj.t_acc  = 2.0f * tj + traj.ta;

    /* 判断是否有足够距离容纳恒速段 */
    if (max_d >= 2.0f * traj.d_acc) {
        /* 情况1: 完整 S 曲线，有恒速段 */
        traj.tv = (max_d - 2.0f * traj.d_acc) / traj.v_peak;
    } else if (traj.ta > 0.0f) {
        /* 情况2: 距离不足以达到 v_peak，需要减小 ta（无恒速段）。
           求解: max_d/2 = a*tj² + 0.5*a*ta'*(3*tj + ta')
           即: 0.5*ta'² + 1.5*tj*ta' - M = 0，其中 M = max_d/(2a) - tj² */
        float M = max_d / (2.0f * a) - tj * tj;
        if (M <= 0.0f) {
            /* 距离太小，连恒定加速度段都不需要 → 情况3 */
            float tj_new = sqrtf(max_d / (2.0f * a));
            traj.tj     = tj_new;
            traj.j      = a / tj_new;
            traj.ta     = 0;
            traj.v_peak = a * tj_new;
            traj.d_acc  = a * tj_new * tj_new;
            traj.t_acc  = 2.0f * tj_new;
        } else {
            float ta_new = -1.5f * tj + sqrtf(2.25f * tj * tj + 2.0f * M);
            traj.ta     = ta_new;
            traj.v_peak = a * (tj + ta_new);
            traj.d_acc  = max_d / 2.0f;
            traj.t_acc  = 2.0f * tj + ta_new;
        }
        traj.tv = 0;
    } else {
        /* 情况3: ta 已为 0，距离太小，需要减小 tj。
           max_d = 2*a*tj'²  →  tj' = sqrt(max_d / (2*a)) */
        float tj_new = sqrtf(max_d / (2.0f * a));
        traj.tj     = tj_new;
        traj.j      = a / tj_new;
        traj.v_peak = a * tj_new;
        traj.d_acc  = a * tj_new * tj_new;
        traj.t_acc  = 2.0f * tj_new;
        traj.tv     = 0;
    }

    traj.t_total     = 2.0f * traj.t_acc + traj.tv;
    traj.total_ticks = (uint32_t)(traj.t_total * 200.0f) + 1;
    traj.active      = 1;
}

/* ===== 插值：根据 tick 计算六关节当前目标角度 =====
 * 减速段通过镜像加速段实现：s(t) = max_d - s_acc(t_total - t)
 */
void Trajectory_Interpolate(uint32_t tick, float out_angles[6]) {
    if (!traj.active || tick >= traj.total_ticks) {
        memcpy(out_angles, traj.q1, 6 * sizeof(float));
        if (tick >= traj.total_ticks) traj.active = 0;
        return;
    }

    float t = (float)tick / 200.0f;
    float dist;

    if (t < traj.t_acc) {
        /* 加速段 */
        dist = s_acc(t);
    } else if (t < traj.t_acc + traj.tv) {
        /* 恒速段 */
        dist = traj.d_acc + traj.v_peak * (t - traj.t_acc);
    } else if (t < traj.t_total) {
        /* 减速段 = 加速段的镜像 */
        float t_rem = traj.t_total - t;
        dist = traj.max_d - s_acc(t_rem);
    } else {
        dist = traj.max_d;
    }

    /* 按比例分配到六个关节 */
    float ratio = (traj.max_d > 0.001f) ? (dist / traj.max_d) : 1.0f;
    for (uint8_t i = 0; i < 6; i++) {
        out_angles[i] = traj.q0[i] + (traj.q1[i] - traj.q0[i]) * ratio;
    }
}

uint8_t Trajectory_IsActive(void) {
    return traj.active;
}

/* ===== 导出当前轨迹的终点目标 (尾段交接用) =====
 * 2026-09-07: 微步流式下发会让舵机内部规划反复重启, 尾段爬行出台阶;
 * 主循环在所有关节距终点 < handoff_deg 时改为一次性写入终点,
 * 让舵机像单关节 'a' 指令那样用自己的 profile 平滑收尾。 */
void Trajectory_Target(float out[6]) {
    memcpy(out, traj.q1, 6 * sizeof(float));
}

uint32_t Trajectory_TotalTicks(void) {
    return traj.total_ticks;
}

void Trajectory_Stop(void) {
    traj.active = 0;
}
