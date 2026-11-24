/* ============================================================
 * arm_control_api.c - 机械臂统一控制接口（实现）
 *
 * 供串口协议 / CAN 协议 / 本地命令调用。全部函数仅在主循环
 * 上下文执行（协议 dispatch 由主循环驱动），可安全进行舵机
 * 总线通信与阻塞打印。
 * ============================================================ */
#include "arm_control_api.h"
#include "arm_main_exports.h"
#include "arm_params.h"
#include "arm_runlog.h"
#include "kinematics.h"
#include "trajectory.h"
#include "servo_driver.h"
#include <math.h>
#include <string.h>

/* 2026-09-23 扩展新增（设计基准 docs/网口指令扩展设计_2026-09-23.md） */
#include "pid_flash.h"
#include "teach_store.h"
#include "arm_link_cfg.h"
#include "arm_net.h"         /* 取 ARM_LINK_NET 的 #ifndef 默认值 0 */
#include "arm_proto_can.h"   /* 取 ARM_LINK_CAN 的 #ifndef 默认值 0 */

/* 修正状态机运行标志（main.c 导出，用于状态快照） */
extern uint8_t correction_pending;

/* ---- 运动控制 ---- */

int ArmApi_MoveJoints(float target_deg[6], float v_max, float a) {
    if (IsEStop()) {
        RLOG_W("API", "MoveJoints rejected: E-STOP");
        return ARM_API_ERR_ESTOP;
    }
    /* 速度/加速度缺省值与范围钳制（arm_params 生效点） */
    if (v_max <= 0.0f) v_max = ARM_SC_DEF_VMAX;
    if (a <= 0.0f)     a     = v_max * ARM_SC_ACCEL_FACTOR;
    if (v_max < ARM_SC_V_MIN) v_max = ARM_SC_V_MIN;
    if (v_max > ARM_SC_V_MAX) v_max = ARM_SC_V_MAX;
    if (a < 1.0f) a = 1.0f;

    RLOG_I("API", "MoveJoints tgt=%d,%d,%d,%d,%d,%d v=%d a=%d",
           (int)target_deg[0], (int)target_deg[1], (int)target_deg[2],
           (int)target_deg[3], (int)target_deg[4], (int)target_deg[5],
           (int)v_max, (int)a);
    cmdMove(target_deg, v_max, a);
    return ARM_API_OK;
}

int ArmApi_MoveCartesian(float x, float y, float z, float pitch_deg,
                         float v_max, float a) {
    if (IsEStop()) {
        RLOG_W("API", "MoveCart rejected: E-STOP");
        return ARM_API_ERR_ESTOP;
    }

    /* IK-CCD 解算（theta1 解析 + theta2~5 多起点 CCD + theta6 补偿） */
    Kinematics_Pos target;
    target.x = x; target.y = y; target.z = z;
    target.yaw = pitch_deg * 3.14159265f / 180.0f;

    float rad[6];
    int ik_ok = IK_Solve(&target, rad);
    if (!ik_ok) {
        RLOG_W("API", "IK unreachable: x=%d y=%d z=%d", (int)x, (int)y, (int)z);
        return ARM_API_ERR_UNREACH;
    }

    /* 关节限位检查 */
    float deg[6];
    for (uint8_t j = 0; j < 6; j++) {
        deg[j] = rad_to_servo_deg(rad[j], j);
        if (deg[j] < JOINT_MIN[j] || deg[j] > JOINT_MAX[j]) {
            RLOG_W("API", "IK out of limits: J%d=%d", j + 1, (int)deg[j]);
            return ARM_API_ERR_LIMIT;
        }
    }

    /* FK 回代验证（容差 = AP_IK_FK_VERIFY_MM 生效点） */
    Kinematics_Pos verify;
    FK_Solve(rad, &verify);
    float fk_err = sqrtf((verify.x - x) * (verify.x - x) +
                         (verify.y - y) * (verify.y - y) +
                         (verify.z - z) * (verify.z - z));
    if (fk_err > KIN_FK_VERIFY_MM) {
        RLOG_W("API", "FK verify failed: err=%d mm", (int)fk_err);
        return ARM_API_ERR_FK_VERIFY;
    }

    RLOG_I("API", "MoveCart x=%d y=%d z=%d p=%d -> J=%d,%d,%d,%d,%d,%d",
           (int)x, (int)y, (int)z, (int)pitch_deg,
           (int)deg[0], (int)deg[1], (int)deg[2],
           (int)deg[3], (int)deg[4], (int)deg[5]);
    return ArmApi_MoveJoints(deg, v_max, a);
}

void ArmApi_Stop(void) {
    Trajectory_Stop();
    RLOG_I("API", "Trajectory stopped by command");
}

int ArmApi_SetTorque(uint8_t joint, uint8_t enable) {
    if (joint == 0xFF) {
        for (uint8_t j = 0; j < 6; j++) SERVO_SetTorque(j, enable);
    } else if (joint < 6) {
        SERVO_SetTorque(joint, enable);
    } else {
        return ARM_API_ERR_PARAM;
    }
    RLOG_I("API", "Torque j=%d en=%d", joint, enable);
    return ARM_API_OK;
}

int ArmApi_StreamJoints(float jdeg[6]) {
    uint8_t rc = cmdStreamJoints(jdeg);
    if (rc == 2U) {
        RLOG_W("API", "StreamJoints rejected: E-STOP");
        return ARM_API_ERR_ESTOP;
    }
    if (rc != 0U) return ARM_API_ERR_LIMIT;
    return ARM_API_OK;
}

/* ⭐0924DC: fw-tag / build 号唯一出口 —— ArmApi_GetStatus 与 RunLog_Init
 * 开机头共用, 改版本号时这里 + main.c 帮助文本两处同步。 */
const char* ArmApi_FwTag(void) {
#if defined(ARM_MASTER_BUILD) && ARM_MASTER_BUILD
    return "ARM-MASTER-1030E";
#elif defined(ARM_LINK_MULTI) && ARM_LINK_MULTI
    return "ARM-BOTH-0924DE";
#elif ARM_LINK_NET
    return "ARM-NET-0924DE";
#elif ARM_LINK_CAN
    return "ARM-CAN-0918H";
#else
    return "ARM-UART";
#endif
}

uint8_t ArmApi_FwTagBuild(void) {
#if defined(ARM_MASTER_BUILD) && ARM_MASTER_BUILD
    return 3U;
#elif defined(ARM_LINK_MULTI) && ARM_LINK_MULTI
    return 2U;
#elif ARM_LINK_NET
    return 0U;
#elif ARM_LINK_CAN
    return 1U;
#else
    return 3U;
#endif
}

/* ---- FK/IK 查询 ---- */

int ArmApi_GetStatus(ArmApiStatus* out) {
    if (!out) return ARM_API_ERR_PARAM;
    memset(out, 0, sizeof(*out));
    out->estop       = IsEStop();
    out->traj_active = Trajectory_IsActive();
    out->corr_active = correction_pending;

    float rad[6];
    uint8_t ok = 1;
    for (uint8_t j = 0; j < 6; j++) {
        float d = -1.0f;
        /* ⭐0924CI (A 需求单第 1 条): 缓存优先 —— 遥测采样器(10ms/轴轮转)
         * 常开后, 这里直接取 <=100ms 新鲜度的缓存值, 不占舵机总线。
         * fi 直通时 GET_STATUS 不再等 fi 的总线临界区: p95 ~68ms -> <1ms。
         * 缓存过期/丢读轴才回落直读(原语义不变), 直读失败按原逻辑重试一次。 */
        if (!ArmMain_GetCachedJointDeg(j, &d, 100U)) {
            d = SERVO_GetPosition(j);
            if (d < 0.0f) d = SERVO_GetPosition(j);   /* 丢帧重试一次 (总线 A 偶发) */
        }
        out->joint_deg[j] = d;
        if (d < 0.0f) {
            ok = 0;
        } else {
            rad[j] = servo_deg_to_rad(d, j);
        }
    }
    if (!ok) {
        out->read_ok = 0;
        RLOG_W("API", "GetStatus: servo read failed");
        return ARM_API_ERR_READ;
    }

    Kinematics_Pos pos;
    FK_Solve(rad, &pos);
    out->x = pos.x;
    out->y = pos.y;
    out->z = pos.z;
    out->pitch_deg = pos.yaw * 180.0f / 3.14159265f;
    out->read_ok = 1;
    return ARM_API_OK;
}

/* ---- 重力补偿 / 自适应修正 ---- */

int ArmApi_SetGravityComp(float k) {
    if (ArmParams_Set(AP_GC_K, k) != 0) return ARM_API_ERR_PARAM;
    float applied = 0.0f;
    ArmParams_Get(AP_GC_K, &applied);
    RLOG_I("API", "Gravity comp K=%d mdeg/mm", (int)(applied * 1000.0f));
    return ARM_API_OK;
}

int ArmApi_SetCorrection(uint8_t enable, float threshold_deg,
                         uint8_t max_rounds) {
    int rc = 0;
    rc |= ArmParams_Set(AP_CORR_ENABLED, enable ? 1.0f : 0.0f);
    rc |= ArmParams_Set(AP_CORR_THRESHOLD, threshold_deg);
    rc |= ArmParams_Set(AP_CORR_MAX_ROUNDS, (float)max_rounds);
    if (rc != 0) return ARM_API_ERR_PARAM;
    RLOG_I("API", "Correction cfg: en=%d thr=%d r=%d",
           enable, (int)threshold_deg, max_rounds);
    return ARM_API_OK;
}

/* ============================================================
 * 2026-09-23 扩展：让协议层（网口 / CAN / 串口二进制）覆盖文本控制台能力
 * 设计基准: docs/网口指令扩展设计_2026-09-23.md
 *
 * 纪律（与 arm_main_exports.h 的约定一致）：
 *   本层只调底层模块 + 那个头文件**已登记**的导出符号。要读 main.c 内部
 *   static 时先去登记 —— 那是唯一的耦合点，也是防 config-drift 的防线
 *   （绕过它 = 把"生效点唯一"这条纪律作废）。
 * ============================================================ */

/* 丢读哨兵。舵机总线是【逐字段瞬时丢读】的（实测 Angle≈5% / Temp≈20%，
 * 各字段独立随机），所以 0.0 绝不能当"真值"用 —— 必须给一个物理上
 * 不可能出现的哨兵值。 */
#define API_TEMP_LOST    0xFFU
#define API_HWERR_LOST   0xFFU

/* name 缓冲长度一致性（ArmApiTeachInfo.name 是硬编码 12，见头文件） */
_Static_assert(TSTORE_NAME_MAX + 1U == 12U,
               "ArmApiTeachInfo.name 与 TSTORE_NAME_MAX 不一致，两处同改");

/* ============================================================
 * 关节详情（替代文本 s / T / O / E）
 * ============================================================ */

static void api_fill_detail(uint8_t j, ArmApiJointDetail* d) {
    memset(d, 0, sizeof(*d));
    d->temp   = API_TEMP_LOST;
    d->hw_err = API_HWERR_LOST;

    /* 一次事务读回整组运行状态（位置/速度/负载/电压/温度），约 0.7ms，
     * 比 5~6 次单寄存器读完一个数量级，而且不干扰运动中的采样。 */
    SERVO_Telemetry_t t;
    memset(&t, 0, sizeof(t));
    (void)SERVO_ReadTelemetry(j, &t);
    if (t.base_valid) {
        d->pos_raw = t.pos;
        d->speed   = t.speed;
        d->load    = t.load;
        d->voltage = t.voltage;
        d->temp    = t.temp;
        d->ok      = 1U;
    }
    /* 硬件错误不在遥测基础组里，单独读一次（读失败时返回 0xFF） */
    d->hw_err = SERVO_GetHardwareError(j);
}

int ArmApi_GetJointDetail(uint8_t joint, ArmApiJointDetail* out) {
    if (out == NULL || joint > 5U) return ARM_API_ERR_PARAM;
    api_fill_detail(joint, out);
    if (!out->ok) {
        RLOG_W("API", "JointDetail J%d: read failed", joint + 1U);
        return ARM_API_ERR_READ;
    }
    return ARM_API_OK;
}

void ArmApi_GetJointDetailAll(ArmApiJointDetail out[6]) {
    if (out == NULL) return;
    for (uint8_t j = 0; j < 6U; j++) api_fill_detail(j, &out[j]);
}

/* ============================================================
 * 限位与 Home 零点（替代文本 zi）
 * ============================================================ */

int ArmApi_GetLimits(ArmApiLimits* out) {
    if (out == NULL) return ARM_API_ERR_PARAM;
    memset(out, 0, sizeof(*out));

    uint8_t p[6], i[6], d[6], corr_pct[6];
    PID_JointLimits_t lim;
    uint16_t threshold_cdeg = 0U, moving_speed = 0U;
    uint8_t  max_rounds = 0U, enabled = 0U, params_loaded = 0U, loaded = 0U;

    memset(p, 0, sizeof(p));
    memset(i, 0, sizeof(i));
    memset(d, 0, sizeof(d));
    memset(corr_pct, 0, sizeof(corr_pct));
    memset(&lim, 0, sizeof(lim));

    /* ⚠ PID_FlashLoad 是 13 参的"整包读"（PID + 修正 + 速度 + 参数 + 限位）。
     *   这里只要限位，所以：arm_params 传 NULL + capacity 0（实现里按
     *   capacity 循环，0 就不会写），其余用一个局部变量接住后丢弃 ——
     *   显式丢弃比一律传 NULL 安全，免得将来某个版本给某个参数加上
     *   "必填"语义时这里静默出错。 */
    uint8_t rc = PID_FlashLoad(p, i, d, corr_pct,
                               &threshold_cdeg, &max_rounds, &enabled,
                               &moving_speed, NULL, 0U, &params_loaded,
                               &lim, &loaded);
    if (rc != PID_FLASH_OK) {
        RLOG_W("API", "GetLimits: PID_FlashLoad rc=%u", rc);
        return ARM_API_ERR_READ;
    }
    for (uint8_t j = 0; j < 6U; j++) {
        out->min_raw[j]   = lim.min[j];
        out->pos_raw[j]   = lim.pos[j];
        out->max_raw[j]   = lim.max[j];
        out->zero_raw[j]  = lim.zero[j];
        out->joint_min[j] = JOINT_MIN[j];   /* 内存里当前生效的运行时限位 */
        out->joint_max[j] = JOINT_MAX[j];
    }
    out->loaded = loaded;
    RLOG_I("API", "GetLimits: loaded=%u zero=J1:%u J2:%u J3:%u",
           loaded, lim.zero[0], lim.zero[1], lim.zero[2]);
    return ARM_API_OK;
}

/* ============================================================
 * 固件与链路信息（替代文本 h 的首行）
 * ============================================================ */

void ArmApi_GetInfo(ArmApiInfo* out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));

    /* fw-tag: 与 main.c 帮助首行的字符串保持同一批。
     * ⭐0924DC: 收敛为 ArmApi_FwTag() 唯一出口 —— runlog 开机头也引用它,
     *   改版本号时本函数 + main.c 帮助文本两处同步即可。 */
    out->build = (ArmApi_FwTagBuild());
    const char* tag = ArmApi_FwTag();
    size_t tn = strlen(tag);
    if (tn > sizeof(out->fw_tag) - 1U) tn = sizeof(out->fw_tag) - 1U;
    memcpy(out->fw_tag, tag, tn);
    out->fw_tag[tn] = '\0';

    /* ⚠ 用 ActiveMode（启动冻结值），不是偏好 —— 红线 69。
     *   用偏好会让"正在跑的链路"和上报值分叉。 */
    out->link_mode = LinkCfg_ActiveMode();
    out->n_params  = ArmParams_Count();
}

int ArmApi_GetFlags(uint8_t* flags) {
    if (flags == NULL) return ARM_API_ERR_PARAM;
    /* 只读几个门内变量，代价近似为零 —— 这是"高频轮询"能成立的前提
     * （对照 ArmApi_GetStatus：那个要读 6 轴舵机，约 4~6ms）。 */
    *flags = ArmMain_Flags();
    return ARM_API_OK;
}

/* ============================================================
 * 参数表列举（替代文本 LP）
 * ============================================================ */

uint8_t ArmApi_ParamList(uint8_t offset, ArmApiParamBrief* out, uint8_t max) {
    if (out == NULL || max == 0U) return 0U;

    const ArmParamMeta* tbl = ArmParams_Table();
    uint8_t total = ArmParams_Count();
    if (tbl == NULL || total == 0U) return 0U;

    /* 直接按索引取。不用 ArmParams_Find(id)：参数 id 有空洞时会原地打转。 */
    uint8_t n = 0U;
    while (n < max) {
        uint16_t idx = (uint16_t)offset + (uint16_t)n;
        if (idx >= (uint16_t)total) break;
        out[n].id   = tbl[idx].id;
        out[n].type = (uint8_t)tbl[idx].type;
        out[n].vmin = tbl[idx].vmin;
        out[n].vmax = tbl[idx].vmax;
        n++;
    }
    return n;
}

/* ============================================================
 * 补偿 / PID 读回（补齐 SET_* 的反向）
 * ============================================================ */

int ArmApi_GetGravity(float* k_out) {
    if (k_out == NULL) return ARM_API_ERR_PARAM;
    if (ArmParams_Get(AP_GC_K, k_out) != 0) return ARM_API_ERR_PARAM;
    return ARM_API_OK;
}

int ArmApi_GetCorrection(uint8_t* en, float* threshold_deg,
                         uint8_t* max_rounds, uint8_t* pending) {
    if (en != NULL)            *en = (correction_enabled != 0U) ? 1U : 0U;
    if (threshold_deg != NULL) *threshold_deg = corr_threshold;
    if (max_rounds != NULL)    *max_rounds = corr_max_rounds;
    if (pending != NULL)       *pending = (correction_pending != 0U) ? 1U : 0U;
    return ARM_API_OK;
}

int ArmApi_GetPid(uint8_t joint, uint8_t* p, uint8_t* i, uint8_t* d,
                  uint8_t* from_eeprom) {
    if (joint > 5U) return ARM_API_ERR_PARAM;

    uint8_t rp = 0U, rd = 0U, ri = 0U;
    /* ⚠ SERVO_ReadPidEeprom 的形参顺序是 (joint, *p, *d, *i) —— 不是 p,i,d！
     *   这里按它的真实顺序接，再按协议约定的 p/i/d 输出。搞反了会把 I 和 D
     *   对调，现象是"参数看起来对但调不动"。 */
    uint8_t ok = SERVO_ReadPidEeprom(joint, &rp, &rd, &ri);

    if (p != NULL) *p = rp;
    if (i != NULL) *i = ri;
    if (d != NULL) *d = rd;
    if (from_eeprom != NULL) *from_eeprom = (ok != 0U) ? 1U : 0U;
    return (ok != 0U) ? ARM_API_OK : ARM_API_ERR_READ;
}

/* ============================================================
 * 单轴移动 / 总线模式 / 舵机初始化
 * ============================================================ */

int ArmApi_MoveJointOne(uint8_t joint, float deg, float v_max, float a) {
    if (joint > 5U) return ARM_API_ERR_PARAM;
    if (IsEStop()) return ARM_API_ERR_ESTOP;

    /* 必须先读全场：只动一轴，意味着另外五轴的目标 = 它们当前的位置。
     * 任一轴丢读就整体拒绝 —— 拿不准位置时不动，比动错强。 */
    ArmApiStatus st;
    int rc = ArmApi_GetStatus(&st);
    if (rc != ARM_API_OK) return rc;

    float tgt[6];
    for (uint8_t j = 0; j < 6U; j++) tgt[j] = st.joint_deg[j];
    tgt[joint] = deg;

    RLOG_I("API", "MoveJointOne J%d -> %d deg (others held)",
           joint + 1U, (int)deg);
    return ArmApi_MoveJoints(tgt, v_max, a);
}

int ArmApi_SetBusMode(uint8_t mode) {
    if (mode > 2U) return ARM_API_ERR_PARAM;
    (void)SERVO_SetBusMode(mode);
    RLOG_I("API", "Bus mode -> %u", mode);
    return ARM_API_OK;
}

int ArmApi_GetBusMode(uint8_t* mode) {
    if (mode == NULL) return ARM_API_ERR_PARAM;
    *mode = SERVO_GetBusMode();
    return ARM_API_OK;
}

int ArmApi_ServoInit(uint8_t joint) {
    if (joint == 0xFFU) {
        SERVO_InitAll();
    } else if (joint < 6U) {
        SERVO_InitJoint(joint);
    } else {
        return ARM_API_ERR_PARAM;
    }
    RLOG_I("API", "ServoInit joint=%u", joint);
    return ARM_API_OK;
}

/* ============================================================
 * 运动学：只解不算
 * ============================================================ */

/* ⚠ 这段与 ArmApi_MoveCartesian 的前半段是同一套判定链
 *   (IK_Solve -> 关节限位 -> FK 回代验证)。改其中一处必须两处同改。
 *   没有抽公共函数是有意的：MoveCartesian 是已验证的关键路径，
 *   先不扰动它。 */
int ArmApi_IkQuery(float x, float y, float z, float pitch_deg, float deg[6]) {
    if (deg == NULL) return ARM_API_ERR_PARAM;

    Kinematics_Pos target;
    target.x = x; target.y = y; target.z = z;
    target.yaw = pitch_deg * 3.14159265f / 180.0f;

    float rad[6];
    if (!IK_Solve(&target, rad)) return ARM_API_ERR_UNREACH;

    for (uint8_t j = 0; j < 6U; j++) {
        deg[j] = rad_to_servo_deg(rad[j], j);
        if (deg[j] < JOINT_MIN[j] || deg[j] > JOINT_MAX[j]) {
            RLOG_W("API", "IkQuery out of limits: J%d=%d", j + 1U, (int)deg[j]);
            return ARM_API_ERR_LIMIT;
        }
    }

    Kinematics_Pos verify;
    FK_Solve(rad, &verify);
    float e = sqrtf((verify.x - x) * (verify.x - x) +
                    (verify.y - y) * (verify.y - y) +
                    (verify.z - z) * (verify.z - z));
    if (e > KIN_FK_VERIFY_MM) {
        RLOG_W("API", "IkQuery FK verify failed: err=%d mm", (int)e);
        return ARM_API_ERR_FK_VERIFY;
    }
    return ARM_API_OK;
}

/* ============================================================
 * 示教 / 轨迹存储（替代文本 list / play / del / compact）
 * ============================================================ */

uint32_t ArmApi_TeachCount(void) {
    return TeachStore_Count();
}

int ArmApi_TeachInfo(uint8_t idx, ArmApiTeachInfo* out) {
    if (out == NULL) return ARM_API_ERR_PARAM;
    memset(out, 0, sizeof(*out));
    if ((uint32_t)idx >= TeachStore_Count()) return ARM_API_ERR_PARAM;

    const char* nm = TeachStore_FileName(idx);
    if (nm != NULL) {
        size_t l = strlen(nm);
        if (l > sizeof(out->name) - 1U) l = sizeof(out->name) - 1U;
        memcpy(out->name, nm, l);
        out->name[l] = '\0';
    }
    out->samples = TeachStore_FileSamples(idx);
    /* samples==0 视作 tombstone（已删、待 compact 回收空间） */
    out->valid = (out->samples > 0U) ? 1U : 0U;
    return ARM_API_OK;
}

uint32_t ArmApi_TeachRead(uint8_t idx, uint32_t off, uint8_t n,
                          uint16_t out[][6]) {
    if (out == NULL || n == 0U) return 0U;

    uint32_t total = TeachStore_FileSamples(idx);
    if (off >= total) return 0U;                 /* 越界 = 正常读完 */

    uint32_t avail = total - off;
    if (avail < (uint32_t)n) n = (uint8_t)avail;

    const uint16_t (*data)[6] = TeachStore_FileData(idx);
    if (data == NULL) return 0U;

    for (uint8_t k = 0; k < n; k++) {
        for (uint8_t j = 0; j < 6U; j++) out[k][j] = data[off + k][j];
    }
    return (uint32_t)n;
}

int ArmApi_TeachDelete(uint8_t idx) {
    if ((uint32_t)idx >= TeachStore_Count()) return ARM_API_ERR_PARAM;
    const char* nm = TeachStore_FileName(idx);
    if (nm == NULL) return ARM_API_ERR_PARAM;
    uint8_t rc = TeachStore_Delete(nm);
    RLOG_I("API", "TeachDelete idx=%u name=%s rc=%u", idx, nm, rc);
    return (rc == TSTORE_OK) ? ARM_API_OK : ARM_API_ERR_PARAM;
}

int ArmApi_TeachCompact(void) {
    uint8_t rc = TeachStore_Compact();
    return (rc == TSTORE_OK) ? ARM_API_OK : ARM_API_ERR_READ;
}

void ArmApi_TeachTotals(uint8_t* n_files, uint32_t* total,
                        uint32_t* free_bytes) {
    if (n_files != NULL)    *n_files = (uint8_t)TeachStore_Count();
    if (total != NULL)      *total = TeachStore_TotalSamples();
    if (free_bytes != NULL) *free_bytes = TeachStore_FreeBytes();
}
