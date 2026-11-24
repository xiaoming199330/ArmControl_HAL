/* ============================================================
 * arm_control_api.h - 机械臂统一控制接口（业务门面）
 *
 * 三大算法能力的程序入口，供串口协议 / CAN 协议 / 本地命令调用：
 *   - FK/IK-CCD 运动学:  MoveCartesian(IK 移动) / FkQuery(正解查询)
 *   - 七段 S 曲线轨迹:   MoveJoints(关节空间 S 曲线) / Stop
 *   - 自适应重力补偿:    SetGravityComp / SetCorrection
 *
 * 所有函数仅在主循环上下文调用（协议 dispatch 在主循环执行），
 * 内部沿用 cmdMove 的安全链：E-Stop 检查 -> 限位钳制 -> 读起点 ->
 * 开扭矩 -> S 曲线规划 -> 轨迹结束后自适应修正。
 *
 * 角度量纲: 舵机度 0~300；位置 mm；速度 deg/s；加速度 deg/s^2。
 * ============================================================ */
#ifndef __ARM_CONTROL_API_H
#define __ARM_CONTROL_API_H

#include <stdint.h>

/* 统一返回码（协议 NACK 错误码与其一一对应） */
enum {
    ARM_API_OK            = 0,    /* 成功，运动已启动 */
    ARM_API_ERR_PARAM     = 1,    /* 参数非法（长度/范围/ID） */
    ARM_API_ERR_CRC       = 2,    /* 帧 CRC 校验失败（协议层使用） */
    ARM_API_ERR_UNREACH   = 3,    /* IK-CCD 不可达 / 不收敛 */
    ARM_API_ERR_LIMIT     = 4,    /* 解算结果超关节限位 */
    ARM_API_ERR_ESTOP     = 5,    /* E-Stop 激活，禁止运动 */
    ARM_API_ERR_FK_VERIFY = 6,    /* IK 解 FK 回代验证超差 */
    ARM_API_ERR_READ      = 7,    /* 舵机总线读取失败 */
    ARM_API_ERR_MAGIC     = 8,    /* 危险命令 magic 不符（防误触发，见 APROTO_MAGIC） */
    ARM_API_ERR_BUSY      = 9,    /* 轨迹/示教/回放进行中，当前操作不允许 */
    ARM_API_ERR_NOTCAL    = 10    /* 未做 Home 零点校准，无法换算 */
};

/* 状态快照（GET_STATUS / FK_QUERY 响应的数据源） */
typedef struct {
    float    joint_deg[6];    /* 六关节实际角度 (舵机度)；读取失败为 -1 */
    float    x, y, z;         /* FK 末端位置 (mm)；read_ok=0 时无意义 */
    float    pitch_deg;       /* FK 末端俯仰 (deg) */
    uint8_t  read_ok;         /* 1=六关节全部读取成功且 FK 已解算 */
    uint8_t  estop;           /* 1=E-Stop 激活 */
    uint8_t  traj_active;     /* 1=S 曲线轨迹执行中 */
    uint8_t  corr_active;     /* 1=自适应修正进行中 */
} ArmApiStatus;

/* ---- 运动控制 ---- */

/* 关节空间 S 曲线移动（七段 jerk-limited，200Hz 六关节同步）。
 * target_deg 会被就地钳制到安全限位。v_max/a 为 0 时用参数
 * AP_SC_DEF_VMAX / AP_SC_DEF_ACCEL 默认值。
 * 返回 ARM_API_OK 或 ARM_API_ERR_ESTOP。 */
int  ArmApi_MoveJoints(float target_deg[6], float v_max, float a);

/* 笛卡尔空间移动（IK-CCD: theta1 解析 + theta2~5 多起点 CCD +
 * theta6 姿态补偿；解算后经 FK 回代验证再执行 S 曲线移动）。
 * x/y/z mm，pitch_deg 末端俯仰(相对竖直)。v_max/a 同上可 0。
 * 返回 ARM_API_OK / ERR_UNREACH / ERR_LIMIT / ERR_FK_VERIFY /
 * ERR_ESTOP。 */
int  ArmApi_MoveCartesian(float x, float y, float z, float pitch_deg,
                          float v_max, float a);

/* 停止当前轨迹（等价文本命令 S；多点/循环状态一并停止由调用方
 * 文本命令负责，协议急停场景直接停轨迹即可）。 */
void ArmApi_Stop(void);

/* ⭐0924DC: fw-tag / build 号唯一出口（GetStatus 与 RunLog_Init 共用） */
const char* ArmApi_FwTag(void);
uint8_t     ArmApi_FwTagBuild(void);

/* 单关节扭矩开关。joint=0~5，0xFF=全部。 */
int  ArmApi_SetTorque(uint8_t joint, uint8_t enable);

/* 主从直写 (2026-09-11)：逻辑关节角 (相对本臂 Home, deg, 会被就地钳制)
 * 直接下发六轴目标，不做轨迹规划/起点读/settle，用于主从实时跟随。
 * 建议调用频率 20~50Hz；连续 500ms 无调用则看门狗自动退出流模式。
 * 返回 ARM_API_OK / ARM_API_ERR_LIMIT (限位/桌面/立柱/跳变被拒) /
 * ARM_API_ERR_ESTOP。 */
int  ArmApi_StreamJoints(float jdeg[6]);

/* ---- FK/IK 查询 ---- */

/* 采集状态快照：读六关节实际角度并做 FK 正解。
 * 返回 ARM_API_OK / ERR_READ（任一关节读取失败）。 */
int  ArmApi_GetStatus(ArmApiStatus* out);

/* ---- 重力补偿 / 自适应修正 ---- */

/* 设置 J2 重力前馈增益 K (deg/mm 水平臂展)，0=关闭。
 * 范围钳制 0~0.5（AP_GC_K 同一生效点）。 */
int  ArmApi_SetGravityComp(float k);

/* 配置自适应修正：enable 0/1，threshold_deg 0.5~10，
 * max_rounds 1~20。 */
int  ArmApi_SetCorrection(uint8_t enable, float threshold_deg,
                          uint8_t max_rounds);

/* ============================================================
 * 2026-09-23 扩展：让协议层（网口/CAN/串口二进制）覆盖文本控制台能力
 * 设计基准: docs/网口指令扩展设计_2026-09-23.md
 *
 * 纪律（与 arm_main_exports.h 的约定一致）：
 *   本层只调底层模块 (servo_driver / pid_flash / teach_store /
 *   arm_params / arm_link_cfg) 与 arm_main_exports.h **已登记**的符号；
 *   需要读 main.c 内部 static 时，先去那个头文件登记（防 config-drift）。
 * ============================================================ */

/* ---- 关节详情：温度 / 电压 / 负载 / 速度 / 硬件错误 ---- */

typedef struct {
    uint16_t pos_raw;    /* Present Position, raw 0~4095 */
    int16_t  speed;      /* Present Speed, step/s (有符号) */
    int16_t  load;       /* Present Load, 有符号 -1024~+1023 */
    uint8_t  voltage;    /* Present Voltage, 0.1V/LSB */
    uint8_t  temp;       /* 摄氏度；0xFF = 丢读（与"真的 0 度"区分） */
    uint8_t  hw_err;     /* 硬件错误位：0=正常，0xFF=读取失败 */
    uint8_t  ok;         /* 1 = 本轴读取成功 */
} ArmApiJointDetail;

/* 返回 ARM_API_OK / ERR_READ（本轴丢读）。out 始终被填充；
 * ok=0 时除 temp/hw_err 两个哨兵外，其余字段无意义。 */
int  ArmApi_GetJointDetail(uint8_t joint, ArmApiJointDetail* out);

/* 六轴一次读完（逐轴读，约 6×0.7ms）。刻意不返回错误码：
 * 总线是【逐字段瞬时丢读】的，所以"整体成功/失败"没有意义 ——
 * 每轴的成败看各自的 ok 字段。 */
void ArmApi_GetJointDetailAll(ArmApiJointDetail out[6]);

/* ---- 限位与 Home 零点 ---- */

typedef struct {
    uint16_t min_raw[6];    /* 舵机 raw 下限（来自 Flash 校准记录） */
    uint16_t pos_raw[6];    /* 舵机 raw 中位 */
    uint16_t max_raw[6];    /* 舵机 raw 上限 */
    uint16_t zero_raw[6];   /* Home 零点 raw；0 = 该轴未校准 */
    float    joint_min[6];  /* 运行时限位（舵机度，当前生效值） */
    float    joint_max[6];
    uint8_t  loaded;        /* 1 = Flash 里有校准记录 */
} ArmApiLimits;

/* ⚠ raw 类字段来自 Flash，joint_min/max 是内存当前生效值。
 *   运行期改了零点但没落盘时两者可能不一致（见设计文档 §6 说明）。 */
int  ArmApi_GetLimits(ArmApiLimits* out);

/* ---- 固件与链路信息 ---- */

typedef struct {
    char     fw_tag[17];    /* 含 NUL，最长 16 字符 */
    uint8_t  build;         /* 0=net 1=can 2=both 3=uart/其他 */
    uint8_t  link_mode;     /* LinkCfg_ActiveMode()：启动冻结值，不是偏好 */
    uint8_t  n_params;      /* 参数表条目数 */
} ArmApiInfo;

void ArmApi_GetInfo(ArmApiInfo* out);

/* 运行状态位图（ARM_MAIN_FLAG_*：E-Stop / 轨迹 / 流模式 / 修正 /
 * 示教录制 / 示教回放）。只读几个变量、不碰舵机总线 ⇒ 可以高频轮询。 */
int  ArmApi_GetFlags(uint8_t* flags);

/* ---- 参数表列举 ---- */

typedef struct {
    uint8_t id;
    uint8_t type;    /* ArmParamType: 0=float 1=int 2=u8 */
    float   vmin;
    float   vmax;
} ArmApiParamBrief;

/* 从 offset 起最多取 max 条，返回实际条数。
 * ⚠ 这里只给元数据不给值 —— 带上 i32 值会让响应帧超过 64B 上限。
 *   值请用 PARAM_READ 逐条单取。 */
uint8_t ArmApi_ParamList(uint8_t offset, ArmApiParamBrief* out, uint8_t max);

/* ---- 补偿 / PID 读回（补齐 SET_* 的反向） ---- */

int  ArmApi_GetGravity(float* k_out);

/* 生效值直接读导出变量，保证与运动链看到的是同一个数（不走参数表，
 * 避免"参数表值"与"生效变量值"分叉）。 */
int  ArmApi_GetCorrection(uint8_t* en, float* threshold_deg,
                          uint8_t* max_rounds, uint8_t* pending);

/* 读单轴 PID。from_eeprom=1 表示读自舵机 EEPROM（权威值）；
 * 0 表示读取失败（此时 p/i/d 不可信）。 */
int  ArmApi_GetPid(uint8_t joint, uint8_t* p, uint8_t* i, uint8_t* d,
                   uint8_t* from_eeprom);

/* ---- 单轴 / 总线 / 初始化 ---- */

/* 单关节移动：其余五轴保持当前位置（内部先读一次全场快照）。
 * 任一轴丢读则整体拒绝（ERR_READ）—— 不猜位置比动起来更重要。 */
int  ArmApi_MoveJointOne(uint8_t joint, float deg, float v_max, float a);

int  ArmApi_SetBusMode(uint8_t mode);   /* 0=正常 1=全A 2=全B */
int  ArmApi_GetBusMode(uint8_t* mode);

/* 舵机初始化。joint=0~5 单轴，0xFF=全部。
 * ⚠ 会重写该轴 PID/限位/中位并 RESET，与文本命令 i 同副作用。 */
int  ArmApi_ServoInit(uint8_t joint);

/* ---- 运动学：只解不算 ---- */

/* 只做 IK 解算 + 限位检查 + FK 回代验证，不启动运动。
 * 成功时 deg[6] 填舵机度。
 * 返回 ARM_API_OK / ERR_UNREACH / ERR_LIMIT / ERR_FK_VERIFY。 */
int  ArmApi_IkQuery(float x, float y, float z, float pitch_deg, float deg[6]);

/* ---- 示教 / 轨迹存储 ---- */

typedef struct {
    /* 名字缓冲 = TSTORE_NAME_MAX + 1 = 12；.c 里有 _Static_assert 守一致性 */
    char     name[12];
    uint32_t samples;
    uint8_t  valid;      /* 0 = tombstone（已删待回收） */
} ArmApiTeachInfo;

uint32_t ArmApi_TeachCount(void);
int      ArmApi_TeachInfo(uint8_t idx, ArmApiTeachInfo* out);

/* 分页读样本：读 [off, off+n) 区间，返回实际读到的条数。
 * 返回 0 表示 off 已越界 —— 那是【正常读完】，不是错误。 */
uint32_t ArmApi_TeachRead(uint8_t idx, uint32_t off, uint8_t n,
                          uint16_t out[][6]);

int      ArmApi_TeachDelete(uint8_t idx);
int      ArmApi_TeachCompact(void);
void     ArmApi_TeachTotals(uint8_t* n_files, uint32_t* total,
                            uint32_t* free_bytes);

#endif /* __ARM_CONTROL_API_H */
