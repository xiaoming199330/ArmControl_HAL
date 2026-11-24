/* ============================================================
 * arm_params.h - 三大算法统一配置参数文件（运行时可调）
 *
 * 覆盖模块:
 *   [IK-CCD ] 运动学逆解: 收敛阈值/迭代上限/失败阈值/FK 验证容差
 *   [S-CURVE] 七段 S 曲线: jerk 时间/默认速度/默认加速度/速度范围
 *   [GRAVITY] 重力补偿+自适应修正: K 增益/开关/阈值/轮数/各关节增益
 *
 * 设计原则（指针绑定，单一生效点）:
 *   每个参数表项持有“实际生效变量”的地址（main.c / kinematics.c /
 *   trajectory.c 中的全局变量），Set 时范围钳制后直接写入生效点，
 *   不存在第二份拷贝 —— 从根上杜绝 arm_config.h 式 config-drift。
 *
 * 调参入口（三者等价，写同一生效点）:
 *   1. 串口文本命令:  LP 列出 / SP <id> <val> 设置
 *   2. 二进制协议:    CMD_PARAM_READ(0x30) / CMD_PARAM_WRITE(0x31)
 *   3. 代码:          ArmParams_Set(AP_xxx, value)
 *
 * 持久化: 执行 'F' 会将本表全部当前参数，以及 PID/修正/速度
 *         一并写入 STM32 内部 Flash；启动时自动恢复并做范围钳制。
 * ============================================================ */
#ifndef __ARM_PARAMS_H
#define __ARM_PARAMS_H

#include <stdint.h>

/* ---- 参数 ID（协议与文本命令共用，新增参数追加勿重排） ---- */
enum {
    /* --- IK-CCD 运动学 (1~15) --- */
    AP_IK_CONVERGE_MM   = 1,    /* CCD 收敛阈值 mm          (0.1~5.0,   默认 0.5)  */
    AP_IK_MAX_ITER      = 2,    /* CCD 单起点最大迭代次数   (10~500,    默认 100)  */
    AP_IK_FAIL_MM       = 3,    /* IK 失败判定阈值 mm       (0.5~10.0,  默认 1.0)  */
    AP_IK_FK_VERIFY_MM  = 4,    /* 运动后 FK 验证容差 mm    (1.0~20.0,  默认 5.0)  */

    /* --- 七段 S 曲线轨迹 (16~31) --- */
    AP_SC_TJ_S          = 16,   /* jerk 过渡时间 s          (0.02~0.50, 默认 0.15) */
    AP_SC_DEF_VMAX      = 17,   /* 默认最大速度 deg/s       (5~300,     默认 40, 0924DA)  */
    AP_SC_DEF_ACCEL     = 18,   /* 默认加速度 deg/s^2       (10~600,    默认 60)   */
    AP_SC_V_MIN         = 19,   /* 允许最小速度 deg/s       (1~50,      默认 5)    */
    AP_SC_V_MAX         = 20,   /* 允许最大速度 deg/s       (50~300,    默认 300)  */
    AP_SC_ACCEL_FACTOR  = 21,   /* 未给加速度时 a=v*factor  (1.0~4.0,   默认 2.0)  */

    /* --- 重力补偿 + 自适应修正 (32~47) --- */
    AP_GC_K             = 32,   /* J2 重力前馈 K deg/mm     (0~0.5,     默认 0)    */
    AP_CORR_ENABLED     = 33,   /* 自适应修正开关 0/1       (默认 0, 0924DA)               */
    AP_CORR_THRESHOLD   = 34,   /* 修正收敛阈值 deg         (0.5~10.0,  默认 2.0)  */
    AP_CORR_MAX_ROUNDS  = 35,   /* 修正最大轮数             (1~20,      默认 10)   */
    AP_CORR_GAIN_J1     = 36,   /* 各关节修正初始增益       (0~0.85, 见下默认值)   */
    AP_CORR_GAIN_J2     = 37,   /*   J1=0.20 J2=0.50 J3=0.30                       */
    AP_CORR_GAIN_J3     = 38,   /*   J4=0.20 J5=0.20 J6=0.20                       */
    AP_CORR_GAIN_J4     = 39,
    AP_CORR_GAIN_J5     = 40,
    AP_CORR_GAIN_J6     = 41,

    /* --- 轨迹收尾降速 (42~43, 2026-09-07 新增; 0924AP 起保留但不再生效) --- */
    AP_TRAJ_APPROACH_SPEED = 42, /* (保留参数, 0924AP 起不影响运动) */
    AP_TRAJ_APPROACH_FRAC  = 43, /* (保留参数, 0924AP 起不影响运动) */

    /* --- 舵机内部加速度 (44, 2026-09-07 新增; 0924AP 恢复单一值 50) --- */
    AP_SERVO_ACCEL      = 44,   /* 舵机 addr41 加速度 (0~254, 默认 50) */

    /* --- 轨迹尾段交接 (45, 2026-09-07 新增) --- */
    AP_TRAJ_HANDOFF_DEG = 45,   /* 尾段交接阈值 deg (0=关, 默认 0; 建议 3 起测) */

    AP_PARAM_COUNT      = 46    /* 表长哨兵（非参数） */
};

/* ---- S 曲线默认参数生效变量（定义在 arm_params.c，main.c 运动命令
 *        的默认速度/加速度引用它们；与参数表 ID 17~21 指针绑定） ---- */
extern float ARM_SC_DEF_VMAX;       /* AP_SC_DEF_VMAX     生效点 */
extern float ARM_SC_DEF_ACCEL;      /* AP_SC_DEF_ACCEL    生效点 */
extern float ARM_SC_V_MIN;          /* AP_SC_V_MIN        生效点 */
extern float ARM_SC_V_MAX;          /* AP_SC_V_MAX        生效点 */
extern float ARM_SC_ACCEL_FACTOR;   /* AP_SC_ACCEL_FACTOR 生效点 */

/* 参数值类型（协议编解码用；统一以 float 交互，整型自动取整） */
typedef enum {
    AP_T_FLOAT = 0,     /* float 生效变量 */
    AP_T_INT   = 1,     /* int   生效变量 */
    AP_T_U8    = 2      /* uint8_t 生效变量 */
} ArmParamType;

typedef struct {
    uint8_t       id;         /* AP_xxx */
    const char*   name;       /* "ik.converge_mm" 等，文本/日志用 */
    ArmParamType  type;
    void*         ptr;        /* 实际生效变量地址（唯一写入点） */
    float         vmin;       /* 允许范围（Set 时钳制） */
    float         vmax;
    float         vdef;       /* 默认值（ArmParams_Reset 恢复） */
    const char*   unit;       /* 显示单位 */
} ArmParamMeta;

/* 初始化：把默认值写入各生效变量（覆盖固件编译期初始值）。
 * 在 main() 中、pid_load_startup() 之后调用（Flash 值优先于默认）。 */
void        ArmParams_Init(void);

/* 读参数表元数据（遍历/格式化输出用） */
const ArmParamMeta* ArmParams_Table(void);
uint8_t             ArmParams_Count(void);

/* 按 ID 查找元数据，未找到返回 0 */
const ArmParamMeta* ArmParams_Find(uint8_t id);

/* 设置参数（范围钳制后写入生效点）。返回 0=OK，-1=ID 不存在。 */
int         ArmParams_Set(uint8_t id, float value);

/* 读取参数当前值。返回 0=OK，-1=ID 不存在。 */
int         ArmParams_Get(uint8_t id, float* out_value);

/* 全部恢复默认值（仅 RAM，不动 Flash）。 */
void        ArmParams_Reset(void);

/* 按参数表顺序导出/恢复全部运行时参数，用于 Flash 持久化。
 * Restore 仍经过 ArmParams_Set 的范围钳制；返回实际处理的参数数。 */
uint8_t     ArmParams_Snapshot(float out_values[], uint8_t capacity);
uint8_t     ArmParams_Restore(const float values[], uint8_t count);

#endif /* __ARM_PARAMS_H */
