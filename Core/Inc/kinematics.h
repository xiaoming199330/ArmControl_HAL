/* 六轴机械臂几何参数、FK/IK 接口和角度/原始值转换函数。
 * 构型: SO-ARM101 (TheRobotStudio/SO-ARM100 项目), 2026-08-31 实测。 */
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/* ============================================================
 * Robot: SO-ARM101 构型 6-DOF Arm (装支架实测 2026-08-31)
 *
 *   J1: Yaw   - 底座水平旋转 (绕竖直 Z 轴)
 *   J2: Pitch - 肩 (竖直平面内旋转)
 *   J3: Pitch - 肘 (竖直平面内旋转)
 *   J4: Pitch - 腕俯仰 (竖直平面内旋转)
 *   J5: Roll  - 腕自转 (绕夹爪轴, 不改变末端位置)
 *   J6: 夹爪开合 (不改变末端位置)
 *
 * 几何来源: TheRobotStudio/SO-ARM100 仓库官方 URDF (Simulation/SO101/so101_new_calib.urdf,
 *           Onshape CAD 导出) + 用户实测轴心距交叉验证 (2026-09-01)。
 *
 * 实测轴心距 (mm, 中性位 = 全部舵机 raw 2048 / 180° 时的姿态,
 *           用户确认 = 官网 Home: 上臂近竖直 + 前臂水平 L 形):
 *   ⚠ 下方"几何来源/官方 URDF"小节是 2026-08-31 的历史记录, 数值已过期;
 *     **现行标定以本文件底部 0924AD 宏为准** (09-28 用户按板实测:
 *     BASE=63.13, J1-J2=53.98, J2-J3=112.45, J3-J4=134.13,
 *     J4-J5=61.55, J5-J6=23.58, J6-TCP=80.77)。
 *   桌面 -> 底座面          17.13
 *   底座面 -> J1 轴         z 48.17            => J1 轴高 65.30 (旧)
 *   J1 轴 -> J2 轴          z 54.80, y 31.32   (旧)
 *   J2 轴 -> J3 轴          官方 (28.0, 112.57) => 上臂 116.00 (旧), 中位倾角 +13.97° (向前)
 *   J3 轴 -> J4 轴          官方 (134.9, 5.2)   => 前臂 135.00 (旧), 中位倾角 +87.79° (水平前指!)
 *   J4 轴 -> J5 轴          z 62.50            (J5 轴心与 J4 机身中心对齐, 官方 61.1)
 *   J5 轴 -> 夹爪末端 TCP   105.30 (旧)         (J5 轴 = 夹爪轴 = 前臂轴线方向)
 *
 * 世界坐标系: X=前, Y=左, Z=上, 原点 = 桌面上的 J1 轴心
 *
 * 连杆方向角约定 (弧度, 臂竖直平面内):
 *   u(psi) = (sin psi, cos psi)   psi=0 连杆竖直向上, psi>0 向前倾
 *   J2->J3 : psi = PSI_UPPER0 + t2
 *   J3->J4 : psi = PSI_FORE0  + t2 + t3
 *   J4->TCP: psi = PSI_FORE0  + t2 + t3 + t4   (J5 轴=前臂轴线, 工具沿前臂; J5 自转不改方向)
 *
 * 关节角 (运动学弧度, 0 = 中性位):
 *   J1: 0 = 朝 +X (前), + = 绕 Z 左转 (朝 +Y)
 *   J2/J3/J4: + = 连杆向前 (+X 侧) 倾
 * 工具俯仰 yaw: 0 = 竖直向上, +90° = 水平向前, 180° = 竖直向下
 *   (注意: 中性位时工具是水平的, 对应 yaw = PSI_FORE0 ≈ 87.8°)
 * ============================================================ */

/* ---- 机械几何 (mm / rad) ----
 * ⭐0924AD: 严格按用户现场实测重新标定 (板上默认值)。
 * 实测: BASE=63.13, J1-J2=53.98, J2-J3=112.45, J3-J4=134.13,
 *       J4-J5=61.55, J5-J6=23.58, J6-TCP=80.77 (全为轴心空间距离)。
 * J1->J2 按原方向角(atan2(31.32,54.80)=29.68°)拆成 Y/Z; PSI 安装零位角保持。 */
#define ROBOT_BASE_H    63.13f    /* 桌面 -> J1 轴高度 */
#define ROBOT_J2_OFF_Y  26.70f    /* J1 轴 -> J2 轴, 臂平面内前向偏移 (53.98*sin 29.68°) */
#define ROBOT_J2_OFF_Z  46.92f    /* J1 轴 -> J2 轴, 向上偏移 (53.98*cos 29.68°) */

#define ROBOT_L_UPPER   112.45f   /* J2->J3 轴心距 (用户实测) */
#define ROBOT_L_FORE    134.13f   /* J3->J4 轴心距 (用户实测) */
#define ROBOT_L_WRIST   61.55f    /* J4->J5 轴心距 (用户实测) */
#define ROBOT_L_TOOL    104.35f   /* J5 轴 -> 夹爪末端 TCP (J5-J6 23.58 + J6-TCP 80.77) */
#define ROBOT_L_DISTAL  (ROBOT_L_WRIST + ROBOT_L_TOOL)  /* J4->TCP */

/* 中性位连杆方向角 (从竖直向上起算, +向前) —— 官方 URDF angle=0 */
#define ROBOT_PSI_UPPER0  0.243787f   /* atan2(28.0, 112.57)  = +13.97 deg */
#define ROBOT_PSI_FORE0   1.532268f   /* atan2(134.9, 5.2)    = +87.79 deg (水平) */
#define ROBOT_PSI_DISTAL0 ROBOT_PSI_FORE0  /* 工具沿前臂轴线方向 */

#define NUM_JOINTS 6

/* J2 平面臂最大伸展 (J2 -> TCP) */
#define ARM_MAX_REACH  (ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL)  /* 418.80 mm */

/* Servo angle range: 0-360 deg (STS3215 12-bit, 0-4095 raw) */
#define SERVO_MIN_DEG     0.0f
#define SERVO_MAX_DEG     360.0f

/* ---- 关节零位 (舵机度): 运动学 0 = 官方 Home 姿态 ----
 * 默认 180 (raw 2048)。联轴装配与 2048 不对齐的臂, 用 'c' 命令 Phase 1 摆
 * 官方 Home 后按 0 记录各关节 raw, 保存后此处即自动更新 (Kinematics_SetZeroRaw)。
 * 当前固件用数组 (kinematics.c 定义), 不在头文件 const。 */
extern float SERVO_OFFSET[6];

/* 以"官方 Home 姿态"为运动学零点: raw[6] = 该姿态下各关节编码器读数 (0..4095)。
 * offset_deg[j] = raw[j] / 4095 * 360。全 0 输入视为"未校准", 恢复默认 180。 */
void Kinematics_SetZeroRaw(const uint16_t raw[6]);

/* ---- 每关节方向符号: +1 = 舵机度增加 -> 关节正向; -1 = 反向 ----
 * 正向定义: J1 = 朝 +Y (左/逆时针俯视); J2/J3/J4 = 连杆向 psi 增大方向
 *   (上臂向前倾 / 前臂向前下 / 工具下沉)。
 * 实测 (2026-09-01, 锚定 Home 后逐关节 +20° 观察):
 *   J1 舵机度增=顺时针(俯视) -> -1; J2 上臂向前倾 / J3 前臂向前倾 /
 *   J4 夹爪向下弯 -> 均 +1。J5/J6 不影响末端位置, 默认 +1。
 * 注意: 'a J deg' 发的是舵机绝对位置, 物理转向与 DIR 无关 (改 DIR 物理转向不变,
 *   只改变运动学解释)。重力补偿 (gravity_comp_j2) 与 IK 限位换算均已按 DIR 折算,
 *   反向关节改 DIR 后会自动正确, 无需改其他代码。 */
static const float JOINT_DIR[6] = { -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

typedef struct {
    float x, y, z;   /* end effector position (mm), X=front Y=left Z=up */
    float yaw;        /* 工具轴俯仰角 (rad): 0=竖直向上, 90°=水平向前, 180°=竖直向下 */
} Kinematics_Pos;

/* Forward kinematics: 6 joint angles (radians) -> position
 * joints[0]=J1 yaw, joints[1..3]=J2/J3/J4 pitch, joints[4]=J5 roll(不改变位置),
 * joints[5]=J6 夹爪(不改变位置) */
int FK_Solve(const float joints_rad[NUM_JOINTS], Kinematics_Pos *result);

/* Inverse kinematics: target position (+工具俯仰 yaw) -> joint angles (radians)
 * J1-J4 求解; J5/J6 输出 0 (=保持中性, 调用方负责保持当前角度) */
int IK_Solve(const Kinematics_Pos *target, float joints_rad[NUM_JOINTS]);

/* Set per-joint safety limits (degrees, servo scale 0-360) */
/* Must be called before IK_Solve for limit-aware solving */
/* NOTE: 0-360 deg is discontinuous at the 0/360 boundary. For joints whose
 * mechanical range crosses 0, use IK_SetJointLimitsRaw() instead. */
void IK_SetJointLimits(const float min_deg[6], const float max_deg[6]);

/* Set per-joint safety limits in raw servo units (0-4095).
 * Correctly handles ranges that wrap across 0/360 (e.g. min=350 deg, max=10 deg).
 * zero_offset_deg should be the current SERVO_OFFSET array. */
void IK_SetJointLimitsRaw(const uint16_t min_raw[6], const uint16_t max_raw[6],
                          const float zero_offset_deg[6]);

/* World frame calibration offset (mm).
 * actual_position = internal_model + offset
 * Use this when your measurement origin/direction differs from code definition.
 * Set: offset = actual_measured - commanded_target */
extern float WORLD_OFFSET_X;
extern float WORLD_OFFSET_Y;
extern float WORLD_OFFSET_Z;
void Kinematics_SetOffset(float dx, float dy, float dz);

/* --- Conversion utilities --- */
/* Servo degrees (0-360) -> kinematics radians (0=center, per-joint offset+dir) */
float servo_deg_to_rad(float deg, int joint);

/* Kinematics radians (0=center) -> servo degrees (0-360, per-joint offset+dir) */
float rad_to_servo_deg(float rad, int joint);

/* Servo degrees (0-360) <-> STS3215 raw position (0-4095) */
uint16_t deg_to_servo_raw(float deg);
float servo_raw_to_deg(uint16_t raw);

/* --- 运行时可调 IK-CCD 参数（arm_params 指针绑定的生效点） ---
 * 默认值与 config/arm_config.h 第 10 节一致；修改入口：
 * LP/SP 文本命令、PARAM_READ/WRITE 协议、ArmParams_Set()。 */
extern float KIN_CCD_CONVERGE_MM;   /* CCD 收敛阈值 mm（默认 0.5） */
extern int   KIN_CCD_MAX_ITER;      /* CCD 单起点最大迭代（默认 100） */
extern float KIN_IK_FAIL_MM;        /* IK 失败判定阈值 mm（默认 1.0） */
extern float KIN_FK_VERIFY_MM;      /* FK 回代验证容差 mm（默认 5.0） */

/* --- 诊断: 读当前生效的 IK 关节限位（关节角空间, 度） ---
 * 用于 `zi` 命令核对 IK_SetJointLimitsRaw 到底算出了什么。
 * 这些物理上应满足: jlim[i] = JOINT_DIR[i] * (servo_deg - SERVO_OFFSET[i])。
 * 若 SERVO_OFFSET 被 Home 零点校准改过、而 jlim 没同步重算, 就会与
 * rad_to_servo_deg 的输出错位 —— 表现为"解析解明明可达却被拒, 静默降级 CCD"。 */
void IK_GetJointLimitsDeg(float min_deg[6], float max_deg[6]);

#endif
