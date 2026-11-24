/* ============================================================
 * arm_config.h - 6-Axis Robotic Arm All-in-One Configuration
 *
 * This file consolidates ALL adjustable parameters scattered across:
 *   kinematics.h/c, servo_driver.c, main.c, trajectory.c
 *
 * To modify any parameter, edit this file and rebuild.
 * For runtime-adjustable params (joint limits, world offset),
 * use serial commands instead (L, C).
 *
 * Hardware: STM32F407ZGT6 + STS3215 (SCS) x6
 * Date: 2026-09-11
 *
 * ---- 重要说明 ----------------------------------------------------------
 * 本头文件是"可调参数镜像"，供阅读 / 移植 / 上位机对齐使用；
 * **固件并不 include 本文件**（没有 #include "arm_config.h"）。
 * 固件的唯一真相源:
 *   参数/命令默认值   Core/Src/main.c (ProcessCommand / cmdHelp)
 *   命令注册表        Core/Src/arm_commands.c
 *   运行时算法参数    Core/Src/arm_params.c (LP/SP, 指针绑定)
 *   二进制协议        Core/Inc/arm_control_protocol.h
 *   持久化            Core/Src/pid_flash.c (Flash Sector 11)
 * 改动固件默认值后, 请同步本文件 + arm_config.json + arm_config_readme.md。
 *
 * ⚠ 本文件不含主从臂 (Leader/Follower, UART4) 相关内容 —— 该链路另行联调,
 *   接口暂不对外发布。
 * ============================================================ */
#ifndef __ARM_CONFIG_H
#define __ARM_CONFIG_H

#include <stdint.h>

/* ############################################################
 * 1. ROBOT MECHANICAL DIMENSIONS (mm)
 *
 * AUTHORITATIVE source (2026-09-01 起): 官方 SO-ARM101 URDF
 *   (TheRobotStudio/SO-ARM100, Simulation/SO101/so101_new_calib.urdf)
 *   + 用户实测轴距。镜像自 Core/Inc/kinematics.h。
 * 上位机/仿真软件必须使用这一组值, 不要用下面的 _LEGACY 旧值。
 *
 * 坐标系: 右手系 X=前 Y=左 Z=上; 原点 = J1 轴心在桌面上的投影。
 * 工具 pitch 约定: 0=竖直向上, +90=水平向前, 180=竖直向下。
 * ############################################################ */
#define ROBOT_BASE_H       65.30f   /* 桌面 -> J1 轴心高度 */
#define ROBOT_J2_VERT      54.80f   /* J1 轴心 -> J2 轴心 垂直分量 */
#define ROBOT_J2_OFF_X     31.32f   /* J2 轴心相对 J1 轴心的前向偏移 */
#define ROBOT_L_UPPER     116.00f   /* 上臂 J2->J3 轴距 (中立倾角 13.97 deg) */
#define ROBOT_L_FORE      135.00f   /* 前臂 J3->J4 轴距 (中立倾角 87.79 deg) */
#define ROBOT_L_WRIST      62.50f   /* 腕 J4->J5 轴距 */
#define ROBOT_L_TOOL      105.30f   /* J5->TCP 沿前臂轴 */
#define ROBOT_L_DISTAL    167.80f   /* 远端 J4->TCP = 62.50 + 105.30 */

/* 关节中立倾角 (deg, 相对竖直, + = 前倾) */
#define ROBOT_K0_UPPER     13.97f
#define ROBOT_K0_FORE      87.79f

/* Derived: 最大水平臂展 (J2 -> TCP) */
#define ARM_MAX_REACH  (ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL)
/* = 418.80 mm */

/* 旧 XL320 时代的简化 DH 估计 (2026-07-22), 已被官方 URDF 校准取代。
 * 仅作历史留存, 与当前 FK/IK 不兼容, 不要使用。 */
#define ROBOT_D1_LEGACY    58.0f
#define ROBOT_A2_LEGACY    54.0f
#define ROBOT_A3_LEGACY    54.0f
#define ROBOT_A4_LEGACY    54.0f
#define ROBOT_A5_LEGACY    64.0f
#define ROBOT_A6_LEGACY     0.0f

/* ############################################################
 * 2. SERVO ZERO OFFSETS (degrees, 0-360 scale) / 零点
 * ############################################################ */
/* 含义: "运动学零点(官方 Home L 形姿态)" 对应的舵机角度。
 * 由 'c' 命令 Phase1 校准逐关节测量 (Kinematics_SetZeroRaw); 未校准时用
 * 默认 180.0 deg / raw 2048。
 *
 * ⚠ 下面这组 150.0 是 XL320 时代的占位值, 仅表示"结构上默认居中"的语义。
 *   本机 (2026-09 换 J2/J3 舵机后) 实测 zero 为置中式装盘: 各关节 raw≈2048
 *   => SERVO_OFFSET≈180 deg。实际值以 Flash 里 'c' 校准的为准, 上位机应通过
 *   's' / 'f' / 'p J' 读取 rel-home 关节角, 不要写死舵机度。
 *
 * ⚠ 位姿/路点常量禁止写死舵机度 (零点漂移可达 ~100 deg);
 *   一律用"关节角 + 运行时换算": servo = SERVO_OFFSET + JOINT_DIR * joint_deg。
 */
#define SERVO_OFFSET_J1  180.0f
#define SERVO_OFFSET_J2  180.0f
#define SERVO_OFFSET_J3  180.0f
#define SERVO_OFFSET_J4  180.0f
#define SERVO_OFFSET_J5  180.0f
#define SERVO_OFFSET_J6  180.0f

/* Array form (for code compatibility) */
#define SERVO_OFFSET_INIT  { 180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f }

/* 关节方向: +1 = 舵机角增大 -> 正向运动; -1 = 反向 (J1 反装) */
#define JOINT_DIR_INIT  { -1, 1, 1, 1, 1, 1 }

/* 舵机角度/raw 范围 (STS3215 为 11 位, 0~4095; 旧 XL320 才是 0~1023) */
#define SERVO_MIN_DEG     0.0f
#define SERVO_MAX_DEG     360.0f
#define SERVO_RAW_MIN     0
#define SERVO_RAW_MAX     4095
#define SERVO_RAW_CENTER  2048
#define SERVO_DEG_PER_RAW (360.0f / 4095.0f)   /* 0.08791 deg/raw */

/* ############################################################
 * 3. JOINT SAFETY LIMITS (degrees, 0-360 scale, 仅显示用)
 * ############################################################
 * ⚠ 权威限位在 raw 空间 (JOINT_MIN_RAW/MAX_RAW, 0~4095), 由 'c' Phase2 校准
 *   测量极值 + 'L J min max' 运行时重算。下面这组 deg 值只用于显示, 且是
 *   开机默认; 上位机要拿实时值请查询 'L' 或 's'。
 *
 * ⚠ J3/J5 特例【已于 2026-09-16 取消】: 旧版这两轴不采用 Phase2 极值, 而是
 *   = calib_zero_raw ± 写死偏移。现改为**六轴一视同仁**, 全部采用 Phase 2 手推实测值
 *   (见 main.c apply_calib_limits, 内置偏移宏与 hm 开关均已删除)。
 *   ⇒ 换舵机或重装盘位器后**重做一遍 Phase 2 手推标定**即可, 无需改宏重编译。
 *
 * 关节含义: J1=Yaw 底座旋转; J2=肩; J3=肘; J4=腕俯仰; J5=腕滚转; J6=夹爪。
 * ⚠ 旧版"J5 永不让 IK 进入 77~174 卡区, 硬覆盖 180~340"已**随 J5 改用实测值一并取消**
 *   (2026-09-16 实机复核: J5 能从 208.35° 平滑走到 89.76° 穿过该区间, 全程未堵转)。
 */
#define JOINT_MIN_INIT  {   0,  58,  10,  83, 180,   0 }
#define JOINT_MAX_INIT  { 360, 264, 177, 289, 340, 360 }

/* J3/J5 挡块偏移【已作废 · 2026-09-16】 (相对 calib_zero_raw, 单位 raw; 曾定义在 main.c)
 *
 * ⚠ J5 诊断结论 (2026-09-15, 两臂 `zi` 实测 + 36 s 抓包, 见
 *   docs/J5跟随慢_限位排查_2026-09-15.md §0.3 / §5):
 *
 *   (a) 这对偏移**本身没错**, 它编码的是"设计意图的 160° 挡块带":
 *         旧 epoch zero=3081 时 -> raw [2048, 3868] = servo [180.0°, 340.0°]
 *         —— 正是 arm_config.h:118 "J5 永不让 IK 进入 77~174 卡区, 硬覆盖 180~340"。
 *       换舵机 / 重装盘位器后 Home 从 raw 3081 变成 2048 (整条 raw 刻度平移 -1033),
 *       同一块**物理**挡块带随之落到 raw [1015, 2835] —— `zero + offset` 自动跟着平移。
 *       ⇒ **不需要因为"是旧舵机时代量的"而改它。** 旧注释 `待重测 (旧舵机值)` 会把人
 *         带向"偏移过时 / 换过舵机"的错误方向 (2026-09-15 已因此绕了一整轮排查), 已更正。
 *       ⚠ 用户 2026-09-15 确认: **J5 舵机从未更换过。**
 *
 *   (b) **真正的 bug: 这个 160° 窗口 ⊂ J5 的实际物理行程。**
 *       主臂是 `ma 0`(扭矩 OFF、人力拖动) ⇒ 读数**不受任何限位约束**, 实测能把 J5
 *       推过 **241.2°** 对应行程; 而从臂窗口只有 160° ⇒ 由 `clamp_raw()` **静默**夹住
 *       (实测从臂 J5 钉死在 servo 89.67° = raw 1015 = 窗口下限, 而窗口内跟随非常准
 *       ⇒ 与舵机无关)。这就是"J5 跟不动 / 跟得慢"。
 *
 *   ⇒ 已结案 (2026-09-16): 用户实机验证"手推完全可用" ⇒ J3/J5 与其余四轴统一,
 *     直接采用 Phase 2 手推实测的 calib_min_raw / calib_max_raw; 两对偏移宏与
 *     运行期开关 hm **均已从 main.c 删除**。原推算的 -1843/+899 一并作废。
 */
/* ⚠⚠ 下面四个宏已【作废并从 main.c 删除】(2026-09-16)。本文件是**从不参与编译**的
 *     镜像副本 (Makefile 里没有 -Iconfig), 这里的值仅作历史留档, 不要再照抄进任何
 *     代码或工具。换舵机 / 重锚 Home 零点后, 走 Phase 2 重新手推标定即可。 */
#define J3_LIMIT_OFF_BACK   (-1163)   /* 已作废 */
#define J3_LIMIT_OFF_FRONT  ( 1090)   /* 已作废 (曾: 1038 -> 1090, 收纳位 raw 3112) */
#define J5_LIMIT_OFF_MIN    (-1033)   /* 已作废 */
#define J5_LIMIT_OFF_MAX    (  787)   /* 已作废 */

/* ############################################################
 * 4. PREDEFINED POSES  (P 0~4)
 * ############################################################
 * 单位 = "相对运动学零点(官方 Home)的关节角" deg, **不是舵机度**。
 * 运行时由 refresh_pose_servo() 换算: servo = SERVO_OFFSET + JOINT_DIR*joint,
 * 随 'c' 校准自动跟随。定义源 = main.c POSE_*_J。
 *
 * P0_HOME = 折叠收纳实测量 (2026-09-11 用户 f 实测), 爪尖离桌 ~10 mm;
 *           该姿态由"折叠位白名单"保证无条件可达 (贴死 J3 前挡块)。
 * P1_FOLD = 备用收拢位。 P2_READY = 前伸待命。 P3/P4 = 挥手位 (J1 反向)。
 * J5/J6 = 0 表示 P 命令不改变这两轴。
 */
#define POSE_HOME_INIT   {   0.18f, -102.95f, 93.54f, 71.65f, -0.88f, -50.29f }
#define POSE_FOLD_INIT   {   0.0f,    40.0f, -70.0f, 40.0f,   0.0f,   0.0f }
#define POSE_READY_INIT  {   0.0f,     0.0f, -30.0f, 40.0f,   0.0f,   0.0f }
#define POSE_WAVE1_INIT  {  45.0f,   -30.0f, -80.0f, 50.0f,   0.0f,   0.0f }
#define POSE_WAVE2_INIT  { -45.0f,   -30.0f, -80.0f, 50.0f,   0.0f,   0.0f }

/* ############################################################
 * 5. WORLD FRAME CALIBRATION OFFSET (mm)
 * ############################################################ */
/* actual_position = internal_model + offset
 * offset = actual_measured - commanded_target
 * Runtime adjustable via 'C dx dy dz'; RAM only -> 需 'F' 才存盘。
 * Default: 0,0,0 (no offset)
 *
 * ⚠ 路径安全检查 (fk_path_z_ok) 恒用基座坐标 (p.z - WORLD_OFFSET_Z),
 *   而 'f' / 'g' 用世界坐标 —— 调试 z 时注意差一个 offset。
 */
#define WORLD_OFFSET_X_DEFAULT  0.0f
#define WORLD_OFFSET_Y_DEFAULT  0.0f
#define WORLD_OFFSET_Z_DEFAULT  0.0f

/* ===== 折叠位白名单 (HOME dock, 2026-09-11) =====
 * 折叠收纳位(POSE_HOME_J) 是贴死 J3 前挡块的姿态, 必须无条件可达:
 *   1) 桌面下限 traj_z_floor_mm = FK(POSE_HOME).z - TRAJ_Z_MARGIN_MM;
 *   2) 采样点 6 轴全在 POSE_HOME ± HOME_DOCK_TOL_DEG 内时, 跳过 raw 限位与
 *      桌面 z 检查 (立柱圆柱检查不豁免)。
 * 改 POSE_HOME_J 即自动跟随, 阈值无需手改。
 * ⚠ 2026-09-16: 下限的真实含义 = "爪尖不低于【停靠位真实高度】- 3mm"。锚点与
 *   路径检查用的是同一套 FK, 模型绝对偏置在比较中自动抵消, 与模型 z 报多少
 *   无关 (09-11 模型 z=4.0mm、09-16 模型 z=19.5mm, 实测都是 ~10mm ⇒ 两次偏差
 *   符号相反, 至少一条目测不准)。这不影响下限安全性, 但**别拿 f 的 z 当桌面
 *   间隙读数**。
 *   注: 2026-09-16 改 POSE_HOME_J 后, 模型 z 由 4.0 抬到 19.5mm ⇒ 下限由
 *   0.98 抬到 16.5mm, 贴近桌面的轨迹会比之前更容易被拒。
 */
#define TRAJ_Z_MARGIN_MM   3.0f    /* 折叠位模型 z 之下留的余量 */
#define HOME_DOCK_TOL_DEG  3.0f    /* 折叠位白名单半径 (deg, 六轴需同时满足) */

/* ############################################################
 * 6. SERVO BUS & ID CONFIGURATION
 * ############################################################ */
/* Two half-duplex TTL buses via 74HC126 direction control */
#define BUS_A  0   /* USART3 (PB10=TX, PB11=RX) */
#define BUS_B  1   /* UART5  (PC12=TX, PD2=RX)  */

/* Servo ID assignment (Feetech ID, 1-253) */
#define JOINT_ID_INIT   { 1, 2, 3, 4, 5, 6 }

/* Which bus each joint is on */
#define JOINT_BUS_INIT  { BUS_A, BUS_A, BUS_A, BUS_B, BUS_B, BUS_B }

/* ############################################################
 * 7. SERVO PID & SPEED  (改动前必读本节注释!)
 * ############################################################
 * 生效链: 串口 K/I/D/N  ->  立即写舵机 RAM (即时生效)
 *          Y J          ->  烧进舵机 EEPROM (舵机掉电保留)
 *          F            ->  存 STM32 内部 Flash (固件开机自动下发)
 *          X            ->  只恢复 RAM + 舵机 RAM 为固件默认, 两处持久化都不动
 * 取值范围 0~254。
 *
 * ---- 三个增益管什么 / 何时增、何时减 ----------------------------------
 *  P (Kp 位置刚度):
 *     增大 -> 响应更快、更"硬"、抗重力下垂更强、稳态误差更小
 *     减小 -> 更柔、噪声更小, 但稳态误差变大、负重会下沉
 *     症状: 到点软 / 负重下沉       -> 加大 P
 *           高频抖动 / 嗡鸣 / 噪声被放大 -> 减小 P
 *
 *  I (Ki 积分):
 *     增大 -> 消除稳态误差(缓慢重力下沉), 到位更准
 *     减小 -> 消除慢速振荡/超调; 太小则永远补不掉静态误差
 *     症状: 到位后慢慢下沉 / 反复微调 -> 加大 I
 *           低频来回晃 / 积分饱和     -> 减小 I
 *
 *  D (Kd 微分/阻尼):
 *     增大 -> 阻尼增强、抑制过冲与振荡、到位更利落
 *     减小 -> 不再放大反馈噪声
 *     症状: 到点来回荡 -> 加大 D
 *           高频剧烈抖动 / 打摆 -> 减小 D
 *
 * ---- 本机血泪结论 (务必遵守) ------------------------------------------
 *  1) D 对"齿隙大的关节(J3)"最敏感, 不要盲目加大。
 *     2026-09-10 曾把 J3 烧成 P50/I2/D30 -> 回折叠位到点剧烈颤抖;
 *     恢复默认 P16/I4/D12 后立即消失。J3 减速箱 9-01 撞击后有损伤齿隙。
 *  2) I 不是越小越好: J3 的 I 压到 2 会重力位姿下沉、位置环反复拉扯。
 *  3) 改 PID 前先记录基线 (N 1~N 6), 一次只改一个关节一个量。
 *  4) 先排除"不是 PID": 本机"到点顿两下"根因是尾段流式微步与舵机内部
 *     planner 的交互 (收尾降速 #42 / 尾段交接 #45), 与 D 无关。
 *  5) 改完必须 Y J (烧舵机 EEPROM) + F (存 STM32 Flash), 否则断电回退。
 *  6) J2 的 P 单独为 32 (其它 16): 肩部承担整臂重力, P 小会明显下沉。
 * ---------------------------------------------------------------------- */

/* 单增益默认值 (仅供参考, 实际逐关节见 *_INIT 数组) */
#define STS3215_P_GAIN         16     /* Position gain */
#define STS3215_I_GAIN          4     /* Integral gain; 消除稳态重力误差 */
#define STS3215_D_GAIN         12     /* Derivative gain; 阻尼 */
#define STS3215_MOVING_SPEED  200     /* 舵机内部速度上限(addr46), 1单位=4.39 deg/s */
#define STS3215_TORQUE_LIMIT 4095     /* 力矩限制 (STS3215: 0~4095, 4095=最大) */
#define STS3215_HOMING_SPEED   30     /* 归零速度 (更慢更安全) */

/* Per-joint PID cache/defaults. Array order is J1..J6. */
#define STS3215_P_GAIN_INIT     { 16, 32, 16, 16, 16, 16 }   /* J2 加重 */
#define STS3215_I_GAIN_INIT     {  4,  4,  4,  4,  4,  4 }
#define STS3215_D_GAIN_INIT     { 12, 12, 12, 12, 12, 12 }
#define STS3215_PID_GAIN_MIN    0
#define STS3215_PID_GAIN_MAX    254

/* STS3215 EEPROM settings (applied during init;
 * 注意 STS3215 与旧 XL320 的 Mode 语义不同!) */
#define STS3215_CW_ANGLE_LIMIT     0      /* Min Angle Limit (EEPROM addr 9) */
#define STS3215_CCW_ANGLE_LIMIT 4095      /* Max Angle Limit (EEPROM addr 11) */
#define STS3215_CONTROL_MODE        0     /* 0=位置伺服(舵机)模式 <- 目标值
                                           * 1=恒速模式(写 GoalSpeed 会自转!)
                                           * 2=PWM 开环, 3=步进 */
#define STS3215_LOCK_EEPROM         1     /* EEPROM 锁 (55): 写完置 1 */

/* 速度寄存器(addr46)单位换算 (上位机算速度必看):
 *   1 单位 = 50 步/s = 4.39 deg/s; **写 0 = 不限速**
 *   20->88, 30->132, 40->176, 60->264, 68~300(物理满速), >=100 近似不限速
 * 加速度(addr41): 1 单位 = 100 步/s^2; 0 = 不限制 */
#define STS3215_SPEED_UNIT_DEG_S   (4.39f)
#define STS3215_DEFAULT_ACCEL      10     /* RAM addr41, 改后需 'i' 重新下发 */

/* ############################################################
 * 8. UART / COMMUNICATION CONFIGURATION
 * ############################################################ */
/* Console UART */
#define CONSOLE_UART   USART6
#ifndef CONSOLE_BAUD          /* 可被 `make CBAUD=460800` 覆盖, 见 Makefile */
#define CONSOLE_BAUD   115200
#endif
#define CONSOLE_TX_PIN  GPIO_PIN_6   /* PC6 */
#define CONSOLE_RX_PIN  GPIO_PIN_7   /* PC7 */
#define CONSOLE_AF      GPIO_AF8_USART6
#define CONSOLE_PORT    GPIOC

/* Servo bus baud rate (STS3215 default = 1 Mbps) */
#define SERVO_BUS_BAUD  1000000

/* Bus A: USART3 */
#define BUS_A_UART      USART3
#define BUS_A_TX_PIN    GPIO_PIN_10   /* PB10 */
#define BUS_A_RX_PIN    GPIO_PIN_11   /* PB11 */
#define BUS_A_PORT      GPIOB

/* Bus B: UART5 */
#define BUS_B_UART      UART5
#define BUS_B_TX_PIN    GPIO_PIN_12   /* PC12 */
#define BUS_B_RX_PIN    GPIO_PIN_2    /* PD2 */
#define BUS_B_TX_PORT   GPIOC
#define BUS_B_RX_PORT   GPIOD

/* Direction control pins (74HC126) */
#define DIR_A_PIN       GPIO_PIN_8    /* PE8 */
#define DIR_A_PORT      GPIOE
#define DIR_B_PIN       GPIO_PIN_10   /* PE10 */
#define DIR_B_PORT      GPIOE

/* Communication timeouts */
#define DXL_TIMEOUT_PING     50    /* ms */
#define DXL_TIMEOUT_READ    100    /* ms */
#define DXL_TIMEOUT_WRITE   100    /* ms */
#define DXL_REBOOT_DELAY    600    /* ms, wait for servo reboot */

/* ############################################################
 * 8b. CONSOLE PROTOCOL CONTRACT  (上位机必读)
 * ############################################################
 * 与控制台(USART6)通信的"接口契约"。固件实现见 main.c Console_TryReadLine()
 * / ProcessCommand(), 协议帧见 Core/Inc/arm_control_protocol.h。
 *
 * ---- 行尾类型 (LINE ENDING) —— 上位机头号坑 --------------------------
 *   固件把 '\r'(0x0D) 和 '\n'(0x0A) 都当作"一行结束":
 *       if (ch == '\r' || ch == '\n') { ... 执行命令 ... }
 *   | 行尾     | 字节       | 是否执行 |
 *   | CRLF     | 0D 0A      | 是 (推荐) |
 *   | LF       | 0A         | 是        |
 *   | CR       | 0D         | 是        |
 *   | 无行尾   | -          | **否, 永不执行** (表现为"下发了没响应") |
 *   | 空行     | -          | 忽略, 不报错 |
 *   结论: **每条命令末尾必须带 \r 或 \n 或 \r\n, 统一用 \r\n 最保险。**
 *
 * ---- 其余解析规则 ----------------------------------------------------
 *   字符集    : ASCII (不要发中文/全角)
 *   单行上限  : 127 字符 (缓冲 128)
 *   分词      : 仅空格 / Tab(0x09); 不认逗号、分号
 *   参数个数  : 最多 16 个 token, 多余忽略
 *   大小写    : 严格区分! m!=M, p!=P, s!=S, t!=T, b!=B, c!=C, d!=D ...
 *   数字      : 角度/坐标 atof(支持小数); 关节号/枚举 atoi(整数)
 *   回显      : 每个字符原样回显; 行尾打印 "\r\n", 执行后打印提示符 "> "
 *   不支持    : ANSI 转义、引号包裹、注释
 *
 * ---- 二进制协议 (同一串口) -------------------------------------------
 *   帧: AA 55 SEQ CMD LEN PAYLOAD CRC16(2B, 低在前)
 *   CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF), 范围 SEQ..PAYLOAD
 *   定点: 角度 int16 x0.1deg; 位置 int16 x0.1mm; 速度 u16 x0.1deg/s
 *   响应: 查询类 CMD|0x80; 控制类 ACK=0x7F / NACK=0x7E + 错误码
 *   分流: 以 0xAA 0x55 作为二进制帧起始判据 (文本控制台不会输出 0xAA)
 * ============================================================ */
#define CONSOLE_LINE_END_RECOMMENDED  "\\r\\n"  /* CRLF */
#define CONSOLE_LINE_END_ALT1         "\\n"     /* LF   */
#define CONSOLE_LINE_END_ALT2         "\\r"     /* CR   */
#define CONSOLE_MAX_LINE_LEN          127
#define CONSOLE_MAX_TOKENS            16
#define APROTO_HEAD0                  0xAA
#define APROTO_HEAD1                  0x55

/* ############################################################
 * 9. TRAJECTORY PARAMETERS
 * ############################################################ */
/* TIM2 interrupt: 2kHz, trajectory update: 200Hz (every 10 ticks) */
#define TRAJ_UPDATE_HZ       200    /* Hz */
#define TRAJ_TICK_DIVIDER      10   /* 2000 / 10 = 200 */

/* Default motion parameters */
#define TRAJ_DEFAULT_V_MAX   30.0f  /* deg/s, default max speed */
#define TRAJ_DEFAULT_ACCEL   60.0f  /* deg/s^2, default acceleration */
#define TRAJ_V_MIN            5.0f  /* deg/s, minimum allowed speed */
#define TRAJ_V_MAX          300.0f  /* deg/s, maximum allowed speed */
#define TRAJ_DEFAULT_ACCEL_FACTOR 2.0f /* m command uses accel = speed * factor */
#define SWING_DEFAULT_COUNT       3   /* W without count */
#define SWING_MAX_COUNT          20
#define SWEEP_SPEED_DEG_S        30.0f

/* ############################################################
 * 10. IK / CCD SOLVER PARAMETERS
 * ############################################################ */
#define CCD_MAX_ITERATIONS    100    /* CCD iterations per start */
#define CCD_CONVERGE_MM      0.5f   /* Convergence threshold (mm) */
#define CCD_START_CONFIGS       5    /* Number of initial configurations */
#define IK_FAIL_THRESHOLD    1.0f   /* mm, IK fails if error > this */
#define IK_FK_VERIFY_MAX      5.0f  /* mm, cancel move if FK verify error > this */
#define IK_DEFAULT_PITCH_DEG  0.0f  /* g x y z without pitch */

/* ############################################################
 * 11. GPIO PIN ASSIGNMENTS
 * ############################################################ */
/* E-Stop (active low) */
#define ESTOP_PIN      GPIO_PIN_11
#define ESTOP_PORT     GPIOE

/* Limit switches J1-J6 (active low, PG0-PG5) */
#define LIMIT_PINS     { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, \
                          GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 }
#define LIMIT_PORT     GPIOG

/* LEDs (active high, need 150 ohm resistor) */
#define LED_R_PIN      GPIO_PIN_5   /* PC5 = Red */
#define LED_R_PORT     GPIOC
#define LED_G_PIN      GPIO_PIN_4   /* PC4 = Green */
#define LED_G_PORT     GPIOC
#define LED_B_PIN      GPIO_PIN_5   /* PA5 = Blue */
#define LED_B_PORT     GPIOA

/* Buzzer (active high) */
#define BUZZER_PIN     GPIO_PIN_5   /* PB5 */
#define BUZZER_PORT    GPIOB

/* SWD debug (do NOT repurpose!) */
#define SWD_IO_PIN     GPIO_PIN_13  /* PA13 */
#define SWD_CLK_PIN    GPIO_PIN_14  /* PA14 */

/* ############################################################
 * 12. CLOCK CONFIGURATION
 * ############################################################ */
/* HSE = 8 MHz external crystal */
#define CLOCK_HSE_MHZ      8
#define CLOCK_PLL_M         8       /* VCO input = 8/8 = 1 MHz */
#define CLOCK_PLL_N         336     /* VCO output = 1*336 = 336 MHz */
#define CLOCK_PLL_P         2       /* SYSCLK = 336/2 = 168 MHz */
#define CLOCK_PLL_Q         7       /* USB = 336/7 = 48 MHz */
#define CLOCK_SYSCLK_MHZ   168
#define CLOCK_APB1_MHZ      42      /* APB1 = 168/4 = 42 MHz */
#define CLOCK_APB2_MHZ      84      /* APB2 = 168/2 = 84 MHz */

/* TIM2: APB1 timer clock = 42*2 = 84 MHz
 * PSC=83999 -> 84MHz/84000 = 1kHz... no:
 * PSC=41 -> 84MHz/42 = 2MHz, ARR=999 -> 2kHz
 * Actually configured in CubeMX, ARR=499 -> 2kHz correct */
#define TIM2_ARR            499     /* 2kHz interrupt */

/* ############################################################
 * 13. GRAVITY FEEDFORWARD & ADAPTIVE CORRECTION
 * ############################################################ */
/* Runtime commands: G [k], A, A e 0/1, A t val, A r val, A g J val, R. */
#define GRAVITY_COMP_K_DEFAULT       0.0f
#define GRAVITY_COMP_K_MIN           0.0f
#define GRAVITY_COMP_K_MAX           0.5f
#define CORR_ENABLED_DEFAULT         1
#define CORR_THRESHOLD_DEFAULT       2.0f
#define CORR_THRESHOLD_MIN           0.5f
#define CORR_THRESHOLD_MAX          10.0f
#define CORR_MAX_ROUNDS_DEFAULT      10
#define CORR_MAX_ROUNDS_MIN          1
#define CORR_MAX_ROUNDS_MAX         20
#define CORR_FIRST_JOINT             1   /* zero-based: J2 */
#define CORR_LAST_JOINT              3   /* zero-based: J4 */
#define CORR_GAIN_INIT               { 0.0f, 0.5f, 0.3f, 0.2f, 0.0f, 0.0f }
#define CORR_GAIN_MIN                0.0f
#define CORR_GAIN_MAX                0.85f
#define CORR_GAIN_OVERSHOOT_SCALE    0.5f
#define CORR_GAIN_UNDERSHOOT_SCALE   1.25f
#define CORR_GAIN_STALL_SCALE        1.4f
#define CORR_OSCILLATION_TRIGGER     2
#define CORR_STALL_P_TRIGGER         2
#define CORR_STALL_I_TRIGGER         3
#define CORR_AUTO_D_STEP             4
#define CORR_AUTO_D_MAX             32
#define CORR_AUTO_P_STEP             8
#define CORR_AUTO_P_MAX             64
#define CORR_AUTO_I_STEP             2
#define CORR_AUTO_I_MAX             16
#define CORR_START_DELAY_MS        500

/* ############################################################
 * 14. SYSTEM CONSTANTS
 * ############################################################ */
#define NUM_JOINTS           6
#define RX_BUF_SIZE        128      /* Console RX ring buffer */
#define CONSOLE_TX_BUF      256     /* Console TX buffer */

/* ############################################################
 * 15. STS3215 REGISTER ADDRESSES (reference, not adjustable)
 * ############################################################ */
/* EEPROM */
#define STS3215_ADDR_MODEL_NUMBER          0
#define STS3215_ADDR_ID                    3
#define STS3215_ADDR_BAUD_RATE             4
#define STS3215_ADDR_RETURN_DELAY           5
#define STS3215_ADDR_CW_ANGLE_LIMIT         6
#define STS3215_ADDR_CCW_ANGLE_LIMIT        8
#define STS3215_ADDR_CONTROL_MODE          11
#define STS3215_ADDR_TEMP_LIMIT            12
#define STS3215_ADDR_MIN_VOLTAGE_LIMIT     13
#define STS3215_ADDR_MAX_VOLTAGE_LIMIT     14
#define STS3215_ADDR_MAX_TORQUE            15
#define STS3215_ADDR_STATUS_RETURN_LEVEL   17
#define STS3215_ADDR_SHUTDOWN              18
/* RAM */
#define STS3215_ADDR_TORQUE_ENABLE         24
#define STS3215_ADDR_LED                   25
#define STS3215_ADDR_D_GAIN                27
#define STS3215_ADDR_I_GAIN                28
#define STS3215_ADDR_P_GAIN                29
#define STS3215_ADDR_GOAL_POSITION         30
#define STS3215_ADDR_MOVING_SPEED          32
#define STS3215_ADDR_TORQUE_LIMIT          35
#define STS3215_ADDR_PRESENT_POSITION      37
#define STS3215_ADDR_PRESENT_SPEED         39
#define STS3215_ADDR_PRESENT_LOAD          41
#define STS3215_ADDR_PRESENT_VOLTAGE       45
#define STS3215_ADDR_PRESENT_TEMP          46
#define STS3215_ADDR_MOVING                49
#define STS3215_ADDR_HW_ERROR              50
#define STS3215_ADDR_PUNCH                 51

/* ############################################################
 * 16. RUNTIME ALGORITHM PARAM TABLE (LP / SP)
 * ############################################################
 * 镜像 Core/Src/arm_params.c。'LP' 列出 / 'SP <id> <val>' 设置。
 * ⚠ 只改 RAM, 需 'F' 才存盘。id 不连续是历史原因。
 * ⚠ 表长固定 24 项 (与 joint_limits 同存一条 Flash 记录); 不要随意增项,
 *   否则 sizeof 变化会让旧 Flash 记录作废 (丢 PID/校准限位)。
 */
/* id 1  ik.converge_mm      0.5   (0.1~5.0)   mm       CCD 收敛阈值        */
/* id 2  ik.max_iter         100   (10~500)    iter     CCD 单起点最大迭代  */
/* id 3  ik.fail_mm          1.0   (0.5~10.0)  mm       IK 失败判定         */
/* id 4  ik.fk_verify_mm     5.0   (1.0~20.0)  mm       运动后 FK 验证容差  */
/* id 16 sc.tj               0.15  (0.02~0.50) s        jerk 过渡时间       */
/* id 17 sc.def_vmax         30    (5~300)     deg/s    默认最大速度        */
/* id 18 sc.def_accel        60    (10~600)    deg/s^2  默认加速度          */
/* id 19 sc.v_min            5     (1~50)      deg/s    最小允许速度        */
/* id 20 sc.v_max            300   (50~300)    deg/s    最大允许速度        */
/* id 21 sc.accel_factor     2.0   (1.0~4.0)   ratio    未给加速度 a=v*factor*/
/* id 32 gc.k                0.0   (0~0.5)     deg/mm   J2 重力前馈(仅示教)  */
/* id 33 corr.enabled        1     (0/1)               自适应修正开关      */
/* id 34 corr.threshold      2.0   (0.5~10.0)  deg      修正收敛阈值        */
/* id 35 corr.max_rounds     10    (1~20)      rounds   最大修正轮数        */
/* id 36~41 corr.gain_j1..j6 0.2/0.5/0.3/0.2/0.2/0.2 (0~0.85) 各轴初始修正增益 */
/* id 42 traj.approach_speed 30    (0~4095)            收尾段舵机速度(见§7换算)*/
/* id 43 traj.approach_frac  0.05  (0~0.25)    ratio    收尾段占全程比例    */
/* id 44 servo.accel         10    (0~254)             舵机内部加速度(改后需 i)*/
/* id 45 traj.handoff_deg    0.0                       尾段交接阈值(0=关)  */

/* ############################################################
 * 17. TEACH PWM ZERO-FORCE PARAMS (示教零力, 2026-09 定稿)
 * ############################################################
 * 混合模式: J2/J3/J4 进 Mode2 开环力控; J1/J5/J6 保持 Mode0 + Torque OFF
 * (真 free)。默认值镜像 main.c。
 */
#define TEACH_PWM_GAIN_INIT   { 0, 6, 6, 6, 0, 0 }   /* ×0.1 duty/mm (质心加权模型) */
#define TEACH_PWM_DAMP_INIT   { 0, 3, 3, 3, 0, 0 }   /* 速度阻尼 */
#define TEACH_PWM_CAP_INIT    { 100, 100, 40, 100, 100, 100 } /* duty 限幅 (J3=40 防损伤共振) */
#define TEACH_PWM_FLIP_INIT   { 1, 1, 1, 1, 1, 1 }   /* 重力方向翻转 */
#define TEACH_FC_DUTY_INIT    { 0, 0, 0, 0, 0, 0 }   /* 库伦摩擦补偿 */
#define TEACH_DITHER_INIT     { 0, 0, 0, 0, 0, 0 }   /* 静摩擦 dither */
#define TEACH_PWM_OFF_INIT    { 0, 0, 0, 0, 0, 0 }   /* 手动恒定偏置 (teach o) */
#define TEACH_PWM_DEADBAND    20                      /* 死区避开值 duty (~2%) */
#define TEACH_SOFT_PID_DEFAULT { 0, 0, 0 }            /* 软跟随 P/I/D */

/* ############################################################
 * 18. SCOPE NOTE
 * ############################################################
 * 本文件/arm_config.json/arm_config_readme.md 描述的是**单臂**接口。
 * 主从臂 (Leader/Follower) 通过 UART4(PA0/PA1) 的链路 (帧 0x13 流式关节角、
 * 命令 ms/lk/mf/ma/sf) **不在本次接口范围内**, 另行联调后再对外发布。
 */

#endif /* __ARM_CONFIG_H */
