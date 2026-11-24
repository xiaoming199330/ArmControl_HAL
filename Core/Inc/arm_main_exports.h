/* ============================================================
 * arm_main_exports.h - main.c 对控制接口层导出的符号清单
 *
 * 背景: main.c 业务层中的运动入口与调参变量原为 static，
 *       控制接口层 (arm_control_api.c / arm_params.c) 需要访问。
 *       本头文件集中声明这些导出符号，是 main.c 与新层之间的
 *       唯一耦合点。新增导出必须先在此登记（防 config-drift）。
 *
 * 约定: 关节索引 0~5；角度=舵机度 0~300；速度=deg/s。
 * 纪律: 这些符号的定义仍在 main.c，生效点唯一，禁止在别处
 *       重复定义同名变量（指针绑定原则，见 arm_params.c）。
 * ============================================================ */
#ifndef __ARM_MAIN_EXPORTS_H
#define __ARM_MAIN_EXPORTS_H

#include <stdint.h>

/* ---- main.c 导出的函数（原 static，已去 static） ---- */

/* 启动一条六关节同步 S 曲线轨迹（含 E-Stop 检查、限位钳制、
 * 读起点、开扭矩、规划、记录修正目标）。target 会被就地钳制。 */
void    cmdMove(float target[6], float v_max, float accel);

/* IK-CCD 解算单点：x/y/z (mm) + pitch (deg) -> 六关节舵机度。
 * 内部: IK_Solve -> 关节限位检查 -> FK 验证 (KIN_FK_VERIFY_MM)。
 * 返回 1 成功 / 0 失败（不可达、超限位或 FK 验证超差）。 */
int     cmdSolveIKPoint(float x, float y, float z, float pitch_deg,
                        float deg[6]);

/* E-Stop 状态：1=按下（低有效 PE11）。E-Stop 下禁止一切运动。 */
uint8_t IsEStop(void);

/* ---- 运行状态位图 (2026-09-17, CAN 周期状态帧用) ----
 * main.c 内部那些 static 状态 (流模式/示教录制/回放...) 集中在这里一次
 * 打包出去, 避免为每个位新增一个导出函数(防 config-drift)。
 * ⚠ 这里只放**读一个变量就能得到**的廉价标志。禁止在这里读舵机总线 ——
 *   状态帧是周期发送的(默认 1s), 每发一帧读 6 轴会把总线拖垮。 */
#define ARM_MAIN_FLAG_ESTOP       0x01U   /* E-Stop 激活 */
#define ARM_MAIN_FLAG_TRAJ        0x02U   /* S 曲线轨迹执行中 */
#define ARM_MAIN_FLAG_STREAM      0x04U   /* 主从直写流模式激活 */
#define ARM_MAIN_FLAG_CORR        0x08U   /* 自适应修正进行中 */
#define ARM_MAIN_FLAG_TEACH_REC   0x10U   /* 示教录制(含跟随录制)中 */
#define ARM_MAIN_FLAG_TEACH_PLAY  0x20U   /* 示教回放中 */
uint8_t ArmMain_Flags(void);

/* ---- 主从直写模式 (2026-09-11, 实时遥操作跟随) ----
 * 把主臂的【逻辑关节角】(相对本臂 Home, deg) 直接写进六轴目标寄存器,
 * 跳过轨迹规划 / 起点多数表决读 / settle 确认 —— 这三件事使 cmdMove()
 * 单次耗时几十~几百 ms 且要求臂已停稳, 无法用于 20~50Hz 连续跟随。
 * 内部仍保留三层安全检查 (raw 硬限位 clamp / 单帧跳变限制 / 桌面+立柱),
 * 任一不满足则【拒绝整帧】并保持上一点。
 * 返回: 0=已下发, 1=被安全层拒绝, 2=E-Stop。 */
uint8_t cmdStreamJoints(const float jdeg[6]);

/* 直写模式看门狗: 主循环每拍调用。断流超时后退出流模式并冻结
 * (位置模式下舵机保持最后目标, 不会失力砸臂)。 */
void    cmdStreamService(void);

/* 退出直写模式。任何轨迹类入口 (cmdMove / 示教回放 / 多点循环) 都应调用,
 * 避免流目标与规划轨迹互相打架。 */
void    cmdStreamExit(const char* why);

/* sf 跟随开关读写 (RC 遥控武装时自动恢复 sf=1, 避免旧 sf=0 状态锁死) */
uint8_t ArmMain_StreamFollowEn(void);
void    ArmMain_SetStreamFollowEn(uint8_t en);

/* ⭐0924CI: GET_STATUS 遥测缓存通道 —— 读 FeedbackSampleService 轮转缓存
 * (10ms/轴) 的关节角(舵机度 L1), 不占总线。返回 1=取到 / 0=缓存不可用
 * (过期/丢读/未采样, 调用方回落 SERVO_GetPosition 直读)。 */
uint8_t ArmMain_GetCachedJointDeg(uint8_t j, float* out_deg, uint32_t max_age_ms);

/* ---- RC 遥控互斥标志 (反向引用: 定义在 arm_rc_ctrl.c) ----
 * RC 遥控发流帧前置 1。cmdStreamJoints 收到标志=0 的流帧(网口/主从)
 * ⇒ 立即 ArmRcCtrl_Disarm()。单线程主循环内使用, 非 ISR 安全。 */
extern volatile uint8_t arm_rc_ctrl_in_frame;

/* ---- main.c 导出的调参变量（arm_params 指针绑定目标） ---- */

/* J2 重力前馈增益 K (deg/mm 水平臂展)。0=关闭，典型 0.02~0.05。 */
extern float    GRAVITY_COMP_K;

/* 自适应修正主开关：0=关 1=开（轨迹结束后自动运行修正状态机）。 */
extern uint8_t  correction_enabled;

/* 修正收敛阈值 (deg)：所有关节误差 <= 该值视为到位。0.5~10。 */
extern float    corr_threshold;

/* 修正最大轮数：超时放弃并打印 TIMEOUT。1~20。 */
extern uint8_t  corr_max_rounds;

/* 修正初始增益（每关节 0~0.85）：修正序列启动时装载。 */
extern float    corr_gain_init[6];

/* 0924AP: 轨迹收尾降速与尾段交接算法已移除, 以下参数仅保留给 LP/SP 兼容显示,
 * 不再影响运动。 */
extern float    TRAJ_APPROACH_SPEED;   /* (保留参数) */
extern float    TRAJ_APPROACH_FRAC;    /* (保留参数) */
extern float    TRAJ_HANDOFF_DEG;      /* (保留参数) */

/* 关节安全限位 (舵机度 0~300)。修改后必须 IK_SetJointLimits()。 */
extern float    JOINT_MIN[6];
extern float    JOINT_MAX[6];

/* ---- 网口文本命令透传 (2026-09-23) ----
 * 让 UDP 下发 COM11 控制台的**任意一条**命令, 而不是逐条加二进制 opcode。
 * 实现方式: 把一次 ProcessCommand 的回显从串口改道到内存, 再分页回传。
 * 这样 124 条命令(11 条抢 ETH 引脚的由 ProcessCommand 自己拦)全部可达,
 * 且**以后新增控制台命令会自动对网口可用**, 不存在"opcode 追不上"的问题。
 *
 * ⚠ 输出上限 2048 B(ARM_TEXT_BUF_SZ), 超出置 *truncated=1 —— 宁可截断并
 *   说明, 也不静默丢字符(读失败/截断都不许兜底成"看起来正常")。
 * ⚠ 必须主循环上下文调用; line 必须是**可写**缓冲(ProcessCommand 用 strtok 就地切)。 */

/* 安全闸门: 0 = 放行, 1 = 交互/阻塞类(会卡死主循环: 如单独一个 `c`),
 * 2 = 急停中只允许 s/h。 */
uint8_t  ArmMain_TextGate(const char* line);

/* 执行一行文本命令并捕获其控制台输出; 返回捕获字节数。 */
uint16_t ArmMain_ExecText(char* line, int len, uint8_t* truncated);

/* 上一次 ExecText 的捕获结果: 总长 / 按偏移读取(返回实际拷贝字节数)。 */
uint16_t ArmMain_TextLength(void);
uint8_t  ArmMain_TextRead(uint16_t off, uint8_t n, char* out);

/* 软复位: 先登记, 由主循环延后执行(保证 ACK 先发出去)。 */
void     ArmMain_RequestReset(uint16_t delay_ms);
void     ArmMain_ServicePendingReset(void);

/* 运行时限位 (RAM, apply_calib_limits 装载的 Phase2 校准值, `L` 同源)。
 * 返回装载轴数 (6=全装载), 未装载轴写 0。R1 的限位闸用这个, 别用
 * ArmApi_GetLimits() (那条路读扇区 11 flash, 本板无有效记录)。 */
uint8_t  ArmMain_GetJointLimitsDeg(float mn[6], float mx[6]);

/* 域转换 L1(servo 度) -> L2(关节度 (-180,180])。R1 喂 StreamJoints 前必须过它:
 * cmdSolveIKPoint/ArmApi_IkQuery/GetStatus 输出 L1, StreamJoints 输入 L2。 */
float    ArmMain_ServoToJoint1(float sdeg, uint8_t j);

/* 流层姿态安全判据只读导出 (桌面 z + 立柱圆柱, 输入 L2 关节度)。
 * RC 遥控发帧前预检用: 不过=缩步长软停, 而不是塞给流层拒帧熔断自锁。 */
uint8_t  ArmMain_StreamPoseOk(const float q[6]);

/* ⭐0924AJ 带实际位姿参考的判据: act_sdeg = 当前实际舵机度 (L1)。
 * 实际已在桌下时启用恢复规则 (目标只需不低于实际, 不许更深),
 * 修"触限位后六轴全冻" —— RC 两处预检都改用这个。 */
uint8_t  ArmMain_StreamPoseOkAct(const float q[6], const float act_sdeg[6]);

#endif /* __ARM_MAIN_EXPORTS_H */
