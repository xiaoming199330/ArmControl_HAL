/* ============================================================
 * arm_rc_ctrl.h - R1: RC 遥控 -> 末端控制 (速度流)
 *
 * 定案依据: docs/RC方案复核_2026-09-23.md (P0-1/P0-2/P0-3) +
 *           docs/RC遥控器控制机械臂_方案设计_2026-09-21.md §2.12
 *
 * 通路: 50Hz 拍读 ArmRc_Get() -> 杆量->速度(底座随动系) -> 笛卡尔积分器 ->
 *       ArmApi_IkQuery(IK+限位+FK验证, 输出 **L1 servo 度**) ->
 *       ArmMain_ServoToJoint1 逐轴转 **L2 关节度** -> 填 J5/J6 ->
 *       ArmApi_StreamJoints()(输入 L2; 内含 raw 硬限位/跳变/桌面+立柱守卫)。
 *
 * ⭐⭐ 域铁律 (2026-09-23 "乱走"事故的根因, 见复核 §8):
 *   本工程两层角度: L1 = servo 度 0~360 (cmdSolveIKPoint/ArmApi_IkQuery/
 *   GetStatus/校准限位/`L` 全是它); L2 = 关节度 (-180,180] (IK/FK 工作空间、
 *   StreamJoints 输入与跳变守卫)。SERVO_OFFSET≈180 ⇒ L1 直喂 L2 入口 =
 *   双重映射, 每轴错 ~180°。任何新代码过这两层必须显式转换, 只许用
 *   ArmMain_ServoToJoint1()。
 *
 * ⭐ P0-1 虚拟目标生命周期:
 *   - 会话启动时用 ArmApi_GetStatus() **实测**位姿做基准;
 *   - 增量只累加到"上次被接受的"目标 —— IK 拒帧/流拒帧都**不累加**;
 *   - 断流超时(500ms)后下一拍自动重新实测基准 (防积分器漂移).
 *   - 0924I 起 pitch 不再冻死: 俯仰双键长按调 pitch_t (45°/s, 钳 15~165°),
 *     同样只在帧被接受时前进, 拒帧即回退 (无 windup)。
 *
 * ⭐ P0-2 与网口/文本运动互斥:
 *   - 本模块每帧打 arm_rc_ctrl_in_frame 标志; cmdStreamJoints 收到
 *     **非本模块**的流帧(网口 STREAM_JOINTS / 主从 #P 中转)⇒ 立即 disarm;
 *   - cmdStreamExit() (文本 m 等轨迹入口) ⇒ disarm;
 *   - 轨迹/修正活跃 (ArmMain_Flags) ⇒ disarm。
 *   ⇒ 网口永远赢, RC 不会自动抢回; 要再动必须重新 `rc go on`。
 *
 * ⭐ P0-3 丢控: R9DS 丢控=继续发帧只改值 ⇒ 新鲜度判据结构性失效。
 *   判丢控唯一通道 = CH6 心跳值判据 (ARM_RC_HB_LOSS_EN=1 时启用;
 *   ⚠ 当前 CH6 是**不复位旋钮**, 用户暂不启用 ⇒ 默认 0)。
 *   判到丢控 = disarm = 停发流帧 = 舵机位置自锁冻结 (夹爪不会追 F/S)。
 *
 * 使能: 控制台 `rc go on` / `rc go off`。默认 off。上电后任何情况下
 *       RC 都不可能自己开始驱臂。
 * ============================================================ */
#ifndef __ARM_RC_CTRL_H
#define __ARM_RC_CTRL_H

#include <stdint.h>

/* 50Hz 拍调用 (main.c 的 ArmRc_Poll() 之后)。主臂构建下为空函数。 */
void ArmRcCtrl_Tick(void);

/* 武装/解除 (控制台 `rc go on|off`)。解除后停发流帧, 舵机保持自锁。 */
void ArmRcCtrl_SetArmed(uint8_t on);
uint8_t ArmRcCtrl_IsArmed(void);

/* 被外部运动强制解除 (cmdStreamExit / cmdStreamJoints 外部帧 / 轨迹活跃)。
 * why 仅用于控制台一次性提示; 未武装时为静默 no-op。 */
void ArmRcCtrl_Disarm(const char* why);

/* ---- 速度配置 (2026-09-24: 可设置、可存 flash, `rc spd` 指令) ----
 * 记录存扇区 11 区 D (CFG_OFF_RCSPD, 0x3000), 带 magic/版本/FNV 校验;
 * 首次使用时懒加载, 无有效记录 ⇒ 用默认值。
 * v[0]=X/Y 末端速度 mm/s, v[1]=Z mm/s, v[2]=J5 角速度 deg/s,
 * v[3]=夹爪跟随速率 deg/s。 */
void    ArmRcCtrl_SpdGet(float v[4]);
/* 设置运行时速度 (不落盘; 合法域外返回 0 且不修改):
 * vxy 1~120 mm/s, vz 1~80 mm/s, vj5 5~300 deg/s;
 * j6r 字段仅为 flash 区 D 记录兼容保留 —— ⭐0924E 起夹爪=滑杆绝对位置
 * 全速实时跟随, 不再限速。 */
uint8_t ArmRcCtrl_SpdSet(float vxy, float vz, float vj5, float j6r);
/* 落盘 (阻塞 ~1s, 只许控制台上下文调)。返回 1=成功。 */
uint8_t ArmRcCtrl_SpdSave(void);

#endif /* __ARM_RC_CTRL_H */
