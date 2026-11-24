# -*- coding: utf-8 -*-
"""
arm_fk.py — 六轴机械臂 FK/IK 参考实现（与 STM32 固件逐位一致）

来源：Core/Inc/kinematics.h + Core/Src/kinematics.c（2026-09 版本）
已对拍验证（见文件末尾 __main__ 自测）：
    POSE_HOME_J = [0.18, -102.95, 93.54, 71.65, -0.88, -50.29]
      -> rho = 131.39 mm, z = 3.98 mm, pitch = 150.03 deg
      与固件实机 'f' 命令打印 (131, 0, 3, 150) 完全一致
    零位（关节角全 0）TCP = (361.895, 0.0, 244.333) mm

⚠ 本文件的常量与公式**不要改**。改了就和固件不一致，3D 模型会对不上实机。

术语
----
- 关节角(joint angle)  : 运动学角，单位 **弧度(本模块)**，0 = 官方 Home 姿态。
                          可正可负。固件的文本命令里用 **度**，本模块统一用弧度，
                          换算函数见 deg2rad/rad2deg。
- 舵机度(servo degree): 舵机绝对角度 0~360°，含 SERVO_OFFSET 零点偏移。
- raw                 : STS3215 编码器原始值 0~4095。

坐标系（世界系）
--------------
    X = 前      Y = 左      Z = 上
    原点 = 桌面上的 J1 轴心
    单位 = mm
    工具俯仰 pitch: 0° = 竖直向上, 90° = 水平向前, 180° = 竖直向下
                    （中性位工具水平 => pitch ≈ 87.79°）
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# ============================================================================
# 1. 机械几何常量（mm / rad）—— 权威值，来自 Core/Inc/kinematics.h
# ============================================================================

ROBOT_BASE_H = 65.30      # 桌面 -> J1 轴高度
ROBOT_J2_OFF_Y = 31.32    # J1 轴 -> J2 轴，臂平面内前向偏移
ROBOT_J2_OFF_Z = 54.80    # J1 轴 -> J2 轴，向上偏移（J2 轴高 120.10）

ROBOT_L_UPPER = 116.00    # J2->J3 轴心距（上臂）
ROBOT_L_FORE = 135.00     # J3->J4 轴心距（前臂）
ROBOT_L_WRIST = 62.50     # J4->J5 轴心距
ROBOT_L_TOOL = 105.30     # J5 轴(夹爪轴) -> 夹爪末端 TCP
ROBOT_L_DISTAL = ROBOT_L_WRIST + ROBOT_L_TOOL      # = 167.80  J4->TCP

# 中性位（关节角=0）连杆方向角：从竖直向上起算，顺时针为正（向前倾）
ROBOT_PSI_UPPER0 = 0.243787    # atan2(28.0, 112.57) = +13.97 deg
ROBOT_PSI_FORE0 = 1.532268     # atan2(134.9, 5.2)   = +87.79 deg（水平）
ROBOT_PSI_DISTAL0 = ROBOT_PSI_FORE0   # 工具沿前臂轴线

NUM_JOINTS = 6
ARM_MAX_REACH = ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL   # 418.80 mm

SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 360.0

# 平面臂 3 段（J2 起）
KIN_LEN = (ROBOT_L_UPPER, ROBOT_L_FORE, ROBOT_L_DISTAL)
KIN_BASE = (ROBOT_PSI_UPPER0, ROBOT_PSI_FORE0, ROBOT_PSI_DISTAL0)

# 每关节方向符号：+1 = 舵机度增加 -> 关节正向
JOINT_DIR = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# 关节零位（舵机度）：0 = 官方 Home 姿态。默认 180（raw 2048）。
# 现场校准后由 Kinematics_SetZeroRaw 重锚定，值随臂而异。
DEFAULT_SERVO_OFFSET = (180.0, 180.0, 180.0, 180.0, 180.0, 180.0)

# 官方 Home / 折叠收纳位（关节角，度）—— 来自 main.c POSE_HOME_J
POSE_HOME_J = (0.18, -102.95, 93.54, 71.65, -0.88, -50.29)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


# ============================================================================
# 2. 角度 / raw 换算
# ============================================================================

def deg2rad(d: float) -> float:
    return d * D2R


def rad2deg(r: float) -> float:
    return r * R2D


def servo_deg_to_rad(deg: float, joint: int, servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> float:
    """舵机度(0~360) -> 运动学弧度(0=中性位)。对应固件 servo_deg_to_rad()。"""
    return JOINT_DIR[joint] * (deg - servo_offset[joint]) * D2R


def rad_to_servo_deg(rad: float, joint: int, servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> float:
    """运动学弧度(0=中性位) -> 舵机度(0~360，已钳制)。对应固件 rad_to_servo_deg()。"""
    deg = servo_offset[joint] + JOINT_DIR[joint] * rad * R2D
    return min(max(deg, SERVO_MIN_DEG), SERVO_MAX_DEG)


def deg_to_servo_raw(deg: float) -> int:
    """舵机度(0~360) -> raw(0~4095)。对应固件 deg_to_servo_raw()（四舍五入）。"""
    deg = min(max(deg, 0.0), 360.0)
    return int(deg / 360.0 * 4095.0 + 0.5)


def servo_raw_to_deg(raw: int) -> float:
    """raw(0~4095) -> 舵机度(0~360)。对应固件 servo_raw_to_deg()。

    注意：固件用 f32 单精度。此处用双精度，差异 < 0.001°（显示级，可忽略）。
    """
    return float(raw) / 4095.0 * 360.0


def joint_deg_to_servo_deg(joint_deg: float, joint: int,
                           servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> float:
    """关节角(度) -> 舵机度(0~360，已钳制)。"""
    return rad_to_servo_deg(deg2rad(joint_deg), joint, servo_offset)


def servo_deg_to_joint_deg(servo_deg: float, joint: int,
                           servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> float:
    """舵机度(0~360) -> 关节角(度)。"""
    return rad2deg(servo_deg_to_rad(servo_deg, joint, servo_offset))


# ============================================================================
# 3. 正运动学 FK
# ============================================================================

@dataclass
class Pose:
    """末端位姿。x/y/z 单位 mm，pitch 单位 **弧度**（与固件一致）。

    pitch: 0=竖直向上, pi/2=水平向前, pi=竖直向下。
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0

    # --- 便捷属性（只读） ---
    @property
    def rho(self) -> float:
        """到 J1 轴的水平距离 sqrt(x^2+y^2)，mm。"""
        return math.hypot(self.x, self.y)

    @property
    def pitch_deg(self) -> float:
        return rad2deg(self.pitch)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.pitch)

    def __str__(self) -> str:
        return (f"Pose(x={self.x:8.3f}, y={self.y:8.3f}, z={self.z:8.3f}, "
                f"pitch={self.pitch_deg:7.2f} deg)")


def planar_fk(ang_rad: Sequence[float]) -> Tuple[float, float]:
    """平面臂 FK：ang = (t2,t3,t4) 弧度。返回相对 J2 轴的 (前向 y, 高度 z)。

    对应固件 planar_fk()。**逐行对应，不要简化。**
    """
    s = 0.0
    y = 0.0
    z = 0.0
    for i in range(3):
        s += ang_rad[i]
        y += KIN_LEN[i] * math.sin(KIN_BASE[i] + s)
        z += KIN_LEN[i] * math.cos(KIN_BASE[i] + s)
    return y, z


def fk(joints_rad: Sequence[float],
       world_offset: Sequence[float] = (0.0, 0.0, 0.0)) -> Pose:
    """正运动学。

    joints_rad: 6 个关节角（弧度）。[0]=J1 yaw, [1..3]=J2/J3/J4 pitch,
                [4]=J5 roll(不影响位置), [5]=J6 夹爪(不影响位置)
    world_offset: (dx,dy,dz) 世界系标定偏移 mm，对应固件 WORLD_OFFSET_*
    """
    t1 = joints_rad[0]
    ang = (joints_rad[1], joints_rad[2], joints_rad[3])

    py, pz = planar_fk(ang)

    # J2 轴在臂平面内的偏移：J1 轴 -> J2 轴
    py += ROBOT_J2_OFF_Y
    pz += ROBOT_BASE_H + ROBOT_J2_OFF_Z

    return Pose(
        x=py * math.cos(t1) + world_offset[0],
        y=py * math.sin(t1) + world_offset[1],
        z=pz + world_offset[2],
        # 工具轴方向 psi_C = PSI_FORE0 + t2 + t3 + t4（J5 自转不改变方向）
        pitch=ROBOT_PSI_FORE0 + joints_rad[1] + joints_rad[2] + joints_rad[3],
    )


def fk_from_joint_deg(joint_deg: Sequence[float],
                      servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET,
                      world_offset: Sequence[float] = (0.0, 0.0, 0.0)) -> Pose:
    """便捷版：输入关节角（**度**），返回 Pose。"""
    return fk([deg2rad(v) for v in joint_deg], world_offset)


def fk_from_servo_deg(servo_deg: Sequence[float],
                      servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET,
                      world_offset: Sequence[float] = (0.0, 0.0, 0.0)) -> Pose:
    """便捷版：输入舵机度（0~360），返回 Pose。"""
    rad = [servo_deg_to_rad(servo_deg[j], j, servo_offset) for j in range(6)]
    return fk(rad, world_offset)


# ============================================================================
# 4. 关节限位
# ============================================================================

class JointLimits:
    """关节限位（弧度，内部表示）。

    两种设置方式，对应固件两个函数：
      * set_from_deg()     <- IK_SetJointLimits()     角度制，不支持跨 0/360
      * set_from_raw()     <- IK_SetJointLimitsRaw()  raw 制，支持跨 0/360（推荐）

    ⚠ 默认构造是"全开放"(±150°)，**不是**安全限位。
      上位机必须先用 'L' 命令读回实机限位，或由用户显式设置，
      否则本地 IK 预校验会放行实机拒绝的点。
    """

    def __init__(self, min_rad: Optional[Sequence[float]] = None,
                 max_rad: Optional[Sequence[float]] = None):
        if min_rad is None:
            min_rad = [-150.0 * D2R] * 6
        if max_rad is None:
            max_rad = [150.0 * D2R] * 6
        self.min_rad: List[float] = list(min_rad)
        self.max_rad: List[float] = list(max_rad)

    # ---- 设置方式 1：角度制 ----
    def set_from_deg(self, min_deg: Sequence[float], max_deg: Sequence[float],
                     servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> None:
        """对应固件 IK_SetJointLimits()。

        ⚠ 与固件一致：DIR=-1 的关节（J1）换算后 min/max 会翻转，这里做归一化。
        """
        for i in range(6):
            a = JOINT_DIR[i] * (min_deg[i] - servo_offset[i]) * D2R
            b = JOINT_DIR[i] * (max_deg[i] - servo_offset[i]) * D2R
            self.min_rad[i] = min(a, b)
            self.max_rad[i] = max(a, b)

    # ---- 设置方式 2：raw 制（推荐） ----
    def set_from_raw(self, min_raw: Sequence[int], max_raw: Sequence[int],
                     servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET) -> None:
        """对应固件 IK_SetJointLimitsRaw()。

        沿 raw 正向弧连续展开，正确处理跨 0/360 的包裹弧（如 J3）。
        span == 0 的关节视为"未校准"，跳过（保持原值）。
        """
        for i in range(6):
            mn, mx = int(min_raw[i]), int(max_raw[i])
            span = (mx + 4096 - mn) % 4096          # raw_dist_fwd(mn, mx)
            if span == 0:
                continue
            off = servo_offset[i]
            span_deg = span * 360.0 / 4095.0
            deg_mn_servo = servo_raw_to_deg(mn)
            deg_mx_servo = deg_mn_servo + span_deg   # 连续，可 > 360
            deg_a = JOINT_DIR[i] * (deg_mn_servo - off)
            deg_b = JOINT_DIR[i] * (deg_mx_servo - off)
            self.min_rad[i] = min(deg_a, deg_b) * D2R
            self.max_rad[i] = max(deg_a, deg_b) * D2R

    # ---- 查询 ----
    def clamp(self, joints_rad: Sequence[float]) -> List[float]:
        return [min(max(joints_rad[j], self.min_rad[j]), self.max_rad[j]) for j in range(6)]

    def inside(self, joints_rad: Sequence[float], tol_rad: float = 0.0) -> bool:
        """6 轴是否都在限位内。tol_rad 可放宽（固件 IK 用 0 容差）。"""
        for j in range(6):
            if joints_rad[j] < self.min_rad[j] - tol_rad:
                return False
            if joints_rad[j] > self.max_rad[j] + tol_rad:
                return False
        return True

    def as_deg(self, servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET):
        """返回 ((min_deg...), (max_deg...)) 舵机度形式，便于 UI 显示。"""
        mn, mx = [], []
        for j in range(6):
            a = rad_to_servo_deg(self.min_rad[j], j, servo_offset)
            b = rad_to_servo_deg(self.max_rad[j], j, servo_offset)
            mn.append(min(a, b))
            mx.append(max(a, b))
        return tuple(mn), tuple(mx)

    def __repr__(self) -> str:
        mn, mx = self.as_deg()
        return ("JointLimits(deg min=" + ",".join(f"{v:.1f}" for v in mn) +
                " max=" + ",".join(f"{v:.1f}" for v in mx) + ")")


# ============================================================================
# 5. 逆运动学 IK
# ============================================================================

class IKResult:
    """IK 求解结果。ok=False 时 joints 仍可能被填充（CCD 最优解），但不要用。"""

    __slots__ = ("ok", "joints_rad", "err_mm", "method")

    def __init__(self, ok: bool, joints_rad: List[float], err_mm: float, method: str):
        self.ok = ok
        self.joints_rad = joints_rad
        self.err_mm = err_mm
        self.method = method        # "analytic" | "ccd" | "fail"

    def __bool__(self) -> bool:
        return self.ok

    def __repr__(self) -> str:
        return (f"IKResult(ok={self.ok}, method={self.method}, err={self.err_mm:.4f}mm, "
                f"joint_deg={[round(rad2deg(v),2) for v in self.joints_rad]})")


# 固件默认的 CCD 参数（可被运行时参数表 #1~#4 覆盖，对应 LP/SP）
KIN_CCD_CONVERGE_MM = 0.5
KIN_CCD_MAX_ITER = 100
KIN_IK_FAIL_MM = 1.0
KIN_FK_VERIFY_MM = 5.0


def _ik_analytic(ty: float, tz: float, tool_pitch: float, lim: JointLimits,
                 ik_fail_mm: float = KIN_IK_FAIL_MM) -> Optional[Tuple[float, float, float]]:
    """解析 2R 解（俯仰精确满足，肘上/肘下都试）。对应固件 ik_analytic()。

    ty/tz: 目标相对 J2 轴的平面坐标；tool_pitch: 工具轴方向角（弧度）
    """
    # J4 轴位置 = 目标 - L_DISTAL * u(tool_pitch)
    j4y = ty - ROBOT_L_DISTAL * math.sin(tool_pitch)
    j4z = tz - ROBOT_L_DISTAL * math.cos(tool_pitch)
    d = math.hypot(j4y, j4z)

    if d < 1e-3:
        return None
    if d > ROBOT_L_UPPER + ROBOT_L_FORE + 0.01:
        return None
    if d < abs(ROBOT_L_UPPER - ROBOT_L_FORE):
        return None

    cosb = (ROBOT_L_UPPER ** 2 + d ** 2 - ROBOT_L_FORE ** 2) / (2.0 * ROBOT_L_UPPER * d)
    cosb = min(max(cosb, -1.0), 1.0)
    beta = math.acos(cosb)
    gamma = math.atan2(j4y, j4z)        # J2 -> J4 方向的 psi 角

    found = None
    best_err = float("inf")

    for side in (0, 1):
        psa = gamma + (-beta if side else beta)     # 上臂方向
        j3y = ROBOT_L_UPPER * math.sin(psa)
        j3z = ROBOT_L_UPPER * math.cos(psa)
        psb = math.atan2(j4y - j3y, j4z - j3z)      # 前臂方向

        t2 = psa - ROBOT_PSI_UPPER0
        t3 = (psb - ROBOT_PSI_FORE0) - t2
        t4 = tool_pitch - ROBOT_PSI_FORE0 - (t2 + t3)

        # 限位检查（J2/J3/J4 -> index 1..3）
        if not (lim.min_rad[1] <= t2 <= lim.max_rad[1]):
            continue
        if not (lim.min_rad[2] <= t3 <= lim.max_rad[2]):
            continue
        if not (lim.min_rad[3] <= t4 <= lim.max_rad[3]):
            continue

        py, pz = planar_fk((t2, t3, t4))
        err = math.hypot(py - ty, pz - tz)
        if err > ik_fail_mm:
            continue

        if err < best_err:
            best_err = err
            found = (t2, t3, t4)

    return found


def _ccd_run(ang: List[float], ty: float, tz: float, lim: JointLimits,
             converge_mm: float, max_iter: int) -> float:
    """平面 3 关节 CCD。返回最终位置误差 mm。对应固件 ccd_run()。"""
    for _ in range(max_iter):
        ey, ez = planar_fk(ang)
        dy, dz = ty - ey, tz - ez
        if math.hypot(dy, dz) < converge_mm:
            break

        for j in (2, 1, 0):
            # 关节 j 轴心 = 前 j 条连杆末端
            qy = qz = 0.0
            s = 0.0
            for i in range(j):
                s += ang[i]
                qy += KIN_LEN[i] * math.sin(KIN_BASE[i] + s)
                qz += KIN_LEN[i] * math.cos(KIN_BASE[i] + s)

            ey2, ez2 = planar_fk(ang)

            ver, vez = ey2 - qy, ez2 - qz
            vtr, vtz = ty - qy, tz - qz

            ve_len = math.hypot(ver, vez)
            vt_len = math.hypot(vtr, vtz)
            if ve_len < 0.001 or vt_len < 0.001:
                continue

            dot = (ver * vtr + vez * vtz) / (ve_len * vt_len)
            cross = (ver * vtz - vez * vtr) / (ve_len * vt_len)
            dot = min(max(dot, -1.0), 1.0)

            delta = math.atan2(cross, dot)
            ang[j] -= delta     # u(psi)=(sin,cos) 约定：-delta 转向目标

            ang[j] = min(max(ang[j], lim.min_rad[j + 1]), lim.max_rad[j + 1])

    ey, ez = planar_fk(ang)
    return math.hypot(ty - ey, tz - ez)


def ik(target: Pose, lim: Optional[JointLimits] = None,
       world_offset: Sequence[float] = (0.0, 0.0, 0.0),
       ik_fail_mm: float = KIN_IK_FAIL_MM,
       converge_mm: float = KIN_CCD_CONVERGE_MM,
       max_iter: int = KIN_CCD_MAX_ITER,
       keep_j5j6: Sequence[float] = (0.0, 0.0)) -> IKResult:
    """逆运动学。对应固件 IK_Solve()。

    参数
    ----
    target        : Pose(x,y,z,pitch)，pitch 为弧度（工具俯仰）
    lim           : JointLimits。None = 全开放（**危险**，见 JointLimits 说明）
    world_offset  : 世界系标定偏移 mm
    keep_j5j6     : J5/J6 输出值（固件输出 0；上位机建议传"当前关节角"以免腕部乱转）

    返回
    ----
    IKResult(ok, joints_rad, err_mm, method)
        method: "analytic" 解析解（俯仰精确） / "ccd" 兜底（俯仰自由，只保位置） / "fail"

    ⚠ 与固件一致：先解析解；解析解限位不可行才 CCD 多起点兜底。
      固件在 CCD 兜底时**放弃俯仰精度**，只保证位置 —— UI 上若要精确姿态，应
      在解析解失败时提示"该姿态不可达，已退化为位置优先"。

    ⚠⚠ **J1 方位角会被静默钳制**：theta1 = atan2(y,x) 若超出 J1 限位，固件
      `IK_Solve` 直接把它钳到限位边界并**仍然返回成功**（返回 1）。此时平面目标
      (rho) 没变，但方位角不对 => 臂指向错误方向。固件靠**调用方**（
      `cmdSolveIKPoint`）随后的 FK 位置回代校验（5mm）把这个错误解拦下来。
      所以：**不要单独用 ik() 的结果下发给实机**，用 ik_checked()。
    """
    if lim is None:
        lim = JointLimits()

    # 去标定偏移
    x = target.x - world_offset[0]
    y = target.y - world_offset[1]
    z = target.z - world_offset[2]
    tool_pitch = target.pitch

    joints = [0.0] * 6

    # --- Step 1: J1 方位角 ---
    theta1 = math.atan2(y, x)
    theta1 = min(max(theta1, lim.min_rad[0]), lim.max_rad[0])
    joints[0] = theta1

    # --- Step 2: 平面目标（相对 J2 轴） ---
    rho = math.hypot(x, y)
    ty = rho - ROBOT_J2_OFF_Y
    tz = z - (ROBOT_BASE_H + ROBOT_J2_OFF_Z)

    planar_dist = math.hypot(ty, tz)
    if planar_dist > ARM_MAX_REACH + 0.5:
        joints[4], joints[5] = keep_j5j6[0], keep_j5j6[1]
        return IKResult(False, joints, planar_dist - ARM_MAX_REACH, "fail")

    # --- Step 3a: 解析解（俯仰优先） ---
    ana = _ik_analytic(ty, tz, tool_pitch, lim, ik_fail_mm)
    if ana is not None:
        joints[1], joints[2], joints[3] = ana
        joints[4], joints[5] = keep_j5j6[0], keep_j5j6[1]
        # 固化件行为：解析解内部已做 FK 回代校验（<= ik_fail_mm），此处误差≈0
        return IKResult(True, joints, 0.0, "analytic")

    # --- Step 3b: CCD 多起点兜底（位置优先，俯仰自由） ---
    aim = math.atan2(ty, tz)
    fold = 0.0
    if 0.1 < planar_dist < ARM_MAX_REACH:
        ratio = min(planar_dist / ARM_MAX_REACH, 1.0)
        fold = math.acos(ratio) * 0.5

    starts = (
        (0.0, 0.0, 0.0),                      # 中性竖直
        (aim * 0.5, 0.0, 0.0),                # 朝目标倾
        (fold, fold, -fold),                  # S 形折叠
        (aim * 0.3, aim * 0.3, aim * 0.4),    # 均匀前倾
    )

    best_ang = [0.0, 0.0, 0.0]
    best_err = float("inf")

    for start in starts:
        a = [min(max(start[i], lim.min_rad[i + 1]), lim.max_rad[i + 1]) for i in range(3)]
        err = _ccd_run(a, ty, tz, lim, converge_mm, max_iter)
        if err < best_err:
            best_err = err
            best_ang = list(a)
        if err < converge_mm:
            break

    joints[1], joints[2], joints[3] = best_ang
    joints[4], joints[5] = keep_j5j6[0], keep_j5j6[1]

    if best_err > ik_fail_mm:
        return IKResult(False, joints, best_err, "fail")
    return IKResult(True, joints, best_err, "ccd")


def ik_xyz(x: float, y: float, z: float, pitch_deg: float = 0.0, **kw) -> IKResult:
    """便捷版：笛卡尔坐标(mm) + pitch(度) -> IKResult。"""
    return ik(Pose(x, y, z, deg2rad(pitch_deg)), **kw)


# ============================================================================
# 5b. raw 限位判定 + 固件的"接受/拒绝"闸门（cmdSolveIKPoint 复刻）
# ============================================================================

# 固件常量（main.c）
KIN_PITCH_TOL_DEG = 10.0     # 俯仰达成容差（cmdSolveIKPoint / cmdSolveIKPoint 后置校验）
PITCH_TOL_DEG = KIN_PITCH_TOL_DEG


def raw_dist_fwd(a: int, b: int) -> int:
    """raw 环上前进距离 (b - a) mod 4096。对应固件 raw_dist_fwd()。"""
    return (b + 4096 - a) % 4096


def raw_in_range(raw: int, mn: int, mx: int) -> bool:
    """raw 是否落在 [mn, mx]（沿正向弧，可跨 0/4096）。

    ⚠ 与固件一致：span == 0（未校准 / 单点）时**恒返回 False**
      => 未校准的关节会**拒绝一切运动**，不是"不限位"。上位机必须把
      "某轴 JOINT_MIN_RAW==JOINT_MAX_RAW==0" 显示成"该轴未校准，禁止运动"。
    """
    span = raw_dist_fwd(mn, mx)
    if span == 0:
        return False
    return raw_dist_fwd(mn, raw) <= span


def wrap_pitch_err(got_pitch_deg: float, want_pitch_deg: float) -> float:
    """俯仰角差，wrap 到 [-180,180)。对应固件 wrap_pitch_err()。"""
    return ((got_pitch_deg - want_pitch_deg + 540.0) % 360.0) - 180.0


class CartesianResult(IKResult):
    """笛卡尔目标在**固件里会不会被接受**的完整判定结果。

    这是上位机最该用的接口：它复刻固件 cmdSolveIKPoint() 的全部闸门，
    本地就能预测实机会不会拒绝，避免"发了没动"却不知道为什么。

    拒绝原因 reason 取值：
        "ok"             接受
        "unreachable"    位置超出 J2 平面臂展 / IK 不收敛
        "joint_limit"    解算结果超关节限位（角度制）
        "raw_limit"      转成 raw 后超实测 raw 限位（未校准轴 = 恒拒）
        "fk_verify"      FK 回代位置误差 > 5mm
        "pitch_not_met"  位置对但工具俯仰误差 > 10°（J4 限位导致的降级）
    """

    __slots__ = ("reason", "servo_deg", "raw", "pos_err_mm", "pitch_err_deg")

    def __init__(self, ok, joints_rad, err_mm, method, reason="ok",
                 servo_deg=None, raw=None, pos_err_mm=0.0, pitch_err_deg=0.0):
        super().__init__(ok, joints_rad, err_mm, method)
        self.reason = reason
        self.servo_deg = servo_deg or [0.0] * 6
        self.raw = raw or [0] * 6
        self.pos_err_mm = pos_err_mm
        self.pitch_err_deg = pitch_err_deg

    def __repr__(self) -> str:
        return (f"CartesianResult(ok={self.ok}, reason={self.reason}, method={self.method}, "
                f"pos_err={self.pos_err_mm:.3f}mm, pitch_err={self.pitch_err_deg:.2f}deg, "
                f"servo_deg={[round(v,2) for v in self.servo_deg]})")


def ik_checked(x: float, y: float, z: float, pitch_deg: float = 0.0,
               min_raw: Optional[Sequence[int]] = None,
               max_raw: Optional[Sequence[int]] = None,
               servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET,
               lim: Optional[JointLimits] = None,
               world_offset: Sequence[float] = (0.0, 0.0, 0.0),
               keep_j5j6_servo_deg: Optional[Sequence[float]] = None) -> CartesianResult:
    """完整复刻固件 cmdSolveIKPoint()：解 IK -> 转舵机度 -> raw 限位 -> FK 校验 -> 俯仰校验。

    上位机在 `g x y z pitch` 之前调用本函数，即可**提前**知道会不会被拒。

    参数
    ----
    min_raw / max_raw : 6 轴实测 raw 限位。**必须**从 `L` 命令或
                        GET_STATUS 之外的渠道读回真实值。传 None 则跳过 raw 检查
                        （会放松判定，实机可能仍拒绝）。
    servo_offset      : 当前零点（现场校准后 ≠ 180）。
    keep_j5j6_servo_deg : J5/J6 保持的舵机度。固件取"当前实际位置"；
                        传 None 表示 0（中性）。
    """
    if lim is None:
        lim = JointLimits()

    keep = (0.0, 0.0)
    if keep_j5j6_servo_deg is not None:
        keep = (servo_deg_to_rad(keep_j5j6_servo_deg[4], 4, servo_offset),
                servo_deg_to_rad(keep_j5j6_servo_deg[5], 5, servo_offset))

    r = ik(Pose(x, y, z, deg2rad(pitch_deg)), lim=lim, world_offset=world_offset,
           keep_j5j6=keep)
    if not r.ok:
        return CartesianResult(False, r.joints_rad, r.err_mm, r.method, "unreachable")

    joints = r.joints_rad
    servo = [rad_to_servo_deg(joints[j], j, servo_offset) for j in range(6)]
    raw = [deg_to_servo_raw(v) for v in servo]

    # --- raw 限位检查（固件：任一轴越限 -> return 0） ---
    if min_raw is not None and max_raw is not None:
        for j in range(6):
            if not raw_in_range(raw[j], int(min_raw[j]), int(max_raw[j])):
                return CartesianResult(False, joints, r.err_mm, r.method, "raw_limit",
                                       servo, raw)

    # --- FK 回代位置校验（容差 5mm） ---
    verify = fk(joints, world_offset)
    pos_err = math.dist((verify.x, verify.y, verify.z), (x, y, z))
    if pos_err > KIN_FK_VERIFY_MM:
        return CartesianResult(False, joints, r.err_mm, r.method, "fk_verify",
                               servo, raw, pos_err)

    # --- 俯仰校验（容差 10°；超差说明解析解降级到了 CCD，方向不对） ---
    got_pitch = rad2deg(verify.pitch) % 360.0
    perr = wrap_pitch_err(got_pitch, pitch_deg)
    if abs(perr) > KIN_PITCH_TOL_DEG:
        return CartesianResult(False, joints, r.err_mm, r.method, "pitch_not_met",
                               servo, raw, pos_err, perr)

    return CartesianResult(True, joints, r.err_mm, r.method, "ok", servo, raw, pos_err, perr)


# ============================================================================
# 6. 安全区检查（桌面 / 立柱）—— 与固件 fk_path_z_ok 的关键规则对齐
# ============================================================================

TRAJ_Z_MIN_MM = 2.0        # 兜底桌面下限
TRAJ_Z_MARGIN_MM = 3.0     # 折叠位模型 z 之下留的余量
HOME_DOCK_TOL_DEG = 3.0    # 折叠位白名单半径（6 轴需同时满足）
COLUMN_RADIUS_MM = 40.0    # ⚠ 立柱安全圆柱半径（**若与实机不符请以实机为准**）


def desk_floor_mm(servo_offset: Sequence[float] = DEFAULT_SERVO_OFFSET,
                  world_offset: Sequence[float] = (0.0, 0.0, 0.0)) -> float:
    """折叠位（POSE_HOME_J）爪尖模型 z - 3mm，作为桌面下限。

    对应固件 update_home_dock()。返回**基座坐标**（不含 WORLD_OFFSET_Z）。
    """
    p = fk_from_joint_deg(POSE_HOME_J, servo_offset, (0.0, 0.0, 0.0))
    return p.z - TRAJ_Z_MARGIN_MM


def is_home_dock(joint_deg: Sequence[float], tol_deg: float = HOME_DOCK_TOL_DEG) -> bool:
    """6 轴是否都在折叠位 ±tol 内（该姿态豁免 raw 限位与桌面 z 检查）。"""
    for j in range(6):
        d = (joint_deg[j] - POSE_HOME_J[j] + 180.0) % 360.0 - 180.0
        if abs(d) > tol_deg:
            return False
    return True


# ============================================================================
# 7. 自测（python arm_fk.py 直接运行）
# ============================================================================

def _selftest() -> int:
    fails = 0

    def check(name, got, want, tol, unit=""):
        nonlocal fails
        ok = abs(got - want) <= tol
        if not ok:
            fails += 1
        print(f"  [{'OK ' if ok else 'FAIL'}] {name:38s} got={got:9.4f} want={want:9.4f} "
              f"tol={tol} {unit}")

    print("=" * 78)
    print("arm_fk.py 自测 —— 与固件实机实测对拍")
    print("=" * 78)

    print("\n[1] 常量")
    check("ARM_MAX_REACH", ARM_MAX_REACH, 418.80, 1e-6, "mm")
    check("ROBOT_L_DISTAL", ROBOT_L_DISTAL, 167.80, 1e-6, "mm")
    check("PSI_UPPER0 -> deg", rad2deg(ROBOT_PSI_UPPER0), 13.97, 0.01, "deg")
    check("PSI_FORE0  -> deg", rad2deg(ROBOT_PSI_FORE0), 87.79, 0.01, "deg")

    print("\n[2] 零位（关节角全 0）")
    p = fk_from_joint_deg((0, 0, 0, 0, 0, 0))
    check("x", p.x, 361.895, 0.01, "mm")
    check("y", p.y, 0.0, 1e-9, "mm")
    check("z", p.z, 244.333, 0.01, "mm")
    check("pitch", p.pitch_deg, 87.79, 0.01, "deg")

    print("\n[3] POSE_HOME_J（折叠收纳位，实机 'f' 打印 131,0,3,150）")
    p = fk_from_joint_deg(POSE_HOME_J)
    check("rho", p.rho, 131.39, 0.02, "mm")
    check("z", p.z, 3.98, 0.02, "mm")
    check("pitch", p.pitch_deg, 150.03, 0.02, "deg")

    print("\n[4] 角度换算往返")
    for j in range(6):
        d = 200.0
        back = rad_to_servo_deg(servo_deg_to_rad(d, j), j)
        check(f"J{j+1} servo deg roundtrip", back, d, 1e-9, "deg")
    check("deg_to_servo_raw(180)", deg_to_servo_raw(180.0), 2048, 0, "raw")
    # ⚠ 固件固有量化artifact：4095 分辨率的"中点"是 2047.5，raw 2048 实际等于
    # 180.044°，不是 180.000°。SERVO_OFFSET 默认取 180.0，故"关节角=0"与
    # "raw=2048"之间有 ~0.044° 的固定偏差 —— 这是固件行为，不要在此处"修正"，
    # 否则模型与实机不一致。上位机只需知道：<0.05° 的差异是正常量化误差。
    check("servo_raw_to_deg(2048) 量化", servo_raw_to_deg(2048), 180.044, 0.001, "deg")

    print("\n[5] IK -> FK 往返（300 随机可达位姿）")
    import random
    random.seed(20260914)
    # 用真正"全开放"限位（±180°），且样本落在限位内 —— 专门测算法本身的
    # 往返精度（J1 一旦被钳制就必然对不上，那是限位行为，不是算法误差）
    BIG = math.pi
    lim_open = JointLimits([-BIG] * 6, [BIG] * 6)
    n_ana = n_ccd = n_fail = 0
    worst_ana = worst_ccd = 0.0
    for _ in range(300):
        jd = [random.uniform(-175, 175),          # J1（|·|<=180 才不被钳制）
              random.uniform(-160, 160),          # J2
              random.uniform(-160, 160),          # J3
              random.uniform(-160, 160),          # J4
              0.0, 0.0]
        p = fk_from_joint_deg(jd)
        r = ik(p, lim_open)
        if not r.ok:
            n_fail += 1
            continue
        p2 = fk(r.joints_rad)
        e = math.dist((p.x, p.y, p.z), (p2.x, p2.y, p2.z))
        if r.method == "analytic":
            n_ana += 1
            worst_ana = max(worst_ana, e)
        else:
            n_ccd += 1
            worst_ccd = max(worst_ccd, e)

    print(f"  解析解 {n_ana} / CCD兜底 {n_ccd} / 失败 {n_fail}")
    print(f"  解析解最差位置误差 {worst_ana:.6f} mm  (应 < 1e-6)")
    print(f"  CCD 最差位置误差   {worst_ccd:.6f} mm  (应 <= 1.0, 即 KIN_IK_FAIL_MM)")
    if n_fail != 0:
        fails += 1
        print("  [FAIL] 有可达位姿被判失败 —— 算法移植有误")
    elif worst_ana > 1e-6:
        fails += 1
        print("  [FAIL] 解析解往返误差过大 —— 公式移植有误")
    elif worst_ccd > KIN_IK_FAIL_MM:
        fails += 1
        print("  [FAIL] CCD 误差超过固件接受阈值")
    else:
        print("  [OK ] 解析解为数值精度级；CCD 与固件阈值一致")

    print("\n[6] raws 限位判定（含 '未校准即拒' 与跨 0 包裹）")
    print(f"  raw_in_range(2048, 0, 4095)      = {raw_in_range(2048, 0, 4095)}   (应 True)")
    print(f"  raw_in_range(2048, 0, 0)         = {raw_in_range(2048, 0, 0)}  (未校准 -> 应 False)")
    print(f"  raw_in_range(10, 4000, 200)      = {raw_in_range(10, 4000, 200)}   (跨0包裹 -> 应 True)")
    print(f"  raw_in_range(3000, 4000, 200)    = {raw_in_range(3000, 4000, 200)}  (跨0包裹 -> 应 False)")
    if not (raw_in_range(2048, 0, 4095) and not raw_in_range(2048, 0, 0)
            and raw_in_range(10, 4000, 200) and not raw_in_range(3000, 4000, 200)):
        fails += 1
        print("  [FAIL] raw_in_range 实现错误")

    print("\n[7] 固件闸门 ik_checked()（预测实机会不会接受）")
    # ⚠ 演示用限位。**真实限位必须从实机读**：
    #   - main.c 里的 JOINT_MIN/MAX 初值 {0,58,10,83,180,0}/{360,264,177,289,340,360}
    #     只是"开机显示占位"，会被校准结果覆盖，**不是**真实安全限位；
    #   - JOINT_MIN_RAW/MAX_RAW 开机全 0 = "未校准"，此时 raw_in_range 恒 False，
    #     实机会**拒绝一切运动**（这是固件刻意行为，不是 bug）。
    #   => 上位机必须用 'L'（或 GET_STATUS 的限位字段）读回真实值再判断。
    min_raw_demo = [1600, 0, 0, 0, 0, 0]     # J1 有意收窄，用于演示"方位角被钳制"
    max_raw_demo = [3400, 4095, 4095, 4095, 4095, 4095]
    # ⚠ 关键：固件用 IK_SetJointLimitsRaw(同一组 raw) 设置 IK 内部限位，
    #   所以本地也必须由同一组 raw 导出 JointLimits，否则"钳制落点"会不一致。
    lim_demo = JointLimits()
    lim_demo.set_from_raw(min_raw_demo, max_raw_demo)

    demo = [
        ("正前方 300mm 高 200mm,pitch90", 300.0, 0.0, 200.0, 90.0),
        ("超出臂展 900mm", 900.0, 0.0, 200.0, 90.0),
        ("正后方 (-300,0,200) 方位角越J1限位", -300.0, 0.0, 200.0, 90.0),
        ("贴桌面 (200,0,2)", 200.0, 0.0, 2.0, 90.0),
    ]
    expect = {"正前方 300mm 高 200mm,pitch90": True,
              "超出臂展 900mm": False,
              "正后方 (-300,0,200) 方位角越J1限位": False,
              "贴桌面 (200,0,2)": None}
    for name, x, y, z, pitch in demo:
        cr = ik_checked(x, y, z, pitch, min_raw=min_raw_demo, max_raw=max_raw_demo,
                        lim=lim_demo)
        print(f"  {name:34s} ok={str(cr.ok):5s} reason={cr.reason:14s} "
              f"pos_err={cr.pos_err_mm:7.2f} pitch_err={cr.pitch_err_deg:7.2f}")
        exp = expect.get(name)
        if exp is not None and cr.ok != exp:
            fails += 1
            print(f"       [FAIL] 期望 ok={exp}")

    # 关键演示：方位角被静默钳制 —— 必须靠 fk_verify 拦下
    cr = ik_checked(-300.0, 0.0, 200.0, 90.0, min_raw=min_raw_demo,
                    max_raw=max_raw_demo, lim=lim_demo)
    if cr.reason == "fk_verify":
        print("  [OK ] 方位角越限 -> 被 fk_verify 拦下（若只看 ik() 会以为成功！）")
    else:
        fails += 1
        print(f"  [FAIL] 期望 fk_verify，实得 {cr.reason}")
    r_raw = ik(Pose(-300.0, 0.0, 200.0, deg2rad(90.0)), lim_demo)
    print(f"       对照：裸 ik() 返回 ok={r_raw.ok}（固件 IK_Solve 也返回 1）—— 这就是陷阱")

    print("\n[8] 桌面下限 / 折叠位白名单")
    print(f"  desk_floor_mm() = {desk_floor_mm():.2f} mm  (≈ 3.98 - 3.0 = 0.98)")
    print(f"  is_home_dock(POSE_HOME_J) = {is_home_dock(POSE_HOME_J)}  (应为 True)")
    if not is_home_dock(POSE_HOME_J):
        fails += 1
        print("  [FAIL] 折叠位白名单判定错误")

    print("\n" + "=" * 78)
    if fails == 0:
        print("全部通过 ✅  —— 本模块与固件模型一致，可放心用于 3D 显示与 IK 预校验")
    else:
        print(f"有 {fails} 项失败 ❌ —— 请勿使用，先核对常量")
    print("=" * 78)
    return fails


if __name__ == "__main__":
    raise SystemExit(_selftest())
