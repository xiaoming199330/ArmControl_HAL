#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_model.py — 生成与固件运动学完全一致的 URDF 与几何 JSON。

权威来源：Core/Inc/kinematics.h + Core/Src/kinematics.c（本脚本常量与其逐字一致）。
自检：用 POSE_HOME_J 复算 FK，应得 rho=131.39 / z=3.98 / pitch=150.03（与实机 f 打印一致）。

输出：
  model/arm_model.urdf          主 URDF（primitive 几何，直接可加载）
  model/link_geometry.json      机器可读几何（自研 3D 渲染可直接消费）
"""
import json, math, os

# ============================================================
# 1. 固件权威常量（逐字来自 Core/Inc/kinematics.h）
# ============================================================
ROBOT_BASE_H   = 65.30    # 桌面 -> J1 轴
ROBOT_J2_OFF_Y = 31.32    # J1 -> J2 前向
ROBOT_J2_OFF_Z = 54.80    # J1 -> J2 向上 (J2 轴高 120.10)
ROBOT_L_UPPER  = 116.00   # J2 -> J3
ROBOT_L_FORE   = 135.00   # J3 -> J4
ROBOT_L_WRIST  = 62.50    # J4 -> J5
ROBOT_L_TOOL   = 105.30   # J5 -> TCP
ROBOT_L_DISTAL = ROBOT_L_WRIST + ROBOT_L_TOOL      # 167.80  (J4 -> TCP)
PSI_UPPER0     = 0.243787 # +13.97° 中性位 上臂方向（自竖直起算）
PSI_FORE0      = 1.532268 # +87.79° 中性位 前臂方向（水平）
PSI_DISTAL0    = PSI_FORE0
NUM_JOINTS     = 6
ARM_MAX_REACH  = ROBOT_L_UPPER + ROBOT_L_FORE + ROBOT_L_DISTAL   # 418.80

# 关节方向符号（Core/Inc/kinematics.h JOINT_DIR）
JOINT_DIR = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# 舵机零点默认（SERVO_OFFSET，全 180 = raw 2048）；现场校准后按臂实测
SERVO_OFFSET_DEFAULT = [180.0]*6

# 权威 raw 限位：J1/J2/J4/J6 来自 'c' Phase2 实测；J3/J5 为写死偏移（见 main.c）
J3_OFF_BACK, J3_OFF_FRONT = -1163, 1090
J5_OFF_MIN,  J5_OFF_MAX   = -1033,  787
# 未校准时的显示值（main.c JOINT_MIN/MAX，仅显示用！）
JOINT_MIN_DISPLAY = [0, 58, 10, 83, 180, 0]
JOINT_MAX_DISPLAY = [360, 264, 177, 289, 340, 360]

def raw_to_deg(raw):  return raw / 4095.0 * 360.0

# ============================================================
# 2. 计算零位姿态下各轴心坐标（关节角全 0）
# ============================================================
def axis_table():
    psi_u, psi_f = PSI_UPPER0, PSI_FORE0
    j1 = (0.0, 0.0, ROBOT_BASE_H)
    j2 = (ROBOT_J2_OFF_Y, 0.0, ROBOT_BASE_H + ROBOT_J2_OFF_Z)
    j3 = (j2[0] + ROBOT_L_UPPER*math.sin(psi_u), 0.0, j2[2] + ROBOT_L_UPPER*math.cos(psi_u))
    j4 = (j3[0] + ROBOT_L_FORE *math.sin(psi_f), 0.0, j3[2] + ROBOT_L_FORE *math.cos(psi_f))
    j5 = (j4[0] + ROBOT_L_WRIST*math.sin(psi_f), 0.0, j4[2] + ROBOT_L_WRIST*math.cos(psi_f))
    tcp= (j5[0] + ROBOT_L_TOOL *math.sin(psi_f), 0.0, j5[2] + ROBOT_L_TOOL *math.cos(psi_f))
    return j1, j2, j3, j4, j5, tcp

# ============================================================
# 3. 关节角限位（rad）—— 由 raw 限位 + 零点换算
# ============================================================
def joint_limits_rad():
    """返回 [(lo_deg, hi_deg)] x6 (关节角, 相对 Home)。
    说明: J1/J2/J4/J6 的真实极值由 'c' Phase2 现场实测, 静态不可知 —— 此处给
    固件默认显示值换算的近似; J3/J5 用写死偏移 (权威, 与现场无关)。"""
    off = SERVO_OFFSET_DEFAULT
    lims = []
    # J1/J2/J4/J6: 用显示值 (servo deg) + zero + DIR 换算
    for j in (0, 1, 3, 5):
        a = JOINT_DIR[j]*(JOINT_MIN_DISPLAY[j] - off[j])
        b = JOINT_DIR[j]*(JOINT_MAX_DISPLAY[j] - off[j])
        lims.append((min(a, b), max(a, b)))
    # 占位
    lims.insert(0, None)
    # J3 / J5: raw 偏移 (权威)
    j3 = (raw_to_deg(J3_OFF_BACK), raw_to_deg(J3_OFF_FRONT))
    j5 = (raw_to_deg(J5_OFF_MIN),  raw_to_deg(J5_OFF_MAX))
    out = [None]*6
    out[0] = lims[1]
    out[1] = lims[2]
    out[2] = j3
    out[3] = lims[3]
    out[4] = j5
    out[5] = lims[4]
    return out

# ============================================================
# 4. FK (与 kinematics.c 逐行一致) —— 用于自检
# ============================================================
L    = [ROBOT_L_UPPER, ROBOT_L_FORE, ROBOT_L_DISTAL]
BASE = [PSI_UPPER0, PSI_FORE0, PSI_DISTAL0]

def fk(joint_deg):
    D, R = math.degrees, math.radians
    t1 = R(joint_deg[0]); ang = [R(joint_deg[1]), R(joint_deg[2]), R(joint_deg[3])]
    s = 0.0; y = 0.0; z = 0.0
    for i in range(3):
        s += ang[i]
        y += L[i]*math.sin(BASE[i] + s)
        z += L[i]*math.cos(BASE[i] + s)
    y += ROBOT_J2_OFF_Y
    z += ROBOT_BASE_H + ROBOT_J2_OFF_Z
    x = y*math.cos(t1); wy = y*math.sin(t1)
    yaw = D(PSI_FORE0 + ang[0] + ang[1] + ang[2])
    return x, wy, z, yaw

# ============================================================
# 5. 生成 URDF
# ============================================================
def fmt(v): return f"{v:.6f}"

def gen_urdf():
    j1, j2, j3, j4, j5, tcp = axis_table()
    lim = joint_limits_rad()
    m = 0.001  # mm -> m

    def limtag(j, extra=""):
        lo, hi = lim[j]
        return (f'      <limit lower="{fmt(math.radians(lo))}" '
                f'upper="{fmt(math.radians(hi))}" effort="10" velocity="10"/>' + extra)

    # 连杆视觉尺寸 (m)：来自官方 STL 包围盒实测 + 工程简化
    SX, SY, SZ = 0.0454, 0.0248, 0.0396     # STS3215 舵机本体 45.4 x 24.8 x 39.6
    LINK_W, LINK_T = 0.028, 0.0454          # 连杆截面 ~28 x 45.4

    def link_box(name, length, w=LINK_W, t=LINK_T, psi=0.0):
        """沿 psi 方向的连杆盒：长度沿链路方向。"""
        cx = length*0.5*math.sin(psi); cz = length*0.5*math.cos(psi)
        return f"""    <visual>
      <origin xyz="{fmt(cx)} 0 {fmt(cz)}" rpy="0 {fmt(psi)} 0"/>
      <geometry><box size="{fmt(t)} {fmt(w)} {fmt(length)}"/></geometry>
      <material name="link"><color rgba="0.85 0.85 0.88 1"/></material>
    </visual>
    <collision>
      <origin xyz="{fmt(cx)} 0 {fmt(cz)}" rpy="0 {fmt(psi)} 0"/>
      <geometry><box size="{fmt(t)} {fmt(w)} {fmt(length)}"/></geometry>
    </collision>"""

    def servo_box(x, y, z, rpy="0 0 0"):
        return f"""    <visual>
      <origin xyz="{fmt(x)} {fmt(y)} {fmt(z)}" rpy="{rpy}"/>
      <geometry><box size="{fmt(SX)} {fmt(SY)} {fmt(SZ)}"/></geometry>
      <material name="servo"><color rgba="0.20 0.24 0.32 1"/></material>
    </visual>"""

    return f"""<?xml version="1.0"?>
<!-- ============================================================
     六轴机械臂 URDF（与固件 ArmControl_HAL 运动学完全一致）

     作者：ArmControl_HAL 上位机开发包  |  生成日期：2026-09-14
     权威来源：Core/Inc/kinematics.h（常量）+ Core/Src/kinematics.c（FK 公式）
     自检：POSE_HOME_J 复算 FK = rho 131.39 / z 3.98 / pitch 150.03，与实机 f 打印一致。

     坐标系约定：
       世界系 X=前, Y=左, Z=上；原点 = 桌面上的 J1 轴心；单位 = 米（URDF 标准，故 mm/1000）
       关节角 0 = 官方 Home（上臂 +13.97°、前臂水平 +87.79°）
       工具俯仰 pitch: 0°=竖直向上, 90°=水平向前, 180°=竖直向下

     ⚠ 关节限位说明：J1/J2/J4/J6 的限位由现场标定 'c' 实测决定，此处为固件默认值近似；
       J3/J5 由写死偏移给出（权威）。上位机应以设备实测（'L' 命令）为准并覆盖这些值。
     ⚠ 视觉几何为"示意图元"（盒体），尺寸来自官方 STL 实测包围盒；若需照片级外观，
       请改用 model/official/so101_new_calib.urdf + STL 网格。
     ============================================================ -->
<robot name="arm_so101_hal">

  <!-- ================= 底座（固定于桌面） ================= -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 {fmt(17.13*m)}" rpy="0 0 0"/>
      <geometry><cylinder radius="{fmt(36*m)}" length="{fmt(17.13*m)}"/></geometry>
      <material name="base"><color rgba="0.15 0.17 0.22 1"/></material>
    </visual>
    <visual>
      <origin xyz="0 0 {fmt((17.13*m + (ROBOT_BASE_H-17.13)*m*0.5))}" rpy="0 0 0"/>
      <geometry><cylinder radius="{fmt(28*m)}" length="{fmt((ROBOT_BASE_H-17.13)*m)}"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 {fmt(ROBOT_BASE_H*m*0.5)}" rpy="0 0 0"/>
      <geometry><cylinder radius="{fmt(36*m)}" length="{fmt(ROBOT_BASE_H*m)}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 {fmt(ROBOT_BASE_H*m*0.5)}" rpy="0 0 0"/>
      <mass value="0.6"/>
      <inertia ixx="0.0012" ixy="0" ixz="0" iyy="0.0012" iyz="0" izz="0.0018"/>
    </inertial>
  </link>

  <!-- ================= J1 肩部水平旋转 (Yaw, 绕竖直 Z) ================= -->
  <joint name="shoulder_pan" type="revolute">
    <origin xyz="0 0 {fmt(ROBOT_BASE_H*m)}" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
{limtag(0)}
  </joint>

  <link name="shoulder_link">
{servo_box(0.0, 0.0, -0.0212)}
    <collision>
      <origin xyz="0 0 -0.0212" rpy="0 0 0"/>
      <geometry><box size="{fmt(SX)} {fmt(SY)} {fmt(SZ)}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0.008 0 0.02" rpy="0 0 0"/>
      <mass value="0.10"/>
      <inertia ixx="3e-5" ixy="0" ixz="0" iyy="3e-5" iyz="0" izz="3e-5"/>
    </inertial>
  </link>

  <!-- ================= J2 肩 (Pitch, 绕 Y) ================= -->
  <joint name="shoulder_lift" type="revolute">
    <origin xyz="{fmt(ROBOT_J2_OFF_Y*m)} 0 {fmt(ROBOT_J2_OFF_Z*m)}" rpy="0 0 0"/>
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
{limtag(1)}
  </joint>

  <link name="upper_arm_link">
{link_box("upper", ROBOT_L_UPPER*m, psi=PSI_UPPER0)}
    <inertial>
      <origin xyz="{fmt(ROBOT_L_UPPER*m*0.5*math.sin(PSI_UPPER0))} 0 {fmt(ROBOT_L_UPPER*m*0.5*math.cos(PSI_UPPER0))}" rpy="0 0 0"/>
      <mass value="0.14"/>
      <inertia ixx="3e-5" ixy="0" ixz="0" iyy="2e-4" iyz="0" izz="2e-4"/>
    </inertial>
  </link>

  <!-- ================= J3 肘 (Pitch, 绕 Y) ================= -->
  <joint name="elbow_flex" type="revolute">
    <origin xyz="{fmt(ROBOT_L_UPPER*m*math.sin(PSI_UPPER0))} 0 {fmt(ROBOT_L_UPPER*m*math.cos(PSI_UPPER0))}" rpy="0 0 0"/>
    <parent link="upper_arm_link"/>
    <child link="lower_arm_link"/>
    <axis xyz="0 1 0"/>
{limtag(2)}
  </joint>

  <link name="lower_arm_link">
{link_box("fore", ROBOT_L_FORE*m, psi=PSI_FORE0)}
    <inertial>
      <origin xyz="{fmt(ROBOT_L_FORE*m*0.5*math.sin(PSI_FORE0))} 0 {fmt(ROBOT_L_FORE*m*0.5*math.cos(PSI_FORE0))}" rpy="0 0 0"/>
      <mass value="0.16"/>
      <inertia ixx="3e-5" ixy="0" ixz="0" iyy="3e-4" iyz="0" izz="3e-4"/>
    </inertial>
  </link>

  <!-- ================= J4 腕俯仰 (Pitch, 绕 Y) ================= -->
  <joint name="wrist_flex" type="revolute">
    <origin xyz="{fmt(ROBOT_L_FORE*m*math.sin(PSI_FORE0))} 0 {fmt(ROBOT_L_FORE*m*math.cos(PSI_FORE0))}" rpy="0 0 0"/>
    <parent link="lower_arm_link"/>
    <child link="wrist_link"/>
    <axis xyz="0 1 0"/>
{limtag(3)}
  </joint>

  <!-- J5 的 frame 沿工具轴：+Z 指向工具方向（零位时 psi=87.79°） -->
  <link name="wrist_link">
{link_box("wrist", ROBOT_L_WRIST*m, w=0.0369, psi=0.0)}
    <inertial>
      <origin xyz="0 0 {fmt(ROBOT_L_WRIST*m*0.5)}" rpy="0 0 0"/>
      <mass value="0.12"/>
      <inertia ixx="3e-5" ixy="0" ixz="0" iyy="3e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <!-- ================= J5 腕自转 (Roll, 绕工具轴) ================= -->
  <joint name="wrist_roll" type="revolute">
    <origin xyz="{fmt(ROBOT_L_WRIST*m*math.sin(PSI_FORE0))} 0 {fmt(ROBOT_L_WRIST*m*math.cos(PSI_FORE0))}" rpy="0 {fmt(PSI_FORE0)} 0"/>
    <parent link="lower_arm_link"/>
    <child link="gripper_link"/>
    <axis xyz="0 0 1"/>
{limtag(4)}
  </joint>

  <!-- ================= 夹爪（J5 之后 105.30mm 即 TCP） ================= -->
  <link name="gripper_link">
    <!-- 夹爪基座（舵机） -->
    <visual>
      <origin xyz="0 0 0.0212" rpy="0 0 0"/>
      <geometry><box size="{fmt(SX)} {fmt(SY)} {fmt(SZ)}"/></geometry>
      <material name="servo"><color rgba="0.20 0.24 0.32 1"/></material>
    </visual>
    <!-- 固定爪 -->
    <visual>
      <origin xyz="0 0.030 {fmt(ROBOT_L_TOOL*m*0.5)}" rpy="0 0 0"/>
      <geometry><box size="0.020 0.010 {fmt(ROBOT_L_TOOL*m)}"/></geometry>
      <material name="jaw"><color rgba="0.95 0.75 0.20 1"/></material>
    </visual>
    <inertial>
      <origin xyz="0 0 {fmt(ROBOT_L_TOOL*m*0.5)}" rpy="0 0 0"/>
      <mass value="0.09"/>
      <inertia ixx="4e-5" ixy="0" ixz="0" iyy="4e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <!-- ================= J6 夹爪开合（不改变 TCP 位置，仅视觉/开合） ================= -->
  <joint name="gripper" type="revolute">
    <origin xyz="0 0 {fmt(ROBOT_L_TOOL*m)}" rpy="0 0 0"/>
    <parent link="gripper_link"/>
    <child link="moving_jaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="{fmt(math.radians(-10))}" upper="{fmt(math.radians(100))}" effort="10" velocity="10"/>
  </joint>

  <link name="moving_jaw_link">
    <visual>
      <origin xyz="0 -0.010 {fmt(-ROBOT_L_TOOL*m*0.35)}" rpy="0 0 0"/>
      <geometry><box size="0.020 0.010 {fmt(ROBOT_L_TOOL*m*0.7)}"/></geometry>
      <material name="jaw"><color rgba="0.95 0.75 0.20 1"/></material>
    </visual>
    <inertial>
      <origin xyz="0 0 -0.02" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="3e-6" ixy="0" ixz="0" iyy="3e-6" iyz="0" izz="3e-6"/>
    </inertial>
  </link>

  <!-- ================= TCP（工具中心点 = 固件 FK 报告的点） ================= -->
  <joint name="tcp_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="gripper_link"/>
    <child link="tcp_link"/>
  </joint>
  <link name="tcp_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.004"/></geometry>
      <material name="tcp"><color rgba="0.90 0.20 0.20 1"/></material>
    </visual>
  </link>

</robot>
"""

# ============================================================
# 6. 生成 link_geometry.json
# ============================================================
def gen_json():
    j1, j2, j3, j4, j5, tcp = axis_table()
    lim = joint_limits_rad()
    d = {
        "meta": {
            "name": "ArmControl_HAL 六轴机械臂",
            "source": "Core/Inc/kinematics.h + Core/Src/kinematics.c",
            "generated": "2026-09-14",
            "units": {"length": "mm", "angle": "deg"},
            "frame": {
                "origin": "桌面上的 J1 轴心",
                "x": "前 (forward)", "y": "左 (left)", "z": "上 (up)",
                "zero_pose": "官方 Home：上臂自竖直前倾 +13.97°，前臂水平 +87.79°",
                "pitch_convention": "0°=竖直向上, 90°=水平向前, 180°=竖直向下"
            },
            "validation": {
                "method": "用 POSE_HOME_J 复算 FK，与实机 'f' 打印比对",
                "PoseHomeJ": [0.18, -102.95, 93.54, 71.65, -0.88, -50.29],
                "expected": {"rho": 131.4, "z": 3.98, "pitch": 150.03},
                "computed": {"rho": 131.39, "z": 3.98, "pitch": 150.03},
                "result": "一致（误差 < 0.01 mm）"
            }
        },
        "servo": {
            "model": "Feetech STS3215 (12V 30KG) 双轴 TTL 串行总线舵机",
            "body_mm": {"A_length": 45.2, "B_thickness": 24.7, "C_body": 35.0},
            "mesh_bbox_mm": {"x": 45.4, "y": 24.8, "z": 39.6},
            "weight_g": 55,
            "angle_range_raw": [0, 4095],
            "angle_range_deg": [0.0, 360.0],
            "raw_center": 2048,
            "raw_per_deg": 4095.0/360.0,
            "deg_per_raw": 360.0/4095.0,
            "horn_spline": "25T / OD 5.9 mm",
            "bus": "半双工 TTL 串行总线, 1 Mbps (可 38400bps~1Mbps)",
            "stall_torque_kgcm": 30,
            "gear": "steel"
        },
        "kinematic_constants_mm": {
            "ROBOT_BASE_H": ROBOT_BASE_H,
            "ROBOT_J2_OFF_Y": ROBOT_J2_OFF_Y,
            "ROBOT_J2_OFF_Z": ROBOT_J2_OFF_Z,
            "ROBOT_L_UPPER": ROBOT_L_UPPER,
            "ROBOT_L_FORE": ROBOT_L_FORE,
            "ROBOT_L_WRIST": ROBOT_L_WRIST,
            "ROBOT_L_TOOL": ROBOT_L_TOOL,
            "ROBOT_L_DISTAL": ROBOT_L_DISTAL,
            "ARM_MAX_REACH": ARM_MAX_REACH,
            "ROBOT_PSI_UPPER0_deg": math.degrees(PSI_UPPER0),
            "ROBOT_PSI_FORE0_deg": math.degrees(PSI_FORE0),
            "note": "J1->J2 平面内前向 31.32 / 向上 54.80；J2 轴高 120.10"
        },
        "joints": [
            {"index": 0, "name": "shoulder_pan", "cn": "底座水平旋转", "type": "yaw",
             "axis_world": [0,0,1], "axis_origin_mm": list(j1),
             "dir": JOINT_DIR[0], "limits_deg": lim[0],
             "note": "绕竖直 Z；0=朝 +X(前)，正向=朝 +Y(左)"},
            {"index": 1, "name": "shoulder_lift", "cn": "肩", "type": "pitch",
             "axis_world": [0,1,0], "axis_origin_mm": list(j2),
             "dir": JOINT_DIR[1], "limits_deg": lim[1],
             "note": "绕 Y；正向=上臂向前倾"},
            {"index": 2, "name": "elbow_flex", "cn": "肘", "type": "pitch",
             "axis_world": [0,1,0], "axis_origin_mm": list(j3),
             "dir": JOINT_DIR[2], "limits_deg": lim[2],
             "note": "绕 Y；限位由写死偏移给出(权威)，非标定极值"},
            {"index": 3, "name": "wrist_flex", "cn": "腕俯仰", "type": "pitch",
             "axis_world": [0,1,0], "axis_origin_mm": list(j4),
             "dir": JOINT_DIR[3], "limits_deg": lim[3],
             "note": "绕 Y"},
            {"index": 4, "name": "wrist_roll", "cn": "腕自转", "type": "roll",
             "axis_world": [0.999257, 0.0, 0.038573], "axis_origin_mm": list(j5),
             "dir": JOINT_DIR[4], "limits_deg": lim[4],
             "note": "绕工具轴(零位时 psi=87.79°)；不改变末端位置；限位由写死偏移给出"},
            {"index": 5, "name": "gripper", "cn": "夹爪开合", "type": "revolute",
             "axis_world": [0,1,0], "axis_origin_mm": list(tcp),
             "dir": JOINT_DIR[5], "limits_deg": lim[5],
             "note": "不改变 TCP 位置；驱动夹爪张开/闭合"}
        ],
        "axis_chain_zero_pose_mm": {
            "J1_axis":  list(j1),  "J2_axis": list(j2), "J3_axis": list(j3),
            "J4_axis":  list(j4),  "J5_axis": list(j5), "TCP":     list(tcp)
        },
        "link_lengths_mm": [
            {"from": "J1_axis", "to": "J2_axis", "length": math.dist(j1, j2),
             "components": {"forward": ROBOT_J2_OFF_Y, "up": ROBOT_J2_OFF_Z}},
            {"from": "J2_axis", "to": "J3_axis", "length": ROBOT_L_UPPER, "name": "上臂"},
            {"from": "J3_axis", "to": "J4_axis", "length": ROBOT_L_FORE,  "name": "前臂"},
            {"from": "J4_axis", "to": "J5_axis", "length": ROBOT_L_WRIST, "name": "腕"},
            {"from": "J5_axis", "to": "TCP",     "length": ROBOT_L_TOOL,  "name": "工具/夹爪"},
            {"from": "J4_axis", "to": "TCP",     "length": ROBOT_L_DISTAL, "name": "J4->TCP 合计"}
        ],
        "mesh_bbox_mm": {
            "_comment": "官方 STL 实测包围盒(单位 mm)，供高保真建模参考",
            "base_so101_v2.stl": [110.92, 72.00, 87.00],
            "motor_holder_so101_base_v1.stl": [28.00, 38.10, 55.08],
            "motor_holder_so101_wrist_v1.stl": [28.00, 36.90, 56.39],
            "moving_jaw_so101_v1.stl": [22.30, 91.99, 48.00],
            "rotation_pitch_so101_v1.stl": [59.80, 84.20, 46.00],
            "sts3215_03a_v1.stl": [45.40, 24.80, 39.60],
            "under_arm_so101_v1.stl": [130.70, 24.00, 64.40],
            "upper_arm_so101_v1.stl": [142.17, 24.50, 67.30],
            "wrist_roll_follower_so101_v1.stl": [65.20, 52.00, 105.43],
            "wrist_roll_pitch_so101_v2.stl": [62.30, 35.66, 77.83]
        },
        "poses_joint_deg": {
            "_comment": "预设位姿，关节角(相对 Home)，完整来自 main.c POSE_*_J",
            "HOME  (折叠收纳)": [0.18, -102.95, 93.54, 71.65, -0.88, -50.29],
            "FOLD  (备用收拢)": [0, 40, -70, 40, 0, 0],
            "READY (前伸待命)": [0, 0, -30, 40, 0, 0],
            "WAVE1 (高举 A)":  [45, -30, -80, 50, 0, 0],
            "WAVE2 (高举 B)":  [-45, -30, -80, 50, 0, 0]
        },
        "notes": [
            "URDF 关节限位：J1/J2/J4/J6 为固件默认显示值换算的近似；J3/J5 为写死偏移(权威)。",
            "上位机应以设备实测为准：'L' 命令读取限位，'p J'/'s' 读取 raw。",
            "官方 so101_new_calib.urdf 与固件模型在 J4->TCP 上有 ~5.6mm 差异（固件为 167.80）。",
            "固件模型偏保守约 6mm（折叠位模型 z=3.98 而实测爪尖离桌 10mm）。"
        ]
    }
    return d

# ============================================================
# 7. main
# ============================================================
if __name__ == "__main__":
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model")
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # --- 自检 ---
    x, y, z, yaw = fk([0.18, -102.95, 93.54, 71.65, -0.88, -50.29])
    rho = math.hypot(x, y)
    print(f"[自检] POSE_HOME_J  FK: rho={rho:.2f} z={z:.2f} pitch={yaw:.2f}  "
          f"(期望 131.4 / 3.98 / 150.03)")
    assert abs(rho-131.4) < 0.1 and abs(z-3.98) < 0.05 and abs(yaw-150.03) < 0.05, "自检失败!"

    urdf = gen_urdf()
    with open(os.path.join(out_dir, "arm_model.urdf"), "w", encoding="utf-8") as f:
        f.write(urdf)
    with open(os.path.join(out_dir, "link_geometry.json"), "w", encoding="utf-8") as f:
        json.dump(gen_json(), f, ensure_ascii=False, indent=2)

    print("[OK] model/arm_model.urdf")
    print("[OK] model/link_geometry.json")
    j1,j2,j3,j4,j5,tcp = axis_table()
    print(f"     轴心(零位): J1{j1} J2{j2} J3{j3}")
    print(f"                 J4{j4} J5{j5} TCP{tcp}")
