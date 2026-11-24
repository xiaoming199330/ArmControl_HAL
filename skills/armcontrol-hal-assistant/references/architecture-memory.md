# ArmControl_HAL 架构记忆（Architecture Memory）

> 维护协议：结构变化时刷新；新增内容标注版本（如 `v2.1+`）。
> 深扫：`grep -n "^\(static \)\?\(void\|int\|float\|uint\|HAL_StatusTypeDef\)" Core/Src/*.c` 提取函数表。

## 0. 硬件与构建

```yaml
mcu: STM32F407ZGT6 (1MB Flash, 168MHz, HSE 8MHz + PLL 8/336/2/7)
servos: STS3215 (SCS) x6, Protocol 2.0, 1Mbps 半双工 TTL
buses:
  BUS_A: USART3 (PB10=TX, PB11=RX), DIR=PE8 (74HC126), 关节 J1-J3, ID=1,2,3
  BUS_B: UART5  (PC12=TX, PD2=RX), DIR=PE10 (74HC126), 关节 J4-J6, ID=4,5,6
console: USART6 (PC6=TX, PC7=RX, AF8) @115200, RXNE 中断 + 128B 环形缓冲
gpio: { E-Stop: PE11 低有效, 限位: PG0-PG5 低有效, LED: PC5红/PC4绿/PA5蓝, 蜂鸣器: PB5, SWD: PA13/PA14 不可占用 }
build: Makefile (arm-none-eabi-gcc, -Og, DEBUG=1) → build/ArmControl_HAL.{elf,hex,bin}
flash_layout: 代码限前 896KB；Sector 11 (0x080E0000~0x080FFFFF) = PID/补偿配置持久区
```

## 1. 分层与文件职责

```yaml
业务层:
  Core/Src/main.c (2106 行):
    控制台: [Console_Init, USART6_IRQHandler, Console_TryReadLine, cprint, ProcessCommand]
    运动调度: [cmdMove, cmdPose, cmdPointLoopStart, cmdMultiPointStart/Advance/Service, cmdGoto, cmdSolveIKPoint]
    反馈: [cmdFK, readActualEndPose, cmdRealtimeFeedback, cmdRealtimePoseFeedback, cmdStatus]
    调参: [set_pid_gain, pid_load_startup, pid_save_current, pid_reset_defaults, gravity_comp_j2]
    安全: [IsEStop, IsLimit, clamp_angle, clamp_all_angles, cmdLimits]
    状态组: [traj_tick/traj_do_update, swing_*, point_loop_*, multi_point_*, correction_*/corr_*, feedback/pose_feedback_*]
    主循环: while(1) { 200Hz轨迹更新 → cmdMultiPointService → 自适应修正 → 周期反馈 → E-Stop → 命令处理 }
    ISR: HAL_TIM_PeriodElapsedCallback (TIM2 2kHz: 心跳LED + 10分频置 traj_do_update)

规划层:
  Core/Src/trajectory.c (213 行): [Trajectory_Plan, Trajectory_Interpolate, Trajectory_IsActive, Trajectory_Stop, s_acc(static)]
    七段 S 曲线; SCURVE_TJ=0.15s; 三档距离自适应(七段/五段/三段); 固定 200Hz; 减速段=加速段镜像

算法层:
  Core/Src/kinematics.c (320 行): [FK_Solve, IK_Solve, ccd_run(static), IK_SetJointLimits, Kinematics_SetOffset,
    servo_deg_to_rad, rad_to_servo_deg, deg_to_sts3215_raw, sts3215_raw_to_deg]
    结构: J1=Yaw(绕Z), J2~J6=Pitch(竖直平面); a6=0 仅姿态
    IK: θ1=atan2(y,x) 解析 → (r,h) 平面 5 起点 CCD(θ2~θ5, 逐关节限位钳制) → θ6=pitch-Σθ2..5

关节抽象层:
  Core/Src/feetech_sts3215.c (297 行): [STS3215_InitAll/InitJoint, STS3215_SetTorque, STS3215_SetPosition(+Fast),
    STS3215_SetSpeed, STS3215_GetPosition(+Raw), STS3215_IsMoving, STS3215_GetPresentLoad, STS3215_GetTemperature, STS3215_Home,
    STS3215_SetInitPID/SetInitSpeed, AngleToRaw/RawToAngle(static)]
    映射: JOINT_ID[6]={1..6}, JOINT_BUS[6]={A,A,A,B,B,B}

协议层:
  Core/Src/feetech_protocol.c (265 行): [DXL_BuildPacket, DXL_ParsePacket, DXL_UpdateCRC, DXL_Ping, DXL_Read,
    DXL_Write, DXL_WriteNoReply, DXL_ScanBus]
    CRC-16/IBM 0x8005 逐位; 状态包 INST=0x55; Alert bit(0x80) 不视为错误

硬件层:
  Core/Src/feetech_hal.c (105 行): [DXL_SetDirection, DXL_FlushErrors, DXL_Transmit, DXL_Receive, DXL_TransmitReceive]
    关键: TX 前 CLEAR_BIT(RE) 禁接收防 74HC126 回显污染; TC 等待 + 忙等延时后切回 RX 再 SET_BIT(RE)

持久层:
  Core/Src/pid_flash.c (122 行): [PID_FlashLoad/Save/Erase, pid_crc(static CRC32)]
    PID_Record_t: magic=0x50494431, version=3, p/i/d[6], corr_pct[6], moving_speed, threshold_cdeg, max_rounds, enabled, crc
```

## 2. 核心数据流（crossLayer）

### 2.1 协调运动链（m/g/P/B/QL/W 共用）

```
命令行 → ProcessCommand('m') → cmdMove(target, v, a=v*2)
  → [安全] IsEStop + clamp_all_angles(JOINT_MIN/MAX, 打印钳制警告)
  → STS3215_GetPosition x6 读起点(失败取0) → STS3215_SetTorque x6 ON
  → Trajectory_Plan(start, target, v_max, accel)  [以最大位移关节为基准, 六关节比例同步]
  → TIM2 2kHz/10 → traj_do_update → 主循环 Trajectory_Interpolate(traj_tick, angles)
  → angles[1] += gravity_comp_j2(angles)  [J2 重力前馈: -K×水平臂展, 再钳限位]
  → STS3215_SetPositionFast x6 (DXL_WriteNoReply, 不等应答)
  → 轨迹结束 → 分支: 多点循环 cmdMultiPointService | AB循环 point_loop | 摇摆 swing | 否则触发自适应修正
```

### 2.2 自适应修正状态机（轨迹结束后）

```
correction_pending=1 + 500ms 延时 → 每轮对 CORR_FIRST_JOINT..CORR_LAST_JOINT (main.c 宏 = 0..5, 即 J1~J6 全部):
  actual=STS3215_GetPosition → error=target-actual
  |error|<=corr_threshold(2°) → OK 跳过
  sign_flip(过冲) → gain×0.5, osc++; osc>=2 → D+4(≤32) 写舵机
  同向有改善     → gain×1.25(≤0.85)
  停滞           → gain×1.4, stall++; stall>=2 → P+8(≤64); stall>=3 → I+2(≤16)
  corrected=target+error×gain → 钳限位 → STS3215_SetPosition
  全部 OK → DONE; correction_count>=corr_max_rounds(10) → TIMEOUT/STOP
```

### 2.3 IK 移动链（g/qg/BG）

```
g x y z [pitch] → cmdGoto → IK_Solve (WORLD_OFFSET 逆变换 → θ1 → 5起点CCD → θ6)
  → rad_to_servo_deg → 全关节限位检查(超界取消并打印)
  → FK_Solve 验证, err>5mm 取消 → cmdMove(deg, 30, 60)
```

### 2.4 配置持久化链

```
K/I/D/N/V/A 命令改 RAM 缓存(pid_p/i/d, corr_gain_init, all_servo_moving_speed) → 写 STS3215 RAM
F → pid_save_current → PID_FlashSave (擦除 Sector 11 + 写记录 + 回读校验)
启动 → pid_load_startup → PID_FlashLoad(magic/version/length/CRC + 范围校验) → STS3215_SetInitPID/SetInitSpeed → STS3215_InitAll
X → 仅恢复固件默认 PID 到 RAM+舵机，不擦 Flash
```

### 2.5 串口接收链

```
USART6 RXNE ISR → rx_ring[128] (满则丢字节, 清 ORE) → Console_TryReadLine (回显/退格/组行)
  → ProcessCommand: strtok 分词 ≤16 tokens, switch(tokens[0][0]); 双字母命令看 tokens[0][1] (pv/pf/qv/qd/qg/QL/BG)
```

## 3. 关键约定（改动必须对齐）

| 约定 | 规则 | 位置 |
|------|------|------|
| 关节索引 | 运动命令(t/a/r/p/z/w/m/q) J=**0~5**；PID/自适应命令(K/I/D/N/A g) J=**1~6** | main.c ProcessCommand |
| 角度量纲 | 命令/限位=舵机度 0~300；运动学内部=弧度(0=中位, 偏移 SERVO_OFFSET=150)；舵机 raw=0~1023 (度/300×1023) | kinematics.c 转换函数 |
| 速度量纲 | 轨迹速度 deg/s (5~300)；STS3215 Moving Speed raw 0~2047 (V 命令)；两者独立 | trajectory.c / sts3215 |
| 限位同步 | 改 JOINT_MIN/MAX 后必须 `IK_SetJointLimits(JOINT_MIN, JOINT_MAX)` | cmdLimits, main() |
| 世界偏移 | actual = model + offset；IK 内先减 offset 再解算 | kinematics.c |
| Flash 布局 | 改 PID_Record_t 字段必须升 PID_FLASH_VERSION，否则旧记录被 CRC 拒绝回退默认 | pid_flash.c |
| 实时路径 | ISR 禁 cprint/舵机通信；200Hz 路径只用 SetPositionFast；日志注入需分频 | main.c, feetech_hal.c |
| 新命令 | 三处同步: ProcessCommand 分支 + cmdHelp 文本 + config/arm_config_readme.md 指令表 | main.c |

## 4. config-drift 清单（arm_config.h 聚合值 vs 代码实际生效值）

> `arm_config.h` 未被 main.c include（实际生效值在各 .c 内）。改参数前核对本表。

| 参数 | arm_config.h | 实际生效 | 生效位置 |
|------|-------------|---------|---------|
| 修正关节范围 | CORR_FIRST/LAST = 1/3 (J2~J4) | **0/5 (J1~J6 全部)** | main.c `#define CORR_FIRST_JOINT 0` |
| 回零速度 | STS3215_HOMING_SPEED=30 | **200** 写死 | feetech_sts3215.c STS3215_Home |
| 轨迹频率/分频 | 200Hz/10 | 相同，但硬编码 `*200.0f`、`>=10` | trajectory.c / main.c ISR |
| 收敛/阈值类 | CCD_CONVERGE_MM=0.5 等 | ccd_run 内字面量 0.5f/100 次/1.0f | kinematics.c |
| 机械尺寸/零位/位姿 | 与 kinematics.h、main.c 双重定义一致 | 编译以 .c/.h 内定义为准 | kinematics.h, main.c POSE_* |
| 重力前馈典型值 | 注释 0.02~0.05 | 运行时 G 命令设置 | main.c GRAVITY_COMP_K |

## 5. 已知坑与脆弱点（评审必查）

1. **`S` 停止分支显示 bug**：`Trajectory_Stop()` 后才判 `Trajectory_IsActive()`，永远打印 "Stopped." 而非 "Trajectory stopped."（main.c case 'S'）。
2. **多点循环读取失败处理**：`cmdMultiPointAdvance` 中 `STS3215_GetPosition` 失败用目标值顶替，可能掩盖总线故障。
3. **FK yaw 字段语义**：`Kinematics_Pos.yaw` 实际是**末端 pitch**（rad），命名误导。
4. **θ6 无 pitch 可行性检查**：IK 只钳 θ6 到限位，pitch 不可达时不报错，仅 FK 验证兜底。
5. **环形缓冲溢出静默丢字节**：console 命令过长 (>127B) 被截断且无提示。
6. **cprint 阻塞**：TX 100ms 超时阻塞发送，200Hz 路径误用会破坏实时性。
7. **E-Stop 下仅放行 s/h**：新增调试命令时注意被拦截。
8. **STS3215_Home 目标 0°**：raw 0 = 舵机 0°，与运动学零位 150° 不同；调用前确认机械安全。
9. **pid_load_startup 范围校验**：threshold 50~1000(0.5~10°)、rounds 1~20、speed≤2047 任一越界整包回退默认。
10. **W 摇摆结束固定回 READY 位姿**，不是起始位姿。

## 6. 架构对齐检查清单（新功能开发前逐项过）

- [ ] 属于哪一层？是否跨层调用（业务层不得直接碰 UART 寄存器）？
- [ ] 上游调用点（同一函数的全部调用者）是否都需要同步改？
- [ ] 关节索引/角度/速度量纲是否符合第 3 节约定？
- [ ] 是否触碰实时路径（ISR/200Hz）？加了阻塞调用？
- [ ] 双总线（J1~J3=A, J4~J6=B）是否都覆盖？
- [ ] 持久化字段变化？PID_FLASH_VERSION 升级？
- [ ] 新命令三处同步（解析/help/readme）？
- [ ] arm_config.h/.json/readme 三件套是否需要同步（文档性）？
