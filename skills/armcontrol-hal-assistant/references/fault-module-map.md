# 故障-模块映射（Fault Module Map）

## 模块索引

| 模块 | 关键文件 | 典型故障 |
|------|---------|---------|
| console 命令层 | `main.c`: ProcessCommand, Console_TryReadLine, cmdHelp | 命令无响应、参数解析错、双字母命令混淆 |
| trajectory 规划层 | `trajectory.c` 全部; `main.c` ISR + 200Hz 更新块 | 抖动顿挫、不同步、速度异常、轨迹不结束 |
| kinematics 算法层 | `kinematics.c` 全部; `main.c` cmdGoto/cmdSolveIKPoint | IK 不可达、FK 超差、姿态错、限位误拒 |
| servo-bus 总线层 | `feetech_hal.c`, `feetech_protocol.c`; `main.c` case 'd' | 无应答、CRC 错、ORE、扫描不到 |
| servo-driver 驱动层 | `feetech_sts3215.c` | 初始化失败、回零异常、模式错(轮子模式) |
| motion-state 状态机 | `main.c`: swing_*, point_loop_*, multi_point_*, correction_* | 循环不停、命令被拒、修正不触发/振荡 |
| pid-flash 持久层 | `pid_flash.c`; `main.c` pid_load/save | 配置掉电丢失、保存失败、回退默认 |
| safety 安全层 | `main.c`: IsEStop, clamp_*, cmdLimits; `kinematics.c` jlim | 误钳制、限位不同步、急停卡死 |
| config 配置层 | `config/arm_config.*`, 各 .c 内生效值 | 改 arm_config.h 无效（drift） |

## 详细故障链

### servo-bus 总线层

```yaml
module: servo-bus
faults:
  - id: BUS-001
    symptom: 单条总线全部无响应（扫描 0 个）
    suspectChain: DXL_ScanBus → DXL_TransmitReceive → DXL_Transmit(方向切换) → UART/74HC126/供电
    firstMove: 执行 'd' 命令分层诊断
    logsToInject: [DXL_Transmit 返回码, DIR 引脚电平, UART SR 寄存器, loopback 收发字节]
    decision:
      loopback失败: PC12→126→PD2 硬件链路（焊接/126供电/方向脚）
      loopback成功+ping失败: 舵机供电、DATA 接线、波特率、ID 冲突
  - id: BUS-002
    symptom: 偶发读写失败 / CRC 错误
    suspectChain: DXL_Transmit 禁RX窗口 → TC等待 → 方向回切延时 → DXL_Receive 超时
    logsToInject: [ORE/FE/NE 标志计数, RX 实际字节数 vs 期望, 失败时总线占用状态]
    knownCauses: [禁RX被意外跳过, 忙等延时时长不足, 200Hz写与读请求碰撞]
  - id: BUS-003
    symptom: 200Hz 轨迹时舵机掉线/丢步
    suspectChain: SetPositionFast fire-and-forget → 总线饱和 → 读请求(GetPosition)超时
    logsToInject: [每总线每秒包数, 插值块执行时长, 读失败计数]
    note: 轨迹期避免高频 STS3215_GetPosition；反馈用 pv 低频
```

### kinematics 算法层

```yaml
module: kinematics
faults:
  - id: KIN-001
    symptom: "IK: unreachable!"
    suspectChain: cmdGoto → IK_Solve: WORLD_OFFSET 逆变换 → 可达性(planar_dist > ARM_MAX_REACH+0.5) → 5起点CCD best_err>1.0mm
    logsToInject: [去除偏移后的 x/y/z, r_target/h_target, planar_dist, 各起点 err, best_err]
    analysis: 先 backtest 复现同一坐标；区分 真不可达 / 限位钳制致 CCD 发散 / 偏移设错
  - id: KIN-002
    symptom: "solution out of joint limits!"（IK 解超限被拒）
    suspectChain: rad_to_servo_deg → JOINT_MIN/MAX 检查
    logsToInject: [解出六角度, 当前限位表]
    note: L 命令改限位后 IK_SetJointLimits 已同步；但 POSE/预设不受此检查保护
  - id: KIN-003
    symptom: IK 移动后末端位置与预期不符（但 FK 验证通过）
    suspectChain: 5 起点 CCD 选了另一组可行解(多解) / WORLD_OFFSET 与实测标定不符
    logsToInject: [5 起点各自 err 与解, 选中起点序号, FK 验证值]
    method: arm_log_lab.py backtest 用同坐标回放，对比固件解与 Python 解
  - id: KIN-004
    symptom: 末端 pitch 不对
    suspectChain: θ6 = pitch - Σθ2..5 → jlim[5] 钳制
    note: Kinematics_Pos.yaw 实为 pitch；a6=0 时 θ6 只影响姿态
```

### trajectory 规划层

```yaml
module: trajectory
faults:
  - id: TRJ-001
    symptom: 启动/停止冲击大（顿挫）
    suspectChain: Trajectory_Plan 三档分支(七段/五段/三段) → SCURVE_TJ=0.15
    logsToInject: [max_d, v_peak, ta, tv, tj, t_total, total_ticks, 分支档]
    note: 短距离自动降档减 tj；若 v/a 配比不当 ta=0 进入三角档
  - id: TRJ-002
    symptom: 六关节不同步到达
    suspectChain: Trajectory_Interpolate 比例分配 ratio=dist/max_d
    analysis: 理论上严格同步；若实际不同步→查舵机侧(PID/速度/负载)而非规划器
  - id: TRJ-003
    symptom: 轨迹"不结束"或提前结束
    suspectChain: total_ticks = t_total*200+1; Interpolate tick>=total → active=0
    logsToInject: [traj_tick 递推, traj_do_update 计数, 主循环一轮耗时]
    knownCauses: [主循环被阻塞操作拖慢(cprint大输出/长DXL读), 插值频率低于200Hz]
```

### motion-state 状态机

```yaml
module: motion-state
faults:
  - id: STA-001
    symptom: "Another loop is active. Use S first."
    suspectChain: point_loop_active | swing_active | multi_point_count 残留
    logsToInject: [三个状态组变量快照, S 命令执行路径]
    note: S 会清零全部循环状态；轨迹自然结束才清 swing/point_loop
  - id: STA-002
    symptom: 多点循环在某点卡住不走
    suspectChain: cmdMultiPointService → cmdMultiPointTargetReached (六轴±2°, 读失败算未达)
    logsToInject: [每轴 actual-target 误差, 读失败计数, dwell 计时]
    knownCauses: [某轴负载大到不了±2°, 自适应修正与多点服务竞争, 读失败顶替目标值掩盖故障]
  - id: STA-003
    symptom: 修正不触发 / 修正永远不收尾
    suspectChain: correction_enabled/pending → 500ms 延时 → 逐轴 OK/TIMEOUT → all_in_tol/any_corrected
    logsToInject: [correction_pending, error/gain/osc/stall 每轮快照, pid 自动调参事件]
  - id: STA-004
    symptom: 修正振荡（OVERSHOOT 刷屏）
    suspectChain: corr_gain 过大 / D-Gain 不足 / 舵机死区
    analysis: gain 应每轮×0.5 收敛；osc>=2 自动 D+4。若仍振荡→查 P 是否被 stall 逻辑推太高
```

### pid-flash 持久层

```yaml
module: pid-flash
faults:
  - id: FLS-001
    symptom: "No valid PID/correction/speed config; using firmware defaults."
    suspectChain: PID_FlashLoad: magic/version/length/CRC → pid_load_startup 范围校验(threshold 50~1000, rounds 1~20, speed≤2047)
    knownCauses: [version 升级后旧记录失效(设计如此, 重发F), 范围校验整包否决, 从未保存过]
  - id: FLS-002
    symptom: "Flash save FAILED."
    suspectChain: PID_FlashSave: 解锁→擦除Sector11→逐字编程→回读全字段比对
    logsToInject: [HAL_FLASHEx_Erase error 码, 编程失败偏移, 回读 mismatch 字段]
    note: 每次 F 整扇区擦除；确认链接脚本未占用 0x080E0000+
```

### safety / console / config

```yaml
module: safety
faults:
  - id: SAF-001
    symptom: 所有运动命令提示 E-STOP
    check: PE11 电平(低有效)；主循环 E-Stop 分支只放行 s/h
  - id: SAF-002
    symptom: 目标角被钳制 "WARNING: n joint(s) clamped"
    check: 当前 JOINT_MIN/MAX (L 命令查看)；注意运行时限位掉电丢失
module: console
faults:
  - id: CON-001
    symptom: 长命令无响应/截断 → rx_ring 128B 溢出静默丢字节
  - id: CON-002
    symptom: 新命令未知 → 检查三处同步(解析分支/cmdHelp/readme)
module: config
faults:
  - id: CFG-001
    symptom: 改 arm_config.h 重编译后行为不变
    rule: 必查 architecture-memory.md 第4节 drift 清单 → 改实际生效点
```

## 跨模块故障链：到位不准（最高频综合故障）

```
症状: m/g 命令完成后实际角度与目标偏差 > 2°
链:
  1. cmdMove 目标钳制?           → 查 clamp 警告日志
  2. S 曲线正常结束?             → "Move complete" 日志
  3. 重力前馈过/欠?              → GRAVITY_COMP_K 值 + J2 水平臂展
  4. 自适应修正触发?             → correction_enabled/pending
  5. 修正收敛还是振荡/停滞?       → corr/OVERSHOOT/STALL 日志序列
  6. PID 是否被自动调参推飞?      → auto-tune 日志 + 当前 P/I/D
  7. 舵机 Moving Speed 太低?     → V 值(raw) 与轨迹 deg/s 不匹配
  8. 负载/温度?                  → s / T 命令
处置: pv 100 采集全程 → arm_log_lab.py backtest --log <file> 自动给出偏差收敛曲线与结论
```
