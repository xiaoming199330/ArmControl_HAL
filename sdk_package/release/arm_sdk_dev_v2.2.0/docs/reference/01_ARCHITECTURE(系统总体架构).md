# 01_ARCHITECTURE.md — 代码总体架构（含"每个子代码包/文件起什么作用、产生什么效果"）

> 目标读者：要写上位机的工程师/AI。**你不需要读懂全部源码**，但必须理解这里的**分层**与**时序约束**，
> 否则会在"为什么我的命令慢/为什么运动时读不到状态"这类问题上反复踩坑。

---

## 1. 系统总览

```
                    ┌──────────── 上位机 (PC/Web) ────────────┐
                    │  你的软件：串口 + 命令 + 解析 + 3D + UI │
                    └───────────────────┬─────────────────────┘
                                        │ USB 转串口
                                        │ 串口: 115200 8N1 无流控
                                        │ 文本命令(ASCII, \r\n) 或 二进制帧(0xAA 0x55)
                    ┌───────────────────▼─────────────────────┐
                    │        STM32F407ZGT6 @168MHz            │
                    │  ┌───────────────────────────────────┐  │
                    │  │ USART6  控制台 (PC6=TX / PC7=RX)  │◄─┼── 与上位机通信
                    │  ├───────────────────────────────────┤  │
                    │  │ USART3  Bus A (PB10/PB11) 1Mbps   │──┼──► J1 J2 J3
                    │  │ UART5   Bus B (PC12/PD2)  1Mbps   │──┼──► J4 J5 J6
                    │  ├───────────────────────────────────┤  │
                    │  │ UART4   主从链路口(PA0/PA1)115200 │──┼──► 另一块板(本版上位机不用)
                    │  ├───────────────────────────────────┤  │
                    │  │ TIM2 2kHz 节拍  │ 急停 PE11 低有效│  │
                    │  └───────────────────────────────────┘  │
                    └─────────────────────────────────────────┘
                                        │ SCS 半双工串行总线 (TTL)
                    ┌───────────────────▼─────────────────────┐
                    │ 6 × Feetech STS3215 (12V 30KG)          │
                    │ J1 底座旋转 J2 肩 J3 肘 J4 腕俯仰         │
                    │ J5 腕自转  J6 夹爪                       │
                    └─────────────────────────────────────────┘
```

---

## 2. 分层架构（自上而下）

```
┌──────────────────────────────────────────────────────────────────────┐
│ L5  命令层（人类/AI 面向）                                            │
│     main.c: ProcessCommand() / cmdHelp()  —— ASCII 文本命令解析        │
│     arm_commands.c: cmd_registry[]        —— 命令登记表(速查/文档源)   │
├──────────────────────────────────────────────────────────────────────┤
│ L4  协议层（机器面向，二进制）                                        │
│     arm_control_protocol.c/h   帧格式/命令码/CRC16/定点编解码/分发     │
│     arm_proto_serial.c/h       串口承载(两条口: 控制台口 + 主从口)     │
│     arm_proto_can.c/h          CAN 承载(可选, 需外接收发器)            │
├──────────────────────────────────────────────────────────────────────┤
│ L3  业务门面层（统一 API，供 L4/L5 调用）                             │
│     arm_control_api.c/h   MoveJoints / MoveCartesian / GetStatus / …   │
│     arm_params.c/h        运行时可调参数表(指针绑定单一生效点)         │
│     arm_runlog.c/h        结构化运行日志([RUN] 前缀)                   │
├──────────────────────────────────────────────────────────────────────┤
│ L2  算法层                                                            │
│     kinematics.c/h    正/逆运动学 (FK / IK-CCD)                       │
│     trajectory.c/h    七段 S 曲线(jerk-limited)速度规划               │
│     teach_store.c/h   示教录制与 Flash 文件管理                       │
│     servo_driver.c/h  STS3215 关节级高层驱动                          │
├──────────────────────────────────────────────────────────────────────┤
│ L1  驱动/硬件抽象层                                                   │
│     scs_protocol.c/h   Feetech SCS 帧编解码/收发                      │
│     servo_bus_hal.c/h  总线切换与方向控制(74HC126 / 开漏)             │
│     usart.c/h          UART 外设初始化(USART3/UART4/UART5)            │
│     tim.c/h  gpio.c/h  stm32f4xx_it.c  HAL MSP                        │
│     pid_flash.c/h      参数持久化(Flash Sector 11)                    │
└──────────────────────────────────────────────────────────────────────┘
```

> **上位机只与 L4（二进制）和 L5（文本）打交道，永远看不到 L1~L3。**
> 上位机**不需要**、也**不应该**知道"SCS 帧怎么拼"、"总线怎么切向"。

---

## 3. 文件清单：每个包做什么、会产生什么效果

> 「上位机相关度」：🔴 = 必须理解　🟡 = 了解即可　⚪ = 可忽略

### 3.1 必须理解的部分

| 文件 | 作用 | 产生什么效果 | 上位机相关度 |
|---|---|---|---|
| `Core/Src/main.c`（6458 行） | **业务主层**：文本命令解析与执行、预设位姿、关节限位、运动入口(`cmdMove`)、示教模式、遥测调度、主循环 | **所有文本命令的实现都在这里**；默认值、限位、位姿常量都在这里定义 | 🔴 |
| `Core/Inc/arm_control_protocol.h` | 二进制协议**权威定义**：帧格式、命令码、CRC 多项式、定点比例 | 决定你二进制接口怎么写 | 🔴 |
| `Core/Src/arm_control_protocol.c` | 协议实现：`Aproto_Crc16` / `Aproto_ParserFeed`（逐字节状态机）/ `Aproto_BuildFrame` / `Aproto_Dispatch` | CRC 算错 → 全部 NACK | 🔴 |
| `Core/Inc/kinematics.h` + `Core/Src/kinematics.c` | **运动学权威**：FK 公式、IK 策略、几何常量、角度换算 | 决定 3D 模型的姿态与实机是否一致 | 🔴 |
| `Core/Inc/arm_params.h` + `arm_params.c` | 运行时参数表（24 项）：id / 名称 / 类型 / 范围 / 默认 / 单位 | `LP` / `SP` / 二进制参数读写的数据源 | 🔴 |
| `Core/Src/arm_control_api.c/h` | 业务门面：`MoveJoints`/`MoveCartesian`/`Stop`/`GetStatus`/`SetTorque`/`SetGravityComp`/`SetCorrection`；返回码 `ARM_API_ERR_*` | 二进制命令的**错误码来源** | 🔴 |
| `Core/Src/arm_commands.c` | 命令登记表 `cmd_registry[]`（语法/摘要/分组/备注） | 文档源。⚠ **本表维护不同步**：缺 `K`/`N`/`G`/`U`/`R`/`X`/`A`/`V`/`LL`/`M`/`e`，且把 `N J P I D` 误记为 `I J P I D`。**以 `03_INTERFACE.md` 与 `cmdHelp` 为准** | 🟡 |

### 3.2 了解即可的部分

| 文件 | 作用 | 产生什么效果 |
|---|---|---|
| `Core/Src/servo_driver.c/h` | STS3215 关节级驱动：位置/速度/加速度/力矩/PID 读写、批量 SyncWrite、遥测整组读 | 决定"一条命令多久发到舵机" |
| `Core/Src/scs_protocol.c/h` | SCS 帧层：`FF FF ID Len Inst Params CK`，含诊断版读（区分无应答/帧坏/告警） | 读失败三级诊断的来源 |
| `Core/Src/servo_bus_hal.c/h` | 总线方向切换：旧 74HC126 DIR 脚 / 主臂开漏单总线 | 主臂/从臂总线差异 |
| `Core/Src/trajectory.c/h` | 七段 S 曲线规划（200Hz 时间基准，jerk 限制） | 运动的平滑度；`TRAJ_SCURVE_TJ` 可调 |
| `Core/Src/teach_store.c/h` | 示教录制：命名文件存 Flash、列表、回放、删除、压缩回收 | `save/list/play/del/compact` 的底座 |
| `Core/Src/pid_flash.c/h` | 持久化：把 PID/修正/速度/限位/零点写入 Flash Sector 11 | `F` 命令；开机自动恢复 |
| `Core/Src/arm_runlog.c/h` | 结构化日志：`[RUN]` 前缀 + 级别 `LL 0~4` | 上位机解析时要**跳过** |
| `Core/Inc/arm_main_exports.h` | `main.c` 向 `arm_control_api.c` 导出的符号清单（唯一耦合点） | 新增导出必须先登记，防 config-drift |
| `Core/Src/tim.c` | TIM2 初始化（PSC=83, ARR=499 → **2 kHz**） | 控制节拍来源 |
| `Core/Src/usart.c` | USART3 / UART4 / UART5 初始化（1Mbps / 115200 / 1Mbps） | 总线参数 |
| `Core/Src/gpio.c` | GPIO 初始化（含 PG0~PG5 限位、PC4/PC5/PA5 LED、PB5 蜂鸣器） | 安全输入 |

### 3.3 可忽略的部分

`system_stm32f4xx.c`（CMSIS）、`stm32f4xx_it.c`（中断向量）、`stm32f4xx_hal_msp.c`、
`Drivers/`（ST HAL 库）、`startup_stm32f407xx.s`。

---

## 4. 运行时序（**关键！决定"为什么运动中读不到状态"**）

### 4.1 三层时间基准

| 基准 | 频率 | 由谁产生 | 干什么 |
|---|---|---|---|
| **TIM2 节拍** | **2 kHz** (0.5 ms) | `TIM2_IRQHandler` → `HAL_TIM_PeriodElapsedCallback` | 心跳 LED（2Hz）；把 2kHz 分频成 **50 Hz**（每 40 拍）置 `traj_do_update=1` |
| **轨迹下发节拍** | **50 Hz** (20 ms) | 主循环看到 `traj_do_update` | 每拍推进 4 个 **200 Hz** 插值 tick，然后 **一次 SyncWrite 六轴目标**（同总线同组轴必须一帧同发，否则机械会歪） |
| **遥测采样节拍** | **50 Hz** (20 ms) | `TELEM_SAMPLE_PERIOD_MS = 20` | 后台每拍**只读 1 个关节**（轮转，6 轴 120ms 刷一遍），打印只读缓存，**不占总线** |

### 4.2 主循环（`main()` 的 `while(1)`）每拍做什么

```
while (1) {
    FeedbackSampleService();          // 舵机反馈采样(单总线"边发边收"，分片)
    cmdMultiPointService();           // 多点/循环运动状态机 (QL/QLC/B/W)
    cmdTrajectorySettleService();     // 轨迹到位 settle 判定
    cmdStreamService();               // 主从直写看门狗(断流>500ms 自动退出)
    if (correction_pending && !traj && !swing) { ...自适应修正状态机... }
    if (feedback_period_ms && 到点) { ...打印 pv 角度流... }
    if (pose_feedback_period_ms && 到点) { ...打印 pf 位姿流... }
    TelemetryPrintService();          // 打印 tm 遥测缓存
    if (IsEStop()) { 停轨迹; 只允许 s/h; }
    int len = Console_TryReadLine();  // ← 文本命令在这里被取出
    if (len > 0) ProcessCommand(...); // ← 文本命令在这里被执行
    SerialProto_DispatchPending();    // ← 二进制帧(控制台口)在这里被执行 + 回包
    LinkProto_DispatchPending();      // 主从链路口(本版上位机不用)
    master_service();                 // 主臂转发(本版上位机不用)
}
```

### 4.3 对上位机的三条实际影响

1. **命令是"主循环空闲时"才执行的**。如果此刻正在跑一条长时间轨迹，你的命令会排队，
   **响应可能延迟到轨迹结束**（除非是 `S`）。
2. **运动期间读状态会明显变慢**：单总线上"读"必须插在两次"批量写"之间的空档，
   每 20 ms 只允许 1 次读事务（6 轴 120 ms 才刷完一轮）。
   ⇒ **上位机不要用高频率(如 50Hz)去轮询 `s` 或 `GET_STATUS`**，会与运动抢总线。
   建议：运动时用 `tm`（只读缓存，不占总线）或把查询频率降到 5~10 Hz。
3. **`cprint` 是阻塞发送**：控制台 115200，100 字符 ≈ 9 ms。
   ⇒ 大量打印会拖慢运动节拍。上位机**不要**在运动时发 `pv`/`pf` 高速流（建议 `>=100ms`）。

---

## 5. 关键数据流：一条命令的完整生命周期

### 5.1 文本命令（例：`m 10 20 30 40 50 60`）

```
上位机 write("m 10 20 30 40 50 60\r\n")
   │  USART6 RX 中断 (每字节)
   ▼
rx_ring[] 环形缓冲
   │  主循环
   ▼
Console_TryReadLine()            // 以 \r 或 \n 判行结束；回显字符
   ▼
ProcessCommand(line, len)        // strtok 空格/Tab 分词
   ├─ teach/回放模式拦截 (可能自动退出 teach / 中止回放)
   ├─ 多字符命令 (tm/pv/pf/teach/save/list/play/del/compact/BG/QL/QLC/…)
   └─ switch (tokens[0][0])     // 单字符命令
        case 'm': cmdMove(target, v, a)
                   ├─ cmdStreamExit()            // 防两套目标互抢
                   ├─ 限位钳制 (JOINT_MIN/MAX)
                   ├─ read_start_pose_checked()  // 读起点(表决)，失败则拒绝
                   ├─ 开扭矩
                   ├─ Trajectory_Plan()          // 七段 S 曲线
                   └─ 置 traj.active = 1
   ▼
[TIM2 2kHz] → traj_do_update (50Hz)
   ▼  主循环
Trajectory_Interpolate(tick, angles)  // 200Hz 时间线，每拍推 4 tick
   ▼
SERVO_SetPositionsSync(angles)        // 按 JOINT_BUS 分组，每组一帧 SyncWrite
   ▼
[USART3 / UART5] SCS 帧 → 6 颗 STS3215
```

### 5.2 二进制命令（例：`MOVE_CART`）

```
上位机 write(AA 55 SEQ 20 LEN <payload> CRC_LO CRC_HI)
   ▼  USART6 RX 中断
SerialProto_FeedByte(ch)   // 先判 0xAA；是协议字节则进 Aproto_ParserFeed，
                           // 否则落到文本 rx_ring（天然分流，无歧义）
   ▼  主循环
SerialProto_DispatchPending()
   ▼
Aproto_Dispatch(seq, cmd, payload, len, rsp, ...)
   ├─ 入口守卫: cmd>=0x80 || cmd==ACK || cmd==NACK → return 0  (防回包 ping-pong)
   ├─ 0x20: 解 payload → ArmApi_MoveCartesian(x,y,z,pitch,v,a)
   │         ├─ IK_Solve()            // 解析解 → 失败则 CCD 多起点
   │         ├─ 关节限位检查          // 超限 → ERR_LIMIT(4)
   │         ├─ FK 回代验证           // 超差 → ERR_FK_VERIFY(6)
   │         └─ ArmApi_MoveJoints()   // 内部走 cmdMove 安全链
   └─ 组响应帧: ACK(0x7F) 或 NACK(0x7E)+错误码
   ▼
HAL_UART_Transmit(&hconsole, rsp, len)   // 二进制响应帧
```

---

## 6. 硬件与时序约束（写上位机必须知道的"物理事实"）

| 约束 | 说明 | 对你的影响 |
|---|---|---|
| **单总线半双工** | Bus A(J1-3) 与 Bus B(J4-6) 各是一条 1Mbps 半双工总线；主臂变体是**一条**总线挂 6 颗（开漏） | 读必须插空档；不能高频轮询 |
| **同组必须同步写** | 同一总线上的多个轴必须**一帧 SyncWrite** 同时起跑，逐轴写会让机械歪 | 你看到"六轴同时到"是固件保证的，不要试图分开控制 |
| **Bus A 偶发丢帧** | Bus A(J1/J2/J3) 物理上偶发丢帧，Bus B 从不。固件用"重试 + 3 采样表决"兜住 | `s` 偶尔出现 `READ FAIL`，重发一次通常就好 |
| **急停是硬线** | PE11 低有效；激活时只允许 `s`/`h` | UI 必须有急停状态显示 |
| **限位是硬输入** | PG0~PG5 低有效，内部上拉 | 限位触发时运动会被拒 |
| **`cprint` 阻塞** | 控制台 115200，100 字符 ≈9ms | 打印要节流 |

---

## 7. 持久化的三个层次（**上位机最容易搞错的地方**）

```
         改值命令                生效范围                 持久在哪
  ┌────────────────────┬─────────────────────┬───────────────────────────┐
  │ K / I / D / N      │ STM32 内存缓存      │ ✗ 掉电丢                  │
  │ (PID 缓存)          │ (pid_p/pid_i/pid_d) │ 要 Y J 烧舵机 EEPROM      │
  ├────────────────────┼─────────────────────┼───────────────────────────┤
  │ V (速度)            │ STM32 + 立即刷舵机  │ ✗ 要 F                    │
  │ SP id v (参数)      │ 直接写生效点(RAM)   │ ✗ 要 F                    │
  ├────────────────────┼─────────────────────┼───────────────────────────┤
  │ Y J                │ 舵机 EEPROM 21/22/23│ ✓ 舵机掉电保留(约0.6s失力)│
  │ F                  │ Flash Sector 11     │ ✓ 开机自动恢复             │
  │ c + s (校准)       │ Flash (限位+零点)   │ ✓ 开机自动恢复             │
  └────────────────────┴─────────────────────┴───────────────────────────┘
```

**上位机在 PID/参数面板上必须体现这条链**：
`改缓存 → Y 烧 EEPROM → F 存 Flash`，三步缺一不可，且要能显示"缓存 vs EEPROM 是否一致"（用 `N J`）。

---

## 8. 编译与角色（本版上位机只关心"从臂/单臂"）

| 命令 | 产出 | 说明 |
|---|---|---|
| `make` | `build/ArmControl_HAL.bin` | **单臂/从臂固件**（本版上位机的对象） |
| `make ROLE=master` | `build_master/ArmControl_Master.bin` | 主臂固件（主从臂功能，本版不做） |
| `make BAUD=19200` | 同上，链路波特率改 19200 | 调试用 |
| `make ROLE=master BAUD=19200` | 主臂 + 19200 | 调试用 |

- 工具链：`C:\STM32Tools\arm-gcc-full\bin` + `C:\STM32Tools\make`
- 编译命令示例：
  ```
  cd "C:/Users/Administrator/Desktop/ArmControl_HAL（1）"
  PATH="/c/STM32Tools/arm-gcc-full/bin:/c/STM32Tools/make:$PATH" make -j4
  ```
- 烧录：`stm32flash -w build/ArmControl_HAL.bin -v -g 0x08000000 COM<xx>`（BOOT0 拉高 + 复位）

---

## 9. 时钟树（钉死波特率等参数的依据）

```
HSE 8 MHz  ──PLLM=8──► 1 MHz ──PLLN=336──► 336 MHz ──PLLP=DIV2──► SYSCLK 168 MHz
   AHB  = /1 → 168 MHz
   APB1 = /4 →  42 MHz   (USART3 / UART4 / UART5 挂 APB1)
   APB2 = /2 →  84 MHz   (USART6 挂 APB2)
   TIM2 挂 APB1，定时器时钟 = 84 MHz → /(83+1)/ (499+1) = 2 kHz
```

> ⚠ 注意：APB1=42 MHz 时，USART3/UART4/UART5 的**实际波特率是 115068**（不是 115200），
> 这是"标称 115200 / 实际 115068"的 −0.11% 偏差，属正常，**不影响上位机**（上位机用标准 115200 即可）。
> 舵机总线 1 Mbps 同理（实际略偏），STS3215 容差足够。

---

## 10. 与上位机交互的"最小必要知识"（一页速记）

```
1. 串口 115200 8N1，无流控。
2. 每条命令以 \r\n 结尾。
3. 回包 = 逐字符回显 + 命令输出 + "> "；解析要过滤回显与提示符。
4. 单轴命令 J=0~5；PID 命令 J=1~6。
5. 舵机度(0~360) 与 关节角(rel-home, 可负) 是两套，不可混用。
6. 运动类命令会做安全检查，可能被拒——要处理"被拒"返回。
7. PID 三步：改缓存(K/I/D/N) → Y 烧 EEPROM → F 存 Flash。
8. 参数：SP 改 RAM → F 存盘。
9. 运动时不要高频轮询状态；用 tm 或降频。
10. 二进制帧以 0xAA 0x55 开头，与文本共用串口。
```
