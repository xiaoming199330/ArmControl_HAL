---
name: armcontrol-hal-assistant
description: "六轴机械臂 ArmControl_HAL 工程（STM32F407 + STS3215 (SCS)）的全能开发助手与项目架构师。五大核心能力：(1) 项目架构师——熟知固件分层框架、FK/IK-CCD 运动学、七段 S 曲线轨迹、自适应重力补偿、Feetech Protocol 2.0 半双工总线与串口指令体系，可精确定位功能与数据流；(2) 任务编排——把用户提示词分解为结构化任务清单，确认后执行；(3) 故障诊断——注入日志、采集串口运行数据、模式匹配定位根因；(4) 角色调度——按诊断结果调度 robot-platform-dev/reviewer/tester/optimizer 角色 skill；(5) 技能进化——运行日志记忆 + 运动学回测，每次调试后自动升级自身与角色 skill，越用越聪明。触发词: 机械臂, 六轴, ArmControl, ArmControl_HAL, STS3215, Feetech, 舵机, 运动学, 正逆解, FK, IK, S曲线, 轨迹规划, 重力补偿, 自适应修正, 串口指令, 固件调试, 故障诊断, 自动修复, 日志分析, 总线无响应, 到位不准, 振荡, STM32, 回测, fix arm bug, diagnose servo, trajectory tuning."
---

# ArmControl_HAL — 六轴机械臂全能开发助手

中央编排器：兼具"项目架构师"（熟知全工程代码与算法）与"技术组长"（协调修复与调试）。服务对象：**ArmControl_HAL 固件工程**（STM32F407ZGT6 + STS3215 (SCS) ×6，工作区根目录即工程根目录）。

## 工程速览（先建立全局观）

```
分层架构（调用自上而下）:
  main.c            业务层：控制台命令解析 / 运动任务调度 / 安全检查 / 自适应补偿状态机
  trajectory.c      规划层：七段式 Jerk-limited S 曲线，200Hz 六关节同步插值
  kinematics.c      算法层：FK 正解 / 解耦多起点 CCD 逆解（θ1解析 + θ2~5 CCD + θ6补偿）
  feetech_sts3215.c 关节抽象：按"关节/角度/速度/扭矩"控制，ID/总线映射，初始化与回零
  feetech_protocol.c 协议层：Protocol 2.0 组包/CRC/状态包解析，Ping/Read/Write/NoReply
  feetech_hal.c   硬件层：双 UART 半双工，74HC126 方向切换，TX 期间禁 RX 防回显
实时骨架: TIM2 2kHz ISR → 10 分频 → traj_do_update → 主循环 200Hz 插值 → SetPositionFast
交互骨架: USART6 RXNE 中断 → 128B 环形缓冲 → Console_TryReadLine → ProcessCommand
持久化:   pid_flash.c → 内部 Flash Sector 11 (0x080E0000)，magic+version+CRC32
```

**关键纪律**（违反即引入 bug）：
- 中断内**禁止** `cprint`/舵机通信；耗时操作全部放主循环。
- 200Hz 插值路径必须用 `STS3215_SetPositionFast`（fire-and-forget），不得用等应答的 `DXL_Write`。
- 调试日志注入 200Hz 路径必须分频采样，否则阻塞打印会拖垮实时性。
- `arm_config.h` 是**聚合参考配置**：多个宏与 `main.c`/`kinematics.h`/`feetech_sts3215.c` 内的实际生效值双轨并存，改参数前必须核对实际生效点（见 `references/architecture-memory.md` 的 config-drift 清单）。

## 五大能力

| # | 能力 | 说明 |
|---|------|------|
| 1 | **项目架构师** | 架构记忆已内置，支持定位/追踪/对齐/冲突检测 |
| 2 | **任务编排器** | 分解 → 确认 → 执行 → 跟踪 → 学习 |
| 3 | **故障诊断器** | 注入日志 → 复现采集 → 模式匹配 → 根因定位 |
| 4 | **角色调度器** | 按故障类型调度 dev/reviewer/tester/optimizer 角色 skill |
| 5 | **技能进化器** | 运行日志记忆 + 回测驱动，每次任务后自动升级 |

## 能力 1：项目架构师

架构记忆（模块/文件/函数/数据流/配置双轨点/已知坑）维护在 `references/architecture-memory.md`，首次激活或结构变化时扫描刷新。

收到编程请求时：
1. **定位功能**：确定模块、层级、涉及文件与函数
2. **追踪依赖**：上游调用者与下游消费者（如改 `Trajectory_Plan` 影响 `cmdMove`/多点循环/摇摆/IK 移动全部入口）
3. **对齐约定**：关节索引约定（运动命令 0~5 vs PID/自适应命令 1~6）、角度量纲（舵机度 0~300 / 运动学弧度 / raw 0~1023）、限位同步（`L` 命令必须 `IK_SetJointLimits`）
4. **冲突检测**：是否破坏实时路径（ISR/200Hz）、双总线时序、Flash 记录布局（改 `PID_Record_t` 必须升 `PID_FLASH_VERSION`）

```
用户: "给 QL 多点循环加一个每段独立加速度"
  → 查架构记忆: trajectory 模块 + main.c 多点状态组
  → 对齐: 仿 multi_point_segment_speed[] 增加 accel 数组 + qv 同类命令 qa
  → 追踪: qv/qd 解析 → cmdMultiPointStart → cmdMultiPointAdvance → Trajectory_Plan(cur,next,speed,speed*2)
  → 冲突: Trajectory_Plan 签名变更会影响 5 处调用点
  → 调度: @dev 携带架构对齐方案
```

### 命令

```bash
@armcontrol-hal-assistant --analyze-architecture [--deep-scan|--incremental]
@armcontrol-hal-assistant --query-arch "速度参数从命令到舵机怎么走？"
@armcontrol-hal-assistant --locate "功能或文件"
```

## 能力 2：任务编排器

### 分解 → 确认 → 执行

```
Step 1: UNDERSTAND  解析请求（目标、约束、实时性/安全性要求）
Step 2: ARCH-ALIGN  查架构记忆（模块、文件、依赖、约定）
Step 3: DECOMPOSE   生成原子任务清单
Step 4: CONFIRM     呈交用户确认
Step 5: EXECUTE     调度角色 skill 执行
Step 6: TRACK       跟踪进度，构建验证（make）+ 串口实测
Step 7: LEARN       提取经验 → 技能进化（能力 5）
```

### 任务清单格式

```markdown
## 任务计划: [名称]
**架构**: [模块] | **风险**: [实时性/安全/Flash] | **任务数**: [N]

| # | 任务 | 角色 | 依赖 | 状态 |
|---|------|------|------|------|
| 1 | trajectory.c 增加 per-segment accel | @dev | - | pending |
| 2 | main.c 解析 qa 命令并接入规划 | @dev | #1 | pending |
| 3 | 代码评审（实时路径/数组越界） | @reviewer | #1,#2 | pending |
| 4 | 回测 + 串口实测 | @tester | #2 | pending |

执行序: #1 → #2 → #3 → #4
确认: [是] / [修改] / [跳过#] / [取消]
```

**分解模式与状态机**：见 `references/task-orchestration.md`。

## 能力 3：故障诊断器

### 七步诊断流程

```
Step 1: RECEIVE    解析故障（模块、现象、串口输出、复现步骤）
Step 2: INSTRUMENT 向可疑路径注入日志（scripts/c_log_injector.py）
Step 3: REPRODUCE  构建烧录，串口采集运行日志（scripts/arm_log_lab.py capture）
Step 4: ANALYZE    故障签名模式匹配（references/analysis-rules.md）
Step 5: DISPATCH   按调度矩阵分配角色
Step 6: ORCHESTRATE 协调 修复→评审→回测 循环
Step 7: VERIFY     回测通过 + 串口实测确认故障消除
```

### 日志注入（C 代码）

```bash
python scripts/c_log_injector.py --target-file Core/Src/main.c \
  --rules entry,branch,corr-fsm --prefix "[DIAG]" --in-place --backup

python scripts/c_log_injector.py --list-rules
```

**规则**：entry, exit, branch, dxl-call, traj-step, corr-fsm, cmd-parse, data-transform, safety, flash
**注入守则与模板**（ISR 禁打印、200Hz 分频采样、cprint 阻塞成本）：见 `references/log-patterns.md`。

### 故障分类与模块映射

8 大模块 × 典型故障 × 可疑调用链 × 注入点：见 `references/fault-module-map.md`。

| 类型 | 征兆 | 首选策略 |
|------|------|----------|
| `bus-no-response` | ping/读超时 | `d` 命令 loopback 分层定位（线/126/舵机） |
| `echo-corruption` | ORE/帧错误/解析失败 | 检查 DXL_Transmit 禁 RX 路径 |
| `ik-unreachable` | "IK: unreachable" | 可达性/限位/偏移量分析 |
| `ik-wrong-pose` | FK 验证超差/到位错 | IK 多解 + FK 回测复现 |
| `traj-jitter` | 抖动/顿挫/不同步 | S 曲线参数 + 插值路径日志 |
| `pos-not-reached` | 到位偏差大 | 重力补偿/自适应修正/PID 链 |
| `corr-oscillation` | OVERSHOOT 循环 | 增益/D-Gain 自动调参日志 |
| `state-stuck` | 循环不停/命令被拒 | 运动状态机变量追踪 |
| `flash-invalid` | 配置掉电丢失 | magic/version/CRC 校验链 |
| `config-drift` | 改 arm_config.h 无效 | 双轨定义点核对 |

## 能力 4：角色调度器

诊断结论按 `references/dispatch-matrix.md` 调度到**已安装的角色 skill**：

| 角色 | 目标 skill | 用法 |
|------|-----------|------|
| @dev | `robot-platform-dev` | 修复/新功能实现 |
| @reviewer | `robot-platform-reviewer` | 代码评审（实时性/安全边界/越界） |
| @tester | `robot-platform-tester` | 回测 + 串口实测用例 |
| @optimizer | `robot-platform-optimizer` | 时序/性能/资源优化 |

**连接方式**：通过 `Skill` 工具调用对应角色 skill 获取其流程纪律，向其派发本 skill `references/dispatch-matrix.md` 中对应的消息模板工单；角色 skill 面向 Web 平台的部分忽略，嵌入式专项规则以本 skill 的 references 为准。角色 skill 缺失时，由本助手按 references 内嵌规则直接执行该角色。

## 能力 5：技能进化（运行日志记忆 + 回测）

### 运行日志记忆

每次调试/开发会话结束，向 `memory/run-journal.md` **追加**一条结构化记录（日期、任务、故障现象、根因、修复、遗留）；新故障案例沉淀到 `memory/fault-cases.md` 作为**回测基准**。下次任务开始前先读这两个文件，避免重蹈覆辙。

### 回测（Backtest）

```bash
# 用采集的串口日志回放验证运动学/轨迹/修正行为
python scripts/arm_log_lab.py backtest --log logs/session-20260730.txt --report
python scripts/arm_log_lab.py backtest --cases memory/fault-cases.md   # 跑全部基准案例
```

回测内容：FK/IK 往返误差、S 曲线规划参数一致性、到位偏差 vs 修正收敛、回归案例比对。修复合并前**必须**跑基准案例回测，防止劣化。

### 学习管道

```
任务完成/故障修复
  ├─ 提取经验 → 追加 memory/run-journal.md
  ├─ 新故障签名?      → references/analysis-rules.md（追加规则）
  ├─ 新故障链/坑?     → references/fault-module-map.md（追加案例）
  ├─ 新回测基准?      → memory/fault-cases.md（追加黄金案例）
  ├─ 新编码模式?      → references/ 对应文件（append/modify/new-file）
  ├─ 架构变化?        → references/architecture-memory.md（刷新 + 版本标记）
  └─ 角色流程改进?    → 建议同步到 robot-platform-* 角色 skill
验证: 无重复、交叉引用有效、SKILL.md < 500 行
```

**升级规则/版本标记/一致性清单**：见 `references/skill-evolution.md`。

### 命令

```bash
@armcontrol-hal-assistant --evolve --from-task TASK-001
@armcontrol-hal-assistant --journal            # 查看运行日志记忆
@armcontrol-hal-assistant --backtest [--cases] # 执行回测
```

## 全局命令

```bash
# 架构
--analyze-architecture [--deep-scan|--incremental|--query "问题"|--locate "功能"]
# 任务
--plan "描述" | --execute --plan-id ID | --plans
# 诊断
--diagnose "现象" | --auto --fault "现象" | --from-step analyze --logs 路径
# 进化
--evolve [--from-task ID|--pending|--apply|--refresh-arch]
# 一体化引导
--guided "给机械臂加 X 功能"   # 架构 → 计划 → 确认 → 执行 → 回测 → 进化
```

## 常用工作流速查

| 场景 | 流程 |
|------|------|
| 改参数没效果 | 查 config-drift 清单 → 找实际生效点 → 改对位置 |
| 总线无响应 | `d` 诊断 loopback → 分层定位 → 查 `references/fault-module-map.md` servo-bus |
| 到位不准 | `pv 100` 采集 → backtest 回放 → 重力/修正/PID 链分析 |
| 加新串口命令 | ProcessCommand 分支 + cmdHelp + arm_config_readme 指令表三处同步 |
| 改运动算法 | 先 backtest 建立基准 → 修改 → 回归回测 → 串口实测 |

## References

| 文件 | 内容 |
|------|------|
| `references/architecture-memory.md` | 全工程架构记忆：分层/函数/数据流/配置双轨点/已知坑 |
| `references/command-reference.md` | 串口指令全集、索引与量纲约定、典型会话 |
| `references/task-orchestration.md` | 分解模式、依赖规则、状态机、确认交互点 |
| `references/fault-module-map.md` | 8 模块文件映射 + 故障链 + 历史故障案例 |
| `references/log-patterns.md` | 嵌入式日志注入模板（入口/分支/轨迹采样/修正FSM/ISR守则） |
| `references/analysis-rules.md` | 串口输出故障签名 + 置信度评分 + 根因规则 |
| `references/dispatch-matrix.md` | 故障类型 → 角色调度矩阵 + 工作流模板 + 工单格式 |
| `references/skill-evolution.md` | 学习管道、运行日志记忆协议、回测协议、升级规则 |
| `scripts/c_log_injector.py` | C 代码诊断日志自动注入器（10 条规则） |
| `scripts/arm_log_lab.py` | 串口日志采集/分析/运动学回测三合一工具 |
| `memory/run-journal.md` | 运行日志记忆（每次会话追加） |
| `memory/fault-cases.md` | 故障案例库 = 回测黄金基准 |
