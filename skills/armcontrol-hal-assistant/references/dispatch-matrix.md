# 角色调度矩阵（Dispatch Matrix）

> 角色 skill 连接：通过 `Skill` 工具调用对应已安装 skill 获取其流程纪律，再派发本文件工单。
> 角色 skill 面向 Web 平台的内容忽略；嵌入式专项规则以本 skill references 为准。
> 角色 skill 缺失时：本助手按下表"内嵌职责"直接执行该角色。

## 角色连接表

| 角色 | 目标 skill（已安装） | 嵌入式内嵌职责（兜底/补充） |
|------|---------------------|---------------------------|
| @dev | `robot-platform-dev` | 按架构记忆实施修复/功能；遵守分层与实时纪律；改后跑 `make` |
| @reviewer | `robot-platform-reviewer` | 评审：ISR/200Hz 阻塞、数组越界、量纲、限位同步、Flash 布局、drift 清单 |
| @tester | `robot-platform-tester` | 回测（arm_log_lab.py backtest）+ 串口实测用例；回归 fault-cases |
| @optimizer | `robot-platform-optimizer` | 时序/CPU 占用/总线带宽/内存优化；前后数据对比 |

## 调度决策树

```
DIAGNOSIS_RECEIVED
  ├─ 硬件/接线嫌疑 (loopback fail, 供电)     → 直接给用户硬件检查清单（无需角色）
  ├─ 安全风险 (E-Stop失效, 限位绕过, Flash区越界) → reviewer(审计) → dev → reviewer → tester
  ├─ 实时性/性能 (ISR overrun, 轨迹卡顿)      → optimizer(分析) → dev+optimizer → reviewer → tester
  ├─ 算法错误 (IK/轨迹/修正逻辑)              → dev → reviewer → tester(回测为主)
  ├─ 数据流/量纲 (角度/索引/限位不同步)        → dev → reviewer → tester(集成)
  ├─ 跨模块 (如 到位不准 综合链)              → dev(设计) → reviewer → dev → tester(回测+实测)
  └─ 未知/复杂                                → dev(带架构记忆调查) → 重新诊断
```

## 故障类型调度矩阵

| 故障类型 | 主角色 | 副角色 | 评审重点 | 测试焦点 | 工作流 |
|---------|--------|--------|---------|---------|--------|
| BUS-NO-RESPONSE | dev | - | 时序安全 | d 诊断实测 | A 快速 |
| BUS-LOOPBACK-FAIL | （硬件清单） | - | - | 接线复查 | 硬件 |
| DXL-READ-FAIL | dev | - | 错误处理 | 总线压力实测 | A 快速 |
| IK-UNREACHABLE | dev | tester | 边界条件 | backtest 坐标集 | A 快速 |
| IK-FK-VERIFY | dev | tester | 收敛逻辑 | backtest 多解集 | C 复杂 |
| TRAJ-JITTER | dev | optimizer | 规划三档分支 | backtest 规划一致性 | B 性能 |
| ISR-OVERRUN | optimizer | dev | 阻塞点清除 | 计时对比 | B 性能 |
| CORR-OSC / STALL | dev | - | 增益边界 | backtest 收敛曲线 | A 快速 |
| POS-NOT-REACHED | dev | tester | 补偿链完整 | backtest + 实测 | C 复杂 |
| LOOP-STUCK | dev | tester | 状态清零路径 | 循环实测 | C 复杂 |
| FLASH-INVALID | dev | - | 版本/校验链 | 掉电重启实测 | A 快速 |
| SAFETY-BYPASS | reviewer | dev | 安全审计 | 急停/限位实测 | D 安全 |
| CONFIG-DRIFT | dev | - | drift 清单 | 行为验证 | A 快速 |
| REGRESSION | dev | tester | 变更 diff | fault-cases 全量回测 | A 快速 |

## 工作流模板

### A：快速修复（10~30 分钟）
```
dispatch → dev:FIX → reviewer:REVIEW(实时/越界/量纲/同步清单) → tester:VERIFY(backtest+必要时实测) → CLOSE
评审或测试失败 → 回 dev；连续失败走升级规则
```

### B：性能优化（1~4 小时）
```
dispatch → optimizer:ANALYZE(测量基线) → dev+optimizer:FIX → reviewer:REVIEW(实时性回归) → tester:BENCHMARK(前后对比+回测)
```

### C：复杂跨模块（2~8 小时）
```
dispatch → dev:DESIGN(方案+影响面) → reviewer:DESIGN_REVIEW → dev:IMPLEMENT → reviewer:CODE_REVIEW → tester:BACKTEST(基准+回归) → tester:LIVE_TEST(串口实测) → CLOSE
```

### D：安全审计（2~6 小时）
```
dispatch → reviewer:SAFETY_AUDIT(E-Stop/限位/Flash/越界) → dev:FIX → reviewer:VERIFY → tester:ABUSE_TEST(越界输入/急停组合) → CLOSE
```

### E：架构引导新功能（2~8 小时）
```
assistant:ARCH(定位+对齐+冲突) → 任务计划 → 用户确认 → dev → reviewer → tester(回测基准+实测) → EVOLVE
```

## 工单消息模板

### 派 @dev
```markdown
@robot-platform-dev
🔧 **故障工单: #{ticketId}** [AUTO-DISPATCHED / ArmControl_HAL]

**现象**: {symptom}    **根因**: {rootCause}    **置信度**: {confidence}%
**涉及文件/函数**: {files}（见 architecture-memory.md 对应条目）
**必须遵守**: {实时路径/量纲约定/限位同步/Flash版本} 清单
**当前代码**:
```c
{brokenCode}
```
**修复要求**: {fixDescription}
**验收**: make 通过 + {backtest或实测项}
```

### 派 @reviewer
```markdown
@robot-platform-reviewer
🔍 **评审请求: #{ticketId}** [ArmControl_HAL]
**背景**: {symptom} 的修复    **文件**: {fileList}
**评审清单**:
- [ ] ISR/200Hz 路径无阻塞调用（cprint/DXL_Read）
- [ ] 数组/索引边界（joint<6, tokens<16, 点≤8）
- [ ] 量纲转换点（度/弧度/raw; 0~5 vs 1~6 索引）
- [ ] 限位/IK 同步、E-Stop 不被绕过
- [ ] Flash 布局变更已升 PID_FLASH_VERSION
- [ ] config-drift 清单（改的是生效点）
```

### 派 @tester
```markdown
@robot-platform-tester
🧪 **测试请求: #{ticketId}** [ArmControl_HAL]
**类型**: {backtest|串口实测|回归}
**回测**: python scripts/arm_log_lab.py backtest --log {log} --report
**回归**: --cases memory/fault-cases.md 全量
**实测步骤**: {reproSteps}（需硬件时先确认上电/急停/串口）
**预期**: {expectedResult}  **边界用例**: {edgeCases}
```

### 派 @optimizer
```markdown
@robot-platform-optimizer
⚡ **优化请求: #{ticketId}** [ArmControl_HAL]
**模块**: {module}  **现象**: {perfSymptom}
**当前指标**: {metrics}（主循环耗时/总线包率/ISR overrun 计数）
**目标**: {target}  **可疑代码**: {suspectCode}
```

## 升级规则

```
修复#1失败 → reviewer 加入复查根因
修复#2失败 → 扩大诊断范围（注入更多日志/补采数据），查次生问题
修复#3失败 → 标记 COMPLEX，切模板 C 全流程
仍失败    → 输出完整诊断报告交用户裁决；案例记入 memory/fault-cases.md
```
