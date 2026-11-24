# 技能进化协议（Skill Evolution）

> 目标：技能越用越聪明。每次任务后执行学习管道；记忆落盘，跨会话生效。

## 1. 学习管道

```
任务完成 / 故障修复
  │
  ├─ Step1 写运行日志记忆 → memory/run-journal.md（追加，不覆盖）
  ├─ Step2 提取经验分类:
  │     新故障签名?      → references/analysis-rules.md（追加 RCA 规则）
  │     新故障链/新坑?   → references/fault-module-map.md（追加案例）
  │     新回测基准?      → memory/fault-cases.md（追加黄金案例）
  │     新注入技巧?      → references/log-patterns.md
  │     新命令/数据流?   → references/command-reference.md / architecture-memory.md
  │     角色流程改进?    → 建议同步 robot-platform-dev/reviewer/tester/optimizer
  │     架构变化?        → references/architecture-memory.md（刷新 + 版本标记）
  ├─ Step3 生成升级补丁（append / modify / new-file）
  ├─ Step4 应用补丁，新版本标记 <!-- v{x} 新增, 来源: 日期+任务 -->
  └─ Step5 一致性验证（见第 6 节清单）
```

## 2. 运行日志记忆协议（run-journal）

每次会话结束向 `memory/run-journal.md` 追加：

```markdown
## YYYY-MM-DD HH:MM | TASK-XXX | {标题}
- **请求**: {用户原始提示词摘要}
- **涉及模块**: {模块列表}
- **故障/现象**: {如有}
- **根因**: {诊断结论 + 置信度}
- **修复**: {文件:函数 变更摘要}
- **验证**: {make 结果 / backtest 结论 / 实测结果}
- **遗留**: {未解决问题 + 下次建议}
- **进化**: {本次追加到哪些 references/案例}
```

**会话开始纪律**：先读 `memory/run-journal.md` 最近 5 条 + `memory/fault-cases.md` 相关案例，再动手。相似故障优先复用历史根因与修法。

## 3. 回测协议（Backtest）

回测 = 用历史/采集的串口日志在 Python 运动学模型上回放，验证修复不引入劣化。

### 3.1 回测类型

| 类型 | 命令 | 时机 |
|------|------|------|
| 会话回测 | `arm_log_lab.py backtest --log logs/x.txt --report` | 每次实测后 |
| 基准回归 | `arm_log_lab.py backtest --cases memory/fault-cases.md` | 修复合并前必跑 |
| 坐标集验证 | `arm_log_lab.py backtest --ik x y z [pitch]` | IK 相关改动后 |

### 3.2 通过判据

- FK/IK 往返误差 ≤ 1.0 mm（与固件 `IK_FAIL_THRESHOLD` 一致）
- 规划时间参数与 S 曲线三档分支计算一致
- 到位偏差序列：修正后终值 ≤ corr_threshold（默认 2°）
- 回归案例：全部 PASS，无新增 FAIL

### 3.3 黄金案例（fault-cases.md）入库标准

1. 真实发生过的故障，有采集日志或精确参数
2. 含：现象 / 输入 / 期望行为 / 修复前错误行为 / 根因 / 修复 commit 摘要
3. 可被脚本自动重放（日志文件路径或内联坐标/角度参数）

## 4. 升级规则

### Append（追加条目到新列表）
用于：新 RCA 规则、新故障案例、新 drift 条目、新命令行。

### Modify（更新既有内容）
用于：函数签名变化、默认参数调整、生效点迁移。修改处加版本标记。

### New-File（新维度）
用于：新增子系统（如未来加夹爪/力控）需要独立 reference。在 SKILL.md References 表登记。

### 版本标记
```markdown
### 条目名 (v2.1 新增)      <!-- 来源: 2026-07-30 TASK-005 -->
### 旧条目 (DEPRECATED v2.1) <!-- 原因: ... -->
```

## 5. 进化触发器速查

| 触发 | 目标文件 |
|------|---------|
| Bug 修复（新签名） | references/analysis-rules.md + memory/fault-cases.md |
| Bug 修复（新坑） | references/fault-module-map.md |
| 新串口命令 | references/command-reference.md + architecture-memory.md |
| 算法参数/流程变化 | references/architecture-memory.md + 重建回测基准 |
| 实测发现回测判据不足 | scripts/arm_log_lab.py 判据 + skill-evolution.md §3.2 |
| 硬件变更（舵机/总线/引脚） | references/architecture-memory.md §0 |
| 角色协作卡点 | references/dispatch-matrix.md（必要时同步角色 skill） |

## 6. 一致性检查清单（每次进化后）

- [ ] 同一事实只存在于一处（references 间无重复）
- [ ] 与工程代码实际行为一致（拿不准就 grep 验证）
- [ ] SKILL.md < 500 行；新增引用已在 References 表登记
- [ ] 新内容带版本标记与来源
- [ ] 回测基准仍可全部通过
- [ ] run-journal 已追加本次记录

## 7. 命令

```bash
@armcontrol-hal-assistant --evolve --from-task TASK-001
@armcontrol-hal-assistant --evolve --pending
@armcontrol-hal-assistant --evolve --apply
@armcontrol-hal-assistant --evolve --refresh-arch
@armcontrol-hal-assistant --journal      # 查看运行日志记忆
```
