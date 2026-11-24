# 诊断分析规则（Analysis Rules）

> 串口输出 → 归一化 → 签名匹配 → 根因评分 → 排序输出。
> 自动分析：`python scripts/arm_log_lab.py analyze --log <file>`（内置本文件全部规则）。
> 新规则追加格式见文末；沉淀后同步 `memory/fault-cases.md`。

## 根因检测规则

### RCA-BUS-NO-RESPONSE：总线无应答
```yaml
severity: CRITICAL
signature:
  - "found 0 servo(s)"           # 或 'No servo response'
  - "LOOPBACK OK"                # 有此行 → 硬件链路 OK，问题在舵机侧
diagnosis:
  rootCause: 舵机供电/DATA接线/ID/波特率（loopback OK 排除 126 与走线）
  nextStep: 查舵机 5V 供电、DATA 线、用 'n' 重扫、核对 ID 冲突
  files: [feetech_hal.c, feetech_protocol.c]
```

### RCA-BUS-LOOPBACK-FAIL：回环失败
```yaml
severity: CRITICAL
signature: ["LOOPBACK FAILED"]
diagnosis:
  rootCause: TX→74HC126→RX 硬件链路断（焊接/126供电/方向脚 PE10）
  files: [feetech_hal.c]
```

### RCA-DXL-READ-FAIL：读失败蔓延
```yaml
severity: MAJOR
signature:
  - "read failed" 或 "J%d=ERR" 或 "FK feedback failed"
  - count: ">= 3 次同关节"
diagnosis:
  rootCause: 单舵机掉线/超时（电源跌落、线序、ID 冲突）或总线竞争
  nextStep: 'd' 诊断 + 'n' 扫描 + 检查 200Hz 期间读频率
```

### RCA-IK-UNREACHABLE：IK 不可达
```yaml
severity: MAJOR
signature: ["IK: unreachable!"]
diagnosis:
  rootCause: 目标超出臂展 226mm / 世界偏移设错 / 限位钳制致 CCD 发散
  nextStep: arm_log_lab.py backtest 同坐标复现；查 C 偏移；查 L 限位
  files: [kinematics.c]
```

### RCA-IK-LIMIT-REJECT：IK 解超限
```yaml
severity: MINOR
signature: ["solution out of joint limits!", "Move cancelled (safety)"]
diagnosis:
  rootCause: 解存在但越限位；L 命令限位过紧或未同步 IK_SetJointLimits
  files: [main.c cmdGoto, kinematics.c jlim]
```

### RCA-IK-FK-VERIFY：FK 验证超差
```yaml
severity: MAJOR
signature: ["IK error too large", "move cancelled"]
diagnosis:
  rootCause: CCD 收敛到次优解(best_err 1~5mm) / θ6 钳制反噬位置
  nextStep: backtest 回放该坐标查看 5 起点收敛细节
```

### RCA-CLAMP：目标被限位钳制
```yaml
severity: MINOR
signature: ["joint(s) clamped to safe limits"] 或 ["(min limit)", "(max limit)"]
diagnosis:
  rootCause: 命令目标超出当前限位表
  note: 运行时限位(L)掉电丢失；编译期默认见 main.c JOINT_MIN/MAX
```

### RCA-CORR-OSC：修正振荡
```yaml
severity: MAJOR
signature:
  - "OVERSHOOT" count: ">= 4 次单关节"
  - "auto-tune: D_Gain" 出现且仍振荡
diagnosis:
  rootCause: 修正增益过高 / 舵机死区 / P-Gain 被 stall 逻辑推太高
  nextStep: A g J 降初增益；查 stall_count 是否同步上升（P/D 互相打架）
```

### RCA-CORR-STALL：修正停滞
```yaml
severity: MAJOR
signature: ["STALL" count >= 3, "TIMEOUT"]
diagnosis:
  rootCause: 负载/摩擦力矩超 P-Gain 能力 / Moving Speed 过低 / 机械卡阻
  nextStep: 查 T 温度、V 速度、手动 w 扫描该关节
```

### RCA-POS-NOT-REACHED：到位偏差
```yaml
severity: MAJOR
signature:
  - "Move complete" 后 pv 反馈稳定值与 To: 目标差 > 2°
  - "Adaptive correction STOP: max" 或缺失修正触发
diagnosis:
  rootCause: 重力下垂(K 未调) / 修正被关(A e 0) / 修正停滞
  nextStep: backtest --log 自动输出偏差收敛报告
```

### RCA-LOOP-STUCK：循环状态卡死
```yaml
severity: MAJOR
signature: ["Another loop is active"] 或 多点循环长时间无 "Point %d reached"
diagnosis:
  rootCause: 到位容差 ±2° 达不到(负载/修正竞争) / 读失败顶替掩盖
  files: [main.c cmdMultiPointService, cmdMultiPointTargetReached]
```

### RCA-FLASH-INVALID：持久化回退
```yaml
severity: MINOR
signature: ["No valid PID/correction/speed config"]
diagnosis:
  rootCause: 版本升级旧记录失效(设计行为) / 范围校验否决 / 从未 F 保存
  nextStep: 重新调参后发 F 一次
```

### RCA-ISR-OVERRUN：主循环过慢
```yaml
severity: CRITICAL
signature: ["ISR OVERRUN"] 或轨迹时间明显长于 t_total
diagnosis:
  rootCause: 主循环存在阻塞操作（大段 cprint/长超时 DXL_Read/多点服务高频读）
  nextStep: 200Hz 路径审查 + 反馈降频 + 分频采样日志
```

### RCA-CONFIG-DRIFT：配置不生效
```yaml
severity: MINOR
signature: [用户报告"改 arm_config.h 无效"]
diagnosis:
  rootCause: arm_config.h 为聚合参考，实际值双轨定义于各 .c
  nextStep: 查 architecture-memory.md 第4节 drift 清单改生效点
```

## 置信度评分

```
score = 签名命中数×20 + 时序相邻(5s内)+10 + 含ERROR关键词+10 - 缺失预期日志×10
0~49 低（需补充日志/注入后复现）  50~79 中  80+ 高（直接派单修复）
```

## 多规则排序

置信度 > 严重度(CRITICAL>MAJOR>MINOR) > 证据数 > 日志新近度

## 新规则追加格式（技能进化用）

```markdown
### RCA-XXX-YYY：名称          <!-- v{version} 新增, 来源: run-journal#日期 -->
​```yaml
severity: ...
signature: [...]
diagnosis: { rootCause, nextStep, files }
​```
```
