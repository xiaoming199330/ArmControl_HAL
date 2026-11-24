# 故障案例库 = 回测黄金基准

> 协议见 `references/skill-evolution.md` §3.3。脚本可重放的案例用 `CASE` 块标记。
> `arm_log_lab.py backtest --cases memory/fault-cases.md` 全量回归。

## 案例索引

| ID | 现象 | 根因 | 状态 |
|----|------|------|------|
| FC-001 | S 停止提示语错误 | 先 Stop 后判 IsActive | OPEN（已知未修） |
| FC-002 | 修正范围配置漂移 | main.c 宏覆盖 arm_config.h | OPEN（待裁定） |
| FC-003 | IK 基准坐标集 | —（正向基准，防回归） | ACTIVE |

---

### FC-001：`S` 命令停止后提示语永远为 "Stopped."
- **位置**: `main.c` ProcessCommand case 'S'
- **现象**: 轨迹运行中发 `S`，期望 "Trajectory stopped."，实际永远 "Stopped."
- **根因**: `Trajectory_Stop()` 先执行，`Trajectory_IsActive()` 后判断，结果必为 0
- **修法**: Stop 前缓存 `uint8_t was_active = Trajectory_IsActive();`
- **回测**: 代码评审断言（非运动学案例）

### FC-002：自适应修正关节范围 drift
- **现象**: `arm_config.h` 声明 CORR_FIRST/LAST=1/3（J2~J4），实际 main.c 宏为 0/5（J1~J6）
- **影响**: 按文档改配置无效；修正实际作用于全部六轴
- **回测**: drift 清单常驻检查项

### FC-003：IK 基准坐标集（正向回归基准）
```CASE
id: FC-003
type: ik-roundtrip
points:
  - [150, 0, 150, 0]      # 正前方中位可达点
  - [100, 100, 100, 0]    # 左前中位
  - [0, 150, 120, -30]    # 左侧带俯仰
  - [180, -60, 90, 45]    # 右前低位大俯仰
  - [60, 40, 250, 0]      # 高位近身
expect: roundtrip_err_mm <= 1.0
note: 与固件 IK_FAIL_THRESHOLD=1.0mm 对齐；固件改动后必须全部通过
```
