# 嵌入式日志注入模板（Log Patterns）

> 自动注入用 `scripts/c_log_injector.py`；手工注入照本文件模板。
> **铁律**：ISR 内禁止 `cprint`/舵机通信；200Hz 实时路径必须分频采样；`cprint` 是阻塞发送（100ms 超时），调试输出会拖慢主循环。

## 目录
1. [通用函数模板](#1-通用函数模板)
2. [轨迹/实时路径采样](#2-轨迹实时路径采样)
3. [总线与协议层](#3-总线与协议层)
4. [自适应修正 FSM](#4-自适应修正-fsm)
5. [命令解析与状态机](#5-命令解析与状态机)
6. [ISR 安全计数器](#6-isr-安全计数器)
7. [免费日志源](#7-免费日志源)

## 1. 通用函数模板

### 函数入口/出口

```c
// BEFORE:
static void cmdMove(float target[6], float v_max, float accel) {
    swing_active = 0;

// AFTER:
static void cmdMove(float target[6], float v_max, float accel) {
    cprint("[DIAG] cmdMove ENTRY | tgt=%d,%d,%d,%d,%d,%d v=%d a=%d\r\n",
           (int)target[0],(int)target[1],(int)target[2],
           (int)target[3],(int)target[4],(int)target[5], (int)v_max, (int)accel);
    swing_active = 0;
```

### 条件分支

```c
if (planar_dist > ARM_MAX_REACH + 0.5f) {
    cprint("[DIAG] IK BRANCH=UNREACHABLE | planar=%d reach=%d\r\n",
           (int)planar_dist, (int)ARM_MAX_REACH);
    return 0;
}
```

### 数据变换（角度/量纲转换点）

```c
deg[j] = rad_to_servo_deg(rad[j], j);
cprint("[DIAG] IK XFORM J%d | rad=%d.%03d deg=%d.%d\r\n", j+1,
       (int)rad[j], (int)(fabsf(rad[j])*1000)%1000, (int)deg[j], (int)(deg[j]*10)%10);
```

## 2. 轨迹/实时路径采样

**200Hz 路径全量打印 = 每秒 200 行 × 6 关节，必然拖垮主循环。必须分频：**

```c
// main.c 200Hz 更新块内：每 40 次(5Hz)采样一次
static uint16_t diag_div = 0;
if (++diag_div >= 40) {
    diag_div = 0;
    cprint("[DIAG] TRAJ t=%lu | a=%d,%d,%d,%d,%d,%d\r\n",
           (unsigned long)traj_tick,
           (int)angles[0],(int)angles[1],(int)angles[2],
           (int)angles[3],(int)angles[4],(int)angles[5]);
}
```

### Trajectory_Plan 规划摘要（低频，可全量）

```c
// trajectory.c Trajectory_Plan 末尾:
cprint("[DIAG] PLAN max_d=%d.%d v_peak=%d a=%d tj=%d.%03d ta=%d.%03d tv=%d.%03d ticks=%lu\r\n",
       (int)traj.max_d, (int)(traj.max_d*10)%10, (int)traj.v_peak, (int)traj.a,
       (int)traj.tj, (int)(traj.tj*1000)%1000, (int)traj.ta, (int)(traj.ta*1000)%1000,
       (int)traj.tv, (int)(traj.tv*1000)%1000, (unsigned long)traj.total_ticks);
```

## 3. 总线与协议层

### DXL 调用追踪（低频操作可全量；Read/Write 高频时配合计数器）

```c
// feetech_protocol.c DXL_Read 失败后:
if (st != HAL_OK) {
    cprint("[DIAG] DXL_READ FAIL | bus=%d id=%d addr=%d st=%d\r\n", bus, id, addr, st);
}
```

### 错误标志计数（ISR 安全，主循环定期打印）

```c
// feetech_hal.c DXL_FlushErrors 内:
static uint16_t diag_ore_count = 0;
if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) diag_ore_count++;
```

## 4. 自适应修正 FSM

```c
// main.c 修正块内，error 计算后:
cprint("[DIAG] CORR J%d | err=%d.%d gain=%d%% osc=%d stall=%d round=%d\r\n",
       j+1, (int)error, (int)(fabsf(error)*10)%10, (int)(corr_gain[j]*100),
       oscillation_count[j], stall_count[j], correction_count);

// 自动调参点:
cprint("[DIAG] AUTOTUNE J%d | D->%d (osc damp)\r\n", j+1, pid_d[j]);
```

## 5. 命令解析与状态机

```c
// ProcessCommand 入口:
cprint("[DIAG] CMD | line=\"%s\" ntok=%d\r\n", line, ntok);

// 状态转换点（多点/摇摆/循环）:
cprint("[DIAG] STATE | multi idx=%d legs=%u waiting=%d\r\n",
       multi_point_index, multi_point_legs_left, multi_point_waiting);
```

## 6. ISR 安全计数器

中断里只改计数器/标志，主循环打印：

```c
// TIM2 callback 内:
static volatile uint16_t isr_overrun = 0;
if (traj_do_update) isr_overrun++;   // 上一次更新还没被主循环消费 → 主循环太慢
traj_do_update = 1;

// 主循环定期:
if (isr_overrun) { cprint("[DIAG] ISR OVERRUN x%d (main loop too slow)\r\n", isr_overrun); isr_overrun = 0; }
```

## 7. 免费日志源

无需注入即可用的运行数据：

| 来源 | 命令 | 用途 |
|------|------|------|
| 周期角度反馈 | `pv 100` | 回测输入、到位偏差曲线 |
| 周期末端位姿 | `pf 100` | FK 链验证 |
| 修正过程输出 | （默认开启） | 收敛/振荡分析 |
| 状态表 | `s` / `T` / `G` / `A` / `L` | 快照取证 |
| 总线诊断 | `d` | 分层定位 |

采集：`python scripts/arm_log_lab.py capture --port COM13 --out logs/session.txt`
