# 串口指令参考（Command Reference）

> 控制台：USART6 @115200。完整说明另见工程 `config/arm_config_readme.md`（改命令时需同步）。

## 1. 索引与量纲约定（极易踩坑）

- **运动类命令** J 索引 = **0~5**（J1~J6）：`t a r p z w`
- **调参类命令** J 索引 = **1~6**：`K I D N` 与 `A g J v`
- 角度 = 舵机度 0~300（中位 150）；空间坐标 = mm；pitch = deg
- 轨迹速度 = deg/s（5~300，默认 30）；`V` 速度 = STS3215 raw 0~2047
- 双字母命令由 `tokens[0][1]` 区分，大小写均兼容：`pv pf qv qd qg QL BG`

## 2. 指令全集

### 扫描与状态
| 指令 | 参数 | 作用 |
|------|------|------|
| `n` | - | 扫描双总线（Ping 0~252） |
| `i` / `i J` | J=0~5 | 初始化全部/单个舵机（EEPROM 关节模式 + 重启 + RAM PID/速度/力矩） |
| `s` | - | 六关节角度/Moving/温度表 |
| `d` | - | Bus B 分层诊断（loopback→UART 标志→Ping ID4） |
| `h` | - | 帮助 |

### 单关节
| 指令 | 参数 | 作用 |
|------|------|------|
| `t J 0/1` | J=0~5 | 扭矩开关（开时蜂鸣 20ms；E-Stop 下拒绝） |
| `a J deg` | 0~300 | 单关节直接设角（钳限位） |
| `r J raw` | 0~1023 | 单关节 raw 写（按限位换算钳制） |
| `p J` | J=0~5 | 读 raw + 角度 |
| `pv ms` / `pf ms` | 10~60000，0=停 | 周期输出六关节角度 / 末端 XYZ+Pitch（**天然日志源**） |
| `z J` / `z a` | - | 回零到舵机 0°（⚠ 非运动学零位，慢速等待到位） |
| `w J` | J=0~5 | 限位内 min→mid→max→mid 扫描（speed=80，每步 1.5s） |

### 协调运动
| 指令 | 参数 | 作用 |
|------|------|------|
| `m a1..a6 [v]` | 度, deg/s | 六关节 S 曲线协调运动，a=v×2 |
| `P 0~4` | - | 预设位姿 0=HOME 1=FOLD 2=READY 3=WAVE1 4=WAVE2（v=30,a=60） |
| `W [n]` | 默认3 最大20 | WAVE1↔WAVE2 往返 n 次，结束回 READY |
| `B a1..a6 b1..b6 rounds [v]` | rounds=0 无限 | 关节空间 A↔B 循环 |
| `BG x1 y1 z1 p1 x2 y2 z2 p2 rounds [v]` | - | 空间 A↔B 循环（先双点 IK，失败不启动） |
| `S` | - | 停止轨迹与全部循环/修正 |

### 多点循环
| 指令 | 参数 | 作用 |
|------|------|------|
| `q n j1..j6` | n=1~8 | 保存关节点（逐轴 clamp_angle） |
| `qg n x y z [pitch]` | n=1~8 | IK 解算存空间点；IK失败/超限/FK>5mm **不覆盖**原点 |
| `qv n v` | 5~300 | 第 n 段（点 n → 下一点）独立速度 |
| `qd n ms` | 0~60000 | 点 n 到位后停留（六轴均入 ±2° 才开始计时） |
| `QL count rounds [v]` | count=2~8 | 顺序循环已存点；rounds=0 无限 |

### 运动学
| 指令 | 参数 | 作用 |
|------|------|------|
| `f` | - | FK：读六轴实际角 → x y z pitch |
| `g x y z [pitch]` | mm, deg | IK 移动；不可达/超限/FK验证>5mm 均取消 |

### 安全与标定
| 指令 | 参数 | 作用 |
|------|------|------|
| `L` / `L J min max` | - | 查看/设置关节限位（设置时同步 IK_SetJointLimits） |
| `C` / `C dx dy dz` | mm | 查看/设置世界偏移（offset = 实际 - 指令） |

### 调参与持久化
| 指令 | 参数 | 作用 |
|------|------|------|
| `G` / `G k` | 0~0.5 deg/mm | 查看自适应&PID状态 / 设 J2 重力前馈 K |
| `K/I/D [J] [val]` | J=1~6, 0~254 | 查看/设置 P/I/D（写舵机 RAM + 缓存） |
| `N J P I D` | J=1~6 | 一次设整组 PID；`N J` 查看 |
| `V [0~2047]` | - | 查看/设置六舵机 Moving Speed（立即生效） |
| `A` / `A e/t/r/g` | - | 自适应修正：开关/阈值0.5~10°/轮数1~20/初增益0~100% |
| `R` | - | 重置自适应状态并对上次目标重触发修正 |
| `F` | - | **保存** PID+补偿+速度到 Flash Sector 11（整扇区擦写，勿频繁） |
| `X` | - | 应用固件默认 PID（不擦 Flash） |
| `T [J]` | - | 温度查询 |

## 3. 典型调试会话（可作为采集脚本模板）

```text
# 到位精度回测采集
> pv 100                # 100ms 周期反馈，开始记录
> m 150 100 110 150 100 150 60
< ... Move started / Move complete. Adaptive correction starting...
<   J2 corr #1: actual=96 tgt=100 -> 102 (err=4, gain=50%)
<   Adaptive correction DONE: all within +/-2 deg (2 rounds)
> pv 0                  # 停止反馈
> f                     # FK 末端验证

# 总线故障分层
> d                     # loopback OK + ping fail → 舵机供电/DATA_B 接线
> n                     # 重新扫描确认
```

## 4. 输出签名（分析规则匹配用）

| 输出片段 | 含义 |
|----------|------|
| `Move started: v=.. a=..` / `From:` / `To:` | 运动开始三行 |
| `Move complete. Adaptive correction starting...` | 轨迹结束，修正启动 |
| `J%d OK / corr #n / OVERSHOOT / STALL / TIMEOUT` | 修正过程 |
| `Adaptive correction DONE / STOP` | 修正收尾 |
| `IK: unreachable!` / `solution out of joint limits!` / `IK error too large` | IK 三类拒绝 |
| `J%d: %d -> %d (min/max limit)` | 限位钳制 |
| `WARNING: %d joint(s) clamped` | 目标超界被钳 |
| `E-STOP active!` / `E-STOP! Trajectory aborted.` | 急停 |
| `PID/correction/speed config saved/loaded/FAILED` | Flash 操作 |
| `Point %d reached; dwell %lu ms` / `Multi-point loop done` | 多点循环 |
| `Another loop is active. Use S first.` | 状态冲突 |
