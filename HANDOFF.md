# SO-ARM101 六轴机械臂 STM32 固件交接文档

> 项目：STM32F407ZGT6（启明欣欣 V4.1）+ 飞特 STS3215 30KG 12V 六轴机械臂
> 协议：SCS（非 Dynamixel Protocol 2.0）
> 最后更新：2026-09-07（内容截至 2026-09-04 会话：示教系统重写、QLC、悬浮托举、指令与几何配置化）

---

## 1. 硬件拓扑

| 总线 | 舵机 | USART | GPIO 方向 | 备注 |
|------|------|-------|-----------|------|
| 总线 A | J1/J2/J3 (ID 1/2/3) | USART3 PB10/PB11 | PE8 | 74HC126D 半双工 |
| 总线 B | J4/J5/J6 (ID 4/5/6) | UART5 PC12/PD2 | PE10 | 74HC126D 半双工 |
| 控制台 | — | USART6 PC6/PC7 | — | 115200 bps |

关键 GPIO：PC5 红 / PC4 绿 / PA5 蓝 LED；PB5 蜂鸣器；PE11 急停（低有效）；PG0-5 限位开关（上拉）。

---

## 2. 工具链与烧录

- Make：`C:\STM32Tools\make\make.exe`
- ARM GCC：`C:\STM32Tools\arm-gcc\bin\`
- OpenOCD：`C:\STM32Tools\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe`
- 烧录：`stm32flash -w build\ArmControl_HAL.bin -v -g 0x08000000 COM11`（BOOT0 拉高 + RESET）

编译：`make -j4`（项目根目录）。

---

## 3. 关键源码文件

| 文件 | 职责 |
|------|------|
| `Core/Src/main.c` | 主循环、控制台命令、运动调度、安全轨迹、校准、补偿 |
| `Core/Src/kinematics.c` | FK/IK、角度/原始值转换、世界偏移 |
| `Core/Inc/kinematics.h` | 几何参数（官方 URDF）、`JOINT_DIR[6]`、运行时 IK 参数 |
| `Core/Src/servo_driver.c/h` | STS3215 高层 API（`SERVO_*` 前缀） |
| `Core/Src/servo_bus_hal.c/h` | 74HC126 半双工方向切换（原 `dynamixel_hal`） |
| `Core/Src/scs_protocol.c/h` | SCS 帧协议（原 `dynamixel_protocol`） |
| `Core/Src/trajectory.c` | 七段 jerk-limited S 曲线（200Hz 插值） |
| `Core/Src/pid_flash.c/h` | Flash S11：PID + 补偿 + 速度 + 限位 + Home 零点 |
| `Core/Src/teach_store.c/h` | 示教文件 Flash 存储（双 256KB 区轮换、断电保留、断尾自愈） |
| `Core/Src/arm_commands.c/h` | 85 条命令集中注册表（与 `config/arm_config.json` 互为镜像） |
| `Core/Src/arm_params.c/h` | 运行时参数表（指针绑定单一生效点：LP/SP 文本、协议、API 三入口） |
| `Core/Src/arm_control_api.c/h` | 统一控制门面（MoveJoints / MoveCartesian / Stop / FkQuery） |
| `Core/Inc/arm_main_exports.h` | main.c 与接口层**唯一耦合点**（新增导出必须先在此登记） |

**分层（Core/Src 22 个 .c，约 11674 行）**：L6 应用 `main.c`(4552) → L5 接口（api/protocol/serial/CAN/runlog/commands）→ L4 算法存储（kinematics/trajectory/arm_params/pid_flash/teach_store）→ L3 `servo_driver` → L2 `scs_protocol` → L1 `servo_bus_hal` → L0 CubeMX 外设。算法层与硬件无关，可脱离 MCU 做单元测试。

**配置文件**：`config/arm_config.json` 是上位机/文档镜像（85 条命令 + `commands_meta` 字母/参数注释 + `mechanical` 官方 URDF 几何 + servo_offsets / joint_limits / predefined_poses），固件运行时**不解析**它，改固件默认值后需同步。

---

## 4. 运动学模型（官方 URDF）

- `ROBOT_BASE_H = 65.30`
- `ROBOT_J2_OFF_Y = 31.32`，`ROBOT_J2_OFF_Z = 54.80`
- `ROBOT_L_UPPER = 116.00`，中位偏角 `PSI_UPPER0 = +13.97°`
- `ROBOT_L_FORE = 135.00`，中位偏角 `PSI_FORE0 = +87.79°`（水平）
- `ROBOT_L_WRIST = 62.50`，`ROBOT_L_TOOL = 105.30`，`ROBOT_L_DISTAL = 167.80`
- 零点：`SERVO_OFFSET[6]` 运行时数组，`Kinematics_SetZeroRaw()` 写入
- 方向：`JOINT_DIR[6] = {-1, +1, +1, +1, +1, +1}`（J1 顺时针；J2-J4 前倾）

---

## 5. 关节限位（2026-09-02 深夜最终版：一律 raw 行程量，严禁角度推导）

权威限位在 **raw 0-4095 空间**（`JOINT_MIN_RAW/MAX_RAW[6]`，wrap-around 判断）；角度版仅显示。

**校准生效后的实际限位规则**（`apply_calib_limits()`）：

| 关节 | 限位来源 | 范围（舵机度显示） |
|------|----------|--------------------|
| J1/J6 | Phase2 实测机械行程 | 全行程安全 |
| J2/J4 | Phase2 实测机械行程 | 不跨 0，直接用 raw min/max |
| J3 | **挡块行程量写死**（`J3_LIMIT_SERVO_BACK/FRONT`） | **合法弧 = 177°→360°→0°→10°（跨 0 包裹）；禁入区 (10,177)°** |
| J5 | 硬编码 | 舵机度 180~340（避开 77°~174° 卡区，永不被校准覆盖） |

**J3 必须这样理解（用户实测挡块 + P 0 卡死 175° 佐证）**：
- 前挡块 servo 10°（raw 114），后挡块 servo 177°（raw 2013）；合法运动区是跨越 0/360 的包裹弧。
- **servo (10,177) 是禁入区，不是允许区**！P 0 目标 J3=133° 落入禁入区 → 途中撞后挡块卡死。
- Phase2 实测 raw 23~4083 是整圈绕行极值，**不是挡块，J3 禁用**。
- history：此前"准全旋转 353°/死区[7,10]"与"WORK_RANGE 关节角 -177~+10"两版理解均错误，已全部废除。

**配套防线（必须保持）**：
- `fk_path_z_ok()` 每个插补采样点逐关节 `raw_in_range()` —— 端点合法≠路径合法（P 0 就是端点合法路径穿禁入区的典型）。
- `kinematics.c IK_SetJointLimitsRaw()` 按 raw 正向弧连续展开换算（勿回退"分支搜索"旧实现，包裹弧会掉到 [-466,-273] 错误分支）。
- `clamp_raw()` 就近边界（比较两侧正向距离取近者）。
- 预设位姿 J3 值全部落在合法弧内（verify_poses.py 校验）。

**核心教训（事故链）**：① 触地事故 —— 限位用角度/关节角推导，跨 0/360 重复、辨不了方向；② P 0 卡死 —— J3 目标落入 (10,177) 禁入区仍被放行。**限位定义与检查必须全程 raw 行程量。**

未校准时用编译默认（舵机度）：J1 0~360 / J2 58~264 / J3 合法弧 177~360~0~10 / J4 83~289 / J5 180~340 / J6 0~360。

---

## 6. 控制台常用命令

| 命令 | 作用 |
|------|------|
| `n` | 扫描总线 A/B |
| `i` | 初始化舵机（Mode=0、扭矩开、限位） |
| `s` | 状态 |
| `t` | 扭矩开关 |
| `a J deg` | 单关节绝对运动（J=0..5） |
| `p 0~3` | 预定位姿（0=HOME 折叠收纳，1=FOLD，2=READY，3=WAVE1） |
| `g x y z pitch` | IK 安全运动到目标（mm，pitch=工具方向角，0=上/90=水平前/180=下） |
| `f` | 当前关节角的 FK |
| `c` | 两阶段校准：Phase1 按 `0` 记录 Home 零点，`1-6` 单关节；`n` 进入 Phase2；`s` 保存 |
| `z` | 所有舵机回 raw 2048（舵机中位，不等于运动学 Home） |
| `L` | 显示/设置限位 |
| `h` | 帮助 |

**示教（2026-09-04 重写，取代旧的 8 点 `rec` 两段式）**：

| 命令 | 作用 |
|------|------|
| `teach` | **默认悬浮托举**：扭矩 ON，按姿态写 J2/J3/J4 的 Torque Limit 托住臂；同时 50Hz 自动连续采样（无需按键，含 J6 夹爪） |
| `teach free` | 旧的全松模式（扭矩 OFF 自由拖） |
| `teach exit` | 停录、恢复全限矩 4095 与正常速度 |
| `teach k [k2 k3 k4]` | 查看/设置悬浮增益（默认 1.2 / 1.6 / 1.0，0~8） |
| `teach s <1~300%>` | 悬浮整体松紧旋钮（默认 100） |
| `save [名字]` | 把临时录音存入 Flash（≤11 字符，重名覆盖，省略自动 recN），存后可再 `teach` 录下一条 |
| `list` / `files` | 列出所有已存文件（序号/名字/样本数/时长） |
| `del <名字\|序号>` | 删除（写墓碑，不立即回收空间） |
| `compact` | 整理 Flash 回收空间（数秒，CPU 冻结属正常） |
| `play` / `play <名字\|序号> [loop]` | 回放临时录音 / 指定文件，支持循环 |
| `rec clear` / `rec list` | 丢弃 / 查看临时录音（不影响已存文件） |

**循环**：`QL n r [v]`（每点停稳 500ms 采样、配合 `qd` 驻留，用于测重复精度）；`QLC n r [v]`（**新增**，段间不停稳/不采样/不驻留的连续流水线循环，消除"点头"，用于演示）。

---

## 7. 安全轨迹（`safe_seq_plan`，2026-09-02 重构）

大范围运动不再直接关节插补，而是拆成经 FK 校验的路点序列：

1. 优先尝试直达（`fk_path_z_ok` 通过）。
2. 两段经高举：cur → rise（工具竖直朝上，z≈486）→ tgt。
3. 折叠展开链：抬肩（保持肘/腕）→ 高举 → 目标（3 段）。
4. 折叠收拢链（目标 J3 关节角 < -60）：高举 → 半折叠（工具保持朝上）→ 肘/腕到目标构型（肩仍高）→ 降肩到目标（4 段）。
5. 失败：尝试 IK 悬停点（x/y 减半、z=260）。

**全部路点/位姿以"相对 Home 零点的关节角"定义**（`joint_to_servo_deg()` 换算），随 Phase1 zero 自适应；折叠判定用 `servo_to_joint_deg()` 关节角。

**关键参数**：
- 爪尖最低 `TRAJ_Z_MIN_MM = 15`（z 判定扣除 WORLD_OFFSET，基座坐标系）
- 立柱圆柱 `TRAJ_COL_R_MM = 25`（**fabsf(rho)**，含臂后折方向），`TRAJ_COL_Z_MM = 120`
- 高举过渡关节角 `RISE_J2/J3/J4 = -40/-10/-60`

**预设位姿（关节角）**：HOME `{0,-60,-150,10,0,0}`（后折收纳，爪尖 z≈65）/ FOLD `{0,-50,-160,20,0,0}` / READY `{0,0,-30,40,0,0}`（前伸待命）/ WAVE1/WAVE2 `{±45,-30,-80,50,0,0}`；`refresh_pose_servo()` 在开机与校准保存后刷新。

**扭矩策略**：`SERVO_InitJoint` 结尾 Torque ON（上电抱闸，防重启失力砸桌）；`c` 校准入口释放全部扭矩、出口恢复；`W` 挥动准备段走安全序列，完成后由 `safe_seq_service()` 触发（`swing_resume_pending` 机制）。

---

## 8. 已验证的物理事实

- Home 锚定后 `f`：x≈358±2 mm，z 与模型差 3~8 mm。
- `g 350 0 200 0` 实测 x≈350±2 mm。
- J5 卡区 77°~174°：必须永远避开。
- J3 机械硬限位 10~177：算法生成 240 会卡死，不是结构禁区，是算法超界。

---

## 9. 已知风险与操作规范

1. **J2/J3/J4 限位**：校准后由 WORK_RANGE（相对 Home 关节角）决定，J3 前折方向硬限 +10°——这是防触地的最后防线，不要放宽。
2. **J5 卡区**：`z` 命令会让 J5 回 2048，若当前 J5 在卡区可能堵转。验证期慎用 `z`。
3. **上电即抱闸**：开机/`i` 初始化后各关节带扭矩保持当前位置；手动摆位前先 `t J 0`（或进 `c` 校准自动释放）。
4. **位姿常量**：新增/修改预设位姿必须用关节角（`POSE_*_J`），经 `verify_poses.py` FK 验证后才能烧录。
5. **烧录前**：BOOT0 拉高 → RESET → 烧录 → BOOT0 拉低 → RESET。
6. **改舵机 ID**：使用外部 ESP32 工具，STM32 的 `U` 命令已删除。
7. **悬浮示教**：进入前先用手扶住臂（限矩从低到高缓升，不会"抓"人）。增益 `k2/k3/k4` 尚未标定——松手慢慢下沉→调大；拖动费劲/回弹→调小；也可直接用 `teach s` 整体缩放。
8. **示教回放不做碰撞过滤**：`play` 按录制样本原样复刻物理动作，**不复跑 FK/桌面碰撞检查**，回放前需人工确认轨迹安全（存在撞桌可能）。
9. **Flash 操作会冻结 CPU**：`compact` 与首次 `save` 会擦除 2×128KB 扇区，期间数秒无任何输出，属正常现象。
10. **重力补偿只在示教生效**：普通运动路径的 `gravity_comp_j2()` 前馈已删除，`gc.k` 降级为占位参数（保留仅为不动参数表结构）。唯一形态是 teach 悬浮托举。

---

## 10. 待办 / 下一步（2026-09-04 会话后的顺序）

- [ ] **烧录 + 标定悬浮**：烧 `bin 127184 B`，用 `teach k` / `teach s` 边拖边调，目标手感 = 松手不坠、拖动轻快。
- [ ] **实测示教闭环**：`teach` 拖一段（含夹爪张合）→ `save demo1` → 再录一条 → `list` → 断电重启确认文件仍在 → `play demo1` 看整体回放。
- [ ] **实测 QLC**：验证连续循环是否真的无点头；若拐角还想更利落，评估"拐点不收尾"选项。
- [ ] **修量纲漂移**：多处注释与帮助文本仍写 XL320 口径"舵机度 0~300"（`arm_control_api.h:13`、`arm_main_exports.h:9/50`、`arm_commands.c:63/99`、`main.c:2095/2681/3074`），实际为 raw 0~4095 / 0~360°，`h` 帮助里的那两处会直接误导使用者。
- [ ] **清理死文件**：`config/arm_config.h` 已无任何 include，确认后删除或标注废弃（避免与 `arm_params.c` 混淆）。
- [ ] **JSON 剩余旧段同步**：`servo_config` 的 raw 1023 / control_mode 2 等 XL320 残留。
- [ ] **上位机建模**：按 `arm_config.json → mechanical` 建运动学模型；注意 OFFSET 每台臂不同，启动时发 `s` / `L` 拉运行时值，不要硬编码。

---

## 11. 交接断点（2026-09-04 会话结束状态）

| 项 | 状态 |
|----|------|
| 固件产物 | `build/ArmControl_HAL.bin` **127184 B**（text 126288 / data 884 / bss 53452），编译通过 |
| 是否已烧录实测 | **否**——9-04 下午主要在做功能实现，最后一版尚未上机 |
| 当日回退项 | 基于 LeRobot 的 `IK_SolveContinuous` + `TRAJ_MAX_RELATIVE_DEG` + `AP_IK_JUMP_DEG`/`AP_SC_MAX_REL_DEG` **已全部回退**（上机恶化：unreachable 增多、J2 顶桌、抖动加剧） |
| 保留的调优参数 | `TRAJ_FOLLOW_MAX_ERR_DEG=30°`、`OVER_SAMPLES=5`、`RESUME_DEG=10°`、`TRAJ_APPROACH_FRAC=0.05`、`TRAJ_APPROACH_SPEED=30` |
| 重力补偿 | `gc.k=0`；仅 teach 悬浮托举一种形态 |
| pitch 工作空间 | (200,200,300) 可行约 77°~100°；基准：0=竖直向上、90°=水平、180°=向下 |
| 备份文件 | 工作区目录 `backup_main_pre_teach2.c`、`backup_servo_driver_pre_teach2.{c,h}` |

**接手后第一件事**：烧录当前 bin，扶住臂发 `teach` 试拖，按手感用 `teach s` / `teach k` 标定，再走完整示教闭环。
