# arm_sdk — 六轴机械臂上位机 SDK（v2.2.0）

STM32F407 + Feetech STS3215×6 六轴机械臂的 **上位机开发包**：三条链路 + 主从遥操 + 遥操示教 + CLI。

* **Linux 优先**（Windows 亦可），核心**零依赖**（纯标准库）
* 网口实测 **p50 1.61 ms**，50 Hz 遥操有 15× 余量
* 内置**内存模拟器**（`sim://`）⇒ 没有硬件也能开发、能跑 CI
* 228 项离线断言全绿（Python），不需要任何硬件

> **v2.1.0（2026-09-29）**：对齐固件 `ARM-NET-0924AV`（三构建 md5/字节数快照刷新，`GET_INFO` 与 `h` 两处 fw-tag 已同步）；错误码 7 改为按命令区分语义（总线读失败 vs Flash 参数区读失败）；`GET_STATUS`/`GET_FLAGS` 两套 flags 位图对照表；§17 增补 0924 系列新文本命令（`zf`/`C`/`colj2`/`bootlog`/`rc go/watch/keysw`/`nx~nt` 等）与 `TEXT_EXEC` 翻页陷阱。
>
> **v2.2.0（2026-09-30，全功能一键化）**：CLI 新增 13 个子命令——观测 `ik`（只解算不运动）/`detail`（六轴遥测，5 拍中位数抗丢读）/`flags`/`pid`/`gravity`/`corr`/`log`；运动 `jog`（MOVE_JOINT_ONE 单轴点动，当前角 None 拒绝盲动）；示教文件 `teach total|list|show|del|compact`；系统类 `zero reset`/`init`/`busmode`/`reboot`/`center`（⚠ 一律 `--yes` 闸门，不加只打印后果）；`params` 补 `save`（= 文本 `F`）。2.1.1：`home`/`limits` 一键命令；2.1.2：`fold`（= `P 0` 一键回折叠收纳位）/`pose 0~4`。模拟器同步实现全部新命令。

---

## 你是谁？先看这一份

| 你是 | 直接去 | 干什么 |
|---|---|---|
| **写上位机软件**（C / C# / Python / Web） | `host_integration/上位机对接指南.md` | ★ §3「决定树干这件事该发哪条指令」决策表；再抄 `arm_protocol.h` / `csharp/ArmLink.cs` / `minimal_client.py` 里的一份参考实现 |
| **只想把遥操/示教跑起来**（不写代码） | `launchers/启动步骤.md` | 改一个 JSON，双击 `.bat` 或跑 `start_teleop.sh`；先 `--preflight-only` 只读预检 |
| **要改配置** | `docs/配置文件说明.md` | 搞清楚 `config/arm_config.json`（**只读接口契约**）和 `launchers/arm_link_config.json`（**你要改的部署配置**）的区别 |
| **排查"为什么不动 / 值不对"** | `docs/网口协议速查.md` → `docs/网口指令大全.md` §11 | 463 行字段级速查 → 1419 行全量 + 故障对照表 |

> 分工：协议要点在 `host_integration/`，落地操作在 `launchers/`，两者都不改动 SDK 库本身（`arm_sdk/`）。

---

## 30 秒上手

```bash
cd sdk_package
export PYTHONPATH=$PWD

python -m arm_sdk info                                     # 能力摘要
python -m arm_sdk --uri sim:// preflight                   # 无硬件冒烟
python -m arm_sdk --uri net://192.168.7.120:5000 ping      # 真机探活
python tests/integration_test.py                           # 全套离线自检
```

```python
from arm_sdk import open_arm

with open_arm("net://192.168.7.120:5000") as arm:
    print(arm.ping())                  # PingInfo(proto_ver=1, run_id=..)
    st = arm.get_status()
    print(st.joints)                   # 舵机度；None = 该轴读失败（不兜底）
    arm.move_joints([180] * 6, v=20)   # ⚠ 舵机度，不是逻辑角
    print(arm.fk_query())              # 末端位姿
```

---

## 三条链路

| URI | 承载 | 线上外壳 | p50 | 适用 |
|---|---|---|---|---|
| `net://<ip>:5000` ★ | UDP | 无（一个数据报 = 一个逻辑包） | **1.61 ms** | 实时遥操、上位机主通道 |
| `serial:///dev/ttyUSB0` | USART6 115200 | `AA 55` + 逻辑包（与文本控制台同口） | 4.78 ms | 保底 + 文本命令 + 链路开关 |
| `can`（0x100+node） | CAN 2.0A 500k | 8 B 分包，首帧 `0x80\|N` | 2.72 ms | 差分总线、抗干扰 |
| `sim://` | 内存 | — | 0 | 无硬件开发 / CI |

**逻辑包三条链路完全一致**：

```
SEQ(1) | CMD(1) | LEN(1) | PAYLOAD(LEN) | CRC16-LE(2)
CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF），范围 = SEQ..PAYLOAD
```

换链路 = 换 Carrier，业务代码一行不改。

> 物理约束：**CAN 与网口互斥**（`PB13 = ETH_TXD1 = CAN2_TX`；`PC4/PC5 = RMII_RXD0/1 = 原绿/红 LED`）。
> 串口（USART6 PC6/PC7）与两者都不冲突 ⇒ **恒常通**，是切链路唯一可靠的通道。

---

## 主从遥操

```python
from arm_sdk import open_arm, TeleopSession, TeleopConfig

lead = open_arm("net://192.168.7.121:5000")     # 主臂
foll = open_arm("net://192.168.7.120:5000")     # 从臂

with TeleopSession(lead, foll, cfg=TeleopConfig(hz=50, slew_deg_per_s=240)) as tp:
    tp.run()                                     # 拖主臂 → 从臂跟随
print(tp.stats.summary())
```

```bash
armctl teleop --leader net://192.168.7.121:5000 --follower net://192.168.7.120:5000 --sec 30
```

**两臂之间一根线都不用接 —— PC 就是那根线。**

关键设计：

* **增量跟随**：`target = foll_ref + gain·sign·(lead − lead_ref)` ⇒ 起始姿态不同也安全
* **限速 240°/s**（50 Hz → 4.8°/帧）：固件单帧单轴跳变 >25° 会**整帧拒收且不更新基准**（自锁），必须从源头避开
* **hold 机制**：主臂读失败时继续发上一有效目标（避免 500 ms 断流自锁），超时后停臂
* **E-Stop 联锁**：主臂 flags.bit0 → 立刻 `STOP` 从臂

三条路线都实现了：网口直写（`TeleopSession`）★ / 固件直连（`FirmwareLink`）/ PC 中转（`PcRelay`）。

---

## 遥操示教

```python
from arm_sdk import TeleopTeachSession, Trajectory, PlaybackSession

with TeleopTeachSession(lead, foll, out="demo1.json", hz=50) as ts:
    ts.record(duration=60)                   # 一边遥操一边录；**无 80 s 上限**

Trajectory.load("demo1.json").save_csv("demo1.csv")     # 导出给 Excel/LeRobot

with open_arm("net://192.168.7.120:5000") as arm:
    PlaybackSession(arm).play(traj, speed=0.5, loop=3)  # 回放（自动先走到起点）
```

* 格式：`.json` / `.jsonl` / `.csv` / `.npz`
* 变换：`resample` / `smooth` / `slice` / `scale_time`
* 体检：`validate()` 查点数、采样率、单帧跳变（>25° 报警）、软限位越界
* 回放安全：**接近段**（先 `MOVE_JOINTS` 到起点，否则第一帧就拒帧）× 单帧硬限 20° × E-Stop 轮询

固件侧录音（`FirmwareTeach`）也封装了，但注意它的 80 s / 4000 点上限、`save` 阻塞 1~3 s、
以及"三种丢录音方式"（`rec clear` / 断电 / 再进 `teach`）。

---

## CLI 速查

```bash
armctl info                                     # 能力 / 链路 / 配置摘要
armctl selftest                                 # 全部离线自检
armctl --uri net://IP:5000 preflight            # 上板前置检查
armctl --uri net://IP:5000 ping --repeat 200    # 兼测延迟
armctl --uri net://IP:5000 status --json        # 状态（JSON）
armctl --uri net://IP:5000 fk                   # 正解
armctl --uri net://IP:5000 move --delta 0,+5,0,0,0,0 --v 5   # ★点动必须 --delta
armctl --uri net://IP:5000 home --wait                       # 一键回中位
armctl --uri net://IP:5000 fold --wait                       # ★一键回折叠收纳位（= P 0，CIR 键同款）
armctl --uri net://IP:5000 pose 2                            # 预定义位姿 0~4（0=HOME=折叠）
armctl --uri net://IP:5000 jog 0 5 --v 10                    # ★单轴点动：J1 +5°，其余五轴保持
armctl --uri net://IP:5000 detail                            # 六轴遥测（角度/速度/负载/电压/温度）
armctl --uri net://IP:5000 ik 250 0 180                      # IK 解算（只解不运动）
armctl --uri net://IP:5000 flags                             # 状态位图（可高频轮询）
armctl --uri net://IP:5000 limits                            # Flash 校准限位/零点（err=7=没标定过）
armctl --uri net://IP:5000 pid 0                             # J1 PID（EEPROM 权威值）
armctl --uri net://IP:5000 gravity                           # 重力前馈 K（带值=设）
armctl --uri net://IP:5000 corr get|on|off                   # 自适应修正
armctl --uri net://IP:5000 log 2                             # 固件日志级别
armctl --uri net://IP:5000 teach list|show 0|del 0|compact   # 板载示教文件
armctl --uri net://IP:5000 params save                       # 参数落盘（= F）
armctl --uri net://IP:5000 zero reset --yes                  # ⚠ 零点复位（= zr），必须 --yes
armctl --uri net://IP:5000 reboot --yes                      # ⚠ 整板软复位（力矩全掉）
armctl --uri net://IP:5000 center --yes                      # ⚠⚠ = `M a`，不可逆，仅标定用
armctl --uri net://IP:5000 stop
armctl --uri net://IP:5000 stream --pose "0,0,0,0,0,0" --hz 50 --sec 2
armctl --uri net://IP:5000 params list|get ID|set ID VAL
armctl teleop --leader ... --follower ... --sec 30
armctl record --leader ... --follower ... --out demo.json --sec 60
armctl traj info demo.json
armctl play demo.json --speed 0.5
armctl console --port COM11 --cmd h --cmd zi --cmd L     # 串口文本
armctl link --port COM11 --set net --save                # 链路开关
armctl relay --leader-port COM9 --follower-port COM11 --hz 30
```

---

## ★ 必须知道的 10 个坑

1. **两套角度**：`GET_STATUS/FK_QUERY/MOVE_JOINTS/MOVE_CART` 用**舵机度**(0~360)；
   `STREAM_JOINTS`/`#P` 用**逻辑角**(rel-home)。混用是头号 bug 源。
2. **点动必须用增量**：绝对角会被 `clamp_all_angles()` **静默钳制**（err=0，但值被改了）。
3. **静默钳制三处**：`MOVE_JOINTS` 目标角 / `PARAM_WRITE` 值 / `STREAM_JOINTS` 单轴。
   要确认真值必须**回读**（`param_write_verified()`）。
4. **err=0 ≠ 动了**：读起点失败时固件也回 ACK 但轨迹没起步。判断"动没动"必须看 `GET_STATUS` 角度变化。
5. **25° 跳变整帧拒收 + 不更新基准** ⇒ 会**自锁冻结**。必须发送端限速。
6. **断流 500 ms** ⇒ 从臂退出流模式并原地自锁 ⇒ 频率必须 >2 Hz。`STREAM_JOINTS` 丢帧**不重发**。
7. **读失败必须如实上报**：某轴 `None` / `-32768` / `read_ok=0` ⇒ 不要兜底，不要读位姿字段。
8. **两条响应负载顺序相反**：`GET_STATUS` 先关节后位姿；`FK_QUERY` 先位姿后关节，且 offset 20 是**裸 0/1**。
9. **六种"一个字节都不回"**：LEN 不符 / 错端口 / 错 IP / `CMD>=0x80`(防死锁) / CRC 错(回 NACK err=2) / 本构建没开网口。
10. **链路不热切换**：`link net` → `link save` → **RESET**。运行期只能看 `h` 的 `active=`。

---

## 静态/配置镜像

* `arm_sdk/arm_config.json`（随包安装）与仓库 `config/arm_config.json` 同一份：
  **114 条文本命令 + 38 条网口命令（字段级）+ 三条承载 + 链路开关 + 主从 + 示教 + SDK 说明**，40 个章节。
* 生成器：`tools/gen_arm_config_json.py`
* **四个落点由生成器一次写出，字节一致**：`config/arm_config.json`、
  `sdk_package/arm_sdk/arm_config.json`、`sdk_package/config/arm_config.json`、`sdk_package/data/arm_config.json`。
  ⚠ 2026-09-23 前两个 SDK 侧是手工同步的旧副本（漂移成 09-21 的 14 条版本），已改为由生成器统一写。
  **改需求改 `tools/net_ext_2026_09_23.py` 或生成器本身，不要手改 JSON —— 会被下次重生成抹掉。**
* **漂移检查**：`python tools/check_arm_config.py` —— 拿固件源码（命令注册表 / 多字符白名单 /
  协议头 / 错误码）和 JSON 对账，CI 可跑。
* **现场改的配置是另一份**：`launchers/arm_link_config.json`（主/从臂地址、频率、限速、示教参数），
  全字段说明见 `docs/配置文件说明.md`。

---

## 自检（全部离线，无需硬件）

| 命令 | 断言数 | 覆盖 |
|---|---|---|
| `armctl selftest` | **228** | 全部 offline 断言汇总（无 numpy 227，少的 1 项会打印跳过） |
| `python -m arm_sdk.protocol` | 45 | CRC 向量、5 条 hex 例子逐字节、定点编解码、CAN 分片/重组、分帧器 |
| `python -m arm_sdk.teleop` | 27 | 映射/限速/软限位/反向/hold/链路丢失/E-Stop/PC 中转编码 |
| `python -m arm_sdk.teach` | 43 | 轨迹变换、4 种格式往返、录制器、回放、接近段、预热帧 |
| `python tests/integration_test.py` | 51 | 14 条基础命令全语义 + 双臂遥操 + 录制回放端到端（sim） |
| `python tests/new_commands_test.py` | 62 | 24 条扩展命令 + TEXT_EXEC 文本透传 / 分页 / 闸门（sim） |
| `cd js && npm test` | **971** | JS 侧 7 组（含 JS↔Python 逐字节对拍 404 项 + 真 UDP 回环 40 项） |

### 交付前的四道闸门（每次改 SDK 都要跑）

```bash
python tools/check_arm_config.py    # ① JSON ↔ 固件源码对账（40 章 / 114 / 38）
python tools/check_proto_doc.py     # ② 文档 hex 报文 ↔ 协议实现（44 条报文）
python tools/check_cprint_len.py    # ③ 固件 cprint 单次 ≤255 字节（红线 4）
python tools/gen_protocol_spec.py --check   # ④ 协议规格 JSON 没漂移
```

再加两段「新东西」的自检：

```bash
python host_integration/minimal_client.py --selftest   # 16 项（独立实现，不 import arm_sdk）
armctl mock --selfcheck                                # 假臂服务 5 项
python launchers/launch_teleop.py --preflight-only --leader sim:// --follower sim://
```

打完包还有**第五道**：`python tools/verify_release.py` —— 拿包里的文件 ↔ 源工作树、
tar.gz ↔ zip ↔ 目录做**逐字节**比对，复算 `SHA256SUMS.txt`，并检查两个包之间必须一致的 15 份文件。
当前 75 项全通过。

---

## 文档

| 文件 | 内容 |
|---|---|
| ★★ `host_integration/上位机对接指南.md` | **场景→发哪条指令**决策表（10 节）： §3 决策树 + §4 高频功能完整调用序列 + §5 错误码→UI 映射 + §8 验收自测 20 项 |
| ★★ `launchers/启动步骤.md` | 主从遥操 / 遥操示教一键启动：6 步打印、4 种示教模式、三条路线选型、10 行排错表 |
| ★ `host_integration/protocol_spec.json` | 38 条命令的**机读**规格（由 `tools/gen_protocol_spec.py` 从 `arm_sdk/protocol.py` 自动生成） |
| ★ `host_integration/README.md` | 对接包文件清单 / 10 分钟上手 / 最容易踩的 5 个坑 / 三份实现一致性保证 |
| `host_integration/arm_protocol.h` | **C99 参考实现**（全 38 命令码 + CRC + 组包/解包，`-Wall -Wextra` 零告警） |
| `host_integration/csharp/ArmLink.cs` | **C# 参考实现**（SEQ 匹配串行化、丢读哨兵 `-32768`、TEXT 自动分页） |
| `host_integration/minimal_client.py` | **零依赖 Python 独立实现**（刻意不 import arm_sdk；自检 16 项 + 冒烟 6 步） |
| ★ `docs/配置文件说明.md` | 两个 JSON 的分工与全字段 +「改完怎么验」+ 三处不在 JSON 里的配置 |
| `docs/网口协议速查.md` | ★ 38 条命令字段级（14 基础 + 24 扩展）+ TEXT_EXEC + 6 种无响应 + 两套角度 + 30 行最小客户端 |
| `docs/主从遥操与示教.md` | 三条路线选型、限速原理、排查表、安全清单 |
| `docs/Linux部署与打包.md` | pip/wheel/离线/容器/systemd/udev/CI |
| `01_ARCHITECTURE(系统总体架构).md` | 系统架构 |
| `02_MECHANICAL(机械臂尺寸与运动学).md` | 尺寸 + 运动学 |
| `03_INTERFACE(详细接口数据).md` | 接口数据 |
| `04_SDK_SPEC(SDK规格书).md` | 串口控制台 SDK 规格（**注意 §6 关于主从臂的说明已被本版 SDK 取代**） |
| `05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md` | 上位机验收标准 |
| `AI_BRIEF(上位机软件制作任务要求参考文件).md` | 交给 AI 做上位机的任务书 |
| `examples/01~07_*.py` | 可运行示例（含 sim 无硬件版） |

---

## 许可

内部工程使用。
