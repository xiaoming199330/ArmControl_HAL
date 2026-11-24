# 六轴机械臂 SDK — 开发包 2.2.0

> **给写上位机软件的人。** 这一页看完就能开始写代码。
> 生成时间（UTC）：2026-09-30 07:36 ｜ 包内 25 项 / 17.2 MB

> 📦 **本包是「开发包」。** 另一份 `arm_sdk_linux_v2.2.0` 是给**部署方**的（少了示例 / 3D 模型 / 研发参考文档，多了 Linux 部署指南）。

---

## 1. 三分钟上手

```bash
# 装（三选一）
python3 -m pip install dist/arm_sdk-2.2.0-py3-none-any.whl     # A. 装 wheel（最快，无编译）
python3 -m pip install -e .                    # B. 可编辑安装（要改源码用这个）
export PYTHONPATH=$PWD                         # C. 不装直接跑

# 不接硬件也能跑：内存假臂
armctl --uri sim:// preflight                  # 上板前置检查（模拟器）
armctl --uri sim:// status                     # 读六轴角度
armctl info                                    # 能力/链路/配置摘要
```

```python
from arm_sdk import open_arm
with open_arm('net://192.168.7.120:5000') as arm:
    print(arm.ping())          # 探活
    print(arm.get_status())    # 六轴状态
```

```bash
python3 examples/02_offline_sim.py               # ① 先跑这个（不用硬件）
python3 examples/01_quickstart.py                # ② 网口最小例子
python3 examples/07_trajectory_tools.py          # ③ 轨迹 JSON/CSV/NPZ 转换
```

### JS 侧（Node ≥ 18，**零依赖**，不需要 npm install）

```bash
cd js && node src/cli.js selftest     # 971 项离线自检
node src/cli.js --uri sim:// ping     # 无硬件冒烟
```

```js
import { openArm } from './js/src/index.js';
const arm = await openArm('net://192.168.7.120:5000');   // Node / Electron 主进程
console.log(await arm.ping());
```

> ⚠ **浏览器发不了原始 UDP**（沙箱铁律，页面内没有绕法）。要网页控制机械臂，必须起边车：
> `node js/src/bridge.js --target 192.168.7.120:5000`，然后页面里 `openArm('ws://127.0.0.1:8765/')`。
> 边车**每个 WS 连接独占一个 UDP socket**，所以**一个进程能同时带主/从两路**（`ws://127.0.0.1:8765/?target=192.168.7.121:5000`）。详见 `js/docs/JS快速上手.md`。

### 不写代码也能跑：一键启动主从遥操 / 遥操示教

```bash
cd launchers
python install_deps.bat        # Windows 装依赖（Linux: ./install_deps.sh）
# 改 arm_link_config.json 里的 leader / follower 地址（默认 192.168.7.121 / .120:5000）

python launch_teleop.py --preflight-only   # ① 只预检，不动臂（安全）
python launch_teleop.py                    # ② 正式起主从遥操（6 步打印 + 安全闸门）

python launch_teach.py record --sec 30 --name demo   # ③ 录一段示教
python launch_teach.py list                          # ④ 列已有轨迹
python launch_teach.py play trajectories/demo.json --speed 0.5   # ⑤ 回放（先半速）
```

Windows 双击 `start_teleop.bat` / `start_teach.bat` 也行（自带 `chcp 65001`，不会因中文乱码崩）。
**完整步骤、排错表、三条路线选型 →  `launchers/启动步骤.md`。**

> ⚠ **遥操/示教是会让臂真动的。** 先跑 `--preflight-only`；第一次上真机把 `arm_link_config.json` 的 `gain` 调到 0.5、`slew_deg_per_s` 保持 240 不要加。
> 没接线也能全流程演练：把地址写成 `sim://`。

### 想用 HTTP 调臂？起边车（板子没 TCP，边车跑在 PC 侧）

```bash
armctl mock --port 5000                                   # 无真机时先起假臂
armctl http-bridge --target 192.168.7.120:5000 --port 8080 # 边车
curl http://127.0.0.1:8080/api/status                       # 就是普通 HTTP
```

> 边车不改协议、不加语义：错误码/丢读哨兵/静默钳制与 UDP 直连**完全一致**。
> 每个响应带 `X-Arm-Rtt-Ms` ⇒ 能把「链路延迟」和「HTTP 开销」分开算。
> ⚠ **50 Hz 遥操流不要走 HTTP**（每帧等应答会把帧率锁死在 RTT 上）；急停也走 UDP。**遥操走 UDP，配置/查询走 HTTP**，两条路并存。

**依赖**：核心纯标准库（网口 `net://` 什么都不用装）。可选：

| 依赖 | 什么时候需要 | 装法 |
|---|---|---|
| `pyserial` | 走串口 `serial:///dev/ttyUSB0` | `pip install pyserial` |
| `python-can` | 直接驱动 USB-CAN 适配器（不走板子） | `pip install python-can` |
| `numpy` | 轨迹存 `.npz` | `pip install numpy` |

---

## 2. 包里有什么

| 路径 | 必需 | 用途 | 体积 |
|---|---|---|---|
| `arm_sdk/（14 个文件）` | 是 | ★ SDK 源码。分层：protocol.py（逻辑包编解码）→ transport.py（UDP/串口/CAN）→ client.py（ArmClient）；teleop / teach / kinematics / console / sim / cli。另含 `mock_server.py`（假臂，无真机联调）与 `http_bridge.py`（HTTP↔UDP 边车，`armctl http-bridge`） | 554.6 KB |
| `dist/arm_sdk-2.2.0-py3-none-any.whl` | 是 | ★ 预构建 wheel（py3-none-any）—— Linux 上直接 pip install，免编译 | 185.9 KB |
| `pyproject.toml` | 是 | ★ pip 安装声明（console_scripts: armctl）；`pip install -e .` 走它 | 1.4 KB |
| `README.md` | 是 | ★ SDK 总说明（30 秒上手 / 10 个坑 / 自检表） | 15.8 KB |
| `js/（28 个文件）` | 是 | ★ JS SDK（零依赖，Node ≥ 18）：Node 原生 UDP + 浏览器 Web Serial + bridge.js 边车；与 Python 版逐字节一致（跨语言对拍 404 项） | 447.4 KB |
| `config/arm_config.json` | 是 | ★ 接口契约（114 文本命令 + 38 网口命令 + 40 章节）。**参数不要硬编码，从这里读** | 111.3 KB |
| `tests/（2 个文件）` | 是 | ★ 离线端到端验收测试（integration_test 51 + new_commands_test 62 断言；无 numpy 时少 1 项（.npz 往返），会打印跳过） | 22.8 KB |
| `docs/配置文件说明.md` | 是 | ★【配置文件】两个 JSON 的分工与全字段：`config/arm_config.json`（只读接口契约，39 节）vs `launchers/arm_link_config.json`（你要改的部署配置）；含「改完怎么验」与三处非 JSON 配置 | 12.9 KB |
| `docs/网口协议速查.md` | 是 | ★【网口协议】字段级速查（14 章 / 463 行）—— 第一优先级，先看这份 | 19.9 KB |
| `docs/网口指令大全.md` | 是 | ★【网口协议】全量版（1419 行）：38 条命令（14 基础 + 24 扩展）逐条详解 + TEXT_EXEC 文本透传 + 故障对照表 + 13 条网口红线 | 70.9 KB |
| `docs/网口下发操作手册.md` | 是 | ★【网口上手】操作步骤在前（探活→观测→运动→收工）+ armctl 全量命令注释 + 真实回传样例注解——拿到包先看这份再动臂 | 11.3 KB |
| `docs/串口下发操作手册.md` | 是 | ★【串口上手】screen 控制台：操作步骤在前（判活→状态→运动）+ 固件原生命令注释 + 真机 cprint 格式回显注解 | 8.7 KB |
| `docs/主从遥操与示教.md` | 是 | ★ 主从遥操与遥操示教：选型、参数、排查 | 10.6 KB |
| `docs/UDP_vs_HTTP_实测.md` | 是 | ★【链路选型】UDP vs HTTP 的速率/延迟/丢包对照：为什么板子没 TCP、HTTP 边车多付多少、板侧 nx 差分测真丢包的方法、以及「遥操走 UDP / 配置走 HTTP」的结论 | 22.5 KB |
| `host_integration/（7 个文件）` | 是 | ★★【上位机对接】场景→发哪条指令决策表（502 行）+ 自动生成的协议规格 JSON（38 条命令）+ 三份参考实现（C 头文件 / C# ArmLink / 零依赖 Python 独立实现）+ 假臂联调用法 | 129.4 KB |
| `launchers/（12 个文件）` | 是 | ★★【一键启动】主从遥操 / 遥操示教启动器（6 步打印 + 预检 + 安全闸门）+ 唯一配置入口 arm_link_config.json + Windows/Linux 启动脚本 + 《启动步骤.md》；带 --preflight-only 只读预检 | 55.4 KB |
| `examples/（7 个文件）` | 是 | 7 个可运行示例（01~07，含 sim 无硬件版）—— 写代码的样板 | 21.4 KB |
| `model/（16 个文件）` | 可选 | URDF + link_geometry.json + 15 个 STL（15.4 MB）—— 上位机做 **3D 显示 / IK 校验**才要。只做协议通讯不用它 | 15.4 MB |
| `docs/reference/01_ARCHITECTURE(系统总体架构).md` | 可选 | 系统总体架构 | 19.9 KB |
| `docs/reference/02_MECHANICAL(机械臂尺寸与运动学).md` | 可选 | 机械尺寸 + 运动学（画 3D / 算 IK 用） | 16.5 KB |
| `docs/reference/03_INTERFACE(详细接口数据).md` | 可选 | 详细接口数据（串口文本命令全表 + 寄存器 + 二进制协议） | 28.1 KB |
| `docs/reference/04_SDK_SPEC(SDK规格书).md` | 可选 | 串口控制台 SDK 规格书 ⚠ §6「主从臂不在范围」已被 v2 取代 | 26.1 KB |
| `docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md` | 可选 | ★ 上位机**功能范围与验收标准** —— 开发方的需求依据 | 20.3 KB |
| `docs/reference/06_AI_BRIEF(上位机任务书).md` | 可选 | 上位机任务书（交给 AI 做上位机时的输入） | 9.7 KB |
| `docs/reference/07_开发包摘要.md` | 可选 | 开发包摘要（09-11 版，整体背景） | 6.7 KB |

### 先读哪份文档

| 你要干什么 | 看哪份 |
|---|---|
| ★★ 写代码：**这个功能该发哪条指令** | `host_integration/上位机对接指南.md` §3 场景决策表 |
| ★★ 拿协议真值写代码 / 生成代码 | `host_integration/protocol_spec.json`（38 条，机读） |
| ★★ 抄一份参考实现 | `host_integration/arm_protocol.h`（C）/ `csharp/ArmLink.cs`（C#）/ `minimal_client.py`（Python，零依赖） |
| ★ 搞懂网口协议（**第一优先级**） | `docs/网口协议速查.md`（463 行，字段级） |
| 排障 / 现象→原因对照 | `docs/网口指令大全.md` §11 故障对照表（1419 行全量版） |
| ★ 两个配置文件的分工与全字段 | `docs/配置文件说明.md` |
| 拿协议真值写代码 | `config/arm_config.json` → `arm_config.network_protocol` |
| 做 3D 显示 / 校验 IK | `model/arm_model.urdf` + `model/link_geometry.json` |
| 上位机功能范围与验收标准 | `docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md` |
| 整体架构 / 机械参数 | `docs/reference/01_...` / `02_...` |
| 主从遥操怎么接 | `docs/主从遥操与示教.md` |
| ★ **不写代码**，直接把遥操/示教跑起来 | `launchers/启动步骤.md` |
| ★ 不想自己处理 CRC/SEQ/分页 → 用 HTTP | `docs/UDP_vs_HTTP_实测.md` + `armctl http-bridge` |
| 链路选哪条、实测多快 | 独立文档《三链路延迟基准_简版》 |

> **`config/arm_config.json` 是接口真值源**：114 条串口文本命令、38 条网口命令（字段级）、三条承载、链路开关、关节限位、舵机偏移、运行时参数表、错误码。**不要把这些硬编码进你的代码** —— 从 JSON 读，固件升级也不用改代码。

---

## 3. 验收：确认你手上的包是好的

```bash
# ① Python：离线自检（不需要机械臂）
armctl selftest                    # 228 项（无 numpy 为 227，少的 1 项会打印跳过）
python3 tests/integration_test.py  # 51 项 (无 numpy 50)
python3 tests/new_commands_test.py # 62 项（2026-09-23 扩展命令）

# ② JS：零依赖
cd js && npm test                  # 7 组 / 971 项（含 JS↔Python 逐字节对拍 + 真 UDP 回环）
```

---

## 4. 部署前必须知道的硬约束（两个包的收件人都要看）

1. **两套角度混用是头号 bug 源**：`GET_STATUS` / `FK_QUERY` / `MOVE_JOINTS` / `MOVE_CART` 用**舵机度（0~360）**；`STREAM_JOINTS` / `#P` 用**逻辑角（相对 Home 零点）**。
2. **点动必须发增量**（`--delta`）：发绝对角会被固件 `clamp_all_angles()` **静默钳制**，回 err=0 但值被改了。
3. **静默钳制有三处**：`MOVE_JOINTS` 目标角 / `PARAM_WRITE` 值 / `STREAM_JOINTS` 单轴。要确认真值必须**回读**。
4. **`err=0` 不代表动了**：读起点失败时固件也回 ACK 但轨迹没起步 ⇒ 判断“动没动”必须看 `GET_STATUS` 角度变化。
5. **单帧跳变 >25° 会整帧拒收，且不更新基准 ⇒ 自锁冻结**。发送端必须限速（SDK 里 `TeleopSession` 已内置）。
6. **断流 >500 ms** 从臂退出流模式并原地自锁 ⇒ 频率必须 >2 Hz，推荐 50 Hz。`STREAM_JOINTS` **丢帧不重发**。
7. **读失败必须如实上报**：某轴返回 `None` / `-32768` / `read_ok=0` ⇒ **不要兜底、不要读位姿字段**。SDK 已强制这个行为。
8. **两条响应的负载顺序相反**：`GET_STATUS` 先关节后位姿；`FK_QUERY` 先位姿后关节，且 offset 20 是**裸 0/1**，不是 bit3。
9. **六种“一个字节都不回”**：LEN 不符 / 错端口 / 错 IP / `CMD>=0x80`（防死锁）/ CRC 错（回 NACK err=2）/ 本固件构建没开网口。
10. **链路不热切换**：`link net` → `link save` → **按 RESET**。板子默认**两条链路都关**，只留串口。
11. **CAN 与网口物理互斥**（`PB13 = ETH_TXD1 = CAN2_TX`）。串口恒常通。
12. **烧录/复位必丢力矩、臂会下沉**（实测 J2 曾掉到距下限仅 1.3°）。上运动测试前先回中位；**绝对不要用 `M a`**（那是标定，会废掉量纲基准）。

---

## 5. 现场信息（直接填进你的配置）

| 项 | 值 |
|---|---|
| 从臂 IP:端口 | `192.168.7.120:5000`（UDP） |
| 主臂 IP:端口 | `192.168.7.121:5000`（UDP） |
| 串口 | USART6，**115200 8N1 无流控**，行尾统一 `\r\n` |
| CAN | 2.0A，**500 kbit/s**，命令 `0x100+node`，响应 `0x180+node` |
| 板内固件（三构建，2026-09-29 重编） | `ARM-NET-0924AV` 263,128 B md5 `4c74c6177fd39d2794107fd871e8c2cb`；`ARM-CAN-0918H` 215,096 B md5 `cd11183e6d6aa59cf81cad77c6ca3b5c`；`ARM-BOTH-0924AV` 263,092 B md5 `d55f6202f20825936b31ad7481276da6` |
| ⚠ fw-tag 两处硬编码 | 控制台 `h` 与 `GET_INFO(0x07)` 现已**同步为 0924AV**，但仍是 main.c / arm_control_api.c 各自硬编码 —— 固件升版时必须两处同改；上位机**以 `h` / PING 为准** |
| 网口前提 | 必须 `link net` → `link save` → RESET 后网口才通 |
| 运行时限位（`L`） | J1 63~303 / J2 75~284 / J3 76~271 / J4 71~284 / J5 12~351 / J6 138~266 |

---

## 6. 刻意没有放进来的东西（别去找）

| 未包含 | 原因 |
|---|---|
| `python/` | ⚠ 已被取代：旧版**纯串口** SDK（arm_sdk.py / arm_fk.py）。能力已并入 arm_sdk/console.py + arm_sdk/kinematics.py。带两份会让部署方不知道 import 哪个。 |
| `web/` | ⚠ 已被取代：旧版**纯串口**浏览器 SDK（Web Serial，不支持网口/遥操）。要做 Web/Electron 且走网口 → 用包内 `js/`。 |
| `tools/` | 研发侧工具（打包器 / 配置生成器 / 模型生成器），交付方不需要。 |
| `data/` | 与 config/arm_config.json **完全重复**，只保留一份。 |
| `build/ · *.egg-info/` | 打包中间产物，重新构建即可生成。 |
| `__pycache__/` | 字节码缓存。 |

> 要 **Starter / 部署** 相关内容 → 找 `arm_sdk_linux_v2.2.0` 那份包。

---

## 7. 出问题找谁

* 协议 / 字段 / 错误码 → `docs/网口协议速查.md`，查不到 → `docs/网口指令大全.md`
* 主从遥操 / 示教 → `docs/主从遥操与示教.md`
* 上位机功能范围与验收口径 → `docs/reference/05_HOST_APP_REQUIREMENTS(上位机功能要求与验收标准).md`
* 以上都没有 → 回头找固件/上位机对接人（本包生成方）
