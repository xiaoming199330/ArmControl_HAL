# 六轴机械臂 SDK — Linux 部署包 2.2.0

> **给负责把 SDK 装到 Linux 机器上的人。** 先看这一页，再进 `docs/Linux部署与打包.md`。
> 生成时间（UTC）：2026-09-30 07:37 ｜ 包内 17 项 / 1.6 MB

> 📦 **本包是「部署包」。** 写上位机软件的人要的是另一份 `arm_sdk_dev_v2.2.0`（多给示例 / 3D 模型 / 上位机需求与验收标准）。

---

## 1. 三分钟上手

```bash
# ★ 方式 A（推荐）：装 wheel，纯 Python 无编译、零依赖
python3 -m pip install dist/arm_sdk-2.2.0-py3-none-any.whl

# 方式 B：源码可编辑安装（要打补丁 / 二次打包时用）
python3 -m pip install -e .

# 方式 C：不安装，直接跑（零依赖也能用）
export PYTHONPATH=$PWD
python3 -m arm_sdk info
```

```bash
armctl info                                    # 能力/链路/配置摘要
armctl selftest                                # 全部离线自检（无硬件：228 项，无 numpy 227）
armctl --uri sim:// preflight                  # 无硬件冒烟

armctl --uri net://192.168.7.120:5000 ping     # ① 真机探活
armctl --uri net://192.168.7.120:5000 status   # ② 读六轴状态（只读，安全）
armctl --uri net://192.168.7.120:5000 preflight  # ③ 上板前置检查
```

```bash
# 离线部署（目标机不能上外网时）
# 有网机器：python3 -m pip download --only-binary=:all: -d /tmp/whl pyserial
# 目标机：  python3 -m pip install --no-index --find-links=/tmp/whl pyserial
```

### JS 侧（Node ≥ 18，**零依赖**，不需要 npm install）

```bash
cd js && node src/cli.js selftest                      # 971 项离线自检
node src/cli.js --uri net://192.168.7.120:5000 status # 真机读状态
```

> ⚠ **浏览器发不了原始 UDP**（沙箱铁律）。要网页控制机械臂，起边车：
> `node js/src/bridge.js --target 192.168.7.120:5000`，页面里 `openArm('ws://127.0.0.1:8765/')`。
> 边车**每个 WS 连接独占一个 UDP socket** ⇒ **一个进程可同时带主/从两路**。
> 生产环境建议用 `systemd` 常驻（见 `docs/Linux部署与打包.md` §5）。

### 一键启动主从遥操 / 遥操示教（部署方最常用）

```bash
cd launchers && ./install_deps.sh          # 装依赖（Windows: install_deps.bat）
# 改 arm_link_config.json 的 leader / follower 地址
python3 launch_teleop.py --preflight-only  # ① 只预检（PING/状态/读失败轴/E-Stop/fw-tag/零点/限位）
python3 launch_teleop.py                   # ② 起主从遥操
python3 launch_teach.py record --sec 30 --name demo && python3 launch_teach.py list
python3 launch_teach.py play trajectories/demo.json --speed 0.5
```

> `./start_teleop.sh` / `./start_teach.sh` 是等价的封装（自动 `-X utf8`）。
> **完整步骤与排错表 → `launchers/启动步骤.md`**；想接成 systemd 常驻，把上面命令塞进 unit 即可（见 `docs/Linux部署与打包.md` §5）。

**依赖**：核心纯标准库。**网口 `net://` 不需要任何第三方库** —— 这是首选链路（实测 p50 1.61 ms、547 帧/s、零超时）。可选：

| 依赖 | 什么时候需要 | 装法 |
|---|---|---|
| `pyserial` | 走串口 `serial:///dev/ttyUSB0` | `pip install pyserial` |
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
| `docs/Linux部署与打包.md` | 是 | ★ Linux 部署指南：安装三法 / 离线部署 / udev 固定串口名 / 网络调优 / systemd / Docker / 升级回滚 / 常见问题 | 8.6 KB |

### 关键目录说明

* **`dist/*.whl`** — 唯一的安装入口。`py3-none-any` ⇒ 纯 Python，Linux 上不需要 gcc / 不需要联网编译。
* **`arm_sdk/`** — 源码。部署只需要 wheel；源码留着是为了**打补丁 / 用 `pip install -e .` / 用 `PYTHONPATH=$PWD` 直跑**。
* **`config/arm_config.json`** — 全部接口真值（114 条文本命令、38 条网口命令字段级、关节限位、舵机偏移、参数表）。**要改参数从这里读，别硬编码。**
* **`docs/Linux部署与打包.md`** — 部署主文档：安装三法 / 离线部署 / udev 固定串口名 / 网络调优（50 Hz 遥操）/ systemd / Docker / 升级回滚 / 常见问题。
* **`js/`** — 零依赖 JS SDK。要出 Web/Electron 界面时用得上；`src/bridge.js` 是浏览器走网口的唯一通路。
* **`tests/` + `docs/网口协议*.md`** — 验收与排障。
* **`launchers/`** — ★ 一个目录搞定「把臂跑起来」：`launch_teleop.py`（主从遥操）/ `launch_teach.py`（示教录放）+ 唯一配置入口 `arm_link_config.json` + `start_teleop.sh` / `start_teach.sh` + 《启动步骤.md》。先跑 `python launch_teleop.py --preflight-only`（只读预检）。
* **`host_integration/`** — ★ 给**写上位机软件的人**：`上位机对接指南.md`（场景→发哪条指令 决策表）+ `protocol_spec.json`（38 条命令机读规格）+ 三份参考实现（`arm_protocol.h` / `csharp/ArmLink.cs` / `minimal_client.py`）。
* **`docs/配置文件说明.md`** — 两个 JSON 的分工与全字段，改配置前先看这页。
* **`docs/UDP_vs_HTTP_实测.md`** — ★ 要不要用 HTTP、多付多少延迟、吞吐砍到几分之一；配 `armctl http-bridge` 用（板子没 TCP，边车跑在 PC 侧）。

---

## 3. 验收动作（交付方与部署方照这个对）

```bash
# ① 离线：不需要机械臂
armctl selftest                    # 228 项（无 numpy 227；少的 1 项是 .npz 往返，会打印跳过）
python3 tests/integration_test.py  # 51 项（无 numpy 50）
python3 tests/new_commands_test.py # 62 项（2026-09-23 扩展命令）

# JS 侧（Node ≥ 18；零依赖）
cd js && npm test                  # 7 组 / 971 项（含跨语言逐字节对拍 + 真 UDP 回环）
```

```bash
# ② 联机：只读、不要动
armctl --uri net://192.168.7.120:5000 preflight
armctl --uri net://192.168.7.120:5000 ping --repeat 200   # 兼测 P50/P99
armctl --uri net://192.168.7.120:5000 status
```

> ⚠ **网口不通先看这两条**：① 板子默认**两条链路都关**，必须先串口敲 `link net` → `link save` → **按 RESET**；② PC 网段要配到 `192.168.7.x`。
> 网口不通时的表现是**一个字节都不回**，不是报错。

> ⚠ **上运动测试之前**：先回中位 `armctl --uri net://... move --delta 0,0,0,0,0,0` 或串口 `m 180 180 180 180 180 180 15`。烧录/复位必丢力矩、臂会下沉。**绝对不要用 `M a`**（那是标定，会废掉量纲基准）。

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

> 需要**示例代码 / 3D 模型 / 上位机需求与验收标准** → 找 `arm_sdk_dev_v2.2.0` 那份包。

---

## 7. 出问题找谁

* 协议 / 字段 / 错误码 → `docs/网口协议速查.md`（查不到 → `docs/网口指令大全.md`，含**故障对照表**与 **13 条网口红线**）
* 连不上 / 装不上 / 容器 / systemd / 网络调优 → `docs/Linux部署与打包.md`
* 主从遥操 / 示教 → `docs/主从遥操与示教.md`
* 链路选哪条、实测多快 → 独立文档《三链路延迟基准_简版》
* 以上都没有 → 回头找固件/上位机对接人（本包生成方）
