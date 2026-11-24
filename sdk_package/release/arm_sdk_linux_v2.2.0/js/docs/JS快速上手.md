# arm-sdk-js 快速上手

> 六轴机械臂上位机 SDK（JavaScript）。**同一套逻辑包协议，四种承载**：
> UDP（Node，p50 1.61ms ●首选）／WebSocket 桥（浏览器）／Web Serial（浏览器串口）／内存模拟器。
>
> 版本 2.0.0 ｜ 零依赖 ｜ Node ≥ 18（浏览器现代版）｜ 与 Python 版 `arm_sdk/` 分层一一对应

---

## 0. 和 Python 版的关系

| 层 | Python | JS | 说明 |
|---|---|---|---|
| 协议编解码 | `protocol.py` | `src/protocol.js` | **逐字节一致**，有跨语言对拍测试（404 项） |
| 承载 | `transport.py` | `src/transport.js` | Python 有串口/CAN；JS 有 UDP/WS 桥/Web Serial/内存 |
| 客户端 | `client.py` | `src/client.js` | SEQ 配对 + 超时重试 + 响应归一化 |
| 遥操 | `teleop.py` | `src/teleop.js` | 主从映射 / 软限位 / 限速 / 丢控保护 |
| 示教 | `teach.py` | `src/teach.js` | 录制 / 轨迹 / 回放（`.npz` 只有 Python 支持） |
| 运动学 | `kinematics.py` | `src/kinematics.js` | FK/IK/限位，与固件逐位一致 |
| 文本控制台 | `console.py` | `src/console.js` | `h/s/f/m…` 人机调试命令的行解析 |
| 模拟器 | `sim.py` | `src/sim.js` | 离线假臂，能主动注入故障 |
| CLI | `armctl` | `armctl-js` | 子命令同构（串口类除外） |

**JS 独有**：`src/bridge.js` —— 浏览器用网口的唯一通路（见 §4）。

---

## 1. 三十秒上手

```js
import { openArm } from 'arm-sdk-js';

const arm = await openArm('net://192.168.7.120:5000');   // Node 直连 UDP
const pi  = await arm.ping();
console.log(pi.proto_ver, pi.run_id, pi.latencyMs);

const st = await arm.getStatus();
console.log(st.joints);        // 舵机度 0~360，读失败轴是 null（不兜底）
await arm.close();
```

没有硬件？把 URI 换成 `sim://`，全部 API 照跑。

---

## 2. 四种承载怎么选

| URI | 承载 | 跑在哪 | 实测 p50 | 备注 |
|---|---|---|---|---|
| `net://<ip>:5000` | UDP | **Node / Electron 主进程** | **1.61 ms** | ● 首选。一个数据报 = 一个逻辑包，无帧头 |
| `udp://<ip>:5000` | UDP | 同上 | 同上 | 别名 |
| `ws://<host>:8765` | WebSocket 桥 | 浏览器 / 渲染进程 | ≈1.61 ms | 先去起 `bridge.js`（§4） |
| `webserial://` | Web Serial | 浏览器 | 4.78 ms | 需 https/localhost + **用户手势**授权 |
| `sim://` | 内存 | 任意 | 0 | 离线开发 / CI |

```js
import { makeTransport, ArmClient } from 'arm-sdk-js';
const arm = new ArmClient(await makeTransport('net://192.168.7.120:5000', { timeoutMs: 2000 }));
await arm.open();
```

> ⚠ Node 里没有 `navigator.serial`，所以 `webserial://` 在 Node 下不可用。
> 要串口请用浏览器，或 Python 版。

---

## 3. 三条链路、一份协议

所有承载上跑的是**同一个逻辑包**：

```
SEQ(1) | CMD(1) | LEN(1) | PAYLOAD(LEN) | CRC16-LE(2)
```

差别只在"怎么塞进物理层"：

| 承载 | 封装 |
|---|---|
| UDP | 无帧头，一个数据报就是一个逻辑包 |
| WebSocket 桥 | 原样透传（一个二进制帧 = 一个逻辑包） |
| Web Serial | `AA 55` 前缀 + 逻辑包（与固件文本控制台同口） |
| CAN | 8 字节分包，需重组（JS 侧只提供了编解码，未接 SocketCAN） |

CRC 是 **CRC-16/CCITT-FALSE**（poly `0x1021`，init `0xFFFF`，不反射）。
标准向量：`crc16("123456789") === 0x29B1`。

---

## 4. 浏览器用网口：bridge.js 边车

**浏览器沙箱做不到原始 UDP**（没有 socket API）。所以：

```
页面(WsTransport) ──ws://127.0.0.1:8765──> bridge.js(Node) ──UDP──> 机械臂
                  <──二进制帧（逻辑包原样）──              <──回包────
```

### 起边车

```bash
node src/bridge.js --target 192.168.7.120:5000
# 可选：--port 8765  --host 127.0.0.1  --allow 127.0.0.1  -v
```

### 页面里连

```js
const arm = await openArm('ws://127.0.0.1:8765/');
```

**同一个边车可以同时服务两路**（每个连接独占一个 UDP socket，回包按源端口隔离）：

```js
const leader   = await openArm('ws://127.0.0.1:8765/?target=192.168.7.121:5000');
const follower = await openArm('ws://127.0.0.1:8765/?target=192.168.7.120:5000');
```

### 边车控制帧（文本帧，JSON）

| 发出 | 作用 |
|---|---|
| `{"op":"setTarget","target":"192.168.7.121:5000"}` | 运行期切换目标臂 |
| `{"op":"stats"}` | 回 `{"type":"stats",...}` |
| `{"op":"ping"}` | 回 `{"type":"pong"}` |

回推事件：`{"type":"hello"|"target"|"stats"|"error", ...}`。

### 安全

默认只允许 `127.0.0.1` 连入。**它没有任何鉴权**，别在公网裸跑。
放开用 `--allow 192.168.1.0/24` 或 `--allow 0.0.0.0/0`。

---

## 5. 命令一览（38 条）

| CMD | 值 | 参数量纲 | 响应 |
|---|---|---|---|
| PING | 0x01 | — | 0x81 |
| GET_STATUS | 0x02 | — | 0x82 |
| LOG_SET_LEVEL | 0x03 | — | ACK/NACK |
| MOVE_JOINTS | 0x10 | **舵机度 0~360** | ACK/NACK |
| STOP | 0x11 | — | ACK/NACK |
| SET_TORQUE | 0x12 | joint 0~5 / 0xFF | ACK/NACK |
| STREAM_JOINTS | 0x13 | **逻辑角（相对 Home）** | ACK/NACK |
| MOVE_CART | 0x20 | mm / deg | ACK/NACK |
| FK_QUERY | 0x21 | — | 0xA1 |
| SET_GRAVITY | 0x30 | deg/mm | ACK/NACK |
| SET_CORRECTION | 0x31 | — | ACK/NACK |
| PARAM_READ | 0x40 | — | 0xC0 |
| PARAM_WRITE | 0x41 | — | ACK/NACK |
| PARAM_COUNT | 0x42 | — | 0xC2 |

错误码：`0 OK / 1 PARAM / 2 CRC / 3 UNREACH / 4 LIMIT / 5 ESTOP / 6 FK_VERIFY / 7 READ /`
`8 MAGIC / 9 BUSY / 10 NOTCAL`

### 5.1 2026-09-23 扩展：另外 24 条

| CMD | 值 | 用途 | 响应 |
|---|---|---|---|
| GET_JOINT_DETAIL | 0x04 | 关节遥测（温度/负载/电压/硬件错误） | 0x84 |
| GET_LIMITS | 0x05 | raw 限位与 Home 零点（Flash 校准记录） | 0x85 |
| GET_INFO | 0x07 | 固件 tag / 构建 / **运行链路** / 参数条目 | 0x87 |
| GET_FLAGS | 0x0A | 状态位图（**不碰总线**，可 10~50Hz 轮询） | 0x8A |
| **TEXT_EXEC** | **0x0B** | ⭐ 在臂上执行**任意控制台命令** | 0x8B |
| **TEXT_FETCH** | **0x0C** | 取输出的后续分页 | 0x8C |
| MOVE_JOINT_ONE | 0x14 | 单轴移动（**舵机度**），其余不动 | ACK/NACK |
| SET_BUS_MODE | 0x17 | 0=正常 / 1=全A / 2=全B | ACK/NACK |
| SERVO_INIT | 0x18 | ⚠ 重写 PID/限位/中位并 RESET | ACK/NACK |
| IK_QUERY | 0x22 | **只解不算**：能不能到 + 会撞哪个限位 | 0xA2 |
| GET_GRAVITY | 0x32 | 读重力前馈 K | 0xB2 |
| GET_CORRECTION | 0x33 | 读自适应修正配置 | 0xB3 |
| GET_PID | 0x36 | 读单轴 PID（EEPROM 权威值） | 0xB6 |
| PARAM_SAVE | 0x43 | 等价文本 `F`（带 MAGIC） | ACK/NACK |
| PARAM_LIST | 0x44 | 参数元数据（6 条/页自动翻页） | 0xC4 |
| TEACH_LIST / READ / DELETE / COMPACT / TOTAL | 0x50/51/53/54/55 | 示教轨迹存储 | 0xD0/0xD1/…/0xD5 |
| CENTER_CALIB | 0x70 | ⚠ 等价 `M a`（改量纲基准） | ACK/NACK |
| CALIB_ENTER | 0x71 | ⚠ **固件一律 NACK** —— 见下 | NACK |
| ZERO_RESET | 0x72 | ⚠ 等价 `zr` / `zr save`（**全六轴**） | ACK/NACK |
| SYS_RESET | 0x73 | ⚠ 软复位（回 ACK 后 150 ms） | ACK/NACK |

三条铁律：**命令码 < 0x80**（`0x80+` 是回包空间）；**单帧 ≤ 64 B**
⇒ 大数组必须分页；**危险命令必须带 `MAGIC = 0x5A5A5A5A`**（SDK 自动填）。

#### TEXT_EXEC — 一把钥匙开整个控制台

```js
const out = await arm.textExec('s');        // 与在 COM11 敲 `s` 同一份输出
const why = await arm.textExec('cx');       // 抢引脚的命令：把**原因**当输出回给你
const no  = await arm.textExec('c');         // 被闸门拒："GATE ..." —— 见下
```

| 情况 | 表现 |
|---|---|
| 124 条里的 **113 条** | 正常执行，输出自动翻页 |
| 11 条抢 ETH 引脚的（`cx cp cy cz cw ps can mg mp mo ml`） | 固件接住并说明原因（不是静默失败） |
| `c`（校准向导） | 闸门拒绝 —— `for(;;)` 等本机串口输入，从网口进会卡死主循环 |
| 急停中 | 只放行 `s` / `h` |
| 命令 > 58 字符 | 客户端就抛错（不让固件钳断成**另一条命令**） |
| 输出 > 2048 B | 末尾追加 `[截断]`（`h` 帮助表 5406 B 就是这种） |

⚠ `CALIB_ENTER` **设计上不可用** —— 它本来就要求人手摆位，是控制台专属命令。
SDK 保留这个方法是为了让"不可用"在 API 上**可见**，而不是给一个没头没尾的参数错。
要校准请接 COM11。

```js
import { JointCalib } from 'arm-sdk-js';
const calib = new JointCalib();          // 默认 offset=180×6, dir=[-1,1,1,1,1,1]
const logical = calib.toLogical(st.joints);   // 舵机度 → 逻辑角（STREAM 用）
const servo   = calib.toServo(logical);       // 逻辑角 → 舵机度（MOVE 用）
```

---

## 6. 必须记住的六条安全约定

1. **点动只用增量**。`MOVE_JOINTS` 超限会被固件 `clamp_all_angles()` **静默钳制**：回 `err=0`，值却被改了。
   ```js
   const st = await arm.getStatus();
   const tgt = st.joints.slice(); tgt[1] += 5;      // ✅ 增量
   await arm.moveJoints(tgt, 10);
   // await arm.moveJoints([180,320,180,180,180,180]) // ❌ 绝对角 + 超限 = 静默钳制
   ```
2. **err=0 不代表到位**，运动后必须回读角度确认。
3. **读失败是 `null`，不兜底**。绝不拿 0 或上次值冒充姿态。
4. **STREAM_JOINTS 单帧单轴跳变 > 25° → 整帧拒收且基线不推进**（自锁）。
   发送端必须自己限速。
5. **流断 500ms → 从臂退出流模式**。丢控时要 hold 重发上一目标，别让流断。
6. **MOVE_CART 会归零 J5/J6**。网络遥操请用 `STREAM_JOINTS`，不要用笛卡尔。

---

## 7. 遥操（主从）

```js
import { openArm, TeleopConfig, TeleopSession } from 'arm-sdk-js';

const leader   = await openArm('net://192.168.7.121:5000');
const follower = await openArm('net://192.168.7.120:5000');

const tp = new TeleopSession(leader, follower, {
  cfg: new TeleopConfig({
    hz: 50,
    slewDegPerSec: 240,          // 每轴限速，压住 25° 门限
    sync: 'hold',                // hold=增量跟随（两臂起始姿态可不同）；align=先绝对对齐
    softLimitsDeg: Array.from({ length: 6 }, () => [-20, 20]),
    estopTrips: true,
  }),
  onEvent: console.log,
});
await tp.start();
await tp.run({ durationS: 30 });   // 省略 durationS = 跑到 Ctrl-C
await tp.stop();
console.log(tp.stats.summary());
```

> ⚠ 真机前置：从臂 `sf` 应为 1（被关过要重新 `sf 1`）；丢控后接收机会继续输出预设行程，
> 所以 X/Y/Z 的 F/S **必须设中位**。

---

## 8. 示教：录制 → 回放

```js
import {
  openArm, TeleopConfig, TeleopTeachSession, PlaybackSession, Trajectory,
} from 'arm-sdk-js';

/* 录制（PC 侧，无固件那条 80s 上限） */
const ts = new TeleopTeachSession(leader, follower, { out: 'demo.json', hz: 50, cfg: new TeleopConfig({ hz: 50 }) });
const traj = await ts.record({ durationS: 30, out: 'demo.json' });
console.log(traj.summary(), traj.validate());      // validate: 单帧跳变/采样率/时间单调

/* 回放（默认先"接近段"低速走到起点，避免第一帧大跳变被拒） */
const pb = new PlaybackSession(follower, {
  cfg: new TeleopConfig({ hz: 50 }),
  slewDegPerSec: 60,
  estopCheckS: 0.3,
  approach: true,
});
const stats = await pb.play(traj, { speed: 1.0, loop: 1, approach: true, approachSpeed: 20 });
console.log(stats.summary());
```

轨迹格式：`.json` / `.jsonl` / `.csv`（`.npz` 只有 Python 版支持）。
`PlaybackSession` 内部有 **20°（=25×0.8）单帧硬限**，超了会自动限速。

---

## 9. CLI：`armctl-js`

```bash
node src/cli.js info
node src/cli.js selftest                        # 跑全部离线自检
node src/cli.js --uri net://192.168.7.120:5000 ping --repeat 20
node src/cli.js --uri net://192.168.7.120:5000 status --json
node src/cli.js --uri net://192.168.7.120:5000 fk
node src/cli.js move 0,5,0,0,0,0 --delta --v 10 --wait
node src/cli.js --uri net://192.168.7.120:5000 params list
node src/cli.js teleop --leader net://192.168.7.121:5000 --follower net://192.168.7.120:5000 --sec 30
node src/cli.js record --leader ... --follower ... --out demo.json --sec 30
node src/cli.js play demo.json --speed 0.5
node src/cli.js traj info demo.json
node src/cli.js bridge --target 192.168.7.120:5000     # 给浏览器用
node src/cli.js --uri net://192.168.7.120:5000 text --cmd s --cmd zi
node src/cli.js --uri net://192.168.7.120:5000 diag --detail
```

全局开关：`--uri`（也可用环境变量 `ARM_URI`）、`--timeout`、`--dry-run`（**一个字节都不发**）、`-v`。

`console` / `link` / `relay` 需要串口 → 用浏览器（`console.js`）或 Python 版 `armctl`。

安装成全局命令（可选）：

```bash
cd js && npm link        # 之后可直接 `armctl-js ...`
```

---

## 10. 离线自检

```bash
cd js && npm test        # = node test/run_all.mjs
```

| 组 | 断言数 | 验的是什么 |
|---|---|---|
| `protocol.js` 自检 | 57 | CRC/编解码/边界/量纲换算/新命令负载 |
| `sim.js` 自检 | 127 | 假臂能否复刻固件的"阴间行为"（含 24 条新命令 + 文本闸门） |
| `bridge.js` 自检 | 14 | WS 握手/帧编解码/白名单/**真透传** |
| `cross_python.mjs` | 404 | **JS ↔ Python 逐字节对拍**（含 13 组新命令向量） |
| `legacy_parity.mjs` | 225 | 旧 `web/arm_sdk.js` 行为回归 |
| `integration.mjs` | 104 | client / teleop / teach 端到端（内存） |
| `udp_loopback.mjs` | 40 | **真实 UDP socket** 回环 + 假固件 |

合计 **971 项**。全绿才收工。

```bash
node src/bridge.js --selftest     # 单独验边车
node src/protocol.js              # 单独验协议
node src/sim.js                   # 单独验假臂
```

---

## 11. 示例

| 文件 | 内容 |
|---|---|
| `examples/01_quickstart.mjs` | 连接 → 读状态 → 增量点动 → 回读 → FK |
| `examples/02_offline_sim.mjs` | 静默钳制 / 读失败哨兵 / 25° 自锁 / CRC 错 |
| `examples/03_teleop_network.mjs` | 主从遥操（网口） |
| `examples/04_record_playback.mjs` | 录制 + 回放 |
| `examples/05_bridge_for_browser.mjs` | 起边车并自证浏览器通路（`--fake` 可无硬件） |
| `examples/06_text_exec.mjs` | `TEXT_EXEC` 文本透传：闸门 / 抢引脚命令 / 分页 / 超长护栏 |
| `examples/browser/index.html` | 浏览器页面：连接 / 点动 / 状态（走边车） |

浏览器示例要经静态服务器打开（ESM + 权限限制）：

```bash
cd js && npx serve .          # 或 python -m http.server 8080
# 打开 http://localhost:8080/examples/browser/index.html
# 另开一个终端： node src/bridge.js --target 192.168.7.120:5000
```

---

## 12. 排障速查

| 现象 | 先查 |
|---|---|
| PING 无响应 | ①`link` 偏好是否开到 net（`link net`→`link save`→**RESET**）②目的端口 5000 ③固件是否 `LINK=net\|both` ④网线/IP |
| 命令超时 | 报文长度是否**恰好** `5+LEN`（UDP 多一个字节就被丢） |
| 网页连不上 | 边车起了吗；`--allow` 是否放行；边车目标 IP 对不对 |
| 从臂不动 | `sf` 是否为 1；是否断流 >500ms 自锁了 |
| 大面积 `err=4` | 单帧跳变 >25°；或接近段没收敛；或撞限位 |
| 值写了没生效 | **静默钳制**：回读确认，别信 err |

---

## 13. 目录

```
js/
├── package.json
├── src/
│   ├── protocol.js     逻辑包编解码（真值源）
│   ├── transport.js    四种承载
│   ├── client.js       ArmClient
│   ├── teleop.js       主从遥操
│   ├── teach.js        录制/轨迹/回放
│   ├── kinematics.js   FK/IK/限位
│   ├── console.js      串口文本控制台
│   ├── sim.js          离线假臂
│   ├── bridge.js       ★ WebSocket↔UDP 边车（浏览器用）
│   ├── cli.js          armctl-js
│   └── index.js        统一出口
├── test/               7 组自检（971 项）
├── examples/           Node + 浏览器示例
└── docs/JS快速上手.md   本文件
```
