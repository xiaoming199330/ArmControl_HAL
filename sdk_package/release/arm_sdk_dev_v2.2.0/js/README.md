# arm-sdk-js — 六轴机械臂上位机 SDK（JavaScript）

零依赖。**同一套逻辑包协议**跑在四种承载上：

| URI | 承载 | 跑在哪 | 实测 p50 |
|---|---|---|---|
| `net://<ip>:5000` | UDP | **Node / Electron 主进程** | **1.61 ms** ●首选 |
| `ws://<host>:8765` | WebSocket 桥 | 浏览器 / 渲染进程 | ≈1.61 ms |
| `webserial://` | Web Serial | 浏览器 | 4.78 ms |
| `sim://` | 内存假臂 | 任意（离线开发 / CI） | 0 |

分层与 Python 版 `arm_sdk/` 一一对应，且**逐字节对拍**（404 项跨语言向量）。

---

## 安装 / 运行

无需 `npm install`（零运行时依赖），只要 Node ≥ 18。

```bash
node src/cli.js info
node src/cli.js selftest                                   # 全部离线自检（971 项，约 12s）
node src/cli.js --uri sim:// ping                          # 无硬件冒烟
node src/cli.js --uri net://192.168.7.120:5000 status --json
node src/cli.js move 0,5,0,0,0,0 --delta --v 10 --wait
node src/cli.js --uri net://192.168.7.120:5000 text --cmd s --cmd zi
node src/cli.js --uri net://192.168.7.120:5000 diag --detail
```

### 网口下发任意控制台命令（`text`）

`TEXT_EXEC` 把**整个控制台**搬到了网口：`arm.textExec('s')` 与在 COM11 上敲 `s`
走的是固件里**同一条** `ProcessCommand`，连回显文本都一样。

```bash
node src/cli.js --uri net://192.168.7.120:5000 text --cmd "link net" --cmd "link save"
```

| 情况 | 表现 |
|---|---|
| 124 条控制台命令里的 **113 条** | 正常执行，输出自动翻页取回 |
| 11 条抢 ETH 引脚的（`cx cp cy cz cw ps can mg mp mo ml`） | 固件**接住并把原因当输出回给你**（不是静默失败） |
| `c`（校准向导） | 安全闸门拒绝 —— 交互式 `for(;;)`，从网口进会卡死主循环 |
| 急停中 | 只放行 `s` / `h` |
| 输出 > 2048 B | 末尾追加 `[截断]` 提示（`h` 帮助表 5406 B 就是这种情况） |

想当命令用：

```bash
npm link        # 之后可直接 `armctl-js ...`
```

## 库用法

```js
import { openArm } from './src/index.js';

const arm = await openArm('net://192.168.7.120:5000');
console.log(await arm.ping());

const st = await arm.getStatus();          // joints 是舵机度 0~360；读失败轴 = null
const tgt = st.joints.slice(); tgt[1] += 5;
await arm.moveJoints(tgt, 10);             // ⚠ 点动用增量，别发绝对角
await arm.close();
```

## 浏览器里用网口

浏览器**发不了原始 UDP**（沙箱限制）。起一个边车：

```bash
node src/bridge.js --target 192.168.7.120:5000
```

页面里：

```js
import { openArm } from './src/index.js';
const arm = await openArm('ws://127.0.0.1:8765/');
```

多台臂（每个连接独占一个 UDP socket，回包按源端口隔离）：

```js
const leader   = await openArm('ws://127.0.0.1:8765/?target=192.168.7.121:5000');
const follower = await openArm('ws://127.0.0.1:8765/?target=192.168.7.120:5000');
```

示例页面：`examples/browser/index.html`（需经静态服务器打开）。

## 目录

```
src/
├── protocol.js     逻辑包编解码（真值源；SEQ|CMD|LEN|PAYLOAD|CRC16-LE）
├── transport.js    UdpTransport / WsTransport / WebSerialTransport / （SimTransport 在 sim.js）
├── client.js       ArmClient：SEQ 配对 + 超时重试 + 响应归一化
├── teleop.js       主从遥操
├── teach.js        录制 / 轨迹 / 回放
├── kinematics.js   FK / IK / 限位
├── console.js      串口**文本**控制台行解析
├── sim.js          离线假臂（复刻固件的静默钳制 / 25° 自锁 / 读失败哨兵）
├── bridge.js       ★ WebSocket↔UDP 边车（浏览器用网口）
├── cli.js          armctl-js
└── index.js        统一出口
test/               7 组自检，971 项
examples/           Node 示例 + 浏览器页面
docs/JS快速上手.md   完整文档（含安全约定与排障表）
```

## 安全约定（务必遵守）

1. **点动只用增量** —— 绝对角超限会被固件**静默钳制**，`err=0` 但值被改了。
2. **`err=0` 不代表到位** —— 运动后必须回读角度确认。
3. **读失败是 `null`，不兜底** —— 绝不拿 0 或上次值冒充姿态。
4. **STREAM_JOINTS 单帧单轴跳变 > 25° → 整帧拒收且基线不推进**（自锁），发送端必须限速。
5. **断流 > 500 ms → 从臂退出流模式**，所以频率要 >2 Hz（推荐 50 Hz）。
6. **`MOVE_CART` 会归零 J5/J6** —— 网络遥操请用 `STREAM_JOINTS`。

完整版见 `docs/JS快速上手.md`。
