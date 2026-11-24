# host_integration — 上位机对接包

给**要写上位机**（WinForm / WPF / Qt / MFC / 组态 / Web）的工程师。
目标：**照着这里能直接把两臂连起来，不用读源码。**

## 文件清单

| 文件 | 用途 | 先看哪个 |
|---|---|---|
| **`上位机对接指南.md`** | ★ **核心**。「我要做 X 功能 → 该发哪条指令」决策表 + 调用序列 + 错误码到 UI 的映射 + 验收自测清单 | **第一个看** |
| `protocol_spec.json` | **机器可读**协议定义（38 条命令 / 载荷字段 / 错误码 / 常量）。做代码生成、CI 校验、`jq` 查询用 | 要写代码生成时 |
| `arm_protocol.h` | **C/C++ 参考实现**（纯 C99，头文件即库）：命令码、CRC16、组包/解包、常用 payload 构造器 | 用 C/C++/Qt 时 |
| `csharp/ArmLink.cs` | **C# 参考实现**（单文件，纯 BCL）：UDP 客户端 + 自动分页的 `Text()` | 用 WinForm/WPF 时 |
| `csharp/ArmLinkDemo.csproj` | C# 冒烟测试工程，`dotnet run -- 127.0.0.1` 就能跑 | — |
| `minimal_client.py` | **零依赖 Python 最小客户端**。故意不 import SDK —— 独立手写实现，可当"标准答案" | 想快速验证/对答案时 |

---

## 不想自己处理 CRC/SEQ/分页？用 HTTP 边车

上面的四份参考实现都要求你自己收发 UDP 二进制包。如果上位机是 **Web / Electron /
脚本 / 快速原型**，更省事的做法是起 **HTTP 边车**（板子上没有 TCP，边车跑在 PC 侧）：

```bash
cd sdk_package
python -m arm_sdk.http_bridge --target 192.168.7.120:5000 --port 8080
# 或 armctl http-bridge --target 192.168.7.120:5000
```

```python
import requests
s = requests.Session()                                # ★ 复用连接，省 51% 延迟
s.post("http://127.0.0.1:8080/api/raw", json={"cmd": 1})              # PING
print(s.get("http://127.0.0.1:8080/api/status").json()["joints_servo_deg"])
print(s.post("http://127.0.0.1:8080/api/text", json={"text": "s"}).json()["text"])
print("链路 RTT:", s.get("http://127.0.0.1:8080/api/health").headers["X-Arm-Rtt-Ms"], "ms")
```

* 16 个端点：`/api/health` `/api/status` `/api/pose` `/api/info` `/api/flags` `/api/limits`
  `/api/joint_detail` `/api/nx` `/api/metrics` + POST `/api/raw` `/api/text` `/api/stream`
  `/api/move_joints` `/api/move_cart` `/api/stop` `/api/reconnect`
* **不改协议、不加语义**：错误码、丢读哨兵（`None` / `-32768`）、静默钳制行为与 UDP 直连**完全一致**
* 每个响应带 `X-Arm-Rtt-Ms` ⇒ 能分开算「链路延迟」与「HTTP 开销」
* ⚠ 边车**没有鉴权**，默认只绑 `127.0.0.1`。**不要改成 `0.0.0.0`。**
* ⚠ **50 Hz 遥操流不要走 HTTP** —— 每帧等应答会把帧率锁死在 RTT 上。
  遥操/急停走 UDP 直连，配置/查询/写参数走 HTTP 边车。数据见 `docs/UDP_vs_HTTP_实测.md`。

## 三条链路，选哪条

| 链路 | 往返 p50 | 什么时候用 |
|---|---|---|
| **UDP 网口** | **1.61 ms** ★ | 默认。50 Hz 有 15× 余量，两臂之间一根线都不用接 |
| CAN 500 kbps | 2.72 ms | 差分总线、抗干扰场景（本包不深入） |
| 串口 USART6 | 4.78 ms | 网口不可用时兜底；也是**唯一能下发全部 124 条文本命令**的路 |

## 10 分钟上手

```bash
# ① 起一台"假臂"（不需要任何硬件，报文与真机逐字节一致）
armctl mock --port 5000 --verbose

# ② 用参考实现连它（换你的语言/代码）
python host_integration/minimal_client.py --ip 127.0.0.1 --smoke
cd host_integration/csharp && dotnet run -- 127.0.0.1

# ③ 真机：把地址换成臂的 IP（端口固定 5000）
python host_integration/minimal_client.py --ip 192.168.7.120 --smoke
```

## 动手前必做的两件事

1. **CRC 算对**：`crc16("123456789") == 0x29B1`。不对就别往下写 —— 后面全是废的。
2. **跑一遍 `--selftest`**：`python host_integration/minimal_client.py --selftest`（16 项）。
   自己的实现跑完也能跟它对照。

## 最容易踩的 5 个坑（详见指南 §2.4）

1. `LEN` 是 **1 字节**（不是 u16），写错 ⇒ 固件**静默丢弃**，只表现为超时
2. `CRC` **小端**存
3. `MOVE_JOINTS` 的 `v/a` 是 **u16 且 ×10**（传 20 → 写 200）
4. `GET_STATUS` 读失败是 **`-32768`**，`temp`/`hw_err` 丢读是 **`0xFF`** —— 不许兜底成 0
5. `TEXT_EXEC` 的正文必须是 **ASCII + 结尾 NUL**，回显要**按字节拼接**再统一 UTF-8 解码

## 一致性保证

| 校验 | 规模 | 命令 |
|---|---|---|
| Python 独立实现 ↔ SDK 逐字节对拍 | **933 项** | 见 `上位机对接指南.md` §7.2 |
| C 头文件语法（`-Wall -Wextra` 零告警） | — | `arm-none-eabi-gcc -std=c99 -fsyntax-only -x c arm_protocol.h` |
| `protocol_spec.json` 与源码同步 | — | `python tools/gen_protocol_spec.py --check` |

## 相关文档（都在本包内）

* `docs/网口协议速查.md` —— 38 条命令逐字节布局（字段级）
* `docs/网口指令大全.md` —— 全量版：TEXT_EXEC 详解 + 故障对照 + 13 条网口红线
* `config/arm_config.json` —— 接口契约（参数表、量纲、命令表）
* `docs/reference/05_HOST_APP_REQUIREMENTS(...).md` —— 上位机功能范围与验收标准
