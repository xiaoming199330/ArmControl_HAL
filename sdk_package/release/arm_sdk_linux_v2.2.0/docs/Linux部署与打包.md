# Linux 部署与打包

---

## 1. 为什么这个包天生适合 Linux

* **核心零依赖**：`protocol / transport / client / teleop / teach / cli` 全部只用标准库。
  `pyserial` 只有串口链路要，`numpy` 只有 `.npz` 要。
* **没有平台相关代码**：路径用 `os.path`，串口名 Linux 是 `/dev/ttyUSB*`（Windows 是 `COM*`），
  URI 形式统一。
* **网口优先**：不需要 udev 规则、不需要 CAN 驱动，只要有 IP 层就行。
* **模拟器内置**：`sim://` 让上层应用在**没有硬件的 CI/容器里**也能跑完整测试。

---

## 2. 三种安装方式

### 2.1 直接跑（不改环境，最省事）

```bash
cd sdk_package
export PYTHONPATH=$PWD
python -m arm_sdk info
python -m arm_sdk --uri net://192.168.7.120:5000 ping
python tests/integration_test.py
```

### 2.2 pip 可编辑安装（开发用）

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -e ".[serial]"
armctl info
```

### 2.3 打 wheel / sdist（分发用）

```bash
pip install build
python -m build                 # 产出 dist/arm_sdk-2.0.0-py3-none-any.whl
pip install dist/arm_sdk-2.0.0-py3-none-any.whl
armctl info
```

> 装完 `armctl` 就在 PATH 里；配置镜像随包安装（`arm_sdk/arm_config.json`）。

### 2.4 离线部署（目标机不能上外网）

```bash
# 有网机器
pip download ".[serial,numpy]" -d wheels/
tar czf arm_sdk_src.tgz sdk_package/
# 目标机
pip install --no-index --find-links wheels arm_sdk
```
零依赖版本连 `wheels/` 都不用：直接 `PYTHONPATH=sdk_package` 就能跑网口 + 遥操 + 示教。

---

## 3. 目标机前置

| 项 | 检查 | 说明 |
|---|---|---|
| Python | `python3 -V`（≥3.8） | 用了 `dataclasses` / f-string / `from __future__` |
| 网口 IP | `ip addr` | 板子默认 `192.168.7.120`，PC 要在同网段（如 `192.168.7.10/24`） |
| 连通性 | `ping 192.168.7.120` | 板子**不一定回 ICMP**（固件会改写 ICMP 校验和，见红线 65/66）⇒ 用 `armctl ping` 判 |
| 串口权限 | `ls -l /dev/ttyUSB*` | 需要 `dialout` 组：`sudo usermod -aG dialout $USER`（重新登录生效） |
| 防火墙 | `sudo ufw status` | UDP 5000 出站一般不受限；若是**监听侧**才要放行 |
| CAN（可选） | `ip link set can0 up type can bitrate 500000` | 只有走 SocketCAN 才需要 |

### 串口 udev 固定名字（可选，避免 `ttyUSB0/1` 漂移）

```bash
# /etc/udev/rules.d/99-arm.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="<主臂适配器序列号>", SYMLINK+="ttyArmLeader"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="<从臂适配器序列号>", SYMLINK+="ttyArmFollower"
```

```bash
sudo udevadm control --reload && sudo udevadm trigger
# 用 serial:///dev/ttyArmLeader
```

> 主从遥操走**网口**时完全不需要这些；只有 PC 中转（串口）路线要。

---

## 4. 网络调优（50 Hz 遥操）

UDP 50 Hz = 100 包/秒，带宽压力可以忽略；要关注的是**抖动**。

```bash
# 1. 看有没有丢包/延迟抖动
armctl --uri net://192.168.7.120:5000 ping --repeat 200

# 2. 若用 USB 网卡，关掉节能（避免 USB autosuspend 造成周期性卡顿）
sudo ethtool --set-eee eth0 eee off 2>/dev/null || true
# 或内核参数：usbcore.autosuspend=-1
```

Python 侧的抖动对策已经在 `TeleopSession.run()` 里：用**补偿式休眠**
（`next += period`，落后就放弃追帧）而不是固定 `sleep(1/hz)` —— 后者实测只能跑到 ~35 Hz。

Linux 上还建议：

```bash
# 降低调度延迟抖动（可选）
sudo nice -n -5 python3 your_teleop.py
# 或 taskset 把遥操进程绑到一个核
taskset -c 2 python3 your_teleop.py
```

---

## 5. systemd 常驻服务（可选）

```ini
# /etc/systemd/system/arm-teleop.service
[Unit]
Description=Arm six-axis teleop (leader->follower over UDP)
After=network-online.target

[Service]
Type=simple
User=robot
WorkingDirectory=/opt/arm_sdk
Environment=PYTHONPATH=/opt/arm_sdk
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/python3 -m arm_sdk teleop \
  --leader net://192.168.7.121:5000 \
  --follower net://192.168.7.120:5000 \
  --hz 50 --slew 240
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now arm-teleop
journalctl -u arm-teleop -f
```

> ⚠ 遥操服务化之前必须确认 **E-Stop 通路**和"人不在扫过区"。
> 固件 E-Stop **不自锁**，服务挂了不会自动停从臂。

---

## 6. Docker / 容器

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY sdk_package /app
ENV PYTHONPATH=/app PYTHONUNBUFFERED=1
# 只用网口 + 模拟器时：不需要任何 pip install
ENTRYPOINT ["python3", "-m", "arm_sdk"]
CMD ["--uri", "net://192.168.7.120:5000", "preflight"]
```

* **网口用 `--network host`**（UDP 回包要回到来源地址；NAT 会造成回包丢失）。
* 串口要 `--device /dev/ttyUSB0`。
* `sim://` 在容器里可跑全套离线自检 ⇒ 适合 CI。

```bash
docker build -t arm-sdk .
docker run --rm --network host arm-sdk --uri net://192.168.7.120:5000 preflight
docker run --rm arm-sdk --uri sim:// preflight          # 无硬件冒烟
docker run --rm arm-sdk selftest                        # 全套离线自检
```

---

## 7. CI 建议

```yaml
# .github/workflows/ci.yml 的要点
- run: python -m compileall -q sdk_package/arm_sdk
- run: python -m arm_sdk.protocol        # 38 项协议字节级断言
- run: python -m arm_sdk.teleop          # 27 项遥操逻辑断言
- run: python -m arm_sdk.teach           # 43 项示教/回放断言
- run: python sdk_package/tests/integration_test.py   # 51 项端到端（sim）
- run: python tools/check_arm_config.py  # 配置镜像 vs 固件源码 漂移检查
```

全部**不需要硬件**。硬件相关的只在人工联调时跑。

---

## 8. 目录结构（打包后）

```
sdk_package/
├── pyproject.toml               # pip 打包 + armctl 入口
├── README.md                    # 总入口
├── arm_sdk/                     # ★ 包本体
│   ├── __init__.py              # 统一导出
│   ├── __main__.py              # python -m arm_sdk
│   ├── protocol.py              # 逻辑包 / CRC / 定点 / 三角载 / 标定
│   ├── transport.py             # Transport 抽象 + UDP/串口 + 工厂
│   ├── sim.py                   # 内存假臂（含全部固件语义与坑）
│   ├── client.py                # ArmClient（SEQ 配对 / RX 泵 / 41 条 API）
│   ├── kinematics.py            # FK/IK 参考实现
│   ├── console.py               # 串口文本控制台（含 #P 位置流）
│   ├── teleop.py                # TeleopSession / FirmwareLink / PcRelay
│   ├── teach.py                 # Trajectory / 录制 / 回放 / FirmwareTeach
│   ├── cli.py                   # armctl
│   └── arm_config.json          # 配置镜像（随包安装）
├── tests/integration_test.py    # 离线端到端
├── examples/                    # 01~07 可运行示例
├── docs/                        # 网口协议速查 / 主从遥操与示教 / 本页
├── config/  data/               # arm_config.json（同一份副本）
├── model/                       # URDF + STL
├── python/                      # 旧版「仅串口文本」SDK（保留兼容）
├── web/  tools/                 # 浏览器端 SDK / 模型生成工具
└── 0X_*.md                      # 上位机任务书（架构/尺寸/接口/规格/验收）
```

---

## 9. 升级 / 回滚

本工程**无版本控制**，所以：

```bash
# 升级前先快照（长文档是单点资产，曾经丢过一份 1125 行文档）
tar czf arm_sdk_snapshot_$(date +%Y%m%d_%H%M).tgz sdk_package config

# 记录当前固件指纹
armctl console --port /dev/ttyUSB0 --cmd h | head -3   # 存 fw-tag
armctl console --port /dev/ttyUSB0 --cmd zi            # 存零点点位
```

回滚 = 解回 tar 包 + 重新 `armctl preflight`。

---

## 10. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `armctl ping` 超时但 `ping` IP 通 | 板子会改写 ICMP 校验和（红线 65/66）⇒ 别用 ICMP 判；用 `armctl ping` |
| `armctl ping` 超时且 `h` 里 `active=off` | `link net` → `link save` → **RESET** |
| 串口 `Permission denied` | 加入 `dialout` 组后重新登录 |
| 50 Hz 实际只有 ~35 Hz | 用 `run()` 而不是自己 `while + sleep(1/hz)`（补偿式休眠） |
| 容器里收不到 UDP 回包 | 用 `--network host` |
| `armctl play` 全是 `ack_limit` | 起点太远（`approach` 失败）或轨迹单帧跳变 >25° ⇒ 先 `traj info` |
| `ModuleNotFoundError: arm_sdk` | 没设 `PYTHONPATH` 也没 pip 安装 |
