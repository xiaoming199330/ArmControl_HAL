# 网口 PHY Bring-up 诊断（LAN8720A / RMII / LINK=net）

> 2026-09-20 起 · 2026-09-21 **结案** · 相关：红线 40/44/45/46/47/48/49/51/52/53/54/55/56/57/58/59/60/**61/62/63/64**、`docs/CAN物理层取证_2026-09-17.md`、`tools/arm_net_host.py`
> 现役固件：**`ARM-NET-0920R`**（`outputs/net_api/02_slave_NET.bin`，225896 B，md5 `5f80e695ebf9d5226296166124012474`）— **网口已通 + 横幅不截断 + `nx` 带 ICMP 自检 + 3 个运行期探针/开关（池回读 / `sent` 哨兵 / `hwck`）；其默认 `nx cic 1` 已含 CIC 修复（见 §14.14）**
> （上一版 0920Q md5 `1630a4b6…` 存为 `.prev`；0920P md5 `e76ac3c5…` 存为 `.prev6`）
> 本文结构：§0 一句话 · §1 证据链 · §2 `HAL_ETH_Init` 定案 · §3 B1 开关 · §4 `nt` v1 复盘 ·
> §5 硬件短路清单 · §6 已排除 · §7 下一步 · §8 引脚对照表（板载手册逐脚核对） ·
> §9 PC4/PC5 上电与 PHY 对打 >500 ms（0920I 已修） ·
> §10 代码全链路审计（逐层结论） · §11 `np` 新增"悬空 vs 被驱动"判据（0920J） ·
> §12 `nt` 自检/通路标定方法 · §13 0920K 的 `[6]` 裁决：TS=7 选到 ETRF；0920L 再踩 HAL 宏二次移位；0920M 修复 ·
> **§14 ★ 根因与结案：PHY nRST = PE2，漏了复位脉冲 ⇒ 0920N 修复，UDP 全通**
> （§14.8 ICMP 校验和算错 ⇒ 0920O；§14.9 ICMP 缓冲按"默认包"定尺寸 ⇒ 大 ping 写穿，0920P；§14.10 `ping` 排障决策表） ·
> **§14.11–14.12 ⚠ 两次复核：板子回的 ICMP 不带校验和，而"同一 tag ⇒ 烧没烧进去不可判" —— 先把可观测性修掉（0920Q）**
> **§14.13 ★ 0920R：软件算对了、帧里也对，**上网线却变 0x0000** ⇒ 逐字节定位到"协议感知的改写"，只有 MAC 发送校验和插入（CIC）能解释 ⇒ 上探针 + 反向开关做裁决**
> **§14.14 ★★★ 定案：`dbg-TXD0=00D00000` ⇒ `CIC=0b11`；HAL `ETH_DMATxDescListInit` :3050 无条件置位 + 我们 `Attributes=0` ⇒ HAL 从不改 CIC ⇒ **MAC 一直在改写 ICMP 校验和**。修法 = 强制 `CIC=BYPASS`（0920R 默认已含）；`nx hwck 1` 可让 bug 复现，做 A/B 因果反证**
> **§15 网口怎么下发命令（收尾必读）** · 运动联调继续看 **`docs/机械臂上线联调_分级步骤_2026-09-21.md`**（S0→S7 分级）

---

## 0. 一句话结论（2026-09-21 结案）

**根因：LAN8720A 的 `nRST`(pin15) 由 STM32 的 `PE2` 驱动，而我们项目的 `net_gpio_init()`
从来没有使能 GPIOE、也没有给过复位脉冲** ⇒ PHY 的数字核（含 SMI/MDIO）一直停在复位，
但模拟侧时钟缓冲仍能输出 REF_CLK。

**这一条同时解释了整整一天里所有"自相矛盾"的现象**：

| 现象 | 用"PHY 卡在复位"一句话解释 |
|---|---|
| `nt`：PA1 有 REF_CLK、通路完好 | 时钟缓冲与数字核**可以独立存活** |
| B1 OFF ⇒ PA1 归零 | 时钟确实来自 PHY 的 REFCLKO |
| `np`：AFR 全 = 11，MDIO 上拉在 | 引脚配置本来就没问题 |
| `nm`：32 地址 MDIO **一次都没被拉低** | PHY 数字核没启动，不会驱动 MDIO |
| `nx`：`phy NOT found addr 0` | 同上 |
| `HAL_ETH_Init` 却能过 | MAC 只要 REF_CLK，不要求 PHY 的 SMI 活着 |

**修法（0920N）**：`net_gpio_init()` 里加 `__HAL_RCC_GPIOE_CLK_ENABLE()`，
PE2 配推挽输出，**先拉低 ~55 ms 再拉高 ~10 ms**，之后才进 `HAL_ETH_Init`。
依据是板厂官方 UB 例程 `USER/LAN8720/lan8720.c` 的 `#define LAN8720_RST PE2` + 复位时序。

**烧后实机（2026-09-21）**：

```text
[NET] MAC init ok, scanning PHY (MDIO PA2/PC1)
[NET] LAN8720A found at PHY addr 0 (id 0007.C0F1)
[NET] link up: 100M full-duplex, udp://192.168.7.120:5000 ready
nx: state = RUN (phy found addr 0)  link = UP, 100M full-duplex
```

**PC 侧（192.168.7.1/24）UDP 协议全链路验证**：

```text
PING        → 0x81  proto_ver=1 run_id=7      (链路全通)
GET_STATUS  → 0x82  J1..J6 = 179.5 76.7 270.6 250.6 182.0 147.1 ; read_ok=True
PARAM_COUNT → 24
```

> ⚠ 顺带修掉一个工具误判：`arm_net_host.py --ping` 原来按 `ACK(0x7F)+err` 判，
> 把 PING 响应 `0x81` 的第 2 字节 `run_id=7` 当成 `err=7` 打印。
> 协议规定 PING 是**查询类**，回 `0x81: u8 proto_ver, u8 run_id`（红线 60）。

### 历史结论保留（§13 及以前）

下面 §1~§13 是排查过程，**其中"PA1 有没有 REF_CLK"这条线在 0920M 之后已经全绿**，
早期的"作废读数"标注仍然有效（那是 `nt` 自己三个连环工具 bug 造成的，全部与板子无关）：

---

## 1. 证据链（按"能不能分辨假设"排序）

| # | 观测量 | 手段 | 实测 | 说明 |
|---|---|---|---|---|
| 1 | REF_CLK 有没有**快速边沿** | `nt` `[1][2]` 复位模式（TIM5_CH2 硬件清 CNT） | CNT 最大 **2 拍**，小样本 **512/512** | 有极快信号，CNT 几乎一直被清 0 |
| 2 | 同上，**另一条独立机理** | `nt` `[3][4]` 外部时钟模式1 + DWT 定窗 | **~34 MHz** / 20 ms | 50 MHz 超过 TIM5 外部时钟上限 (~42 MHz) ⇒ 欠采样，读数偏低，但"有高频时钟"成立 |
| 3 | MDIO 线上有没有芯片 | `nm` 软 MIIM，全 32 个 PHY 地址 | 全 `FFFF`，**每帧"被拉低的 bit 数 = 0"** | 18 拍里 MDIO **一次都没被拉低** ⇒ 对端根本没驱动这根线 |
| 4 | MDIO 线本身是不是真实网络 | `nm` 松手回高（DWT CYCCNT） | 132 拍回高 ⇒ **有外部上拉** | 线是"设计出来的"（有上拉电阻），但**这不证明 PHY 活着** |
| 5 | MDC 脚能不能驱动 | `nm` PC1 写 0/写 1 自检 | 通过 | 排除"MCU 侧引脚全坏" |
| 6 | MDC 分频合不合规 | `np` `MACMIIAR` | `CR=4` ⇒ 1.65 MHz | 已符合 LAN8720A 的 MIIM ≤2.5 MHz 上限（0920C 已修） |
| 7 | RMII 引脚映射 | `np` 9 脚 AFR 全 `11` | A 组 PG11/PG13/PG14 | 见红线 40：B 组被 USART3_RX / CAN2 占用，出局 |
| **8** | **MAC 的时钟域能不能走完复位** | **`HAL_ETH_Init()` 返回值 + `DMABMR.SR`** | **一直失败，`SR` 位不自清** | **见 §2：这是"REF_CLK 不可用"的第三种测法** |
| **9** | **B1 开关是不是自变量** | **拨通 B1 前后对比** | **拨通后 Init 从"成功"变"失败"** | **⇒ 通上那条线是"死的"（静止电平），比悬空更糟（见 §3）** |

⇒ **#1 / #2 已在 0920M 上首次真正测量 PA1，结论：PA1 上存在高频时钟，通路完好。**
**#3 / #4 / #8 目前仍然有效**：MDIO 上 PHY 未应答。问题已转到"PHY 为什么不答 MDIO"。
（0920K/0920L 的 PA1 读数因触发源编码 bug 作废，见 §13。）

---

## 2. `HAL_ETH_Init` 失败到底意味着什么（2026-09-20 定案）

**别再猜了 —— 看 ST 源码就能定案。** `Drivers/.../stm32f4xx_hal_eth.c`：

```c
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth)
{
  ...
  HAL_ETH_MspInit(heth);                                    /* 只配 GPIO/时钟 */
  SYSCFG->PMC |= heth->Init.MediaInterface;                 /* 选 RMII */
  SET_BIT(heth->Instance->DMABMR, ETH_DMABMR_SR);           /* ← 383 行: DMA 软复位 */
  tickstart = HAL_GetTick();
  while (READ_BIT(heth->Instance->DMABMR, ETH_DMABMR_SR) > 0U)   /* ← 389 行: 等自清 */
  {
    if (((HAL_GetTick() - tickstart) > ETH_SWRESET_TIMEOUT))     /* 500 ms */
    {
      heth->ErrorCode = HAL_ETH_ERROR_TIMEOUT;
      return HAL_ERROR;                                          /* ← 就死在这 */
    }
  }
  ...
}
```

**三个关键点：**

1. **它一上来就做 DMA 软复位，而这发生在任何 MDIO / PHY 操作之前** ⇒
   `HAL_ETH_Init` 失败**和"PHY 应不应答"无关**。它根本不读 PHY。
2. **ST 官方在该等待处的注释原文**（社区多处引用同一句）：

   > *"The SWR is not performed if the ETH_RX_CLK or the ETH_TX_CLK are not available,
   > please check your external PHY or the IO configuration"*

3. **RMII 模式下，`ETH_RX_CLK` 与 `ETH_TX_CLK` 合起来就是那一根 `REF_CLK`。**

⇒ **`HAL_ETH_Init failed` 等价于 "REF_CLK 不可用"**，与 `nt` 数不到边沿是
**同一结论的两种测法**（一个数引脚频率，一个看 MAC 时钟域能否走完复位）。

### ⛔ 绝对不要加"绕过软复位"的 hack

社区里流传一个"偏方"：把 `HAL_ETH_Init` 里那段等待改成 `return → break`，网口就"通"了。
**在本工程不许这么干**：没有 REF_CLK 时网口物理上**不可能**工作，
绕过它只会把"没时钟"伪装成"初始化成功"，然后让人去查一个不存在的问题。

### 与 `nm` 的关系（别混为一谈）

| 现象 | 独立成因 | 判据 |
|---|---|---|
| `HAL_ETH_Init failed` / `nt` 无边沿 | **没有 REF_CLK** | MAC 时钟域 / PA1 引脚 |
| `nm` 全 32 地址 `FFFF`、低 0 bit | **MDIO 上没有芯片驱动** | SMI 协议线（不经过 B1） |

两者**可以同时成立**（都指向 PHY 死），但**逻辑上互不依赖**：
只要给 REF_CLK 照样能读 MDIO（见 §6 参考案例），所以**修好一个不代表另一个就好了**。

---

## 3. B1 拨码开关：REF_CLK 路径上的自变量

原理图片段：

```
PA1 ──────[ B1 · 1 位拨码开关 ]────── RMII REF CLK
```

- **拨通 = 接通**（1 位拨码只有通/断两态）。丝印通常一侧 `ON`、另一侧 `OFF`；
  **按丝印字样判断，别按位置猜**。
- ⚠ **先断电再拨**：带电拨动时若 PHY 侧在推挽输出、MCU 侧 PA1 是输入，
  瞬间悬空/短接容易打坏 PHY 的 REFCLKO 驱动或 MCU 的输入保护。
- ⚠ 图上是**原理图符号**，不代表实物当前状态 ⇒ **用万用表蜂鸣档量通断**最可靠
  （或量 `PA1` 到 PHY 的 REFCLKO 脚是否导通）。

### 拨通前后的 Init 行为 —— ⚠ 原判读方向**是错的**，2026-09-20 傍晚已更正

**原先本节写的是"接通比悬空更差"，那是建立在"当时 B1 已接通"这个前提上的 —— 而那个前提是错的。**
用户 2026-09-20 18:00 明确澄清："**这才是 B1 真正接通上的，上次是搞错了**"。
把前提换对之后，实测方向**整个反过来**：

| B1 **实际**状态 | `HAL_ETH_Init` | 状态机 | 当时的误判 |
|---|---|---|---|
| **断开**（PA1 悬空） | **一直失败**（`DMABMR.SR` 不自清） | 卡在 `NET_ST_MAC` | 以为是"接通"，于是把"失败"归因给"接通的死线" |
| **接通**（PA1 接到 REF_CLK 节点） | **成功** | 进到 `PHY-scan` | — |

⇒ **所以"接通后 Init 成功"既不能当"REF_CLK 有时钟"的证据，也不能当它的反证。** 理由：
同一份 0920F 固件里，`nt` 用**硬件**（TIM5 复位模式）数 PA1 边沿，两条独立路径**都数不到**
（CNT 涨满 30800 拍）。**两个现象互相矛盾，而 0920F 没有能裁决它们的工具。**

**⇒ 这条矛盾未裁决，必须由 0920J 的 `nt` v2 自检来裁：**

- 自检 ✔（`TIM3_TRGO` @100kHz 阳性对照通过）**且** 边沿数 = 0 ⇒ **尺子可信 ⇒ PA1 确实没有时钟**
- 自检 ✗ ⇒ 整段输出作废，改用示波器；**不许**在此之下写任何结论

> ⭐ 这一节的订正本身就是教训（见红线 51）：那份"行为反转"表当时**看起来非常支持我的假设**，
> 于是被当成了证据写进文档 —— 但它建立在**对开关状态的错误判断**上。
> **证据的价值取决于它前提的真值；前提是"用户口述的开关位置"这种易错输入时，必须先量通断再引用。**

### 由此产生的一个待确认问题

这个网络标号 `RMII REF CLK` **接到 PHY 的哪只脚**？

- 接到 **`REFCLKO`**（PHY 输出）⇒ 开关只管"给不给 MCU"，PHY 自己是独立的
- 接到 **`XTAL1/CLKIN`** 或与晶振/有源晶振**同一节点** ⇒ 这是个**共享时钟节点**，
  断则 **PHY 自己也断粮** ⇒ 可**同时**解释"无时钟 + MDIO 不应答"

**顺带确认：板上有没有 25MHz 晶振、一颗还是两颗。**

---

## 4. `nt` v1 的自相矛盾（必须记下来的一课）

0920F（v1）的 `nt` 在实机上**自己打自己**：

```
[1] CNT 最大 30789 拍        ⇒ 一个上升沿都没有
[3] 相邻上升沿间隔 64 个: min=27 中位=27 max=32 拍   ⇒ 有很规则的沿
```

两者物理上不可能同时成立 ⇒ **至少有一条路本身是坏的**。
病根：`[3]` 用**软件 while 轮询 `CC2IF`** 读捕获值 —— 当信号快过轮询、或**一个沿都没有**时，
数到的"间隔"就是**自己的轮询周期**（≈27 拍，与实测**逐字吻合**）。同族错误见红线 11。

> **后续复核（同一天第二次跑）**：`[3]` 变成 **n=0**，与 `[1][2]` 一致 ⇒
> 上一次的 64 个"规则边沿"确实是**自证**，已被排除。

**v2 的改法**（详见红线 45）：
- `[1][2]` 复位模式（硬件清 CNT）× NOPULL/PULLDOWN 对照，并**丢掉起步 24 个样本**
- `[3][4]` 外部时钟模式1 + **DWT 卡固定时间窗**（只读窗口首尾两次 CNT）⇒ 直接给频率
- `[5]` **自检**：触发源换成已知的 **`TIM3_TRGO` @100kHz** ⇒ **自检不过，整段输出作废**
- 时间基现算（**TIM5 在 APB1 ⇒ 84 MHz**，v1 写死 168MHz，所有换算差 2 倍）

---

## 5. 硬件短路清单（免烧录、万用表即可，按顺序做）

> 一颗 LAN8720A 要**同时**满足下面 4 条才会起振并把 50MHz 送到 PA1。

### ① 先量 VDDCR ≈ 1.2V ★最高优先级
LAN8720A 内部 1.2V 稳压器输出。**量到 0.56~0.76V ⇒ 芯片根本没跑起来**，
连 25MHz 晶振也不会起振 → REFCLKO 必然无输出 → 症状与实测完全一致。
- 常见成因：**`RBIAS`(24 脚) 的 12.1kΩ 下拉缺失**（原理图漏画/漏贴，最常见的坑）、或 `REGOFF`(3 脚) strap 不对。

### ② `nINTSEL`(2 脚) 复位后必须是**低**
用 25MHz 无源晶振、要芯片自己出 50MHz 时，nINTSEL 必须下拉。
**悬空/上拉 ⇒ 芯片按"外部时钟输入 + nINT 中断输出"工作 ⇒ REFCLKO 一个时钟都不出。**
- 很多板把它和 **Speed LED** 复用：LED 不导通时靠内部上拉拉高 ⇒ **每次上电复位都可能锁成错误模式**。
- ⇒ 量这个脚在复位后的电平；必要时 10kΩ 硬下拉（正点原子用跳线帽隔离 LED 就是这个原因）。

### ③ 25MHz 晶振有没有起振
万用表量两端应有 **0.5~1.5V 的直流偏置**（两端略有差异）；有示波器直接看。
- 附带坑：RJ45 那几颗对地电容应为 **~10pF**，错贴成 0.1µF 会让**整个晶振不起振**。

### ④ nRST 没被按住
有的板把 nRST 接到某个 MCU GPIO（甚至接到 MCU 的 NRST）。量它应为高。

### ⑤ 兜底
芯片贴装/虚焊；最后才是**对原理图**确认这板子到底把 PHY 接在哪些脚上。

---

## 6. 已经排除的（别再回头查）

- RMII 发送脚两组之争 → **A 组 `PG11/PG13/PG14`**（B 组 PB11/PB12/PB13 被 USART3_RX / CAN2 占，红线 40）
- MDC 分频超规格 → 已改 `Div102`（1.65 MHz）
- `board_alive.py` 误报 `OLD_FW` → `FW_TAG`→`FW_TAGS` 元组 + 双前缀兜底已修
- "烧录会抹 CAN 配置"类问题 → 本构建与 CAN 无关（`LINK=net` 是独立构建）
- PA1 上"在跳" ⇒ 有信号 → **已证伪**：比例型读数排除不了悬空脚（红线 44）
- `nt` v1 `[3]` 报的"64 个规则边沿" → **已证伪**：那是它自己的轮询周期
- **"`HAL_ETH_Init failed` 是因为 MDIO 读不到 PHY" → 已证伪**：HAL 不读 PHY，
  它死在 DMA 软复位（§2）。所以这条 **不能**当成"PHY 地址不对/走线断"的依据。

---

## 7. 下一步（A 优先；A/B 可并行）

**A. 免烧录：按 §5 量电压/看晶振**（3 分钟，能直接判"芯片死"还是"我们没接对"）。
   拍照也行：PHY 周边（25MHz 晶振、`RBIAS` 电阻、`nINTSEL`/LED 跳线）一张、RJ45 一张、
   **`B1` 拨码开关特写一张**。

**B. 烧 `0920H` 复测**（这一版把 §2 的结论写进了日志）：
```bash
# BOOT0 拉高 → RESET → 烧 → BOOT0 拉回 → RESET
PY="$USERPROFILE/.workbuddy/binaries/python/envs/default/Scripts/python.exe"
"$PY" tools/flash_arm.py --link net --port COM8 --no-check
# 在 COM11(控制台 115200) 敲:
h      # 认指纹: fw-tag ARM-NET-0920H ... clkprobe2,selftest,apb1-84m,init-fail-explain
nt     # 看 [5] 自检是否 ✔；再看 [1]..[4]
nx     # 看状态机停在哪一段
```

**0920H 的日志改进**：`HAL_ETH_Init` 失败时**只打一次**详细解释（含
`st=` / `err=0x…` / `DMABMR.SR=` 三个现场值，并直接写明"⇒ REF_CLK 不可用"），
之后每 20 次重试（约 10 s）报一行"仍未起来"——**不再每 506 ms 刷屏**。

判读：
- `[5] ✗` ⇒ **整段作废**，先 RESET 重跑；仍不过就别用 `nt`，改用示波器。
- `[5] ✔` 且 `[1][2]` 大 + `[3][4]` 0 ⇒ **确证 PA1 无时钟 ⇒ PHY 没工作**（回到 A）。
- `[5] ✔` 且 `[3][4] > 0` ⇒ **PA1 有时钟** ⇒ 转而查 MDC/MDIO 走线（`nm` 已证 MDIO 有上拉但无人应答）。

⚠ `nt` 量测期间 REF_CLK 会被借走，跑完**建议 RESET 一次**再做业务。
⚠ NET 构建里 **`cx` / `ps` / `can` 等已被显式拦掉**，别敲。

---

## 8. 引脚对照表（板载手册 ⇄ 固件，2026-09-20 逐脚核对完毕）

板子自带的引脚表（用户提供，5 张截图）给出了 **9 只**网口相关脚。把这 9 只与
`arm_net.c` 的 `net_gpio_init()`、`gpio.c`、`usart.c`、`tim.c` **逐脚比对：完全一致**。

| # | MCU 脚 | LQFP144 脚号 | 板载网络名 | LAN8720A 脚 | LAN8720A 脚号 | 方向 |
|---|---|---|---|---|---|---|
| 1 | **PA1** | 35 | `RMII REF CLK`（经 **B1** 开关） | `nINT/REFCLKO` | **14** | PHY → MCU |
| 2 | PC1 | 27 | `RMII MDC` | `MDC` | 13 | MCU → PHY |
| 3 | PA2 | 36 | `RMII MDIO` | `MDIO` | 12 | **双向** |
| 4 | PA7 | 43 | `RMII CRS DV` | `CRS_DV/MODE2` | 11 | PHY → MCU |
| 5 | PC4 | 44 | `RMII RXD0` | `RXD0/MODE0` | 8 | PHY → MCU |
| 6 | PC5 | 45 | `RMII RXD1` | `RXD1/MODE1` | 7 | PHY → MCU |
| 7 | PG11 | 126 | `RMII TX EN` | `TXEN` | 16 | MCU → PHY |
| 8 | PG13 | 128 | `RMII TXD0` | `TXD0` | 17 | MCU → PHY |
| 9 | PG14 | 129 | `RMII TXD1` | `TXD1` | 18 | MCU → PHY |

**板载手册直接写成 `RMII TX EN/TXD0/TXD1 = PG11/PG13/PG14` ⇒ 红线 40 的 A 组定案
由板厂文档二次确认**（B 组 `PB11/PB12/PB13` 在本板上被 `USART3_RX` 与 `CAN2` 占掉）。
板载表里 `PG12 = FSMC_NE4`、`PG10 = FSMC_NE3`、`PG15 = DCMI_RESET`，都不是 ETH。

⇒ **结论：引脚这一层已经彻底排除，9 只脚一只不差。** 网口不通与引脚映射无关。

### 顺带排除掉的两个"抢脚"嫌疑

| 嫌疑 | 查证 | 结论 |
|---|---|---|
| **TIM2 的通道输出会不会占住 PA1？**（板载表里 PA1 也标着 `TIM2_CH2`） | `tim.c`：TIM2 走的是 `HAL_TIM_Base_Init`（纯时基），`HAL_TIM_Base_MspInit` 只使能时钟、**不碰 GPIO**；无 PWM/输出通道 | ✅ 排除。控制环用的是**不带引脚**的 TIM2 时基 |
| **USART2 会不会抢 PA2（MDIO）？**（板载表里 PA2 也标着 `U2_TX`） | `usart.c` 文件头注释：*"USART1 and USART2 are NOT used (pins not available on board)"*；`main.c` 里也没有 `MX_USART2_Init` | ✅ 排除 |

> ⭐ 这两条之所以值得记：**板载手册会把同一只脚的所有复用功能都列出来**，
> 于是"手册上写着 `TIM2_CH2` / `U2_TX`"很容易被误读成"被占用了"。
> 手册说的是**这只脚能干什么**，不是**它现在在干什么** —— 后者只能从固件里读。
> （与红线 48 同族：**不能拿"典型画法/功能列表"代替"实际连接/实际配置"**。）

---

## 9. 上电时序里的隐患：PC4/PC5 被配成推挽输出，与 PHY 对打 >500 ms（0920I 已修）

**这是本轮新发现的一个真实缺陷**（不是网口不通的根因，但必须消除）。

`gpio.c` 的 `MX_GPIO_Init()` 里原本**无条件**执行：

```c
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);   /* 先拉低 */
...
GPIO_InitStruct.Pin  = GPIO_PIN_4|GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;                        /* 再配成推挽输出 */
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

在 `LINK=net` 构建里，**PC4/PC5 就是 `ETH_RMII_RXD0/RXD1`，是 PHY 驱动的输入脚**。
调用顺序（`main.c`）：

```
8125  MX_GPIO_Init()          ← PC4/PC5 被配成推挽输出并拉低
8190  MX_TIM2_Init()
...
8226  HAL_Delay(500)          ← 舵机自举等待
...
8345  NetProto_Init()         ← 到这里 net_gpio_init() 才把它们改成 AF11
```

⇒ 从 `MX_GPIO_Init()` 到 `NetProto_Init()` **中间隔着 >500 ms**，这段时间
**MCU 的推挽输出与 PHY 的 RXD0/RXD1 输出对着打**（互相灌电流，且把上电时序搅乱）。

- 为什么一直没被发现：`np` 读 AFR 时**已经是 11**（`NetProto_Init` 之后），
  **"读的时候是对的"掩盖了"中间有一段是错的"**。
- ✅ 修法（0920I）：把这两段用 `#if !ARM_LINK_NET` 包起来，net 构建整段让位给
  `net_gpio_init()` 的 AF11 配置。`LINK=uart` / `LINK=can` 构建行为不变（已回归重编验证）。
- ⚠ **`gpio.c` 是 CubeMX 生成的**：重新生成 / 改 `.ioc` 会**覆盖掉这个守卫**
  ⇒ 生成后必须回来复查。这条已写进文件注释里。

> ⭐ 通用形态：**同一个引脚在两处被配置、靠"后者覆盖前者"来收敛，就是隐患 ——
> 因为"最终值正确"不等于"全过程正确"。** 多构建变体（uart/can/net）下的
> CubeMX 生成文件尤其容易这样：`.ioc` 只描述一种配置，别的变体只能靠"事后改"。
> 判据：**任何引脚只要在不同构建里语义不同，就必须在所有配置点都带构建守卫**，
> 而不只是加在"最后一处"。


---

## 10. 代码全链路审计（2026-09-20，逐层结论）

背景：用户坚持"板子是没问题的，查代码"。于是把 ETH 从 `main()` 到 `HAL_ETH_Init()`
的**每一层**都读了一遍，并逐项给出判据。

| # | 层 | 检查点 | 位置 | 结论 |
|---|---|---|---|---|
| 1 | 引脚 AF | 9 只脚是否都是 `AF11`，PA1 是否为 `AF_PP/NOPULL` | `arm_net.c` `net_gpio_init()` 123–149 | ✅ 正确（`np` 回读 AFR 全 11） |
| 2 | 接口选择 | `SYSCFG->PMC` 的 RMII 位 | `net_gpio_init()` 131 + `HAL_ETH_Init` 内部 | ✅ `rmii_sel=1`；`HAL_ETH_RMII_MODE ≡ SYSCFG_PMC_MII_RMII_SEL`（头文件 404 行核对） |
| 3 | 外设时钟 | ETHMAC / ETHMACTX / ETHMACRX | `net_step_mac()` 368–370 | ✅ 三个都使能（HAL 的 `HAL_ETH_MspInit` 在本工程是 `__weak` 空实现，故必须手动开 —— 已开） |
| 4 | MDC 分频 | `MACMIIAR.CR`（**bit[5:2]**，不是 [2:0]） | `net_step_mac()` 之后 | ✅ `MACMIIAR=0000F890` ⇒ `CR=(0xF890>>2)&0xF=4=Div102`（168/102≈1.65MHz，合 LAN8720A ≤2.5MHz） |
| 5 | 主从链路抢脚 | UART4(PA0/PA1) 是否在 net 构建被跳过 | `main.c` 8198–8217 | ✅ `#if ARM_LINK_NET` 分支**只打印不初始化**，`MX_UART4_Init()` 没被调用 |
| 6 | 定时器抢脚 | TIM2 是否占 PA0–PA3 | `tim.c` 70–88 | ✅ 走 `HAL_TIM_Base_Init`，`HAL_TIM_Base_MspInit` **只使能时钟、不碰 GPIO**，无 PWM 通道 |
| 7 | 上电时序 | PC4/PC5 是否还被 LED 配置对打 | `gpio.c` 81/106 | ✅ 已由 0920I 用 `#if !ARM_LINK_NET` 整段跳过（反汇编验证 `HAL_GPIO_Init` 调用数 net=5 / can=6） |
| 8 | 其它配置点 | 全工程还有谁会写 PA1/PA2/PA7/PC1/PC4/PC5/PG11/PG13/PG14 | 全量 `grep HAL_GPIO_Init/DeInit` | ✅ 只有 CAN 驱动（PA11/PA12、PB8/PB9、PB12/PB13）与 USART（PA0/PA1、PC12/PD2、PB10/PB11），**都不碰 ETH 九脚** |
| 9 | 诊断命令的还原 | `np`/`nm`/`nt` 退出时是否把脚还给 AF11 | `arm_net.c` 837、1077、`pa1_restore_af11()` | ✅ 三个都还原（`np` 原本只读、不动脚；0920J 起它临时借用 PA1 后也还原） |
| 10 | 主臂引脚工具 | `mg`/`mp`/`mo`/`ml` 会不会把 PA1 改成 `AF8_UART4` | `main.c` 2502/2513–2557、2710–2737、2796–2803 | ⚠ **会**（收尾**无条件**写 AF8_UART4）。当前 net 构建是 `ROLE=slave` ⇒ 被 `#if ARM_MASTER_BUILD` 挡住、**已用 `arm-none-eabi-nm build_net/*.elf` 确认没编进来**；但 `make ROLE=master LINK=net` 一组合就全生效。**0920J 已在命令分发处显式挡住**（§10.1） |

**审计结论：代码在"能把 PA1 变成非 ETH"这一点上是干净的。** 上表第 10 项是唯一一条
"能掐死 REF_CLK 的软件路径"，而它当前并未生效。

### 10.1 顺手修掉的两个真问题（0920J）

1. **`mg` / `mp` / `mo` / `ml` 在 net 构建下未被拦住**（只是"恰好没编进来"）。
   `master_gpio_loopback_test()` / `master_pin_probe_mini()` / `master_pin_hold_release()`
   的收尾都是 `Alternate = GPIO_AF8_UART4; HAL_GPIO_Init(GPIOA, PA0|PA1)`
   ⇒ 在 net 构建里敲一次就把 `ETH_RMII_REF_CLK` 换成 `UART4_RX`，
   **现象与"PHY 坏了"一模一样**，且从网络侧怎么查都查不出来。
   现在在 `ProcessCommand` 的 `#if ARM_LINK_NET` 块里显式挡掉并说明原因。
2. `HAL_UART_MspDeInit(UART4)` 里也有 `HAL_GPIO_DeInit(GPIOA, PA0|PA1)`
   （`usart.c` 234）⇒ 运行期任何 `HAL_UART_DeInit(&huart4)` 都会把 PA1 变模拟输入。
   本构建没有这条调用路径，**留档备查**。

---

## 11. `np` 新增"悬空 vs 被驱动"判据（0920J 新增，本条最关键）

### 11.1 先说固件原来那句判读错在哪

`np` 原本的判据是："PA1 连采 2000 次，高/低计数，**恒定** ⇒ 没时钟；**在跳** ⇒ REF_CLK 有"。
**这个判据对两个相反的假设给出同一个读数：**

| PA1 的实际情况 | 2000 次采样里"高"的比例 |
|---|---|
| 真的有 50MHz 方波 | ~50%（采样相位伪随机） |
| **悬空脚**捡到附近走线的耦合噪声 | **~50%（一模一样）** |

⇒ 用户在 COM11 看到 `高 1081 低 919` 就认为"REF_CLK 有"，而 `nt`（硬件边沿计数）说"没有"。
**两个读数矛盾。** 按红线 45，此时第一件事是判"哪把尺子坏了"——而真相是：
**`np` 这一条本来就没有鉴别力，它不是尺子坏了，是它压根不是尺子。**
（这就是红线 44 "在跳 ≠ 有时钟" 的原始案例，却一直印在固件输出里。0920J 已改掉文案。）

### 11.2 新增的三态对照（不用万用表）

把 PA1 临时配成**普通输入**，分别在 内部不拉 / 上拉 / 下拉 三种状态下各采 400 次电平：

| 内部不拉 | 上拉 | 下拉 | 判读 | 指向 |
|---|---|---|---|---|
| 高 | 高 | **高** | 外部**推挽拉高**（40kΩ 内部下拉压不动） | 有源在驱动，但钉在静态高 ⇒ 查 **`nINTSEL`(2脚)**：锁成 1 时 pin14 变开漏 `nINT`，只静态拉高、不输出时钟（且 PHY 整体不跑） |
| 低 | 低 | **低** | 外部**主动拉低** | 查这条网络短路 / 被别的推挽输出占（对原理图） |
| 高 | 高 | **低** | **跟着内部上下拉走 ⇒ 悬空** | **① B1 实际没接通**（断电蜂鸣档量）② 走线断/虚焊 ③ PHY 的 REFCLKO 处于高阻 |
| 混合 | 混合 | 混合 | 节点有活动信号 | 静态判据不适用，改以 `nt` 为准 |

⚠ 它回答的是"**这根脚上有没有外部源在驱动**"，**不回答"有没有 50MHz"**（那归 `nt`）。
**两个问题分开问** —— 这正是 §11.1 那个错误的根源。

### 11.3 现在怎么用（顺序固定）

```bash
# 1) BOOT0 拉高 → RESET → 烧 → BOOT0 拉回 → RESET
& $PY tools\flash_arm.py --link net --port COM8
# 2) COM11 (115200) 敲:
np      # 先看 PA1 三态对照 ⇒ 判定"悬空 / 被驱动" (这一步不用万用表)
nt      # 再看 [5] 自检 + 边沿数 ⇒ 判定"有没有 50MHz"(v2 才带自检)
nx      # 最后看 MDIO 有没有应答
```

`np` 与 `nt` **连起来读**才有结论：

| `np` 三态 | `nt` `[5]` 自检 | `nt` 边沿数 | 结论 | 下一步 |
|---|---|---|---|---|
| 悬空 | — | ~0 | **B1 没真接通 / 走线断** | 断电蜂鸣档量 B1；量 PA1 到 PHY pin14 通断 |
| 被拉高 | ✅ | ~0 | 有源但**静态高** ⇒ 极可能 `nINTSEL` 锁错 | 量 pin2 复位后电平；加 10kΩ 下拉 |
| 被拉高 | ✅ | ≠0 | **REF_CLK 正常** ⇒ 回到 SMI 侧 | 查 MDC/MDIO 走线；冷断电重启 |
| — | ❌ | — | **工具不可信，整段输出作废** | 改用示波器；不要下结论 |

---

## 12. `[5]` 自检的 ITR 接错线 ⇒ 0920K：把"尺子"改成三路互证 + 通路标定

### 12.1 实机现象与判读（2026-09-20 傍晚，用户在 0920J 上跑 `nt`）

```
TIM5 实际时钟 = 84000000 Hz (APB1 定时器时钟; 不是 168MHz) ; DWT=可用
[1] 复位模式 NOPULL   : CNT 最大 27943 拍 ⇒ 沿率 ≤ 3006 Hz ; 小样本 0/512
[2] 复位模式 PULLDOWN : CNT 最大 27941 拍 ⇒ 沿率 ≤ 3006 Hz ; 小样本 0/512
[3] 外时钟1 NOPULL   : 0 个沿 / 20000 µs ⇒ ≈ 0.000 MHz
[4] 外时钟1 PULLDOWN : 0 个沿 / 20000 µs ⇒ ≈ 0.000 MHz
[5] 自检 TIM3_TRGO 100kHz : 0 个沿 / 20000 µs ⇒ ≈ 0.000 kHz  ✗ 不过
结论: ✗ **自检没过 ⇒ 本命令的输出整段作废, 别拿它下任何判断**
```

按固件里写的规则，这段输出整体作废。**但规则里隐含了一个从未被检查过的前提：
"自检本身是对的"。** 这一轮就是去查这个前提 —— 结果是它不成立。

### 12.2 病根：`TIM5` 的 `ITR2` 不是 `TIM3`（纯代码 bug）

`netclk_selftest()` 想用 `TIM3_TRGO` 当已知 100kHz 源，写的是：

```c
netclk_tim_extclk_mode(2U);     /* 注释还写着 "TS=ITR2 (TIM3_TRGO)"  ← 错 */
```

按 **RM0090 Table 76（TIM2..TIM5 内部触发连接）**：

| 从定时器 | ITR0 (TS=0) | ITR1 (TS=1) | ITR2 (TS=2) | ITR3 (TS=3) |
|---|---|---|---|---|
| TIM2 | TIM1_TRGO | TIM8_TRGO | TIM3_TRGO | TIM4_TRGO |
| TIM3 | TIM1_TRGO | TIM2_TRGO | TIM5_TRGO | TIM4_TRGO |
| TIM4 | TIM1_TRGO | TIM2_TRGO | **TIM3_TRGO** | TIM8_TRGO |
| **TIM5** | TIM2_TRGO | **TIM3_TRGO** | **TIM4_TRGO** | TIM8_TRGO |

⇒ **本工程用 TIM5，它的 ITR2 指向 `TIM4_TRGO`；要接 TIM3 必须用 `TS=1`（ITR1）。**
而 TIM4 在本工程**从来没有配置过、连时钟都没使能** ⇒ TRGO 永远不动 ⇒ **自检必然数到 0。**

> ⚠ 隐蔽点：`SMS=111` / `CEN=1` / `TS=2` 全都是**合法寄存器值**，编译器、HAL、寄存器回读
> 都不会报错 —— 它只是**安静地去听一根安静的线**。同一个 `TS=2`，在 TIM4 上是
> `TIM3_TRGO`（对），在 TIM5 上是 `TIM4_TRGO`（错）。**"编号对"不等于"指向对"。**

0 字节的代码改动之外，这件事的**方法论代价**更大：`[5]` 从引入起**一次都没通过过**，
而它的失败被当成了"被测对象无法判定"，**连续几轮没人去审那把尺子**。

### 12.3 0920K 的把尺子结构（三路互证 + 通路标定）

| 项 | 测什么 | 期望 | 不通过时的归因 |
|---|---|---|---|
| `[5]` 自检A | **自由跑 + DWT 定窗**（`SMS=0`，不依赖任何触发源） | ΔCNT ≈ `tim_hz × 窗长µs / 1e6` = **1 680 000** | 定窗或计数器本身有问题 ⇒ 所有窗口读数作废 |
| `[5]` 自检B | 已知 100kHz 的 `TIM3_TRGO` **同时**喂 **TIM5(ITR1)** 与 **TIM4(ITR2)** | 两条都 ≈ 2000 个沿 | 见下方四方对照 |
| `[5]` 旁证 | `TIM3` 自身 CNT 有没有变 | 在跑 | 区分"源没起振"与"通路断" |
| `[5]` AFR | 量测期间 **PA1 的 AFR 回读**（应为 2 = `AF2_TIM5`） | 2 | 证明"借脚"这个动作真的生效 |
| `[6]` 通路标定 | **PA1 → TI2FP2** 这一段（`[1]~[4]` 全都经过它） | 见 §12.4 | **当前唯一未标定的一段** |

`[5]` 自检B 的四方对照表：

| TIM5(ITR1) | TIM4(ITR2) | TIM3 自身 | 结论 |
|---|---|---|---|
| ≈2000 | ≈2000 | 在跑 | **整条计数链路完好** ✔ |
| 0 | ≈2000 | 在跑 | **TIM5 那条 ITR 通路坏** ⇒ 改代码 |
| 0 | 0 | **没在跑** | TIM3 没起振（查 PSC/ARR 与 APB1 时钟） |
| 0 | 0 | 在跑 | TRGO 没出去（查 `TIM3->CR2` 的 `MMS`） |

⇒ 判据从"pass/fail"升级成"**失败时能说清是哪一环**"，正是上一版缺的东西。

### 12.4 `[6]`：为什么必须单独标定"引脚 → TI2"，以及怎么标才安全

`[1]~[4]` 四条**全都**走 `PA1 → TI2FP2 → TIM5` 这**一条**路；而 `[5]` 走的是内部 ITR，
**校准不到引脚这一段**。所以"0 个沿"一直有两种解释：线上真没信号 / 信号进了不定时器 ——
**处置完全相反**。而 `np` 的三态读数（`内部不拉=269 上拉=271 下拉=271`，即**搬不动**）
正在说"这根脚有外部驱动器" ⇒ **与 `nt` 的"无边沿"直接冲突** ⇒ 必须把这截链路测掉。

手法（**电气安全**）：

1. PA1 临时配成 `AF2_TIM5` 的**输入型**通道（`CC2S=01`）⇒ 定时器**不驱动**该脚，
   只有内部 40k 上下拉在起作用 ⇒ **不可能和外部驱动器对着打**（这一步很关键：
   若用输出驱动去驱动一个可能有外部推挽输出的节点，会灌大电流）。
2. 软件来回搬动 PA1 的内部上下拉 200 轮（≈400 次脉冲，每相 ~27µs 建立时间），
   制造一串**已知的人工方波**。
3. **同时**用两条互不依赖的路观测**同一次激励**：`IDR` 软采样（数"电平跟着我们走了几次"）
   与 `TIM5/TI2`（数边沿）。

| `lb_follow` | `lb_ti2` | 结论 | 下一步 |
|---|---|---|---|
| ≈400 | ≈400 | **引脚→TI2 通路完好** ⇒ `[1]~[4]` 成立 | 按 `[1]~[4]` 的结论走 |
| ≈400 | ≈0 | **引脚→TI2 通路坏** ⇒ `[1]~[4]` **全部作废** | 查 AFR 回读是不是 2、`CC2S/CCER` 有没有被别处改（红线 49） |
| ≈200 | ≈0 | **节点被外部驱动器钉住**（不是悬空） | 断电把 **B1 拨到 OFF** → 上电 → 再敲 `nt`（**不用重烧**） |

### 12.5 怎么用（0920K）

```bash
# 1) BOOT0 拉高 → RESET → 烧 → BOOT0 拉回 → RESET
& $PY tools\flash_arm.py --link net --port COM8
# 2) COM11 (115200):
nt        # 先看 [5]自检A / [5]自检B(+旁证) / [5]AFR / [6]通路标定
np        # PA1 三态 + MDIO 原始值（与 [6] 交叉读）
```

连读判据：

| `[5]A` | `[5]B` | `[6]` | 含义 |
|---|---|---|---|
| ✔ | ✔ | 跟着走 + 数到 | **尺子全绿** ⇒ `[1]~[4]` 的读数可以采信 |
| ✔ | ✔ | 跟着走 + **数到 0** | **找到 bug**：引脚→TI2 断 ⇒ 前面所有结论作废 |
| ✔ | ✔ | **搬不动** | 节点被外部驱动 ⇒ 把 B1 拨 OFF 再跑一次（不用重烧） |
| ✔ | ✗ | — | ITR 链路问题 ⇒ 看 TIM4 旁证定位 |
| ✗ | — | — | 定窗/计数器问题 ⇒ 先 RESET 重跑 |

> ⚠ 老规矩：`nt` 量测期间 REF_CLK 被借走，**跑完 RESET 一次**再做业务；
> NET 版**别敲 `cx` / `ps` / `can`**。

### 12.6 现状（诚实版）

- **`nm` 那条证据不受本轮影响**：它是软件 bit-bang 的 SMI 读写（PA2/PC1），
  和 TIM5 / REF_CLK **完全无关** ⇒ "PHY 一次都没驱动过 MDIO"仍然成立。
- **`[1]~[4]` 的"无边沿"暂时挂起**：等 `[6]` 标定完引脚通路才能采信。
- **`HAL_ETH_Init` 成功**这条旁证仍然指向"MAC 时钟域见到了活动"。
- 三条放在一起，最像的解释是"**节点有东西在驱动、但不是能被定时器数出来的那个波形**"
  （例如 `nINTSEL` 锁错 ⇒ pin14 变开漏 `nINT` 后**静态**被外部上拉拉高 —— 但那样
  `np` 的"跟着上下拉走"又解释不通），**所以先不要下判断，先跑 `[5]`+`[6]`。**

---

## 13. 0920K `[6]` 的裁决结果：第二个触发源 bug（2026-09-20 晚）

### 13.1 实机输出（用户在 0920K 上跑 `nt`）

```text
[5] 自检A 定窗+计数器(自由跑) : 数到 1680004 / 期望 1680000  ✔ 通过
[5] 自检B 已知100kHz(TIM5-ITR1): 2000 个沿 / 20000 µs  ✔ 通过
量测期间 PA1 的 AFR 回读 = 2 (期望 2=AF2_TIM5)
[6] 通路标定(引脚→TI2): 上下拉脉冲 400 次, 电平跟着走 201 次, TI2 数到 0 个沿
```

这组数字其实非常干净：

- `[5]A` 过 ⇒ **TIM5 计数器 + DWT 定窗**没问题；
- `[5]B` 过且 TIM4 旁证 2000 ⇒ **TIM3_TRGO → TIM5-ITR1**没问题；
- AFR=2 ⇒ **PA1 真的借给了 TIM5_CH2**；
- `[6]` IDR 跟随 201/400 ⇒ **内部上下拉确实把 PA1 电平搬动了**，不是外部强驱动把脚钉死。

⇒ 剩下的唯一解释就是：**TIM5 没有在听 TI2**。一查代码，正是：

```c
/* 0920K：错误 */
netclk_tim_extclk_mode(7U);   // 作者以为 7 = TI2FP2
netclk_tim_reset_mode(7U);
```

### 13.2 病根：`TS=7` 在 STM32F4 上是 `ETRF`，不是 `TI2FP2`

`Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h` 明确定义：

```c
#define TIM_TS_ITR0          0x00000000U   /* 000 */
#define TIM_TS_ITR1          TIM_SMCR_TS_0 /* 001 */
#define TIM_TS_ITR2          TIM_SMCR_TS_1 /* 010 */
#define TIM_TS_ITR3          (TS_0|TS_1)   /* 011 */
#define TIM_TS_TI1F_ED       TIM_SMCR_TS_2 /* 100 */
#define TIM_TS_TI1FP1        (TS_0|TS_2)   /* 101 = 5 */
#define TIM_TS_TI2FP2        (TS_1|TS_2)   /* 110 = 6 */
#define TIM_TS_ETRF          (TS_0|TS_1|TS_2) /* 111 = 7 */
```

所以 0920K 的 `[1]~[4]` 与 `[6]` **全部在数 ETR 引脚**，而 ETR 在本工程根本没接。
"IDR 跟着走、TI2 一个边沿都没有"不是板子/引脚/AF 的锅，而是**触发源选到了另一根线**。
这也解释了为什么之前 `np` 与 `nt` 看起来互相矛盾：`np` 看的确实是 PA1，`nt` 看的不是。

### 13.3 0920L 修复（但引入第三个 bug：HAL 宏二次移位）

`Core/Src/arm_net.c` 把触发源从裸数字改成 HAL 宏，意图消除"手写 7 选错"的风险：

```c
/* 0920L：意图对，实现错 */
netclk_tim_reset_mode(TIM_TS_TI2FP2);
netclk_tim_extclk_mode(TIM_TS_TI2FP2);
netclk_tim_extclk_mode(TIM_TS_ITR1);   /* TIM5 自检 */
```

但 `netclk_tim_*_mode` 函数内部还在做 `(ts << 4) | SMS`：

```c
static void netclk_tim_extclk_mode(uint32_t ts)
{
    TIM5->SMCR = (ts << 4) | (7U << 0);   /* 0920L 错误：HAL 宏已移位，再移一次 */
}
```

查 `stm32f4xx_hal_tim.h`：

```c
#define TIM_TS_ITR1       TIM_SMCR_TS_0            /* 0x0010, TS=001, 已左移 4 位 */
#define TIM_TS_ITR2       TIM_SMCR_TS_1            /* 0x0020, TS=010, 已左移 4 位 */
#define TIM_TS_TI2FP2     (TIM_SMCR_TS_1|TS_2)     /* 0x0060, TS=110, 已左移 4 位 */
```

所以 `(TIM_TS_TI2FP2 << 4) | 7 = 0x0607`，SMCR 的 TS 字段(bit6:4)实际只取到 `000` = **ITR0(TIM2)**；
`(TIM_TS_ITR1 << 4) | 7 = 0x0107`，TS 字段同样变成 `000` = **ITR0(TIM2)**。

本工程 TIM2 没配 TRGO，0920L 的 `[1]~[4]` 和 `[6]` 实际仍在听一个**静止的 ITR0**。

0920L 的实机输出因此非常"有价值"：

```text
[5] 自检B 已知100kHz(TIM5-ITR1): 0 个沿 / 20000 µs  ✗ 不过
      旁证: TIM4(ITR2) 同一激励数到 2000 个沿 ; TIM3 自身 = 在跑
[6] 通路标定(引脚→TI2): 上下拉脉冲 400 次, 电平跟着走 201 次, TI2 数到 0 个沿
```

- TIM4 直接写 `TIM_TS_ITR2 | TIM_SLAVEMODE_EXTERNAL1`（没二次移位）⇒ 数到 2000；
- TIM5 因为函数里二次移位 ⇒ TS 错成 ITR0 ⇒ 数到 0。

⇒ 这把矛盾**精确锁定到 TIM5 的 SMCR 编码**，而不是"TIM5 芯片坏"或"PA1 通路坏"。

### 13.4 0920M 修复

`Core/Src/arm_net.c` 改成：

```c
static void netclk_tim_reset_mode(uint32_t ts)
{
    TIM5->SMCR = ts | TIM_SLAVEMODE_RESET;       /* HAL 宏不再二次移位 */
}
static void netclk_tim_extclk_mode(uint32_t ts)
{
    TIM5->SMCR = ts | TIM_SLAVEMODE_EXTERNAL1;
}
```

调用点保持不变：

```c
netclk_tim_reset_mode(TIM_TS_TI2FP2);
netclk_tim_extclk_mode(TIM_TS_TI2FP2);
netclk_tim_extclk_mode(TIM_TS_ITR1);   /* TIM5 自检 */
TIM4->SMCR = TIM_TS_ITR2 | TIM_SLAVEMODE_EXTERNAL1;
```

固件指纹升为 **`ARM-NET-0920M`**，`tools/arm_preflight.py` 已加入该指纹。
编译产物已发布：`outputs/net_api/02_slave_NET.bin`

- 大小：**223784 B**
- md5：**`f00f95421c9ce3493d536d07f3cf7bfd`**

### 13.5 实机结果（用户 19:2x 跑 0920M）

```text
[1] 复位模式 NOPULL   : CNT 最大 2 拍 ; 小样本 512/512
[2] 复位模式 PULLDOWN : CNT 最大 2 拍 ; 小样本 512/512
[3] 外时钟1 NOPULL   : 679970 个沿 / 20000 µs ⇒ ≈ 33.998 MHz
[4] 外时钟1 PULLDOWN : 679969 个沿 / 20000 µs ⇒ ≈ 33.998 MHz
[5] 自检A/B: 通过, TIM4 旁证=2000
[6] 通路标定: 电平跟着走 195/400, TI2 数到 1057661 个沿
np: PA1 2000 次采样 高1079/低921 ; MDIO 全 32 地址 FFFF+R
```

判读：

| `[6] follow` | `[6] TI2` | 含义 |
|---|---|---|
| ≈400 | ≈400 | 节点能被 40k 上下拉搬动，TI2 也数到 ⇒ 通路好，但**没有外部强驱动** |
| **很小** | **大量沿** | **PA1 被外部强驱动（REF_CLK 输出），内部 40k 抢不过，但 TI2 通路完全正常** ← 当前状态 |
| ≈400 | ≈0 | 节点能被搬动，但 TI2 不工作 ⇒ TI2 通路真断了 |

结论：**尺子全绿，PA1 上存在高频时钟，PA1→TIM5_CH2 通路完好。**

注意 `[3][4]` 读数只有 ~34 MHz 而不是 50 MHz：TIM5 外部时钟模式1 的输入上限约为
`fCK_INT/2 = 84 MHz / 2 = 42 MHz`（RM0090），50 MHz REF_CLK 会**欠采样/漏计**，
所以绝对频率读数不准；`[1][2]` 的"CNT 最大 2 拍"和 `np` 的 54/46 采样比才是
"有快速时钟"的可靠证据。

### 13.6 当前状态与下一步

- ✅ **PA1 REF_CLK 存在**（`nt` 尺子全绿 + `np` 采样比符合方波）。
- ✅ **PA1→TIM5_CH2 通路完好**（`[6]` TI2 数到大量沿）。
- ⚠️ **MDIO 仍全 32 地址 `FFFF+R`** ⇒ PHY 没有应答 SMI，独立于 REF_CLK 问题。
- 🔍 问题已从"REF_CLK 有没有"转到"**PHY 为什么不答 MDIO**"。

现在做：

1. 跑 `nx` 看 `HAL_ETH_Init` 状态：
   - 若仍 `Init failed / DMABMR.SR timeout` ⇒ REF_CLK 虽然存在，但 MAC 没拿到**稳定**
     的 50 MHz 边沿（例如 PHY 的 REFCLKO 实际没起，34 MHz 是晶振泄漏/谐波）⇒ 继续查
     PHY 供电/nRST/nINTSEL/25 MHz 晶振；
   - 若 `Init OK` ⇒ MAC 侧已好，问题纯在 MDIO/SMI 链路或 PHY 地址/状态。
2. 硬件查 PHY（LAN8720A）：
   - pin6 VDDCR ≈ 1.2 V；
   - pin15 nRST = 高；
   - pin2 nINTSEL = 低（确保 pin14=REFCLKO，不是 nINT）；
   - pin9 PHYAD0 上拉/下拉（决定 PHY 地址 0 或 1）；
   - pin5/pin4 25 MHz 晶振是否起振；
   - MDC/MDIO 走线有没有断/短路/被其他器件占脚。
3. 必要时把 B1 拨 OFF 再跑 `nt`（不用重烧），确认 REF_CLK 是否来自 PHY；若 OFF 后
   `[1]~[4]` 全 0 ⇒ 确认 REF_CLK 确实经过 B1 来自 PHY。

> ⚠ 老规矩：`nt` 量测期间 PA1 会短暂离开 ETH；跑完 `nt` 先 RESET 一次再做业务。
> NET 版**别敲 `cx` / `ps` / `can`**。

---

## 14. ★ 根因与结案（2026-09-21）：PHY `nRST` = `PE2`，漏了复位脉冲

### 14.1 决定性线索的来源

用户提示：`D:/BaiduNetdiskDownload/启明欣欣STM32F407(高配版V6.1)学习资料/.../2 各资源学习例程\标准库版本\29 UDP数据收发实验\`
是**板厂自己验证通过的官方 UDP 例程**。对照它，问题当场落地。

在 `.../29 UDP数据收发实验/USER/LAN8720/lan8720.c` 里：

```c
/* 板厂 V6.1 官方例程 */
#define LAN8720_RST   PE2          /* ETH_RESET --------------------> PE2 */

/* 初始化时主动给复位脉冲 */
LAN8720_RST = 0;
delay_ms(50);
LAN8720_RST = 1;
```

而本工程 `Core/Src/arm_net.c` 的 `net_gpio_init()`（0920M 及以前）只做了：

```c
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOG_CLK_ENABLE();
__HAL_RCC_SYSCFG_CLK_ENABLE();
/* SYSCFG->PMC |= MII_RMII_SEL;  PA1/PA2/PA7, PC1/PC4/PC5, PG11/13/14 配 AF11 */
```

**没有 GPIOE，没有 PE2，没有任何复位动作。**
⇒ PE2 复位后保持**浮空输入**，PHY 的 nRST 端没有确定的释放动作，PHY 数字核不启动。

### 14.2 为什么"时钟活、SMI 死"是自洽的（关键推理）

LAN8720A 内部大致分成两块：

- **模拟/时钟侧**：XTAL 振荡 + REFCLKO 缓冲输出。只要 VDDIO/晶振正常，**即使数字核被复位**，
  这个缓冲仍可能输出 50 MHz；
- **数字核侧**：SMI（MDC/MDIO）状态机、寄存器组、MAC 接口逻辑。**nRST 未释放 ⇒ 整块不工作**。

所以：

- `nt` 看到 PA1 有 REF_CLK ✔（时钟侧活）
- `nm` 看到 32 个地址 MDIO 一次都没被拉低 ✔（数字核死）
- `HAL_ETH_Init` 能过 ✔（MAC 只检查 REF_CLK，不查 SMI）
- `nx` 报 `phy NOT found addr 0` ✔
- **两者并不矛盾** —— 之前的"矛盾"是错误地把 PHY 当成一个整体。

### 14.3 0920N 的修改

```c
/* Core/Src/arm_net.c : net_gpio_init() 新增 */
__HAL_RCC_GPIOE_CLK_ENABLE();

GPIO_InitTypeDef rst = {0};
rst.Pin   = GPIO_PIN_2;
rst.Mode  = GPIO_MODE_OUTPUT_PP;
rst.Pull  = GPIO_NOPULL;
rst.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOE, &rst);

HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   /* nRST 拉低 */
net_delay_ms(55);                                        /* > 官方 50 ms */
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);      /* 释放 */
net_delay_ms(10);
/* 之后才进 HAL_ETH_Init */
```

版本指纹升为 `ARM-NET-0920N`，`tools/arm_preflight.py` 同步加入该指纹。

### 14.4 实机验证（2026-09-21 11:3x）

烧 0920N 后，COM11 上电日志：

```text
[RUN][07][T+00001764][INFO ][NET] MAC init ok, scanning PHY (MDIO PA2/PC1)
[RUN][07][T+00001770][INFO ][NET] LAN8720A found at PHY addr 0 (id 0007.C0F1)
[RUN][07][T+00003354][INFO ][NET] link up: 100M full-duplex, udp://192.168.7.120:5000 ready
```

```text
nx
===== nx NET status =====
state    : RUN (phy found addr 0)
link     : UP, 100M full-duplex
addr     : udp://192.168.7.120:5000  (MAC 02:4E:45:54:00:01)
```

**PC 侧（网卡 `以太网 5` = 192.168.7.1/24）**：

```text
arp -a 192.168.7.120  →  02-4e-45-54-00-01      (ARP 正常解析)
```

```text
python tools/arm_net_host.py --ping
  ✅ PING: 回包 cmd=0x81 proto_ver=1 run_id=7 (链路全通)

python tools/arm_net_host.py --status
  姿态  J1..J6 =  179.5   76.7  270.6  250.6  182.0  147.1
  位姿  X=140.1 Y=0.9 Z=17.5 pitch=145.8  read_ok=True
  标志  E-Stop=False traj=False corr=False

python tools/arm_net_host.py --param-count
  参数个数 = 24
```

⇒ **网口 UDP 承载的整套二进制协议（PING / GET_STATUS / PARAM_COUNT）全部打通，可进入业务联调。**

### 14.5 顺带修掉的工具误判（红线 60）

`tools/arm_net_host.py --ping` 原来写：

```python
ok = rcmd == 0x7F and len(rpl) >= 2 and rpl[1] == 0
print("... proto_ver=%u err=%u" % (rpl[0], rpl[1]))
```

协议规定 `0x01 PING → 0x81: u8 proto_ver, u8 run_id`（LEN=2）。
⇒ 打印出的 `err=7` 其实是 **`run_id=7`**，不是错误码；且该判据**永远判不出"全通"**。
已改为：

```python
ok = (rcmd == 0x81 and len(rpl) >= 2)
print("✅ PING: 回包 cmd=0x%02X proto_ver=%u run_id=%u (链路全通)" % ...)
```

### 14.6 复盘：这一天到底踩了几个坑

| # | 假故障 | 真实原因 | 固件 |
|---|---|---|---|
| 1 | "MDIO 全 FFFF ⇒ 板子/线坏" | 前期判读句无鉴别力 + 未排除 PHY 未复位 | — |
| 2 | "`[5]` 自检不过 ⇒ 时钟没有" | `nt` 把 TIM5 的触发源写成 ITR2(TIM4) 而 TIM4 未配 | 0920K |
| 3 | "`[1]~[4]/[6]` 全 0 ⇒ PA1 没时钟" | `TS=7` 是 ETRF，不是 TI2FP2 | 0920L |
| 4 | "`[5]B` TIM5=0 而 TIM4=2000 ⇒ TIM5 通路坏" | HAL 宏被二次 `<<4`，TS 截断成 ITR0(TIM2) | 0920M |
| 5 | **"PHY 芯片 defective / 建议换板"** | **`net_gpio_init()` 漏了 PHY nRST(PE2) 复位脉冲** | **0920N** |
| 6 | "PING err=7 ⇒ 业务报错" | 工具按 ACK 判查询类响应，把 run_id 当 err | 工具侧修 |

> **#5 是最该记住的一课**：在建议"换板/售后"之前，**必须先拿板厂自己的、已知能通的例程逐项对配置**。
> 板厂例程里那行 `#define LAN8720_RST PE2` 是**板上接线的事实**，任何数据手册都推不出来。
> 对照成本 10 分钟，走了整整一天弯路。

### 14.7 下一步（业务层）

网口通道已可用，接下来按 `sdk_package/03_INTERFACE` 联调：

1. **只读验证**：`--status` 连续读几十次，看丢包率与 `read_ok` 是否稳定；
2. **小动作**：`--move` 单点小角度（先确保机械臂周围无人/无障碍），确认 MOVE_JOINTS 的
   ACK 与实机动作一致；
3. **急停**：`--stop` 必须先验证可用（安全前置）；
4. **长稳**：`nx` 观察 `rx/tx 帧数`、`udp cmd=`、`crc err=`、`tx err=` 是否长期为 0；
5. ⚠ NET 构建下**别敲 `cx` / `ps` / `can`**（已被显式拦掉）；`nt` 量测会让 PA1 短暂离开 ETH，
   跑完先 RESET。

### 14.8 续修：ICMP 应答校验和算错 ⇒ `ping` 永远不通（0920O）

网口通了之后，用户敲 `nx` 看到：

```text
rx/tx : 92/40 帧   udp cmd=34  crc err=0  tx err=0
peer  : arp=3  icmp(ping)=3
```

`icmp(ping)=3` 说明**板子收到了 3 个 echo request**，但 PC 侧 `ping 192.168.7.120`
**100% 超时**。UDP 命令收发完全正常 ⇒ 问题只在 ICMP 应答帧本身。

病根在 `Core/Src/arm_net.c`：ICMP 路径用「复制请求头 + 改两处地址」写法，
但**忘了把要重算的校验和字段先清零**（而 `udp_send()` 是逐字段写、有显式清零）：

```c
/* ✗ 旧代码 */
ic[0] = 0U; ic[1] = 0U;                     /* 注释写"清校验和"，实际只清了 type/code */
uint16_t c  = ip_checksum(ic, ilen);        /* 请求的旧 ICMP 校验和仍在 ic[2..3] */
memcpy(&hdr[14], &p[14], 12U);              /* 旧 IP 头校验和也一起被带进 hdr[24..25] */
uint16_t c2 = ip_checksum(&hdr[14], 20U);
```

求和里混进了旧校验和 ⇒ 算出来的新校验和是错的 ⇒ **包被 PC 协议栈静默丢弃**，
而板侧计数器照常 +1（"看得见计数、看不见回包"的假死）。

**修法（0920O）**：

- ICMP：`ic[2] = ic[3] = 0U;`（连同 type/code 一起清）再算；
- IPv4 头改成**逐字段写**（`Ver/IHL、TOS、TotalLen、ID、flags、TTL、proto、src、dst`），
  校验和字段显式清零，顺带 IHL 固定为 5（无选项，避免请求带 IP 选项时长度对不上）。

**离线自证**（不烧板，用 python 复刻两版算法跑同一个标准请求）：

```text
旧算法    回包 IP 校验=✗无效  ICMP 校验=✗无效
新算法    回包 IP 校验=✔有效  ICMP 校验=✔有效
```

⇒ 红线新增 **61**（算校验和前必须先清校验和字段；"复制+打补丁"最容易漏）。

---

### 14.9 续修二：ICMP 应答缓冲区按"默认包"定尺寸 ⇒ 大 ping 写穿（0920P）

修完校验和后（0920O），把同一段代码拿去做**边界推演**（不改板、纯离线复刻），发现：

```c
static uint8_t f2[34 + 74U];          /* ✗ 74 = 20(IP)+8(ICMP)+32(默认数据)+14(MAC) */
...
uint16_t ilen = (uint16_t)(len - 14U - ihl);   /* ← 直接来自收到的帧长! */
memcpy(&f2[34], ic, ilen);                     /* ← 对面发大包就写穿 */
```

**`ping -l 1000` ⇒ `ilen ≈ 1008` ⇒ 越界约 900 字节**，踩掉相邻静态变量。
默认 32B ping 看不出来，属于"平时没事、一压测就诡异"的类型。

**修法（0920P）**：

```c
static uint8_t f2[NET_BUF_SIZE];                                   /* 与 RX/TX 池同为 1536 */
if ((uint16_t)(34U + ilen) > (uint16_t)sizeof(f2)) return;         /* 兜底 */
```

| 项 | 值 |
|---|---|
| 文件 | `outputs/net_api/02_slave_NET.bin` |
| 大小 | 223936 B |
| MD5 | `e76ac3c5d36217ae8a02becec77bbd1d` |
| 指纹 | `fw-tag ARM-NET-0920P` |
| BSS | 72036 → **73460** B（64KB CCM + 128KB SRAM，余量充足） |

**离线复刻结果**（喂 4 种请求长度）：

```text
ping -l 32    回包 len= 74  IP✔ ICMP✔  写 f2[34:74]   OK
ping -l 0     回包 len= 42  IP✔ ICMP✔  写 f2[34:42]   OK
ping -l 56    回包 len= 98  IP✔ ICMP✔  写 f2[34:98]   OK
ping -l 1000                           写 f2[34:1042]  ⚠ 溢出(旧 f2 只有 108B) ⇒ 0920P 已消
```

⇒ 红线新增 **62**（应答缓冲区按"对端最大能发多少"定尺寸，不按"默认包多大"）。

### 14.10 `ping` 排障决策表（下次直接查）

| `nx` 里看到的 | `ping` 结果 | 含义 |
|---|---|---|
| `arp=0` | 不通 | ARP 就没通 ⇒ 查链路/PC 网卡网段 |
| `arp` 在涨、`icmp=0` | 不通 | 板子**没收到** echo request ⇒ 查 PC 路由/防火墙出站 |
| `arp` 在涨、`icmp` 在涨 | 不通 | 板子**收到了**、回包走不出去或对端丢弃 ⇒ **查校验和/帧长**（红线 61/62） |
| `arp` 在涨、`icmp` 在涨、`tx` 在涨 | 通 | 正常 |

> ⚠ 顺带：Windows 自带防火墙默认允许多数出站，但**若装了第三方安全软件/杀软**，
> 可能拦截本机发出的 ICMP 回包；若上表最后一行成立却仍不通，先在 PC 侧用
> python 原始 socket 抓一次回包验证（比 ping 更直接）。

### 14.11 2026-09-21 复核：现役板仍在发「校验和=0」的 ICMP 回包 ⇒ **板上跑的不是 0920P**

用户报 `ping 192.168.7.120` 仍 100% 超时，且 COM11 敲 `h` **0 字节**。用 `tools/net_capture.py`
（Npcap/NDIS 层，能看见被 Windows **静默丢弃**的坏包）实测：

```text
← 板子发来 192.168.7.120 -> 192.168.7.1 ttl=64 ipid 沿用请求 totlen=60 IP校验✔
  | ICMP type=0 code=0 ident=0x1234 seq=N icmpLen=40 ICMP校验✗(存=0x0000 应为0x3502)
```

⇒ 板子**在回** echo reply，IP 头完全正确（TTL=64 / IHL=5 / IP 校验✔ / ID 沿用请求），
**唯独 ICMP 校验和字段 = 0x0000**（正确 0x3502）⇒ 被 PC 协议栈静默丢弃 ⇒ `ping` 全丢。
这与 §14.8 描述的 0920N 及以前的老 bug **同源**（算校验和没把字段清零/没写对）。

三个对照探针（都用 `tools/net_capture.py`）：

| 探针 | 实测 | 判读 |
|---|---|---|
| `--ihl6`（请求带 IP 选项 IHL=6） | 回包 **ver/ihl=0x45**、totlen=60 | 板子跑的是「逐字段写头 + IHL 固定 5」的**新式**代码（0920O 才引入） |
| `--ping-payload 1000` | 回包 **len=1042**、payload 完整、IP✔ | 回包缓冲 **≥1042 B**，**不是** 0920P 之前的 `f2[34+74]`(108 B)，否则必写穿 |
| `--forge-zero`（伪造「正确校验和恰为 0」的请求） | 回包被判 **ICMP校验✔** | 回包的 type/ident/seq/payload/头部**全对** ⇒ 缺陷被隔离到「**只差校验和字段**」 |

**结论**：板子回包内容**完美**，唯一缺陷是**从不写 ICMP 校验和字段**。而仓库
`outputs/net_api/02_slave_NET.bin`(0920P) 的**源码与机器码双向确认会写**：

```text
arm-none-eabi-objdump -d build_net/ArmControl_HAL.elf   (net_process @ 0x08017334)
  0x080174a8–b4  strb r9,[r7,#0/#1/#2/#3]   ← 先把 type/code/校验和 一起清零
  0x080174c8     bl  ip_checksum            ← c = ip_checksum(ic, ilen)
  0x080174d0–d2  strb r2,[r7,#2] / strb r0,[r7,#3]   ← 把 c 存回 ic[2..3]
  0x0801756a     add r0, r9, #34 → memcpy(&f2[34], ic, ilen)  ← 之后才拷进发送帧
```

⇒ **现役板上跑的 .bin 与仓库里这份 0920P 不是同一份**（§14.12 已用实机 `h` 复核：
板子的 tag **就是** `ARM-NET-0920P`，但它的 ICMP 路径**没有那条 store**）。
多半是**更早的一次 0920P 构建**，或烧写**没整份生效**（详见 §14.12）。

**处置**：见 **§14.12** —— 重烧 `outputs/net_api/02_slave_NET.bin`（**已升为 0920Q**），
`h` 必须显示 `fw-tag ARM-NET-0920Q` 且 fix 清单**三条都完整**；随后 `ping 192.168.7.120` 必通。

⚠ **本次无法远程重烧**：ROM bootloader 要 **BOOT0 跳线拉高 + 手动按 RESET**
（`flash_arm.py` 不驱动复位线）；COM8/COM11 在 `0x7F` 与全部 DTR/RTS 组合下**都不应答**。

⚠ **串口静默 ≠ 板子死**（红线 63）：UDP 侧 `python tools/arm_net_host.py --ping --status`
回 `proto_ver=1 run_id=7` 且有实时关节角 ⇒ 主循环活着、协议栈活着。控制台无输出时应先查
**CH340 的 RX 是否接在板上 PC6（USART6_TX）**、是否共地、是否 115200 ——
而**不要**据此判「固件没烧进去 / 板子坏了」。

---

### 14.12 2026-09-21 二次复核：**「同一 tag ⇒ 烧没烧进去不可判」才是真问题** ⇒ 0920Q

用户回复「已烧录」。复核结果：**回包仍然是 ICMP 校验和 = 0x0000**（`ping` 仍 100% 丢）。
这次控制台通了（上一轮 COM11 被别的终端占着，所以是 0 字节 —— 印证红线 63 那条「串口静默
先查线/占口，别判死」），于是拿到了**板上固件的自述**：

```text
COM11 @115200  →  h
  fw-tag ARM-NET-0920P  phy=LAN8720A(RMII)  udp=192.168.7.120:5000  build=LINK=net  txpins=PG11/PG13/PG14
  fix=net-udp,nocancmds,txpins-pg,mdc-div102,miim-probe32,clkprobe4,selftest,apb1-84m,init-fail-explain,pc45-led-guard,pa1-pull-probe,np-text-fix,ne  ⚠ NET 构建: ...（后面换行了）
```

**⚠⚠ 关键判读：`...np-text-fix,ne` 不是"清单到此为止"，而是「被截断了」。**
`ne` 是 `net-master-cmd-block` 的前两个字母。原因见 `main.c` 的 `cprint()`：
它用 `char buf[256]` + `vsnprintf` 钳位，**单次输出成品 >255 B 就静默截断**（红线 5 / 62）。

复刻 `main.c:4824` 那一段拼接出来的整行 = **368 字节**，前 255 字节正好断在
`...,pa1-pull-probe,np-text-fix,ne` —— **与实机打印逐字节一致**。

**为什么这条"截断"是致命的**（而不是"少看几个字"）：

| 事实 | 后果 |
|---|---|
| 0920J 的横幅整行 **343 B**（也 >255） | J~Q 各版的横幅**前 255 字节完全相同** |
| 前 255 字节里含 tag、不含 `pe2-nrst/icmp-cksum/icmp-buf-size` 这些**区分性 token** | **横幅前缀对 K~Q 没有任何鉴别力** |
| 结论 | 只剩 `fw-tag` 那一行字面量可用；**同一 tag 的两个不同构建，从板上完全区分不出来** |

这正是本轮"烧了却看不出变化"的死结：**指纹不够细 + 指纹本身还被截断。**

**算术上也钉死了"跑的不是仓库这份"**：回包内容 = 请求内容 + type 改 0。
若板子是 §14.8 描述的**旧款**（只清 `ic[0..1]`、把请求的旧校验和一起算进去），
用本机请求的实测值（请求 ICMP 校验和 `0x2d01`，回包应为 `0x3501`）代入旧式公式，
**应得 `0x0800`**；而实测是 **`0x0000`**。⇒ 板上的代码是
**「先把 `ic[2..3]` 清零、算完 `c` 却**没有**写回」**这一种形态，
既不是旧款、也不是仓库里这份（机器码确认它**有**那条 store）。

**0920Q 的四处改动（先修"可观测性"，再谈修 bug）**：

1. **横幅拆成 3 条 `cprint`（各自 ≤255 B）** ⇒ fix 清单**完整可见**，不再截断。
   新增 token `banner-255fix`、`icmp-diag` 作为"这份新固件"的识别点。
2. **tag 升为 `ARM-NET-0920Q`**（`tools/arm_preflight.py` 的 `FW_TAGS` 已加）——
   **这是本轮唯一可靠的"烧进去没有"判据**：同 tag 是判不出来的。
3. **ICMP 自检**：`NetProtoStatus` 增 `icmp_calc / icmp_frame / icmp_ilen`，
   `nx` 多打一行把「**算得**」和「**帧里实际的**」分开报：
   ```text
   icmp自检 : ilen=40  算得=3501  帧里=3501  一致 ✔
   ```
   一次就能分开三种缺陷：**算得==帧里==正确**（通路没问题）/ **算了没进帧**（存写·拷贝）/
   **算得 0xFFFF**（根本没算，长度 0）。
4. **新增 `tools/check_cprint_len.py`**：扫 `main.c` 所有 `cprint(...)`，把相邻字面量拼起来
   按**真实字节**算长度，超 255 就报错。**把红线 5/62 从"靠人记住"变成"跑一下就知道"**
   （自测：喂修改前的 368 B 横幅 → 报错；改后的 109/169/124 B → 通过）。

**实测复核（本机）**：
```text
新固件  outputs/net_api/02_slave_NET.bin   224240 B   md5 1630a4b640b89b84d091282ac176f251
反汇编  net_process @0x0801737c  size 0x294
  0x080174f0–fa  清零 ic[0..3]
  0x08017514–51c ubfx r3,r0,#8,#8 / strb r3,[r8,#2] / strb r0,[r8,#3]   ← 写回校验和(仍在)
  0x080175bc–5c0 ldrb r2,[sl,#36] / ldrb r3,[sl,#37]                    ← 新增自检读帧内字段
存档轮转  02_slave_NET.bin=0920Q / .prev=0920P(e76ac3c5) / .prev6=0920J(2aa87a8b)
```

⚠ **别再用"指令字节指纹"判固件代次**：加了几行诊断后，`ubfx+strb` 的字节序列
**因寄存器分配变化而消失**（本次实测），但那条 store **仍然在**（反汇编可见）。
判"有没有这条修复"要看**反汇编**，不是 grep 字节。

**下一步**：重烧 0920Q（BOOT0 拉高 + RESET；或让我用 `stm32flash -r/-w -v` 直接读回对证再烧）。
烧完在 **COM11** 敲 `h`：
* 显示 **`fw-tag ARM-NET-0920Q`** 且 fix 清单**三行齐全**（末尾 `...,banner-255fix,icmp-diag`）
  ⇒ 这份新固件**确实进去了**；
* 再 `ping 192.168.7.120`；若仍丢，敲 **`nx`** 看 `icmp自检`：
  * `一致 ✔` ⇒ 板子回包没问题，**锅在 PC 侧**（防火墙/安全软件/网络栈）；
  * `不一致 ✗` ⇒ 缺陷在存写/拷贝，**此时才轮到改代码**；
  * `ilen=0` 或没这行 ⇒ 分支没走到（那 `icmp(ping)=` 也不会涨）。
* 若 `h` 还是 `0920P` ⇒ **烧写根本没生效**，走 `stm32flash -r` 读回 `0x08017000` 附近
  与本地 `02_slave_NET.bin` 逐字节对证（BOOT0 拉高 + RESET 后我这边可直接做）。

**通用教训（已升为红线 64）**：**改动固件后必须让"指纹"能区分新旧**——同 tag 的两次
构建等于没有版本号；而且**指纹所在的输出本身不能超限**（`cprint` 255），
否则"能证伪"的能力先被截断掉，人就只能在"没烧进去"和"代码没生效"之间空转。

---

### 14.13 2026-09-21 三次复核（0920R）：**软件侧全对，帧却在上线前被"协议感知地"改写**

**先说结论性的新事实**：0920Q **确实烧进去了**（`h` 回 `fw-tag ARM-NET-0920Q`，fix 清单
三行齐全、末尾 `banner-255fix,icmp-diag`——截断问题已解决，指纹可判了）。**可 `ping` 依旧 100% 丢。**

#### 14.13.1 `nx` 的 `icmp自检` 把软件侧彻底洗清了

```text
  icmp自检 : ilen=40  算得=3500  帧里=3500  一致 ✔
```

两个字段是**两个不同的变量**（`s_icmp_calc` / `s_icmp_frame`，打印处 main.c:5814 核对过没写错），
反汇编也确认：`ubfx/strb.w [r8,#2] / [r8,#3]`（结果存回 `ic`）之后，`ldrb [sl,#36]/[sl,#37]`
才去读 `f2`。⇒ **"算"对、"写回 ic"对、"拷进 f2"对。软件侧没有可挑的了。**

#### 14.13.2 逐字节对帧：只差 ICMP 校验和这 2 字节

```text
6c1ff7681c19 | 024e45540001 | 0800 | 4500 003c 59c1 0000 4001 9136 c0a80778 c0a80701 | 0000 0000 1234 0001 6162…
   ↑PC的MAC         ↑板子MAC            IP 校验 0x9136（手工验算=0x9136，✔）                        ↑ICMP 校验=0x0000 ✗
```

**关键对照**：`hdr[24..25]`（IP 校验）与 `ic[2..3]`（ICMP 校验）都是软件写的，走的还是
**同一次 `memcpy(s_txpool[idx], f2, 74)`**。前者**上了线**（值精确等于软件算的），后者没有。
⇒ 拷贝没坏（前 34 字节全对就证明了），损坏**只发生在 36..37 这 2 字节上**。

#### 14.13.3 这 2 字节不是"内存被写"，而是**只针对 ICMP 的改写**

拿 **UDP 回包**做对照（UDP 帧的 34..37 = 源端口/目的端口，是普通数据）：

```text
板→PC  UDP  bytes[34..37] = 13 88 fa d5   ⇒ 源端口 5000 ✔ 目的端口 64213 ✔   (完好)
板→PC  ICMP bytes[36..37] = 00 00         ⇒ 应为 0x3502                    (被清零)
```

⇒ 同一个池、同一个拷贝、同一个 DMA，**UDP 帧的 36..37 完好** ⇒ **不是通用内存改写**；
只有 ICMP 帧的那个字段被清零 ⇒ **改写着是"认识协议"的**。在 MCU 里，"认识协议又会改写
校验和字段"的只有一处：**ETH MAC 的发送校验和插入（描述符 DESC0 的 CIC 位）**。

#### 14.13.4 但读代码"看起来是关的"——所以要读寄存器

查遍了确实都指向"关"：

| 位置 | 值 | 含义 |
|---|---|---|
| `arm_net.c:491` `mc.ChecksumOffload` | `DISABLE` | MACCR.IPCO 被清 0（link-up 时 628-632 行是 Get→改 2 字段→Set，IPCO 会原样带回 0） |
| `arm_net.c:510` `s_txcfg.Attributes` | `0U` | ⇒ HAL **从不**执行 `MODIFY_REG(DESC0, CIC, ChecksumCtrl)` |
| `arm_net.c:513` `s_txcfg.ChecksumCtrl` | `CIC_BYPASS`(0) | 同上，且 `ETH_DMATxDescListInit` 把 DESC0 清零后只置了 TCH ⇒ CIC=0 |
| `HAL_ETH_Transmit` | 阻塞等 `OWN` 清零才返回 | ⇒ 返回时 DMA 已读完该槽；`tx err=0`、`tx_pkts=11=arp2+icmp7+udp2` ⇒ 11 次发送**全部成功** |

**⇒ 按代码推断，MAC 不该动这 2 字节；可观测事实是它动了。** 这正是红线 40 的老毛病：
**"看起来的配置"不等于"寄存器里的值"**。⇒ 0920R 的做法是**把疑点全部变成可回读的数字**，
而不是继续读代码猜。

#### 14.13.5 0920R = 三个只读探针 + 三个运行时开关

`nx` 多打 6 行（**只读**，不改发送行为）：

```text
  dbg-MACCR: ............  (bit10 IPCO=?)          ← MAC 层校验和卸载的真值
  dbg-TXD0 : ... ... ... ...   (CIC=bit23:22)      ← 发送描述符里 CIC 的真值
  dbg-TXD2 : ... ... ... ...                        ← 发送缓冲地址（对账用）
  dbg-RXD2 : ... ... ... ...                        ← 接收缓冲地址（排除 RX DMA 别名到 TX 池）
  dbg-池回读: slot=? [24..25]=.... [36..37]=.... [38]=..  ← **发送返回后**从 s_txpool 同槽回读
  dbg-开关 : cic-fix=? sentinel=? hwck=?
```

**判读表（一次 `nx` 就能定案）**：

| 现象 | 结论 |
|---|---|
| 池[36..37] **=** 帧里(正确值)，而网线仍是 0 | 帧在内存里是好的 ⇒ **是 MAC 在发送时改的**（硬件插入） |
| 池[36..37] **≠** 帧里（变成了 0） | 拷贝/内存被写 ⇒ 与 MAC 无关，回去查池的写者 |
| `dbg-TXD0` 的 bit23:22 非 0 | CIC 在插入 ⇒ 就是它，`nx cic 1` 应能立刻修好 |
| `dbg-RXD2` 落在 `2000D950~2000F150` | RX DMA 别名进了 TX 池 ⇒ DMA 对打 |

三个**运行时开关**（`nx cic/sent/hwck 0|1`，烧一次即可反复对照）：

* `nx cic 0` —— 复现原始状态（不让 HAL 显式写 CIC）；`nx cic 1`（默认）= 发送前把
  `Attributes |= CSUM` 且 `ChecksumCtrl = BYPASS` ⇒ **强制 HAL 把 CIC 写成 0**，把"默认恰好是 0"
  升级成"显式就是 0"（红线 40 的同族：外设字段没人设 ≠ 默认值一定对）。
* `nx sent 1` —— **哨兵**：ICMP 校验和故意写成 `0xAABB`。若网线上出现 `AABB` ⇒ 软件写的值
  **能**上网线（那正常模式下的 0 就发生在更早环节）；若网上仍是 `0000` ⇒ **离开软件后被协议感知地
  改写**（硬件）。这是"谁改的"的判决实验。
* `nx hwck 1` —— **反向利用**：既然怀疑 MAC 在插校验和，就干脆让它按
  `CIC = TCP/UDP/ICMP 全计算` 自己算——**若假设成立，它会算出正确值 ⇒ `ping` 直接通**；
  假设不成立则网线值不变，也就把"MAC 改写"这条线彻底排除。**一个开关既是诊断也是候选修复。**

**复现步骤**（串口 COM11 敲，配合 PC 侧 `tools/net_capture.py --local 192.168.7.1 --peer 192.168.7.120 --sec 2 --dump`）：

1. `nx` ⇒ 抄 `dbg-池回读` / `dbg-TXD0` / `dbg-RXD2`，同时抓一次包；
2. `nx sent 1` ⇒ 抓包看 36..37 是 `AABB` 还是 `0000`；
3. `nx sent 0`；`nx hwck 1` ⇒ 抓包（及 `ping`）看是否变正确；
4. `nx hwck 0` 恢复（`nx cic 1` 保持默认）。

**产物**：`outputs/net_api/02_slave_NET.bin` = **`ARM-NET-0920R`**，225896 B，
md5 `5f80e695ebf9d5226296166124012474`（`.prev`=0920Q、`.prev6`=0920P）。

**通用教训（已升为红线 65）**：**"软件自检说对、外部观测说错"时，别继续在软件层翻代码**——
①先用**同一帧内的对照字段**把范围从"整条链路"缩到"某几个字节"（本帧里 IP 校验能上线、
ICMP 校验不能，是一对天然对照）；②再**沿数据通路逐跳设可回读探针**（f2 → 返回后回读池 →
描述符 → 寄存器），把"内存被写"和"介质改写"分开；③最好再准备一个**反向利用**的开关，
让"假设成立的修复"和"假设不成立的反证"共用一次烧录。

### 14.14 ★★★ 2026-09-21 定案：`CIC` 是 **HAL 默认开着**的，MAC 一直在改写 ICMP 校验和

0920R 第一次 `nx` 就把 §14.13 的预测**原样命中**（那次决策表第 3 行："`dbg-TXD0` bit23:22
非 0 ⇒ CIC 在插入 ⇒ 就是它"）。**注意这次 `tx=0`（一帧都还没发过），所以读到的就是描述符的
"初始化后原值"——不是我们代码写进去的。**

```text
dbg-MACCR: 0200CA0C            (bit10 IPCO=0)
dbg-TXD0 : 00D00000 00D00000 00D00000 00D00000     <-- 四个都一样
dbg-TXD2 : 00000000 ...        (发送前=0，正常，见下)
dbg-RXD2 : 2000F158 2000F758 2000FD58 20010358     (间距 0x600 = 1536 ✔)
dbg-池回读: slot=0 [24..25]=0000 [36..37]=0000     (还没发过 ICMP，池未动)
```

**`0x00D00000` 逐位拆开，两位都对上了 HAL 源码：**

| 位 | 值 | 含义 | 出处 |
|---|---|---|---|
| bit20 | 1 | `TCH` 第二地址链（`0x00100000`） | `ETH_DMATxDescListInit` :3038 |
| **bit23:22** | **`0b11`** | **`CIC = TCP/UDP/ICMP 校验和全计算`（`0x00C00000`）** | **`ETH_DMATxDescListInit` :3050** |

`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c`：

```c
static void ETH_DMATxDescListInit(ETH_HandleTypeDef *heth)   // :3020
{
  ...
    WRITE_REG(dmatxdesc->DESC0, 0x0U); ... WRITE_REG(dmatxdesc->DESC2, 0x0U);   // :3030-3033 全清零
    SET_BIT (dmatxdesc->DESC0, ETH_DMATXDESC_TCH);                              // :3038
    ...
    /* Set the DMA Tx descriptors checksum insertion */
    SET_BIT (dmatxdesc->DESC0, ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL);           // :3050 ★ 无条件！
}

static uint32_t ETH_Prepare_Tx_Descriptors(...)              // :3122（每次发送都走）
{
  ...
  WRITE_REG(dmatxdesc->DESC2, (uint32_t)txbuffer->buffer);                      // :3147 缓冲地址每发一次写一次
  MODIFY_REG(dmatxdesc->DESC1, ETH_DMATXDESC_TBS1, txbuffer->len);              // :3150 帧长
  if (READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CSUM) != 0U)      // :3152 ★ 守卫！
  { MODIFY_REG(dmatxdesc->DESC0, ETH_DMATXDESC_CIC, pTxConfig->ChecksumCtrl); } // :3154
}
```

**这就是完整的因果链，两头都闭合了：**

1. `ETH_DMATxDescListInit()` **无条件**给每个 TX 描述符 `SET_BIT(..., CHECKSUMTCPUDPICMPFULL)`
   ⇒ 初始化完 `CIC = 0b11`（**不是我们设的，是 HAL 设的**）。
2. 我们 `s_txcfg.Attributes = 0`（`arm_net.c:547`）⇒ `:3152` 的守卫为假 ⇒
   **HAL 一辈子不会去动 CIC 那两位** ⇒ 它就一直停在 `0b11`。
3. ⇒ MAC 在发送时**按帧内容重算** IPv4 校验和 + TCP/UDP/ICMP 校验和并**改写**字段。

**为什么网上是干净的 `0x0000`：** 头文件注释写得很直白 ——
`ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL = "TCP/UDP/ICMP checksum fully in hardware
**including pseudo header**"`（`stm32f4xx_hal_eth.h:1534`）。**ICMP 根本没有伪首部**，
"全计算"模式先**把字段清零**再写回算出的值；对 ICMP 这一步没能正确完成（伪首部口径不适用于 ICMP），
于是**"清零"留下的 `0x0000` 直接上了网线**。这与实测**逐位一致**。

**两个"看起来矛盾"的点，其实都不矛盾：**

| 现象 | 解释 |
|---|---|
| IP 头校验和（`[24..25]`）网上是**对的** | MAC 也重算了 IPv4 头校验和，而它算得对；且软件写的本来就对 ⇒ 二者一致，看不出区别（**所以 IP 侧不是独立证据**，独立证据是描述符那两位） |
| UDP 回包 `[34..37]`（源/目的端口）**完好** | 端口不是校验和字段；我们 UDP 校验和本来就填 0，MAC 的 UDP 插入写在该写的位置 ⇒ 端口字节不动 ✔（与"协议感知改写"完全相容） |
| `dbg-TXD2` 全 0 | `DESC2` 每次发送才由 `:3147` 写入；`nx` 时 `tx=0` ⇒ 读到的是初始化后的 0，**属于正常**，不是缺陷 |
| `dbg-RXD2` 间距 0x600、首值 `2000F158` | RX 描述符初始化**确实跑过**，地址正确 ⇒ 回读可信（TX 表同一套） |

**修复（两种写法等价）：**

* **A（推荐，0920R 的默认已经是它）**：`s_txcfg.Attributes |= ETH_TX_PACKETS_FEATURES_CSUM;`
  `s_txcfg.ChecksumCtrl = ETH_CHECKSUM_DISABLE;`（= `CIC_BYPASS` = `0x00000000`）
  ⇒ 让 `:3154` 的 `MODIFY_REG(DESC0, CIC, 0)` **每次发送都把 CIC 清 0**。
  把"默认恰好是 0"升级成"**每次显式就是 0**"。
* B（一次性）：`ETH_DMATxDescListInit()` 之后自己 `CLEAR_BIT(s_dmtx[i].DESC0, ETH_DMATXDESC_CIC);`。

⭐⭐ **所以：0920R 烧上去之后，`ping 192.168.7.120` 应该直接通**（`s_cic_fix=1` 是它的默认）。

**A/B 反证（把"相关"升级成"因果"）**：`nx hwck 1` ⇒ `ChecksumCtrl = ..._INSERT_PHDR_CALC`
（`0x00C00000`）⇒ CIC 又被显式置回 `0b11`（**等价于 0920Q 的行为**）⇒ 网上应回到 `0x0000`、`ping` 挂；
`nx hwck 0` ⇒ 又通。**能用一个开关让 bug 复现、再让它消失，就排除了"碰巧"**。

**红线 66**：用 HAL 的 `ETH_TxPacketConfigTypeDef` + `Attributes = 0` 时，
**HAL 不会写 `CIC`，但 `ETH_DMATxDescListInit()` 已经把 CIC 置成 `TCPUDPICMP_FULL`**。
⇒ **"我没开" ≠ "它是关的"**。凡是"外设会自己改数据"的功能（**校验和插入/卸载**、CRC、padding、
VLAN、插入/剥离），都必须**回读描述符/寄存器**确认，**不能读自己的 config 结构体**。
（这与红线 40 同族，但更阴：这次被"改"的是**我们发出去的数据本身**，所以表现为"网线上的帧和内存里的不一样"。）

---

## 15. 网口到底怎么下发命令（收尾必读）

> ⭐ **全部 14 条命令的逐字段详解 + 回传注释 + 可直接照抄的 hex 示例**：
> **`docs/网口指令大全_2026-09-21.md`**（本章是速览，那份是权威全集）

⚠ 先记住一条：**网口只承载二进制协议，不承载文本命令。**
所有 `h / s / m / g / P / teach ...` 这类文本命令**只能从 USART6 控制台（COM11, 115200）敲**。

### 15.1 报文格式（与 CAN 版逐字节相同）

UDP → `192.168.7.120:5000`，每包一个「逻辑包」：

```
[ SEQ(1) | CMD(1) | LEN(1) | PAYLOAD(LEN) | CRC16-LE(2) ]
CRC16 = Aproto_Crc16(前 3+LEN 字节)   —— 非反射、MSB-first、init 0xFFFF
```

- **查询类**响应：`CMD | 0x80` + 数据（如 `0x01 PING → 0x81: {proto_ver, run_id}`）
- **控制类**响应：`ACK(0x7F)` / `NACK(0x7E)` + `{原命令码, 错误码}`（错误码见 `ARM_API_ERR_*`）

### 15.2 三条下发通道

| 场景 | 怎么做 |
|---|---|
| **手动点测 / 脚本** | `tools/arm_net_host.py`（见下） |
| **写上位机 / 集成** | 用 `sdk_package/python/` 的 codec，或照 `tools/arm_net_host.py` 复用 `can_host.py` 的编解码 |
| **发文本命令** | 只能走 COM11 控制台，网口不行 |

### 15.3 `tools/arm_net_host.py` 常用命令

```bash
PY="$HOME/.workbuddy/binaries/python/envs/default/Scripts/python.exe"

$PY tools/arm_net_host.py --ping                    # 链路探活 → 0x81 proto_ver/run_id
$PY tools/arm_net_host.py --status                  # 读 6 关节角 + FK 位姿 + 标志
$PY tools/arm_net_host.py --watch 30                # ★连读 30 次: 丢包/时延/read_ok/E-Stop
$PY tools/arm_net_host.py --fk                      # 正解查询 0x21 → 0xA1
$PY tools/arm_net_host.py --torque all 1            # 力矩开关 0x12 (J=all|0..5, EN=0|1)
$PY tools/arm_net_host.py --param-count             # 读参数个数
$PY tools/arm_net_host.py --stop                    # ★ 急停（先测这个！）
$PY tools/arm_net_host.py --delta 0 0 0 0 0 2 --speed 5              # ★ 相对微动: 只动 J6 +2°
$PY tools/arm_net_host.py --move 180 90 270 250 180 150 --speed 20   # 6 关节绝对运动 (deg)
$PY tools/arm_net_host.py --selftest                # 离线编解码自检，不需要板子 (11 项)
```

> ⚠ **点动优先用 `--delta`（相对），不要用 `--move`（绝对）**：`cmdMove()` 里的
> `clamp_all_angles()` 对超限目标是**静默钳制 + 一条 WARNING**，不报错 —— 所以绝对角
> 必须先 `--status` 核对当前角。分级运动联调流程见
> `docs/机械臂上线联调_分级步骤_2026-09-21.md`。

> ⚠ **本节命令按 Git Bash 写。用 PowerShell 的话：`$PY` 要写成 `& $PY`（变量当命令须加调用符），
> 赋值时 `=` 两边不能有空格，路径用反斜杠。** 两种写法对照：

```bash
# Git Bash
cd "/c/Users/Administrator/Desktop/ArmControl_HAL（1）"
PY="/c/Users/Administrator/.workbuddy/binaries/python/envs/default/Scripts/python.exe"
$PY tools/arm_net_host.py --status
```

```powershell
# PowerShell  (推荐: 直接 py -3, 脚本只依赖标准库)
cd "C:\Users\Administrator\Desktop\ArmControl_HAL（1）"
py -3 tools/arm_net_host.py --status
```

```powershell
# PowerShell  (指定隔离解释器时: ①等号两边无空格 ②变量当命令调用要加 & ③路径用反斜杠)
cd "C:\Users\Administrator\Desktop\ArmControl_HAL（1）"
$PY = "C:\Users\Administrator\.workbuddy\binaries\python\envs\default\Scripts\python.exe"
& $PY tools/arm_net_host.py --status
```

| 报错 | 原因 | 修法 |
|---|---|---|
| `无法将"PY=/c/..."项识别为 cmdlet` | bash 的 `变量=值` 写法，`=` 两边还带了空格 | 改 `$PY = "C:\..."`（**无空格**） |
| `意外的标记"tools/arm_net_host.py"` | PowerShell 里变量名不构成命令 | 改 `& $PY tools/...` |

### 15.4 PC 侧网络要求

| 项 | 值 |
|---|---|
| PC 网卡 IP | `192.168.7.1/24`（本机是「以太网 5」） |
| 臂 IP / 端口 | `192.168.7.120:5000` |
| 臂 MAC | `02:4e:45:54:00:01` |
| 链路 | 直连网线即可，**不需要网关、不需要路由** |

### 15.5 实测基线（0920N/O，30 次连读）

```text
30 次 GET_STATUS: ok=30 fail=0，往返 min 2.5ms / avg 11.8ms / max 31.8ms
```

### 15.6 上线顺序建议

1. `--ping` 确认链路；
2. **`--stop` 确认急停可用**（安全前置，务必先做）；
3. `--status` 连读几十次看 `read_ok` 与丢包；
4. `--move` 小角度单点（周围无人/无障碍），比对实机动作；
5. 板侧 `nx` 看 `rx/tx 帧数`、`udp cmd`、`crc err`、`tx err` 是否长期为 0；
6. 最后再上 `pv/pf` 之类高频流。

> ⚠ NET 构建下**别敲 `cx/ps/can/cp/cy/cz/cw`**（已被固件显式拦掉）。
> `nt` 量测期间 PA1 会短暂离开 ETH，跑完先 RESET。
> ⚠ 文本命令走 COM11，**不是**网口。
