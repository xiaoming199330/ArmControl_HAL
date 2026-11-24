# 舵机 ID 检测识别代码说明

> 工程：`ArmControl_HAL（1）`
>
> 协议：Dynamixel Protocol 2.0
>
> 硬件：STM32F407ZGT6 + 6× XL320，双总线（USART3 + UART5）



---

## 1. 总体调用链

```
串口 "n" 命令 / 上电启动
  → cmdScan() / main()
    → ScanBothBuses()
      → DXL_ScanBus(BUS_A, ...)   遍历 ID 0~252 逐个 Ping
      → DXL_ScanBus(BUS_B, ...)
        → DXL_Ping(bus, id, 80ms)  底层：组 PING 包 → 收发 → 解析
          → DXL_BuildPacket(id, INST_PING, ...)
          → DXL_TransmitReceive(...)
          → DXL_ParsePacket(...)
```

---

## 2. 三层代码位置

| 层级        | 文件                              | 关键函数/变量                                        |
| --------- | ------------------------------- | ---------------------------------------------- |
| 协议层       | `Core/Src/dynamixel_protocol.c` | `DXL_Ping()`、`DXL_ScanBus()`                   |
| XL320 驱动层 | `Core/Src/dynamixel_xl320.c`    | `JOINT_ID[]`、`JOINT_BUS[]`、`XL320_InitJoint()` |
| 应用层       | `Core/Src/main.c`               | `ScanBothBuses()`、`cmdScan()`                  |

---

## 3. 协议层：`dynamixel_protocol.c`

### 3.1 DXL_Ping（第 117 行）

对单个 ID 发送 Protocol 2.0 PING 指令，等待状态包回复。

```c
HAL_StatusTypeDef DXL_Ping(uint8_t bus, uint8_t id, uint32_t timeout_ms) {
    /* Ping instruction: 无参数，Length=3 */
    uint8_t tx[16];
    uint16_t tx_len = DXL_BuildPacket(id, INST_PING, NULL, 0, tx);

    /* Ping 状态回复：Model Number(2) + Firmware Version(1) = 3 参数
       Length = 1+1+3+2 = 7，线路上 = 7 + 7 = 14 字节 */
    uint8_t rx[32];
    HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len, rx, 14, timeout_ms);
    if (st != HAL_OK) return st;

    DXL_Packet_t pkt;
    if (!DXL_ParsePacket(rx, 14, &pkt)) return HAL_ERROR;
    if (pkt.id != id) return HAL_ERROR;
    /* Alert(0x80) = 硬件警告，数据仍有效，不拒绝 */
    if ((pkt.error & 0x7F) != 0) return HAL_ERROR;
    return HAL_OK;
}
```

- 返回 `HAL_OK` = 该 ID 的舵机在线
- 返回 `HAL_TIMEOUT` / `HAL_ERROR` = 不在线或通信异常
- `Alert(0x80)` 位不拦截（硬件警告但数据有效）

### 3.2 DXL_ScanBus（第 269 行）

遍历 ID 0~252 逐个 Ping，收集所有在线舵机的 ID。

```c
uint8_t DXL_ScanBus(uint8_t bus, uint8_t* found_ids, uint8_t max_ids) {
    uint8_t count = 0;
    uint8_t tx[16];

    for (uint8_t id = 0; id <= 252; id++) {
        if (id == DXL_BROADCAST_ID) continue;   /* 跳过 0xFE 广播地址 */

        uint16_t tx_len = DXL_BuildPacket(id, INST_PING, NULL, 0, tx);
        uint8_t rx[32];

        HAL_StatusTypeDef st = DXL_TransmitReceive(bus, tx, tx_len,
                                                  rx, 14, 80);  /* 80ms 超时 */
        if (st != HAL_OK) continue;             /* 无回复 = 不在线 */

        DXL_Packet_t pkt;
        if (!DXL_ParsePacket(rx, 14, &pkt)) continue;
        /* Alert(0x80) = 硬件警告，Ping 回复仍有效 */
        if ((pkt.error & 0x7F) != 0) continue;

        if (count < max_ids) {
            found_ids[count] = id;              /* 记录找到的 ID */
        }
        count++;
    }

    return count;
}
```

- 每个地址 80ms 超时，252 个地址全扫完约 20~30 秒（视在线数量而定）
- 返回值为找到的舵机总数

---

## 4. XL320 驱动层：`dynamixel_xl320.c`

### 4.1 关节到舵机的映射表（第 12~13 行）

```c
const uint8_t JOINT_ID[6]  = {1, 2, 3, 4, 5, 6};
const uint8_t JOINT_BUS[6] = {BUS_A, BUS_A, BUS_A, BUS_B, BUS_B, BUS_B};
```

| 关节 | 数组索引 | 舵机 ID | 所在总线  | 物理外设               |
| -- | ---: | ----: | ----- | ------------------ |
| J1 |    0 |     1 | BUS_A | USART3 (PB10/PB11) |
| J2 |    1 |     2 | BUS_A | USART3             |
| J3 |    2 |     3 | BUS_A | USART3             |
| J4 |    3 |     4 | BUS_B | UART5 (PC12/PD2)   |
| J5 |    4 |     5 | BUS_B | UART5              |
| J6 |    5 |     6 | BUS_B | UART5              |

### 4.2 XL320_InitJoint 中的 Ping（第 94 行）

初始化关节前先 Ping，确认舵机存在才继续：

```c
void XL320_InitJoint(uint8_t joint) {
    if (joint >= 6) return;

    uint8_t bus = JOINT_BUS[joint];
    uint8_t id  = JOINT_ID[joint];

    /* 1) Quick ping: skip if servo not on bus */
    if (DXL_Ping(bus, id, 50) != HAL_OK) {
        return;   /* 舵机不在线，跳过初始化 */
    }

    /* 2) Torque OFF (required before EEPROM writes) */
    ...
}
```

- 超时设为 50ms（比扫描时的 80ms 更短，因为这里 ID 是已知的）
- 不在线时静默跳过，不报错

---

## 5. 应用层：`main.c`

### 5.1 ScanBothBuses（第 193 行）

扫描两条总线，结果存入全局数组：

```c
static uint8_t    found_a[8], found_b[8];
static uint8_t    count_a, count_b;

static void ScanBothBuses(void) {
    count_a = DXL_ScanBus(BUS_A, found_a, 8);
    HAL_Delay(100);
    count_b = DXL_ScanBus(BUS_B, found_b, 8);
}
```

### 5.2 cmdScan（第 887 行）

串口 `n` 命令的处理函数，打印扫描结果：

```c
static void cmdScan(void) {
    cprint("\r\nScanning buses...\r\n");
    ScanBothBuses();
    cprint("  Bus A (USART3): found %d servo(s)\r\n", count_a);
    for (uint8_t i = 0; i < count_a; i++)
        cprint("    ID=%d\r\n", found_a[i]);
    cprint("  Bus B (UART5):  found %d servo(s)\r\n", count_b);
    for (uint8_t i = 0; i < count_b; i++)
        cprint("    ID=%d\r\n", found_b[i]);
}
```

串口输入 `n` 后的输出示例：

```
Scanning buses...
  Bus A (USART3): found 3 servo(s)
    ID=1
    ID=2
    ID=3
  Bus B (UART5):  found 3 servo(s)
    ID=4
    ID=5
    ID=6
```

### 5.3 调用时机

| 时机   | 位置                   | 说明                    |
| ---- | -------------------- | --------------------- |
| 上电启动 | `main()` 中（第 2052 行） | 自动扫描一次，结果存全局变量供后续逻辑使用 |
| 串口命令 | `n` 命令（第 1177 行）     | 手动触发，打印结果到控制台         |

---

## 6. Ping 包结构（Protocol 2.0）

### 6.1 发送包（PING 指令）

```
FF FF FD 00  ID  03 00  01  CRC_L  CRC_H
└──头部──┘  └ID┘ └长度┘ └指令┘ └─CRC-16─┘
```

| 字段          | 值             | 说明                    |
| ----------- | ------------- | --------------------- |
| Header      | `FF FF FD 00` | Protocol 2.0 固定头      |
| ID          | 目标 ID         | 0~252                 |
| Length      | `03 00`       | 小端，=3（指令1 + CRC2）     |
| Instruction | `01`          | INST_PING             |
| CRC         | 2 字节          | CRC-16/IBM，多项式 0x8005 |

### 6.2 状态回复包

```
FF FF FD 00  ID  07 00  00  Model_L  Model_H  FW  CRC_L  CRC_H
└──头部──┘  └ID┘ └长度┘ └Err┘ └──Model Number──┘ └FW┘ └─CRC─┘
```

| 字段               | 说明                                 |
| ---------------- | ---------------------------------- |
| Length           | `07 00` = 7                        |
| Error            | 0x00 = 正常；0x80 = Alert（硬件警告，数据仍有效） |
| Model Number     | XL320 = `0x0E 0x00`（14）            |
| Firmware Version | 1 字节                               |

---

## 7. 上位机实现建议

### 7.1 通过二进制协议 Ping

上位机发送 APROTO `PING (0x01)` 命令，固件返回 `proto_ver` 和 `run_id`。这验证的是"上位机 ↔ STM32"链路，不是"STM32 ↔ 舵机"链路。

### 7.2 通过文本命令扫描

上位机通过 USART6 发送 `n\r\n`，按行读取返回文本，解析 `ID=` 行即可获取在线舵机列表。

### 7.3 直接 Ping 指定舵机

固件二进制协议当前没有"Ping 指定舵机"的透传命令。如果上位机需要验证某个 XL320 是否在线，有两种方式：

1. **文本方式**：发送 `p J\r\n`（J=0~5），如果返回角度则舵机在线，如果返回 `--` 则不在线
2. **二进制方式**：发送 `GET_STATUS (0x02)`，检查返回的关节角度是否为 `0x8000`（读取失败的哨兵值）

### 7.4 修改舵机 ID

固件当前没有封装"修改舵机 ID"的命令。XL320 的 ID 存在 EEPROM 地址 4，修改方法：

1. 每次只接一个舵机
2. 通过文本命令或直接写 EEPROM 地址 4
3. 写入后必须 Reboot 舵机生效

如果需要此功能，建议在固件中新增一个 `SET_SERVO_ID` 二进制命令。
