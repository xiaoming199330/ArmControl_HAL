/*
 * Feetech HAL 适配层。
 * 本文件不处理 Protocol 2.0 数据包，只负责把字节可靠地发送到
 * 总线 A/B。
 * 输出级两种模式 (2026-11-02, 0924AY):
 *   默认 —— 开漏单总线 (AF_OD)，板上无 74HC126D，无需方向切换；
 *   BUS_PP=1 —— 回退推挽 + 74HC126D 方向切换，供仍挂 126 的旧板。
 */
#include "servo_bus_hal.h"
#include "stm32f4xx_hal.h"

/* CubeMX 生成的两个舵机总线 UART 句柄。 */
extern UART_HandleTypeDef huart3;  /* Bus A: USART3 @ PB10/PB11 */
extern UART_HandleTypeDef huart5;  /* Bus B: UART5 @ PC12/PD2 */

void DXL_SetDirection(uint8_t bus, uint8_t dir) {
#if defined(BUS_PP) && BUS_PP
    /* 回退模式 (make BUS_PP=1): 板上仍挂 74HC126D, 靠方向脚切半双工。 */
    if (bus == BUS_A) {
        HAL_GPIO_WritePin(DIR_A_PORT, DIR_A_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR_B_PORT, DIR_B_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    /* 74HC126 enable/disable propagation delay + TXS0108E delay (~25us) */
    for (volatile int i = 0; i < 150; i++);
#else
    /* 默认 (0924AY): 已去掉 74HC126D, 舵机总线走"开漏单总线"。
     * 方向切换这件事本身由 UART 开漏完成 —— 发 0 拉低 / 发 1 释放给上拉,
     * 所以这里不需要任何方向引脚动作, 也不能再去动 PA8/PA10 (已不接芯片)。
     * 收益: 每次事务少两次 DIR 切换 ≈ 省 50µs。
     * 注意: 线上的 TX 回声依然存在 (TX/RX 并在同一条线), 故 drain_echo
     *       按 TX 包长排回声的逻辑保持原样, 一行不改。 */
    (void)bus; (void)dir;
#endif
}

void DXL_FlushErrors(uint8_t bus) {
    UART_HandleTypeDef* huart = (bus == BUS_A) ? &huart3 : &huart5;

    /* Drain stale bytes (up to 8) and clear all error flags.
     * ORE clear = read SR then DR (drains one byte); RXNE drain = read DR.
     * NOTE: only call this where no servo reply can have started yet
     * (i.e. right after TX) — otherwise it may discard a real byte. */
    for (int guard = 0; guard < 8; guard++) {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(huart);
        } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
            (void)huart->Instance->DR;
        } else {
            break;
        }
    }

    /* Clear framing, noise, parity error flags */
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
}

/* ===== 排空"恰好 len 个"回声字节(非阻塞, 立即轮询) =====
 * 74HC126 回环会把刚发的 TX 包原样回灌到 RX —— 回声字节数 = TX 包长 len。
 * 只排 len 个: 若舵机应答紧贴回声开始, 应答头不会被误当回声排掉。
 *
 * ⚠ 关键纪律(2026-08-28 实测回归教训):
 * 绝不能在此处加"等待回声到达的时间窗口"!上一版加了
 *   budget_us = n*8us + 150us 的时间窗口, 而 HAL_GetTick() 粒度只有 1ms,
 *   实际窗口被拉长到 ≈1ms —— 恰好完整覆盖了 STS3215 的应答延迟(几百 us),
 *   把 6 字节【真应答】全部当回声排掉 → DXL_Receive 永远空手 →
 *   两条总线 0/6 全 MISSING(用户实测回归)。
 * 正确做法: 只排"此刻已在 DR 里的字节"(RXNE=1), 没有就立刻 break,
 * 剩下的残留回声交给 DXL_Receive + find_header 去容忍, 真应答必须留给接收器。 */
static void drain_echo(uint8_t bus, uint16_t n) {
    UART_HandleTypeDef* huart = (bus == BUS_A) ? &huart3 : &huart5;

    for (uint16_t k = 0; k < n; k++) {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
            /* ORE = 读 SR 再读 DR(丢 1 字节) —— 回声阶段正是想要的 */
            __HAL_UART_CLEAR_OREFLAG(huart);
        } else if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
            (void)huart->Instance->DR;
        } else {
            break;   /* 没有待排字节 —— 立即退出, 不作任何等待 */
        }
    }

    /* 清残余帧错误标志 */
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
}

HAL_StatusTypeDef DXL_Transmit(uint8_t bus, uint8_t* data, uint16_t len) {
    UART_HandleTypeDef* huart = (bus == BUS_A) ? &huart3 : &huart5;

    /* ===== KEY FIX v2: RE stays enabled during TX =====
     * The old approach (CLEAR RE during TX, re-enable ~50-80us after TC)
     * created a ~110us blind window. STS3215 reply latency can be <100us,
     * so the reply's first byte(s) arrived while the receiver was OFF:
     * either lost, or RE re-enabled mid-byte desynced the start bit —
     * every ping then failed checksum -> 0/6 servos found.
     * The ESP32 tool works precisely because its UART RX is always on.
     *
     * Now RE stays on the whole time. The TX echo (74HC126 loopback) may
     * land in DR / set ORE during transmission, but it is fully drained
     * right after TX by DXL_FlushErrors — the receiver is already live
     * when the servo reply begins. */
    DXL_SetDirection(bus, 1);

    /* Clear stale flags before transmit */
    DXL_FlushErrors(bus);

    /* Transmit the packet */
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, data, len, 100);
    if (status != HAL_OK) {
        DXL_SetDirection(bus, 0);
        return status;
    }

    /* Wait for last byte to fully complete transmission (TC flag).
     * ⚠ 这里原来是**无超时的裸自旋** `while (TC == RESET);`。TC 一旦因为任何
     *   原因不来(总线被拉死 / 收发器异常 / 时钟被关 / 外设被复位), **整个 CPU
     *   就永久停在这一行**: 主循环死、控制台一个字不说、USB 却还在枚举 ——
     *   这正是"静默死亡"的典型形态, 和掉电、CPU 故障在现场无法区分。
     *   `HAL_UART_Transmit` 返回 HAL_OK 时 TC 其实已经置位, 所以正常路径不会被
     *   这里耽误; 有界化只是把"万一不来"从死循环变成一次**可诊断的失败**。 */
    {
        uint32_t tc_start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
            if ((uint32_t)(HAL_GetTick() - tc_start) >= 5U) {
                DXL_SetDirection(bus, 0);
                return HAL_TIMEOUT;   /* 让上层按"这次事务失败"处理, 而不是卡死 */
            }
        }
    }

    /* One byte-time settle (~15us at 1Mbps) so the stop bit fully clears
     * the 74HC126. Buffer disable propagation is only ~10ns. */
    for (volatile int i = 0; i < 300; i++);

    /* Switch to RX mode: DIR=LOW -> TX buffer high-Z, bus released */
    DXL_SetDirection(bus, 0);

    /* Drain exactly the TX echo (len bytes) from DR and clear ORE/FE/NE
     * so the servo reply (which starts later) is received from a clean
     * state — without eating any reply bytes that may already be arriving. */
    drain_echo(bus, len);

    return status;
}

/* ===== Frame receiver (公共实现) =====
 * 逐字节收集, 两种收尾方式:
 *   (a) expect_id != 0xFF: 定位到 `FF FF <expect_id>` 后由 Length 字段
 *       算出整帧长度, 收满立即返回 —— 快路径, 单次事务 ~0.3ms@1Mbps。
 *   (b) expect_id == 0xFF: 只能静默判停 —— 首字节等 timeout_ms(舵机应答
 *       延迟), 后续字节静默 5ms 判停(旧路径, ~5ms/事务)。
 * 找不到匹配帧头时一律退回 (b), 因此不会比旧版更早截断。
 * *out_len 返回实际收到的字节数(可能 < cap)。
 * 返回 HAL_OK 表示收到 >=1 字节, HAL_TIMEOUT 表示一个都没有。
 *
 * Old version used HAL_UART_Receive: it demanded exactly N bytes, aborted
 * on any FE/ORE flag, and one noise byte shifted the whole frame. */
static HAL_StatusTypeDef receive_common(uint8_t bus, uint8_t* data, uint16_t cap,
                                        uint16_t* out_len, uint32_t timeout_ms,
                                        uint8_t expect_id) {
    UART_HandleTypeDef* huart = (bus == BUS_A) ? &huart3 : &huart5;
    uint16_t got  = 0;
    uint16_t want = 0;   /* 已知整帧长度(含前导噪声偏移), 0 = 未知 */

    while (got < cap) {
        /* 首字节: 等满 timeout_ms(舵机应答延迟); 后续字节: 静默 5ms 判停。
         * 1Mbps 背靠背字节间隔仅 10us, 5ms 的静默阈值不会误伤应答;
         * 但能给"回声残留 + 74HC126 传播 + 方向切换抖动"留足缓冲,
         * 防止把连续到达的应答字节误判为帧尾提前截断(旧版 2ms 偶发截断)。 */
        uint32_t budget = (got == 0) ? timeout_ms : 5;
        uint32_t start  = HAL_GetTick();

        for (;;) {
            /* ⚠ 超时检查必须放在**循环最顶上**。原版把它放在最后, 而上面那个
             *   ORE 分支末尾有一条 `continue`, 会**把超时检查整个跳过去** ——
             *   只要 RX 线持续报 ORE (舵机没上电时开漏总线悬空就是一台"字节
             *   制造机"), 这里就是个**死循环**: CPU 永久停在本函数内, 现场
             *   表现同样是"USB 还在枚举、板子一个字不说"。挪到顶上之后,
             *   任何分支都不可能绕过超时。 */
            if ((HAL_GetTick() - start) >= budget) {
                goto done;
            }
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
                data[got++] = (uint8_t)(huart->Instance->DR & 0xFF);
                break;
            }
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
                /* ORE = 新字节丢失, 但 DR 里的旧字节仍然有效 —— 必须保留!
                 * 旧版直接丢字节 -> 1Mbps 背靠背字节偶发溢出时丢 1 字节
                 * -> 整帧位移 -> 校验失败 -> 该 ID 偶发 MISSING。
                 * 清除序列: 读 SR, 再读 DR。DR 有数据(RXNE)则作为数据收下,
                 * 没有则丢弃这次读数(ORE 残留标志的清理, 不注入垃圾字节)。 */
                (void)huart->Instance->SR;
                if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
                    data[got++] = (uint8_t)(huart->Instance->DR & 0xFF);
                    break;
                }
                (void)huart->Instance->DR;
            }
        }

        /* ---- 快路径: 帧头 + ID 已知 -> 用 Length 字段定帧长, 收满即返回 ----
         * 必须同时确认 ID 匹配, 否则可能锁到噪声里的假 FF FF 提前截断。 */
        if (expect_id != 0xFFU && want == 0U && got >= 4U) {
            for (uint16_t i = 0U; (uint16_t)(i + 4U) <= got; i++) {
                if (data[i] == 0xFFU && data[i + 1U] == 0xFFU &&
                    data[i + 2U] == expect_id) {
                    uint8_t  len   = data[i + 3U];
                    uint16_t total = (uint16_t)(i + 4U + len);
                    if (len >= 2U && total <= cap) want = total;
                    break;   /* 候选帧头无效应退回静默判停 */
                }
            }
        }
        if (want != 0U && got >= want) goto done;
    }

done:
    if (out_len != NULL) *out_len = got;
    return (got > 0) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef DXL_Receive(uint8_t bus, uint8_t* data, uint16_t cap,
                              uint16_t* out_len, uint32_t timeout_ms) {
    return receive_common(bus, data, cap, out_len, timeout_ms, 0xFFU);
}

HAL_StatusTypeDef DXL_ReceiveEx(uint8_t bus, uint8_t* data, uint16_t cap,
                                uint16_t* out_len, uint32_t timeout_ms,
                                uint8_t expect_id) {
    return receive_common(bus, data, cap, out_len, timeout_ms, expect_id);
}

HAL_StatusTypeDef DXL_TransmitReceive(uint8_t bus, uint8_t* tx_data, uint16_t tx_len,
                                       uint8_t* rx_data, uint16_t rx_cap,
                                       uint16_t* rx_got, uint32_t timeout_ms) {
    HAL_StatusTypeDef status;

    /* Transmit packet (RE stays on; echo drained after TX) */
    status = DXL_Transmit(bus, tx_data, tx_len);
    if (status != HAL_OK) return status;

    /* Collect reply frame (stops on inter-byte silence) */
    status = DXL_Receive(bus, rx_data, rx_cap, rx_got, timeout_ms);
    return status;
}

HAL_StatusTypeDef DXL_TransmitReceiveEx(uint8_t bus, uint8_t* tx_data, uint16_t tx_len,
                                        uint8_t* rx_data, uint16_t rx_cap,
                                        uint16_t* rx_got, uint32_t timeout_ms,
                                        uint8_t expect_id) {
    HAL_StatusTypeDef status;

    status = DXL_Transmit(bus, tx_data, tx_len);
    if (status != HAL_OK) return status;

    status = DXL_ReceiveEx(bus, rx_data, rx_cap, rx_got, timeout_ms, expect_id);
    return status;
}
