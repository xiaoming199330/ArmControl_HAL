/*
 * Feetech 半双工硬件接口。
 * BUS_A/B 选择两条独立总线，方向引脚控制 74HC126D 的发送使能。
 */
#ifndef __FEETECH_HAL_H
#define __FEETECH_HAL_H

#include "stm32f4xx_hal.h"

#define BUS_A 0
#define BUS_B 1

#define DIR_A_PIN   GPIO_PIN_8   /* PA8  — Bus A direction control */
#define DIR_A_PORT  GPIOA
#define DIR_B_PIN   GPIO_PIN_10  /* PA10 — Bus B direction control */
#define DIR_B_PORT  GPIOA

void DXL_SetDirection(uint8_t bus, uint8_t dir);
void DXL_FlushErrors(uint8_t bus);
HAL_StatusTypeDef DXL_Transmit(uint8_t bus, uint8_t* data, uint16_t len);
/* 逐字节收帧: 静默判停(首字节等 timeout_ms, 后续 2ms/字节), 容忍 FE/ORE。
 * rx_cap 为缓冲容量, *out_len 返回实际收到的字节数(可能 < cap)。
 * 返回 HAL_OK 表示收到 >=1 字节, HAL_TIMEOUT 表示一个都没有。 */
HAL_StatusTypeDef DXL_Receive(uint8_t bus, uint8_t* data, uint16_t cap,
                              uint16_t* out_len, uint32_t timeout_ms);
HAL_StatusTypeDef DXL_TransmitReceive(uint8_t bus, uint8_t* tx_data, uint16_t tx_len,
                                       uint8_t* rx_data, uint16_t rx_cap,
                                       uint16_t* rx_got, uint32_t timeout_ms);

/* ===== 已知期望 ID 的快速接收 (2026-09-10, 单总线"边发边收"的关键) =====
 * SCS 状态帧自带 Length 字段 (字节3): 整帧线上字节 = 4 + Length。
 * 一旦在已收字节里定位到 `FF FF <expect_id>`, 就按 Length 算出整帧长度,
 * 收满即返回 —— 不再等 5ms 字节间静默判停。
 *
 * 效果: 单次读事务从 ~5ms 降到 ~0.3ms (1Mbps), 于是可以在 50Hz 控制节拍
 * 里插入遥测读而不拖慢运动。旧路径下 6 轴轮询一次要 ~30ms, 正好吃掉
 * 一个半控制节拍, 舵机内部 profile 被拖成"走走停停" → 观感就是抖动。
 *
 * expect_id = 0xFF 时退化为 DXL_Receive 的行为(未知 ID, 只能静默判停)。
 * 找不到匹配帧头时同样退回静默判停, 因此不会比旧版更早截断。 */
HAL_StatusTypeDef DXL_ReceiveEx(uint8_t bus, uint8_t* data, uint16_t cap,
                                uint16_t* out_len, uint32_t timeout_ms,
                                uint8_t expect_id);
HAL_StatusTypeDef DXL_TransmitReceiveEx(uint8_t bus, uint8_t* tx_data, uint16_t tx_len,
                                        uint8_t* rx_data, uint16_t rx_cap,
                                        uint16_t* rx_got, uint32_t timeout_ms,
                                        uint8_t expect_id);

#endif
