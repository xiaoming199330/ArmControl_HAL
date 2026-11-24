/* USER CODE BEGIN Header */
/**
  * @file    usart.c
  * @brief   USART3 (Bus A) / UART5 (Bus B) for Feetech servos + UART4 master-slave link.
  *          USART3: PB10/PB11, 1000000 baud -- Bus A (J1-J3), 74HC126 DIR=PA8
  *          UART5:  PC12/PD2,  1000000 baud -- Bus B (J4-J6), 74HC126 DIR=PA10
  *          UART4:  PA0/PA1, 115200  baud -- 主从臂链路 (主臂板 -> 本板), 纯二进制帧
  *
  *          USART1 and USART2 are NOT used (pins not available on board).
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* ===== 舵机总线 TX 输出类型 (2026-11-02, 0924AY: 双板统一开漏) =====
 * 默认 (BUS_PP 未定义): **开漏单总线** —— 板上已拆 74HC126D, TX/RX 短接成一根,
 *   两条总线各自 1kΩ 上拉到 3.3V。发 0 拉低 / 发 1 释放, 高电平由外部上拉提供。
 *   与舵机应答天然不打架 —— 舵机推挽驱动时 MCU 处于释放(高阻)状态。
 *   ⚠ 上升沿靠 RC: τ = 1kΩ × C。1m 线 + 6 节点 (C≈240pF) 时到 2.0V 约 0.22µs,
 *     占 1Mbps 位宽(1µs)的 22%, 采样点在位中央仍然够。
 *     线长 >1.5m 或节点继续增加时, 先降到 500kbps, 不要靠加大上拉硬扛。
 *   ⚠ 硬件前提: 126 必须已拆除且 TX/RX 已短接。若 126 还挂着, DIR 默认低会把
 *     总线关断 -> 扫描 0/6, 现场表现和"固件烧坏了"完全一样。
 * 回退 make BUS_PP=1: 恢复推挽 (AF_PP) + 74HC126 方向切换, 供仍挂 126 的旧板。
 * 注意: 仅作用于舵机总线 (USART3 / UART5)。
 *       UART4 是主从臂全双工链路, 必须保持推挽, 不受此宏影响。 */
#if defined(BUS_PP) && BUS_PP
  #define SERVO_BUS_TX_MODE   GPIO_MODE_AF_PP
#else
  #define SERVO_BUS_TX_MODE   GPIO_MODE_AF_OD
#endif

/* 主从臂 UART4 波特率开关 (2026-09-11 物理层地弹排查)。
 * 默认 115200; 若链路 CRC 全错且硬件整改(独立 GND/串阻)无效,
 * 可两块板一起切到 19200 甚至 9600, 提高时序余量。
 * 用法: make BAUD=19200  或  make ROLE=master BAUD=19200 */
#ifndef UART4_BAUDRATE
  #define UART4_BAUDRATE    115200U
#endif

UART_HandleTypeDef huart3;  /* Bus A: J1-J3, PB10/PB11 */
UART_HandleTypeDef huart5;  /* Bus B: J4-J6, UART5 PC12/PD2 */
UART_HandleTypeDef huart4;  /* 主从臂链路, UART4 PA0/PA1 */

/* USER CODE END 0 */

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */
  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */
  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 1000000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  /* USER CODE END UART5_Init 2 */

}

/* UART4 init function -- 主从臂链路 (PA0=TX / PA1=RX, AF8) */
void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = UART4_BAUDRATE;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration  (与 ArmControl_HAL.ioc 一致)
    PA0-WKUP ------> UART4_TX
    PA1      ------> UART4_RX
    F407 的 UART4 只有 PA0/PA1 与 PC10/PC11 两组 AF8 可选。本工程 CubeMX
    选的是 PA0/PA1 (ioc: PA0-WKUP.Signal=UART4_TX, PA1.Signal=UART4_RX)。
    2026-09-11 由误写的 PC10/PC11 改正 —— 那两脚在本板上没引出, 从臂一个字节都收不到。
    */
    /* TX: 推挽输出, 无需上下拉。
     * 2026-09-11: 主臂自环 PA0↔PA1 出现 NE≈100%、FE≈36%、frames=0。
     * 115200 下不需要 VERY_HIGH 的摆率; 过陡的边沿会在短杜邦线/芯片内
     * 部地弹上产生振铃, 导致 RX 采样点误判。降到 LOW 后上升沿仍远快于
     * 1 位周期(8.7us), 不会损失带宽, 但能显著抑制振铃和地弹噪声。 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* RX: 禁用内部上拉 (2026-09-11)。
     * 原先用上拉是为了"线没接时一眼看出 0 字节", 但实测自环 + 链路都出现
     * NE≈100%。PA1 配内部上拉后, 当 PA0(TX) 输出低电平时, 电流经 40kΩ
     * 上拉从 VDD 灌入 PA0 的地驱动器; PA0/PA1 共享芯片内部键合地线, 该
     * 电流脉冲会在地弹上产生毛刺, 直接叠加到 PA1 的输入阈值参考, 导致
     * RX 采样电路几乎每一位都判噪声。外部推挽驱动时 RX 本就不需要上拉,
     * 悬空未接线的诊断价值不值得牺牲信号完整性。 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */
  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2      ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = SERVO_BUS_TX_MODE;   /* 主臂=AF_OD 开漏单总线 / 从臂=AF_PP 推挽 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN UART5_MspInit 1 */
  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */
  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = SERVO_BUS_TX_MODE;   /* 主臂=AF_OD 开漏单总线 / 从臂=AF_PP 推挽 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */
  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP ------> UART4_TX
    PA1      ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */
  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2      ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* USER CODE BEGIN UART5_MspDeInit 1 */
  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */
  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */
  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* ⭐0924BJ: 运行期切舵机总线波特率 (sid baud 降速过渡 / 上电自适配用)。
 * A/B 两路必须同挡 —— 12 只舵机的 EEPROM 是同批写的, 固件两路一起换挡。
 * 直接改 BRR 而不走 HAL_UART_Init: Init 会复位 RXNE 使能与外设状态,
 * 而本层收发是纯轮询(servo_bus_hal.c), 只需 BRR 生效即可。
 * ⭐0924BK 修正: 时钟源**不得硬编码** —— 实测 SystemClock_Config: SYSCLK=168M,
 * AHB=÷1, APB1=÷4 ⇒ USART3/UART5(APB1) 内核时钟 = 42MHz (不是 84M!)。
 * 0924BJ 硬编码 84M ⇒ BRR 翻倍 ⇒ 线上实际波特率只有标称一半
 * (固件以为发 1M 实发 500k) ⇒ 舵机全聋、扫描 0 只 —— 全部现象的根因。
 * 改为 HAL_RCC_GetPCLK1Value() 运行时取真值, 任何主频/分频下都正确。 */
void ServoBus_SetBaudBoth(uint32_t baud)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();    /* 本 HAL 版本无 GetPCLK1Value */
  uint32_t brr = (pclk1 + (baud / 2U)) / baud;
  if (brr < 1U) brr = 1U;
  __HAL_UART_DISABLE(&huart3);
  __HAL_UART_DISABLE(&huart5);
  USART3->BRR = brr;
  UART5->BRR  = brr;
  __HAL_UART_ENABLE(&huart3);
  __HAL_UART_ENABLE(&huart5);
}
/* USER CODE END 1 */
