/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO initialization for six-axis robotic arm.
  *
  *          Pin assignments (v5 -- PC5/PC4/PA5 LEDs, UART5 Bus B):
  *          PA5  = LED_BLUE       (run/status indicator)
  *          PB5  = Buzzer
  *          PB10 = USART3_TX      (Bus A TX, AF7 -- init in usart.c)
  *          PB11 = USART3_RX      (Bus A RX, AF7 -- init in usart.c)
 *          PC4  = LED_GREEN      (power indicator)   ┐ LINK=net 构建里整体让位给
 *          PC5  = LED_RED        (fault indicator)   ┘ ETH_RMII_RXD0/RXD1 (PHY 驱动)
  *          PC12 = UART5_TX       (Bus B TX, AF8 -- init in usart.c)
  *          PD2  = UART5_RX       (Bus B RX, AF8 -- init in usart.c)
  *          PA8  = DIR_A          (74HC126 bus A direction control)
  *          PA10 = DIR_B          (74HC126 bus B direction control)
  *          PE11 = E-Stop         (emergency stop, input, pull-up)
  *          PG0  = Limit switch J1 (input, pull-up)
  *          PG1  = Limit switch J2 (input, pull-up)
  *          PG2  = Limit switch J3 (input, pull-up)
  *          PG3  = Limit switch J4 (input, pull-up)
  *          PG4  = Limit switch J5 (input, pull-up)
  *          PG5  = Limit switch J6 (input, pull-up)
  *
  *          Note: USART3 (PB10/PB11) and UART5 (PC12/PD2) GPIO is
  *                initialized in usart.c HAL_UART_MspInit().
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided "AS IS".
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* ⚠ (2026-09-22) 本文件是 CubeMX 生成的: 重新生成/改 .ioc 会**覆盖掉**
 * 下面的 ARM_LINK_MULTI / ARM_LINK_NET 守卫与这一行 include。生成后务必
 * 回来复查(工程红线 49)。 */
#include "arm_link_cfg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
#if ARM_LINK_MULTI
  /* both 构建: PC4/PC5 的去留由**开机引脚形态**决定
   * (LinkCfg_Init() 必须已在本函数之前跑过, 见 main())。
   *   NET 或 AUTO ⇒ 它们是 ETH_RMII_RXD0/RXD1, 整段让位给 net_gpio_init() 的 AF11;
   *   CAN/OFF     ⇒ 它们是 LED, 配成推挽输出。
   * ⚠ 0924CE: 判据必须是 LinkCfg_BootNetPins() —— auto 开机时 ActiveMode()
   *   是动态值 OFF, 用 LINK_IS_NET() 判会把 RMII 引脚错配成 LED ⇒ 网口废掉。 */
  if (!LinkCfg_BootNetPins()) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  }
#elif !ARM_LINK_NET
  /* ⚠ LINK=net 构建里 PC4/PC5 是 **ETH_RMII_RXD0/RXD1 —— 由 PHY 驱动**。
   * 本行 + 下面的 OUTPUT_PP 原本无条件执行: 从 MX_GPIO_Init() 到
   * NetProto_Init() 之间(本工程中间还夹着 HAL_Delay(500) 等舵机自举),
   * MCU 与 PHY **两个推挽输出对着打, 且持续 >500ms** —— 既灌电流又把
   * 上电时序搅乱。net 构建必须整段让位给 net_gpio_init() 的 AF11。
   * (2026-09-20) ⚠ 本文件是 CubeMX 生成的: 重新生成/改 .ioc 会**覆盖掉这个
   *   守卫**, 生成后务必回来复查(见 docs/工程红线速查 第 49 条)。 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
#endif

  /*Configure GPIO pin Output Level (DIR_A=PA8, DIR_B=PA10) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /* 0924CM: 状态指示灯 PE4~PE7 初始电平
   * PE4=电源灯 上电即亮; PE5=运行灯 PE7=通信灯 熄灭待服务。
   * ⭐0924DA: PE6 报警灯为低电平点亮 (ARM_ALARM_LED_ACTIVE_LOW=1, 见 main.c)
   * ⇒ 初始必须拉高(灭), 否则开机到 Led_Service 首拍之间会先亮一下。 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
#if ARM_LINK_MULTI
  /* both 构建: 同上, 由开机引脚形态决定 —— NET/AUTO 时整段跳过(AF11 归 net_gpio_init)。 */
  if (!LinkCfg_BootNetPins()) {
      GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
#elif !ARM_LINK_NET
  /* ⚠ LINK=net: PC4/PC5 = ETH_RMII_RXD0/RXD1, 由 net_gpio_init() 配成 AF11。
   * 这里整段跳过 —— 理由见上面 Output Level 处的注释(两推挽对打 >500ms)。 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 */
  /* 限位开关低有效（闭合拉低）, 必须内部上拉,否则悬空误触发。
     原为 GPIO_NOPULL —— 与"低有效开关"接法不匹配, 已改为 PULLUP (2026-08-26)。 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 (DIR_A/DIR_B, 74HC126 direction) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 (E-Stop, active low, needs pull-up) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE6 PE7 (0924CM 状态指示灯, 推挽输出)
   * PE4=电源(常亮) PE5=运行(主循环心跳) PE6=报警(E-STOP/上次卡死提示)
   * PE7=通信(1s 内收到过指令即亮) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
