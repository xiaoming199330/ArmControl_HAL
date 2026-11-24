/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART3 (Bus A) and UART5 (Bus B) declarations.
  *          USART3: PB10/PB11, 1Mbps -- Bus A (J1-J3)
  *          UART5:  PC12/PD2,  1Mbps -- Bus B (J4-J6)
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;   /* 主从臂链路 (PA0=TX / PA1=RX), 115200 */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);
void MX_UART5_Init(void);
void MX_UART4_Init(void);
void ServoBus_SetBaudBoth(uint32_t baud);   /* 0924BJ: A/B 同切 BRR (sid baud 用) */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
