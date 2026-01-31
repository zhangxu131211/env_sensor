/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#define DEBUG_UART huart2
/* USER CODE END Includes */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define GNSS_BUF_SIZE 2048
extern uint8_t gnss_rx_buf[GNSS_BUF_SIZE];
extern uint16_t gnss_rx_len;
extern uint8_t gnss_rx_complete;
extern uint8_t gnss_frame_flag;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// -------------------------- 调试模式开关（量产时注释此行关闭所有日志）--------------------------
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
// 带文件、函数、行号的日志宏，支持可变参数
#define LOG_PRINT(fmt, ...)  printf("[%s | %s:%d] " fmt "\n", \
                                    __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
// 关闭调试时，日志宏为空，不占用资源
#define LOG_PRINT(fmt, ...)
#endif

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

