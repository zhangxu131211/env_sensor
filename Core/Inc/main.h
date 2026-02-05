/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define TMP117_ALE_Pin GPIO_PIN_8
#define TMP117_ALE_GPIO_Port GPIOB
#define BMP580_INT_Pin GPIO_PIN_9
#define BMP580_INT_GPIO_Port GPIOB
#define GY95T_INT_PIN    GPIO_PIN_15
#define GY95T_INT_GPIO_PORT   GPIOA


#define E22_M0_Pin GPIO_PIN_0
#define E22_M0_GPIO_Port GPIOB
#define E22_M1_Pin GPIO_PIN_1
#define E22_M1_GPIO_Port GPIOB
#define E22_AUX_Pin GPIO_PIN_2
#define E22_AUX_GPIO_Port GPIOB


// #define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "stdio.h"
// 复用你的调试日志宏（带文件、函数、行号）
#define DEBUG_LOG(fmt, ...)  printf("[%s | %s:%d] " fmt "\n", \
                                  __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...)
#endif

#define DATA_LOG_ENABLE

#ifdef DATA_LOG_ENABLE
#define DATA_LOG(fmt, ...)  printf("[%s | %s:%d] " fmt "\n", \
                                  __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define DATA_LOG(fmt, ...)
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
