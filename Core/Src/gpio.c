/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "rtc.h"

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TMP117_ALE_Pin */
  GPIO_InitStruct.Pin = TMP117_ALE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TMP117_ALE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP580_INT_Pin */
  GPIO_InitStruct.Pin = BMP580_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMP580_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E22_M0_Pin|E22_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : E22_M0_Pin E22_M1_Pin */
  GPIO_InitStruct.Pin = E22_M0_Pin|E22_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E22_AUX_Pin */
  GPIO_InitStruct.Pin = E22_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(E22_AUX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // 上升沿中断（PPS脉冲）
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GY95T_INT_Pin */
  GPIO_InitStruct.Pin = GY95T_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GY95T_INT_GPIO_PORT, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0); // 优先级低于USART1
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief PPS脉冲中断服务函数（EXTI9_5_IRQHandler）
 * @note 只做RTC秒数同步，不涉及任何E22发送逻辑
 */
void EXTI9_5_IRQHandler(void)
{
#if 1  
  static uint32_t last_pps_tick = 0; // 频率限制：1秒内只处理一次
  uint32_t current_tick = HAL_GetTick();

  // 1. 软件防抖+频率限制（核心：避免中断连发）
  if(current_tick - last_pps_tick < 990)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5); // 清标志后直接返回
    EXTI->PR1 |= EXTI_PR1_PIF5;
    return;
  }
  last_pps_tick = current_tick;

  // 2. 确认是PB5中断并彻底清标志
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    EXTI->PR1 |= EXTI_PR1_PIF5;
    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);

    // 3. RTC秒数同步（仅保留这个核心逻辑）
    RTC_Read_Time();
    g_rtc_time.sec++;
    if(g_rtc_time.sec >= 60)
    {
      g_rtc_time.sec = 0;
      g_rtc_time.min++;
      if(g_rtc_time.min >= 60)
      {
        g_rtc_time.min = 0;
        g_rtc_time.hour++;
        if(g_rtc_time.hour >= 24)
        {
          g_rtc_time.hour = 0;
          g_rtc_time.date++;
        }
      }
    }
    RTC_Set_Time(&g_rtc_time);

    // 可选：打印PPS同步日志（方便调试）
    // printf("PPS同步RTC：秒数更新为%d\r\n", g_rtc_time.sec);
  }
#endif
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
