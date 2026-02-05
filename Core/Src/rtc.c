/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"
#include "gpio.h"
#include <stdio.h>
#include "main.h"

/* USER CODE BEGIN 0 */
// RTC时间结构体（方便读写）
//typedef struct
//{
//  uint8_t hour;
//  uint8_t min;
//  uint8_t sec;
//  uint8_t date;
//  uint8_t month;
//  uint8_t year;
//} RTC_Time_Date_t;

// 全局RTC时间（供外部调用）
RTC_Time_Date_t g_rtc_time;
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  // 初始化时读取一次RTC时间
  RTC_Read_Time();
  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief 读取RTC时间和日期
 * @retval uint8_t: 0=失败，1=成功
 */
uint8_t RTC_Read_Time(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // 读取RTC时间
  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }
  // 读取RTC日期（必须在读取时间后）
  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }

  // 赋值到全局结构体
  g_rtc_time.hour = sTime.Hours;
  g_rtc_time.min = sTime.Minutes;
  g_rtc_time.sec = sTime.Seconds;
  g_rtc_time.date = sDate.Date;
  g_rtc_time.month = sDate.Month;
  g_rtc_time.year = sDate.Year;

  return 1;
}

/**
 * @brief 设置RTC时间和日期
 * @param time_date: 要设置的时间日期
 * @retval uint8_t: 0=失败，1=成功
 */
uint8_t RTC_Set_Time(RTC_Time_Date_t *time_date)
{
  if(time_date == NULL) return 0;

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  // 设置时间
  sTime.Hours = time_date->hour;
  sTime.Minutes = time_date->min;
  sTime.Seconds = time_date->sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }

  // 设置日期
  sDate.WeekDay = RTC_WEEKDAY_MONDAY; // 不关注星期，随便设
  sDate.Month = time_date->month;
  sDate.Date = time_date->date;
  sDate.Year = time_date->year;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    return 0;
  }

  // 同步到全局变量
  RTC_Read_Time();

  return 1;
}

/**
 * @brief 用GPS UTC时间更新RTC（核心同步函数）
 * @param utc_time: GPS UTC时间（HHMMSS.sss）
 * @param utc_date: GPS日期（DDMMYY）
 * @retval uint8_t: 0=失败，1=成功
 */
uint8_t RTC_Update_From_GPS(char *utc_time, char *utc_date)
{
  if(utc_time == NULL || utc_date == NULL) return 0;

  RTC_Time_Date_t gps_time = {0};

  // 解析UTC时间：HHMMSS.sss → 时/分/秒
  gps_time.hour = (utc_time[0] - '0') * 10 + (utc_time[1] - '0');
  gps_time.min = (utc_time[2] - '0') * 10 + (utc_time[3] - '0');
  gps_time.sec = (utc_time[4] - '0') * 10 + (utc_time[5] - '0');

  // 解析UTC日期：DDMMYY → 日/月/年
  gps_time.date = (utc_date[0] - '0') * 10 + (utc_date[1] - '0');
  gps_time.month = (utc_date[2] - '0') * 10 + (utc_date[3] - '0');
  gps_time.year = (utc_date[4] - '0') * 10 + (utc_date[5] - '0');

  // 转换为北京时间（UTC+8）
  gps_time.hour += 8;
  if(gps_time.hour >= 24)
  {
    gps_time.hour -= 24;
    gps_time.date += 1; // 跨天，日期+1（简化处理，未考虑月末/年末）
  }

  // 设置RTC时间
  return RTC_Set_Time(&gps_time);
}

/**
 * @brief 格式化打印RTC时间
 * @retval 无
 */
void RTC_Print_Time(void)
{
  if(RTC_Read_Time())
  {
    DEBUG_LOG("RTC本地时间：%02d:%02d:%02d  %02d/%02d/20%02d\r\n",
           g_rtc_time.hour, g_rtc_time.min, g_rtc_time.sec,
           g_rtc_time.date, g_rtc_time.month, g_rtc_time.year);
  }
  else
  {
    DEBUG_LOG("RTC读取失败！\r\n");
  }
}
/* USER CODE END 1 */
