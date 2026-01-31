/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "rtc.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_sht40.h"
#include "stdio.h"
#include "app_bmp580.h"
#include "app_t117.h"
#include "temp.h"
#include "app_e22.h"
#include "app_e22_protocol.h"
#include "gps.h"
#include "task_scheduler.h"  // 调度器头文件
#include "gy95t.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
	MX_DMA_Init();
	MX_RTC_Init();
	MX_TIM6_Init();
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
		/*温湿度，大气压传感器单独测试*/
//		SHT40AD1B_Test();
		// APP_BMP580_Test();
//		APP_T117_Test();


	/*整体传感器初始化*/
	if (Sensor_Global_Init() != 0)
  {
      printf("[Sensor] 传感器初始化失败，程序退出！\r\n");
      Error_Handler(); // 初始化失败可选择进入错误处理
  }
	
//		// 2. 初始化调度器（启动TIM6中断）
//		Task_Scheduler_Init();

//    // 核心：只用一个顺序采集任务，100ms周期（实际T117 300ms一次，SHT40 100ms一次）
//    Task_Scheduler_Register(Task_Sensor_Sequence_Collect, 100);
//    
//    // BMP580：与温湿度错开，且降低频率到500ms
//    Task_Scheduler_Register(Task_BMP580_Collect, 500);
//    
//    // 应用层读取：仅打印，不操作硬件（改为200ms，减少终端刷屏）
//    Task_Scheduler_Register(Temp_Collect, 200);
//    
//    // 其他任务...
//    Task_Scheduler_Register(Task_GPS_Collect, 1000);
//    Task_Scheduler_Register(Task_GY95T_Collect, 10);
//    Task_Scheduler_Register(Task_E22_Send, 1000);
	
  while (1)
  {
	  /*双温度传感器测试*/	
//        Sensor_Test_Print_Real_Data();
//        HAL_Delay(100); // 1秒读取一次

		/*E22测试*/		
//		App_E22_Send_Data(E22_RX_ADDR, E22_RX_CHANNEL, "%x%x%x", 0xAA, 0xBB, 0xCC);

		/*PPS秒脉冲*/
//		Temp_Collect();
//		Task_Sensor_Sequence_Collect();
		Task_BMP580_Collect();
    /* GNSS和RTC同步测试 */    
    /* 原有GPS/RTC同步逻辑（保留，事件驱动） */
#if 0		
    if(gnss_rx_complete)
    {      
      gnss_rx_complete = 0;
      uint8_t parse_ret = GPS_Parse_RMC(gnss_rx_buf, gnss_rx_len);
      if(parse_ret == 1 && g_gps_data.status == 'A')
      {
        RTC_Update_From_GPS(g_gps_data.utc_time, g_gps_data.date);
        printf("\n===== GPS有效（同步RTC）=====\r\n");
//        GPS_Print_Result();
//        RTC_Print_Time();
        // 可选：GPS有效时额外触发一次发送（非必须）
        // Task_E22_Send();
      }
      else
      {
        printf("\n===== GPS无效（使用RTC）=====\r\n");
        printf("GPS解析失败/无定位信号！\r\n");
//        RTC_Print_Time();
      }
      gnss_rx_len = 0;
      memset(gnss_rx_buf, 0, GNSS_BUF_SIZE);  
    }

    // 调度器核心循环（唯一的发送入口）
    Task_Scheduler_Run();
    HAL_Delay(1); // 降低CPU占用
#endif			

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
