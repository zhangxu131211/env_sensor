#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// 任务状态枚举
typedef enum {
    TASK_STATE_IDLE = 0,    // 空闲
    TASK_STATE_READY        // 就绪
} TaskState;

// 任务结构体
typedef struct {
    void (*task_func)(void); // 任务执行函数
    uint32_t period_ms;      // 执行周期（ms）
    uint32_t last_run_tick;  // 上一次执行的系统时基
    TaskState state;         // 任务当前状态
    uint8_t enable;          // 任务使能标志（1=启用，0=禁用）
} Task_t;

// 全局时基外部声明（在TIM6中断中递增）
//extern volatile uint32_t g_sys_tick;

/**
 * @brief  调度器初始化（启动TIM6 1ms中断）
 */
void Task_Scheduler_Init(void);

/**
 * @brief  注册任务到调度器
 * @param  func: 任务执行函数指针
 * @param  period_ms: 任务执行周期（ms）
 */
void Task_Scheduler_Register(void (*func)(void), uint32_t period_ms);

/**
 * @brief  修改已有任务的周期（新增功能）
 * @param  func: 任务函数指针
 * @param  new_period_ms: 新周期
 * @retval true=成功, false=未找到任务
 */
bool Task_Scheduler_ModifyPeriod(void (*func)(void), uint32_t new_period_ms);

/**
 * @brief  调度器主循环（需在main的while(1)中调用）
 */
void Task_Scheduler_Run(void);

/**
 * @brief  获取系统当前时基（ms）
 */
uint32_t Scheduler_GetSysTick(void);

/**
 * @brief  禁用指定任务
 */
void Task_Scheduler_Disable(void (*func)(void));

/**
 * @brief  启用指定任务
 */
void Task_Scheduler_Enable(void (*func)(void));

#endif /* TASK_SCHEDULER_H */

