#include "task_scheduler.h"
#include "tim.h"
#include <string.h>
#include "stm32l4xx_hal.h" 

#define MAX_TASK_NUM 8
static Task_t g_task_list[MAX_TASK_NUM];
static uint8_t g_task_count = 0;


uint32_t Scheduler_GetSysTick(void)
{
    return g_sys_tick;
}

void Task_Scheduler_Register(void (*func)(void), uint32_t period_ms)
{
    if (g_task_count >= MAX_TASK_NUM) return;
    if (func == NULL) return;

    g_task_list[g_task_count].task_func = func;
    g_task_list[g_task_count].period_ms = period_ms;
    g_task_list[g_task_count].last_run_tick = Scheduler_GetSysTick();
    g_task_list[g_task_count].state = TASK_STATE_IDLE;
    g_task_list[g_task_count].enable = 1;

    g_task_count++;
}

bool Task_Scheduler_ModifyPeriod(void (*func)(void), uint32_t new_period_ms)
{
    for (uint8_t i = 0; i < g_task_count; i++) {
        if (g_task_list[i].task_func == func) {
            g_task_list[i].period_ms = new_period_ms;
            g_task_list[i].last_run_tick = Scheduler_GetSysTick(); // 重置计时
            return true;
        }
    }
    return false;
}

void Task_Scheduler_Disable(void (*func)(void))
{
    for (uint8_t i = 0; i < g_task_count; i++) {
        if (g_task_list[i].task_func == func) {
            g_task_list[i].enable = 0;
            break;
        }
    }
}

void Task_Scheduler_Enable(void (*func)(void))
{
    for (uint8_t i = 0; i < g_task_count; i++) {
        if (g_task_list[i].task_func == func) {
            g_task_list[i].enable = 1;
            g_task_list[i].last_run_tick = Scheduler_GetSysTick();
            break;
        }
    }
}

void Task_Scheduler_Run(void)
{
    uint32_t now_tick = Scheduler_GetSysTick();

    for (uint8_t i = 0; i < g_task_count; i++) {
        Task_t *task = &g_task_list[i];
        if (!task->enable) continue;

        // 处理32位溢出情况，计算经过时间
        uint32_t elapsed = now_tick - task->last_run_tick;
        if (elapsed >= task->period_ms) {
            task->state = TASK_STATE_READY;
            task->last_run_tick = now_tick;
        }

        if (task->state == TASK_STATE_READY) {
            if (task->task_func != NULL) {
                task->task_func();
            }
            task->state = TASK_STATE_IDLE;
        }
    }
}

void Task_Scheduler_Init(void)
{
    memset(g_task_list, 0, sizeof(g_task_list));
    g_task_count = 0;
    g_sys_tick = 0;
    HAL_TIM_Base_Start_IT(&htim6);
}


