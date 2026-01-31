
#ifndef APP_TMP117_H
#define APP_TMP117_H

#include <stdbool.h>
#include <stdint.h>

#define tmp117_i2c	hi2c2

// º¯ÊýÉùÃ÷
bool App_TMP117_Init(void);
bool App_TMP117_ReadTemp(double *temp);
bool App_TMP117_SetAlarmThreshold(double high_temp, double low_temp);
bool App_TMP117_Calibrate(double target_temp);
void App_TMP117_MainTask(void);

#endif  // APP_TMP117_H
