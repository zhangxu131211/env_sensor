#include "gy95t.h"
#include "main.h"


// 全局变量：存储传感器数据
GY95T_DataDef gy95t_data = {0};

/**
 * @brief  I2C写单个寄存器（使用HAL_I2C_Mem_Write，匹配传感器时序）
 */
HAL_StatusTypeDef GY95T_WriteReg(uint8_t reg_addr, uint8_t data)
{
    // HAL_I2C_Mem_Write：设备地址、寄存器地址、寄存器地址长度、数据、数据长度、超时
    return HAL_I2C_Mem_Write(&hi2c3, GY95T_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief  GY95T初始化（配置更新频率）
 * @param  freq_level: 0=10Hz,1=50Hz,2=100Hz,3=200Hz（手册0x02寄存器）
 */
HAL_StatusTypeDef GY95T_Init(uint8_t freq_level)
{
    HAL_Delay(30); // 上电稳定延时
    freq_level = (freq_level > 3) ? 3 : freq_level; // 限制频率范围
    return GY95T_WriteReg(0x02, freq_level); // 配置更新频率
}

/**
 * @brief  GY95T 无人机专用初始化函数
 * @note   配置：±16G加速度 + ±2000°/s陀螺仪 + ±30Guass磁力计 + 水平模式 + 200Hz刷新率
 * @retval HAL_StatusTypeDef 初始化状态
 */
HAL_StatusTypeDef GY95T_Drone_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

		if (GY95T_Init(1) == HAL_OK)
		{
				DEBUG_LOG("GY95T Init Success!\r\n");
		}
		else
		{
				DEBUG_LOG("GY95T Init Failed!\r\n");
		}
	
    // 2. 配置传感器量程（无人机专用参数）
    status = GY95T_SetRange(ACC_FS_16G, GYRO_FS_2000, MAG_FS_30G, WORK_MODE_HORIZONTAL);
    if (status != HAL_OK)
    {
        DEBUG_LOG("Range Config Failed!\r\n");
        return status;
    }

    // 3. 配置更新频率为200Hz（最大刷新率，提升飞控响应速度）
    status = GY95T_WriteReg(0x02, 0x03);
    if (status != HAL_OK)
    {
        DEBUG_LOG("Freq Config Failed!\r\n");
        return status;
    }

    // 4. 配置输出模式为连续输出
    status = GY95T_WriteReg(0x03, 0x00);
    if (status != HAL_OK)
    {
        DEBUG_LOG("Output Mode Config Failed!\r\n");
        return status;
    }

    // 5. 执行自动校准流程（加陀+磁场校准）
    DEBUG_LOG("=====================================\r\n");
    DEBUG_LOG("GY95T Drone Initialization...\r\n");
    if (GY95T_AutoCalibrate() == HAL_OK)
    {
        // 读取校准后数据，验证校准效果
        GY95T_ReadData(&gy95t_data);
        DEBUG_LOG("Calibration Completed! Level: %d (100为最佳)\r\n", gy95t_data.level);
        DEBUG_LOG("ACC Range: ±16G | GYRO Range: ±2000°/s | Freq: 200Hz\r\n");
        DEBUG_LOG("Start Data Acquisition...\r\n");
    }
    else
    {
        DEBUG_LOG("Calibration Failed! Continue with default data\r\n");
        status = HAL_ERROR;
    }
    DEBUG_LOG("=====================================\r\n\r\n");

    // 6. 掉电保存所有配置（避免重启后参数丢失）
    GY95T_WriteReg(0x05, 0x55);

    return status;
}

/**
 * @brief  传感器校准（加陀/磁场）
 * @param  cal_type: 0x57=加陀校准, 0x58=磁场开始校准, 0x59=磁场结束校准, 0x5A=保存磁场校准
 */
HAL_StatusTypeDef GY95T_Calibrate(uint8_t cal_type)
{
    if (!(cal_type == 0x57 || cal_type == 0x58 || cal_type == 0x59 || cal_type == 0x5A))
    {
        return HAL_ERROR;
    }
    return GY95T_WriteReg(0x05, cal_type);
}

/**
 * @brief  自动校准流程（初始化时调用，含加陀+磁场校准）
 */
HAL_StatusTypeDef GY95T_AutoCalibrate(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    DEBUG_LOG("=====================================\r\n");
    DEBUG_LOG("Auto Calibration Start!\r\n");
    DEBUG_LOG("Step 1: Keep module HORIZONTAL & STABLE\r\n");
    HAL_Delay(10);
    
    // 加陀校准（手册0x57指令，2秒稳定时间）
    if (GY95T_Calibrate(0x57) == HAL_OK)
    {
        for (int i = 2; i > 0; i--)
        {
            DEBUG_LOG("Acc/Gyro Calibrating... %ds\r\n", i);
            HAL_Delay(10);
        }
        DEBUG_LOG("Step 1: Acc/Gyro Calibration Success!\r\n");
    }
    else
    {
        DEBUG_LOG("Step 1: Acc/Gyro Calibration Failed!\r\n");
        status = HAL_ERROR;
    }
    
    // 磁场校准（0x58开始→旋转→0x59结束→0x5A保存）
    DEBUG_LOG("\r\nStep 2: Rotate module 360° around 3 axes\r\n");
    HAL_Delay(1000);
    
    if (GY95T_Calibrate(0x58) == HAL_OK)
    {
        for (int i = 5; i > 0; i--)
        {
            DEBUG_LOG("Mag Calibrating... Remaining %d0ms\r\n", i);
            HAL_Delay(10);
        }
        GY95T_Calibrate(0x59);
        GY95T_Calibrate(0x5A);
        DEBUG_LOG("Step 2: Mag Calibration Success!\r\n");
    }
    else
    {
        DEBUG_LOG("Step 2: Mag Calibration Failed!\r\n");
        status = HAL_ERROR;
    }
    
    // 保存所有配置（掉电不丢失）
    GY95T_WriteReg(0x05, 0x55);
    DEBUG_LOG("\r\nAll Calibration Data Saved!\r\n");
    DEBUG_LOG("=====================================\r\n\r\n");
    
    return status;
}

/**
 * @brief  四元数归一化（确保在-1.0~1.0范围）
 */
void GY95T_NormalizeQuaternion(GY95T_DataDef *gy_data)
{
    float norm = sqrt(gy_data->q0 * gy_data->q0 + 
                      gy_data->q1 * gy_data->q1 + 
                      gy_data->q2 * gy_data->q2 + 
                      gy_data->q3 * gy_data->q3);
    if (norm < 0.0001f) norm = 1.0f; // 避免除零
    gy_data->q0 /= norm;
    gy_data->q1 /= norm;
    gy_data->q2 /= norm;
    gy_data->q3 /= norm;
}

/**
 * @brief  原始数据换算为实际物理值（修正之前的复制粘贴bug）
 */
void GY95T_ConvertData(GY95T_DataDef *gy_data)
{
    /* 1. 加速计换算（默认±16G） */
    switch (ACC_FS_DEFAULT)
    {
        case 0: 
            gy_data->acc_x_g = (float)gy_data->acc_x_raw / ACC_LSB_2G;
            gy_data->acc_y_g = (float)gy_data->acc_y_raw / ACC_LSB_2G;
            gy_data->acc_z_g = (float)gy_data->acc_z_raw / ACC_LSB_2G;
            break;
        case 1: 
            gy_data->acc_x_g = (float)gy_data->acc_x_raw / ACC_LSB_4G;
            gy_data->acc_y_g = (float)gy_data->acc_y_raw / ACC_LSB_4G;
            gy_data->acc_z_g = (float)gy_data->acc_z_raw / ACC_LSB_4G;
            break;
        case 2: 
            gy_data->acc_x_g = (float)gy_data->acc_x_raw / ACC_LSB_8G;
            gy_data->acc_y_g = (float)gy_data->acc_y_raw / ACC_LSB_8G;
            gy_data->acc_z_g = (float)gy_data->acc_z_raw / ACC_LSB_8G;
            break;
        case 3: 
            gy_data->acc_x_g = (float)gy_data->acc_x_raw / ACC_LSB_16G;
            gy_data->acc_y_g = (float)gy_data->acc_y_raw / ACC_LSB_16G;
            gy_data->acc_z_g = (float)gy_data->acc_z_raw / ACC_LSB_16G;
            break;
    }

    /* 2. 陀螺仪换算（默认±2000°/s） */
    switch (GYRO_FS_DEFAULT)
    {
        case 0: 
            gy_data->gyro_x_dps = (float)gy_data->gyro_x_raw / GYRO_LSB_250DPS;
            gy_data->gyro_y_dps = (float)gy_data->gyro_y_raw / GYRO_LSB_250DPS;
            gy_data->gyro_z_dps = (float)gy_data->gyro_z_raw / GYRO_LSB_250DPS;
            break;
        case 1: 
            gy_data->gyro_x_dps = (float)gy_data->gyro_x_raw / GYRO_LSB_500DPS;
            gy_data->gyro_y_dps = (float)gy_data->gyro_y_raw / GYRO_LSB_500DPS;
            gy_data->gyro_z_dps = (float)gy_data->gyro_z_raw / GYRO_LSB_500DPS;
            break;
        case 2: 
            gy_data->gyro_x_dps = (float)gy_data->gyro_x_raw / GYRO_LSB_1000DPS;
            gy_data->gyro_y_dps = (float)gy_data->gyro_y_raw / GYRO_LSB_1000DPS;
            gy_data->gyro_z_dps = (float)gy_data->gyro_z_raw / GYRO_LSB_1000DPS;
            break;
        case 3: 
            gy_data->gyro_x_dps = (float)gy_data->gyro_x_raw / GYRO_LSB_2000DPS;
            gy_data->gyro_y_dps = (float)gy_data->gyro_y_raw / GYRO_LSB_2000DPS;
            gy_data->gyro_z_dps = (float)gy_data->gyro_z_raw / GYRO_LSB_2000DPS;
            break;
    }

    /* 3. 姿态角+温度换算 */
    gy_data->roll_deg  = (float)gy_data->roll_raw / 100.0f;
    gy_data->pitch_deg = (float)gy_data->pitch_raw / 100.0f;
    gy_data->yaw_deg   = (float)gy_data->yaw_raw / 100.0f;
    gy_data->temp_c    = (float)gy_data->temp_raw / 100.0f;
}

/**
 * @brief  读取完整传感器数据（核心修正：使用HAL_I2C_Mem_Read匹配I2C时序）
 */
HAL_StatusTypeDef GY95T_ReadData(GY95T_DataDef *gy_data)
{
    uint8_t raw_buf[GY95T_DATA_LEN] = {0};
    HAL_StatusTypeDef status;

    // 关键修正：使用HAL_I2C_Mem_Read，自动处理传感器要求的I2C读时序
    // 参数：I2C句柄、设备地址、寄存器地址、寄存器地址长度、数据缓冲区、数据长度、超时
    status = HAL_I2C_Mem_Read(&hi2c3, GY95T_I2C_ADDR, GY95T_REG_START, 
                              I2C_MEMADD_SIZE_8BIT, raw_buf, GY95T_DATA_LEN, 100);
    if (status != HAL_OK) return status;

    /* 解析原始数据（严格对应手册寄存器顺序） */
    // 1. 加速计（0x08~0x0D → raw_buf[0]~[5]）
    gy_data->acc_x_raw = (int16_t)(raw_buf[1] << 8 | raw_buf[0]); // ACC_X_H(0x09)<<8 | ACC_X_L(0x08)
    gy_data->acc_y_raw = (int16_t)(raw_buf[3] << 8 | raw_buf[2]); // ACC_Y_H(0x0B)<<8 | ACC_Y_L(0x0A)
    gy_data->acc_z_raw = (int16_t)(raw_buf[5] << 8 | raw_buf[4]); // ACC_Z_H(0x0D)<<8 | ACC_Z_L(0x0C)

    // 2. 陀螺仪（0x0E~0x13 → raw_buf[6]~[11]）
    gy_data->gyro_x_raw = (int16_t)(raw_buf[7] << 8 | raw_buf[6]);// GYRO_X_H(0x0F)<<8 | GYRO_X_L(0x0E)
    gy_data->gyro_y_raw = (int16_t)(raw_buf[9] << 8 | raw_buf[8]);// GYRO_Y_H(0x11)<<8 | GYRO_Y_L(0x10)
    gy_data->gyro_z_raw = (int16_t)(raw_buf[11] << 8 | raw_buf[10]);// GYRO_Z_H(0x13)<<8 | GYRO_Z_L(0x12)

    // 3. 姿态角（0x14~0x19 → raw_buf[12]~[17]）
    gy_data->roll_raw  = (int16_t)(raw_buf[13] << 8 | raw_buf[12]);// ROLL_H(0x15)<<8 | ROLL_L(0x14)
    gy_data->pitch_raw = (int16_t)(raw_buf[15] << 8 | raw_buf[14]);// PITCH_H(0x17)<<8 | PITCH_L(0x16)
    gy_data->yaw_raw   = (int16_t)(raw_buf[17] << 8 | raw_buf[16]);// YAW_H(0x19)<<8 | YAW_L(0x18)

    // 4. 校准精度与温度（0x1A~0x1C → raw_buf[18]~[20]）
    gy_data->level     = raw_buf[18];                              // Level(0x1A)
    gy_data->temp_raw  = (int16_t)(raw_buf[20] << 8 | raw_buf[19]);// TEMP_H(0x1C)<<8 | TEMP_L(0x1B)

    // 5. 磁场（0x1D~0x22 → raw_buf[21]~[26]）
    gy_data->mag_x_raw = (int16_t)(raw_buf[22] << 8 | raw_buf[21]);// MAG_X_H(0x1E)<<8 | MAG_X_L(0x1D)
    gy_data->mag_y_raw = (int16_t)(raw_buf[24] << 8 | raw_buf[23]);// MAG_Y_H(0x20)<<8 | MAG_Y_L(0x1F)
    gy_data->mag_z_raw = (int16_t)(raw_buf[26] << 8 | raw_buf[25]);// MAG_Z_H(0x22)<<8 | MAG_Z_L(0x21)

    // 6. 四元数（0x23~0x2A → raw_buf[27]~[34]，严格按手册索引）
    int16_t q0_raw = (int16_t)(raw_buf[28] << 8 | raw_buf[27]);// Q0_H(0x24)<<8 | Q0_L(0x23)
    int16_t q1_raw = (int16_t)(raw_buf[30] << 8 | raw_buf[29]);// Q1_H(0x26)<<8 | Q1_L(0x25)
    int16_t q2_raw = (int16_t)(raw_buf[32] << 8 | raw_buf[31]);// Q2_H(0x28)<<8 | Q2_L(0x27)
    int16_t q3_raw = (int16_t)(raw_buf[34] << 8 | raw_buf[33]);// Q3_H(0x2A)<<8 | Q3_L(0x29)
    
    // 四元数换算（手册定义：原始值/10000 → -1.0~1.0）
    gy_data->q0 = (float)q0_raw / 10000.0f;
    gy_data->q1 = (float)q1_raw / 10000.0f;
    gy_data->q2 = (float)q2_raw / 10000.0f;
    gy_data->q3 = (float)q3_raw / 10000.0f;

    // 四元数归一化（修正传感器误差导致的超出范围问题）
    GY95T_NormalizeQuaternion(gy_data);

    // 原始数据换算为实际物理值
    GY95T_ConvertData(gy_data);

    return HAL_OK;
}


/**
 * @brief  配置磁场校准方法（寄存器0x06）
 * @param  cal_cfg: 校准配置参数，按位定义组合
 *         bit0~bit2: 磁场校准方法 000=2D校准 001=3D校准 010=硬铁椭球 011=软硬铁椭球(默认)
 *         bit3: 磁场融合校准 0=不使用 1=使用
 *         bit4: 磁场抗干扰校准 0=不使用 1=使用
 *         bit5: 磁场自动校准 0=不使用 1=使用
 *         bit6: 陀螺自校准 0=不使用 1=使用
 *         bit7: 校准阀值选择 0=level(70-130) 1=level(80-120)
 * @retval HAL_StatusTypeDef 通信状态
 */
HAL_StatusTypeDef GY95T_SetCalibrationMethod(uint8_t cal_cfg)
{
    return GY95T_WriteReg(0x06, cal_cfg);
}

/**
 * @brief  配置传感器量程（寄存器0x07）
 * @param  acc_fs: 加速度计量程 0=±2G 1=±4G 2=±8G 3=±16G(默认)
 * @param  gyro_fs: 陀螺仪量程 0=±250 1=±500 2=±1000 3=±2000(默认)
 * @param  mag_fs: 磁力计量程 0=±2Guass 1=±8Guass 2=±12Guass 3=±30Guass(默认)
 * @param  work_mode: 工作模式 0=竖直 1=水平(默认)
 * @retval HAL_StatusTypeDef 通信状态
 */
HAL_StatusTypeDef GY95T_SetRange(uint8_t acc_fs, uint8_t gyro_fs, uint8_t mag_fs, uint8_t work_mode)
{
    // 按寄存器0x07位定义组合配置值
    // bit0~bit1: 加计量程  bit2~bit3: 磁场量程  bit4~bit5: 保留  bit6: 水平/竖直  bit7: 陀螺量程
    uint8_t range_cfg = 0;
    range_cfg |= (acc_fs & 0x03);                // 加计量程 bit0~bit1
    range_cfg |= ((mag_fs & 0x03) << 2);         // 磁力计量程 bit2~bit3
    range_cfg |= ((work_mode & 0x01) << 6);      // 工作模式 bit6
    range_cfg |= ((gyro_fs & 0x03) << 7);        // 陀螺量程 bit7

    return GY95T_WriteReg(0x07, range_cfg);
}




