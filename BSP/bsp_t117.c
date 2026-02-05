#include "bsp_t117.h"

/* ========================== 私有函数实现 ========================== */
/**
 * @brief  CRC8校验计算（T117P手册9.5节）
 * @param  pData: 待校验数据指针
 * @param  len: 数据长度
 * @retval CRC8校验结果
 */
static uint8_t BSP_T117_CRC8(uint8_t* pData, uint8_t len) {
    uint8_t result = 0x00;
    uint8_t pDataBuf;
    uint8_t i;
    while(len--) {
        pDataBuf = *pData++;
        for(i=0; i<8; i++) {
            if((result^(pDataBuf))&0x01) {
                result ^= 0x18;
                result >>= 1;
                result |= 0x80;
            } else {
                result >>= 1;
            }
            pDataBuf >>= 1;
        }
    }
    return result;
}

/**
 * @brief  写T117P单个寄存器（单字节数据）
 * @param  hi2c: I2C句柄
 * @param  regAddr: 寄存器地址
 * @param  data: 待写入数据
 * @retval true=成功，false=失败
 */
static bool BSP_T117_WriteReg(I2C_HandleTypeDef* hi2c, uint8_t regAddr, uint8_t data) {
    uint8_t txBuf[2] = {regAddr, data};
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, T117_I2C_W_ADDR, txBuf, 2, 100);
    return (ret == HAL_OK) ? true : false;
}

/**
 * @brief  读T117P单个寄存器（单字节数据）
 * @param  hi2c: I2C句柄
 * @param  regAddr: 寄存器地址
 * @param  pData: 读取数据存储指针
 * @retval true=成功，false=失败
 */
static bool BSP_T117_ReadReg(I2C_HandleTypeDef* hi2c, uint8_t regAddr, uint8_t* pData) {
    HAL_StatusTypeDef ret;
    // 1. 发送寄存器地址
    ret = HAL_I2C_Master_Transmit(hi2c, T117_I2C_W_ADDR, &regAddr, 1, 100);
    if (ret != HAL_OK) return false;
    // 2. 读取数据
    ret = HAL_I2C_Master_Receive(hi2c, T117_I2C_R_ADDR, pData, 1, 100);
    return (ret == HAL_OK) ? true : false;
}

/**
 * @brief  读取温度相关3个寄存器（Temp_LSB + Temp_MSB + CRC_TEMP）
 * @param  hi2c: I2C句柄
 * @param  pData: 3字节数据存储指针
 * @retval true=成功，false=失败
 */
static bool BSP_T117_ReadTempRegs(I2C_HandleTypeDef* hi2c, uint8_t* pData) {
    HAL_StatusTypeDef ret;
    uint8_t regAddr = T117_TEMP_LSB;  // 从温度低位开始读，自动递增地址
    // 1. 发送起始寄存器地址
    ret = HAL_I2C_Master_Transmit(hi2c, T117_I2C_W_ADDR, &regAddr, 1, 100);
    if (ret != HAL_OK) return false;
    // 2. 连续读取3字节数据
    ret = HAL_I2C_Master_Receive(hi2c, T117_I2C_R_ADDR, pData, 3, 100);
    return (ret == HAL_OK) ? true : false;
}

/* ========================== 公有函数实现 ========================== */
/**
 * @brief  初始化T117P
 * @param  hi2c: I2C句柄（需提前初始化I2C2）
 * @param  pBuffer: 临时缓冲区（至少3字节）
 * @retval true=初始化成功，false=失败
 */
bool BSP_T117_Init(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer) {
    if (hi2c == NULL || pBuffer == NULL) return false;

    // 1. 软件复位（装载E2PROM默认配置）
    if (!BSP_T117_SoftReset(hi2c, pBuffer)) {
        return false;
    }
    HAL_Delay(50);  // 复位后等待稳定

    // 2. 读取状态寄存器，验证通信
    uint8_t status = 0;
    if (!BSP_T117_GetStatus(hi2c, pBuffer, &status)) {
        return false;
    }
		
    // 3. 配置测量模式：单次测量
    if (!BSP_T117_SetConvMode(hi2c, pBuffer, T117_SINGLE_CONVERT)) {
        return false;
    }

    // 4. 配置平均次数：AVG8（默认，平衡精度和速度）
    if (!BSP_T117_SetAveraging(hi2c, pBuffer, T117_AVG_8)) {
        return false;
    }

    // 5. 配置低功耗模式：开启（默认）
    if (!BSP_T117_SetPDMode(hi2c, pBuffer, T117_PD_ON)) {
        return false;
    }

    return true;
}

/**
 * @brief  软件复位（T117P手册9.8节）
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（未使用，保持接口一致）
 * @retval true=成功，false=失败
 */
bool BSP_T117_SoftReset(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer) {
    (void)pBuffer;  // 未使用
    // 写EE_CMD寄存器：0x6A（软复位指令）
    return BSP_T117_WriteReg(hi2c, T117_EE_CMD, 0x6A);
}

/**
 * @brief  设置测量模式
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（未使用）
 * @param  mode: 测量模式（T117_CONV_MODE）
 * @retval true=成功，false=失败
 */
bool BSP_T117_SetConvMode(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_CONV_MODE mode) {
    (void)pBuffer;
    // Temp_Cmd寄存器：仅bit7-6有效，其他位保持0（加热关闭）
    uint8_t cmd = mode | T117_HEAT_OFF;
    return BSP_T117_WriteReg(hi2c, T117_TEMP_CMD, cmd);
}

/**
 * @brief  设置平均次数
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（存储配置寄存器值）
 * @param  avg: 平均次数（T117_AVG_MODE）
 * @retval true=成功，false=失败
 */
bool BSP_T117_SetAveraging(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_AVG_MODE avg) {
    // 1. 读取当前配置寄存器值
    uint8_t cfg = 0;
    if (!BSP_T117_ReadReg(hi2c, T117_TEMP_CFG, &cfg)) {
        return false;
    }

    // 2. 清除原有平均次数位（bit4-3），设置新值
    cfg &= ~0x18;  // 0x18 = 0b00011000
    cfg |= avg;

    // 3. 写回配置寄存器
    return BSP_T117_WriteReg(hi2c, T117_TEMP_CFG, cfg);
}

/**
 * @brief  设置测量频率（连续模式下有效）
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（存储配置寄存器值）
 * @param  freq: 测量频率（T117_MEAS_FREQ）
 * @retval true=成功，false=失败
 */
bool BSP_T117_SetMeasFreq(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_MEAS_FREQ freq) {
    // 1. 读取当前配置寄存器值
    uint8_t cfg = 0;
    if (!BSP_T117_ReadReg(hi2c, T117_TEMP_CFG, &cfg)) {
        return false;
    }

    // 2. 清除原有频率位（bit7-5），设置新值
    cfg &= ~0xE0;  // 0xE0 = 0b11100000
    cfg |= freq;

    // 3. 写回配置寄存器
    return BSP_T117_WriteReg(hi2c, T117_TEMP_CFG, cfg);
}

/**
 * @brief  设置低功耗模式
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（存储配置寄存器值）
 * @param  pd: 低功耗模式（T117_PD_MODE）
 * @retval true=成功，false=失败
 */
bool BSP_T117_SetPDMode(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_PD_MODE pd) {
    // 1. 读取当前配置寄存器值
    uint8_t cfg = 0;
    if (!BSP_T117_ReadReg(hi2c, T117_TEMP_CFG, &cfg)) {
        return false;
    }

    // 2. 清除原有低功耗位（bit0），设置新值
    cfg &= ~0x01;
    cfg |= pd;

    // 3. 写回配置寄存器
    return BSP_T117_WriteReg(hi2c, T117_TEMP_CFG, cfg);
}

/**
 * @brief  启动单次测温（仅单次测量模式有效）
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（未使用）
 * @retval true=成功，false=失败
 */
bool BSP_T117_StartSingleConv(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer) {
    (void)pBuffer;
    // 重新写入单次测量指令（触发一次测温）
    return BSP_T117_WriteReg(hi2c, T117_TEMP_CMD, T117_SINGLE_CONVERT | T117_HEAT_OFF);
}

/**
 * @brief  读取温度值（含CRC校验）
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（至少3字节）
 * @param  pTemp: 温度值存储指针（单位：℃）
 * @retval true=读取成功，false=失败（含CRC校验失败）
 */
bool BSP_T117_GetTemperature(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, float* pTemp) {
    if (hi2c == NULL || pBuffer == NULL || pTemp == NULL) return false;

    // // 1. 启动单次测温
    // if (!BSP_T117_StartSingleConv(hi2c, pBuffer)) {
    //     printf("[T117] 测温失败！ ---> 1 \r\n");
    //     return false;
    // }

    // // 2. 等待转换完成（AVG8对应转换时间5.2ms，预留冗余）
    // HAL_Delay(7);

    // 3. 读取温度低位、高位、CRC校验字节
    if (!BSP_T117_ReadTempRegs(hi2c, pBuffer)) {
//        printf("[T117] 测温失败！ ---> 2 \r\n");
        return false;
    }

    // 4. CRC校验（前2字节温度数据，对比第3字节CRC）
    uint8_t crcCalc = BSP_T117_CRC8(pBuffer, 2);
    if (crcCalc != pBuffer[2]) {
//        printf("[T117] 测温失败！ ---> 3 \r\n");
        return false;  // CRC校验失败
    }

    // 5. 温度数据解析（16位有符号二进制补码）
    int16_t rawTemp = (pBuffer[1] << 8) | pBuffer[0];  // MSB在前，LSB在后
    //*pTemp = rawTemp * T117_RESOLUTION;  // 转换为℃
		*pTemp = ((float)rawTemp / 256.0f) + 25.0f; 
    return true;
}

/**
 * @brief  读取状态寄存器
 * @param  hi2c: I2C句柄
 * @param  pBuffer: 临时缓冲区（未使用）
 * @param  pStatus: 状态值存储指针
 * @retval true=成功，false=失败
 */
bool BSP_T117_GetStatus(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, uint8_t* pStatus) {
    (void)pBuffer;
    return BSP_T117_ReadReg(hi2c, T117_STATUS, pStatus);
}

