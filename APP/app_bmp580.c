#include "app_bmp580.h"

struct bmp5_dev bmp580_dev;
BMP580_Data_t bmp580_data;

/* I2C读函数（适配BMP5驱动接口） */
static BMP5_INTF_RET_TYPE BMP580_I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    HAL_StatusTypeDef hal_status;
    (void)intf_ptr;  // 未使用

    /* HAL库I2C多字节读：DevAddress + MemAddress + MemAddSize + pData + Size + Timeout */
    hal_status = HAL_I2C_Mem_Read(&bmp580_i2c, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, length, 100);
    
    if (hal_status != HAL_OK)
    {
        return BMP5_E_COM_FAIL;  // 通信失败
    }
    return BMP5_INTF_RET_SUCCESS;
}

/* I2C写函数（适配BMP5驱动接口） */
static BMP5_INTF_RET_TYPE BMP580_I2C_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    HAL_StatusTypeDef hal_status;
    (void)intf_ptr;  // 未使用

    /* HAL库I2C多字节写：DevAddress + MemAddress + MemAddSize + pData + Size + Timeout */
    hal_status = HAL_I2C_Mem_Write(&bmp580_i2c, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, length, 100);
    
    if (hal_status != HAL_OK)
    {
        return BMP5_E_COM_FAIL;  // 通信失败
    }
    return BMP5_INTF_RET_SUCCESS;
}

/**
 * @brief  微秒级延迟函数（基于SysTick实现，适配STM32L431）
 */
void BMP580_DelayUs(uint32_t us, void *intf_ptr)
{
    (void)intf_ptr;
    uint32_t start_tick = SysTick->VAL;
    uint32_t ticks = us * (SystemCoreClock / 1000000);  // 换算成SysTick计数
    uint32_t curr_tick;

    /* SysTick为向下计数，需处理溢出 */
    do {
        curr_tick = SysTick->VAL;
    } while ((start_tick - curr_tick) < ticks);
}

/**
 * @brief  BMP580初始化
 */
int8_t APP_BMP580_Init(struct bmp5_dev *dev)
{
    int8_t rslt;

    if (dev == NULL)
    {
        return BMP5_E_NULL_PTR;
    }

    /* 初始化BMP5设备结构体 */
    dev->intf = BMP5_I2C_INTF;          // 使用I2C接口
    dev->read = BMP580_I2C_Read;        // 绑定I2C读函数
    dev->write = BMP580_I2C_Write;      // 绑定I2C写函数
    dev->delay_us = BMP580_DelayUs;     // 绑定延迟函数
    dev->intf_ptr = NULL;               // 接口指针未使用

//    /* BMP580芯片初始化（读取芯片ID、校验等） */
//    rslt = bmp5_init(dev);
//    if (rslt != BMP5_OK)
//    {
//        return rslt;
//    }

//    /* 软复位传感器 */
//    rslt = bmp5_soft_reset(dev);
//    if (rslt != BMP5_OK)
//    {
//        return rslt;
//    }

    return BMP5_OK;
}

/**
 * @brief  BMP580配置（连续采集模式）
 */
int8_t APP_BMP580_Config(struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_osr_odr_press_config osr_odr_cfg = {0};
    struct bmp5_iir_config iir_cfg = {0};

    if (dev == NULL)
    {
        return BMP5_E_NULL_PTR;
    }

    /* 1. 设置为待机模式（配置前需进入待机） */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    /* 2. 获取默认OSR/ODR配置，并使能压力采集（温度默认使能，无需配置） */
    rslt = bmp5_get_osr_odr_press_config(&osr_odr_cfg, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }
    osr_odr_cfg.press_en = BMP5_ENABLE;  // 仅使能压力采集，温度默认采集
    rslt = bmp5_set_osr_odr_press_config(&osr_odr_cfg, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    /* 3. 配置IIR滤波（温度/压力滤波系数为1） */
    iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
    iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
    iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
    iir_cfg.shdw_set_iir_p = BMP5_ENABLE;
    rslt = bmp5_set_iir_config(&iir_cfg, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    /* 4. 设置为连续采集模式 */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_CONTINOUS, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    return BMP5_OK;
}

/**
 * @brief  获取BMP580传感器数据
 */
int8_t APP_BMP580_GetData(struct bmp5_dev *dev, BMP580_Data_t *data)
{
    int8_t rslt;
    struct bmp5_sensor_data sensor_data = {0};
    struct bmp5_osr_odr_press_config osr_odr_cfg = {0};

    if (dev == NULL || data == NULL)
    {
        return BMP5_E_NULL_PTR;
    }

    /* 获取OSR/ODR配置（需传入bmp5_get_sensor_data） */
    rslt = bmp5_get_osr_odr_press_config(&osr_odr_cfg, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    /* 读取传感器数据（压力+温度） */
    rslt = bmp5_get_sensor_data(&sensor_data, &osr_odr_cfg, dev);
    if (rslt != BMP5_OK)
    {
        return rslt;
    }

    /* 转换为用户可读格式（默认启用浮点模式） */
    data->pressure = sensor_data.pressure;
    data->temperature = sensor_data.temperature;

    return BMP5_OK;
}

/**
 * @brief  封装的BMP580测试函数（main直接调用）
 */
void APP_BMP580_Test(void)
{
    int8_t ret;

    /* 测试开始提示 */
    printf("======== BMP580 Test Start ========\r\n");

    /* 初始化传感器 */
    ret = APP_BMP580_Init(&bmp580_dev);
    if (ret != BMP5_OK) {
				printf("BMP580 Init Failed %d\r\n", ret);
    } else {
        printf("BMP580 Init Success\r\n");
    }

    /* 配置传感器 */
    ret = APP_BMP580_Config(&bmp580_dev);
    if (ret != BMP5_OK) {
        printf("Config Failed %d\r\n", ret);
    }
    printf("Config Success\r\n");

    /* 循环采集并打印数据 */
    printf("Pressure(Pa)\tTemperature(°C)\r\n");
    printf("----------------------------------------\r\n");

    while (1) {
        ret = APP_BMP580_GetData(&bmp580_dev, &bmp580_data);
        if (ret == BMP5_OK) {
            printf("%.2f\t\t%.2f\r\n", bmp580_data.pressure, bmp580_data.temperature);
        } else {
            printf("Read Failed (Code: %d)\r\n", ret);
        }
        HAL_Delay(500); // 500ms采集一次
    }
}

int8_t BMP580_Init(void)
{

    int8_t ret;

    /* 初始化传感器 */
    ret = APP_BMP580_Init(&bmp580_dev);
    if (ret != BMP5_OK) {
        printf("BMP580 Init Failed %d\r\n", ret);
    } else {
        printf("BMP580 Init Success\r\n");
    }

    /* 配置传感器 */
    ret = APP_BMP580_Config(&bmp580_dev);
    if (ret != BMP5_OK) {
        printf("Config Failed %d\r\n", ret);
    } else {
			printf("BMP580 Config Success\r\n");
		}

		return ret;
	
//	    int8_t ret;

    /* 测试开始提示 */
////    printf("======== BMP580 Test Start ========\r\n");

//    /* 初始化传感器 */
//    ret = APP_BMP580_Init(&bmp580_dev);
//    if (ret != BMP5_OK) {

//    }
//    printf("Init Success\r\n");

//    /* 配置传感器 */
//    ret = APP_BMP580_Config(&bmp580_dev);
//    if (ret != BMP5_OK) {
//        printf("Config Failed %d\r\n", ret);
//    }
//    printf("Config Success\r\n");
//		return ret;
}

