#ifndef __E22_CONF_H
#define __E22_CONF_H
#define E22_UART_BUFFER_MAX_LENGTH (240)
#define E22_UART_BUFFER_DEPTH      (20)

#define E22_UART_HANDLE            huart3

// E22无线模块配置参数宏定义
// ******************************************************************************************
// 模块地址（16位），用于区分同网络下不同设备，需和通信对端匹配（固定点传输模式下）
#define E22_ADDRESS                ((uint16_t)0x1234)

// 网络ID（8位），用于划分不同通信网络，相同网络ID的设备才能互相通信
#define E22_NET_ID                 ((uint8_t)0x07)

// UART串口波特率配置，取值为枚举类型E22_UARTSpeed_9600（对应9600bps）
#define E22_UART_SPEED             E22_UARTSpeed_9600

// UART串口奇偶校验位配置，取值为枚举类型E22_UARTParityBit_8N1（8位数据位、无校验、1位停止位）
#define E22_PARITY_BIT             E22_UARTParityBit_8N1

// 空中传输速率配置，取值为枚举类型E22_AirRate_2400（对应2400bps）
// 空中速率越低，传输距离越远，抗干扰能力越强；速率越高，传输速率越快，距离越近
#define E22_AIR_RATE               E22_AirRate_2400

// 子包长度配置，取值为枚举类型E22_SubPacket_240B（子包长度240字节）
// 子包长度影响单次传输的数据量和传输效率
#define E22_SUB_PACKET             E22_SubPacket_240B

// RSSI（接收信号强度指示）功能使能/禁用，取值为枚举类型E22_RSSI_Disable（禁用RSSI）
#define E22_RSSI                   E22_RSSI_Disable

// 发射功率配置，取值为枚举类型E22_TransmissionPower_33dBm（发射功率33dBm）
// 发射功率越高，传输距离越远，功耗也越大
#define E22_TRANSMISSION_POWER     E22_TransmissionPower_33dBm

// 通信信道（8位），E22模块信道范围通常为1~32，不同信道对应不同工作频率
#define E22_CHANNEL                ((uint8_t)13)

// RSSI字节功能使能/禁用，取值为枚举类型E22_RSSIByte_Disable（禁用RSSI字节）
// 启用后接收数据时会附带RSSI值字节，用于判断信号强度
#define E22_RSSI_BYTE              E22_RSSIByte_Disable

// 传输模式配置，取值为枚举类型E22_TransmissionMode_FixedPoint（定点传输模式）
// 定点模式：数据仅发送给指定地址和网络ID的设备；透传模式：无需指定地址，同信道设备均可接收
#define E22_TRANSMISSION_MODE      E22_TransmissionMode_FixedPoint

// 中继功能使能/禁用，取值为枚举类型E22_Repeater_Disable（禁用中继）
// 启用中继可延长传输距离，但会增加传输延迟和功耗
#define E22_REPEATER               E22_Repeater_Disable

// LBT（先听后发）功能使能/禁用，取值为枚举类型E22_LBT_Disable（禁用LBT）
// LBT：发送数据前先检测信道是否空闲，避免同信道干扰，适用于多设备通信场景
#define E22_LBT                    E22_LBT_Disable

// WOR（唤醒接收）功能控制，取值为枚举类型E22_WOR_Receiver（WOR接收模式）
// WOR模式：接收端低功耗休眠，等待发送端唤醒信号，降低整体功耗
#define E22_WOR_CONTROL            E22_WOR_Receiver

// WOR唤醒周期配置，取值为枚举类型E22_WORPeriod_1000ms（唤醒周期1000ms）
// 唤醒周期越长，接收端功耗越低，但唤醒响应延迟越大
#define E22_WOR_PERIOD             E22_WORPeriod_1000ms

// 加密密钥（16位），用于数据加密传输，通信双方需配置相同密钥才能解密数据
#define E22_CRYPT                  ((uint16_t)0x2222)
// ******************************************************************************************
#endif // !__E22_CONF_H

