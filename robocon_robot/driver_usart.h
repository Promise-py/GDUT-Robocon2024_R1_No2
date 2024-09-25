#ifndef __DRIVER_USART_H
#define __DRIVER_USART_H

#include "stm32f4xx_hal.h"

#define RX_BUF_SIZE 12
#define Max_BUFF_Len 100

extern int Action_State;
/*
 *  函数名：EnableDebugIRQ
 *  功能描述：使能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
extern void EnableDebugIRQ(void);

/*
 *  函数名：DisableDebugIRQ
 *  功能描述：失能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
extern void DisableDebugIRQ(void);


typedef struct
{
    uint16_t Size;                     // 接收到的数据长度
    uint8_t ReceiveBuff[Max_BUFF_Len]; // 接收缓存
    uint8_t ProcessBuff[Max_BUFF_Len]; // 处理缓存

    uint8_t HEAD; // 帧头
    uint8_t ADDR; // 地址


    uint8_t ID;                 // 功能码
    uint8_t Len;                // 数据长度
//    uint8_t DATA[Max_DATA_Len]; // 数据内容
    uint8_t SC;                 // 和校验
    uint8_t AC;                 // 附加校验
    int new_data;               // 新 且 有效 数据标志
    int Checked;                // 校验结果

    uint8_t END[2]; // 帧尾
} Usart_struct;

extern Usart_struct Usart2;
extern Usart_struct Usart6;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

#endif /* __DRIVER_USART_H */
