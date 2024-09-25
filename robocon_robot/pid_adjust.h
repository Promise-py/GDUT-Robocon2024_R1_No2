/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      pid_adjust.h
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     pid_adjust.c 的头文件
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024年4月29日
      author: Keten
      change: create file

*****************************************************************************/
#ifndef PID_ADJUST_H
#define PID_ADJUST_H

#include "stm32f4xx_hal.h"


extern uint8_t RxBuffer[1];//串口接收缓冲
extern uint16_t RxLine;//指令长度
extern uint8_t DataBuff[200];//指令内容


#endif /* PID_ADJUST_H */

