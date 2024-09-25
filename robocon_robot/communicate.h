/*!
 *****************************************************************************
 *
 *  Copyright ? 2017-2018 All Rights Reserved.
 *
 * \file      communicate.h
 * \author    Py
 * \version   1.0
 * \date      2024��5��12��
 * \brief      communicate.c��ͷ�ļ�
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024��5��12��
      author: py
      change: create file

*****************************************************************************/
#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "stm32f4xx_hal.h"
extern union uart4_ReceiveData real_x,real_y,real_w;

extern union uart4_ReceiveData_int chassic_flag;

#define START 0X11
#define  UART_DISABLE_RE(USARTx)      USARTx.Instance->CR1&= (~(uint32_t)0x0004)


/**
  ******************************************************************************
  * @file    communicate.c
  * @author  Py
  * @version V1.0.0
  * @date    2024/5/12
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "communicate.h"
#include "usart.h"

/* Private  variables ---------------------------------------------------------*/


#define Remote_ZONE1 0x11
#define Remote_ZONE2 0x12

#define Move_Flag 0x21
#define Receive 0x22

#define Laser 0x02


#define STOP 0x00
#define Shoot  0x01
#define Rise   0x02
#define Turn   0x03
#define Absorb 0x04
#define Inspection 0x05
#define Action_Flag  0x06
#define ZONE2_Inspection 0x07
#define Clor_Choice 0x08
#define Open 0x011

#define Red 0x01
#define Blue 0x02

#define Retry 0x09
#define Seedling 0x10
#define AbsorbChoice 0xAB

typedef struct
{
  float WOrld_x;
  float WOrld_y;
  float WOrld_w;
  int flag;
}NO1_DATA;
extern NO1_DATA No1_Data;
extern unsigned char UART4_Receiver;
extern unsigned char UART2_Receiver;
extern unsigned char UART6_Receiver;
extern int move_flag;

void Usart4_Filter(void);
void No1_Porcessing(NO1_DATA Receive_NO1);

void UART4_Send_String(uint8_t *p,uint16_t sendSize);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);
int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag);
void Usart4_SendData(float X,float Y,float W,int flag);
void Waiting_MoveFlag(float Rising_LeftLevel,float Rising_RightLevel);


#endif


