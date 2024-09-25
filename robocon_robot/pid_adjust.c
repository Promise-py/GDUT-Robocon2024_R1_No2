/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      pid_adjust.c
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     上位机调参的文件
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
#include "pid_adjust.h"
#include "driver_usart.h"
#include "usart.h"
#include "motor.h"
#include "pid.h"
#include "pid_adjust.h"



uint8_t RxBuffer[1];//串口接收缓冲
uint16_t RxLine = 0;//指令长度
uint8_t DataBuff[200];//指令内容


/*!
 * \fn     Adjust_Init
 * \brief  打开串口接收中断并将数据存到缓冲区中的设置
 *          
 * \param  [in] void   #
 * 
 * \retval void
 */
void Adjust_Init(void)
{
	/* 在这里打开串口的接收发送中断设置 */
    HAL_UART_Receive_IT(&huart1,(uint8_t *)RxBuffer,1);//开启串口中断，我用的是串口1
}


float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    int lsb; // 用来存储小数部分
    for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
    {
        if (DataBuff[i] == '=') 
        {
            data_Start_Num = i + 1; // +1是直接定位到数据起始位
        }
        if (DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    /* 识别出小数点的位置，然后分别处理整数部分和小数部分 */
    uint8_t point_position = 0; 
    for(uint8_t i = data_Start_Num; i <= data_End_Num; i++)
    {
        if(DataBuff[i] == '.')
        {
            point_position = i; 
            break;
        }
    }
    for(uint8_t i = data_Start_Num; i < point_position; i++ )
    {
        data_return = data_return*10 + DataBuff[i] - 48;
    }
    float divisor = 10.0;
    for(uint8_t i = point_position + 1; i <= data_End_Num; i++)
    {
         data_return += (DataBuff[i] - 48) / divisor;
         divisor *= 10;              
    }
    if(minus_Flag == 1)  
    {
        data_return = -data_return;
    }
    return data_return;
}


void USART_PID_Adjust(uint8_t motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
    printf("data=%.2f\r\n",data_Get);
	if(motor_n ==1)
	{
		/* 先是速度环 */
	    if(DataBuff[0]=='P' && DataBuff[1]=='1') // 
		    MOTOR_PID_RPM[0].Proportion = data_Get;
			//MOTOR_PID_POS[0].Proportion = data_Get;
	    else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 
	        MOTOR_PID_RPM[0].Integral = data_Get;
			//MOTOR_PID_POS[0].Integral = data_Get;
	    else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 
	        MOTOR_PID_RPM[0].Derivative = data_Get;
			//MOTOR_PID_POS[0].Derivative = data_Get;
	    else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 电机2速度环P
		    MOTOR_PID_RPM[1].Proportion = data_Get;
			//MOTOR_PID_POS[1].Proportion = data_Get;
	    else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 电机2速度环I
	        MOTOR_PID_RPM[1].Integral = data_Get;
			//MOTOR_PID_POS[1].Integral = data_Get;
	    else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 电机2速度环D
	        MOTOR_PID_RPM[1].Derivative = data_Get;
			//MOTOR_PID_POS[1].Derivative = data_Get;
		else if(DataBuff[0]=='P' && DataBuff[1]=='3') // 电机3速度环P
		    MOTOR_PID_RPM[2].Proportion = data_Get;
			//MOTOR_PID_POS[2].Proportion = data_Get;
	    else if(DataBuff[0]=='I' && DataBuff[1]=='3') // 电机3速度环I
	        MOTOR_PID_RPM[2].Integral = data_Get;	
			//MOTOR_PID_POS[2].Integral = data_Get;
	    else if(DataBuff[0]=='D' && DataBuff[1]=='3') // 电机3速度环D
	        MOTOR_PID_RPM[2].Derivative = data_Get;
			//MOTOR_PID_POS[2].Derivative = data_Get;
	    else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
	        MOTOR_REAL_INFO[0].TARGET_RPM = data_Get;
			//MOTOR_REAL_INFO[0].TARGET_POS = data_Get;
	    else if((DataBuff[0]=='s' && DataBuff[1]=='P') && DataBuff[2]=='e') 
	        MOTOR_REAL_INFO[1].TARGET_RPM = data_Get;		
			//MOTOR_REAL_INFO[1].TARGET_POS = data_Get;
		else if((DataBuff[0]=='s' && DataBuff[1]=='p') && DataBuff[2]=='E') 
	        MOTOR_REAL_INFO[2].TARGET_RPM = data_Get;		
			//MOTOR_REAL_INFO[2].TARGET_POS = data_Get;
	}

}


