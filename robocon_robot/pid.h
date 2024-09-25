/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      pid.h
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     pid.c 的头文件
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
#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

#define ABS(x)      ((x)>0? (x):(-(x)))

typedef struct _pid
{
	float  Proportion;		   // 比例常数Kp
	float  Integral;		   // 积分常数Ki  
	float  Derivative;		   // 微分常数Kd  
	  float  PrevError; 	   // 上上次的Error[-2]
	float  LastError;		   // 上次的Error[-1]  
	  float  Error; 		   // 当前的Error[0]
	  float  DError;		   // 当前的误差变化率 Error[0]-Error[-1]
	float  SumError;		   //  误差累计值	
	  float  Integralmax;	   //  积分限幅
	  float  output;		   //  pid控制器的输出值
	  float  outputmax; 	   //  pid控制器输出值限幅
	  float  errormax;		   //  误差的最大值限制  
	  uint8_t first_flag;	   //  第一次标志位，用于初始化控制器或特殊处理
	  float  deadzone;		   //  死区值  

}pid;

void PID_parameter_init(pid *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone);
float PID_abs_limit(float a, float ABS_MAX);
void PID_position_PID_calculation(pid *pp, float CurrentPoint, float NextPoint); 
void PID_incremental_PID_calculation(pid *pp, float CurrentPoint, float NextPoint);
void PID_position_PID_calculation_by_error(pid *pp, float error);



#endif /* PID_H */

