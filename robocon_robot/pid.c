/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      pid.c
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     存放pid参数设置和计算函数的文件
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
#include "pid.h"

/* pid参数的初始化 */
void PID_parameter_init(pid *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone)  
{  
	pp->Integralmax = Integralmax;
	pp->outputmax = outputmax;
    pp->Proportion = Kp;
	pp->Integral   = Ki;
	pp->Derivative = Kd;
    pp->DError = pp->Error = pp->SumError = pp->output = pp->LastError = pp->PrevError = pp->errormax = 0.0f;
	pp->first_flag = 1;
	pp->deadzone = deadzone;
} 

// 对变量进行范围限制
float PID_abs_limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}


/* 位置式pid计算 */
void PID_position_PID_calculation(pid *pp, float CurrentPoint, float NextPoint)  
{   
	
	if(pp->first_flag == 1)
	{
		pp->LastError = NextPoint - CurrentPoint;
		pp->PrevError = NextPoint - CurrentPoint;
		pp->first_flag = 0;
	}
	
	pp->Error =  NextPoint -  CurrentPoint;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->output =  pp->Proportion * pp->Error +   \
								PID_abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
								pp->Derivative * pp->DError ;  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}

// 位置式PID,直接传入误差
void PID_position_PID_calculation_by_error(pid *pp, float error)  
{   
	if(pp->first_flag == 1)
	{
		pp->LastError = error;
		pp->PrevError = error;
		pp->first_flag = 0;
	}	
	
	pp->Error =  error;          
	pp->SumError += pp->Error;                      
	pp->DError = pp->Error - pp->LastError;
	
	pp->output =  pp->Proportion * pp->Error +   \
								PID_abs_limit(pp->Integral * pp->SumError, pp->Integralmax ) +   \
								pp->Derivative * pp->DError ;  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  pp->output = -pp->outputmax; 
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
}


/* 增量式PID计算 */
void PID_incremental_PID_calculation(pid *pp, float CurrentPoint, float NextPoint)  
{  
	pp->Error =  NextPoint - CurrentPoint;                               
	pp->DError = pp->Error - pp->LastError;
	
	pp->output +=  pp->Proportion * (pp->DError)+   \
								 PID_abs_limit(pp->Integral * pp->Error, pp->Integralmax ) +   \
								 pp->Derivative * ( pp->Error +  pp->PrevError - 2*pp->LastError);  

	if(pp->output > pp->outputmax )  pp->output = pp->outputmax;
	if(pp->output < - pp->outputmax )  
		pp->output = -pp->outputmax;
	pp->PrevError = pp->LastError;  
	pp->LastError = pp->Error;
	
	if(ABS(pp->Error) < pp->deadzone)
	{
		pp->output = 0;
	}
	
}



