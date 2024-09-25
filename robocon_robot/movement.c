/**
  ******************************************************************************
  * @file    movement.c
  * @author  
  * @version V1.0.0
  * @date    2024/6/10
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "movement.h"
#include "MoveBase.h"
#include "pid.h"
#include "hardware.h"
#include "math.h"
#include "calculation.h"
#include "hardware.h"
#include "driver_usart.h"
#include "communicate.h"
#include "FSM.h"
/* Private  variables ---------------------------------------------------------*/




unsigned char Flag_SendAction=0;//action状态发送完成标志位
/*
 *  函数名：Action_Inspection
 *  功能描述：action初始化完成检测函数
 *  输入参数：void
 *  输出参数：无
 *  返回值：无
*/
void Action_Inspection(void)
{
	if(Action_State>1000&&!Flag_SendAction)//遥控启动标志位==0)
	{
//		Usart4_SendData(0,0,0,Action_Flag);
		HAL_GPIO_WritePin(Action_GPIO_Port,Action_Pin,GPIO_PIN_RESET);
		Flag_SendAction=1;
	}
}



//左手夹苗
void Left_Clamp(void)
{
	Usart4_SendData(130,1,0,Rise);
}


//左手取苗
void Left_Take(int finish)
{
	if(!Left_Hand)
	{
		VelCrl(&MOTOR_REAL_INFO[0],0);
		VelCrl(&MOTOR_REAL_INFO[1],0);
		VelCrl(&MOTOR_REAL_INFO[2],0);
		Usart4_SendData(130,1,0,Rise);
		HAL_Delay(400);
		Move_State=TrapezoidMove_Zone1;
		finish++;
		
	}
	else Move_State=LaserMove_Zone1;
}

//一区重启
void Retry_Location(void)
{
	if(finish>7&&finish<=10)finish=7;
	else if(finish>10&&finish<=13)finish=10;
	else if(finish>13&&finish<=16)finish=13;
	else if(finish>16&&finish<=19)finish=16;
	else if(finish>19&&finish<=22)finish=19;
	else if(finish>22&&finish<=25)finish=22;
	else if(finish>25&&finish<=28)finish=25;
	else if(finish>28&&finish<=31)finish=28;
	else if(finish>31&&finish<=34)finish=31;//十一号苗
	
	Location_Adjust(finish);
}

/*!
 * \fn     Cloud_Turn
 * \brief  
 *          
 * \param  
 * \param  
 * 
 * \retval void
 */
void Cloud_TurnShoot(void)
{
	if(Colour_Choice==Red)
	{
		if(Laser_Real_Data.Laser_X<-1.7&&Laser_Real_Data.Laser_X>-2.2)
		{
			Usart4_SendData(0,0,1,Turn);
		}
		
		else if(Laser_Real_Data.Laser_X<-2.2&&Laser_Real_Data.Laser_X>-2.65)
		{
			Usart4_SendData(0,0,2,Turn);
		}
		
		else if(Laser_Real_Data.Laser_X<-2.65&&Laser_Real_Data.Laser_X>-3.2)
		{
			Usart4_SendData(0,0,3,Turn);
		}
		
		else if(Laser_Real_Data.Laser_X<-3.2&&Laser_Real_Data.Laser_X>-3.65)
		{
			Usart4_SendData(0,0,4,Turn);
		}
		
		else if(Laser_Real_Data.Laser_X<-3.65&&Laser_Real_Data.Laser_X>-4.2)
		{
			Usart4_SendData(0,0,5,Turn);
		}
		
		else if(Laser_Real_Data.Laser_X<-2.65&&Laser_Real_Data.Laser_X>-3.2)
		{
			Usart4_SendData(0,0,3,Turn);
		}
		
	}
	else if(Colour_Choice==Blue)
	{
		if(Laser_Real_Data.Laser_X>3)Usart4_SendData(0,0,0.85-(Laser_Real_Data.Laser_X-3)*0.55,Turn);
		else Usart4_SendData(0,0,0.85,Turn);
		
		if(Laser_Real_Data.Laser_Y>0.85)
		{
			HAL_Delay(1000);
			Usart4_SendData(0,0,6000,Shoot);
		}
		else 
		{
			HAL_Delay(1000);
			Usart4_SendData(0,0,5000,Shoot);
		}
	}
	
}
////右手放苗
//void 