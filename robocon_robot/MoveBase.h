/*!
 *****************************************************************************
 *
 *  Copyright ? 2017-2018 Keten. All Rights Reserved.
 *
 * \file      MoveBase.h
 * \author    Py
 * \version   1.0
 * \date      2024��5��5��
 * \brief      MoveBase.c��ͷ�ļ�
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024��5��5��
      author: py
      change: create file

*****************************************************************************/
#ifndef MOVEBASE_H
#define MOVEBASE_H

#include "stm32f4xx_hal.h"
#include "hardware.h"

#define PI 3.1415926
typedef struct
{
	float X;
	float Y;
	float Yaw;
	float V_x;
	float V_y;
	float W;
}PATH_TYPEDEF;

typedef struct
{
 float Laser_X;
 float Laser_Y;
}LASER_REAL_DATA;



extern ROBOT_CHASSIS Robot_Chassis;
extern float X0[4+7];
extern float Y0[4+7];
extern float Yaw0[4+7];

extern float X1[4+7];
extern float Y1[4+7];
extern float Yaw1[4+7];

extern float X2[4+7];
extern float Y2[4+7];
extern float Yaw2[4+7];

extern float X3[4+7];
extern float Y3[4+7];
extern float Yaw3[4+7];

extern float YawAdjust_error;

int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);
void AngleLimit(float *angle);
int YawAdjust(float Target_angle);
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max);
void PDController(PATH_TYPEDEF target_point,ROBOT_REAL_POS robot_now_pos);

int chassis_TrapezoidPlaning(float POS_X_start,
							float POS_Y_start,
							float POS_X_end,
							float POS_Y_end,
							float POS_YAW,
							float V_start,
							float V_end,
							float V_max,
							float R_ac,
							float R_de);
void Move_Init(void);
void MoveCtrl(void);							
int Laser_calibration(float X, float Y,float yaw,float v_max,int location);
void Location_Adjust(int ZONE1_finish);
extern pid yaw_pid;
#endif

							
							