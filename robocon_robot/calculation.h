#ifndef CALCULATION_H
#define CALCULATION_H

#include "stm32f4xx_hal.h"
#include "hardware.h"

#define number2 					1.414213562373095 //!< //The square root of two 
#define RADIUS 						0.26577 //!< 轮子（中心）到底盘圆心的距离（近似） 

#define Rwheel						0.0719
#define PI 							3.1415926

//卡尔曼滤波参数结构体
typedef struct KalmanFilter
{
	float LastP;		//上一次协方差
	float NewP;		//最新的协方差
	float Out;			//卡尔曼输出
	float Kg;				//卡尔曼增益
	float Q;				//过程噪声的协方差
	float R;				//观测噪声的协方差
}KalmanFilter;

extern KalmanFilter Kalman_LaserX;
extern KalmanFilter Kalman_LaserY;
extern float one_yaw;
//float pos_calculate(int num,float vx,float vy,float Vw);
void Kinematic_Analysis1(ROBOT_CHASSIS motor_target);
void Kinematic_Analysis2(ROBOT_CHASSIS motor_target);
float v_to_rpm(float v);
float rpm_to_v(float rpm);
void world_to_robot(ROBOT_CHASSIS* robot_data);
void Kalmanfiter_Init(KalmanFilter *EKF);
void kalmanfiter(KalmanFilter *EKF,float input);



#endif


