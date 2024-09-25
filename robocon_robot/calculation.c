#include "calculation.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "hardware.h"
#include "MoveBase.h"
#include "FSM.h"
///*!
// * \fn     pos_calculate
// * \brief  底盘速度逆解 
// *         根据电机摆放位置更改 
// * \param  [in] int num    #
// * \param  [in] float vx   #
// * \param  [in] float vy   #
// * \param  [in] float w    #
// * 
// * \retval float
// */
//float pos_calculate(int num,float vx,float vy,float Vw)
//{
//	float va=Vw*RADIUS;
//	if(num==1)
//	{
//		float out1=(float)(-vx*0.5-vy*sqrt(3)/2+va);
//		return out1;
//	}
//	
//	else if(num==0)
//	{
//		float out2=(float)(vx+va);
//		return out2;
//	
//	}
//	
//	else if(num==2)
//	{
//		float out3=(float)(-vx*0.5+vy*sqrt(3)/2+va);
//		return out3;
//	}
//	
//	else
//		return 0;
//}


/*!
 * \fn     v_to_rpm
 * \brief  将轮子线速度转化为转速
 *          
 * \param  （float）v 
 * 
 * \retval float
 */
 
 float v_to_rpm(float v)
 {
	float rpm=v/Rwheel/(2*PI)*60*19;
	 return rpm;
 }
 
 
/*!
 * \fn     rpm_to_v
 * \brief  将轮子转速转化为线速度
 *          
 * \param  （float）rpm
 * 
 * \retval float
 */
 
 float rpm_to_v(float rpm)
 {
	float v=rpm*2*PI/60*Rwheel;
	 return v;
 }
 
 
 float one_yaw = 45.0* PI / 180.0f;
 /*!
 * \fn     Kinematic_Analysis1
 * \brief  一区姿态三全向轮运动逆解 将目标世界坐标速度速度转化为每个轮子转速
 *
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 * 
 * \retval void
 */                    
void Kinematic_Analysis1(ROBOT_CHASSIS motor_target)
{

	float now_yaw=0;
	if(Yaw_Clock==0)now_yaw = -ROBOT_REAL_POS_DATA.POS_YAW* PI / 180.0f;
	float Vx=cos(one_yaw+now_yaw) * (motor_target.World_V[1]) + sin(one_yaw+now_yaw) * (motor_target.World_V[0]);
	float Vy=-sin(one_yaw+now_yaw) * (motor_target.World_V[1]) + cos(one_yaw+now_yaw)*(motor_target.World_V[0]);
	float W=-motor_target.World_V[2];
	
	MOTOR_REAL_INFO[1].TARGET_RPM = -v_to_rpm((W*RADIUS+0.6712*Vx-Vy*0.7412));

	MOTOR_REAL_INFO[2].TARGET_RPM = -v_to_rpm(W*RADIUS+0.6712*Vx+Vy*0.7412);

	MOTOR_REAL_INFO[0].TARGET_RPM= v_to_rpm(-Vx+RADIUS*W);
}

/*!
 * \fn     Kinematic_Analysis
 * \brief  二区三全向轮运动逆解 将目标世界坐标速度速度转化为每个轮子转速
 *
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 * 
 * \retval void
 */                    
void Kinematic_Analysis2(ROBOT_CHASSIS motor_target)
{
	
	world_to_robot(&motor_target);
	
	MOTOR_REAL_INFO[1].TARGET_RPM = v_to_rpm((motor_target.Robot_V[2]*RADIUS+0.6712*motor_target.Robot_V[1]-motor_target.Robot_V[0]*0.7412));

	MOTOR_REAL_INFO[2].TARGET_RPM = v_to_rpm(motor_target.Robot_V[2]*RADIUS+0.6712*motor_target.Robot_V[1]+motor_target.Robot_V[0]*0.7412);

	MOTOR_REAL_INFO[0].TARGET_RPM= -v_to_rpm(-motor_target.Robot_V[1]+RADIUS*motor_target.Robot_V[2]);
}


/*!
 * \fn     world_to_robot
 * \brief  将世界坐标系速度换算成机器人坐标系速度
 *          
 * \param  ROBOT_REAL_POS robot_now_pos
 * 
 * \retval void
 */
void world_to_robot(ROBOT_CHASSIS* robot_data)
{
	float now_yaw=-ROBOT_REAL_POS_DATA.POS_YAW* PI / 180.0f;
	
	// x方向上的转换
	robot_data->Robot_V[1]= cos(now_yaw) * (robot_data->World_V[1]) + sin(now_yaw) * (robot_data->World_V[0]);
	// y方向上的转换
	robot_data->Robot_V[0]= -sin(now_yaw) * (robot_data->World_V[1]) + cos(now_yaw)*(robot_data->World_V[0]);
	// yaw上的转换
	robot_data->Robot_V[2]= robot_data->World_V[2];
	

	return;
}


///*!
// * \fn     Kinematic_Analysis
// * \brief  二区三全向轮运动逆解 将目标世界坐标速度速度转化为每个轮子转速
// *
// * \param  [in] float Vx   #
// * \param  [in] float Vy   #
// * 
// * \retval void
// */                    
//void Kinematic_Analysis3(ROBOT_CHASSIS motor_target)
//{
//	float now_yaw=ROBOT_REAL_POS_DATA.POS_YAW* PI / 180.0f;
//	
//	MOTOR_REAL_INFO[1].TARGET_RPM = v_to_rpm(motor_target.World_V[2]*RADIUS+0.6712*(motor_target.World_V[1]*cos(now_yaw)-motor_target.World_V[0]*sin(now_yaw));

//	MOTOR_REAL_INFO[2].TARGET_RPM = v_to_rpm(motor_target.Robot_V[2]*RADIUS+0.6712*motor_target.Robot_V[1]+motor_target.Robot_V[0]*0.7412);

//	MOTOR_REAL_INFO[0].TARGET_RPM= -v_to_rpm(motor_target.World_V[2]*RADIUS-motor_target.World_V[1]*cos(now_yaw)-motor_target.World_V[0]*sin(now_yaw));
//}


KalmanFilter Kalman_LaserX;
KalmanFilter Kalman_LaserY;




/*
 *  函数名：Kalmanfiter_Init
 *  功能描述：卡尔曼系数初始化
 *  输入参数：KalmanFilter *EKF
 *  输出参数：无
 *  返回值：无
*/
void Kalmanfiter_Init(KalmanFilter *EKF)
{
	EKF->LastP=0.02;
	EKF->NewP=0;
	EKF->Out=0;
	EKF->Kg=1;
	EKF->Q=0.01;
	EKF->R=5;
}

/*
 *  函数名：kalmanfiter
 *  功能描述：卡尔曼滤波器
 *  输入参数：KalmanFilter *EKF float input
 *  输出参数：无
 *  返回值：无
*/
void kalmanfiter(KalmanFilter *EKF,float input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}






