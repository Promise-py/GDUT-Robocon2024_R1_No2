
#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f4xx_hal.h"
#include "motor.h"

//define


#define SWA		PPM_buf[4]				//AUX4 1000~2000//没用
#define SWB		PPM_buf[5]				//AUX2 1000-1500-2000
#define SWD		PPM_buf[7]			//AUX1 1000~2000
#define SWC		PPM_buf[6]				//AUX3 1000-1500-2000

#define ROCK_R_X			PPM_buf[0]					//YAW  1000-1500-2000
#define ROCK_R_Y			PPM_buf[1]					//THR  1000-1500-2000

#define ROCK_L_Y			PPM_buf[2]				//ROLL 1000-1500-2000//未知bug
#define	ROCK_L_X		  	PPM_buf[3]				//PITCH 1000-1500-2000P

#define Rotary_L PPM_buf[8]

#define ZONE_1   0x01
#define ZONE_2   0x02


static uint16_t PPM_Databuf[10]={0};

extern uint16_t PPM_buf[10];
typedef struct
{
	struct  //遥控原始数据，8通道
	{
	 uint16_t roll;			//右摇杆
	 uint16_t pitch;		//
	 uint16_t thr;
	 uint16_t yaw;
	 uint16_t AUX1;
	 uint16_t AUX2;
	 uint16_t AUX3;
	 uint16_t AUX4; 
	 uint16_t BUX1;
	 uint16_t BUX2;		
	}Remote; 

}Air_Contorl;

typedef struct KEY_Type//发射数据
{
  
	int KEY_armtop;//armtop
	int KEY_armbottom;//armbottom
	int KEY_push;//push
}KEY_Type;


/* Action读到的数据 */
// 东大全场定位模块定位的位置
typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;	
	float POS_X;
	float POS_Y;
	float W_Z;
	
	float LAST_POS_X;
	float LAST_POS_Y;
	
	float DELTA_POS_X;
	float DELTA_POS_Y;	
	
	//最后的真实位置
	float REAL_X;
	float REAL_Y;
}ACTION_GL_POS;


/* 机器人的真实位置 */
typedef struct ROBOT_CHASSIS
{

	float World_V[3]; // Y , X , W
	float Robot_V[3];//Y , X , W
	float Position[2];//当前坐标
	float Motor_RPM[3];
	float expect_angle ;
	float Angle;
} ROBOT_CHASSIS;


// 机器人的真实位置
typedef struct ROBOT_REAL_POS
{
  float POS_X;
  float POS_Y;     
  float POS_YAW;
int robot_location;
}ROBOT_REAL_POS;


/*激光数据*/
typedef struct Laser_Data
{
	float Laser_X;
	float Laser_Y;
	unsigned char Error_Flag1;
	unsigned char Error_Flag2;
} Laser_Data;

extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern struct ACTION_GL_POS ACTION_GL_POS_INFO;
extern ROBOT_CHASSIS ROBOT_REAL_POS_INFO;
extern Laser_Data Laser_Real_Data;
extern int BallPoint;
extern uint32_t TIME_ISR_CNT;
extern uint32_t LAST_TIME_ISR_CNT;
extern uint16_t Time_Sys[4];
extern uint16_t Microsecond_Cnt;

void remote_control(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Update_Action_gl_position(float value[6]);
void Laser_ReadData2(float* Laser_Data);
void Laser_ReadData(float* Laser_Data);
 void processData(uint8_t* data,Laser_Data* laser,int uart);
void Adjust_Countrol(void);
void Remote_Process(void);
void ZONE2_Adjust(void);
#endif


