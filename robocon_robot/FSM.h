
#ifndef FSM_H
#define FSM_H

#include "stm32f4xx_hal.h"

#define TrapezoidMove_Zone1 0x01
#define LaserMove_Zone1 0x02
#define Photogate 0x03
#define REMOTE 0x04
#define Remote_Adjust 0x08

#define TrapezoidMove_Zone2 0x05
#define LaserMove_Zone2 0x06
#define ChooseBall 0x07
#define ZONE2_Remote 0x09
#define PUT_Adjust 0x10

#define ZONE1 0x01
#define ZONE2 0x02

#define Left_Hand HAL_GPIO_ReadPin(Left_Hand_GPIO_Port,Left_Hand_Pin)
#define Right_Hand HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)

#define Left_Wall HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)
#define Right_Wall HAL_GPIO_ReadPin(Right_Wall_GPIO_Port,Right_Wall_Pin)
typedef struct
{
	int Move_point;
	int Absorb_point; 
}MOVE_FLAG;

void remote_FSM(void);
static int a=0;
extern float angle_left;
extern float angle_right;
extern int Time_ms;
extern int Absorb_pointColumn;//�����е�λ
extern int Absorb_pointRow;//�����е�λ
extern unsigned char Remote_MoveFlag;
extern float yaw;
extern unsigned char Move_State;
extern unsigned char Colour_Choice;//����ѡ���־
extern uint8_t ZONE_State;
extern uint8_t PointChoice;
extern uint8_t Choice_Flag;
extern int finish;
extern float SeedlingTAKE_X[12];
extern float SeedlingTAKE_Y[12];
extern float SeedlingPUT_X[12];
extern float SeedlingPUT_Y[12];
extern float Red_SeedlingTAKE_X[12];
extern float Red_SeedlingTAKE_Y[12];
extern float Red_TakeCalibration_X[12];
extern float Red_TakeCalibration_Y[12];
extern float Red_SeedlingPUT_X[12];
extern float Red_SeedlingPUT_Y[12];
extern float Blue_SeedlingTAKE_X[12];
extern float Blue_SeedlingTAKE_Y[12];
extern float Blue_TakeCalibration_X[12];
extern float Blue_TakeCalibration_Y[12];
extern float Blue_SeedlingPUT_X[12];
extern float Blue_SeedlingPUT_Y[12];
extern unsigned char Yaw_Clock;
void move_test(void);
void move_Laser(float SeedlingTAKE_X[12],float SeedlingTAKE_Y[12],float SeedlingPUT_X[12],float SeedlingPUT_Y[12],float TakeCalibration_X[12],float TakeCalibration_Y[12]);
void Activate_FSM(void);
void Laser_FSM(void);
void Photogate_FSM(void);

void Retry_FSM(void);
void MOVE_FSM(void);

#endif

