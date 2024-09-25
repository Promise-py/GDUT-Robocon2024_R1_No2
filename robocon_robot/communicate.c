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
#include "hardware.h"
#include "MoveBase.h"
/* Private  variables ---------------------------------------------------------*/


unsigned char UART4_Receiver= 0;
unsigned char UART2_Receiver=0;
unsigned char UART6_Receiver=0;
unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};


int Mode=0;//shoot/rising/turn/absorb_ball

int move_flag=0;
int Last_Flag=0;

NO1_DATA No1_Data;				
/*
 *  函数名：UART4_Send_String
 *  功能描述：usart4发送字符串
 *  输入参数：八位字符串，字符串长度
 *  输出参数：无
 *  返回值：无
*/
void UART4_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//静态变量防止数据丢失
	while(length<sendSize)
	{
		while(!(UART4->SR&(0x01<<7)));//发送缓冲区为空(发送数据缓冲位应该为第八位)
		UART4->DR=*p;
		p++;
		length++;
	}
	length=0;
}


/**
  * @brief  计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
  * @param   数组地址、数组大小
  * @retval 
  */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}



/**
  * @brief  使用uart4和底盘通讯（接受）
  * @param   
  * @retval 
  */
//接受联合体
unsigned char  receiveBuff_u4[22] = {0};
union uart4_ReceiveData
{
	float d;
	unsigned char data[4];
}real_x,real_y,real_w;

union uart4_ReceiveData_int
{
	int d;
	unsigned char data[4];
}chassic_flag; 

int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1); // 继续监听
	//接收消息头
	if(Start_Flag == START)
	{
		if(UART4_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				receiveBuff_u4[0]=header[0];         //buf[0]
				receiveBuff_u4[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收数据的长度
				receiveBuff_u4[2] = UART4_Receiver;
				dataLength     = receiveBuff_u4[2];            //buf[2]
				USARTBufferIndex++;
				break;
			
			case 1://接收所有数据，并赋值处理 
				receiveBuff_u4[j + 3] = UART4_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
				
			case 2://接收校验值信息(设定为0x07)
				receiveBuff_u4[2 + dataLength] = UART4_Receiver;
				checkSum = getCrc8(receiveBuff_u4, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						real_x.data[k]  = receiveBuff_u4[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						real_y.data [k] = receiveBuff_u4[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						real_w.data [k]  = receiveBuff_u4[k + 11]; //buf[11]  buf[12] buf[13]  buf[14]
						chassic_flag.data[k] = receiveBuff_u4[k + 15]; //buf[15]  buf[16] buf[17]  buf[18]
					}				
					
					//赋值操作
					*action_x = -real_x.d;
					*action_y = -real_y.d;
					*action_w = real_w.d;
					
					if(chassic_flag.d>100||chassic_flag.d<0)*flag=Last_Flag;
					else 
					{
						*flag = chassic_flag.d;
						Last_Flag=chassic_flag.d;
					}
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}


/**
  * @brief  使用uart4和底盘通讯（发送）
* @param   
  * @retval 
  */
//unsigned char buf[22]={0};//数据缓存区

union Uart4_SendData//发送数据的共用体
{
	float d;
	unsigned char data[4];
}uart4_vx,uart4_vy,uart4_vw;

union Uart4_SendData_int//发送数据的共用体
{
	int d;
	unsigned char data[4];
}uart4_flag;



void Usart4_SendData(float X,float Y,float W,int flag)
{
	
	int i,length = 0;
	unsigned char  buf_u4[22] = {0};
	
	 //memset(buf,0,50);//清空数组
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u4[i]=header[i];//协议数据头
	}
	length = 17;
	buf_u4[2] =length;//sizeof
	
	for(i=0;i<4;i++)
	{
		buf_u4[i+3]=uart4_vx.data[i];
		buf_u4[i+7]=uart4_vy.data[i];
		buf_u4[i+11]=uart4_vw.data[i];
		buf_u4[i+15]=uart4_flag.data[i];
	}

	buf_u4[3+length-1]=getCrc8(buf_u4,3+length);
	buf_u4[3+length]=ender[0];
	buf_u4[3+length+1]=ender[1];
	
	UART4_Send_String(buf_u4,sizeof(buf_u4));//利用字符串发送函数发送数据
}

float last_world_vx=0;
float last_world_vy=0;

/**
  * @brief  上层信息处理
  * @param   
  * @retval 
  */
void No1_Porcessing(NO1_DATA Receive_NO1)
{
	switch(Receive_NO1.flag)
	{
		case Remote_ZONE1:
			ROBOT_REAL_POS_DATA.robot_location=ZONE_1;
//			Robot_Chassis.World_V[1]=0.3*Receive_NO1.WOrld_x*0.5+last_world_vx*0.7;
//			Robot_Chassis.World_V[0]=0.3*Receive_NO1.WOrld_y*0.5+last_world_vy*0.7;
//			Robot_Chassis.World_V[2]=Receive_NO1.WOrld_w;
//			last_world_vx=Robot_Chassis.World_V[1];
//			last_world_vx=Robot_Chassis.World_V[0];
			Robot_Chassis.World_V[1]=-Receive_NO1.WOrld_x*0.05;
			Robot_Chassis.World_V[0]=-Receive_NO1.WOrld_y*0.05;
			Robot_Chassis.World_V[2]=0;
		break;

		case Remote_ZONE2:
			ROBOT_REAL_POS_DATA.robot_location=ZONE_2;
			Robot_Chassis.World_V[1]=Receive_NO1.WOrld_x;
			Robot_Chassis.World_V[0]=Receive_NO1.WOrld_y;
			Robot_Chassis.World_V[2]=0;
		break;
		
		case Laser:
			Laser_Real_Data.Laser_X=Receive_NO1.WOrld_x/1000;
			Laser_Real_Data.Laser_Y=Receive_NO1.WOrld_y/1000;
		break;
	}


}



/*
 *  函数名：Usart4_Filter
 *  功能描述：串口4过滤器
 *  输入参数：void
 *  输出参数：无
 *  返回值：无
*/
void Usart4_Filter(void)
{
	if(No1_Data.flag>100||No1_Data.flag<0)No1_Data.flag=Last_Flag;
	else return;
}



/*
 *  函数名：Waiting_MoveFlag
 *  功能描述：给No1应答
 *  输入参数：void
 *  输出参数：无
 *  返回值：无
*/

void Waiting_MoveFlag(float Rising_LeftLevel,float Rising_RightLevel)
{
	while(No1_Data.flag!=Move_Flag);
	No1_Data.flag=0;
	Usart4_SendData(Rising_LeftLevel,Rising_RightLevel,Receive,Rise);
	return;
}




