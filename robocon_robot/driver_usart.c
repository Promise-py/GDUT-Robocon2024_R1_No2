#include "driver_usart.h"
#include "usart.h"
#include "main.h"
#include "ring_buffer.h"
#include "pid_adjust.h"
#include "pid.h"
#include <stdio.h>
#include "calculation.h"
#include "hardware.h"
#include "communicate.h"

static volatile uint8_t txcplt_flag = 0;    // 发送完成标志，1完成0未完成
static volatile uint8_t rx_data = 0;

float pos_x =0;
float pos_y=0;
float zangle =0;
float xangle = 0;
float yangle =0;
float w_z = 0;
int xx=0;
int yy=0;
int Action_State=0;

Usart_struct Usart2;
Usart_struct Usart6;

uint8_t USART2_Rx_buf[12];
uint8_t USART6_Rx_buf[12];





/*
 *  函数名：EnableDebugIRQ
 *  功能描述：使能USART1的中断+使能UART4的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
void EnableDebugIRQ(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 2);    // 设置USART1中断的优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);            // 使能USART1的中断
    
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE);   // 使能USRAT1的发送和接收中断


	
//	HAL_NVIC_SetPriority(USART2_IRQn, 0, 4);    // 设置USART2中断的优先级
//    HAL_NVIC_EnableIRQ(USART2_IRQn);            // 使能USART2的中断
//    
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC | UART_IT_RXNE);   // 使能USART2的发送和接收中断
	
	
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 2);    // 设置USART3中断的优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);            // 使能USART3的中断
    
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC | UART_IT_RXNE);   // 使能USART3的发送和接收中断
    
	HAL_UART_Receive_IT(&huart3, RxBuffer_for4, 1);//每接收一个数据，就打开一次串口中断接收
	
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);    // 设置UART4中断的优先级
    HAL_NVIC_EnableIRQ(UART4_IRQn);            // 使能UART4的中断
    
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC | UART_IT_RXNE);   // 使能Uuart4的发送和接收中断
	
//	HAL_NVIC_SetPriority(USART6_IRQn, 0, 3);    // 设置USART6中断的优先级
//    HAL_NVIC_EnableIRQ(USART6_IRQn);            // 使能USART6的中断
//    
//    __HAL_UART_ENABLE_IT(&huart6, UART_IT_TC | UART_IT_RXNE);   // 使能USART2的发送和接收中断
	
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//开启USART2空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//开启USART2空闲中断
	
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_Rx_buf, RX_BUF_SIZE);
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_Rx_buf, RX_BUF_SIZE);
		
}

/*
 *  函数名：DisableDebugIRQ
 *  功能描述：失能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
void DisableDebugIRQ(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE);      // 失能USRAT1的发送和接收中断
    
    HAL_NVIC_DisableIRQ(USART1_IRQn);   // 失能USART1的中断
}

/*
 *  函数名：fputc
 *  功能描述：printf/putchar 标准输出函数的底层输出函数
 *  输入参数：ch --> 要输出的数据
 *  输出参数：无
 *  返回值：无
*/
int fputc(int ch, FILE *f)
{
    //txcplt_flag = 0;
    //HAL_UART_Transmit_IT(&huart1, (uint8_t*)&ch, 1);
    //while(txcplt_flag==0);
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    return ch;
}
/*
 *  函数名：fgetc
 *  功能描述：scanf/getchar 标准输出函数的底层输出函数
 *  输入参数：
 *  输出参数：无
 *  返回值：接收到的数据
*/
//int fgetc(FILE *f)
//{
    //uint8_t data = (uint8_t)ch;
    //HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    //return ch;
//}

/*
 *  函数名：USART1_IRQHandler
 *  功能描述：USART1的中断服务函数
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
//void USART1_IRQHandler(void)
//{
    //unsigned char c = 0;
    //if((USART1->SR &(1<<5)) != 0)   // 判断USART1的状态寄存器的第五位即RXNE位是否被置位
    //{
        //c = USART1->DR; // RXNE=1，表明DR寄存器有值，就将它读出来保存到临时变量中；
        //ring_buffer_write(c, &test_buffer); // 将数据保存到环形缓冲区中
    //}
    //HAL_UART_IRQHandler(&huart1);   // HAL库中的UART统一中断服务函数，通过形参判断是要处理谁的中断
//}

/*
 *  函数名：HAL_UART_RxCpltCallback
 *  功能描述：HAL库中的UART接收完成回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   
   if(huart==&huart1)
   {
	   RxLine++;					  //每接收到一个数据，进入回调数据长度加1
	   DataBuff[RxLine-1]=RxBuffer[0];	//把每次接收到的数据保存到缓存数组
	   if(RxBuffer[0]=='!') 		  //接收结束标志位
	   {
		   printf("RXLen=%d\r\n",RxLine);
		   for(int i=0;i<RxLine;i++)
			   printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
		   USART_PID_Adjust(1);//数据解析和参数赋值函数
		   memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
		   RxLine=0;  //清空接收长度
	   }
	   RxBuffer[0]=0;
	   HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);//每接收一个数据，就打开一次串口中断接收
   }
   
   	static uint8_t ch;
	static union
	{
	 uint8_t data[24];
	 float ActVal[6];
	}posture;
	
   if(huart==&huart3)
   {
	Action_State++;
	static uint8_t count = 0;
    static uint8_t i = 0;
	// 如果接收数据寄存器非空，代表USART3接收到了数据，可以读取
	// 处理接收到的数据
	ch = RxBuffer_for4[0];
		// 此时RxBuffer中存储着接收到的数据

	switch(count)
	{
		case 0:
		{
			if(ch == 0x0d) count ++;
			else count = 0;
		}
		break;
			 
		case 1:
		{
			if(ch == 0x0a)
			{
				i = 0;
				count ++;
			}
			else if(ch == 0x0d);
			else count = 0;
		}
		break;
			 
		case 2:
		{
		 posture.data[i] = ch;
		 i ++;
		 if(i >= 24)
		 {
			 i = 0;
			 count ++;
		 }
		}
		break;
			 
		case 3:
		{
			if(ch == 0x0a) count++;
			else
			count = 0;
		}
		break;
			 
		case 4:
		{
			if(ch == 0x0d)
			{					 
				//更新传感器数据					 
				Update_Action_gl_position(posture.ActVal);
				//zangle = posture.ActVal[0];
				//xangle = posture.ActVal[1];
				//yangle = posture.ActVal[2];
				//pos_x = posture.ActVal[3];
				//pos_y = posture.ActVal[4];
				//w_z = posture.ActVal[5];
			}
		  count = 0;
		}
		break;
		 
		default:
		{
			count = 0;
		}
		break;		 
	}
	 RxBuffer_for4[0]=0;
     HAL_UART_Receive_IT(&huart3,RxBuffer_for4, 1);//每接收一个数据，就打开一次串口中断接收
   }
   
//   if(huart==&huart2)
//   {
//	   if( HAL_UART_Receive_IT(&huart2,&UART2_Receiver,1)!= HAL_OK)Error_Handler();
//		/* 开启接收错误中断 */
//		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
//	   
//	   Laser_ReadData(&Laser_Real_Data.Laser_X);
//	   xx++;
//	   
//	__HAL_UNLOCK(&huart2);
//	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_PE);//清标志
//	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_FE);
//	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_NE);
//	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE);
//   }
//   
//   if(huart==&huart6)
//   {
//	   if( HAL_UART_Receive_IT(&huart6,&UART6_Receiver,1)!= HAL_OK)Error_Handler();
//		/* 开启接收错误中断 */
//		__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
//	   Laser_ReadData2(&Laser_Real_Data.Laser_Y);
//	   yy++;
//       
//	   
//	 __HAL_UNLOCK(&huart6);

//	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_PE);//清标志
//	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_FE);
//	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_NE);
//	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_ORE);
//        
//	   
//   }
   if(huart==&huart4)
   {
	   
		uart4_ReceiveData(&No1_Data.WOrld_x,&No1_Data.WOrld_y,&No1_Data.WOrld_w,&No1_Data.flag);
	    HAL_UART_Receive_IT(&huart4, &UART4_Receiver,1);
//		No1_Porcessing(No1_Data);
	   Remote_Process();
//	   __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
	   __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
	   
	__HAL_UNLOCK(&huart4);
	__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_PE);//清标志
	__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_FE);
	__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_NE);
	__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE);
	
   }
}

/*
 *  函数名：HAL_UARTEx_RxEventCallback
*  功能描述：DMA空闲回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart == &huart6)
	{

		processData(USART6_Rx_buf, &Laser_Real_Data,6);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_Rx_buf, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}
	
	else if (huart == &huart2)
	{
		processData(USART2_Rx_buf, &Laser_Real_Data,2);
		
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_Rx_buf, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}
}



