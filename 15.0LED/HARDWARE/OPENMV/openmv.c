#include "openmv.h"
#include "usart.h"
#include "control.h"
#include <string.h>
#include "stdio.h"
#include "blue.h"

uint8_t  Num=0, LR =0, FindTask = 0,Finded_flag,Find_State=0,Target_Room;
uint8_t uart1_rxbuff,sendBuf[4];
uint8_t RoomNum=0,TargetNum=0;
extern int circle,rpm,pos_flat;
uint8_t g_ucUsart3ReceiveData; 
extern uint8_t task,Gray_flat,Car_stop_flat,Car_Goon_flag;
uint8_t LR_tem;

void Receive_Data_K210(uint8_t com_data)
{
		uint8_t i;
		static uint8_t RxCounter1=0;
		static uint16_t RxBuffer1[4]={0};
		static uint8_t RxState = 0;	
		static uint8_t RxFlag1 = 0;

		if(RxState==0&&com_data==0x2C)  //0x2c帧头
		{
			
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;  
		}

		else if(RxState==1&&com_data==0x12)  //0x12帧头
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}
		
		else if(RxState==2)
		{
			 
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=5||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
			{
				RxState=3;
				RxFlag1=1;				
				Num =          RxBuffer1[RxCounter1-3]; 
				LR_tem =          RxBuffer1[RxCounter1-2];    
				Finded_flag =  RxBuffer1[RxCounter1-1];
//				FindTask =      RxBuffer1[RxCounter1-2];
				
			}
		}

		else if(RxState==3)		
		{
				if(RxBuffer1[RxCounter1] == 0x5B)
				{
							
							RxFlag1 = 0;
							RxCounter1 = 0;
							RxState = 0;
						
				}
				else   //接收错误
				{
							RxState = 0;
							RxCounter1=0;
							for(i=0;i<6;i++)
							{
									RxBuffer1[i]=0x00;      //将存放数据数组清零
							}
				}
		} 

		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<6;i++)
				{
						RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}

void If_Find(void)
{		
//	uint8_t numcount=0,lastnum;
			
	if(Num==TargetNum)
	{
//		lastnum=Num;
//		numcount++;
//	}
//	else if(numcount>=2)
//	{
//		if(lastnum==TargetNum)
//		{
			LR=LR_tem;
//		}
	}
}


void Set_TargetRoom(void)
{	
	
	if(Num==1) Target_Room='A';
	if(Num==2) Target_Room='B';
	if(Num==3) Target_Room=9;
	if(Num>3) Target_Room=9;
	RoomNum = Num;
	
	
	switch(RoomNum)
		{
			case 1:
				TargetNum = 1;
			break;
			
			case 2:
				TargetNum = 2;
			break;
			
			case 3:
				TargetNum = 3;
			break;
			
			case 4:
				TargetNum = 4;
			break;
			
			case 5:
				TargetNum = 5;
			break;
			
			case 6:
				TargetNum = 6;
			break;

			case 7:
				TargetNum = 7;
			break;
			
			case 8:
				TargetNum = 8;
			break;	 		
		}
		


}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	//接收中断回调
{
  uint8_t tem;
	
	if( huart == &huart1)
	{	   
//	 HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);	
    tem=uart1_rxbuff;
    Receive_Data_K210(tem);
		
	}	
	if( huart == &huart3)//判断中断源
	{
		if(g_ucUsart3ReceiveData == 0x31) Spin_right();
		if(g_ucUsart3ReceiveData == 0x32) Spin_left() ;
		if(g_ucUsart3ReceiveData == 0x33) Car_go(219);
		if(g_ucUsart3ReceiveData == 0x34) Car_go(80);
		if(g_ucUsart3ReceiveData == 0x35) Spin_Turn(back);
		if(g_ucUsart3ReceiveData == 0x36) Spin_Turn(right);
		if(g_ucUsart3ReceiveData == 0x37) Car_stop_flat=1;
		if(g_ucUsart3ReceiveData == 0x38) Car_Goon_flag=1;
		if(g_ucUsart3ReceiveData == 0x39) Spin_Turn(left);
	}	
	HAL_UART_Receive_IT( &huart3,&g_ucUsart3ReceiveData, 1);
   HAL_UART_Receive_IT(&huart1,&uart1_rxbuff,1); 
}
