/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"

#include "pid.h"
#include "control.h"
#include "encoder.h"
#include "protocol.h"
#include <string.h>
#include "gray_sensor.h"
#include "openmv.h"
#include "blue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern float Motor1Speed ;
extern float Motor2Speed ;
extern long Target_encodersum;
extern int encoder1;
extern long encodersum1;
extern long encodersum2;

extern float Mileage1;//里程数
extern float Mileage2;
extern float Target_Mileage;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidpos1;
extern tPid pidpos2;

extern uint8_t LR,LR_tem,Num,Find_State,TargetNum,right_flag,left_flag,Finded_flag;

//extern uint8_t Usart1_ReadBuf[255];	//串口1 缓冲数组

uint8_t OledString[50];

extern uint8_t g_ucUsart3ReceiveData;
extern uint8_t uart1_rxbuff;

extern float pos_pwm;
extern uint8_t HD_Read[5];

int load_flat;						//药品装载标志
extern uint8_t Line_flag, Turn_flag,Stop_Flag,motor_flat,Spin_start_flag,Spin_succeed_flag;
extern float circle;
extern uint8_t stop_count , spin_count;
extern uint8_t Num;
uint8_t Do_count;
uint16_t ADC_Value[5];
extern float pos_pwm1;
extern int Line_num;
extern uint16_t TimerCount;
uint8_t task=1;
extern uint8_t Target_Room;
extern uint16_t FindTimeCount,loadCount;
int caseflag=0;
extern uint8_t Gray_flat,Car_stop_flat,Car_Goon_flag,Speed_Up;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();			//初始化OLED  
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1 通道1 PWM输出
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1 通道4 PWM输出
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//定时器3 舵机
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4
  HAL_TIM_Base_Start_IT(&htim2);				//开启定时器2 中断
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);                //开启定时器4 中断 
  HAL_TIM_Base_Start_IT(&htim1);                //开启定时器1 中断
  
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//开启串口1接收中断
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);	//开启串口2接收中断
  
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);
//  HAL_UART_Receive_IT(&huart2,&uart2_rxbuff,1); 
  HAL_UART_Receive_IT(&huart1,&uart1_rxbuff,1);   
//  PID_init();  
  protocol_init();
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0); 
  LED_OFF;
  HAL_ADCEx_Calibration_Start(&hadc1);
  Motor_Set(60,60);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(TimerCount>=100)
	{
		
//		Send_Data_K210();	
		Set_task();
		TimerCount=0;
		
	}
	
	if(loadCount>=500)
	{
		If_load();
		loadCount=0;
		
	}

	if(task==1)
	{
		Set_TargetRoom();
	}
	
//	sprintf((char*)OledString, "V1:%.2f V2:%.2f", Motor1Speed,Motor2Speed);//
//	OLED_ShowString(0,0,OledString,12);
	
	sprintf((char*)OledString, "grayflag:%d",Gray_flat);
	OLED_ShowString(0,3,OledString,12);
	
	sprintf((char*)OledString, "encoder:%d",encoder1);
	OLED_ShowString(0,4,OledString,12);
	
	sprintf((char*)OledString, "Task:%d", task);
	OLED_ShowString(0,0,OledString,12);
	
	sprintf((char*)OledString, "caseflag:%d",caseflag);
	OLED_ShowString(0,2,OledString,12);

//	sprintf((char*)OledString, "M1:%.2f",Mileage1);
//	OLED_ShowString(0,5,OledString,12);

	sprintf((char*)OledString, "%d %d %d %d %d",HD_Read[0], HD_Read[1] ,HD_Read[2],HD_Read[3],HD_Read[4]);
	OLED_ShowString(0,1,OledString,12);
	
//	sprintf((char*)OledString, "M2:%.2f",Mileage2 );
//	OLED_ShowString(0,6,OledString,12);
	
	sprintf((char*)OledString, "num:%d TNum:%d", Num,TargetNum );
	OLED_ShowString(0,7,OledString,12);
	
	sprintf((char*)OledString, "%d %d %d",ADC_Value[2],ADC_Value[1],ADC_Value[0]);
	OLED_ShowString(0,6,OledString,12);

	sprintf((char*)OledString, "%d %d",ADC_Value[4],ADC_Value[3]);
	OLED_ShowString(0,5,OledString,12);


if(task==2)
{//近端病房
	if(Target_Room=='B')
	{
		switch(Do_count)
		{
			case 0:
					Do_count++;
					Car_go(78);
					break;
			case 1:
					if(Stop_Flag==1)
					{
						Spin_Turn(right);
						Do_count++;		
					}break;
			case 2:
					if(Spin_succeed_flag==1)
					{
						Car_go(42);
						Do_count++;
					}break;
			case 3:
					if(Stop_Flag==1)
					{
						LED_ON;
						Do_count++;
					}break;
			case 4:						  
	
					if(load_flat==0)
						{
						Spin_Turn(back);
						Do_count++;
						break;
						}
			case 5:			
					if(Spin_succeed_flag==1)
						{
							Car_go(44);
							Do_count++;
						}break;			
			case 6:
						if(Stop_Flag==1)
						{
							Spin_Turn(left);
							Do_count++;
						}break;
			case 7:
						if(Spin_succeed_flag==1)
						{
							Car_go(62);
						}break;	
					
			}
		}


	else if(Target_Room=='A')		
	{
		switch(Do_count)
		{
			case 0:
					Do_count++;
					Car_go(78);
					break;
			case 1:
					if(Stop_Flag==1)
					{
						Spin_Turn(left);
						Do_count++;		
					}break;
			case 2:
					if(Spin_succeed_flag==1)
					{
						Car_go(42);
						Do_count++;
					}break;
			case 3:
					if(Stop_Flag==1)
					{
						LED_ON;
						Do_count++;
					}break;
			case 4:
			
					if(load_flat==0)
						{
						Spin_Turn(back);
						Do_count++;
						}break;
			case 5:			
					if(Spin_succeed_flag==1)
						{
							Car_go(45);
							Do_count++;
						}break;			
			case 6:
						if(Stop_Flag==1)
						{
							Spin_Turn(right);
							Do_count++;
						}break;
			case 7:
						if(Spin_succeed_flag==1)
						{
							Car_go(60);
						}break;		
					
			
		}
	
	}
				
	
	//中远端病房
	else if(Target_Room==9)		
		{
			switch(Do_count)
		{					
			case 0:	
					Car_go(128);
					Do_count++;
					break;
			case 1:
					if(Stop_Flag==1)
					{		
							Find_State=1;		
							if(FindTimeCount<=WaitTime)
							{	
							
								If_Find();	
								if(LR==1)		//中左
								{
									Car_go(37);
									Find_State=0;					
									caseflag=-1;
									FindTimeCount = 0;
									Do_count=11;
								}
								else if(LR==2)		//中右
								{
									Car_go(37);
									Find_State=0;								
									caseflag=1;
									FindTimeCount = 0;
									Do_count=11;		
								}								//这里执行完小车在中端十字路口
							}	
							else if(FindTimeCount>WaitTime)
								{
										Car_go(87);			//这里去远端
										Find_State=0;
										FindTimeCount = 0;	
										Do_count++;		
								}	
								}break;
			
			case 2:								
						if(Stop_Flag == 1)     
								{		
										Find_State=1;						
										if(FindTimeCount<=WaitTime)
									{		
											If_Find();							
										if(LR == 1) 
										{
											 Car_go(37);   
											 Find_State=0;
											 FindTimeCount = 0;		
											 caseflag=-1;
											 Do_count=5;
										}
										else if(LR == 2)  	
										{
											 Car_go(37);	
											 Find_State=0;
											 FindTimeCount = 0;
											 caseflag=1;
											 Do_count=5;
										}
										}
										if(FindTimeCount>WaitTime)
									{	 
										{		Spin_right();
												Find_State=0;
												FindTimeCount = 0;
												Do_count++;
										}
									}	
								}break;	
			case 3:		
							if(right_flag==0)
							{
									Find_State=1;
								if(FindTimeCount<=WaitTime)
								{	
									If_Find();
									if(LR==1||LR == 2)
									{	
										Spin_left();
										caseflag=1;
										Find_State=0;
										FindTimeCount = 0;
										Do_count++;
									}
								}
								else if(FindTimeCount>WaitTime)
								{
									Spin_left();
									caseflag=-1;
									Find_State=0;
									FindTimeCount = 0;
									Do_count++;
								}
								}break;
							
	
			case 4:
						if(left_flag==0)
						{						
							Car_go(30);
							Do_count=5;
						}	
						break;						//远端十字口	
			
			case 5:
						if(Stop_Flag==1)
						{
							if(caseflag==1)
							{
								Spin_Turn(right);								
								Do_count++;		
							}
							else if(caseflag==-1)
							{
								Spin_Turn(left);
								Do_count++;
							}break;
						}
				case 6:
						if(Spin_succeed_flag==1)
						{	
							if(caseflag==-1)
							{
							Car_go(50);
							}	
							else
							{
								Car_go(47);
							}
							LR=0;	
							Do_count++;
						
						}break;
				case 7:
						if(Stop_Flag==1)
						{	
								Find_State=1;
								If_Find();
							if(FindTimeCount<=WaitTime)
							{		
								if(LR==1||LR==2)
								{	
									if(caseflag==1)
									{
										Car_go(38);}
										
								  else
								  {
										Car_go(37);
								  }
								 Find_State=0;
								 FindTimeCount=0;
								 Do_count++;
								 }
							}
							
							else if(FindTimeCount>WaitTime)
							{	 				
								if(caseflag==-1)
								{										
									if(LR==0 && LR_tem==1) LR=2;
									else if(LR==0 && LR_tem==2) LR=1;
										Car_go(36);
									
								}
								else if(caseflag==1)
								{
									if(LR==0 && LR_tem==1) LR=2;
									else if(LR==0 && LR_tem==2) LR=1;
									Car_go(39);
								}		
								 Find_State=0;
								 FindTimeCount=0;
								 Do_count++;
							}
							
						}break;
				case 8:
						if( LR == 1 &&caseflag==-1) 
										{
											    											
											 caseflag=-2;
											 Do_count++;
										}	 
						else if(LR == 2 &&caseflag==-1) 
										{
											     
											 caseflag=-3;
											 Do_count++;
										}
						else if(LR == 2 &&caseflag==1) 
										{
											    
											 caseflag=2;
											 Do_count++;
										}			
					   else if(LR == 1 &&caseflag==1 )  	
										{
											
											 caseflag=3;
											 Do_count++;		
										}break;
				case 9:
						if(Stop_Flag==1)
						{	
							if(caseflag==2||caseflag==-3)
							{
								Spin_Turn(right);
								Do_count++;						
							}
							if(caseflag==3||caseflag==-2)
							{
								Spin_Turn(left);
								Do_count++;							
							}	
							
						}break;
				
				case 10:
						if(Spin_succeed_flag==1)
						{	
							if(caseflag==2)
							{
								Car_go(41);
							}
							else if(caseflag==-3)
							{
								Car_go(42);							
							}
							else 
							{
							Car_go(43);
							}
							Do_count=18;
							break; 
						}       //到达远端病房
				

				//这里进中端病房并返回	
				case 11:
						if(Stop_Flag==1)
						{
							if(caseflag==-1)
							{
								Spin_Turn(left);
								Do_count++;
							}
							if(caseflag==1)
							{
								Spin_Turn(right);
								Do_count++;
							}		
						}break;
				case 12:
						if(Spin_succeed_flag==1)
							{	
								if(caseflag==1)
							{
								Car_go(41);
								Do_count++;
								}								
							
							else if(caseflag==-1)
							{
								Car_go(42);
								Do_count++;								
							}		
						}break;
				case 13:
						if(Stop_Flag==1)
						{
						if(load_flat==0)
						{
							Spin_Turn(back);
							Do_count++;
						}
						}break;
				case 14:
						if(Spin_succeed_flag==1)
						{
							if(caseflag==1)
							{
								Car_go(43);
								Do_count++;
								}								
							
							else if(caseflag==-1)
							{
								Car_go(45);
								Do_count++;								
							}													
						}break;
				case 15:
						if(Stop_Flag==1)
						{
							if(caseflag==-1)		//这里要相反
								{
									Spin_Turn(right);
									Do_count++;			}
							if(caseflag==1)
								{
									Spin_Turn(left);
									Do_count++;		
									
								}	
						}break;
				case 16:
						if(Spin_succeed_flag==1)
						{
							if(caseflag==-1)		//这里要相反
								{
									Car_go(148);
									Do_count++;	
									}
							if(caseflag==1)
								{
									Car_go(151);
									Do_count++;		
						}
						}break;
				case 17:
						if(Stop_Flag==1)
						{
							LED_OFF;
						
						}break;
					
					
				//远端返回	
					
				case 18:
					if(Stop_Flag==1)
					{
						if(load_flat==0)
						{
							Spin_Turn(back);
							Do_count++;
						}
						}break;
				case 19:
					if(Spin_succeed_flag==1)
						{	
							if(caseflag==-3)
							{
								Car_go(45);
								Do_count++;
							}
							else if(caseflag==-2)
							{
								Car_go(45);
								Do_count++;
							}
							else
							{
								Car_go(45);
								Do_count++;
							}
						}break;
				case 20:
					if(Stop_Flag==1)
					{
						if(caseflag==-2||caseflag==3)
						{
							Spin_Turn(right);
							Do_count++;
						}
						else if(caseflag==-3||caseflag==2)
						{
							Spin_Turn(left);
							Do_count++;
						}	
					}break;
				case 21:
					if(Spin_succeed_flag==1)
					{	if(caseflag==2)
						{
							Car_go(89);
							Do_count++;
						}
						else if(caseflag==3)
						{
							Car_go(85);
							Do_count++;	
						}
						else if(caseflag==-2)
						{
							Car_go(83);
							Do_count++;
						}
						else
						{
							Car_go(88);
							Do_count++;						
						}
						
					}break;
				case 22:
					if(Stop_Flag==1)
					{
							if(caseflag==-2||caseflag==-3)
							{
								Spin_Turn(right);
								Do_count++;
							}
							else if(caseflag==2||caseflag==3)
							{
								Spin_Turn(left);
								Do_count++;
							}
						
					}break;
						
				case 23:
					if(Spin_succeed_flag==1)
					{	
						Speed_Up=1;
						if(caseflag==-2)
						{
							Car_go(236);
						}
						else
						{
							Car_go(237);
						}
						
					}break;
				
						
}
}										
			


	}
	}				
}						
								
				
 
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
