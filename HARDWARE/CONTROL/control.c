#include "control.h"
#include "tim.h"
#include "motor.h"
#include "pid.h"
#include "main.h"
#include "encoder.h"
#include "protocol.h"
#include "gray_sensor.h"
#include "gpio.h"
#include "openmv.h"
#include "blue.h"

uint8_t Spin_start_flag , Spin_succeed_flag , Stop_Flag,right_flag,left_flag,Speed_Up=0;
uint8_t Line_flag = 0, Turn_flag = 0 ;
uint8_t stop_count=0 , spin_count=0;
extern uint8_t motor_flat,task,tem,Car_stop_flat,Car_Goon_flag;
int Target_Position=0;
int Target_Velocity;
float circle=0.00f;
float rpm=0.00f;
extern int encoder1,encoder2;
extern long encodersum1;
extern long encodersum2;


float Mileage1;
float Mileage2;
float Target_Mileage;
long Target_encodersum;
extern float Motor1Speed ;
extern float Motor2Speed ;

float pos_pwm1=0,pos_pwm2=0;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidpos1;
extern tPid pidpos2;
extern int Line_num;
float Line_Outval;
short timeout=0;
int way1,way2;
extern uint16_t ADC_Value[5];
extern uint8_t load_flat;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)//htim3  500HZ 2ms中断一次
	{
			
			timeout++;
		if(timeout%5==0)	
		{
			for(uint8_t i=0;i<5;i++)
				{
				 ADC_Value[i]=Get_adc();
				}
			Gray_control();	
		}
		
		if(timeout%10==0)
		{	
			GetMotorPulse();
			
			Motor1Speed = (float)encoder1*50/56/11/4;
			Motor2Speed = (float)encoder2*50/56/11/4;
			
			Mileage1+=0.02*Motor1Speed*3.1415*6.5;
			Mileage2+=0.02*Motor2Speed*3.1415*6.5;
			
			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &encodersum2, 1);
			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &encodersum1, 1);
			

		if(Line_flag==1)
		{	
			
			if((Mileage1>(Target_Mileage-24)) && (Mileage1<(Target_Mileage+24))) 	//pwm为50，速度大概是1.32转，1s大概40cm
			{
				stop_count++;
				if(stop_count>=50) //1s
				{
					Line_flag=0;
					Stop_Flag=1;
					stop_count=0;
					pos_pwm1=0;
					pos_pwm2=0;
					motor_disable();
				}
				else
				{

					Stop_Flag=0;
				}
			}
			
			if(motor_flat==1)
			{
				V_S_control();
//				if(Line_num==0)
//				{
//					encodersum_average();
//				}
          
				Line_Outval=Line_num;
				if(Car_stop_flat==1)
				{
					 abs_limit(&pos_pwm1,60);
					 abs_limit(&pos_pwm2,60);
				}
				
				if(Speed_Up==1)	
					{
						abs_limit(&pos_pwm1,90);
						abs_limit(&pos_pwm2,90);
						
					}
				else
					{	
						abs_limit(&pos_pwm1,80);
						abs_limit(&pos_pwm2,80);
					}						
				if(Mileage1<(Target_Mileage-5))
				{					
				pos_pwm1=pos_pwm1-Line_Outval;
				pos_pwm2=pos_pwm2+Line_Outval;

				}
//				if(encoder1<-50)							//倒车灰度不稳
//				{
//					encodersum_average();
//				
//				}	
				
				
				Motor_Set(pos_pwm1,pos_pwm2);
				
			}
		}
		
		if(Spin_start_flag==1)
		{
			if(motor_flat==1)
			{	
				spin_count++;
				Turn_control();
				
				if(spin_count>=70)
				{
					Spin_start_flag=0;
					Spin_succeed_flag=1;
					motor_disable();	
					spin_count=0;
				
				}
							
				abs_limit(&pos_pwm1,70);
				abs_limit(&pos_pwm2,70);
				Motor_Set(pos_pwm1,pos_pwm2);
			}
		
		}
		if(right_flag==1)
		{
			if(motor_flat==1)
			{	
				spin_count++;
				Turn_control();
				
				if(spin_count>=40)
				{
					right_flag=0;
					motor_disable();	
					spin_count=0;
				
				}
					
						abs_limit(&pos_pwm1,60);
						abs_limit(&pos_pwm2,60);
						Motor_Set(pos_pwm1,0);
					
			}
		
		}
		if(left_flag==1)
		{
			if(motor_flat==1)
			{	
				spin_count++;
				Turn_control();
				
				if(spin_count>=40)
				{
					left_flag=0;
					motor_disable();	
					spin_count=0;
				
				}
							
				abs_limit(&pos_pwm1,70);
				abs_limit(&pos_pwm2,70);
				Motor_Set(0,pos_pwm2);
			}
		
		}			
	
			timeout=0;	
		}	
	}
}



void V_S_control()
{

			Target_Position=Encoder_Num(circle,56,11);
			Target_Velocity=Encoder_V(rpm,11,56,20);  
			
			Pid_target(&pidpos1,Target_Position);
			Pid_target(&pidpos2,Target_Position);
			
			pos_pwm1=PID_realize(&pidpos1,encoder1);
			pos_pwm2=PID_realize(&pidpos2,encodersum2);			

			
//			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &Target_Velocity, 1);
//			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &encoder1, 1);
		
			Pid_target(&pidMotor1Speed,pos_pwm1);
			Pid_target(&pidMotor2Speed,pos_pwm2);
			
//         pos_pwm1 = PID_Increase(&pidMotor1Speed,encoder1,pos_pwm1);      /* 增量式速度控制 */
//			pos_pwm2 = PID_Increase(&pidMotor2Speed,encoder2,pos_pwm2);
		
}

void Turn_control(void)
{

			Target_Position=Encoder_Num(circle,56,11);
			Target_Velocity=Encoder_V(rpm,11,56,20);   
			
			Pid_target(&pidpos1,way1*Target_Position);
			Pid_target(&pidpos2,way2*Target_Position);
			
			pos_pwm1=PID_realize(&pidpos1,encodersum1);
			pos_pwm2=PID_realize(&pidpos2,encodersum2-200);			

//			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &Target_Position, 1);
//			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &encodersum1, 1);
//			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &Target_Velocity, 1);
//			set_computer_value(SEND_FACT_CMD, CURVES_CH2, &encoder1, 1);
		
			Pid_target(&pidMotor1Speed,pos_pwm1);
			Pid_target(&pidMotor2Speed,pos_pwm2);
			
         pos_pwm1 = PID_Increase(&pidMotor1Speed,encoder1,pos_pwm1);      /* 增量式速度控制 */
			pos_pwm2 = PID_Increase(&pidMotor2Speed,encoder2,pos_pwm2);
		
}

void Car_go(int location_cm)  
{	
	  Line_flag = 1;   
	  Stop_Flag = 0;

	  Spin_start_flag = 0;
	  Spin_succeed_flag = 0;  
	
	  Mileage1 = 0;
	  Mileage2 = 0;
	
	  Target_Mileage=location_cm;
	  circle = location_cm/3.1416/6.5;   
	 
	  motor_enable();     
	 
	  encodersum1 = 0;    
	  encodersum2 = 0; 
}


void Spin_Turn(spin_way Turn_way)   
{
	
	   Line_flag = 0;  //不进行巡线的补偿了
	   Stop_Flag = 0;   //执行转弯时，将直走完成的标志位清零. 即如果上一次是直行，这次是转弯，则不用在业务代码里手动置位
	  
      Spin_start_flag = 1;   
	   Spin_succeed_flag = 0;  
	
		float spin90_val;
		
		Mileage1 = 0;
		Mileage2 = 0;
		
		spin90_val = 15.5;
	
		if(Turn_way==right)
		{

			circle = spin90_val/3.1416/6.5; 
			circle = 0.99 * circle;      
			way1=1,way2=-1;
		}
		
		if(Turn_way==left)
		{
			circle = spin90_val/3.1416/6.5; 
			circle = 0.89* circle;
		
			way1=-1,way2=1;	
		}
		
		if(Turn_way==back)
		{
			circle = spin90_val/3.1416/6.5; 
			circle = 0.97 * circle;   
			circle = 2*circle; 
			way1=1,way2=-1;	
		}	
		
			encodersum1 = 0;    
			encodersum2 = 0; 
		
			motor_enable();     

}

void If_load()
{
	READ_HW;
	if(READ_HW==SET) load_flat=0;
	else load_flat=1;
	HAL_Delay(30);
}

void Spin_right()   
{
	
	   Line_flag = 0;  
	   Stop_Flag = 0;   		
		right_flag = 1;   
	   
		float spin90_val;
		
		Mileage1 = 0;
		Mileage2 = 0;
		
		spin90_val = 7;
	

		circle = spin90_val/3.1416/6.5; 			    
		way1=1,way2=1;	
		
		encodersum1 = 0;    
		encodersum2 = 0; 
	
		motor_enable();  
		
}	

void Spin_left()   
{
	
	   Line_flag = 0; 
	   Stop_Flag = 0;   
	  
		left_flag = 1;   
	   
		float spin90_val;
		
		Mileage1 = 0;
		Mileage2 = 0;
		
		spin90_val = 4.5;
	

		circle = spin90_val/3.1416/6.5; 			    
		way1=1,way2=1;	
		
		encodersum1 = 0;    
		encodersum2 = 0; 
	
		motor_enable();  
		
}	

void Set_task()
{
	if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0)
		{	
			while(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0);
			task++;
			if(task>2) task=0;		
		}	
}
