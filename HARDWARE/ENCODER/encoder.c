#include "encoder.h"
#include "tim.h"
#include "pid.h"
long encoder1=0;
long encoder2=0;
int encodersum1=0,encodersum2=0;//ȫ�ֱ����� ������������ֵ

void GetMotorPulse(void)//��ȡ�������
{
			encoder1 = -(short)__HAL_TIM_GET_COUNTER(&htim4);
			encoder2 = (short)__HAL_TIM_GET_COUNTER(&htim2);
							
			__HAL_TIM_SET_COUNTER(&htim4,0);
			__HAL_TIM_SET_COUNTER(&htim2,0);
	
			encodersum1 += encoder1;
			encodersum2 += encoder2;

}

//����ʵ��ת��
float Moto_Speed(int encoder_cnt,short ppr,short ratio,short cnt_time)
{
    encoder_cnt = myabs(encoder_cnt);  
    return (encoder_cnt/4/ppr/ratio)*(1000/cnt_time)*60;   
}


//����ת����Ӧ������������
float Encoder_Num(float num,int ppr,float ratio)
{
    return (num*ratio*ppr*4);                                  
}

//����ת�ٶ�Ӧ������������
float Encoder_V(float rpm,short ppr,short ratio,short cnt_time)
{
    return (rpm*ratio*ppr*4)/(1000/cnt_time);                 
}

void encodersum_average(void)
{
	long enco_sum;
	enco_sum=(encodersum1+encodersum2)/2;
	encodersum1=enco_sum;
	encodersum2=enco_sum;

}




