#include "main.h"
#include "adc.h"
#include "gray_sensor.h"

extern uint16_t ADC_Value[5];
uint8_t Gray_flat=0,Car_stop_flat=0,gray=0;

uint16_t Get_adc(void)
{
	 HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,100);
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
	 {
       return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}



uint8_t HD_Read[5] = {0};//保存灰度电平的数组
int Line_num;


	//ADC_Value[4]为左2
void Gray_control(void)
{
	if(ADC_Value[4]<2000) HD_Read[0]=0;else HD_Read[0]=1;
	if(ADC_Value[3]<2900) HD_Read[1]=0;else HD_Read[1]=1;
	if(ADC_Value[2]<1700) HD_Read[2]=0;else HD_Read[2]=1;
	if(ADC_Value[1]<2400) HD_Read[3]=0;else HD_Read[3]=1;
	if(ADC_Value[0]<1900) HD_Read[4]=0;else HD_Read[4]=1;
	
	if(HD_Read[0]==1 && HD_Read[1]==1 && HD_Read[2]==0 && HD_Read[3]==1 && HD_Read[4]==1 ) Line_num=0;
	
	if(HD_Read[0]==1 && HD_Read[1]==0 && HD_Read[2]==0 && HD_Read[3]==1 && HD_Read[4]==1) Line_num=5; //偏you
	if(HD_Read[0]==1 && HD_Read[1]==0 && HD_Read[2]==1 && HD_Read[3]==1 && HD_Read[4]==1) Line_num=9; 
	if(HD_Read[0]==0 && HD_Read[1]==0 && HD_Read[2]==1 && HD_Read[3]==1 && HD_Read[4]==1) Line_num=13;
	if(HD_Read[0]==0 && HD_Read[1]==1 && HD_Read[2]==1 && HD_Read[3]==1 && HD_Read[4]==1) Line_num=17;
	
	if(HD_Read[0]==1 && HD_Read[1]==1 && HD_Read[2]==0 && HD_Read[3]==0 && HD_Read[4]==1) Line_num=-6; //偏zuo
	if(HD_Read[0]==1 && HD_Read[1]==1 && HD_Read[2]==0 && HD_Read[3]==1 && HD_Read[4]==1) Line_num=-9;
	if(HD_Read[0]==1 && HD_Read[1]==1 && HD_Read[2]==1 && HD_Read[3]==0 && HD_Read[4]==0) Line_num=-13;
	if(HD_Read[0]==1 && HD_Read[1]==1 && HD_Read[2]==1 && HD_Read[3]==1 && HD_Read[4]==0) Line_num=-17;
	
	if(HD_Read[0]==0 && HD_Read[1]==0 && HD_Read[2]==0 && HD_Read[3]==0 && HD_Read[4]==0) Line_num=0;
	
	if(Car_stop_flat==1)
	{
		if(HD_Read[0]==0 && HD_Read[4]==0) Gray_flat=1;
	}
//	if(Car_stop_flat==0)
//		{
//			Gray_flat=gray;
//		}
}
