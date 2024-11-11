#include "blue.h"
#include "usart.h"
#include "control.h"
#include "gray_sensor.h"
#include "pid.h"

extern uint8_t  Num;
extern uint8_t Spin_start_flag , Spin_succeed_flag , Stop_Flag,right_flag,left_flag,stop_fLag;
extern uint8_t Gray_flat,Car_stop_flat,gray;
uint8_t Car_elude_Conut=0,Car_Goon_Conut=0,Car_Goon_flag;
extern uint16_t eludetime;

void Car_elude(void)
{
	

		

	switch(Car_elude_Conut)
	{
		case 0:			
			Spin_Turn(back);
			Car_elude_Conut++;
			break;
			
		case 1:
			if(Spin_succeed_flag==1)
				{
					Car_go(196);
					Car_elude_Conut++;
				}break;			

		case 2:
				if(eludetime==120)
				{	

					Spin_Turn(right);				
					Car_elude_Conut++;
				}break;
	
		case 3:		
			if(Spin_succeed_flag==1)
			{	
				Car_go(30);
				Car_stop_flat=0;
			}break;
	}
}

void Car_Go_on(void)
{				
		switch(Car_Goon_Conut)
		{
			case 0:
						Spin_Turn(back);
						Car_Goon_Conut++;
						break;
			case 1:
				if(Spin_succeed_flag==1)
				{
					Car_go(30);
					Car_Goon_Conut++;
				}break;
			case 2:
				if(Stop_Flag==1)
					{
						Spin_Turn(left);
						Car_Goon_Conut++;
					}break;	
			case 3:
				if(Spin_succeed_flag==1)
				{
					Car_go(152);
				}break;
		}
	

}
