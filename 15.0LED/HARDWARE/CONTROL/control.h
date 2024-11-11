#ifndef __CONTROL_H
#define __CONTROL_H

#define WaitTime  500

typedef enum
{
   left,
	right,
	back
}spin_way;

void V_S_control(void);
void Car_go(int location_cm);
void Spin_Turn(spin_way Turn_way);
void Turn_control(void);
void If_load(void);
void Spin_right(void);
void Spin_left(void);    
void Set_task(void);

#endif
