#ifndef __PID_H
#define __PID_H

//声明一个结构体类型
typedef struct 
{
	float target_val;//目标值
	float actual_val;//实际值
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
	
} tPid;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidHW_Tracking;//红外循迹的PID
extern tPid pidpos1;
extern tPid pidpos2;


void abs_limit(float *a, float ABS_MAX);


//声明函数
float P_realize(tPid * pid,float actual_val);
void PID_init(void);
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
void Pid_target(tPid * pid,float target);
float PID_pos(tPid * pid,float actual_val);
float myabs(float x);
float PID_Increase(tPid *pid, int NowPlace, int Point);
void set_p_i_d(tPid *pid, float p, float i, float d);
#endif
