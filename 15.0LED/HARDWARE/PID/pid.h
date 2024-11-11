#ifndef __PID_H
#define __PID_H

//����һ���ṹ������
typedef struct 
{
	float target_val;//Ŀ��ֵ
	float actual_val;//ʵ��ֵ
	float err;//��ǰƫ��
	float err_last;//�ϴ�ƫ��
	float err_sum;//����ۼ�ֵ
	float Kp,Ki,Kd;//���������֣�΢��ϵ��
	
} tPid;

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern tPid pidHW_Tracking;//����ѭ����PID
extern tPid pidpos1;
extern tPid pidpos2;


void abs_limit(float *a, float ABS_MAX);


//��������
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
