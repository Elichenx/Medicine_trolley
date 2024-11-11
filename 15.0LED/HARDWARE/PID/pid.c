#include "pid.h"

tPid pidMotor1Speed;
tPid pidMotor2Speed;

tPid pidpos1;
tPid pidpos2;


//���ṹ�����ͱ�������ֵ
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=2.00;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=1.9;//0.2;		   //λ��ʽ1.9;
	pidMotor1Speed.Ki=0.5;//1.9;			//0.5;
	pidMotor1Speed.Kd=0;
	
	pidMotor2Speed.actual_val=0.0;
	pidMotor2Speed.target_val=0.00;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=0.2;
	pidMotor2Speed.Ki=1.9;
	pidMotor2Speed.Kd=0;

	pidpos1.actual_val=0.0;
	pidpos1.target_val=0.0;
	pidpos1.err=0.0;
	pidpos1.err_last=0.0;
	pidpos1.err_sum=0.0;
	pidpos1.Kp=0.45;
	pidpos1.Ki=0.0;
	pidpos1.Kd=0.00;
	
	pidpos2.actual_val=0.0;
	pidpos2.target_val=0.0;
	pidpos2.err=0.0;
	pidpos2.err_last=0.0;
	pidpos2.err_sum=0.0;
	pidpos2.Kp=0.45;
	pidpos2.Ki=0;
	pidpos2.Kd=0;
	
}
//����p���ڿ��ƺ���
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	//�������Ƶ���   ���=Kp*��ǰ���
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//����P ����I ���ƺ���
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	//ʹ��PI���� ���=Kp*��ǰ���+Ki*����ۼ�ֵ
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
//	if(pid->actual_val>99) pid->actual_val=99;
//	if(pid->actual_val<-99) pid->actual_val=-99;
	return pid->actual_val;
}
// PID���ƺ���
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;////��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	//ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//�����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;		
	return pid->actual_val;
}



void Pid_target(tPid * pid,float target)
{
	 pid->target_val=target;
}

void set_p_i_d(tPid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}



float PID_Increase(tPid *pid, int NowPlace, int Point)
{
 
	float iError,	//��ǰ���
		Increase;	//���ó���ʵ������
 
	iError = Point - NowPlace;	// ���㵱ǰ���
	
	if(iError>1000) iError=1000;
	if(iError<-1000) iError=-1000;
	
	Increase =  pid->Kp * (iError - pid->err)   //����P
			  + pid->Ki * iError      //����I
			  + pid->Kd * (iError - 2 * pid->err + pid->err_last);  //΢��D
	
	pid->err_last = pid->err;	// ����ǰ�����
	pid->err = iError;		  	// �����ϴ����
	
	return Increase;	// ��������
}



void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

float myabs(float x)
{	
	float temp;
	if(x<0)
	{
		temp=-x;
		x=temp;
	}
	return x;

}
   