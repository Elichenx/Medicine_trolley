#include "pid.h"

tPid pidMotor1Speed;
tPid pidMotor2Speed;

tPid pidpos1;
tPid pidpos2;


//给结构体类型变量赋初值
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=2.00;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=1.9;//0.2;		   //位置式1.9;
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
//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节   输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
//	if(pid->actual_val>99) pid->actual_val=99;
//	if(pid->actual_val<-99) pid->actual_val=-99;
	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;		
	return pid->actual_val;
}



void Pid_target(tPid * pid,float target)
{
	 pid->target_val=target;
}

void set_p_i_d(tPid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}



float PID_Increase(tPid *pid, int NowPlace, int Point)
{
 
	float iError,	//当前误差
		Increase;	//最后得出的实际增量
 
	iError = Point - NowPlace;	// 计算当前误差
	
	if(iError>1000) iError=1000;
	if(iError<-1000) iError=-1000;
	
	Increase =  pid->Kp * (iError - pid->err)   //比例P
			  + pid->Ki * iError      //积分I
			  + pid->Kd * (iError - 2 * pid->err + pid->err_last);  //微分D
	
	pid->err_last = pid->err;	// 更新前次误差
	pid->err = iError;		  	// 更新上次误差
	
	return Increase;	// 返回增量
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
   