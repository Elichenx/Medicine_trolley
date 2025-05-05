#ifndef __ENCODER_H
#define __ENCODER_H

void GetMotorPulse(void);
float Moto_Speed(int encoder_cnt,short ppr,short ratio,short cnt_time);
float Encoder_Num(float num,int ppr,float ratio);
float Encoder_V(float rpm,short ppr,short ratio,short cnt_time);
void encodersum_average(void);


#endif
