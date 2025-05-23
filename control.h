#ifndef  _CONTROL_H
#define  _CONTROL_H

#include "sys.h" 




int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z,int RC);
int PI_Left(int encoder_left,int Target);
int PI_Right(int encoder_left,int Target);

void EXTI9_5_IRQHandler(void);

float Stra_control1(float current,float target);
float Stra_control2(float current,float target);
float Stra_control3(float current,float target);
float Stra_control4(float current,float target);
float Trac_Turn1(int Target,int error);
float Trac_Turn2(int Target,int error);
void Balance_Control(int target_speed);
void Tra_control(uint8_t mode);
void Bluetooth_Control(void );
void control(void);

#endif

