#include "control.h"
#include "usart3.h"
#include "track.h"
float Med_Angle=0;		//机械中值。---在这里修改你的机械中值即可。
float Target_Speed=0;	//期望速度（俯仰）。---二次开发接口，用于控制小车前进后退及其速度。
float Turn_Speed=0;		//期望速度（偏航）
extern uint8_t Fore,Back,Left,Right;
extern  uint8_t number;
float 
	Vertical_Kp=-400,//直立环KP、KD
	Vertical_Kd=-1.95;//
float 
	Velocity_Kp=-0.44,//速度环
	Velocity_Ki=-0.0022;

float 
	Turn_Kd=0,//转向环KP、KD
	Turn_Kp=0;
float //一圈AB段
	Diff_Kp1=50,
	Diff_Ki1=3,
	Diff_Kd1;
float //一圈BC出弯后
	Diff_Kp2=35,
	Diff_Ki2=-3,
	Diff_Kd2;
float //八字AC段
	Diff_Kp3=35,
	Diff_Ki3,
	Diff_Kd3=2;
float //八字BD段
	Diff_Kp4=25,
	Diff_Ki4,
	Diff_Kd4;
float 
	Trac_Kp1=-550,
	Trac_Ki1,
	Trac_Kd1;																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																									
float 
	Trac_Kp2=650,
	Trac_Ki2,
	Trac_Kd2;
volatile  uint8_t mode;
int bias;
extern uint8_t flag_enable;

#define SPEED_Y 20   //俯仰(前后)最大设定速度
#define SPEED_Z 50//偏航(左右)最大设定速度 
float Pitch,Roll,Yaw;

int Vertical_out,Velocity_out,Turn_out;//直立环&速度环&转向环 的输出变量

int Vertical(float Med,float Angle,float gyro_Y);//函数声明
int Velocity(int Target,int encoder_left,int encoder_right);
void Tra_control(uint8_t mode);
int Turn(int gyro_Z,int RC);

float Stra_control1(float current,float target);
float Stra_control2(float current,float target);
float Stra_control3(float current,float target);
float Stra_control4(float current,float target);

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=0)//一级判定
	{
		if(PBin(5)==0)//二级判定
		{
			
			EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位
			//1、采集编码器数据&MPU6050角度信息。
			Encoder_Left=-Read_Speed(2);//电机是相对安装，刚好相差180度，为了编码器输出极性一致，就需要对其中一个取反。
			Encoder_Right=Read_Speed(4);			
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);			//角度
			MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	//陀螺仪
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//加速度
			
//			Bluetooth_Control();
			
//			Balance_Control(0);
				if(flag_enable==0||mode==0||flag==0)
			{		
					Target_Speed+=3;
					Target_Speed=Target_Speed>0?0:Target_Speed;
					Balance_Control(Target_Speed);
			}
			else if(flag_enable==1)
			{
				Tra_control(3);
			}
		}
	}
}

/*********************
直立环环PD控制
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	 int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}



/*********************
转向环
*********************/
int Turn(int gyro_Z,int RC)
{
	int PWM_out;
	PWM_out=Turn_Kd*gyro_Z + Turn_Kp*RC;
	return PWM_out;
}
/*******
直线差速环
********/
float Stra_control1(float current,float target)//ABCD AB
{
		static float Err_last,Err,output,Err_S;
		Err=current-target;
		Err_S+=Err;
		output=Diff_Kp1*Err+Diff_Ki1*Err_S+Diff_Kd1*(Err-Err_last);
		Err_last=Err;
		Err_S=Err_S>150?150:(Err_S<(-150)?(-150):Err_S );
		return output;
}

float Stra_control2(float current,float target)//ABCD CD
{
		static float Err_last,Err,output,Err_S;
		Err=current-target;
		Err_S+=Err;
		output=Diff_Kp2*Err+Diff_Ki2*Err_S+Diff_Kd3*(Err-Err_last);
		Err_last=Err;
		Err_S=Err_S>150?150:(Err_S<(-150)?(-150):Err_S );
		return output;
}
float Stra_control3(float current,float target)//ACBD AC
{
		static float Err_last,Err,output,Err_S;
		Err=current-target;
		Err_S+=Err;
		output=Diff_Kp3*Err+Diff_Ki3*Err_S+Diff_Kd3*(Err-Err_last);
		Err_last=Err;
		Err_S=Err_S>150?150:(Err_S<(-150)?(-150):Err_S );
		return output;
}
float Stra_control4(float current,float target)//ACBD BD
{
		static float Err_last,Err,output,Err_S;
		Err=current-target;
		Err_S+=Err;
		output=Diff_Kp4*Err+Diff_Ki4*Err_S+Diff_Kd4*(Err-Err_last);
		Err_last=Err;
		Err_S=Err_S>150?150:(Err_S<(-150)?(-150):Err_S );
		return output;
}
/******
循迹转向环
******/
float Trac_Turn1(int Target,int error)
{
	static int  Error0;
	static float Error,Error_last,output,Error_S;//滤波后的差值
	float a=0.70;
	Error0=get_TraErr()-Target;
	Error=(1-a)*Error0+a*Error_last;
	Error_S+=Error;
	Error_last=Error;
	output=Trac_Kp1*Error+Trac_Ki1*Error_S+Trac_Kd1*(Error-Error_last);
	return output;
}
float Trac_Turn2(int Target,int error)
{
	static int  Error0;
	static float Error,Error_last,output,Error_S;//滤波后的差值
	float a=0.70;
	Error0=get_TraErr()-Target;
	Error=(1-a)*Error0+a*Error_last;
	Error_S+=Error;
	Error_last=Error;
	output=Trac_Kp2*Error+Trac_Ki2*Error_S+Trac_Kd2*(Error-Error_last);
	return output;
}
/**********
速度环
**********/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//计算速度偏差
	Encoder_Err=(encoder_left+encoder_right-Target);
	//对速度偏差进行低通滤波
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//积分限幅
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	
	if(stop==1)Encoder_S=0,stop=0;//清零积分量
	
	//速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}


/*********************************
蓝牙遥控
**********************************/

void Bluetooth_Control(void )
{

			Target_Speed=0;
			if((Fore==0)&&(Back==0))Target_Speed=0;
			if(Fore==1)
			{
				Target_Speed--;
			}
			
			if(Back==1){Target_Speed++;}
			Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
			
			/*左右*/
			if((Left==0)&&(Right==0))Turn_Speed=0;
			if(Left==1)Turn_Speed+=30;	//左转
			if(Right==1)Turn_Speed-=30;	//右转
			Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//限幅
			
			/*转向约束*/
			if((Left==0)&&(Right==0))Turn_Kd=0.6;//Kd可以起到阻尼作用
			else if((Left==1)||(Right==1))Turn_Kd=0;//没有阻尼车子转动
			/*********************************************************************************************/
			Velocity_out=Velocity(Encoder_Left,Encoder_Right,Target_Speed);		
			
			Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);			
			
			Turn_out=Turn(gyroz,Turn_Speed);																
			
			MOTO1=Vertical_out-Turn_out;
			MOTO2=Vertical_out+Turn_out;
			Limit(&MOTO1,&MOTO2);		
			Load(MOTO1,MOTO2);
			
			Stop(&Med_Angle,&Pitch);//安全检测
}
/*****************
平衡功能
******************/
void Balance_Control(int target_speed)
{
	Target_Speed=target_speed;
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);			//速度环
	Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);			//直立环
	Turn_Kd=0.6;
	Turn_out=Turn(gyroz,Turn_Speed);																//转向环
	
	MOTO1=Vertical_out-Turn_out;
	MOTO2=Vertical_out+Turn_out;
	Limit (&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}
/*****************
题目控制
*****************/
void Tra_control(uint8_t mode)
{	
	Target_Speed=-25;//设置速度基值
	switch(mode)
{
		case 1:
	{
		bias=Stra_control1(Yaw,0);	
		break;
	}
		case 2:
	{
		bias=Trac_Turn1(0,get_TraErr());
		break;
	}
		case 3:
	{
		if(flag==1)  bias=Stra_control3(Yaw,0);
		else if(flag==2)  bias=Trac_Turn1(0,get_TraErr());
		else if(flag==3)
		{
			/***绕半圈后换算角度***/
			float newYaw;
			if(Yaw>0)
			{
				newYaw=Yaw-180;
			}else newYaw=Yaw+180;
			bias=Stra_control2(newYaw,1);
		}
		else if(flag==4) bias=Trac_Turn1(0,get_TraErr());
		break;
	}
		case 4://flag=1,2,3,4分别表示在AC,CB,BD,DA上
	{
		if(flag==1) bias=Stra_control1(Yaw,-36.5);		//arctan(80/100)约等于38.6598度
		if(flag==2) bias=Trac_Turn2(0,get_TraErr());
		if(flag==3)
		{
			/***绕半圈后换算角度***/
			float newYaw;
			if(Yaw>0)
			{
				newYaw=Yaw-180;
			}else newYaw=Yaw+180;
			bias=Stra_control4(newYaw,51);
		}
		if(flag==4) 
			bias=Trac_Turn2(0,get_TraErr());
		break;
	}
		case 5://flag=1,2,3,4分别表示在AC,CB,BD,DA上
		{
		if(flag==1)
		{
			if(number==1)      bias=Stra_control1(Yaw,-37);		//arctan(80/100)约等于38.6598度
			else if(number==2) bias=Stra_control1(Yaw,-39);
			else if(number==3) bias=Stra_control1(Yaw,-39);
			else if(number==4) bias=Stra_control1(Yaw,-39);
		}
		if(flag==2) bias=Trac_Turn2(0,get_TraErr());
		if(flag==3)
		{
			/***绕半圈后换算角度***/
			float newYaw;
			if(Yaw>0)
			{
				newYaw=Yaw-180;
			}else newYaw=Yaw+180;
			if(number==1)      bias=Stra_control1(newYaw,50);		
			else if(number==2) bias=Stra_control1(newYaw,45);
			else if(number==3) bias=Stra_control1(newYaw,45);
			else if(number==4) bias=Stra_control1(newYaw,45);
		}
		if(flag==4) 
			bias=Trac_Turn1(0,get_TraErr());
		break;
		}
}
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);//速度环输出
	Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);//平衡环输出
	MOTO1=Vertical_out-bias ;
	MOTO2=Vertical_out+bias ;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}

