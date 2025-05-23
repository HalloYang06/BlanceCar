#include "control.h"
#include "usart3.h"
#include "track.h"
float Med_Angle=0;		//��е��ֵ��---�������޸���Ļ�е��ֵ���ɡ�
float Target_Speed=0;	//�����ٶȣ���������---���ο����ӿڣ����ڿ���С��ǰ�����˼����ٶȡ�
float Turn_Speed=0;		//�����ٶȣ�ƫ����
extern uint8_t Fore,Back,Left,Right;
extern  uint8_t number;
float 
	Vertical_Kp=-400,//ֱ����KP��KD
	Vertical_Kd=-1.95;//
float 
	Velocity_Kp=-0.44,//�ٶȻ�
	Velocity_Ki=-0.0022;

float 
	Turn_Kd=0,//ת��KP��KD
	Turn_Kp=0;
float //һȦAB��
	Diff_Kp1=50,
	Diff_Ki1=3,
	Diff_Kd1;
float //һȦBC�����
	Diff_Kp2=35,
	Diff_Ki2=-3,
	Diff_Kd2;
float //����AC��
	Diff_Kp3=35,
	Diff_Ki3,
	Diff_Kd3=2;
float //����BD��
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

#define SPEED_Y 20   //����(ǰ��)����趨�ٶ�
#define SPEED_Z 50//ƫ��(����)����趨�ٶ� 
float Pitch,Roll,Yaw;

int Vertical_out,Velocity_out,Turn_out;//ֱ����&�ٶȻ�&ת�� ���������

int Vertical(float Med,float Angle,float gyro_Y);//��������
int Velocity(int Target,int encoder_left,int encoder_right);
void Tra_control(uint8_t mode);
int Turn(int gyro_Z,int RC);

float Stra_control1(float current,float target);
float Stra_control2(float current,float target);
float Stra_control3(float current,float target);
float Stra_control4(float current,float target);

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=0)//һ���ж�
	{
		if(PBin(5)==0)//�����ж�
		{
			
			EXTI_ClearITPendingBit(EXTI_Line5);//����жϱ�־λ
			//1���ɼ�����������&MPU6050�Ƕ���Ϣ��
			Encoder_Left=-Read_Speed(2);//�������԰�װ���պ����180�ȣ�Ϊ�˱������������һ�£�����Ҫ������һ��ȡ����
			Encoder_Right=Read_Speed(4);			
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);			//�Ƕ�
			MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);	//������
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//���ٶ�
			
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
ֱ������PD����
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	 int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}



/*********************
ת��
*********************/
int Turn(int gyro_Z,int RC)
{
	int PWM_out;
	PWM_out=Turn_Kd*gyro_Z + Turn_Kp*RC;
	return PWM_out;
}
/*******
ֱ�߲��ٻ�
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
ѭ��ת��
******/
float Trac_Turn1(int Target,int error)
{
	static int  Error0;
	static float Error,Error_last,output,Error_S;//�˲���Ĳ�ֵ
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
	static float Error,Error_last,output,Error_S;//�˲���Ĳ�ֵ
	float a=0.70;
	Error0=get_TraErr()-Target;
	Error=(1-a)*Error0+a*Error_last;
	Error_S+=Error;
	Error_last=Error;
	output=Trac_Kp2*Error+Trac_Ki2*Error_S+Trac_Kd2*(Error-Error_last);
	return output;
}
/**********
�ٶȻ�
**********/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//�����ٶ�ƫ��
	Encoder_Err=(encoder_left+encoder_right-Target);
	//���ٶ�ƫ����е�ͨ�˲�
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S+=EnC_Err_Lowout;
	//�����޷�
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	
	if(stop==1)Encoder_S=0,stop=0;//���������
	
	//�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}


/*********************************
����ң��
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
			Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//�޷�
			
			/*����*/
			if((Left==0)&&(Right==0))Turn_Speed=0;
			if(Left==1)Turn_Speed+=30;	//��ת
			if(Right==1)Turn_Speed-=30;	//��ת
			Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//�޷�
			
			/*ת��Լ��*/
			if((Left==0)&&(Right==0))Turn_Kd=0.6;//Kd��������������
			else if((Left==1)||(Right==1))Turn_Kd=0;//û�����ᳵ��ת��
			/*********************************************************************************************/
			Velocity_out=Velocity(Encoder_Left,Encoder_Right,Target_Speed);		
			
			Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);			
			
			Turn_out=Turn(gyroz,Turn_Speed);																
			
			MOTO1=Vertical_out-Turn_out;
			MOTO2=Vertical_out+Turn_out;
			Limit(&MOTO1,&MOTO2);		
			Load(MOTO1,MOTO2);
			
			Stop(&Med_Angle,&Pitch);//��ȫ���
}
/*****************
ƽ�⹦��
******************/
void Balance_Control(int target_speed)
{
	Target_Speed=target_speed;
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);			//�ٶȻ�
	Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);			//ֱ����
	Turn_Kd=0.6;
	Turn_out=Turn(gyroz,Turn_Speed);																//ת��
	
	MOTO1=Vertical_out-Turn_out;
	MOTO2=Vertical_out+Turn_out;
	Limit (&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}
/*****************
��Ŀ����
*****************/
void Tra_control(uint8_t mode)
{	
	Target_Speed=-25;//�����ٶȻ�ֵ
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
			/***�ư�Ȧ����Ƕ�***/
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
		case 4://flag=1,2,3,4�ֱ��ʾ��AC,CB,BD,DA��
	{
		if(flag==1) bias=Stra_control1(Yaw,-36.5);		//arctan(80/100)Լ����38.6598��
		if(flag==2) bias=Trac_Turn2(0,get_TraErr());
		if(flag==3)
		{
			/***�ư�Ȧ����Ƕ�***/
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
		case 5://flag=1,2,3,4�ֱ��ʾ��AC,CB,BD,DA��
		{
		if(flag==1)
		{
			if(number==1)      bias=Stra_control1(Yaw,-37);		//arctan(80/100)Լ����38.6598��
			else if(number==2) bias=Stra_control1(Yaw,-39);
			else if(number==3) bias=Stra_control1(Yaw,-39);
			else if(number==4) bias=Stra_control1(Yaw,-39);
		}
		if(flag==2) bias=Trac_Turn2(0,get_TraErr());
		if(flag==3)
		{
			/***�ư�Ȧ����Ƕ�***/
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
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);//�ٶȻ����
	Vertical_out=Vertical(Velocity_out+Med_Angle,Pitch,gyroy);//ƽ�⻷���
	MOTO1=Vertical_out-bias ;
	MOTO2=Vertical_out+bias ;
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
}

