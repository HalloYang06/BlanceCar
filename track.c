#include "track.h"
volatile int Area;
volatile int Err;
volatile uint8_t flag;
int count=0;
uint8_t number;
void Track_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStrture;
	GPIO_InitStrture.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStrture.GPIO_Pin=Track_Channel_1|Track_Channel_2|Track_Channel_3|Track_Channel_4;
	GPIO_InitStrture.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrture);
}


//uint8_t Track_error(void)
//{
//	if((X4==1)&&(X3==0)&&(X2==0)&&(X1==1)){Area=0;Err=0;}
//	else if(X2==1 && X3==0){Area=1;Err=-500;}//ƫ��һ��
//	else if((X4==0)&&(X3==1)&&(X2==1)&&(X1==1)){Area=2;Err=-1000;}//ƫ������
//	
//	else if(X2==0 && X3==1){Area=-1;Err=500;}//ƫ��һ��
//	else if(X1==0){Area=-2;Err=1000;}//ƫ������
//	else if((X4==1)&&(X3==1)&&(X2==1)&&(X1==1))
//	{	
//		if(Area==-1)        Err=1500;	//ƫ�Ҷ���
//		else if(Area==-2)   Err=2500;//ƫ���ļ�
//		else if(Area==2)    Err=-2500;//ƫ���ļ�
//		else if(Area==1)    Err=-1500;//ƫ�����
//	}
//	return Err;
//}

//uint8_t Trac_error2(void)
//{
//	if((X2==0)&&(X3==0)){Area=0;Err=0;}
//	else if((X2==0)&&(X3==1)){Area=1;Err=-2000;}//ƫ��
//	else if((X2==1)&&(X3==0)){Area=2;Err=2000;}//ƫ��
//	else if((X2==1)&&(X3)==1)
//	{
//		if(Area==1){Err=-2500;}
//		else if(Area==2){Err=2500;}
//	}
//	return Err;
//}

//X1 X2 X3 X4(�����Ҷ�Ӧ��·��)
int get_TraErr(void )
{	
	int error;
	error=-2*X1-X2+X3+2*X4;
	return error;
}
//int get_TraErr(void )
//{	
//	int error;
//	int X0;
//	if((X2==0)&&(X3==0)){Area=0;}
//	else if((X2==0)&&(X3==1)){Area=1;}//ƫ��
//	else if((X2==1)&&(X3==0)){Area=2;}//ƫ��
//	else if((X1==1)&&(X2==1)&&(X3==1)&&(X4==1))
//	{	
//		if     (Area==0){X0=0;}
//		else if(Area==1){Area=3;X0=1;}//����λ��
//		else if(Area==2){Area=4;X0=-1;}
//	}
//	if(Area==3||Area==4) error=2*X0;
//	else error=-3*X1+X2+X3+3*X4;	
//	return error;
//}
/**return��Ϊ�˱�־λ�仯���ú������µ���������״̬**/
int route_walk(void)
{	
		if(mode== 1)
	{
		flag=1;
		if(!X1||!X2||!X3||!X4)
		{
			count++;
			if(count>black_cnt1)
			{
				flag_prompt=1;
				flag=0;
				count=0;
				return 0;
			}
			
		}
		else count=0;
	}
		else if( mode==2)
	{
		flag=2;
		if(X1 && X2 && X3 && X4)
		{
			count++;
			if(count>white_cnt2)//�ж��Ƿ����㹻��İ�����ȷ���Ƿ񵽴�C��
			{
				flag_prompt=1;
				
				flag=0;
				
				count=0;
				return 1;
			}
		}
		else count=0;
	 }
		else if(mode==3)//��ABCD flag=1,2,3,4�ֱ��ʾС������AB BC CD DA��
	{	
		if(flag==1)
		{	/***�ж��Ƿ񵽴�B��***/
			if(!X1||!X2||!X3||!X4)
			{
				count++;
				if(count>black_cnt1)
				{	
					flag_prompt=1;
					flag=2;
					count=0;
					return 2;
				}
			}
			else count=0;
		}
		/***�ж��Ƿ񵽴�C��***/
		else if(flag==2)
		{
			if(X1 && X2 && X3 && X4)
			{
				count++;
				if(count>white_cnt2)//�ж��Ƿ����㹻��İ�����ȷ���Ƿ񵽴�C��
				{
					flag_prompt=1;	
					flag=3;
			
					count=0;
					return 3;
				}
			}
			else count=0;
		}
		/***�ж��Ƿ񵽴�D��***/
		else if(flag==3)
		{
			if(!X1||!X2||!X3||!X4)
			{
				count++;
				if(count>black_cnt1)
				{	
					flag_prompt=1;
					flag=4;
					count=0;
					return 4;
				}
			}
			else count=0;
		}
		/***�ж��Ƿ񵽴�A��**/
		else if(flag==4)
		{
			if(X1 && X2 &&X3 && X4)
			{
				count++;
				if(count>white_cnt2)
				{	
					flag_prompt=1;
					flag=0;
					
					return 5;
				}
			}
			else count=0;
		}
	}
		else if (mode==4)
	{
		Walk_routeACBD(0);
		return 3;
	}
		else if(mode==5)
	{	
		number=1;
		Walk_routeACBD(1);
		number=2;
		Walk_routeACBD(1);
		number=3;
		Walk_routeACBD(1);
		number=4;
		Walk_routeACBD(0);	
		return 4;
	}
	return 8;
}
	
/*************
�Խ�����
*************/

int Walk_routeACBD(int stop_flag)//��stop_falg����flag,����0��ʾֹͣ������1��ʾ����
{	
	static int count1=0;
		if(flag==1)
		{	
			/***�ж��Ƿ񵽴�C��***/
			if(X3==0||X4==0)
			{ 
				flag_prompt=1;
				flag=2;
				
				count1=0;
				return 2;
			}
			else count1=0;
		}
		/***�ж��Ƿ񵽴�B��***/
		else if(flag==2)
		{
			if(X1==1 && X2==1 && X3==1 && X4==1)
			{
				count1++;
				if(count1>white_cnt3)//�ж��Ƿ����㹻��İ�����ȷ���Ƿ񵽴�B��
				{
					flag_prompt=1;
					flag=3;
					
					count1=0;
					return 3;
				}
			}
			else count1=0;
		}
		/***�ж��Ƿ񵽴�D��***/
		else if(flag==3)
		{
			if(X2==0||X1==0)
			{
				count1++;
				if(count1>black_cnt3)
				{
					flag_prompt=1;
					flag=4;
					
					count1=0;
					return 4;
				}
			}
			else count1=0;
		}
		/***�ж��Ƿ񵽴�A��**/
		else if(flag==4)
		{
			if(X1 && X2 &&X3 && X4)
			{
				count1++;
				if(count1>white_cnt2)
				{	
					flag_prompt=1;
					flag=stop_flag;
					return 5;
				}
			}
			else count1=0;
		}
		return 0;
}	

