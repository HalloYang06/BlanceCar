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
//	else if(X2==1 && X3==0){Area=1;Err=-500;}//偏左一级
//	else if((X4==0)&&(X3==1)&&(X2==1)&&(X1==1)){Area=2;Err=-1000;}//偏左三级
//	
//	else if(X2==0 && X3==1){Area=-1;Err=500;}//偏右一级
//	else if(X1==0){Area=-2;Err=1000;}//偏右三级
//	else if((X4==1)&&(X3==1)&&(X2==1)&&(X1==1))
//	{	
//		if(Area==-1)        Err=1500;	//偏右二级
//		else if(Area==-2)   Err=2500;//偏右四级
//		else if(Area==2)    Err=-2500;//偏左四级
//		else if(Area==1)    Err=-1500;//偏左二级
//	}
//	return Err;
//}

//uint8_t Trac_error2(void)
//{
//	if((X2==0)&&(X3==0)){Area=0;Err=0;}
//	else if((X2==0)&&(X3==1)){Area=1;Err=-2000;}//偏右
//	else if((X2==1)&&(X3==0)){Area=2;Err=2000;}//偏左
//	else if((X2==1)&&(X3)==1)
//	{
//		if(Area==1){Err=-2500;}
//		else if(Area==2){Err=2500;}
//	}
//	return Err;
//}

//X1 X2 X3 X4(从左到右对应的路数)
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
//	else if((X2==0)&&(X3==1)){Area=1;}//偏右
//	else if((X2==1)&&(X3==0)){Area=2;}//偏左
//	else if((X1==1)&&(X2==1)&&(X3==1)&&(X4==1))
//	{	
//		if     (Area==0){X0=0;}
//		else if(Area==1){Area=3;X0=1;}//黑线位于
//		else if(Area==2){Area=4;X0=-1;}
//	}
//	if(Area==3||Area==4) error=2*X0;
//	else error=-3*X1+X2+X3+3*X4;	
//	return error;
//}
/**return是为了标志位变化后让函数重新调用来更新状态**/
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
			if(count>white_cnt2)//判断是否检测足够多的白线来确定是否到达C点
			{
				flag_prompt=1;
				
				flag=0;
				
				count=0;
				return 1;
			}
		}
		else count=0;
	 }
		else if(mode==3)//走ABCD flag=1,2,3,4分别表示小车会于AB BC CD DA上
	{	
		if(flag==1)
		{	/***判断是否到达B点***/
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
		/***判断是否到达C点***/
		else if(flag==2)
		{
			if(X1 && X2 && X3 && X4)
			{
				count++;
				if(count>white_cnt2)//判断是否检测足够多的白线来确定是否到达C点
				{
					flag_prompt=1;	
					flag=3;
			
					count=0;
					return 3;
				}
			}
			else count=0;
		}
		/***判断是否到达D点***/
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
		/***判断是否到达A点**/
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
对角走线
*************/

int Walk_routeACBD(int stop_flag)//把stop_falg传给flag,传入0表示停止，传入1表示继续
{	
	static int count1=0;
		if(flag==1)
		{	
			/***判断是否到达C点***/
			if(X3==0||X4==0)
			{ 
				flag_prompt=1;
				flag=2;
				
				count1=0;
				return 2;
			}
			else count1=0;
		}
		/***判断是否到达B点***/
		else if(flag==2)
		{
			if(X1==1 && X2==1 && X3==1 && X4==1)
			{
				count1++;
				if(count1>white_cnt3)//判断是否检测足够多的白线来确定是否到达B点
				{
					flag_prompt=1;
					flag=3;
					
					count1=0;
					return 3;
				}
			}
			else count1=0;
		}
		/***判断是否到达D点***/
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
		/***判断是否到达A点**/
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

