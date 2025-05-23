#include "LED.h"
void LED_Init(void )
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStrture;
	GPIO_InitStrture.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStrture.GPIO_Pin=GPIO_Pin_13 ;
	GPIO_InitStrture.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStrture);
	GPIO_SetBits(GPIOC,GPIO_Pin_13 );
}
void LED_ON(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_13 );
}
void LED_OFF(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13 );
}
void LED_Turn(void)
{
	LED_ON();
	delay_ms(500);
	LED_OFF();
}
