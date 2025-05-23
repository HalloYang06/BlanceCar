#ifndef __TRACK_H
#define __TRACK_H
#include "sys.h"

#define Track_Channel_1  GPIO_Pin_6
#define Track_Channel_2  GPIO_Pin_5
#define Track_Channel_3  GPIO_Pin_4
#define Track_Channel_4  GPIO_Pin_3

#define X1							 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
#define X2							 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)
#define X3							 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
#define X4							 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)

#define white_cnt1 5
#define white_cnt2 30
#define white_cnt3 10
#define black_cnt1 3
#define black_cnt2 100
#define black_cnt3 1
void Track_Init(void);
int  get_TraErr(void );
uint8_t Track_error(void);
uint8_t Trac_error2(void);
int Walk_routeACBD(int stop_flag);
int route_walk(void);


#endif
