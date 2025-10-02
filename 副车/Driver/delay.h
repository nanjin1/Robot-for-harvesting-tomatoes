#ifndef __DELAY_H
#define __DELAY_H
#include "main.h"

extern void delay_init(void);
extern void delay_us(uint16_t nus);  //΢�뼶��ʱ
extern void delay_ms(uint16_t nms);  //���뼶��ʱ

struct TIM{
	uint32_t time;
	uint32_t times;
	float sys_times;
};

extern struct TIM tim;
#endif   
