#ifndef __VISION_H
#define __VISION_H
#include "main.h"
#include "stdio.h"
#include "usart.h"
void vision_receive_init(void);

typedef struct{
	
	 uint8_t ID;
	 uint8_t data_X_H;
	 uint8_t data_X_L;
	 int data_X;
	 uint8_t data_Y_H;
   uint8_t data_Y_L;	
	 int data_Y;
   uint8_t data_area;
}VIS_t;


volatile extern VIS_t vision;


#endif

