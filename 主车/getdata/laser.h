#ifndef __LASER_H
#define __LASER_H

#include "usart.h"
#include "stdbool.h"

void SendCMD(uint8_t CMD,int time, uint8_t ID1,int angle1,uint8_t ID2,int angle2,uint8_t ID3,int angle3,uint8_t ID4,int angle4);
typedef struct{
	
    UART_HandleTypeDef *laser_uart;
    bool laser_enable_switch;
    uint8_t rec_len;
    uint8_t laser_RX_OK;
    uint8_t laser_RX_data[20];
	uint8_t distance_H;
	uint8_t distance_L;
	
}LASER_t;
extern LASER_t laser;
void LASER_decode(void);
bool Get_LASER_finish(void);
void LASER_receive(void);
void cmd_encode_laser(void);
void send_value_laser(void);
void Sendone(uint8_t CMD,int time, uint8_t ID1,int angle1);
#endif
