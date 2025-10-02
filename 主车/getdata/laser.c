#include "laser.h"
#include <string.h>
#include "usart.h"




void SendCMD(uint8_t CMD,int time, uint8_t ID1,int angle1,uint8_t ID2,int angle2,uint8_t ID3,int angle3,uint8_t ID4,int angle4)
{
	uint8_t Tx_CMD[19]={0};
  int data1=0,data2=0,data3=0,data4=0;
  data1=500+angle1*(2000/270);
	data2=500+angle2*(2000/270);
	data3=500+angle3*(2000/270);
	data4=500+angle4*(2000/270);
	
	Tx_CMD[0] = 0x55; Tx_CMD[1] = 0x55;        Tx_CMD[2] = 0x0E+3;        Tx_CMD[3] = CMD; 
	Tx_CMD[4] = 0x04; Tx_CMD[5] = time&0xFF;   Tx_CMD[6] = time>>8;
	Tx_CMD[7] = ID1;  	Tx_CMD[8] = data1&0x00FF;    Tx_CMD[9] = data1>>8;
	Tx_CMD[10] = ID2;  	Tx_CMD[11] = data2&0x00FF;   Tx_CMD[12] = data2>>8;
	Tx_CMD[13] = ID3;  	Tx_CMD[14] = data3&0x00FF;   Tx_CMD[15] = data3>>8;
	Tx_CMD[16] = ID4;  	Tx_CMD[17] = data4&0x00FF;   Tx_CMD[18] = data4>>8;

	HAL_UART_Transmit(&huart3,Tx_CMD,19,0xffff);

}


void Sendone(uint8_t CMD,int time, uint8_t ID1,int angle1)
{
	uint8_t Tx_CMD[10]={0};
  int data1=0;
  data1=500+angle1*(2000/270);

	Tx_CMD[0] = 0x55; Tx_CMD[1] = 0x55;        Tx_CMD[2] = 0x08;        Tx_CMD[3] = CMD; 
	Tx_CMD[4] = 0x01; Tx_CMD[5] = time&0xFF;   Tx_CMD[6] = time>>8;
	Tx_CMD[7] = ID1;  	Tx_CMD[8] = data1&0x00FF;    Tx_CMD[9] = data1>>8;


	HAL_UART_Transmit(&huart3,Tx_CMD,10,0xffff);

}