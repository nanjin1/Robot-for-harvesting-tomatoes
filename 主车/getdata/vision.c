#include "vision.h"
#include "usart.h"
#include "string.h"
/*

        ���ڽ���NANO����
		  ������ ��ȷ��ʽ��FF  ID data_X_H data_X_L data_Y_H data_Y_L data_area У���
		  (IDΪ1ʱ������Ŀ��,area �����,�����ӳ�䵽0-255,���ش�С)
        
                                   */

#define BUFFER_SIZE 20
uint8_t vision_rx_buf[BUFFER_SIZE] = {0};
uint8_t vision_rx_len = 0;
volatile VIS_t vision;

void vision_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4,vision_rx_buf,BUFFER_SIZE);
}


void UART4_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);//��������жϱ�־λ
		HAL_UART_DMAStop(&huart4); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);  //��ȡ DMA ��ʣ������ֽ�û�д��䣬����õ����ν��յ������ݳ��� 
		vision_rx_len = BUFFER_SIZE - temp; 
		
		 if(vision_rx_buf[0] == 0xFF)//֡ͷ
		{
			uint8_t sum = 0;
			for (int i=2; i<7; i++)
				  sum += vision_rx_buf[i];//1-6������У���
			if (sum == vision_rx_buf[7])
			{
         vision.ID=vision_rx_buf[1];
				 vision.data_X_H= vision_rx_buf[2];
				 vision.data_X_L= vision_rx_buf[3];
				 vision.data_Y_H= vision_rx_buf[4];
				 vision.data_Y_L= vision_rx_buf[5];
				 vision.data_area=vision_rx_buf[6];
				
				 vision.data_X=((uint16_t)vision.data_X_H<<8)|(vision.data_X_L);
				 vision.data_Y=((uint16_t)vision.data_Y_H<<8)|(vision.data_Y_L);
				 HAL_UART_Transmit(&huart5,vision_rx_buf,8,0xffff);
			}
		}
		memset(vision_rx_buf,0,vision_rx_len);
		vision_rx_len = 0;
	}
	HAL_UART_Receive_DMA(&huart4,vision_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart4);
}

#if 1
//#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((UART4->SR&0X40)==0);//ѭ������,ֱ���������   
	UART4->DR=(uint8_t)ch;      
	return ch;
}
#endif 

