#include "gyro.h"
#include "usart.h"
#include "main.h"
#include "uart.h"
#include "string.h"
#include "usart.h"
#include "vision.h"
volatile struct Imu imu = {0,0,0,0,0};
#define gyro huart4

#define BUFFER_SIZE  20
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;
void gyro_init(void)
{	
	__HAL_UART_ENABLE_IT(&gyro, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&gyro,imu_rx_buf,BUFFER_SIZE);
}

////陀螺仪接收中断
void UART4_IRQHandler(){
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&gyro,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&gyro);
		HAL_UART_DMAStop(&gyro); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);   
		imu_rx_len = BUFFER_SIZE - temp; 
		
		if(imu_rx_buf[0] == 0x55)
		{
//			uint8_t sum = 0;
//			for (int i=0; i<10; i++)
//				sum += imu_rx_buf[i];
//			if (sum == imu_rx_buf[10]){
				if (imu_rx_buf[1] == 0X53)
				{
					//根据陀螺仪的安装方向确定roll和pitch
					imu.roll  = 180.0*(short)((imu_rx_buf[3]<<8)|imu_rx_buf[2])/32768.0f;  
					imu.pitch = 180.0*(short)((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0f;
					imu.yaw   = 180.0*(short)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0f;
//					if(imu.yaw>360) imu.yaw -= 360;
//					else if(imu.yaw<0) imu.yaw += 360;
				}
	//		}
		}
		imu_rx_len = 0;
		memset(imu_rx_buf,0,imu_rx_len);
	}
	HAL_UART_Receive_DMA(&gyro,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&gyro); 	
}
