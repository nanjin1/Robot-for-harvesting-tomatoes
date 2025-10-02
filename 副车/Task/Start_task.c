#include "Start_task.h"
#include "encoder.h"
#include "uart.h"
#include "delay.h"
#include  "motor.h"
#include "usmart.h"
#include "motor_task.h"
#include "buzzer.h"
#include "gyro.h"
#include "main_task.h"
#include "usmart_task.h"
#include "turn.h"
#include "vision.h"
TaskHandle_t Start_handler ; //���忪ʼ������

void Start_task(void *pvParameters){
			taskENTER_CRITICAL();           //�����ٽ���
	    #if sin_work == 1
			sin_task_create();
			#endif
			#if main_task_open == 1
			main_task_create();
			#endif
		  #if usmart_task_open==1
			usmart_task_create();
			#endif
			user_init();                 //�û���ʼ��
			vTaskDelete(Start_handler); //ɾ����ʼ����
	  	taskEXIT_CRITICAL();            //�˳��ٽ���
}

/*****************************************************************************
�������� GET_free_RAM
�������ܣ���������ʣ���ջ��С ���Ҵ�ӡ  
�βΣ�������ľ��      ������NULL����Ϊ������Ķ�ջ 
ע�⣺INCLUDE_uxTaskGetStackHighWaterMark 1    ����ʹ��
*******************************************************************************/
void GET_free_RAM(TaskHandle_t xTask){
//		printf("RAM = %d\r\n",(int32_t)uxTaskGetStackHighWaterMark(xTask));
		vTaskDelay(500);
}

/*****************************************************************************
�������� user_init
�������ܣ���ʼ�����裬�ṹ��� 
*******************************************************************************/
void user_init(void){	
	
	uart_init(115200);  		//��ʼ���ض��򴮿�
	usmart_dev.init(72);	  //��ʼ��USMART	
	delay_init();        //��ʼ����ʱ����
	Encoder_init();     		//��ʼ��������
	LED_init();							//��ʼ��LED
	gyro_init();			     //�����ǳ�ʼ��
	vision_receive_init();
	motor_init();          //��ʼ�������Լ�PID����
//    mv_init(115200);
}

void Start_task_create(void){
	  xTaskCreate((TaskFunction_t ) Start_task,//������
	               (const char *)"Start_task",	  //��������
								 (uint32_t) Start_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) Start_task_priority, //��������ȼ�
								(TaskHandle_t *)&Start_handler ); //������
}

void LED_init(void){
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void LED_twinkle(void){
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void  GYRO_Start(void){
	float mpuZ_reset_val = 0;
	float temp_compensatePitch = 0;
	vTaskDelay(2000);
	for (int i = 0; i<10; i++)
	{
		vTaskDelay(20);
		mpuZ_reset_val += imu.yaw;
		temp_compensatePitch += imu.pitch;
	}
	mpuZ_reset_val /= 10;
	mpuZreset(mpuZ_reset_val, 0);
	imu.compensatePitch = temp_compensatePitch/10;
}
