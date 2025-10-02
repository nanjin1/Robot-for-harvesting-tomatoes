#include "buzzer.h"
TaskHandle_t buzzer_task_handler;

void buzzer_task(void *pvParameters){
			buzzer_warn();
		  vTaskDelete(NULL); //ɾ����ʼ����
}

//�����񴴽�
void buzzer_task_create(void){
	  xTaskCreate((TaskFunction_t ) buzzer_task,//������
	               (const char *)"buzzer_task",	  //��������
								 (uint32_t) buzzer_task_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) buzzer_task_priority, //��������ȼ�
								(TaskHandle_t *)&buzzer_task_handler ); //������
							 }

							 
void buzzer_init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void buzzer_on(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
}

void buzzer_off(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
}

void buzzer_warn(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	vTaskDelay(200);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
}
