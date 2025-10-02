#include "buzzer.h"
TaskHandle_t buzzer_task_handler;

void buzzer_task(void *pvParameters){
			buzzer_warn();
		  vTaskDelete(NULL); //删除开始任务
}

//主任务创建
void buzzer_task_create(void){
	  xTaskCreate((TaskFunction_t ) buzzer_task,//任务函数
	               (const char *)"buzzer_task",	  //任务名字
								 (uint32_t) buzzer_task_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) buzzer_task_priority, //任务的优先级
								(TaskHandle_t *)&buzzer_task_handler ); //任务句柄
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
