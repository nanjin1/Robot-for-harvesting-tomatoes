#ifndef BUZZER_H
#define BUZZER_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
void buzzer_on(void);
void buzzer_off(void);
void buzzer_warn(void);
void buzzer_init(void);

extern TaskHandle_t buzzer_handler; 				//定义开始任务句柄
void buzzer_task(void *pvParameters);//声明任务函数
#define buzzer_task_size  30   					//任务堆栈大小
#define buzzer_task_priority 8 			//任务优先级
void buzzer_task_create(void);
#endif
