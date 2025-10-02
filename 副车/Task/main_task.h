#ifndef __main_task_h__
#define __main_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern TaskHandle_t main_task_handler; 				//定义开始任务句柄
void main_task(void *pvParameters);//声明任务函数
#define main_task_size  512				//任务堆栈大小
#define main_task_priority 8 			//任务优先级

#define main_task_open 1
void main_task_create(void);
#endif
