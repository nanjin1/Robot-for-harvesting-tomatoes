#ifndef __USMART_TASK_H
#define __USMART_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern TaskHandle_t usmart_task_handler; 				//定义开始任务句柄
void usmart_task(void *pvParameters);//声明任务函数
#define usmart_task_size  256   					//任务堆栈大小
#define usmart_task_priority 8 			//任务优先级

#define usmart_task_open 0
void usmart_task_create(void);

#endif
