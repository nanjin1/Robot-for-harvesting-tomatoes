#ifndef __USMART_TASK_H
#define __USMART_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern TaskHandle_t usmart_task_handler; 				//���忪ʼ������
void usmart_task(void *pvParameters);//����������
#define usmart_task_size  256   					//�����ջ��С
#define usmart_task_priority 8 			//�������ȼ�

#define usmart_task_open 0
void usmart_task_create(void);

#endif
