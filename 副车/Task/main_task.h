#ifndef __main_task_h__
#define __main_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern TaskHandle_t main_task_handler; 				//���忪ʼ������
void main_task(void *pvParameters);//����������
#define main_task_size  512				//�����ջ��С
#define main_task_priority 8 			//�������ȼ�

#define main_task_open 1
void main_task_create(void);
#endif
