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

extern TaskHandle_t buzzer_handler; 				//���忪ʼ������
void buzzer_task(void *pvParameters);//����������
#define buzzer_task_size  30   					//�����ջ��С
#define buzzer_task_priority 8 			//�������ȼ�
void buzzer_task_create(void);
#endif
