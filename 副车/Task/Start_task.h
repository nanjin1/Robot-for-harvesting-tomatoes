#ifndef __Start_task_h__
#define __Start_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//��ʼ����
extern TaskHandle_t Start_handler ; //���忪ʼ������
void Start_task(void *pvParameters);//����������
#define Start_size  512   //�����ջ��С
#define Start_task_priority 32  //�������ȼ�

void Start_task_create(void);
void user_init(void);
void GET_free_RAM(TaskHandle_t xTask);
void LED_init(void);
void LED_twinkle(void);
void  GYRO_Start(void);
#endif
