#ifndef __SIN_GENERATE_H
#define __SIN_GENERATE_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

struct sin_param {
	uint16_t time;	    //ʱ��
	float actual_t;			//�Ƕ�
	float gain;					//����ٶ�
	float angular_velocity;		//�Ƕȿ��
};

#define sin_work 0			
extern struct sin_param sin_use_motor;
float sin_generator(struct sin_param *param);

extern TaskHandle_t sin_handler; 				//���忪ʼ������
void sin_task(void *pvParameters);//����������
#define sin_task_size  256   					//�����ջ��С
#define sin_task_priority 8 			//�������ȼ�
void sin_task_create(void);
#endif

