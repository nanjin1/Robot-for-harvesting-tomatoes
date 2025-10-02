#include "sin_generate.h"
#include "math.h"
#include "motor.h"
#include "uart.h"
TaskHandle_t sin_task_handler;
#define PI 3.1415926f
//����ٶ�   �Ƕȿ��
struct sin_param sin_use_motor={0,0,300,0.15};

void sin_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
	while(1){
			motor_R0.target = sin_generator(&sin_use_motor);		//����sin����
			vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));//��������1ms // INCLUDE_vTaskDelayUntil 1
	}
}

//�����񴴽�
void sin_task_create(void){
	  xTaskCreate((TaskFunction_t ) sin_task,//������
	               (const char *)"sin_task",	  //��������
								 (uint32_t) sin_task_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) sin_task_priority, //��������ȼ�
								(TaskHandle_t *)&sin_task_handler ); //������
							 }


float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
