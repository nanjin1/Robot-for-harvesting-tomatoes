#include "sin_generate.h"
#include "math.h"
#include "motor.h"
#include "uart.h"
TaskHandle_t sin_task_handler;
#define PI 3.1415926f
//最大速度   角度跨度
struct sin_param sin_use_motor={0,0,300,0.15};

void sin_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //获取系统节拍
	while(1){
			motor_R0.target = sin_generator(&sin_use_motor);		//产生sin波形
			vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));//绝对休眠1ms // INCLUDE_vTaskDelayUntil 1
	}
}

//主任务创建
void sin_task_create(void){
	  xTaskCreate((TaskFunction_t ) sin_task,//任务函数
	               (const char *)"sin_task",	  //任务名字
								 (uint32_t) sin_task_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) sin_task_priority, //任务的优先级
								(TaskHandle_t *)&sin_task_handler ); //任务句柄
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
