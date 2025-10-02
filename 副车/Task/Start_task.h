#ifndef __Start_task_h__
#define __Start_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//开始任务
extern TaskHandle_t Start_handler ; //定义开始任务句柄
void Start_task(void *pvParameters);//声明任务函数
#define Start_size  512   //任务堆栈大小
#define Start_task_priority 32  //任务优先级

void Start_task_create(void);
void user_init(void);
void GET_free_RAM(TaskHandle_t xTask);
void LED_init(void);
void LED_twinkle(void);
void  GYRO_Start(void);
#endif
