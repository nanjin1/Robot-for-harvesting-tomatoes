#include "usmart_task.h"
#include "usmart.h"
TaskHandle_t usmart_task_handler;
void usmart_task(void *pvParameters){
	while(1){
			usmart_dev.scan();	//执行usmart扫描
			vTaskDelay(100);  
	}
}

//主任务创建
void usmart_task_create(void){
	  xTaskCreate((TaskFunction_t ) usmart_task,//任务函数
	               (const char *)"main_task",	  //任务名字
								 (uint32_t) usmart_task_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) usmart_task_priority, //任务的优先级
								(TaskHandle_t *)&usmart_task_handler ); //任务句柄
							 }
