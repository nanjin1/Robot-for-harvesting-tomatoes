#include "usmart_task.h"
#include "usmart.h"
TaskHandle_t usmart_task_handler;
void usmart_task(void *pvParameters){
	while(1){
			usmart_dev.scan();	//ִ��usmartɨ��
			vTaskDelay(100);  
	}
}

//�����񴴽�
void usmart_task_create(void){
	  xTaskCreate((TaskFunction_t ) usmart_task,//������
	               (const char *)"main_task",	  //��������
								 (uint32_t) usmart_task_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) usmart_task_priority, //��������ȼ�
								(TaskHandle_t *)&usmart_task_handler ); //������
							 }
