#ifndef __motor_task_h__
#define __motor_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

enum PID_Mode {
	is_No = 0,  //�ر����в���
	is_Free,   //�����л�ǰ��״̬
	is_Line,   //ѭ��
	is_Turn,   //ת��
	is_Gyro   //��ƽ��
};
struct Find_line{
	uint8_t id;
	uint8_t help_open;
	uint8_t sleep_dargon;
	uint8_t need;
};
extern struct Find_line find_line;
extern TaskHandle_t motor_handler; 				//���忪ʼ������
void motor_task(void *pvParameters);//����������
#define motor_size  512  					//�����ջ��С
#define motor_task_priority 8 			//�������ȼ�

extern uint8_t water_help;
extern volatile uint8_t PIDMode;
extern float back_compensate;  //���ֲ���
void motor_task_create(void); 
void pid_mode_switch(uint8_t target_mode);
void GET_MOTOR(void);
#endif
