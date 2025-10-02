#ifndef __motor_task_h__
#define __motor_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

enum PID_Mode {
	is_No = 0,  //关闭所有操作
	is_Free,   //保留切换前的状态
	is_Line,   //循迹
	is_Turn,   //转弯
	is_Gyro   //自平衡
};
struct Find_line{
	uint8_t id;
	uint8_t help_open;
	uint8_t sleep_dargon;
	uint8_t need;
};
extern struct Find_line find_line;
extern TaskHandle_t motor_handler; 				//定义开始任务句柄
void motor_task(void *pvParameters);//声明任务函数
#define motor_size  512  					//任务堆栈大小
#define motor_task_priority 8 			//任务优先级

extern uint8_t water_help;
extern volatile uint8_t PIDMode;
extern float back_compensate;  //后轮补偿
void motor_task_create(void); 
void pid_mode_switch(uint8_t target_mode);
void GET_MOTOR(void);
#endif
