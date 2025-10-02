#include "main_task.h"
#include "buzzer.h"
#include "Start_task.h"
#include "motor.h"
#include "turn.h"
#include "motor_task.h"
#include "math.h"
#include "speed_ctrl.h"
#include "uart.h"
#include "encoder.h"
#include "uart.h"
#include "delay.h"
#include "servo.h"
#include "vision.h"
#define SPPED 300
#define MS SPPED+100
#define basic_p -6.7
uint16_t times=0;
uint8_t flag = 0;
uint16_t a;
TaskHandle_t main_task_handler;
//配送要Rudder_control(100,11)是管子下放
//Rudder_control(400,11)管子回收
//Rudder_control(200,10)仓门打开
//Rudder_control(400,10)仓门关闭
void main_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //获取系统节拍
	GYRO_Start();	 	 //陀螺仪初始化
	Servo_Init();
	PID_init();
	vTaskDelay(2000);
	//motor_task_create();   //轮子开始运动
	OUT_servo_motor_angel();
//	SendCMD(0x03,SPPED,0x00,100,0x01,100,0x02,100,0x03,100);

	Sendone(0x03,SPPED,0x04,200);
		Sendone(0x03,SPPED,0x04,200);//夹住
	 HAL_Delay(MS);
		
		//扯下
	 one_servo.temp_angle = data2angel(125);
	 two_servo.temp_angle = data2angel(-125);
	 W_servo.temp_angle = data2angel(-90);
	 bastic_servo.temp_angle = data2angel(95);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);			

		//放果实过程中
	 one_servo.temp_angle = data2angel(125);
	 two_servo.temp_angle = data2angel(-125);
	 W_servo.temp_angle = data2angel(0);
	 bastic_servo.temp_angle = data2angel(-90);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);								

	//放果实
	 one_servo.temp_angle = data2angel(110);
	 two_servo.temp_angle = data2angel(-90);
	 W_servo.temp_angle = data2angel(-120);
		bastic_servo.temp_angle = data2angel(-90);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);
		
		
	 Sendone(0x03,SPPED,0x04,250);//松开
	 HAL_Delay(MS);
	 
	 //回去
	 one_servo.temp_angle = data2angel(90);
	 two_servo.temp_angle = data2angel(-90);
	 W_servo.temp_angle = data2angel(0);
	 bastic_servo.temp_angle = data2angel(-90);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);
							 
	SendCMD(0x03,SPPED,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	vTaskDelay(MS);
	Sendone(0x03,SPPED,0x04,250);
	vTaskDelay(MS);
	vTaskDelay(MS);
	while(1)
	
	while(1)
	{
				pid_mode_switch(is_Free);
	motor_set_pwm(4,0);//右前
motor_set_pwm(3,3000);//右前
			motor_set_pwm(1,0);//右前
motor_set_pwm(2,0);//右前
	}
while(1)

//	angle.AngleG = getAngleZ();
////	int nums = motor_all.Distance;
//	pid_mode_switch(is_Gyro); 
//	motor_all.Gspeed = 40;	
	//motor_k = 1.25f;

	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==1)
	{  //红外没有扫到
			pid_mode_switch(is_Free);
	motor_set_pwm(4,4000);//右前

		vTaskDelay(2);
	}	
	//红外扫到了
	while(1)
	{
//	motor_set_pwm(1,0);//右前
//	motor_set_pwm(2,0);//左后
//	motor_set_pwm(3,0);//右后
//	motor_set_pwm(4,0);//左前
		CarBrake();
		servo_control();
		vTaskDelay(2);
	}
	vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
}


//主任务创建
void main_task_create(void){
	  xTaskCreate((TaskFunction_t ) main_task,//任务函数
	               (const char *)"main_task",	  //任务名字
								 (uint32_t) main_task_size,    //任务堆栈大小
								 (void* )NULL,                  //传递给任务参数的指针参数
								 (UBaseType_t) main_task_priority, //任务的优先级
								(TaskHandle_t *)&main_task_handler ); //任务句柄
							 }
