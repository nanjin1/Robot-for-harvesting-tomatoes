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
//����ҪRudder_control(100,11)�ǹ����·�
//Rudder_control(400,11)���ӻ���
//Rudder_control(200,10)���Ŵ�
//Rudder_control(400,10)���Źر�
void main_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
	GYRO_Start();	 	 //�����ǳ�ʼ��
	Servo_Init();
	PID_init();
	vTaskDelay(2000);
	//motor_task_create();   //���ӿ�ʼ�˶�
	OUT_servo_motor_angel();
//	SendCMD(0x03,SPPED,0x00,100,0x01,100,0x02,100,0x03,100);

	Sendone(0x03,SPPED,0x04,200);
		Sendone(0x03,SPPED,0x04,200);//��ס
	 HAL_Delay(MS);
		
		//����
	 one_servo.temp_angle = data2angel(125);
	 two_servo.temp_angle = data2angel(-125);
	 W_servo.temp_angle = data2angel(-90);
	 bastic_servo.temp_angle = data2angel(95);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);			

		//�Ź�ʵ������
	 one_servo.temp_angle = data2angel(125);
	 two_servo.temp_angle = data2angel(-125);
	 W_servo.temp_angle = data2angel(0);
	 bastic_servo.temp_angle = data2angel(-90);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);								

	//�Ź�ʵ
	 one_servo.temp_angle = data2angel(110);
	 two_servo.temp_angle = data2angel(-90);
	 W_servo.temp_angle = data2angel(-120);
		bastic_servo.temp_angle = data2angel(-90);
	 OUT_servo_motor_angel();
	 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
	 HAL_Delay(2000+100);
		
		
	 Sendone(0x03,SPPED,0x04,250);//�ɿ�
	 HAL_Delay(MS);
	 
	 //��ȥ
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
	motor_set_pwm(4,0);//��ǰ
motor_set_pwm(3,3000);//��ǰ
			motor_set_pwm(1,0);//��ǰ
motor_set_pwm(2,0);//��ǰ
	}
while(1)

//	angle.AngleG = getAngleZ();
////	int nums = motor_all.Distance;
//	pid_mode_switch(is_Gyro); 
//	motor_all.Gspeed = 40;	
	//motor_k = 1.25f;

	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==1)
	{  //����û��ɨ��
			pid_mode_switch(is_Free);
	motor_set_pwm(4,4000);//��ǰ

		vTaskDelay(2);
	}	
	//����ɨ����
	while(1)
	{
//	motor_set_pwm(1,0);//��ǰ
//	motor_set_pwm(2,0);//���
//	motor_set_pwm(3,0);//�Һ�
//	motor_set_pwm(4,0);//��ǰ
		CarBrake();
		servo_control();
		vTaskDelay(2);
	}
	vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//��������5ms // INCLUDE_vTaskDelayUntil 1
}


//�����񴴽�
void main_task_create(void){
	  xTaskCreate((TaskFunction_t ) main_task,//������
	               (const char *)"main_task",	  //��������
								 (uint32_t) main_task_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) main_task_priority, //��������ȼ�
								(TaskHandle_t *)&main_task_handler ); //������
							 }
