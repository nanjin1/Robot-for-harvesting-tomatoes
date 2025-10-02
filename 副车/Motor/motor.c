#include "motor.h"
#include "uart.h"
struct I_pid_obj motor_L0 = {0,0,0,0,0,0};
struct I_pid_obj motor_L1 = {0,0,0,0,0,0};
struct I_pid_obj motor_R0 = {0,0,0,0,0,0};
struct I_pid_obj motor_R1 = {0,0,0,0,0,0};

struct PID_param L0_param = {0,0,0,0,0,0,0};
struct PID_param L1_param = {0,0,0,0,0,0,0};
struct PID_param R0_param = {0,0,0,0,0,0,0};
struct PID_param R1_param = {0,0,0,0,0,0,0};

struct P_pid_obj line_pid_obj = {0,0,0,0,0,0};
struct PID_param line_pid_param;

struct P_pid_obj water_pid_obj = {0,0,0,0,0,0};
struct PID_param water_pid_param;

struct P_pid_obj gyroT_pid = {0,0,0,0,0,0};
struct P_pid_obj gyroG_pid = {0,0,0,0,0,0};
struct PID_param gyroT_pid_param, gyroG_pid_param;

//初始化轮子以及PID参数
void motor_init(void){
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
		
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
		pid_init();
}


void pid_init(void){
	
	L0_param.kp = 50;
	L0_param.ki = 4;
	L0_param.kd = 0;
	L0_param.differential_filterK = 0.5;
	L0_param.outputMax = MOTOR_PWM_MAX + 100;
	L0_param.actualMax = 300;
	
	L1_param.kp = 50;
	L1_param.ki = 4;
	L1_param.kd = 0;
	L1_param.differential_filterK = 0.5;
	L1_param.outputMax = MOTOR_PWM_MAX + 100;
	L1_param.actualMax = 300;
	
	R0_param.kp = 50;
	R0_param.ki = 4;
	R0_param.kd = 0;
	R0_param.differential_filterK = 0.5;
	R0_param.outputMax = MOTOR_PWM_MAX + 100;
	R0_param.actualMax = 300;
	
	R1_param.kp = 50;
	R1_param.ki = 4;
	R1_param.kd = 0;
	R1_param.differential_filterK = 0.5;
	R1_param.outputMax = MOTOR_PWM_MAX + 100;
	R1_param.actualMax = 300;

	
	gyroG_pid_param.kp=10;
	gyroG_pid_param.ki=0;
	gyroG_pid_param.kd=0;
	gyroG_pid_param.differential_filterK =1;
	gyroG_pid_param.outputMax=2000;
	
	gyroT_pid_param.kp = 40;
	gyroT_pid_param.ki = 0;
	gyroT_pid_param.kd = 0;
	gyroT_pid_param.outputMax = 300;
	gyroT_pid_param.differential_filterK = 1;
	

	motor_pid_clear();
}


void motor_pid_clear(void){
	motor_L0 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_L1 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_R0 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_R1 = (struct I_pid_obj){0,0,0,0,0,0};
}

/*
函数名：motor_set_pwm   L0 L1 R0 R1  顺序
*/
void motor_set_pwm(uint8_t motor, int32_t pid_out){


	int32_t ccr = 0;
	
	if (pid_out >= 0)
	{
		if (pid_out > MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR4 = 0; TIM1->CCR3  = ccr;	break;  
			case 2: TIM1->CCR2 = 0; TIM1->CCR1 = ccr; break; 	
			case 3: TIM8->CCR3 = 0; TIM8->CCR4 = ccr;	break;	
			case 4: TIM8->CCR1 = 0; TIM8->CCR2 = ccr; break;		

//			case 1: TIM8->CCR1 = 0;TIM8->CCR2 =  ccr;	break;  
//			case 2: TIM1->CCR2 = 0;TIM1->CCR1 = ccr; break; 	
//			case 3: TIM8->CCR3 = 0;TIM8->CCR4 = ccr;	break;	
//			case 4: TIM1->CCR4 = 0;TIM1->CCR3  = ccr;	break;			
			default: ; //TODO
		}
	}
	
	else if (pid_out < 0)
	{
		if (pid_out < -MOTOR_PWM_MAX)
			ccr = MOTOR_PWM_MAX;
		else
			ccr = -pid_out;
		
		switch (motor)
		{
			case 1: TIM1->CCR4 = ccr; TIM1->CCR3 = 0;	break;  
			case 2: TIM1->CCR2 = ccr; TIM1->CCR1 = 0;  break; 	
			case 3: TIM8->CCR3 = ccr; TIM8->CCR4 = 0;		break;	
			case 4: TIM8->CCR1 = ccr; TIM8->CCR2 = 0; 	break;		
			
//			case 1: TIM8->CCR1 = ccr;TIM8->CCR2 =  0;	break;  
//			case 2: TIM1->CCR2 = ccr; TIM1->CCR1 = 0; break; 	
//			case 3: TIM8->CCR3 = ccr; TIM8->CCR4 = 0;	break;	
//			case 4: TIM1->CCR4 = ccr;TIM1->CCR3  = 0;	break;		
			default: ; //TODO
		}
	}
}

//usmart的调试函数，用于修改PID参数
//由于usmart不支持浮点数，所以输入一个整数和一个要除以的位数(deno)
void usmart_pid(uint8_t motor,uint16_t aim,int mode)
{
	float use_aim = (float)aim;
	struct PID_param *motor_pid_param;
	struct I_pid_obj *motor_obj;
	switch(motor){
		case 0:
			motor_pid_param = &L0_param;
		  motor_obj = &motor_L0;
			break;
		case 1:
			motor_pid_param = &L1_param;
		  motor_obj = &motor_L1;
			break;
		case 2:
			motor_pid_param = &R0_param;
		  motor_obj = &motor_R0;
			break;
		case 3:
			motor_pid_param = &R1_param;
		  motor_obj = &motor_R1;
			break;
	}
	switch(mode)
	{
		case 1:
			motor_pid_param->kp=use_aim/100;  //mode1: 修改Kp
			break;
		case 2:
			motor_pid_param->ki=use_aim/100;  //mode2: Ki
			break;
		case 3:
			motor_pid_param->kd=use_aim/100;  //mode3: Kd
			break;
		case 4:
			motor_obj->target=use_aim/100;  //mode4: Target
			break;
	}
	printf("Kp:%f, Ki:%f, Kd:%f, Target:%f\r\n",
				motor_pid_param->kp,motor_pid_param->ki,motor_pid_param->kd,motor_obj->target);
		motor_pid_clear();
}
