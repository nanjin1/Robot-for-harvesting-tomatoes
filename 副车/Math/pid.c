#include "pid.h"
#include "uart.h"

volatile struct P_pid_obj  X_obj ={0,0,0,0,0,0,0};
volatile struct P_pid_obj  Y_obj ={0,0,0,0,0,0,0};
volatile struct P_pid_obj  Z_obj ={0,0,0,0,0,0,0};

struct PID_param X_param = {0,0,0,0,0,0,0};
struct PID_param Y_param = {0,0,0,0,0,0,0};
struct PID_param Z_param = {0,0,0,0,0,0,0};

void PID_init(void){
	X_param.kp = 0.05f;
	X_param.ki = 0;
	X_param.kd = 0;
	X_param.outputMax = 10;
	X_param.outputMin = -10;
	X_param.differential_filterK = 1;
	
	Y_param.kp = 0.05f;
	Y_param.ki = 0;
	Y_param.kd = 0;
	Y_param.outputMax = 10;
	Y_param.outputMin = -10;
	Y_param.differential_filterK = 1;
	
	Z_param.kp = 0.1f;
	Z_param.ki = 0;
	Z_param.kd = 0;
	Z_param.outputMax = 10;
	Z_param.outputMin = -10;
	Z_param.differential_filterK = 1;
}


//����ʽPID
//�������ֱ���
void incremental_PID (struct I_pid_obj *motor, struct PID_param *pid)
{
	float proportion = 0, integral = 0, differential = 0;
	
	motor->bias = motor->target - motor->measure;
	
	proportion = motor->bias - motor->last_bias;
	
	//�����ֱ���
	if (motor->output > pid->outputMax || motor->measure > pid->actualMax)
	{
		if (motor->bias < 0)
			integral = motor->bias;
	}
	else if (motor->output < -pid->outputMax || motor->measure < -pid->actualMax)
	{
		if (motor->bias > 0)
			integral = motor->bias;
	}
	else
	{
		integral = motor->bias;
	}
	
	differential = (motor->bias - 2 * motor->last_bias + motor->last2_bias);
	
	motor->output += pid->kp*proportion + pid->ki*integral + pid->kd*differential;
	
	motor->last2_bias = motor->last_bias;
	motor->last_bias = motor->bias;
}

//λ��ʽPID
//�������ֱ���
//��΢�����ͨ�˲�
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid)
{
	float differential = 0;
	
	obj->bias = obj->target - obj->measure;
	
	if (obj->output >= pid->outputMax)
	{
		if (obj->bias < 0)
			obj->integral += obj->bias;
	}
	else if (obj->output <= pid->outputMin)
	{
		if (obj->bias > 0)
			obj->integral += obj->bias;
	}
	else
	{
		obj->integral += obj->bias;
	}
	
	//΢�����ͨ�˲�
	differential = (obj->bias - obj->last_bias) * pid->differential_filterK + 
					(1 - pid->differential_filterK) * obj->last_differential;
	
	obj->output = pid->kp * obj->bias + pid->ki * obj->integral + pid->kd * differential;
	
	obj->last_bias = obj->bias;
	obj->last_differential = differential;
	
	return obj->output;
}

