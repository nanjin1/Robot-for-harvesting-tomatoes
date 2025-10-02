#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "motor.h"

struct I_pid_obj {
	float output;					//���
	float bias;							//���
	float last_bias;				//�ϴ����
	float last2_bias;				
	float measure;					//����ֵ
	float target;						//Ŀ��ֵ
};

struct P_pid_obj {
	float output;
	float bias;
	float measure;
	float last_bias;
	float integral;						//����
	float last_differential;	//��һ��΢��
	float target;
};

//differential_filterK: ΢�����˲�ϵ����ȡֵ��Χ(0,1]
//ϵ��ԽС�˲�ЧӦԽ�󣬵�ϵ��Ϊ1ʱ�������˲�
struct PID_param {
	float kp;
	float ki;
	float kd;
	float differential_filterK;			//�˲�ϵ��
	float outputMin;
	float outputMax;
	float actualMax;								//ʵ��ֵ�޷�
};

volatile extern struct P_pid_obj  X_obj,Y_obj,Z_obj;
extern struct PID_param X_param,Y_param,Z_param;
void incremental_PID (struct I_pid_obj *motor, struct PID_param *pid);
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid);
void PID_init(void);

#endif
