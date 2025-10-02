#ifndef __PID_H
#define __PID_H


struct P_pid_obj {
	float output;
	float bias;
	float measure;
	float last_bias;
	float integral;						//积分
	float last_differential;	//上一次微分
	float target;
};

//differential_filterK: 微分项滤波系数，取值范围(0,1]
//系数越小滤波效应越大，当系数为1时不进行滤波
struct PID_param {
	float kp;
	float ki;
	float kd;
	float differential_filterK;			//滤波系数
	float outputMin;
	float outputMax;
	float actualMax;								//实际值限幅
};
volatile extern struct P_pid_obj  X_obj,Y_obj,Z_obj;
extern struct PID_param X_param,Y_param,Z_param;
float positional_PID (volatile struct P_pid_obj *obj, struct PID_param *pid);
void PID_init(void);
#endif
