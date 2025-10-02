#ifndef __motor_h__
#define __motor_h__
#include "main.h"
#include "pid.h"
#include "tim.h"

#define MOTOR_PWM_MAX 7100

extern	struct PID_param L0_param;
extern	struct PID_param L1_param;
extern	struct PID_param R0_param;
extern  struct PID_param R1_param;


extern struct I_pid_obj motor_L0;
extern struct I_pid_obj motor_L1;
extern struct I_pid_obj motor_R0;
extern struct I_pid_obj motor_R1;


extern struct P_pid_obj line_pid_obj;    //巡线PID
extern struct PID_param line_pid_param;

extern struct P_pid_obj water_pid_obj;
extern struct PID_param water_pid_param;

// G自平衡  ,T转弯    陀螺仪PID
extern struct P_pid_obj gyroT_pid; 
extern struct P_pid_obj gyroG_pid;
extern struct PID_param gyroT_pid_param, gyroG_pid_param;	

void motor_init(void);
void motor_set_pwm(uint8_t motor, int32_t pid_out);
void pid_init(void);
void motor_pid_clear(void);
void usmart_pid(uint8_t motor,uint16_t aim,int mode);
#endif
