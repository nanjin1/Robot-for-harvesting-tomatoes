#include "speed_ctrl.h"
#include "motor_task.h"
#include "pid.h"
#define need_gradual_cal 0
volatile struct Motors motor_all = {
	.Lspeed = 0,
	.Rspeed = 0,
	.encoder_avg = 0,
	.GyroG_speedMax = 800,
	.GyroT_speedMax = 800,
	.Gincrement = 150,		//��ѭ�����ٶ�
	
	
	.is_UP = false,
	.is_DOWM = false,
};
//TC_speed:Ѳ���ٶ�       TG_speed : ��ƽ���ٶ�
struct Gradual TG_speed = {0,0,0},CG_speed = {0,0,0};


void CarBrake(void)
{
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = 0;
}

//��һ�κ����������ٻ��߻���ֹͣ
//�β� ���ٶȾ��  Ŀ���ٶ�  ���ٶ�
void gradual_cal(struct Gradual *gradual, float target, float increment)  
{
	#if need_gradual_cal
	uint8_t direction = 0;
	
	if(target - gradual->Now < 0) 
		direction = 0;			//������� 
	else
		direction = 1;			//�������
		
	if(gradual->Now != target)	
	{
		if (direction)     
			gradual->Now += increment;
		else							
			gradual->Now -= increment;
	}
	else 
	{	
		return;				//�ﵽĿ���ٶ�
	}
	
	//���ٴ���
	if(direction == 1)
	{
		if(gradual->Now > target) 		
		{
			gradual->Now = target;
		}
	}
	else if(direction == 0)			
	{
		if(gradual->Now < target)
		{
			gradual->Now = target;
		}
	}
	#else
		gradual->Now = target;
	#endif
}

