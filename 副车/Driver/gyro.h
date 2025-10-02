#ifndef __gyro_h__
#define __gyro_h__
#include "main.h"
struct Imu
{
	float yaw;   //����
	float roll;		//�෭
	float pitch;  //̧ͷ

	float compensateZ;
	float compensatePitch;
};

volatile extern struct Imu imu;
void gyro_init(void);
#endif
