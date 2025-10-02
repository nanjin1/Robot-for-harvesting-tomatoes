#ifndef __gyro_h__
#define __gyro_h__
#include "main.h"
struct Imu
{
	float yaw;   //×óÓÒ
	float roll;		//²à·­
	float pitch;  //Ì§Í·

	float compensateZ;
	float compensatePitch;
};

volatile extern struct Imu imu;
void gyro_init(void);
#endif
