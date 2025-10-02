#ifndef __Servo_H
#define __Servo_H

void Servo_Init(void);
void OUT_servo_motor_angel(void);
int slove_angle(float x,float y,float z);
void get_XYZ(void) ;
int Calculate_coordinate(float x,float y,float z);

//角度单位均为数值 如180度为3.1415926
typedef struct servo_motor { 
	float MIN_angle;//最大角度  -相对于坐标轴
	float MAX_angle;//最小角度  -相对于坐标轴
	float zero_angle;//零角度   第一个舵机转动到与地面平行的角度  其余舵机跟着舵机为直线的角度
	float temp_angle;//坐标系角度  
	unsigned char dir;//舵机角度变大方向  
	float now_angle;//目前角度  -转化为舵机转动的真实角度   
	float weight;  //加权  权重越大意味着越不想动  
}Servo_motor;

typedef struct coordinate {
	float X;  //前方距离
	float Y;  //高度
	float Z;  //左右

}Coordinate;
float data2angel(float data);
volatile extern Servo_motor bastic_servo, one_servo , two_servo,W_servo;
volatile extern Coordinate  W_Coordinate;
void servo_control(void);
#endif
