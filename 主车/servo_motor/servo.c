#include "servo.h"
#include <math.h>
#include "pid.h"
#include "vision.h"
#include "laser.h"
//全局单位cm
#define L1 22.736f
#define L2 22.402f
#define PI 3.1415926f
#define SPPED 300
#define MS SPPED+100
#define back  1
#define forward 0
#define LEFT 2
#define RIGHT 3
volatile Servo_motor bastic_servo, one_servo , two_servo,W_servo;
volatile Coordinate  W_Coordinate;

//0     300

//300
void servo_control(void){
	static uint16_t times =0 ;
	if(vision.data_X==0||vision.data_Y==0){ //找番茄模式
			get_XYZ(); //获得当前位置	 		
		  times++;
		  if(times>8){
				if(times>1000){
					times= 9;
				}
				one_servo.temp_angle = data2angel(125);
				two_servo.temp_angle = data2angel(-125);
				W_servo.temp_angle = data2angel(0);
				OUT_servo_motor_angel();
				SendCMD(0x03,SPPED,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
				HAL_Delay(MS);						
			 }
			else
			{
				if(Calculate_coordinate(W_Coordinate.X -2,W_Coordinate.Y,W_Coordinate.Z)){
							OUT_servo_motor_angel();
							SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
							HAL_Delay(MS);						
				}		
		 }
	}
	else{
		times=0;
	  if(vision.ID == 2){
			Z_obj.measure = (float)vision.data_X;  //左右
			Y_obj.measure = (float)vision.data_Y;  //上下
			X_obj.measure = (float)vision.data_area;  //像素值
			
			//中心
			Z_obj.target = 160.0f;
			Y_obj.target = 160.0f;
			X_obj.target  = 20.0f;
			
			float Z_error = positional_PID(&Z_obj,&Z_param); //angle
			float Y_error = positional_PID(&Y_obj,&Y_param);
			float X_error = positional_PID(&X_obj,&X_param);
			get_XYZ(); //获得当前位置
			
			
		//	if(X_error<1.0f){
		//		if(Calculate_coordinate(W_Coordinate.X + 10 ,15,data2angel(90))){
		//						OUT_servo_motor_angel();
		//						SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
		//						HAL_Delay(MS);
		//						printf("BB = %f   %d\r\n",X_error,vision.data_area);
		//	        }
		//		Sendone(0x03,SPPED,0x05,260);
		//	  HAL_Delay(MS);
		//		 while(1);
		//	}
		//	if(Calculate_coordinate(W_Coordinate.X + X_error,15,data2angel(90))){
		//						OUT_servo_motor_angel();
		//						SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
		//						HAL_Delay(MS);
		//						printf("BB = %f   %d\r\n",X_error,vision.data_area);
		//	        }


			if(fabs(Z_error)+ fabs(Y_error) <10.0f ||vision.data_area>40){  //误差很小时
					
					X_obj.measure = (float)vision.data_area;  //像素值
					X_obj.target  = 15.0f;
					float X_error = positional_PID(&X_obj,&X_param);

					if(fabs(vision.data_area - X_obj.target)<3.0f||vision.data_area>40){
						float aa = 0;
						if(vision.data_area>50)
						   aa = W_Coordinate.X;
						else if(vision.data_area>40)
							aa = W_Coordinate.X+5;
						else 
							 aa = W_Coordinate.X+15;
							if(Calculate_coordinate(aa,W_Coordinate.Y,W_Coordinate.Z)){
								OUT_servo_motor_angel();
								SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
								HAL_Delay(MS);
								printf("BB = %f   %d\r\n",X_error,vision.data_area);
								Sendone(0x03,SPPED,0x05,150);
							 HAL_Delay(MS);
								
							 one_servo.temp_angle = data2angel(125);
							 two_servo.temp_angle = data2angel(-125);
							 W_servo.temp_angle = data2angel(-90);
							 bastic_servo.temp_angle = data2angel(95);
							 OUT_servo_motor_angel();
							 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
							 HAL_Delay(2000+100);			

							 one_servo.temp_angle = data2angel(125);
							 two_servo.temp_angle = data2angel(-125);
							 W_servo.temp_angle = data2angel(0);
							 bastic_servo.temp_angle = data2angel(-90);
							 OUT_servo_motor_angel();
							 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
							 HAL_Delay(2000+100);								
					
							 one_servo.temp_angle = data2angel(110);
							 two_servo.temp_angle = data2angel(-90);
							 W_servo.temp_angle = data2angel(-120);
								bastic_servo.temp_angle = data2angel(-90);
							 OUT_servo_motor_angel();
							 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
							 HAL_Delay(2000+100);
								
								
							 Sendone(0x03,SPPED,0x05,	50);
							 HAL_Delay(MS);
							 
							 one_servo.temp_angle = data2angel(90);
							 two_servo.temp_angle = data2angel(-90);
							 W_servo.temp_angle = data2angel(0);
							 bastic_servo.temp_angle = data2angel(-90);
							 OUT_servo_motor_angel();
							 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
							 HAL_Delay(2000+100);
							 
							 one_servo.temp_angle = data2angel(125);
							 two_servo.temp_angle = data2angel(-125);
							 W_servo.temp_angle = data2angel(0);
								bastic_servo.temp_angle = data2angel(95);
							 OUT_servo_motor_angel();
							 SendCMD(0x03,2000,0x00,bastic_servo.now_angle,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle);
							 HAL_Delay(2000+100);
							}
					}
					else{
						if(Calculate_coordinate(W_Coordinate.X + X_error,W_Coordinate.Y+ Y_error,W_Coordinate.Z+ data2angel(Z_error))){
							OUT_servo_motor_angel();
							SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
							HAL_Delay(MS);
							printf("AA = %f   %d\r\n",X_error,vision.data_area);
							
						}
					}
			}
			else{
				if(Calculate_coordinate(15,W_Coordinate.Y + Y_error,W_Coordinate.Z + data2angel(Z_error))){
					OUT_servo_motor_angel();
					SendCMD(0x03,SPPED,0x01,one_servo.now_angle,0x02,two_servo.now_angle,0x03,W_servo.now_angle,0x00,bastic_servo.now_angle);
					HAL_Delay(MS);
					printf("Z = %d   Y = %d\r\n",vision.data_X,vision.data_Y);
					//HAL_Delay(200);
				}
			}
		}
 } 
}



//计算跟随所需达到的位置    //传入理想目标值 计算输出经过约束条件的值  有限幅
int Calculate_coordinate(float x,float y,float z) {
	if(x<10)
		x = 10;
	if(x>40)
		x = 40;
	if(y>40)
		y=40;
	if(y<10)
		y=10;
	if (!slove_angle(x, y, z)) {  //不满足约束条件    --大概率是机械臂伸不够长了，考虑用小车移动来靠近
		while (sqrt(x * x + y * y) > (L1 + L2)) {  //当两边之和小于第三边时有问题  优先牺牲前方距离
			x -= 0.1f;//牺牲x距离 
			if (x <= 1.0f) {  //限幅，不能回缩那么短
				break;
			}
		}
		while (sqrt(x * x + y * y) > (L1 + L2)) {  //还是不行，高度限制
			y -= 0.1f;//牺牲y距离 
			if (y <= 5) {  //限幅
				break;
			}
		}
		
		return slove_angle(x, y, z);
	}
	return 1;
}

void Servo_motor_init(volatile Servo_motor* servo_motor, float MAX_angle, float MIN_angle, float zero_angle, float temp_angle, unsigned char dir, float weight) {
	servo_motor->MIN_angle = MIN_angle;
	servo_motor->MAX_angle = MAX_angle;
	servo_motor->zero_angle = zero_angle;
	servo_motor->temp_angle = temp_angle;
	servo_motor->dir = dir;
	servo_motor->weight = weight;
}

//将数值转为角度，如30->30°
float data2angel(float data) {
	return data * PI / 180;

}

//计算目标角度 传入零角度 和坐标系角度 返回舵机转动角度 
float Calculate_angle(float zero_angle,float temp_angle,unsigned char dir){
	if (dir == back) {
		return zero_angle + temp_angle;
	}
	else if(dir == forward) {
		return zero_angle - temp_angle;
	}
	else if (dir == LEFT){
		return zero_angle + temp_angle;
	}
	return 0;
}

//临时算子
float temp_T[4][4];
//矩阵乘法 A*B  输出保存在临时算子上
void matrix_mul(float (* A)[4], float(*B)[4]) {
	for (unsigned char i = 0; i < 4; i++) {
		for (unsigned char j = 0; j < 4; j++) {
			temp_T[i][j] = 0;
			for (unsigned char u = 0; u < 4; u++) {
				temp_T[i][j] += A[i][u] * B[u][j];
			}
		}
	}
}

//正解算  输出位置信息到W_Coordinate
void get_XYZ(void) {
	//定义算子
	float one_T[4][4] = { {cos(one_servo.temp_angle),-sin(one_servo.temp_angle),0,0},
						  {sin(one_servo.temp_angle),cos(one_servo.temp_angle),0,0},
						  {0,0,1,0},
						  {0,0,0,1} };
	float two_T[4][4] = { {cos(two_servo.temp_angle),-sin(two_servo.temp_angle),0,L1},
					      {sin(two_servo.temp_angle),cos(two_servo.temp_angle),0,0},
					      {0,0,1,0},
					      {0,0,0,1} };
	float W_T[4][4] = { {cos(W_servo.temp_angle),-sin(W_servo.temp_angle),0,L2},
					    {sin(W_servo.temp_angle),cos(W_servo.temp_angle),0,0},
					    {0,0,1,0},
					    {0,0,0,1} };
	matrix_mul(one_T, two_T);
	float T[4][4];
	for (unsigned char i = 0; i < 4; i++) {
		for (unsigned char j = 0; j < 4; j++) {
			T[i][j] = temp_T[i][j];
		}
	}
	matrix_mul(T, W_T);
	W_Coordinate.X = temp_T[0][3];
	W_Coordinate.Y = temp_T[1][3];
	W_Coordinate.Z = bastic_servo.temp_angle;
}

//逆解算
//解算出机械臂所需的三个夹角  X为前方距离  Y为高度   Z为左右转动  返回1表示有解  
int slove_angle(float x,float y,float z) {
	float cosA2 = ((x * x) + (y * y) - (L1 * L1) - (L2 * L2))/(2*L1 *L2);
	if ((cosA2 < -1) || (cosA2 > 1)) {
		return 0; //无解
	}
	float sinA2_1 = sqrt(1 - (cosA2 * cosA2));
	float sinA2_2 = -sqrt(1 - (cosA2 * cosA2));
	float angel2_1 = -atan2(sinA2_1, cosA2);
	float angel2_2 = -atan2(sinA2_2, cosA2);


	float B1 = atan2(y, x);
	float cosT = ((x * x) + (y * y) + (L1 * L1) - (L2 * L2)) / (2 * L1 * sqrt((x * x) + (y * y)));
	float sinT = sqrt(1 - (cosT * cosT));
	float T = atan2(sinT, cosT);
	float angel1_1 = B1 + T;
	float angel1_2 = B1 - T;

	float angel3_1 = 0 - angel2_1 - angel1_1;
	float angel3_2 = 0 - angel2_2 - angel1_2;

	//加权计算选择最优解  均方差
	float error_1 = ((one_servo.temp_angle - angel1_1) * (one_servo.temp_angle - angel1_1)) * one_servo.weight
				  + ((two_servo.temp_angle - angel2_1) * (two_servo.temp_angle - angel2_1)) * two_servo.weight
				  + ((W_servo.temp_angle - angel3_1) * (W_servo.temp_angle - angel3_1)) * W_servo.weight;

	float error_2 = ((one_servo.temp_angle - angel1_2) * (one_servo.temp_angle - angel1_2)) * one_servo.weight
		          + ((two_servo.temp_angle - angel2_2) * (two_servo.temp_angle - angel2_2)) * two_servo.weight
		          + ((W_servo.temp_angle - angel3_2) * (W_servo.temp_angle - angel3_2)) * W_servo.weight;

	
	bastic_servo.temp_angle = z;
	if (error_1 < error_2) { 
//		//角度保护 
//		if (angel1_1 > one_servo.MAX_angle || angel1_1 < one_servo.MIN_angle)
//			return 0;
//		if (angel2_1 > two_servo.MAX_angle || angel2_1 < two_servo.MIN_angle)
//			return 0;
//		if (angel3_1 > W_servo.MAX_angle || angel3_1 < W_servo.MIN_angle)
//			return 0;

		one_servo.temp_angle = angel1_1;
		two_servo.temp_angle = angel2_1;
		W_servo.temp_angle = angel3_1;
	}
	else {

//		//角度保护 
//			if (angel1_2 > one_servo.MAX_angle || angel1_2 < one_servo.MIN_angle)
//				return 0;
//		if (angel2_2 > two_servo.MAX_angle || angel2_2 < two_servo.MIN_angle)
//			return 0;
//		if (angel3_2 > W_servo.MAX_angle || angel3_2 < W_servo.MIN_angle)
//			return 0;

		one_servo.temp_angle = angel1_2;
		two_servo.temp_angle = angel2_2;
		W_servo.temp_angle = angel3_2;
	}
	return 1;
}

void OUT_servo_motor_angel(void) {
	one_servo.now_angle = Calculate_angle(one_servo.zero_angle, one_servo.temp_angle, one_servo.dir) *180 /PI;
	two_servo.now_angle = Calculate_angle(two_servo.zero_angle, two_servo.temp_angle, two_servo.dir) *180 /PI;
	W_servo.now_angle = Calculate_angle(W_servo.zero_angle, W_servo.temp_angle, W_servo.dir) *180 /PI;
	bastic_servo.now_angle = Calculate_angle(bastic_servo.zero_angle, bastic_servo.temp_angle,bastic_servo.dir) *180 /PI;
}

void Servo_Init(void) {
	                //舵机句柄    //最大角度     //最小角度       //零角度      //坐标系角度   //角度变大方向 //权重
	Servo_motor_init(&one_servo , data2angel(140) , data2angel(0)   , data2angel(122+90) , data2angel(125)  , forward ,0.6f);
	Servo_motor_init(&two_servo , data2angel(120) , data2angel(-120) , data2angel(145) , data2angel(-125) , forward ,0.3f);
	Servo_motor_init(&W_servo   , data2angel(120) , data2angel(-120) , data2angel(145) , data2angel(0)   , back ,0.1f);
	Servo_motor_init(&bastic_servo ,data2angel(120) , data2angel(-120) , data2angel(190) , data2angel(95)  , LEFT ,0.1f); //左正 右负
}
