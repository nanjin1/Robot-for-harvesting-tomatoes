#include "servo.h"
#include <math.h>
#include "pid.h"
#include "vision.h"
#include "laser.h"
//ȫ�ֵ�λcm
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
	if(vision.data_X==0||vision.data_Y==0){ //�ҷ���ģʽ
			get_XYZ(); //��õ�ǰλ��	 		
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
			Z_obj.measure = (float)vision.data_X;  //����
			Y_obj.measure = (float)vision.data_Y;  //����
			X_obj.measure = (float)vision.data_area;  //����ֵ
			
			//����
			Z_obj.target = 160.0f;
			Y_obj.target = 160.0f;
			X_obj.target  = 20.0f;
			
			float Z_error = positional_PID(&Z_obj,&Z_param); //angle
			float Y_error = positional_PID(&Y_obj,&Y_param);
			float X_error = positional_PID(&X_obj,&X_param);
			get_XYZ(); //��õ�ǰλ��
			
			
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


			if(fabs(Z_error)+ fabs(Y_error) <10.0f ||vision.data_area>40){  //����Сʱ
					
					X_obj.measure = (float)vision.data_area;  //����ֵ
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



//�����������ﵽ��λ��    //��������Ŀ��ֵ �����������Լ��������ֵ  ���޷�
int Calculate_coordinate(float x,float y,float z) {
	if(x<10)
		x = 10;
	if(x>40)
		x = 40;
	if(y>40)
		y=40;
	if(y<10)
		y=10;
	if (!slove_angle(x, y, z)) {  //������Լ������    --������ǻ�е���첻�����ˣ�������С���ƶ�������
		while (sqrt(x * x + y * y) > (L1 + L2)) {  //������֮��С�ڵ�����ʱ������  ��������ǰ������
			x -= 0.1f;//����x���� 
			if (x <= 1.0f) {  //�޷������ܻ�����ô��
				break;
			}
		}
		while (sqrt(x * x + y * y) > (L1 + L2)) {  //���ǲ��У��߶�����
			y -= 0.1f;//����y���� 
			if (y <= 5) {  //�޷�
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

//����ֵתΪ�Ƕȣ���30->30��
float data2angel(float data) {
	return data * PI / 180;

}

//����Ŀ��Ƕ� ������Ƕ� ������ϵ�Ƕ� ���ض��ת���Ƕ� 
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

//��ʱ����
float temp_T[4][4];
//����˷� A*B  �����������ʱ������
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

//������  ���λ����Ϣ��W_Coordinate
void get_XYZ(void) {
	//��������
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

//�����
//�������е������������н�  XΪǰ������  YΪ�߶�   ZΪ����ת��  ����1��ʾ�н�  
int slove_angle(float x,float y,float z) {
	float cosA2 = ((x * x) + (y * y) - (L1 * L1) - (L2 * L2))/(2*L1 *L2);
	if ((cosA2 < -1) || (cosA2 > 1)) {
		return 0; //�޽�
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

	//��Ȩ����ѡ�����Ž�  ������
	float error_1 = ((one_servo.temp_angle - angel1_1) * (one_servo.temp_angle - angel1_1)) * one_servo.weight
				  + ((two_servo.temp_angle - angel2_1) * (two_servo.temp_angle - angel2_1)) * two_servo.weight
				  + ((W_servo.temp_angle - angel3_1) * (W_servo.temp_angle - angel3_1)) * W_servo.weight;

	float error_2 = ((one_servo.temp_angle - angel1_2) * (one_servo.temp_angle - angel1_2)) * one_servo.weight
		          + ((two_servo.temp_angle - angel2_2) * (two_servo.temp_angle - angel2_2)) * two_servo.weight
		          + ((W_servo.temp_angle - angel3_2) * (W_servo.temp_angle - angel3_2)) * W_servo.weight;

	
	bastic_servo.temp_angle = z;
	if (error_1 < error_2) { 
//		//�Ƕȱ��� 
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

//		//�Ƕȱ��� 
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
	                //������    //���Ƕ�     //��С�Ƕ�       //��Ƕ�      //����ϵ�Ƕ�   //�Ƕȱ���� //Ȩ��
	Servo_motor_init(&one_servo , data2angel(140) , data2angel(0)   , data2angel(122+90) , data2angel(125)  , forward ,0.6f);
	Servo_motor_init(&two_servo , data2angel(120) , data2angel(-120) , data2angel(145) , data2angel(-125) , forward ,0.3f);
	Servo_motor_init(&W_servo   , data2angel(120) , data2angel(-120) , data2angel(145) , data2angel(0)   , back ,0.1f);
	Servo_motor_init(&bastic_servo ,data2angel(120) , data2angel(-120) , data2angel(190) , data2angel(95)  , LEFT ,0.1f); //���� �Ҹ�
}
