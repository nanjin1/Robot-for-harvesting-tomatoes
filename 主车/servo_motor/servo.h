#ifndef __Servo_H
#define __Servo_H

void Servo_Init(void);
void OUT_servo_motor_angel(void);
int slove_angle(float x,float y,float z);
void get_XYZ(void) ;
int Calculate_coordinate(float x,float y,float z);

//�Ƕȵ�λ��Ϊ��ֵ ��180��Ϊ3.1415926
typedef struct servo_motor { 
	float MIN_angle;//���Ƕ�  -�����������
	float MAX_angle;//��С�Ƕ�  -�����������
	float zero_angle;//��Ƕ�   ��һ�����ת���������ƽ�еĽǶ�  ���������Ŷ��Ϊֱ�ߵĽǶ�
	float temp_angle;//����ϵ�Ƕ�  
	unsigned char dir;//����Ƕȱ����  
	float now_angle;//Ŀǰ�Ƕ�  -ת��Ϊ���ת������ʵ�Ƕ�   
	float weight;  //��Ȩ  Ȩ��Խ����ζ��Խ���붯  
}Servo_motor;

typedef struct coordinate {
	float X;  //ǰ������
	float Y;  //�߶�
	float Z;  //����

}Coordinate;
float data2angel(float data);
volatile extern Servo_motor bastic_servo, one_servo , two_servo,W_servo;
volatile extern Coordinate  W_Coordinate;
void servo_control(void);
#endif
