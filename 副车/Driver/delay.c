#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "Start_task.h"
#define detect_dog 3
//��ʱ����+�����
TIM_HandleTypeDef TIM6_Handler;
struct TIM tim = {0,0,0};
void delay_init(void)
{
    //��ʱ��7
    __HAL_RCC_TIM6_CLK_ENABLE();
     
    TIM6_Handler.Instance=TIM6;                          //ͨ�ö�ʱ��7
    TIM6_Handler.Init.Prescaler=72-1;                     //��Ƶ
    TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM6_Handler.Init.Period=1500-1;                        //�Զ�װ��ֵ
    TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM6_Handler);
	  HAL_NVIC_SetPriority(TIM6_IRQn,9,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM6_IRQn);          //����ITM4�ж�  
    HAL_TIM_Base_Start_IT(&TIM6_Handler); //ʹ�ܶ�ʱ��7�Ͷ�ʱ��7�ж� 
}

//1.5ms  
void TIM6_IRQHandler(void)
{
		static uint8_t mouse = 0;    //С����
    if(__HAL_TIM_GET_IT_SOURCE(&TIM6_Handler,TIM_IT_UPDATE)==SET)//����ж�
    {
					tim.time++;
					if(tim.time>60000){
						tim.times++;
						tim.time = 0;
					}
					tim.sys_times = tim.time + 60000*tim.times;
					for(uint8_t i=0;i<4;i++){
						dog[i]++;   //�����
						if(dog[i]>=detect_dog)
							high_time[i]=0;
						if(dog[i]>=255){
									dog[i]=0;   //�����
						}
					}					
					mouse++;
					if(mouse>100){
						mouse =0;
						LED_twinkle();
					}
		}
    __HAL_TIM_CLEAR_IT(&TIM6_Handler, TIM_IT_UPDATE);//����жϱ�־λ
}

//�����ʱ1500us
void delay_us(uint16_t nus)
{
   TIM6->CNT = 0;
	 while(TIM6->CNT<nus);
}

void delay_ms(uint16_t nms)
{
	for(uint16_t i=0;i<nms;i++){
		delay_us(1000);
	}
}
