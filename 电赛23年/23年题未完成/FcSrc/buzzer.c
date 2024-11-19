#include "buzzer.h"

int beep_flag;

void beep_init()
{
	  GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ�� GPIOA ʱ��
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;						//����GPIO��A9
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//����GPIO��Ϊ�����������
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//����GPIO���ٶ�50Mhz
		GPIO_Init(GPIOE,&GPIO_InitStructure);					//��ʼ��GPIOB

		GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);	
}

void beep_bsp(unsigned int flag)
{
	if(flag == 1)GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
	if(flag == 0)GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);	
}

void beep_delay()
{
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);	
	MyDelayMs(300);
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
}	
