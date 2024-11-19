/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ����ʱ������
**********************************************************************************/

#include "Drv_Timer.h"
#include "Drv_led.h"
#include "Drv_led.h"
#include "I2C_Init.h"

#define SYS_TIMx_IRQn TIM7_IRQn
#define SYS_TIMx TIM7
#define SYS_RCC_TIMx RCC_APB1Periph_TIM7


extern u8 start_trig_flag;
extern int cont_t2_echo;
extern int cont_t2_echo_tail;

int dis_tim2_front_cnt=0;
int dis_tim2_tail_cnt=0;


u8 end_dis_front_flag=0;
u8 end_dis_tail_flag=0;

u8 dis_front_flag=0;
u8 dis_tail_flag=0;


void TIM_CONF(u16 period_ms) //APB1  84M
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* ʹ��ʱ�� */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);

    TIM_DeInit(SYS_TIMx);

    /* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    TIM_TimeBaseStructure.TIM_Period = period_ms;

    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    /* ʱ��Ԥ��Ƶ��Ϊ */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���

    TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);

    TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(SYS_TIMx, ENABLE);

    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, DISABLE); /*�ȹرյȴ�ʹ��*/
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = SYS_TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DrvTimerFcInit(void)
{
    TIM_CONF(1000);
    TIM_NVIC();

    /* TIM7 ���¿�ʱ�ӣ���ʼ��ʱ */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);
}




void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
void handle_front_sonix(void)
{
	if(task_pc6==0&&cont_t2_echo==0)
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ;
			return;
		}
		
		if(task_pc6==1)
		{
			cont_t2_echo++;
		}
		
		if(task_pc6==0&&cont_t2_echo!=0)
		{
			dis_tim2_front_cnt=cont_t2_echo;
			end_dis_front_flag=1;
		}
}



void handle_tail_sonix(void)
{
	if(task_pc7==0&&cont_t2_echo_tail==0)
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ;
			return;
		}
		
		if(task_pc7==1)
		{
			cont_t2_echo_tail++;
		}
		
		if(task_pc7==0&&cont_t2_echo_tail!=0)
		{
			dis_tim2_tail_cnt=cont_t2_echo_tail;
			end_dis_tail_flag=1;
		}
}

//��ʱ��3�жϷ�����
void TIM2_IRQHandler(void)
{
	
	handle_front_sonix();
	
	handle_tail_sonix();
	

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
}













/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
