/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：定时器驱动
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

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);

    TIM_DeInit(SYS_TIMx);

    /* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period = period_ms;

    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 时钟预分频数为 */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数

    TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);

    TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(SYS_TIMx, ENABLE);

    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, DISABLE); /*先关闭等待使用*/
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

    /* TIM7 重新开时钟，开始计时 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx, ENABLE);
}




void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
void handle_front_sonix(void)
{
	if(task_pc6==0&&cont_t2_echo==0)
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位;
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
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位;
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

//定时器3中断服务函数
void TIM2_IRQHandler(void)
{
	
	handle_front_sonix();
	
	handle_tail_sonix();
	

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}













/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
