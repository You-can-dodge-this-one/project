/*
  ******************************************************************************
  * @file    stm32v3.5_RobotPellatizer/user/main.c 
  * @author  shu@1154890719.com
  * @version V3.5.0
  * @date    2018.9.25
  * @brief   Main program body
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "tcs.h"
#include "timer.h"
#include"stdbool.h"
#include "delay.h"

#define QTI1_Pin   GPIO_Pin_0
#define QTI2_Pin   GPIO_Pin_1
#define QTI3_Pin   GPIO_Pin_2
#define QTI4_Pin   GPIO_Pin_3
#define QTI15_Pin  GPIO_Pin_15



#define leftservo    GPIO_Pin_6
#define rightservo   GPIO_Pin_7

/* -------------------------��ɫ���������ŵ�ƽ���ƶ���--------------------------------------*/
#define S0_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_4) //д1
#define S0_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_4) //д0
#define S1_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_5) 
#define S1_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define S2_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define S2_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_6)
#define S3_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_8)
#define S3_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_8)
#define LED_Write_1()    GPIO_SetBits(GPIOE,GPIO_Pin_10)
#define LED_Write_0()    GPIO_ResetBits(GPIOE,GPIO_Pin_10)
//OUT �ӵ�PD2


 /*********��ȡQTI�ĵ�ƽ�궨��*******/
#define PE0_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)//��PE0�ϵ�ֵ
#define PE1_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)//��PE1�ϵ�ֵ
#define PE2_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//��PE2�ϵ�ֵ
#define PE3_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��PE3�ϵ�ֵ

#define Yellow 3
#define White  1
#define Red    4
#define Black  2
#define Blue   5


extern int count_tcs230;


/************************************��������************************/
void BlueCarry(void);              //����ɫɫ�鵽������
void BlackCarry(void);             //���ɫɫ�鵽������
void YellowCarry(void);            //���ɫɫ�鵽������
void WhiteCarry(void);             //���ɫɫ�鵽������
void RedCarry(void);                //���ɫɫ�鵽������ 

/*---------------------------------�ṹ�������궨�塢��������������------------------------------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
GPIO_InitTypeDef GPIO_InitStructure; 
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef USART_ClockInitStructure;

volatile u16 times;
volatile unsigned int time;		

char process=0;
char Ahavesome = 1;//A����ɫ���־   //����ʱ�����ֱ�ӽ��鵽��ı�־ȫ����1
char Bhavesome = 1;//B����ɫ���־
char Chavesome = 1;//C����ɫ���־
char Dhavesome = 1;//D����ɫ���־
char Ehavesome = 1;//E����ɫ���־
char Fhavesome = 0;//F����ɫ���־
char Ghavesome = 0;//G����ɫ���־
char Hhavesome = 0;//H����ɫ���־
char Ihavesome = 0;//I����ɫ���־

char pointsth=0;//��ֱ���Ϸŵڼ���ɫ��
char findsekuai = 0;// �Ѿ�����ɫ����ܸ���

//char pointsth=3;//��ֱ���Ϸŵڼ���ɫ��
//char findsekuai = 3;// �Ѿ�����ɫ����ܸ���




char Idone=0;
char sekuai = 5;//ɫ����ܸ��� 
u16 pcolor[3]={0,0,0};//��ɫ�������ı������� 
u16 RGB[3]={0,0,0};
int hadedone = 1;
unsigned char QTIS;//ѭ�ߵ�QTI״̬

unsigned int color =0;//��ɫ����ֵ

unsigned int distance=0;//���������

unsigned int speed=0;//����ٶ�
unsigned int pulses=0;//������
bool colorCheck = true;
volatile char i=0,j=0;
unsigned int i1=0;
unsigned int goal=38;	  //���˵�Ŀ���Ʒ����Ĳ���   //��ֵԽ�����Լ��
unsigned int a=0;        //�����жϰ�fg  ��hi��ɫ��  ��Ҫ�����ķ���
int count = 1;                //��ɫʶ���������
int LRp = 12;
int aim = 7;
bool have=true;	
/*************************************
  * @brief  Configure peripheral RCC.
  * @param  None
  * @retval None
  ************************************/
void RCC_Configuration(void)
{
	/* ������RCC�Ĵ�������ΪĬ��ֵ�����йؼĴ�����λ�����ú������ĶRCC_CR��HSITRIM[4:0]λ��Ҳ�����üĴ���RCC_BDCR�ͼĴ���RCC_CSR */
	RCC_DeInit();
	/* ʹ���ⲿHSE���پ��� */
	RCC_HSEConfig(RCC_HSE_ON);
	/* �ȴ�HSE���پ����ȶ��������ڳ�ʱ��������˳� */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	/* SUCCESS:HSE�����ȶ��Ҿ�����ERROR��HSE����δ���� */
	if(HSEStartUpStatus == SUCCESS)
	{
		/* ʹ��flashԤȡָ��������������RCCûֱ�ӹ�ϵ */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		/* ����FLASH�洢����ʱʱ����������2����Ը�Ƶʱ�ӵģ�
		FLASH_Latency_0��0��ʱ���ڣ�FLASH_Latency_1��1��ʱ���� 
		FLASH_Latency_2��2��ʱ���� */
		FLASH_SetLatency(FLASH_Latency_2);
		/* HCLK=SYSCLK ���ø�������ʱ��=ϵͳʱ�� */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK1=HCLK/2 ���õ�������1ʱ��=����ʱ�ӵĶ���Ƶ*/
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/* PCLK2=HCLK ���õ�������2ʱ��=��������ʱ�� */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* Set PLL clock output to 72MHz using HSE (8MHz) as entry clock */
		/* �������໷��HSE�ⲿ8MHz����9��Ƶ��72MHz */ 
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		/* Enable PLL��ʹ��PLL���໷ */
		RCC_PLLCmd(ENABLE);
		
		 /* Wait till PLL is ready���ȴ����໷����ȶ� */
     /* RCC_FLAG_HSIRDY��HSI���������RCC_FLAG_HSERDY��HSE�������
        RCC_FLAG_PLLRDY��PLL������RCC_FLAG_LSERDY��LSE�������
        RCC_FLAG_LSIRDY��LSI���������RCC_FLAG_PINRST�����Ÿ�λ
        RCC_FLAG_PORRST��POR/PDR��λ��RCC_FLAG_SFTRST�������λ
        RCC_FLAG_IWDGRST��IWDG��λ��RCC_FLAG_WWDGRST��WWDG��λ
        RCC_FLAG_LPWRRST���͹��ĸ�λ */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	;
		
		 /* Select PLL as system clock source�������໷�������Ϊϵͳʱ�� */
     /* RCC_SYSCLKSource_HSI��ѡ��HSI��Ϊϵͳʱ��
        RCC_SYSCLKSource_HSE��ѡ��HSE��Ϊϵͳʱ��
        RCC_SYSCLKSource_PLLCLK��ѡ��PLL��Ϊϵͳʱ��*/
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		 /* �ȴ�PLL��Ϊϵͳʱ�ӱ�־λ��λ */
     /* 0x00��HSI��Ϊϵͳʱ�ӣ�0x04��HSE��Ϊϵͳʱ��
        0x08��PLL��Ϊϵͳʱ�� */
		while(RCC_GetSYSCLKSource() != 0x08);	
	}
	
	 /* Enable GPIOA~E and AFIO clocks��ʹ����Χ�˿�����ʱ�ӡ�ע�������������������ͬоƬ�Ϳ�����ķ��䲻ͬ*/
 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
													RCC_APB2Periph_GPIOB|
													RCC_APB2Periph_GPIOC|
													RCC_APB2Periph_GPIOD|
													RCC_APB2Periph_GPIOE|
													RCC_APB2Periph_AFIO, ENABLE);
//   /* USART1 clock enable��USART1ʱ��ʹ�� */
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//	 /* TIM1 clock enable��TIM1ʱ��ʹ�� */
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//   /* TIM2 clock enable��TIM2ʱ��ʹ��*/
//	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	 /* ADC1 clock enable��ADC1ʱ��ʹ��*/
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	
		/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 ,ENABLE);
	/* TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	/* Enable USART1 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	

}


/************************************************
  * @brief  Configure NVIC.
  * @param  None
  * @retval None
  ***********************************************/
void NVIC_Configuration(void)
{
	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* Configure the Priority Grouping with 0 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Configure the Priority Grouping with 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	
	/* Enable TIM2 global interrupt with Preemption Priority 1 and Sub
	Priority as 5 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the Priority Grouping with 2 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* Enable TIM3 global interrupt with Preemption Priority 1 and Sub
	Priority as 6 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Configure the Priority Grouping with 2 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);	

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 //������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/************************************************
  * @brief  Configure the TIM2 Timer for 1ms.
  * @param  None
  * @retval None
  ***********************************************/
void TIM2_Configure(void)
{
   TIM_DeInit( TIM2);//��λTIM2��ʱ��
   TIM_TimeBaseStructure.TIM_Period = 99;
   TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
   /* Clear TIM2 update pending flag[���TIM2����жϱ�־] */
   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   /* Enable TIM2 Update interrupt [TIM2����ж�����]*/
   TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  
   /* TIM2 enable counter [����tim2����]*/
   TIM_Cmd(TIM2, DISABLE);
}
/************************************************
  * @brief  Configure the TIM3 Counter.
  * @param  None
  * @retval None
  ***********************************************/
void TIM3_Counter_Configure(void)
{
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;//�����Զ�װ�ؼĴ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;//��Ƶ����
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//ѡ�����ϼ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);// Time base configuration

	TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

	TIM_SetCounter(TIM3, 0);//���ü�������ֵ
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//ʹ�ܶ�ʱ���ж�
	TIM_Cmd(TIM3, DISABLE); //ʹ�ܶ�ʱ��
}
void TIM4_Configuration(void)
{ 
  TIM_DeInit(TIM4);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);   
  TIM_PrescalerConfig(TIM4,71, TIM_PSCReloadMode_Immediate);   
   /* enable preload value */ 
  TIM_ARRPreloadConfig(TIM4,DISABLE);  
  TIM_SetCounter(TIM4,0);			 //TIM4����ֵ����
  TIM_Cmd(TIM4,DISABLE); 			 //TIM4������ʧ��
}
  
void TIM5_Configure(void)
{
   TIM_DeInit(TIM5);//��λTIM5��ʱ��
	
   TIM_TimeBaseStructure.TIM_Period = 9;
   TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
	 
	
	
	
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM5, & TIM_TimeBaseStructure);
   /* Clear TIM2 update pending flag[���TIM2����жϱ�־] */
   TIM_ClearFlag(TIM5, TIM_FLAG_Update);
   /* Enable TIM2 Update interrupt [TIM2����ж�����]*/
   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  
   /* TIM2 enable counter [����tim2����]*/
   TIM_Cmd(TIM5, DISABLE);
}
/************************************************
  * @brief  Configure USART1.
  * @param  None
  * @retval None
  ***********************************************/
void USART1_Configuration(void)
{
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	/* Enables the USART1 transmit interrupt */
	USART_ClearFlag(USART1, USART_FLAG_TC);  
	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
}

/************************************************
  * @brief  Configure the TCS230 Pins.
  * @param  None
  * @retval None
  ***********************************************/
void GPIO_TCS230_Configure(void)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOE, &GPIO_InitStructure); 	
	
	

}
/***********************************************
  * @brief  Configure USART1 Pins.
  * @param  None
  * @retval None
  ***********************************************/
void GPIO_USART1_Configure(void)
{
	/*USART1_TX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*USART1_RX*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}
/************************************************
  * @brief  Configure the TIM3 Pins.
  * @param  None
  * @retval None
  ***********************************************/
void GPIO_TIM3_Configure(void)
{
	//����TIM3���ⲿ������ PD2  ETR	 �����ŵ����ø�Ӳ���йأ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void GPIO_Dist_Config(void)
{			 	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void GPIO_Motor_Config(void)
{
  /* Configure Motors IO*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void GPIO_QTI_Config(void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //��������
  GPIO_Init(GPIOE,&GPIO_InitStructure);
}

u8 QTI_State(u8 pin)//��ȡ����ֵ
{
	return GPIO_ReadInputDataBit(GPIOE,pin); 	
}


/**************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
**************************************************/
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	USART1->SR;//�����٣���������״η������ַ���ʧ
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (u8) ch);
	/* Loop until the end of transmission */
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  // waiting here
	
	return ch;
}

void delay_nus(unsigned int n)  //��ʱn us: n>=6,��С��ʱ��λ6us
{ 
  unsigned int j;
  while(n--)              // �ⲿ����8M��PLL��9��8M*9=72MHz
  {
    j=8;				  // ΢����������֤��ʱ�ľ���
	while(j--);
  }
}

void delay_nms(unsigned int n)  //��ʱn ms
{
  while(n--)		   // �ⲿ����8M��PLL��9��8M*9=72MHz
    delay_nus(1100);   // 1ms��ʱ����
}
void motor_motion1(unsigned int left_val, unsigned int right_val, unsigned int count)     //��     ��     �ƴ�
{
   unsigned int i; 
   for(i=0; i<count; i++)
   {
			GPIO_SetBits(GPIOC, GPIO_Pin_7);
			delay_nus(left_val);
			GPIO_ResetBits(GPIOC,GPIO_Pin_7);

			GPIO_SetBits(GPIOC, GPIO_Pin_6);
			delay_nus(right_val);
			GPIO_ResetBits(GPIOC,GPIO_Pin_6);

			delay_nms(20);
   } 							 
}
/*******************************************************************************
  *TCS230_CurrentColor(u16 pRGB[3],u16 rgb[3])
  *  ��ȡ��ɫɫ��RGBֵ���ֱ��������RGB[3]={0,0,0}��
  ********************************************************************************/
u8 TCS230_CurrentColor(u16 pRGB[3])
{
	u8 currentcolor=0;
	S0_Write_0();	S1_Write_1();           //���Ƶ��Ϊ(1/50)*500KHz=10KHz	
	LED_Write_1();          //��LED
	
	S2_Write_0();	S3_Write_0();           //ѡ���ɫ
	
	times=0;
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	while(pRGB[0] != times);
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	RGB[0] = TIM_GetCounter(TIM3);
	
	S3_Write_1();//ѡ����ɫ
	
	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(pRGB[1] != times);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	RGB[1] = TIM_GetCounter(TIM3);	
	
	S2_Write_1();//ѡ����ɫ
	
	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(pRGB[2] != times);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	RGB[2] = TIM_GetCounter(TIM3);
	
	LED_Write_0();//�ر�LED	
	
	printf("Red: %d  Blue: %d  Green: %d\n",RGB[0],RGB[1],RGB[2]);
	return currentcolor;
}
/*******************************************************************************
  * @brief  TCS230_WhiteBalance Function.
  *  Description�����ɫɫ��RGBֵ����255ʱ����Ӧʱ�����ֱ�洢������pColor[]��
  ********************************************************************************/
void TCS230_WhiteBalance(u8 pColor[3])                   //��ƽ���ʼ��
{
	S0_Write_0();	
	S1_Write_1();//���Ƶ��Ϊ(1/50)*500KHz=10KHz
	LED_Write_1();//��LED
	
	S2_Write_0();	
	S3_Write_0();//ѡ���ɫ

	times=0;TIM_SetCounter(TIM3,0);//������3���
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[0] = times;//ʱ���������	
	
	S3_Write_1();//ѡ����ɫ

	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[1] = times;//ʱ���������	

	S2_Write_1();//ѡ����ɫ

	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[2] = times;//ʱ���������	
	
	LED_Write_0();//�ر�LED	
	
	printf("Red**: %d  Blue**: %d  Green**: %d\n",pColor[0],pColor[1],pColor[2]);
}
/*******************************************************************************
  * @brief  Robot_checkColor(void)
  *  ��ȡ��ɫɫ��RGBֵ�������ݶ�ȡ��RGBֵת������ɫ
  ********************************************************************************/
 unsigned int Robot_checkColor(void)             
 {
 	  TCS230_CurrentColor(pcolor);                      //��ȡ��ɫɫ���RGBֵ
	  printf("Red: %d  Blue: %d  Green: %d\n",RGB[0],RGB[1],RGB[2]);
		//�ж���ɫ                                             //ͨ��RGBֵ�ж���ɫ
	 		if((RGB[0] > 90)&&((RGB[0] - RGB[2]) > 60) && ((RGB[0] - RGB[1]) > 60)&&((RGB[2] - RGB[1]) < 80))
		{
			return 3;//��ɫ
		}
		else if(((abs(RGB[0] - RGB[2])<50)&&(abs(RGB[0] - RGB[1])<50)&& (RGB[0]>100) ) ||(  RGB[1]-RGB[2]<15 && RGB[0]>200 && (RGB[1]<80)) || ( (RGB[0]>200)&& (RGB[2]>200)&&(RGB[1]<15)      )  )
		{
			return 2;//��ɫ
		}
		else if((abs(RGB[0] - RGB[2])<30)&&(abs(RGB[0] - RGB[1])<30)&& (RGB[0]<40)&& (RGB[1]<40)&& (RGB[2]<40) )
		{
			return 4;//��ɫ
		}
		else if(	((RGB[0] < RGB[1]) && (RGB[2] < RGB[1])&&((RGB[1] - RGB[0])>10)) || ((RGB[0] < RGB[1]) && (RGB[2] < RGB[1])&&((RGB[1] - RGB[0])>5))	)
		{
			return 5;//��ɫ
		}
		else if( (((RGB[2] - RGB[1]) > 80) && ((RGB[1] - RGB[0]) > 80)&&(RGB[2] > 90)) || (((RGB[2] - RGB[1]) > 50) && ((RGB[0]-RGB[1]) > 65) && RGB[1]>80   ) )
		{
			return 1;//��ɫ
		}
		return 0;
		
		
 }
int GetDis(int echo,int trig)     //��ȡ����������������ֵ 
{
  int dis;
	int count;
	GPIO_ResetBits(GPIOC,echo);			//echo�˿ڸ�λ
	GPIO_ResetBits(GPIOC,trig);			//trig�˿ڸ�λ
  TIM_SetCounter(TIM4,0);			 //TIM4����ֵ����
	GPIO_SetBits(GPIOC,trig);		          //trig�ø� ����10us�ĸߵ�ƽ�ź� 
	delay_nus(10); 
	GPIO_ResetBits(GPIOC,trig);
	delay_nus(100);  		  
	while(GPIO_ReadInputDataBit(GPIOC, echo) == 0);
	TIM_Cmd(TIM4,ENABLE);    //����������
       //������ʱ����ʼ��ʱ
	while(GPIO_ReadInputDataBit(GPIOC, echo));   //�ȴ�echo�õ�
	TIM_Cmd(TIM4,DISABLE);   //�رռ�����

	count = TIM_GetCounter(TIM4);//��ȡ������ֵ
	dis = (int)count/60.034;//ת��Ϊ����,��29.034us�������ܴ���1cm
	return dis;
}

/*
 * //����м���QTI�Ƿ��ں�����
 */

bool IsMLeftQtiBlack(void)   //����м���QTI�Ƿ��ں�����
{
	if(QTI_State(QTI1_Pin)==true)    //�ж��м���QTI�Ƿ��⵽����
	{                                //�ں��߷���ture    ���ں��߷���false  
		delay_nms(2);
		if(QTI_State(QTI1_Pin)==true)
		{
			return true;   
		}
	  else
		{
			return false;   
		}
	}
	else
	{
		return false;
	}
}
/*
 * //����м���QTI�Ƿ��ں�����
 */
bool IsMRightQtiBlack(void)  //����м���QTI�Ƿ��ں�����
{
	if(QTI_State(QTI4_Pin)==true)   //�ж��м���QTI�Ƿ��⵽����
	{                               //�ں��߷���ture    ���ں��߷���false  
		delay_nms(2);
		delay_nms(2);
		if(QTI_State(QTI4_Pin)==true)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

void PulseOut(uint8_t pin,int speed)    //������ת����
{
	GPIO_SetBits(GPIOC,pin );
	delay_nus(speed);
	GPIO_ResetBits(GPIOC,pin);
}

void stop(void)//ֹͣ
{
	int i;
	for(i=1;i<=2;i++)
	{
    	GPIO_SetBits(GPIOC, GPIO_Pin_6);
    	delay_nus(1500);
    	GPIO_ResetBits(GPIOC,GPIO_Pin_6);
    	
		GPIO_SetBits(GPIOC, GPIO_Pin_7);
		delay_nus(1500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_7);

		delay_nms(20);
	}	
}

void SpinLeft(void)// ����ת
{
	PulseOut(leftservo,1480);
	PulseOut(rightservo,1480);
	delay_nms(20);
}

void SpinRight(void)// ����ת
{
	PulseOut(leftservo,1520);
	PulseOut(rightservo,1520);
	delay_nms(20);
}

void TurnLeftAnyPulse(int pulses)//����תĳ������     pluses������ת�Ƕ�
{
	while(pulses--)
	{
		SpinLeft();      //����ת
		delay_nms(2);
	}
}
void TurnRightAnyPulse(int pulses) //����תĳ������   pluses������ת�Ƕ�
{
	while(pulses--)
	{
		SpinRight();
		delay_nms(2);
	}
}


void findLline(void)  //�������һ����
{ 
  	TurnLeftAnyPulse(8);
	while(1)
	{
		SpinLeft();                        //����ת
		if(IsMLeftQtiBlack())              //  �м����Ƿ��⵽����
		{
			i1++;
			if(i1==2)
			{
				i1=0;
				break;
			}
		}
	}
	while(1)
	{
		SpinLeft();          
		if(!IsMLeftQtiBlack())
		{
			i1++;
			if(i1==2)
			{
				i1=0;
				break;
			}
		}
	}
	stop();
}
//void findL1line(void)  //���ڻ�ɫ
//{ 
//  	TurnLeftAnyPulse(8);
//	while(1)
//	{
//		SpinLeft();                        //����ת
//		if(IsMLeftQtiBlack())              //  �м����Ƿ��⵽����
//		{
//			i1++;
//			if(i1==1)
//			{
//				i1=0;
//				break;
//			}
//		}
//	}
//	while(1)
//	{
//		SpinLeft();          
//		if(!IsMLeftQtiBlack())
//		{
//			i1++;
//			if(i1==2)
//			{
//				i1=0;
//				break;
//			}
//		}
//	}
//	stop();
//}
void findRline(void) //���ұ���һ����
{
	TurnRightAnyPulse(10);
	while(1)
	{
		SpinRight();
		if(IsMRightQtiBlack())
		{
			i1++;
			if(i1==2)
			{ 	
				i1=0;
				break;
			}
		}
	}
//	stop();
	while(1)
	{
		SpinRight();
		if(!IsMRightQtiBlack())
		{
			i1++;
			if(i1==2)
			{
				i1=0;
				break;
			}
		}
	}
//	stop();
}

/*
//�������ƣ�PE0_state()
//���ܣ� �����ߵ�һ��QTI�ķ����ź�
//�������޲���
//����ֵ��1���ߵ�ƽ���������ߣ�0���͵�ƽ����������
*/

int PE0_state(void)
{	
 	return PE0_ReadBit();
}

/*
�������ƣ�PE1_state()
���ܣ� �����ߵڶ���QTI�ķ����ź�
�������޲���
����ֵ��1���ߵ�ƽ���������ߣ�0���͵�ƽ����������
*/
int PE1_state(void)
{
	return PE1_ReadBit();
}

/*
�������ƣ�PE2_state()
���ܣ� ����ұߵڶ���QTI�ķ����ź�
�������޲���
����ֵ��1���ߵ�ƽ���������ߣ�0���͵�ƽ����������
*/
int PE2_state(void)
{
	return PE2_ReadBit();
}

/*
�������ƣ�PE3_state()
���ܣ� ����ұߵ�һ��QTI�ķ����ź�
�������޲���
����ֵ��1���ߵ�ƽ���������ߣ�0���͵�ƽ����������
*/
int PE3_state(void)
{
	return PE3_ReadBit();
}
void motor_motion2(unsigned int left2_val, unsigned int right2_val)    
{
	GPIO_SetBits(GPIOC, GPIO_Pin_7);
	delay_nus(left2_val);
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);

	GPIO_SetBits(GPIOC, GPIO_Pin_6);
	delay_nus(right2_val);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);

	delay_nms(20);	//���һ��������PWM����
}
void Robot_hunting1(unsigned int speed)             
{
	 QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	 
	 switch (QTIS)
		{
		 //case 0:motor_motion1(speed, (3000-speed)-35-10,3);break;
			case 1:motor_motion1(speed, (3000-speed)-35-10,3);break;
			case 2:motor_motion1(speed, (3000-speed)-35,1);break;
			case 3:motor_motion1(speed, (3000-speed)-35,1);break;
			case 4:motor_motion1(speed+35, (3000-speed),1);break;
		
			case 8:motor_motion1(speed+35+10, (3000-speed),3);break;
			case 12:motor_motion1(speed+35, (3000-speed),2);break;
		
			case 6:motor_motion2(speed+100, 3000-speed-100-2);break;	
			default:motor_motion2(1550, 1450);break;	  
		}
}
void Robot_hunting2(unsigned int speed)              
{                                                   
	 QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);       
	 
	 switch (QTIS)
		{
			case 1:motor_motion1(1500, 1400,2);break;
			case 2:motor_motion1(1500, 1400,1);break;
			case 3:motor_motion1(1500, 1400,1);break;

			case 4:motor_motion1(1600, 1500,1);break;
			case 8:motor_motion1(1600, 1500,2);break;
			case 12:motor_motion1(1600, 1500,1);break;
			case 6:motor_motion2(speed, 3000-speed);break;	 
			default:motor_motion2(speed, 3000-speed);break;	   
		}
}
void Robot_hunting(unsigned int speed)              
{
	 QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	 
	 switch (QTIS)
		{
			case 1:motor_motion1(speed, (3000-speed)-40-10,2);break;//��ת
			case 2:motor_motion1(speed, (3000-speed)-40,1);break;//��ת
			case 3:motor_motion1(speed, (3000-speed)-40,1);break;//��ת

			case 4:motor_motion1(speed+40, (3000-speed),1);break;//��ת
			case 8:motor_motion1(speed+40+10, (3000-speed),2);break;//��ת
			case 12:motor_motion1(speed+40, (3000-speed),1);break;//��ת
			
			case 6:motor_motion2(speed, 3000-speed-2);break;	  //ֱ��
			default:motor_motion2(speed, 3000-speed-2);break;	   //��һ�䲻��ʡ
		}
}
void Peripheral_Init(void)    //��ģ���ʼ�����ú���
{
		RCC_Configuration();     //ʱ�ӡ�����ʹ��
		NVIC_Configuration();    //�ж����ȼ�����
		GPIO_USART1_Configure(); //����1 IO������
		USART1_Configuration();	 //����1����
		GPIO_TCS230_Configure();  //��ɫ������IO������
		TIM2_Configure();        // ��ʱ��2����
		GPIO_TIM3_Configure(); //��ʱ��3�ⲿ������������
		TIM3_Counter_Configure();//	��ʱ��3����������
		TIM5_Configure();	  //   ��ʱ��5����
		GPIO_Motor_Config();  //���ֶ��IO��������
		GPIO_QTI_Config();	  //QTI������������
		TIM4_Configuration(); //��ʱ��4����
		GPIO_Dist_Config();	//
}
/*ת����ƺ���*/
void TurnR_toBlackLine(void)                  //��תѰ����һ������
{	 
		motor_motion1(1515, 1515,5);              //��ת
		do
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);   
				motor_motion1(1515, 1515,1);	     
			} while(QTIS!=6);                       //���ѭ���Ƿ�Ϊ1001  ����ֹͣ��ת
		do
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);  
				//motor_motion1(1515, 1515,1);	
			} while(QTIS!=6);                     //�ٴμ��ѭ���Ƿ�Ϊ1001   
		motor_motion1(1515, 1515,1);
}

void TurnL_toBlackLine(void)                   //��תѰ����һ������
{	
		motor_motion1(1485, 1485,10);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1485, 1485,1);	
			} while(QTIS!=6);                    //���ѭ���Ƿ�Ϊ1001  ����ֹͣ��ת
		delay_nms(5);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				//motor_motion1(1485, 1485,1);	
			} while(QTIS!=6);                   //�ٴμ��ѭ���Ƿ�Ϊ1001 
		motor_motion1(1485, 1485,1);           //��ƫһ��
}
void Turn1_180(void)                    //   ������ת  //������ƫ��һ���Ƕȣ�Ȼ��������תѰ����һ������
{	 
		motor_motion1(1350, 1350,15	);       //��ƫһ���Ƕ�
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1400, 1400,1);	
			} while(QTIS!=6);                  //����תѰ�Һ���   Ҳ���Ǽ��ѭ���Ƿ�Ϊ1001
		delay_nms(5);
		 while(QTIS!=6)
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1480, 1480,1);	
			}                   //�ٴμ���Ƿ��Ѿ��ҵ���һ������
}
void Turn_180(void)                    //   ������ת  //������ƫ��һ���Ƕȣ�Ȼ��������תѰ����һ������
{	 
		motor_motion1(1465, 1465,32	);       //��ƫһ���Ƕ�
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			} while(QTIS!=6);                  //����תѰ�Һ���   Ҳ���Ǽ��ѭ���Ƿ�Ϊ1001				0110
			motor_motion2(1500, 1500);
//		motor_motion1(1475, 1475,1);
//		delay_nms(5);
//		do{
//				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
//				motor_motion1(1465, 1465,1);	
//			}while(QTIS!=6);                  //�ٴμ���Ƿ��Ѿ��ҵ���һ������
}



void Turn_180_four(void)                    //   ������ת  //������ƫ��һ���Ƕȣ�Ȼ��������תѰ����һ������
{	 
		motor_motion1(1465, 1465,35	);       //��ƫһ���Ƕ�
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			} while((QTIS!=6)&&(QTIS!=0x0f));//while(QTIS!=6);                  //����תѰ�Һ���   Ҳ���Ǽ��ѭ���Ƿ�Ϊ1001
		motor_motion1(1475, 1475,1);
		delay_nms(5);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			}while((QTIS!=6)&&(QTIS!=0x0f));                  //�ٴμ���Ƿ��Ѿ��ҵ���һ������
}















void TurnRight_nDegree(char degree)     //��ת   degree   ���Ƕ� 
{
	motor_motion1(1522, 1522,degree);//1517
	return;
}
void TurnLeft_nDegree(char degree)       //��ת   degree   ���Ƕ� 
{
	motor_motion1(1478, 1478,degree);//1485
	return;
}
void Back_toBlack(void)                  //���ص�����
{
	QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	while(QTIS != 15)
	{
		QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		motor_motion1(1485,1512,1);
	}
	return;
}
void CCheck(void)         //���C���Ƿ���ɫ��
{
	motor_motion1(1500,1500,1);
	delay_nms(200);
	j=0;
	process = 9;

}
void BCheck(void)         //���B���Ƿ���ɫ��   
{
//	TurnLeft_nDegree(10);
	findLline(); 
	delay_nms(200);

	process = 11;//1;	
}

void ACheck(void)         //���A���Ƿ���ɫ��
{
	TurnLeft_nDegree(30);   //֮ǰ��60
	findLline();            //��findLline();��TurnLeft_nDegree(40);��20
	delay_nms(200);
	process = 1;
}
void ICheck(void)         //���I���Ƿ���ɫ��
{
	findLline(); 
	findLline(); 
	TurnLeft_nDegree(35);//��תn��
	delay_nms(200);
	j=0;
	for(i=0,j=0;i<6;i++)
	{
		distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		if(distance>0&&distance < 34)
		{
			j=j+1;
		}
		delay_nms(200);
	}
	if(j<1)
	{
		TurnLeft_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
				j=j+1;
			}
			delay_nms(200);
	 }
	}
		if(j<1)
	{
		TurnLeft_nDegree(4);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
				j=j+1;
			}
			delay_nms(200);
	 }
	}
	if(j<1)
	{
		TurnRight_nDegree(8);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
				j=j+1;
			}
			delay_nms(200);
	 }
	}
	if(j >= 1)
	{
		delay_nms(100);
		Ihavesome = 1;				
	}
	else
	{
		Ihavesome = 0;
	}	
	j=0;	
	process = 11;	
}
void HCheck(void)         //���H���Ƿ���ɫ��
{findLline(); 
	findLline(); 
	TurnLeft_nDegree(38);//��תn��
	delay_nms(200);
	for(i=0;i<6;i++)
	{
		distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		if(distance>0&&distance < 30)	
		{
				Hhavesome = 1;
			return ;
		}
	}
		TurnLeft_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 10)
			{
				return ;
			}
	 }

		TurnLeft_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 10)
			{
					Hhavesome = 1;
				return;
			}
	 }

		TurnRight_nDegree(8);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
					Hhavesome = 1;
				return ;
			}
	}
	
					Hhavesome = 1;

	
	
}
void GCheck(void)         //���G���Ƿ���ɫ��
{findRline(); 
	findRline(); 
	TurnRight_nDegree(46);
	delay_nms(200);
	j=0;
	for(i=0,j=0;i<10;i++)
	{
		distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		if(distance>0&&distance < 34)
		{		Ghavesome = 1;		
			return ;
		}
	}
		TurnRight_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{		Ghavesome = 1;		
				return ;
			}
	 }
		TurnRight_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{		Ghavesome = 1;		
				return;
			}
	 }
	 
		TurnLeft_nDegree(8);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{		Ghavesome = 1;		
				return ;
			}

	 }

		

}
void FCheck(void)         //���F���Ƿ���ɫ��
{
	findRline();
	findRline();
	TurnRight_nDegree(21);//��תn��
	delay_nms(200);
	/*j=0;
	for(i=0,j=0;i<10;i++)
	{
		distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		if(distance>0&&distance < 34)
		{
			j=j+1;
		}
		delay_nms(50);
	}
	if(j<1)
	{
		TurnRight_nDegree(2);//��תn��
		delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 30)
			{
				j=j+1;
			}
			delay_nms(50);
	 }
	}
		if(j<1)
	{
		TurnRight_nDegree(2);//��תn��
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
				j=j+1;
			}
			delay_nms(200);
	 }
	}
			if(j<1)
	{
		TurnLeft_nDegree(8);//
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{
				j=j+1;
			}
			delay_nms(200);
	 }
	}
	if(j >= 1)
	{
		delay_nms(100);
		Fhavesome = 1;	
	}
	else
	{
		Fhavesome = 0;
	}	
	j=0;
	a=1;	*/
	Fhavesome = 1;	
	process = 11;		
	LED_Write_1();
}
                                                                                                                                                                                                                                                                                                        void ECheck(void)         //���E���Ƿ���ɫ��
{
	TurnRight_nDegree(30);//��תn��
	delay_nms(200);
	findRline(); 
	delay_nms(200);
	process = 2;//11;	
}

void DCheck(void)         //���D���Ƿ���ɫ��
{
	findRline(); 
	delay_nms(200);
	process =8;		
}


/*---------------------------------------------------------------------------------------------------------
*	Function name��DownFirst(void)
*  Description���Ѵ�A����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ���� ,��ת��ص�ʮ��·��λ��
---------------------------------------------------------------------------------------------------------*/ 
void DownFirst(void)                                  //��ʮ��·�ڰ�Aɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{				
		time =0;
		
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE); //��ʼ��ʱ��5
		do
		{
			Robot_hunting2(1550);     

		}while(time < 2300);                              
		TIM_Cmd(TIM5, DISABLE);                         	//�رն�ʱ��5
		motor_motion1(1450,1550,16);                      //����
		Turn1_180();	                                    //ת��180��                                  
		do
		{
			Robot_hunting2(1540);                        
		}while(QTIS !=15);                                        
		pulses = LRp;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	                 
		}               
			//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
	
		pointsth++;	                                 	
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��DownSecond(void)
*  Description���Ѵ�B����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ���� ,��ת��ص�ʮ��·��λ�ã���ϸ���̲�DownFirst(void)
---------------------------------------------------------------------------------------------------------*/ 
void DownSecond(void)                                 //��ʮ��·�ڰ�Bɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{	
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //��ʼ��ʱ��5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1900);                           
		TIM_Cmd(TIM5, DISABLE);
		motor_motion1(1450,1550,16);//����                           
		Turn1_180();
		do
		{
			Robot_hunting2(1550);                         
		}while(QTIS != 15);                               
		pulses = LRp;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	                    //ֱ��
		}   
	//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
			//ǰ��һС�ξ���
		pointsth++;			                                  //ֱ���Ϸ���2��ɫ��
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��Downthird(void)    
*  Description���Ѵ�c����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ���� ,��ת��ص�ʮ��·��λ�ã���ϸ���̲�DownFirst(void)

��һ�����Ϊc
---------------------------------------------------------------------------------------------------------*/
void Downthird(void)                                  //��ʮ��·�ڰ�Cɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //��ʼ��ʱ��5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1500);                              
		TIM_Cmd(TIM5, DISABLE);
		motor_motion1(1450,1550,16);                      //����                             
		Turn1_180();
		do
		{
			Robot_hunting2(1530);                          
		}while(QTIS != 15);                               //ֱ��ʮ��·��
		pulses =LRp;
		while(pulses--)
		{
			Robot_hunting2(1545);                  
		}                                                 //ǰ��һС�ξ��룬Ŀ����Ϊ��һ��ת��ֱ����
			//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
			
		pointsth++;			                              //ֱ���Ϸ���3��ɫ��
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��DownFourth(void)
*  Description���Ѵ�D����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ���� ,��ת��ص�ʮ��·��λ�ã���ϸ���̲�DownFirst(void)
---------------------------------------------------------------------------------------------------------*/
void DownFourth(void)                                 //��ʮ��·�ڰ�Dɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //��ʼ��ʱ��5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1000);
		/*TIM_Cmd(TIM5, DISABLE);*/
		motor_motion1(1450,1550,16);                      //����
		Turn_180();                               //��ת 
//		Turn_180_four();
		
		do
		{
			Robot_hunting1(1550);                           //ֱ��        
		}while(QTIS != 15);                               //ֱ��ʮ��·��
		pulses = LRp;
		
		while(pulses--)
		{
				motor_motion2(1550, 1450-50);                    //ֱ��
		}
		pulses = 2;
		while(pulses--)  
		{
			Robot_hunting1(1580);	  
		}	 		
		pointsth++;			                                  //ֱ���Ϸ���4��ɫ��
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��DownFiveth(void)
*  Description���Ѵ�E����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ����,��ϸ���̲�DownFirst(void)
---------------------------------------------------------------------------------------------------------*/
void DownFiveth(void)                                 //��ʮ��·�ڰ�Eɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //��ʼ��ʱ��5
		do
		{
			Robot_hunting2(1550);
		}while(time < 800);
		TIM_Cmd(TIM5, DISABLE);
//		motor_motion1(1450,1550,16);                      //����
		Turn_180();                               //��ת 
//		Turn_180_four();
		
//		do
//		{
//			Robot_hunting1(1550);                           //ֱ��        
//		}while(QTIS != 15);                               //ֱ��ʮ��·��
//		
//		pulses = LRp;
//		
//		while(pulses--)
//		{
//				motor_motion2(1550, 1450);                    //ֱ��
//		}
		motor_motion2(1500,1500);
		delay_ms(50);
//		pulses = 2;
//		while(pulses--)  
//		{
//			Robot_hunting1(1580);	  
//		}	 		
		pointsth++;			                                                                  //ֱ���Ϸ���5��ɫ��
}
void GotoLine(void)
{
	
	int tempa=0;
	switch(pointsth)	                                  // pointsth��ʼֵΪ0
	{
		case 0:DownFirst();	                              //	����A��ʱѡ��
				break;
		case 1:DownSecond();                              //����B��ʱѡ��
				break;
		case 2:Downthird();                             	//����C��ʱѡ��
				break;
		case 3:DownFourth();                              //����E��ʱѡ��
				break;
		case 4:DownFiveth();	                            //����F��ʱѡ��
				break;
		default:
				break;
	}
	tempa++;
	tempa=0x22;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryGpoint(void)
*  Description���Ѵ�G����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ����,��ϸ���̲� CarryIpoint(void)
****************��ʮ��·��λ�ð��˻ص�ֱ����***************
---------------------------------------------------------------------------------------------------------*/
void Gback(){
		pulses=8;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		//TurnRight_nDegree(80);     //��ת120
		findLline();
		/*do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	ѭ��*/
		pulses=15;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
	  TurnLeft_nDegree(80);//��תn��
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		pulses=8;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findRline();
				
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
			//  gai wei 
		pulses = 2;
		while(pulses--)  
		{
			Robot_hunting1(1580);	  
		}	 
}

void Hback(){
		motor_motion1(1550, 1450,10);
		findRline();
		pulses=15;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
	  TurnRight_nDegree(100);//��ת
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		pulses=12;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		pulses = 2;
		while(pulses--)  
		{
			Robot_hunting1(1580);	  
		}	 
}
	
void CarryGpoint(void)                              //��ʮ��·�ڰ�Gɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		Ghavesome=0;
		do
		{
			motor_motion2(1540, 1460);
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
				if(distance > 4 && QTIS == 15){
					have = false;
					Gback();
					process=3;     //*****����******����***********����******����***********����*********************����************************8888
					return;
				}
		}while(distance > 4);  //��������I��λ����鷽��ֱ��
		pulses=15;
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=13;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		//TurnRight_nDegree(80);     //��ת120
		findLline();
		/*do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	ѭ��*/
		pulses=13;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		motor_motion1(1450,1550,25);//����
	  TurnLeft_nDegree(80);//��תn��
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findRline();
		
		
		do
		{
			Robot_hunting1(1540);
		}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
			//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
	
		process=3;
		
}
 /*---------------------------------------------------------------------------------------------------------
*	Function name��CarryHpoint(void)
*  Description���Ѵ�H����˵�ɫ���ʮ��·��λ�ð��˻ص�ֱ����,��ϸ���̲� CarryIpoint(void)
---------------------------------------------------------------------------------------------------------*/

void CarryHpoint(void)                 
{
		Hhavesome=0;
		do
		{
			motor_motion2(1540, 1460);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				if(distance > 4 && QTIS == 15){
					have = false;
					Hback();             //
					process=3;     //*****����******����***********����******����***********����*********************����************************8888
					return;
				}
		}while(distance > 4);  //
		pulses=15;
		while(pulses--)
		{
			motor_motion2(1540, 1460);	  //ֱ��
		}
		
	 //TurnLeft_nDegree(80);     //��ת120  
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	ѭ��
		motor_motion1(1550, 1450,10);
		findRline();
		/*GotoLine();
		findsekuai++;
		if(findsekuai==5)
	{
		process=11;
		Idone=1;                

	}*/
		pulses=13;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		motor_motion1(1450,1550,25);//����
	  TurnRight_nDegree(90);//��ת
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS );
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		
		
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
			//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
	
		process=3;/////////////////////////////////////////BBBBBBBB

}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryCpoint(void)
*  Description����C��λ����ɫ����˻ص�ֱ���� ,��ϸ���̲ο�����A��CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryCpoint(void)                                 
{
		Chavesome=0;
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 30;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//��һ��·��
		pulses = 15;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		GotoLine();
		findsekuai++;
  if(findsekuai==5)
	{
		process=11;
		Idone=1;
	}
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryDpoint(void)
*  Description����D��λ����ɫ����˻ص�ֱ���� ,��ϸ���̲ο�����A��CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryDpoint(void)                              //��ʮ��·�ڰ�Aɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		Dhavesome=0;
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		Turn_180();
		delay_nms(200);
		do
		{
			Robot_hunting1(1540);
		}while(QTIS !=15);//��һ��·��
		pulses = 18;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		GotoLine();
		findsekuai++;
  if(findsekuai==5)
	{
		process=11;
		Idone=1;
	}
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryEpoint(void)
*  Description����E��λ����ɫ����˻ص�ֱ���� ,��ϸ���̲ο�����A��CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryEpoint(void)                                     //��ʮ��·�ڰ�Aɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
	Ehavesome=0;
		do
		{
			Robot_hunting1(1550);
			
		}while(QTIS);
		pulses = 40;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//��һ��·��
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		GotoLine();
	  findsekuai++;
	  if(findsekuai==5)
		{
			process=11;
			Idone=1;
		}
	/*
		Ehavesome=0;
		do
		{
			Robot_hunting1(1550);
			
		}while(QTIS);
		pulses = 40;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//��һ��·��
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		GotoLine();
		pulses = 18;
		while(pulses--)
		{
			motor_motion2(1550,1450);
		}
	  findsekuai++;
	  if(findsekuai==5)
		{
			process=11;
			Idone=1;
		}
		*/
}

/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryBpoint(void)
*  Description����B��λ����ɫ����˻ص�ֱ���� ,��ϸ���̲ο�����A��CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryBpoint(void)                        //��ʮ��·�ڰ�Aɫ���˵���ʼλ���ڷ��ص�ʮ��·��
{
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}		
		Turn_180();
//		stop();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//��һ��·��
		pulses = 20;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		stop();
		TurnLeftAnyPulse(10);
		findRline();
		
		GotoLine();
		motor_motion2(1500, 1500);
		delay_ms(50);
		findsekuai++;
  if(findsekuai==5)
	{
		process=11;
		Idone=1;
	}
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryApoint(void)
*  Description����A��λ����ɫ����˻ص�ֱ����
---------------------------------------------------------------------------------------------------------*/ 
void CarryApoint(void)                   
{
		Ahavesome=0;
		do
		{
			Robot_hunting1(1550);
		}while(QTIS); 
    pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}	
		Turn_180();	   //ת��180��
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);
		pulses = 15;
		while(pulses--)	//
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findRline();  //����ת�䵽���������
		GotoLine();
		findsekuai++;
  if(findsekuai==5)
	{
		process=11;
		Idone=1;
	}
}
/*---------------------------------------------------------------------------------------------------------
*	Function name�� RedCarry(void)
*  Description���Ѻ�ɫɫ����˵���Ӧ�ĺ�ɫ������
---------------------------------------------------------------------------------------------------------*/

void goBackCenter(){
		pulses = 25;
		while(pulses--)
		{
			Robot_hunting1(1540);
		}//�ж��Ƿ񵽴��һ��·��
		Turn_180();//ת��180��	
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
		
		pulses = LRp;
		while(pulses--)
		{
			motor_motion2(1550, 1450);
		}			//ֱ��
}

void gotoG(){
	void moveBack();
	
if(!have){
		moveBack();
		process = 13;
		return ;	
	}
		pulses = 22;
		while(pulses--)
		{
			Robot_hunting1(1550);
		}//�ж��Ƿ񵽴��һ��·��
		TurnLeft_nDegree(35);//��תn��
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=10;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();  //����ת�䵽���������
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>4 );
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		TurnLeft_nDegree(110);//��תn��
			pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1540, 1460);	  //ֱ��
		}
		findRline();  //����ת�䵽���������
		process = 11;

}



void gotoH(){
	void moveBack();
	
if(!have){
		moveBack();
		process = 13;
		return ;	
	}
		pulses = 22;
		while(pulses--)
		{
			Robot_hunting1(1550);
		}//�ж��Ƿ񵽴��һ��·��
		TurnRight_nDegree(27);//��תn��
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=10;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
	
		findRline();  //����ת�䵽���������
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>4 );
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		TurnRight_nDegree(79);//��תn��
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		
		pulses=13;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();  //����ת�䵽���������
		process = 11;

}
/////////////////////////////////////////////////

void RedCarry(void)//���ɫɫ�鵽������    
{
	hadedone++;
	printf("hadedone:%d",hadedone);
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);//�������м�׵�λ��ʱ��ֹͣ
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1530);	  //ֱ��
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //ֱ��
		}
		motor_motion1(1450,1550,25);//����
	  Turn_180();//ת��180��
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//�ص���һ��·��
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		
		if(hadedone >= 6){
				process=12;
			printf("process:%d",process);
			return;
		}
//		if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter(); //  F �� ��i �� ��Ҫ�õ�  �л� F I  �� G H 
//		//gotoG();
//			gotoH();
//				return;
//		}
		
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5 );
		pulses =10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		delay_nms(50);
		Turn_180();
		process=11;
		
}
/*---------------------------------------------------------------------------------------------------------
*	Function name�� WhiteCarry(void)
*  Description���Ѱ�ɫɫ����˵���Ӧ��������  ����ϸ������ο�����RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void WhiteCarry(void)//���ɫɫ�鵽������
{		
	hadedone++;
	printf("hadedone:%d",hadedone);
	  TurnLeft_nDegree(10);
		findLline();                              //��ƫѰ����һ������  Ҳ���ǰ׵��Ӧ����	
		delay_nms(200);
	  do
		{
			Robot_hunting1(1550);
		}while(QTIS);                             //����һ�μ�⵽�׵�����ѭ��0
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1530);	  //ֱ��
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //ֱ��
		}	
		
		motor_motion1(1450,1550,25);//����
		Turn_180();                                //��ת180��   ���Ż��޸�
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);                       //��һ��·��
		pulses = 20;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findRline();
		delay_nms(200);
			if(hadedone >= 6){
				process=12;
				printf("process:%d",process);
			return;
		}
//			if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter();
//			//gotoG();
//			gotoH();
//				return;
//		}
		
		do{
			Robot_hunting(1535);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5);
		pulses =10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		delay_nms(50);
		Turn_180();
		process=11;
		
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��YellowCarry(void)
*  Description���ѻ�ɫɫ����˵���Ӧ��������  ����ϸ������ο�����RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void YellowCarry(void)//���ɫɫ�鵽������
{
	hadedone++;
	printf("hadedone:%d",hadedone);

//		if(QTTS == )
		TurnLeft_nDegree(8);//��תn��
		
		findLline();                        //������Ҫ�Ż�  �ݶ�  
		delay_nms(200);	
		findLline();            //Ѱ���ұ���һ�������ֱ��
	
		motor_motion2(1500,1500);
		delay_ms(50);
//		while(1);
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
			pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1530);	  //ֱ��
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //ֱ��
		}	
		motor_motion1(1450,1550,25);//����
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//��һ��·��
		pulses = 15;           //���ֵ����Ҫ�޸ĵ�
		while(pulses--)
		{
			Robot_hunting2(1530);
		}
		findRline();
		delay_nms(200);
			if(hadedone >= 6){
				process=12;
				printf("process:%d",process);
			return;
		}
//			if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter();
//			//gotoG();
//		gotoH();
//				return;
//		}
		
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5);
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��BlackCarry(void)
*  Description���Ѻ�ɫɫ����˵���Ӧ��������  ����ϸ������ο�����RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void BlackCarry(void)//���ɫɫ�鵽������
{		
	hadedone++;
	printf("hadedone:%d",hadedone);
    findRline();
		delay_nms(200);			
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 40;
		while(pulses--)
		{
			Robot_hunting1(1520);	  //ֱ��
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //ֱ��
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //ֱ��
		}	
	
		motor_motion1(1450,1550,25);//����
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//��һ��·��
		pulses = 18;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		if(hadedone >= 6){
				process=12;
				printf("process:%d",process);
			return;
		}
//			if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter();
//		  //gotoG();
//			gotoH();
//				return;
//		}
			
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5);
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��BlueCarry(void)
*  Description������ɫɫ����˵���Ӧ��������  ����ϸ������ο�����RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void BlueCarry(void)//����ɫɫ�鵽������
{
	hadedone++;
	printf("hadedone:%d",hadedone);
	  findRline();
		delay_nms(200);
		findRline();
		delay_nms(200);
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
	while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //ֱ��
		}	
		delay_nms(200);
		
	/*	pulses = 5;
		while(pulses--)
		{
			delay_nms(20);
			Robot_hunting1(1502);	  //ֱ��
		}*/
		while( (GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1501);	  //ֱ��
		}	
		delay_nms(200);
		motor_motion1(1450,1550,18);//����
		Turn_180();
		 do
	    {
	       Robot_hunting1(1540);
    	}while(QTIS != 15);//�ڶ���·��
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		findLline();
		delay_nms(200);
			if(hadedone >= 6){
				process=12;
			return;
		}
//			if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter();
//		  //gotoG();
//			gotoH();
//				return;
//		}
//		
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5);
		pulses = 10;                      //ǰ�岽��   ���޸�  �Ż�
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��StartCarryToScore(void)
*  Description����ʼ����ֱ�������
---------------------------------------------------------------------------------------------------------*/
void StartCarryToScore(void)
{			
	count = 1;                // ÿ�δ�1��ʼ 
	printf("hadedone: %d\n",hadedone);
	if(hadedone >= 6){
		process = 12;
		return;
	}
		do{
			Robot_hunting1(1530);
		}while(QTIS !=15);
		pulses = LRp-2;
			while(pulses--)
			{
			 	motor_motion2(1550, 1450);	 //ֱ��
			}
				//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
	
		delay_nms(200);
//		printf("Color: %d\n",Robot_checkColor());
		printf("Color: %d\n",xm_colourjudge(pcolor));
	
		colorCheck = true;
		while(colorCheck)
		{
			//color = Robot_checkColor();//ʶ����ɫ
			color =xm_colourjudge(pcolor);
			count++;
//			if(color == 4)
					//color = Robot_checkColor();//ʶ����ɫ
//					color =xm_colourjudge(pcolor);
			switch(color)//��ɫѡ��
			 {
					case Red:RedCarry();colorCheck = false;break;
					case White:WhiteCarry();colorCheck = false;break;
					case Yellow:YellowCarry();colorCheck = false;break;
					case Black:BlackCarry();colorCheck = false;break;
					case Blue:BlueCarry();colorCheck = false;break;
					default:colorCheck = true;break; ////////////////////////////////////////////////////////////////////////////////
			}
		}
		if(hadedone >= 6){
			
				  process = 12;
				colorCheck = false;
			 }
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryIpoint(void)
*   Description����I��λ��ɫ����˵���Ӧ��������  
---------------------------------------------------------------------------------------------------------*/
void CarryIpoint(void)
{
		Ihavesome=0;
		do
		{
			motor_motion2(1540, 1460);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
			if(distance>4 && QTIS == 15){
					pulses=10;
					while(pulses--)
					{
						motor_motion2(1550, 1450);	  //ֱ��
					}
					findLline();
					do
					{
						Robot_hunting1(1550);
					}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
					motor_motion1(1550, 1450,15);
					TurnRight_nDegree(50);       //r->180
					pulses=25;
					while(pulses--)
					{
						motor_motion2(1550, 1450);	  //ֱ��
					}
					process = 13;
					return ;
			}
		}while(distance > 4);  //��������I��λ����鷽��ֱ�ߣ�ֱ������С��3
		pulses=25;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		
		TurnLeft_nDegree(90);     //��ת150  
		do
		{
			motor_motion2(1550, 1450);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}
		while(QTIS != 15);//	ѭ��
		motor_motion1(1550, 1450,12);
		findLline();
		//TurnLeft_nDegree(55);
		
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name��CarryFpoint(void)
*   Description����F��λ��ɫ����˵���Ӧ�������� ,��ϸ������ο�����CarryIpoint(void)   
---------------------------------------------------------------------------------------------------------*/
void CarryFpoint(void)
{
	if(Fhavesome==1)
	{
		Fhavesome=0;
		do
		{
			motor_motion2(1550, 1450);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);		
				if(distance>5 && QTIS == 15){
						pulses=10;
						while(pulses--)
						{
							motor_motion2(1550, 1450);	  //ֱ��
						}
						findRline();
						do
						{
							Robot_hunting1(1550);
						}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
						motor_motion1(1550, 1450,25);
						TurnLeft_nDegree(30);       //r->180
						pulses=25;
						while(pulses--)
						{
							motor_motion2(1550, 1450);	  //ֱ��		
						}
						process = 13;
						return ;
					}
		}while(distance > 5);  //��������F��λ����鷽��ֱ�ߣ�ֱ������С��3
		pulses=25;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		
		TurnRight_nDegree(90);       //r->180
		do{
			motor_motion2(1550, 1445);	  //ֱ��
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while( !QTIS );
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //ֱ��
		}
		//TurnRight_nDegree(55);//��תn��
		findRline();
			process=11;
	}
}

/*******************************************************************************
  * GoStart()
  * Description�����ɫɫ��RGBֵ����255ʱ����Ӧʱ�����ֱ�洢������pColor[]��
  ********************************************************************************/
 void GoStart(void)
 {
	 do
	{
		Robot_hunting1(1550);
	}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
	
	pulses = 15;
	while(pulses--)   //�߳�������
	{
		motor_motion2(1650, 1350);	  //ֱ��
	}	 
	
	do
	{
		Robot_hunting1(1550);
	}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
	
	pulses = LRp-2;
	while(pulses--)
	{
		motor_motion2(1550, 1450);	  //ֱ��
	}
	//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1550);	  
	}	 
	
  }	
 
void moveBack(){
	printf("gohome");
	pulses = 10;
	while(pulses--)
	{
		Robot_hunting1(1560);	  //ֱ��
	}
	do
	{
		Robot_hunting1(1560);
		QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	}while(QTIS != 15);//�ж��Ƿ񵽴��һ��·��
	pulses = 40;
	while(pulses--)
	{
		motor_motion2(1550, 1450);	  //ֱ��
	}
	process = 13;
}




void turn_back(){
	Turn_180();
	process = 11;
	return;
}


//***************************************************************************************
void EXTIX_Init(void)
{
 
   	EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

   GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//ʹ��PORTA,PORTEʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;//|GPIO_Pin_6|GPIO_Pin_7;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //���ó���������
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE4,3

//	//��ʼ�� WK_UP-->GPIOA.0	  ��������
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0���ó����룬Ĭ������	  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.0

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��



//   //GPIOE.3	  �ж����Լ��жϳ�ʼ������ �½��ش��� //KEY1
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
//  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

//   //GPIOE.4	  �ж����Լ��жϳ�ʼ������  �½��ش���	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
//  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���



   //GPIOE.5	  �ж����Լ��жϳ�ʼ������  �½��ش���	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	 //GPIOE.6	  �ж����Լ��жϳ�ʼ������  �½��ش���	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line6;
//  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	 //GPIOE.7	  �ж����Լ��жϳ�ʼ������  �½��ش���	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
//  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


//   //GPIOA.0	  �ж����Լ��жϳ�ʼ������ �����ش��� PA0  WK_UP
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 

//  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//  	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
//  	NVIC_Init(&NVIC_InitStructure); 

//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//ʹ�ܰ���KEY1���ڵ��ⲿ�ж�ͨ��
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
//  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ�ܰ���KEY0���ڵ��ⲿ�ж�ͨ��
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
//  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY0���ڵ��ⲿ�ж�ͨ��
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
//  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
//	
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY0���ڵ��ⲿ�ж�ͨ��
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
//  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY0���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//�����ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
}



//******************************************************************************************/

void EXTI9_5_IRQHandler(void)
{
	if( EXTI_GetITStatus(EXTI_Line5) != RESET) 
	{
		count_tcs230++;
//		tim5_interruput_flag=0x01;
	}
	EXTI_ClearITPendingBit(EXTI_Line5);
	
}

//******************************************************************************************/
int main(void)
{		
						Peripheral_Init();//��ģ���ʼ������//
						TIM5_Int_Init(0xfff0-1,720-1);
						delay_init();
						EXTIX_Init();
   					process =3;	 //��C/B/A/D/E/G/H���е�ɫ��ᵽ��ʼ���ϣ����������ֻ��Ҫ�޸ĸò�������  �������ȴ�case11��ʼ
//						process =8;	 //��C/B/A/D/E/G/H���е�ɫ��ᵽ��ʼ���ϣ����������ֻ��Ҫ�޸ĸò�������  �������ȴ�case11��ʼ
						led=1;
						delay_nms(1000);
						led=0;
						delay_nms(1000);
						xm_TCS230_WhiteBalance(pcolor);  //��ƽ��
						led=0;
//	while(1){
////				findLline();
//		QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
//		delay_ms(1000);
//	}
//						while(1)
//						{
//							moveBack();
//							delay_ms(2000);
//						motor_motion2(1500, 1500);while(1);							
//						}
						
						
//						delay_ms(3000);
	
//				i=4;
//				while(i){
//				i--;
//				Turn_180();
//					delay_ms(1000);
//				}while(1);

//	while(1){
////						motor_motion1(1500,1500,3);
//						color=xm_colourjudge(pcolor);
//						delay_nms(1000);
//						}
//	
						//color = Robot_checkColor();//ʶ����ɫ
//	
//						QTIS=1;
//	
//						while(true){
//						
//							
////							TurnRight_nDegree(45);
////							delay_nms(2000);
////						    TurnLeft_nDegree(90);
////						    delay_nms(2000);
////							TurnLeft_nDegree(90);
////							delay_nms(2000);
////							TurnLeft
////							_nDegree(120);
//						
////							motor_motion2(1540, 1450);
//							
//							//color = Robot_checkColor();//ʶ����ɫ
////							xm_colourjudge(pcolor);
////							
////							delay_nms(2000);
//							
////									 switch (QTIS)
////									{
////									 //case 0:motor_motion1(speed, (3000-speed)-35-10,3);break;
////										case 1:motor_motion1(speed, (3000-speed)-35-10,3);break;
////										case 2:motor_motion1(speed, (3000-speed)-35,1);break;
////										case 3:motor_motion1(speed, (3000-speed)-35,1);break;
////										case 4:motor_motion1(speed+35, (3000-speed),1);break;
////									
////										case 8:motor_motion1(speed+35+10, (3000-speed),3);break;
////										case 12:motor_motion1(speed+35, (3000-speed),2);break;
////									
////										case 6:motor_motion2(speed+100, 3000-speed-100-2);break;	
////										default:motor_motion2(1550, 1450);break;	  
////									}


//									 QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
//									 delay_nms(200);
//									 
//								}
	
					//	while(1)
						//{ 
						//	printf("Color: %d\n",Robot_checkColor());
					//	}
						/*	do{
								GPIO_SetBits(GPIOC, GPIO_Pin_6);
								delay_nus(1500);
								GPIO_ResetBits(GPIOC,GPIO_Pin_6);
								GPIO_SetBits(GPIOC, GPIO_Pin_7);
								delay_nus(1500);
								GPIO_ResetBits(GPIOC,GPIO_Pin_7);
								delay_nms(20);	//���һ��������PWM����
								//i++;	
							}while(true);*/
		led=0;
		delay_nms(1000);
		delay_nms(1000);
//		process = 8;
		GoStart();		 //�߳���ʼ����
//	process = 11;
		while(1)
		{ 
		//printf("process:%d\n",process);
   //     motor_motion2(1500, 1500); 		
			
			
	
//			
//			process = 9;
			switch(process)	 //��ʼʱprocess=1
			{
				/**************һ��1~10ѡ���ò���********************************/
				case 1:	CCheck();
				        CarryCpoint();
						break;//���������c��
				
				case 2:	BCheck();
				        CarryBpoint();
//								while(1);
						break;//���������b��
				
				case 3:	ACheck();
				        CarryApoint();
								
						break;//���������a��
				
				case 4:	ICheck();
						CarryIpoint();
						break;//���������i��    //i��Ϊ��ɫʶ��������������֮��
				
				case 5:	HCheck();
				        CarryHpoint();
						break;//���������h��
				
				case 6: GCheck();
				        CarryGpoint();
						break;//���������g��
						
				case 7:	FCheck();
				        CarryFpoint();
						break;//���������f��
						
				case 8:	ECheck();
				        CarryEpoint();
						break;//���������e��
						
				case 9: DCheck();
				        CarryDpoint();
						break;//���������d��	
						
				//case 10:turn_back();break;
				case 11:StartCarryToScore();
						break;//��C/B/A/D/E/G/H���е�ɫ���Ʒ���
						
				case 12:moveBack();
						break;
				default:break;
			}
//
//			while(1);
	}
}

