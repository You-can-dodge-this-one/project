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

/* -------------------------颜色传感器引脚电平控制定义--------------------------------------*/
#define S0_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_4) //写1
#define S0_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_4) //写0
#define S1_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_5) 
#define S1_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define S2_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_6)
#define S2_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_6)
#define S3_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_8)
#define S3_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_8)
#define LED_Write_1()    GPIO_SetBits(GPIOE,GPIO_Pin_10)
#define LED_Write_0()    GPIO_ResetBits(GPIOE,GPIO_Pin_10)
//OUT 接到PD2


 /*********读取QTI的电平宏定义*******/
#define PE0_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)//读PE0上的值
#define PE1_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)//读PE1上的值
#define PE2_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//读PE2上的值
#define PE3_ReadBit()   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读PE3上的值

#define Yellow 3
#define White  1
#define Red    4
#define Black  2
#define Blue   5


extern int count_tcs230;


/************************************函数声明************************/
void BlueCarry(void);              //搬蓝色色块到分数区
void BlackCarry(void);             //搬黑色色块到分数区
void YellowCarry(void);            //搬黄色色块到分数区
void WhiteCarry(void);             //搬白色色块到分数区
void RedCarry(void);                //搬红色色块到分数区 

/*---------------------------------结构变量、宏定义、变量、常量定义------------------------------------------------------------------------------*/
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
char Ahavesome = 1;//A有无色块标志   //比赛时候可以直接将抽到点的标志全部置1
char Bhavesome = 1;//B有无色块标志
char Chavesome = 1;//C有无色块标志
char Dhavesome = 1;//D有无色块标志
char Ehavesome = 1;//E有无色块标志
char Fhavesome = 0;//F有无色块标志
char Ghavesome = 0;//G有无色块标志
char Hhavesome = 0;//H有无色块标志
char Ihavesome = 0;//I有无色块标志

char pointsth=0;//在直线上放第几个色块
char findsekuai = 0;// 已经搬运色块的总个数

//char pointsth=3;//在直线上放第几个色块
//char findsekuai = 3;// 已经搬运色块的总个数




char Idone=0;
char sekuai = 5;//色块的总个数 
u16 pcolor[3]={0,0,0};//颜色传感器的比例因子 
u16 RGB[3]={0,0,0};
int hadedone = 1;
unsigned char QTIS;//循线的QTI状态

unsigned int color =0;//颜色代表值

unsigned int distance=0;//超声波测距

unsigned int speed=0;//电机速度
unsigned int pulses=0;//脉冲数
bool colorCheck = true;
volatile char i=0,j=0;
unsigned int i1=0;
unsigned int goal=38;	  //搬运到目标点计分区的参数   //数值越大表步数约长
unsigned int a=0;        //用来判断搬fg  和hi颜色是  需要修正的方向
int count = 1;                //颜色识别次数计数
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
	/* 将外设RCC寄存器重设为默认值，即有关寄存器复位，但该函数不改禦CC_CR的HSITRIM[4:0]位，也不重置寄存器RCC_BDCR和寄存器RCC_CSR */
	RCC_DeInit();
	/* 使能外部HSE高速晶振 */
	RCC_HSEConfig(RCC_HSE_ON);
	/* 等待HSE高速晶振稳定，或者在超时的情况下退出 */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	/* SUCCESS:HSE晶振稳定且就绪，ERROR：HSE晶振未就绪 */
	if(HSEStartUpStatus == SUCCESS)
	{
		/* 使能flash预取指令缓冲区。这两句跟RCC没直接关系 */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		/* 设置FLASH存储器延时时钟周期数，2是针对高频时钟的，
		FLASH_Latency_0：0延时周期，FLASH_Latency_1：1延时周期 
		FLASH_Latency_2：2延时周期 */
		FLASH_SetLatency(FLASH_Latency_2);
		/* HCLK=SYSCLK 设置高速总线时钟=系统时钟 */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK1=HCLK/2 设置低速总线1时钟=高速时钟的二分频*/
		RCC_PCLK1Config(RCC_HCLK_Div2);
		/* PCLK2=HCLK 设置低速总线2时钟=高速总线时钟 */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* Set PLL clock output to 72MHz using HSE (8MHz) as entry clock */
		/* 利用锁相环将HSE外部8MHz晶振9倍频到72MHz */ 
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		/* Enable PLL：使能PLL锁相环 */
		RCC_PLLCmd(ENABLE);
		
		 /* Wait till PLL is ready：等待锁相环输出稳定 */
     /* RCC_FLAG_HSIRDY：HSI晶振就绪，RCC_FLAG_HSERDY：HSE晶振就绪
        RCC_FLAG_PLLRDY：PLL就绪，RCC_FLAG_LSERDY：LSE晶振就绪
        RCC_FLAG_LSIRDY：LSI晶振就绪，RCC_FLAG_PINRST：引脚复位
        RCC_FLAG_PORRST：POR/PDR复位，RCC_FLAG_SFTRST：软件复位
        RCC_FLAG_IWDGRST：IWDG复位，RCC_FLAG_WWDGRST：WWDG复位
        RCC_FLAG_LPWRRST：低功耗复位 */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	;
		
		 /* Select PLL as system clock source：将锁相环输出设置为系统时钟 */
     /* RCC_SYSCLKSource_HSI：选择HSI作为系统时钟
        RCC_SYSCLKSource_HSE：选择HSE作为系统时钟
        RCC_SYSCLKSource_PLLCLK：选择PLL作为系统时钟*/
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		 /* 等待PLL作为系统时钟标志位置位 */
     /* 0x00：HSI作为系统时钟；0x04：HSE作为系统时钟
        0x08：PLL作为系统时钟 */
		while(RCC_GetSYSCLKSource() != 0x08);	
	}
	
	 /* Enable GPIOA~E and AFIO clocks：使能外围端口总线时钟。注意各外设的隶属情况，不同芯片和开发板的分配不同*/
 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
													RCC_APB2Periph_GPIOB|
													RCC_APB2Periph_GPIOC|
													RCC_APB2Periph_GPIOD|
													RCC_APB2Periph_GPIOE|
													RCC_APB2Periph_AFIO, ENABLE);
//   /* USART1 clock enable：USART1时钟使能 */
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//	 /* TIM1 clock enable：TIM1时钟使能 */
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//   /* TIM2 clock enable：TIM2时钟使能*/
//	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	 /* ADC1 clock enable：ADC1时钟使能*/
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

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 //超声波
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
   TIM_DeInit( TIM2);//复位TIM2定时器
   TIM_TimeBaseStructure.TIM_Period = 99;
   TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, & TIM_TimeBaseStructure);
   /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
   TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  
   /* TIM2 enable counter [允许tim2计数]*/
   TIM_Cmd(TIM2, DISABLE);
}
/************************************************
  * @brief  Configure the TIM3 Counter.
  * @param  None
  * @retval None
  ***********************************************/
void TIM3_Counter_Configure(void)
{
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;//设置自动装载寄存器
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;//分频计数
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//选择向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);// Time base configuration

	TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

	TIM_SetCounter(TIM3, 0);//设置计数器的值
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//使能定时器中断
	TIM_Cmd(TIM3, DISABLE); //使能定时器
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
  TIM_SetCounter(TIM4,0);			 //TIM4计数值清零
  TIM_Cmd(TIM4,DISABLE); 			 //TIM4计数器失能
}
  
void TIM5_Configure(void)
{
   TIM_DeInit(TIM5);//复位TIM5定时器
	
   TIM_TimeBaseStructure.TIM_Period = 9;
   TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
	 
	
	
	
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM5, & TIM_TimeBaseStructure);
   /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
   TIM_ClearFlag(TIM5, TIM_FLAG_Update);
   /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  
   /* TIM2 enable counter [允许tim2计数]*/
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
	//设置TIM3的外部计数器 PD2  ETR	 （引脚的设置跟硬件有关）
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
  GPIO_Init(GPIOE,&GPIO_InitStructure);
}

u8 QTI_State(u8 pin)//获取红外值
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
	USART1->SR;//不可少，否则出现首次发送首字符丢失
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (u8) ch);
	/* Loop until the end of transmission */
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  // waiting here
	
	return ch;
}

void delay_nus(unsigned int n)  //延时n us: n>=6,最小延时单位6us
{ 
  unsigned int j;
  while(n--)              // 外部晶振：8M；PLL：9；8M*9=72MHz
  {
    j=8;				  // 微调参数，保证延时的精度
	while(j--);
  }
}

void delay_nms(unsigned int n)  //延时n ms
{
  while(n--)		   // 外部晶振：8M；PLL：9；8M*9=72MHz
    delay_nus(1100);   // 1ms延时补偿
}
void motor_motion1(unsigned int left_val, unsigned int right_val, unsigned int count)     //左     右     计次
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
  *  读取颜色色块RGB值，分别存在数组RGB[3]={0,0,0}内
  ********************************************************************************/
u8 TCS230_CurrentColor(u16 pRGB[3])
{
	u8 currentcolor=0;
	S0_Write_0();	S1_Write_1();           //输出频率为(1/50)*500KHz=10KHz	
	LED_Write_1();          //打开LED
	
	S2_Write_0();	S3_Write_0();           //选择红色
	
	times=0;
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	while(pRGB[0] != times);
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	RGB[0] = TIM_GetCounter(TIM3);
	
	S3_Write_1();//选择蓝色
	
	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(pRGB[1] != times);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	RGB[1] = TIM_GetCounter(TIM3);	
	
	S2_Write_1();//选择绿色
	
	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(pRGB[2] != times);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	RGB[2] = TIM_GetCounter(TIM3);
	
	LED_Write_0();//关闭LED	
	
	printf("Red: %d  Blue: %d  Green: %d\n",RGB[0],RGB[1],RGB[2]);
	return currentcolor;
}
/*******************************************************************************
  * @brief  TCS230_WhiteBalance Function.
  *  Description计算白色色块RGB值到达255时，对应时间数分别存储在数组pColor[]里
  ********************************************************************************/
void TCS230_WhiteBalance(u8 pColor[3])                   //白平衡初始化
{
	S0_Write_0();	
	S1_Write_1();//输出频率为(1/50)*500KHz=10KHz
	LED_Write_1();//打开LED
	
	S2_Write_0();	
	S3_Write_0();//选择红色

	times=0;TIM_SetCounter(TIM3,0);//计数器3清除
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[0] = times;//时间比例因子	
	
	S3_Write_1();//选择蓝色

	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[1] = times;//时间比例因子	

	S2_Write_1();//选择绿色

	times=0;TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM2, ENABLE);TIM_Cmd(TIM3, ENABLE);
	while(TIM_GetCounter(TIM3)<255);
	TIM_Cmd(TIM2, DISABLE);TIM_Cmd(TIM3, DISABLE);
	pColor[2] = times;//时间比例因子	
	
	LED_Write_0();//关闭LED	
	
	printf("Red**: %d  Blue**: %d  Green**: %d\n",pColor[0],pColor[1],pColor[2]);
}
/*******************************************************************************
  * @brief  Robot_checkColor(void)
  *  读取颜色色块RGB值，并根据读取的RGB值转换成颜色
  ********************************************************************************/
 unsigned int Robot_checkColor(void)             
 {
 	  TCS230_CurrentColor(pcolor);                      //读取颜色色块的RGB值
	  printf("Red: %d  Blue: %d  Green: %d\n",RGB[0],RGB[1],RGB[2]);
		//判断颜色                                             //通过RGB值判断颜色
	 		if((RGB[0] > 90)&&((RGB[0] - RGB[2]) > 60) && ((RGB[0] - RGB[1]) > 60)&&((RGB[2] - RGB[1]) < 80))
		{
			return 3;//红色
		}
		else if(((abs(RGB[0] - RGB[2])<50)&&(abs(RGB[0] - RGB[1])<50)&& (RGB[0]>100) ) ||(  RGB[1]-RGB[2]<15 && RGB[0]>200 && (RGB[1]<80)) || ( (RGB[0]>200)&& (RGB[2]>200)&&(RGB[1]<15)      )  )
		{
			return 2;//白色
		}
		else if((abs(RGB[0] - RGB[2])<30)&&(abs(RGB[0] - RGB[1])<30)&& (RGB[0]<40)&& (RGB[1]<40)&& (RGB[2]<40) )
		{
			return 4;//黑色
		}
		else if(	((RGB[0] < RGB[1]) && (RGB[2] < RGB[1])&&((RGB[1] - RGB[0])>10)) || ((RGB[0] < RGB[1]) && (RGB[2] < RGB[1])&&((RGB[1] - RGB[0])>5))	)
		{
			return 5;//蓝色
		}
		else if( (((RGB[2] - RGB[1]) > 80) && ((RGB[1] - RGB[0]) > 80)&&(RGB[2] > 90)) || (((RGB[2] - RGB[1]) > 50) && ((RGB[0]-RGB[1]) > 65) && RGB[1]>80   ) )
		{
			return 1;//黄色
		}
		return 0;
		
		
 }
int GetDis(int echo,int trig)     //获取超声波所测物块距离值 
{
  int dis;
	int count;
	GPIO_ResetBits(GPIOC,echo);			//echo端口复位
	GPIO_ResetBits(GPIOC,trig);			//trig端口复位
  TIM_SetCounter(TIM4,0);			 //TIM4计数值清零
	GPIO_SetBits(GPIOC,trig);		          //trig置高 发出10us的高电平信号 
	delay_nus(10); 
	GPIO_ResetBits(GPIOC,trig);
	delay_nus(100);  		  
	while(GPIO_ReadInputDataBit(GPIOC, echo) == 0);
	TIM_Cmd(TIM4,ENABLE);    //开启计数器
       //开启定时器开始计时
	while(GPIO_ReadInputDataBit(GPIOC, echo));   //等待echo置低
	TIM_Cmd(TIM4,DISABLE);   //关闭计数器

	count = TIM_GetCounter(TIM4);//获取计数器值
	dis = (int)count/60.034;//转换为距离,即29.034us超声波能传播1cm
	return dis;
}

/*
 * //检测中间左QTI是否在黑线内
 */

bool IsMLeftQtiBlack(void)   //检测中间左QTI是否在黑线内
{
	if(QTI_State(QTI1_Pin)==true)    //判断中间左QTI是否检测到黑线
	{                                //在黑线返回ture    不在黑线返回false  
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
 * //检测中间右QTI是否在黑线内
 */
bool IsMRightQtiBlack(void)  //检测中间右QTI是否在黑线内
{
	if(QTI_State(QTI4_Pin)==true)   //判断中间左QTI是否检测到黑线
	{                               //在黑线返回ture    不在黑线返回false  
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

void PulseOut(uint8_t pin,int speed)    //单轮旋转设置
{
	GPIO_SetBits(GPIOC,pin );
	delay_nus(speed);
	GPIO_ResetBits(GPIOC,pin);
}

void stop(void)//停止
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

void SpinLeft(void)// 左旋转
{
	PulseOut(leftservo,1480);
	PulseOut(rightservo,1480);
	delay_nms(20);
}

void SpinRight(void)// 右旋转
{
	PulseOut(leftservo,1520);
	PulseOut(rightservo,1520);
	delay_nms(20);
}

void TurnLeftAnyPulse(int pulses)//左旋转某个脉冲     pluses设置旋转角度
{
	while(pulses--)
	{
		SpinLeft();      //左旋转
		delay_nms(2);
	}
}
void TurnRightAnyPulse(int pulses) //右旋转某个脉冲   pluses设置旋转角度
{
	while(pulses--)
	{
		SpinRight();
		delay_nms(2);
	}
}


void findLline(void)  //找左边下一根线
{ 
  	TurnLeftAnyPulse(8);
	while(1)
	{
		SpinLeft();                        //左旋转
		if(IsMLeftQtiBlack())              //  中间左是否检测到黑线
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
//void findL1line(void)  //用于黄色
//{ 
//  	TurnLeftAnyPulse(8);
//	while(1)
//	{
//		SpinLeft();                        //左旋转
//		if(IsMLeftQtiBlack())              //  中间左是否检测到黑线
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
void findRline(void) //找右边下一根线
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
//函数名称：PE0_state()
//功能： 获得左边第一个QTI的返回信号
//参数：无参数
//返回值：1：高电平，看到黑线；0：低电平，看到白线
*/

int PE0_state(void)
{	
 	return PE0_ReadBit();
}

/*
函数名称：PE1_state()
功能： 获得左边第二个QTI的返回信号
参数：无参数
返回值：1：高电平，看到黑线；0：低电平，看到白线
*/
int PE1_state(void)
{
	return PE1_ReadBit();
}

/*
函数名称：PE2_state()
功能： 获得右边第二个QTI的返回信号
参数：无参数
返回值：1：高电平，看到黑线；0：低电平，看到白线
*/
int PE2_state(void)
{
	return PE2_ReadBit();
}

/*
函数名称：PE3_state()
功能： 获得右边第一个QTI的返回信号
参数：无参数
返回值：1：高电平，看到黑线；0：低电平，看到白线
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

	delay_nms(20);	//输出一定数量的PWM波形
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
			case 1:motor_motion1(speed, (3000-speed)-40-10,2);break;//左转
			case 2:motor_motion1(speed, (3000-speed)-40,1);break;//左转
			case 3:motor_motion1(speed, (3000-speed)-40,1);break;//左转

			case 4:motor_motion1(speed+40, (3000-speed),1);break;//右转
			case 8:motor_motion1(speed+40+10, (3000-speed),2);break;//右转
			case 12:motor_motion1(speed+40, (3000-speed),1);break;//右转
			
			case 6:motor_motion2(speed, 3000-speed-2);break;	  //直行
			default:motor_motion2(speed, 3000-speed-2);break;	   //这一句不能省
		}
}
void Peripheral_Init(void)    //各模块初始化配置函数
{
		RCC_Configuration();     //时钟、外设使能
		NVIC_Configuration();    //中断优先级划分
		GPIO_USART1_Configure(); //串口1 IO口配置
		USART1_Configuration();	 //串口1配置
		GPIO_TCS230_Configure();  //颜色传感器IO口配置
		TIM2_Configure();        // 定时器2配置
		GPIO_TIM3_Configure(); //定时器3外部计数引脚配置
		TIM3_Counter_Configure();//	定时器3计数器配置
		TIM5_Configure();	  //   定时器5配置
		GPIO_Motor_Config();  //两轮舵机IO引脚配置
		GPIO_QTI_Config();	  //QTI输入引脚配置
		TIM4_Configuration(); //定时器4配置
		GPIO_Dist_Config();	//
}
/*转弯控制函数*/
void TurnR_toBlackLine(void)                  //右转寻找下一条黑线
{	 
		motor_motion1(1515, 1515,5);              //右转
		do
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);   
				motor_motion1(1515, 1515,1);	     
			} while(QTIS!=6);                       //检测循迹是否为1001  是则停止右转
		do
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);  
				//motor_motion1(1515, 1515,1);	
			} while(QTIS!=6);                     //再次检测循迹是否为1001   
		motor_motion1(1515, 1515,1);
}

void TurnL_toBlackLine(void)                   //左转寻找下一条黑线
{	
		motor_motion1(1485, 1485,10);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1485, 1485,1);	
			} while(QTIS!=6);                    //检测循迹是否为1001  是则停止右转
		delay_nms(5);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				//motor_motion1(1485, 1485,1);	
			} while(QTIS!=6);                   //再次检测循迹是否为1001 
		motor_motion1(1485, 1485,1);           //左偏一点
}
void Turn1_180(void)                    //   快速旋转  //先向左偏移一定角度，然后向左旋转寻找下一条黑线
{	 
		motor_motion1(1350, 1350,15	);       //左偏一定角度
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1400, 1400,1);	
			} while(QTIS!=6);                  //左旋转寻找黑线   也就是检测循迹是否为1001
		delay_nms(5);
		 while(QTIS!=6)
			{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1480, 1480,1);	
			}                   //再次检测是否已经找到下一条黑线
}
void Turn_180(void)                    //   慢速旋转  //先向左偏移一定角度，然后向左旋转寻找下一条黑线
{	 
		motor_motion1(1465, 1465,32	);       //左偏一定角度
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			} while(QTIS!=6);                  //左旋转寻找黑线   也就是检测循迹是否为1001				0110
			motor_motion2(1500, 1500);
//		motor_motion1(1475, 1475,1);
//		delay_nms(5);
//		do{
//				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
//				motor_motion1(1465, 1465,1);	
//			}while(QTIS!=6);                  //再次检测是否已经找到下一条黑线
}



void Turn_180_four(void)                    //   慢速旋转  //先向左偏移一定角度，然后向左旋转寻找下一条黑线
{	 
		motor_motion1(1465, 1465,35	);       //左偏一定角度
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			} while((QTIS!=6)&&(QTIS!=0x0f));//while(QTIS!=6);                  //左旋转寻找黑线   也就是检测循迹是否为1001
		motor_motion1(1475, 1475,1);
		delay_nms(5);
		do{
				QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
				motor_motion1(1465, 1465,1);	
			}while((QTIS!=6)&&(QTIS!=0x0f));                  //再次检测是否已经找到下一条黑线
}















void TurnRight_nDegree(char degree)     //右转   degree   个角度 
{
	motor_motion1(1522, 1522,degree);//1517
	return;
}
void TurnLeft_nDegree(char degree)       //右转   degree   个角度 
{
	motor_motion1(1478, 1478,degree);//1485
	return;
}
void Back_toBlack(void)                  //返回到黑线
{
	QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	while(QTIS != 15)
	{
		QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		motor_motion1(1485,1512,1);
	}
	return;
}
void CCheck(void)         //检查C点是否有色块
{
	motor_motion1(1500,1500,1);
	delay_nms(200);
	j=0;
	process = 9;

}
void BCheck(void)         //检测B点是否有色块   
{
//	TurnLeft_nDegree(10);
	findLline(); 
	delay_nms(200);

	process = 11;//1;	
}

void ACheck(void)         //检测A点是否有色块
{
	TurnLeft_nDegree(30);   //之前是60
	findLline();            //加findLline();后TurnLeft_nDegree(40);减20
	delay_nms(200);
	process = 1;
}
void ICheck(void)         //检测I点是否有色块
{
	findLline(); 
	findLline(); 
	TurnLeft_nDegree(35);//左转n度
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
		TurnLeft_nDegree(2);//右转n度
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
		TurnLeft_nDegree(4);//右转n度
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
		TurnRight_nDegree(8);//右转n度
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
void HCheck(void)         //检测H点是否有色块
{findLline(); 
	findLline(); 
	TurnLeft_nDegree(38);//左转n度
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
		TurnLeft_nDegree(2);//右转n度
			delay_nms(200);
	  for(i=0,j=0;i<6;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 10)
			{
				return ;
			}
	 }

		TurnLeft_nDegree(2);//右转n度
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

		TurnRight_nDegree(8);//右转n度
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
void GCheck(void)         //检测G点是否有色块
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
		TurnRight_nDegree(2);//右转n度
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{		Ghavesome = 1;		
				return ;
			}
	 }
		TurnRight_nDegree(2);//右转n度
			delay_nms(200);
	  for(i=0,j=0;i<10;i++)
	 {
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
			if(distance>0&&distance < 34)
			{		Ghavesome = 1;		
				return;
			}
	 }
	 
		TurnLeft_nDegree(8);//右转n度
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
void FCheck(void)         //检测F点是否有色块
{
	findRline();
	findRline();
	TurnRight_nDegree(21);//右转n度
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
		TurnRight_nDegree(2);//右转n度
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
		TurnRight_nDegree(2);//右转n度
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
                                                                                                                                                                                                                                                                                                        void ECheck(void)         //检测E点是否有色块
{
	TurnRight_nDegree(30);//右转n度
	delay_nms(200);
	findRline(); 
	delay_nms(200);
	process = 2;//11;	
}

void DCheck(void)         //检测D点是否有色块
{
	findRline(); 
	delay_nms(200);
	process =8;		
}


/*---------------------------------------------------------------------------------------------------------
*	Function name：DownFirst(void)
*  Description：把从A点搬运的色块从十字路口位置搬运回到直线上 ,并转弯回到十字路口位置
---------------------------------------------------------------------------------------------------------*/ 
void DownFirst(void)                                  //从十字路口把A色块运到起始位置在返回到十字路口
{				
		time =0;
		
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE); //开始定时器5
		do
		{
			Robot_hunting2(1550);     

		}while(time < 2300);                              
		TIM_Cmd(TIM5, DISABLE);                         	//关闭定时器5
		motor_motion1(1450,1550,16);                      //后退
		Turn1_180();	                                    //转弯180度                                  
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
*	Function name：DownSecond(void)
*  Description：把从B点搬运的色块从十字路口位置搬运回到直线上 ,并转弯回到十字路口位置，详细过程参DownFirst(void)
---------------------------------------------------------------------------------------------------------*/ 
void DownSecond(void)                                 //从十字路口把B色块运到起始位置在返回到十字路口
{	
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //开始定时器5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1900);                           
		TIM_Cmd(TIM5, DISABLE);
		motor_motion1(1450,1550,16);//后退                           
		Turn1_180();
		do
		{
			Robot_hunting2(1550);                         
		}while(QTIS != 15);                               
		pulses = LRp;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	                    //直行
		}   
	//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
			//前进一小段距离
		pointsth++;			                                  //直线上放了2块色块
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：Downthird(void)    
*  Description：把从c点搬运的色块从十字路口位置搬运回到直线上 ,并转弯回到十字路口位置，详细过程参DownFirst(void)

第一个搬的为c
---------------------------------------------------------------------------------------------------------*/
void Downthird(void)                                  //从十字路口把C色块运到起始位置在返回到十字路口
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //开始定时器5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1500);                              
		TIM_Cmd(TIM5, DISABLE);
		motor_motion1(1450,1550,16);                      //后退                             
		Turn1_180();
		do
		{
			Robot_hunting2(1530);                          
		}while(QTIS != 15);                               //直行十字路口
		pulses =LRp;
		while(pulses--)
		{
			Robot_hunting2(1545);                  
		}                                                 //前进一小段距离，目的是为下一次转到直线上
			//  gai wei 
	pulses = 2;
	while(pulses--)  
	{
		Robot_hunting1(1580);	  
	}	 
			
		pointsth++;			                              //直线上放了3块色块
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：DownFourth(void)
*  Description：把从D点搬运的色块从十字路口位置搬运回到直线上 ,并转弯回到十字路口位置，详细过程参DownFirst(void)
---------------------------------------------------------------------------------------------------------*/
void DownFourth(void)                                 //从十字路口把D色块运到起始位置在返回到十字路口
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //开始定时器5
		do
		{
			Robot_hunting2(1550);
		}while(time < 1000);
		/*TIM_Cmd(TIM5, DISABLE);*/
		motor_motion1(1450,1550,16);                      //后退
		Turn_180();                               //后转 
//		Turn_180_four();
		
		do
		{
			Robot_hunting1(1550);                           //直行        
		}while(QTIS != 15);                               //直行十字路口
		pulses = LRp;
		
		while(pulses--)
		{
				motor_motion2(1550, 1450-50);                    //直行
		}
		pulses = 2;
		while(pulses--)  
		{
			Robot_hunting1(1580);	  
		}	 		
		pointsth++;			                                  //直线上放了4块色块
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：DownFiveth(void)
*  Description：把从E点搬运的色块从十字路口位置搬运回到直线上,详细过程参DownFirst(void)
---------------------------------------------------------------------------------------------------------*/
void DownFiveth(void)                                 //从十字路口把E色块运到起始位置在返回到十字路口
{
		time =0;
		TIM5_Configure();
		TIM_Cmd(TIM5, ENABLE);                            //开始定时器5
		do
		{
			Robot_hunting2(1550);
		}while(time < 800);
		TIM_Cmd(TIM5, DISABLE);
//		motor_motion1(1450,1550,16);                      //后退
		Turn_180();                               //后转 
//		Turn_180_four();
		
//		do
//		{
//			Robot_hunting1(1550);                           //直行        
//		}while(QTIS != 15);                               //直行十字路口
//		
//		pulses = LRp;
//		
//		while(pulses--)
//		{
//				motor_motion2(1550, 1450);                    //直行
//		}
		motor_motion2(1500,1500);
		delay_ms(50);
//		pulses = 2;
//		while(pulses--)  
//		{
//			Robot_hunting1(1580);	  
//		}	 		
		pointsth++;			                                                                  //直线上放了5块色块
}
void GotoLine(void)
{
	
	int tempa=0;
	switch(pointsth)	                                  // pointsth初始值为0
	{
		case 0:DownFirst();	                              //	搬运A点时选择
				break;
		case 1:DownSecond();                              //搬运B点时选着
				break;
		case 2:Downthird();                             	//搬运C点时选着
				break;
		case 3:DownFourth();                              //搬运E点时选着
				break;
		case 4:DownFiveth();	                            //搬运F点时选着
				break;
		default:
				break;
	}
	tempa++;
	tempa=0x22;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：CarryGpoint(void)
*  Description：把从G点搬运的色块从十字路口位置搬运回到直线上,详细过程参 CarryIpoint(void)
****************从十字路口位置搬运回到直线上***************
---------------------------------------------------------------------------------------------------------*/
void Gback(){
		pulses=8;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		//TurnRight_nDegree(80);     //后转120
		findLline();
		/*do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	循迹*/
		pulses=15;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}
	  TurnLeft_nDegree(80);//左转n度
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		pulses=8;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findRline();
				
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//判断是否到达第一个路口
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
			Robot_hunting1(1550);	  //直行
		}
	  TurnRight_nDegree(100);//右转
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		pulses=12;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findLline();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//判断是否到达第一个路口
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		pulses = 2;
		while(pulses--)  
		{
			Robot_hunting1(1580);	  
		}	 
}
	
void CarryGpoint(void)                              //从十字路口把G色块运到起始位置在返回到十字路口
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
					process=3;     //*****待改******待改***********待改******待改***********待改*********************待改************************8888
					return;
				}
		}while(distance > 4);  //机器朝着I点位置物块方向直走
		pulses=15;
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=13;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		//TurnRight_nDegree(80);     //后转120
		findLline();
		/*do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	循迹*/
		pulses=13;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}
		motor_motion1(1450,1550,25);//后退
	  TurnLeft_nDegree(80);//左转n度
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findRline();
		
		
		do
		{
			Robot_hunting1(1540);
		}while(QTIS != 15);//判断是否到达第一个路口
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryHpoint(void)
*  Description：把从H点搬运的色块从十字路口位置搬运回到直线上,详细过程参 CarryIpoint(void)
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
					process=3;     //*****待改******待改***********待改******待改***********待改*********************待改************************8888
					return;
				}
		}while(distance > 4);  //
		pulses=15;
		while(pulses--)
		{
			motor_motion2(1540, 1460);	  //直行
		}
		
	 //TurnLeft_nDegree(80);     //后转120  
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);//	循迹
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
			Robot_hunting1(1550);	  //直行
		}
		motor_motion1(1450,1550,25);//后退
	  TurnRight_nDegree(90);//右转
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS );
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findLline();
		
		
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//判断是否到达第一个路口
		
		pulses = LRp-2;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryCpoint(void)
*  Description：把C点位置上色块搬运回到直线上 ,详细过程参看搬运A点CarryApoint(void)
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
			Robot_hunting1(1550);	  //直行
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//第一个路口
		pulses = 15;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryDpoint(void)
*  Description：把D点位置上色块搬运回到直线上 ,详细过程参看搬运A点CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryDpoint(void)                              //从十字路口把A色块运到起始位置在返回到十字路口
{
		Dhavesome=0;
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}
		Turn_180();
		delay_nms(200);
		do
		{
			Robot_hunting1(1540);
		}while(QTIS !=15);//第一个路口
		pulses = 18;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryEpoint(void)
*  Description：把E点位置上色块搬运回到直线上 ,详细过程参看搬运A点CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryEpoint(void)                                     //从十字路口把A色块运到起始位置在返回到十字路口
{
	Ehavesome=0;
		do
		{
			Robot_hunting1(1550);
			
		}while(QTIS);
		pulses = 40;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//第一个路口
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
			Robot_hunting1(1550);	  //直行
		}
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//第一个路口
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryBpoint(void)
*  Description：把B点位置上色块搬运回到直线上 ,详细过程参看搬运A点CarryApoint(void)
---------------------------------------------------------------------------------------------------------*/ 
void CarryBpoint(void)                        //从十字路口把A色块运到起始位置在返回到十字路口
{
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}		
		Turn_180();
//		stop();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//第一个路口
		pulses = 20;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
*	Function name：CarryApoint(void)
*  Description：把A点位置上色块搬运回到直线上
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
			Robot_hunting1(1550);	  //直行
		}	
		Turn_180();	   //转弯180度
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);
		pulses = 15;
		while(pulses--)	//
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findRline();  //向右转弯到最近黑线上
		GotoLine();
		findsekuai++;
  if(findsekuai==5)
	{
		process=11;
		Idone=1;
	}
}
/*---------------------------------------------------------------------------------------------------------
*	Function name： RedCarry(void)
*  Description：把红色色块搬运到对应的红色区域上
---------------------------------------------------------------------------------------------------------*/

void goBackCenter(){
		pulses = 25;
		while(pulses--)
		{
			Robot_hunting1(1540);
		}//判断是否到达第一个路口
		Turn_180();//转弯180度	
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//判断是否到达第一个路口
		
		pulses = LRp;
		while(pulses--)
		{
			motor_motion2(1550, 1450);
		}			//直行
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
		}//判断是否到达第一个路口
		TurnLeft_nDegree(35);//左转n度
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=10;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findLline();  //向右转弯到最近黑线上
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>4 );
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		TurnLeft_nDegree(110);//左转n度
			pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1540, 1460);	  //直行
		}
		findRline();  //向右转弯到最近黑线上
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
		}//判断是否到达第一个路口
		TurnRight_nDegree(27);//左转n度
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(QTIS != 15);
		
		pulses=10;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
	
		findRline();  //向右转弯到最近黑线上
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>4 );
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		TurnRight_nDegree(79);//左转n度
		pulses=5;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while(!QTIS);
		
		pulses=13;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		findLline();  //向右转弯到最近黑线上
		process = 11;

}
/////////////////////////////////////////////////

void RedCarry(void)//搬红色色块到分数区    
{
	hadedone++;
	printf("hadedone:%d",hadedone);
		do
		{
			Robot_hunting1(1550);
		}while(QTIS);//当到达中间白点位置时就停止
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1530);	  //直行
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //直行
		}
		motor_motion1(1450,1550,25);//后退
	  Turn_180();//转弯180度
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//回到第一个路口
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		
		if(hadedone >= 6){
				process=12;
			printf("process:%d",process);
			return;
		}
//		if(hadedone >= 5){
//			//process=aim;
//			//goBackCenter(); //  F 点 和i 点 需要用的  切换 F I  和 G H 
//		//gotoG();
//			gotoH();
//				return;
//		}
		
		do{
			Robot_hunting(1540);	
			//delay_nms(50);
			distance = GetDis(GPIO_Pin_12,GPIO_Pin_13);
		}while(distance>5 );
		pulses =10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		delay_nms(50);
		Turn_180();
		process=11;
		
}
/*---------------------------------------------------------------------------------------------------------
*	Function name： WhiteCarry(void)
*  Description：把白色色块搬运到对应的区域上  ，详细讲解请参考函数RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void WhiteCarry(void)//搬白色色块到分数区
{		
	hadedone++;
	printf("hadedone:%d",hadedone);
	  TurnLeft_nDegree(10);
		findLline();                              //左偏寻找下一条黑线  也就是白点对应的线	
		delay_nms(200);
	  do
		{
			Robot_hunting1(1550);
		}while(QTIS);                             //当第一次检测到白点跳出循环0
		pulses = 35;
		while(pulses--)
		{
			Robot_hunting1(1530);	  //直行
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //直行
		}	
		
		motor_motion1(1450,1550,25);//后退
		Turn_180();                                //旋转180度   可优化修改
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);                       //第一个路口
		pulses = 20;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
		pulses =10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		delay_nms(50);
		Turn_180();
		process=11;
		
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：YellowCarry(void)
*  Description：把黄色色块搬运到对应的区域上  ，详细讲解请参考函数RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void YellowCarry(void)//搬黄色色块到分数区
{
	hadedone++;
	printf("hadedone:%d",hadedone);

//		if(QTTS == )
		TurnLeft_nDegree(8);//左转n度
		
		findLline();                        //可能需要优化  暂定  
		delay_nms(200);	
		findLline();            //寻找右边下一条最近的直线
	
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
			Robot_hunting1(1530);	  //直行
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //直行
		}	
		motor_motion1(1450,1550,25);//后退
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS !=15);//第一个路口
		pulses = 15;           //这个值是需要修改的
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
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：BlackCarry(void)
*  Description：把黑色色块搬运到对应的区域上  ，详细讲解请参考函数RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void BlackCarry(void)//搬黑色色块到分数区
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
			Robot_hunting1(1520);	  //直行
		}
		
		/*pulses = goal-1;
		while( (GPIO_ReadInputData(GPIOE)&0x08000) || pulses--)
		{
			Robot_hunting1(1550);	  //直行
		}	*/
		
		while( !(GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1502);	  //直行
		}	
	
		motor_motion1(1450,1550,25);//后退
		Turn_180();
		do
		{
			Robot_hunting1(1550);
		}while(QTIS != 15);//第一个路口
		pulses = 18;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：BlueCarry(void)
*  Description：把蓝色色块搬运到对应的区域上  ，详细讲解请参考函数RedCarry(void)
---------------------------------------------------------------------------------------------------------*/
void BlueCarry(void)//搬蓝色色块到分数区
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
			Robot_hunting1(1502);	  //直行
		}	
		delay_nms(200);
		
	/*	pulses = 5;
		while(pulses--)
		{
			delay_nms(20);
			Robot_hunting1(1502);	  //直行
		}*/
		while( (GPIO_ReadInputData(GPIOE)&0x08000))
		{
			delay_nms(50);
			Robot_hunting1(1501);	  //直行
		}	
		delay_nms(200);
		motor_motion1(1450,1550,18);//后退
		Turn_180();
		 do
	    {
	       Robot_hunting1(1540);
    	}while(QTIS != 15);//第二个路口
		pulses = 14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
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
		pulses = 10;                      //前冲步数   可修改  优化
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		delay_nms(50);
		Turn_180();
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：StartCarryToScore(void)
*  Description：开始搬运直线上物块
---------------------------------------------------------------------------------------------------------*/
void StartCarryToScore(void)
{			
	count = 1;                // 每次从1开始 
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
			 	motor_motion2(1550, 1450);	 //直行
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
			//color = Robot_checkColor();//识别颜色
			color =xm_colourjudge(pcolor);
			count++;
//			if(color == 4)
					//color = Robot_checkColor();//识别颜色
//					color =xm_colourjudge(pcolor);
			switch(color)//颜色选着
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
*	Function name：CarryIpoint(void)
*   Description：把I点位置色块搬运到对应的区域上  
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
						motor_motion2(1550, 1450);	  //直行
					}
					findLline();
					do
					{
						Robot_hunting1(1550);
					}while(QTIS != 15);//判断是否到达第一个路口
					motor_motion1(1550, 1450,15);
					TurnRight_nDegree(50);       //r->180
					pulses=25;
					while(pulses--)
					{
						motor_motion2(1550, 1450);	  //直行
					}
					process = 13;
					return ;
			}
		}while(distance > 4);  //机器朝着I点位置物块方向直走，直到距离小于3
		pulses=25;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		
		TurnLeft_nDegree(90);     //后转150  
		do
		{
			motor_motion2(1550, 1450);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}
		while(QTIS != 15);//	循迹
		motor_motion1(1550, 1450,12);
		findLline();
		//TurnLeft_nDegree(55);
		
		process=11;
}
/*---------------------------------------------------------------------------------------------------------
*	Function name：CarryFpoint(void)
*   Description：把F点位置色块搬运到对应的区域上 ,详细讲解请参看函数CarryIpoint(void)   
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
							motor_motion2(1550, 1450);	  //直行
						}
						findRline();
						do
						{
							Robot_hunting1(1550);
						}while(QTIS != 15);//判断是否到达第一个路口
						motor_motion1(1550, 1450,25);
						TurnLeft_nDegree(30);       //r->180
						pulses=25;
						while(pulses--)
						{
							motor_motion2(1550, 1450);	  //直行		
						}
						process = 13;
						return ;
					}
		}while(distance > 5);  //机器朝着F点位置物块方向直走，直到距离小于3
		pulses=25;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		
		TurnRight_nDegree(90);       //r->180
		do{
			motor_motion2(1550, 1445);	  //直行
			QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
		}while( !QTIS );
		pulses=14;
		while(pulses--)
		{
			motor_motion2(1550, 1450);	  //直行
		}
		//TurnRight_nDegree(55);//右转n度
		findRline();
			process=11;
	}
}

/*******************************************************************************
  * GoStart()
  * Description计算白色色块RGB值到达255时，对应时间数分别存储在数组pColor[]里
  ********************************************************************************/
 void GoStart(void)
 {
	 do
	{
		Robot_hunting1(1550);
	}while(QTIS != 15);//判断是否到达第一个路口
	
	pulses = 15;
	while(pulses--)   //走出出发区
	{
		motor_motion2(1650, 1350);	  //直行
	}	 
	
	do
	{
		Robot_hunting1(1550);
	}while(QTIS != 15);//判断是否到达第一个路口
	
	pulses = LRp-2;
	while(pulses--)
	{
		motor_motion2(1550, 1450);	  //直行
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
		Robot_hunting1(1560);	  //直行
	}
	do
	{
		Robot_hunting1(1560);
		QTIS = (GPIO_ReadInputData(GPIOE)&0x0f);
	}while(QTIS != 15);//判断是否到达第一个路口
	pulses = 40;
	while(pulses--)
	{
		motor_motion2(1550, 1450);	  //直行
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
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;//|GPIO_Pin_6|GPIO_Pin_7;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE4,3

//	//初始化 WK_UP-->GPIOA.0	  下拉输入
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0设置成输入，默认下拉	  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟



//   //GPIOE.3	  中断线以及中断初始化配置 下降沿触发 //KEY1
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
//  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

//   //GPIOE.4	  中断线以及中断初始化配置  下降沿触发	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器



   //GPIOE.5	  中断线以及中断初始化配置  下降沿触发	//KEY0
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	 //GPIOE.6	  中断线以及中断初始化配置  下降沿触发	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line6;
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	 //GPIOE.7	  中断线以及中断初始化配置  下降沿触发	//KEY0
//  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
//  	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
//  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


//   //GPIOA.0	  中断线以及中断初始化配置 上升沿触发 PA0  WK_UP
// 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); 

//  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
//  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键WK_UP所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//子优先级3
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure); 

//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//使能按键KEY1所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

//  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键KEY0所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY0所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
//	
//		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY0所在的外部中断通道
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
//  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY0所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
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
						Peripheral_Init();//各模块初始化配置//
						TIM5_Int_Init(0xfff0-1,720-1);
						delay_init();
						EXTIX_Init();
   					process =3;	 //将C/B/A/D/E/G/H点有的色块搬到开始线上，如果比赛，只需要修改该参数即可  程序首先从case11开始
//						process =8;	 //将C/B/A/D/E/G/H点有的色块搬到开始线上，如果比赛，只需要修改该参数即可  程序首先从case11开始
						led=1;
						delay_nms(1000);
						led=0;
						delay_nms(1000);
						xm_TCS230_WhiteBalance(pcolor);  //白平衡
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
						//color = Robot_checkColor();//识别颜色
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
//							//color = Robot_checkColor();//识别颜色
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
								delay_nms(20);	//输出一定数量的PWM波形
								//i++;	
							}while(true);*/
		led=0;
		delay_nms(1000);
		delay_nms(1000);
//		process = 8;
		GoStart();		 //走出开始区域
//	process = 11;
		while(1)
		{ 
		//printf("process:%d\n",process);
   //     motor_motion2(1500, 1500); 		
			
			
	
//			
//			process = 9;
			switch(process)	 //开始时process=1
			{
				/**************一般1~10选着用不到********************************/
				case 1:	CCheck();
				        CarryCpoint();
						break;//超声波检测c点
				
				case 2:	BCheck();
				        CarryBpoint();
//								while(1);
						break;//超声波检测b点
				
				case 3:	ACheck();
				        CarryApoint();
								
						break;//超声波检测a点
				
				case 4:	ICheck();
						CarryIpoint();
						break;//超声波检测i点    //i点为颜色识别跳出程序所在之处
				
				case 5:	HCheck();
				        CarryHpoint();
						break;//超声波检测h点
				
				case 6: GCheck();
				        CarryGpoint();
						break;//超声波检测g点
						
				case 7:	FCheck();
				        CarryFpoint();
						break;//超声波检测f点
						
				case 8:	ECheck();
				        CarryEpoint();
						break;//超声波检测e点
						
				case 9: DCheck();
				        CarryDpoint();
						break;//超声波检测d点	
						
				//case 10:turn_back();break;
				case 11:StartCarryToScore();
						break;//将C/B/A/D/E/G/H点有的色块搬计分区
						
				case 12:moveBack();
						break;
				default:break;
			}
//
//			while(1);
	}
}

