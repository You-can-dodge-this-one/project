

#include "stm32f10x.h"
//#include "delay.h"
#include <stdio.h>
//#include "cfg.h"
#include "tcs.h"
#include "stm32f10x_exti.h"
#include <stdio.h>
//#include "main.h"
//#include "timer.h"
#include <string.h>

#include "timer.h"

/* -------------------------��ɫ���������ŵ�ƽ���ƶ���--------------------------------------*/
#define S0_Write_1()     GPIO_SetBits(GPIOE,GPIO_Pin_15) //д1
#define S0_Write_0()     GPIO_ResetBits(GPIOE,GPIO_Pin_15) //д0
#define S1_Write_1()     GPIO_SetBits(GPIOC,GPIO_Pin_10) 
#define S1_Write_0()     GPIO_ResetBits(GPIOC,GPIO_Pin_10)
#define S2_Write_1()     GPIO_SetBits(GPIOC,GPIO_Pin_11)
#define S2_Write_0()     GPIO_ResetBits(GPIOC,GPIO_Pin_11)
#define S3_Write_1()     GPIO_SetBits(GPIOC,GPIO_Pin_12)
#define S3_Write_0()     GPIO_ResetBits(GPIOC,GPIO_Pin_12)
#define LED_Write_1()    GPIO_SetBits(GPIOC,GPIO_Pin_13)
#define LED_Write_0()    GPIO_ResetBits(GPIOC,GPIO_Pin_13)

/*���ļ�Ϊ��ɫ�������ĺ���*/


#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����1
#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����3(WK_UP) 

 

#define KEY0_PRES 	1	//KEY0����
#define KEY1_PRES	  2	//KEY1����
#define WKUP_PRES   3	//KEY_UP����(��WK_UP/KEY_UP)



volatile u16 time5_conut=0;  //����
volatile u16 count_tcs230=0;

extern int speak_len,speak_i,s_i;
extern int speak_len1;//(sizeof(buf_xm)/sizeof(buf_xm[0]));
extern char buf1[30];//\[b1] ��ӭ���٣����




u16 xm_RGB[3]={0};
int colour;
int red_time=0x00;
int blue_time=0x00;
int green_time=0x00;
int tim5_interruput_flag=0x00;
int tim7_interruput_flag=0x00;

int red_num=0x00;
int blue_num=0x00;
int green_num=0x00; 

int colour_flag=0;
char buf1[30]={"�Լ� "};//\[b1] ��ӭ���٣����
int speak_len1=0x00;//(sizeof(buf_xm)/sizeof(buf_xm[0]));

/************************************************
  * @brief  TCS230_WhiteBalance Function.
  * @param  None
  * @retval None
  ***********************************************/

void xm_TCS230_WhiteBalance(u16 pColor[3])
{
//	S0_Write_0();	
//	
//	S1_Write_1();//���Ƶ��Ϊ(1/50)*500KHz=10KHz
//	
//	LED_Write_1();//��LED
//	
//	S2_Write_0();	S3_Write_0();//ѡ���ɫ
	////////////////////////////////////
	//balance red
	/////////////////////////////
	colour_flag=1;
	
	
	s0=1;
	s1=1;//����Ƶ
	
	led=1;
	
	s2=0;//��ɫ�˾�
	s3=0;

	time5_conut=0;
	
	
	//TIM5_Int_Init(0xfff0,72-1);
	
	count_tcs230=0;//count tcs230 plus 
	
	
//	TIM_Cmd(TIM7, DISABLE);
//	TIM_SetCounter(TIM7, 0);
//	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
//	TIM_Cmd(TIM7, ENABLE);
//			
//	EXTI->IMR |= 1<<5;//enable exit5 interrupt
//	
//	while(count_tcs230<255);
//	
////	
////	TIM_Cmd(TIM5, DISABLE);

////	EXTI->IMR &= 0<<5;//disable exit5 interrupt
//	
//	time5_conut=TIM_GetCounter(TIM7);
	
	
	
	
	
	TIM_Cmd(TIM5, DISABLE);
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
			
	EXTI->IMR |= 1<<5;//enable exit5 interrupt
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(count_tcs230<255);
	
//	
//	TIM_Cmd(TIM5, DISABLE);

//	EXTI->IMR &= 0<<5;//disable exit5 interrupt
	
	time5_conut=TIM_GetCounter(TIM5);
	
	
	
	

	
	pColor[0] = time5_conut;
	red_time=time5_conut;
	////////////////////////////////////
	//balance blue
	/////////////////////////////
	s2=0;
	s3=1;//ѡ����ɫ

	time5_conut=0;
	
	count_tcs230=0;//count tcs230 plus 
	
	TIM_Cmd(TIM5, DISABLE);
	
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
			
	EXTI->IMR |= 1<<5;//enable exit5 interrupt
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(count_tcs230<255);
	
//	TIM_Cmd(TIM5, DISABLE);

//	EXTI->IMR &= 0<<5;//disable exit5 interrupt
	
	time5_conut=TIM_GetCounter(TIM5);
	

	
	pColor[1] = time5_conut;
	blue_time=time5_conut;
	
	////////////////////////////////////
	//balance green
	/////////////////////////////
	s2=1;
	s3=1;//ѡ����ɫ
	//

	time5_conut=0;
	
	count_tcs230=0;//count tcs230 plus 
	
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
			
	EXTI->IMR |= 1<<5;//enable exit5 interrupt
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(count_tcs230<255);
	
//	TIM_Cmd(TIM5, DISABLE);

//	EXTI->IMR &= 0<<5;//disable exit5 interrupt
	
	time5_conut=TIM_GetCounter(TIM5);
	
	pColor[2] = time5_conut;
	green_time=time5_conut;
	
	//LED_Write_0();//�ر�LED	
	
	printf("Red: %d  Blue: %d  Green: %d\n",pColor[0],pColor[1],pColor[2]);
	
	colour_flag=0;
	
}

u16 xm_colourjudge(u16 pRGB[3]) //������ɫ�����ж�ʶ����ɫ
{	
//	int colour;

//	S0_Write_0();	
//	S1_Write_1();//���Ƶ��Ϊ(1/50)*500KHz=10KHz	
//	LED_Write_1();//��LED
	
	TIM5_Int_Init(0xfff0-1,720-1);
	
	colour_flag=1;
	
	led=1;
	
	s0=1;
	s1=1;//1:1
	///////////////////////////////////////
	//���Ժ���ɫ��
	//////////////////////////////////////
	s2=0;
	s3=0;//ѡ���ɫ
	
//	S2_Write_0();	
//	
//	S3_Write_0();
	
	time5_conut=0;
	
	count_tcs230=0;//count tcs230 plus 
	tim5_interruput_flag=0x00;
	
	TIM_SetAutoreload(TIM5,red_time);
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
	
	EXTI->IMR |= 1<<5;
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(tim5_interruput_flag==0x00);
	
	tim5_interruput_flag=0x00;
	
	
	TIM_Cmd(TIM5, DISABLE);
	
	EXTI->IMR &= 0<<5;
	red_num=count_tcs230;
	///////////////////////////////////////
	//��������ɫ��
	//////////////////////////////////////
	
	s2=0;
	s3=1;//ѡ����ɫ
	
//	S2_Write_0();	
//	
//	S3_Write_0();
	
	time5_conut=0;
	
	count_tcs230=0;//count tcs230 plus 
	tim5_interruput_flag=0x00;
	
	TIM_SetAutoreload(TIM5,blue_time);
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
	
	EXTI->IMR |= 1<<5;
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(tim5_interruput_flag==0x00);
	tim5_interruput_flag=0x00;
	
	
	TIM_Cmd(TIM5, DISABLE);
	
	EXTI->IMR &= 0<<5;
	blue_num=count_tcs230;
	///////////////////////////////////////
	//��������ɫ��
	//////////////////////////////////////
	
	s2=1;
	s3=1;//ѡ����ɫ
	
//	S2_Write_0();	
//	
//	S3_Write_0();
	
	time5_conut=0;
	
	count_tcs230=0;//count tcs230 plus 
	tim5_interruput_flag=0x00;
	
	TIM_SetAutoreload(TIM5,green_time);
	TIM_SetCounter(TIM5, 0);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
	TIM_Cmd(TIM5, ENABLE);
	
	EXTI->IMR |= 1<<5;
	EXTI->RTSR |= 1<<5;//enable raise interrupt
	
	while(tim5_interruput_flag==0x00);
	tim5_interruput_flag=0x00;
	
	
	TIM_Cmd(TIM5, DISABLE);
	
	EXTI->IMR &= 0<<5;
	green_num=count_tcs230;
	
	led=0;
	/////////////////////////////
	//������ɫ����
	////////////////////////////
//	sprintf(buf1,"�����������������");
//	speak_len1=strlen(( const char *)buf1);
//	//speak_context((u8*)buf1,speak_len1);
//	delay_ms(1000);
//	delay_ms(1000);
//	
//	sprintf(buf1,"��ɫ %d ",red_num);
//	speak_len1=strlen(( const char *)buf1);
//	//speak_context((u8*)buf1,speak_len1);
//	delay_ms(1000);
//	delay_ms(1000);
//	
//	
//	
//	sprintf(buf1,"��ɫ %d",blue_num);
//	speak_len1=strlen(( const char *)buf1);
//	//speak_context((u8*)buf1,speak_len1);
//	delay_ms(1000);
//	delay_ms(1000);
//	
//	sprintf(buf1,"��ɫ %d",green_num);
//	speak_len1=strlen(( const char *)buf1);
//	//speak_context((u8*)buf1,speak_len1);
//	delay_ms(1000);
//	delay_ms(1000);
		
	
		
		/////////////////////////////
		//������ɫ����
		////////////////////////////
		
		LED_Write_0();//�ر�LED		
		//printf("Red: %d  Blue: %d  Green: %d\n",xm_RGB[0],//RGB[1],//RGB[2]);//red_num
		printf("Red: %d  Blue: %d  Green: %d\n",red_num,blue_num,green_num);
		
		
//		sprintf(buf1,"��ʼ�ж���ɫ");
//		speak_len1=strlen(( const char *)buf1);
//		//speak_context((u8*)buf1,speak_len1);
//		delay_ms(1000);
	
	///////////////////////////////
	//�ж���ɫ
	///////////////////////////////////////
	if(200<red_num&&red_num<300&&200<blue_num&&blue_num<300&&200<green_num)//white
	{
		colour=white;
		printf("colour is white\r\n");
		
		sprintf(buf1,"��ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	}
	else if(20<red_num&&red_num<60&&20<blue_num&&blue_num<50&&20<green_num&&green_num<68)//black
	{ 	
		colour=black;
		printf("colour is black\r\n");
		
		sprintf(buf1,"��ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	}
	 else if(180<red_num&&red_num<300&&80<blue_num&&blue_num<150&&150<green_num&&green_num<450)//yellow
	{
		colour=yellow;
		printf("colour is yellow\r\n");
		
		sprintf(buf1,"��ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	
	}
	 else if(150<red_num&&red_num<200&&30<blue_num&&blue_num<110&&25<green_num&&green_num<100)// red
	{
		colour=red;
		printf("colour is red\r\n");
		sprintf(buf1,"��ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	
	}
	  else if(10<red_num&&red_num<35&&40<blue_num&&blue_num<63&&10<green_num&&green_num<40)// blue
	{
		colour=blue;
		printf("colour is blue\r\n");
		sprintf(buf1,"��ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	
	}
	else
	{
		colour=other;	
		printf("NULL\r\n");
		sprintf(buf1,"����ɫ");
		speak_len1=strlen(( const char *)buf1);
		//speak_context((u8*)buf1,speak_len1);	
	
	}
	//delay_ms(300);
	
	
//	while(colour==other) //���²�����ɫ
//	{
//		s0=1;
//		s1=1;//1:1
//		led=1;
//		///////////////////////////////////////
//		//���Ժ���ɫ��
//		//////////////////////////////////////
//		s2=0;
//		s3=0;//ѡ���ɫ
//		
//	//	S2_Write_0();	
//	//	
//	//	S3_Write_0();
//		
//		time5_conut=0;
//		
//		count_tcs230=0;//count tcs230 plus 
//		tim5_interruput_flag=0x00;
//		
//		TIM_SetAutoreload(TIM5,red_time);
//		TIM_SetCounter(TIM5, 0);
//		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
//		TIM_Cmd(TIM5, ENABLE);
//		
//		EXTI->IMR |= 1<<5;
//		
//		while(tim5_interruput_flag==0x00);
//		
//		tim5_interruput_flag=0x00;
//		
//		
//		TIM_Cmd(TIM5, DISABLE);
//		
//		EXTI->IMR &= 0<<5;
//		red_num=count_tcs230;
//		///////////////////////////////////////
//		//��������ɫ��
//		//////////////////////////////////////
//		
//		s2=0;
//		s3=1;//ѡ����ɫ
//		
//	//	S2_Write_0();	
//	//	
//	//	S3_Write_0();
//		
//		time5_conut=0;
//		
//		count_tcs230=0;//count tcs230 plus 
//		tim5_interruput_flag=0x00;
//		
//		TIM_SetAutoreload(TIM5,blue_time);
//		TIM_SetCounter(TIM5, 0);
//		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
//		TIM_Cmd(TIM5, ENABLE);
//		
//		EXTI->IMR |= 1<<5;
//		
//		while(tim5_interruput_flag==0x00);
//		tim5_interruput_flag=0x00;
//		
//		
//		TIM_Cmd(TIM5, DISABLE);
//		
//		EXTI->IMR &= 0<<5;
//		blue_num=count_tcs230;
//		///////////////////////////////////////
//		//��������ɫ��
//		//////////////////////////////////////
//		
//		s2=1;
//		s3=1;//ѡ����ɫ
//		
//		time5_conut=0;
//		
//		count_tcs230=0;//count tcs230 plus 
//		tim5_interruput_flag=0x00;
//		
//		TIM_SetAutoreload(TIM5,green_time);
//		TIM_SetCounter(TIM5, 0);
//		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�
//		TIM_Cmd(TIM5, ENABLE);
//		
//		EXTI->IMR |= 1<<5;
//		
//		while(tim5_interruput_flag==0x00);
//		tim5_interruput_flag=0x00;
//		
//		
//		TIM_Cmd(TIM5, DISABLE);
//		
//		EXTI->IMR &= 0<<5;
//		green_num=count_tcs230;
//		
//		led=0;
//		
//		///////////////////////////////////////
////		sprintf(buf1,"�����������������");
////		speak_len1=strlen(( const char *)buf1);
////		//speak_context((u8*)buf1,speak_len1);
////		delay_ms(1000);
////		delay_ms(1000);
////		
////		sprintf(buf1,"��ɫ %d ",red_num);
////		speak_len1=strlen(( const char *)buf1);
////		//speak_context((u8*)buf1,speak_len1);
////		delay_ms(1000);
////		delay_ms(1000);
////		
////		
////		
////		sprintf(buf1,"��ɫ %d",blue_num);
////		speak_len1=strlen(( const char *)buf1);
////		//speak_context((u8*)buf1,speak_len1);
////		delay_ms(1000);
////		delay_ms(1000);
////		
////		sprintf(buf1,"��ɫ %d",green_num);
////		speak_len1=strlen(( const char *)buf1);
////		//speak_context((u8*)buf1,speak_len1);
////		delay_ms(1000);
////		delay_ms(1000);
//			
//		
//	
//		
//		/////////////////////////////
//		//������ɫ����
//		////////////////////////////
//		
//		LED_Write_0();//�ر�LED		
//		//printf("Red: %d  Blue: %d  Green: %d\n",//RGB[0],//RGB[1],//RGB[2]);//red_num
//		printf("Red: %d  Blue: %d  Green: %d\n",red_num,blue_num,green_num);
//		
//		
////		sprintf(buf1,"��ʼ�ж���ɫ");
////		speak_len1=strlen(( const char *)buf1);
////		//speak_context((u8*)buf1,speak_len1);
////		delay_ms(1000);
//		
//			///////////////////////////////
//		//�ж���ɫ
//		///////////////////////////////////////
//		if(red_num>200&&blue_num>200&&green_num>200)//white
//		{
//			colour=white;
//			printf("colour is white\r\n");
//			
//			sprintf(buf1,"��ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		}
//		else if(25<red_num&&red_num<35&&15<blue_num&&blue_num<28&&20<green_num&&green_num<30)//black
//		{ 	
//			colour=black;
//			printf("colour is black\r\n");
//			
//			sprintf(buf1,"��ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		}
//		 else if(185<red_num&&red_num<400&&60<blue_num&&blue_num<160&&130<green_num&&green_num<270)//yellow
//		{
//			colour=yellow;
//			printf("colour is yellow\r\n");
//			
//			sprintf(buf1,"��ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		
//		}
//		else if(100<red_num&&red_num<600&&20<blue_num&&blue_num<100&&20<green_num&&green_num<50)// red
//		{
//			colour=red;
//			printf("colour is red\r\n");
//			sprintf(buf1,"��ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		
//		}
//		 else if(15<red_num&&red_num<25&&28<blue_num&&blue_num<40&&10<green_num&&green_num<25)// blue
//		{
//			colour=blue;
//			printf("colour is blue\r\n");
//			sprintf(buf1,"��ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		
//		}
//		else
//		{
//			colour=other;	
//			printf("NULL\r\n");
//			sprintf(buf1,"����ɫ");
//			speak_len1=strlen(( const char *)buf1);
//			//speak_context((u8*)buf1,speak_len1);	
//		
//		}
//		//delay_ms(300);
//		
//		
//		
//	}
//	
//	
//	
//	
//	
	
	
	colour_flag=0;
		
	return colour;

}





//void EXTI9_5_IRQHandler(void)
//{
//	if( EXTI_GetITStatus(EXTI_Line5) != RESET) 
//	{
//		count_tcs230++;
//		tim5_interruput_flag=0x01;
//	}
//	EXTI_ClearITPendingBit(EXTI_Line5);
//	
//}

