#include "stm32f10x.h"
#include "sys.h" 
#define TRIG PAout(3) //����˿�
#define ECHO PAin(2)  //����˿�
int overcount=0;      //��¼��ʱ���������
int length;

float Pitch,Roll,Yaw;						//�Ƕ�
short gyrox,gyroy,gyroz;				//������--���ٶ�
short aacx,aacy,aacz;						//���ٶ�
int Encoder_Left,Encoder_Right;	//���������ݣ��ٶȣ�

int PWM_MAX=5000,PWM_MIN=-5000;	//PWM�޷�����
int MOTO1,MOTO2;								//���װ�ر���

extern int Vertical_out,Velocity_out,Turn_out;
/*
//void TIM3_Int_Init()
//{
//		GPIO_InitTypeDef GPIO_InitStruct;
//		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//		NVIC_InitTypeDef NVIC_InitStructure;

//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//			
//		
//		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//����Ϊ�������
//		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
//		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
//		GPIO_Init(GPIOA,&GPIO_InitStruct);

//		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
//		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
//		GPIO_Init(GPIOA,&GPIO_InitStruct);
//	
//		//��ʱ��3��ʼ��
//		TIM_TimeBaseStructure.TIM_Period = 999; //ARR
//		TIM_TimeBaseStructure.TIM_Prescaler =7199; //PSC
//		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
//		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

//		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );//ʹ���ƶ�TIM3�жϣ���������ж�
//		//�ж�����
//		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);

//		TIM_Cmd(TIM3, DISABLE);
//}
//��ຯ��
//int Senor_Using() 
//{
//		unsigned int sum=0;
//		unsigned int tim;
//		unsigned int i=0;
//		unsigned int length;
//		u16 cnt_i=0;
//		while(i!=3)        
//		{
//			TRIG=1;          
//			delay_us(20);    
//			TRIG=0;          
//			cnt_i=0;
//			while(ECHO==0)
//			{
//				cnt_i++;
//				delay_us(1);
//				if(cnt_i>30000)
//				{	TRIG=1;          
//					delay_us(20);    
//					TRIG=0;cnt_i=0;          
//				}
//			}
//			TIM_Cmd(TIM3,ENABLE);
//			
//			i+=1;                     
//			
//			while(ECHO==1);	
//			TIM_Cmd(TIM3,DISABLE);    
//			
//			tim=TIM_GetCounter(TIM3);         
//			length=(tim*100)/58.0; 
//			sum=length+sum;
//			TIM3->CNT=0; 
//			overcount=0; 
//			delay_ms(100);
//		}
//		length=sum/3;
//		return length; 
//}

*/
void peripheral_configure()
{
	delay_init();
	NVIC_Config();//�ж����ȼ�����
	OLED_Init();
	OLED_Clear();
	
	while(MPU_Init());//��ʼ��MPU6050
	while(mpu_dmp_init());
	MPU6050_EXTI_Init();
	
	Encoder_TIM2_Init();
	Encoder_TIM4_Init();
	Motor_Init();
	PWM_Init_TIM1(0,7199);
	//	TIM3_Int_Init();//��������ʱ���ĳ�ʼ��
}


u8 lock=1;
int main(void)	
{
	peripheral_configure();
	OLED_Num3(4,5,1);
	OLED_ShowString(0,3,"angle:",12);
	while(1)
	{
		OLED_Num3(4,5,1);
		OLED_ShowString(0,3,"angle:",12);//�Ƕ�
		Load(0,0);
	}
	OLED_ShowString(0,3,"angle:",12);//�Ƕ�
	OLED_ShowString(0,4,"distance:",12);//����
	OLED_ShowString(0,5,"speed:",12);//�ٶ�
  while(1)	
	{
		Load(	0,0);
//	length = Senor_Using(); //���ò�ຯ��
		OLED_Float(3,70,Pitch,1);
		OLED_Float(4,64,length,1);
		OLED_Num3(8,5,(int)((Encoder_Left+Encoder_Right)*3.3));//��ʾ����ٶ�
	} 	
}

void TIM3_IRQHandler(void)//�������ж�
{
		if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) //����Ƿ���TIM3�ж�
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update );   //����жϸ��±�־
			overcount++;
		}
}
