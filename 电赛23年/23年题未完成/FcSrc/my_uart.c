//////////////////////////////////////////////////////////////////////
/*
串口通信
*/
//////////////////////////////////////////////////////////////////////

#include "my_uart.h"
#include "Drv_Uart.h"
#include "get_data.h"
#include "mode.h"
#include "mbotLinuxUsart.h"
#include "Ano_Scheduler.h"
//#include "KEY.h"

int16_t test_data_1,test_data_2;
int16_t OpenMV_data_0,OpenMV_data_1,OpenMV_data_2;

int16_t OpenMV_help_fix_x,OpenMV_help_fix_y,new_fire_x,new_fire_y,fire_x,fire_y;

unsigned char car_data_0,car_data_1;
//接收f1 0xAA 火源标志位 舵机完成标志位 火源x坐标 火源y坐标 0xDD
void MY_urst1_Receive(uint8_t data)
{
	static int data_cnt=0;
	static uint8_t  data_0,data_1,data_2,data_3,data_4,data_5;
	if((data_cnt==0)&&(data==0xAA)){data_cnt=1;data_0=0xAA;}//判断帧头
	else if(data_cnt==1){data_1=data;data_cnt=2;}
	else if(data_cnt==2){data_2=data;data_cnt=3;}
	else if(data_cnt==3){data_3=data;data_cnt=4;}
	else if(data_cnt==4){data_4=data;data_cnt=5;}
	else if(data_cnt==5){data_5=data;data_cnt=0;}//帧尾
	else data_cnt=0;
		//解包,校验
	if((data_0==0xAA)&&(data_5==0xDD))//获取校验位
	{
		Send_fire_flag = data_1;//获取火源标志位
		servo_flag = data_2;//获取舵机完成标志位
		fire_x = data_3;//火源x坐标
		fire_y = data_4;//火源y坐标
	}
}
//暂时不需要
void MY_urst1_car_Receive(uint8_t data)///////////////////////////////////////////////////////////////
{
	static int data_cnt=0;
		
	if((data_cnt==0)&&(data==0xAA))data_cnt=1;
	else if(data_cnt==1){car_data_0=data;data_cnt=2;}
	else if(data_cnt==2){dis_x=data;data_cnt=3;}
	else if(data_cnt==3){dis_y = data;data_cnt=4;}
	else if(data_cnt==4){fire_x = data;data_cnt=5;}
	else if(data_cnt==5){fire_y = data;data_cnt=6;}
	else data_cnt=0;
	
	if(car_data_0==22){
	switch(car_data_1)
	{
		case 1:{fire_x=60;fire_y=80;}break;
		case 2:{fire_x=220;fire_y=110;}break;
		case 3:{fire_x=330;fire_y=80;}break;
		case 4:{fire_x=80;fire_y=260;}break;
		case 5:{fire_x=230;fire_y=200;}break;
		case 6:{fire_x=330;fire_y=200;}break;
	}
}
}
//发的是uchr
void MY_urst1_Send_radar(void)//发送火源数据
{
	//帧头，舵机是否标志位，雷达数据x，y，帧尾
	u8 Buf[]={0x0A,servo_flag,radar_data_1/10,radar_data_2/10,0x0D};//消息包封装
	DrvUart1SendBuf(Buf,sizeof(Buf));
}
//不需要
void MY_urst2_Send(int to_OpenMV_data_0)
{
	u8 Buf[]={to_OpenMV_data_0,0x0A};
	DrvUart2SendBuf(Buf,sizeof(Buf));
}

void MY_urst2_Receive(uint8_t data)
{
	static int data_cnt=0;
	static uint8_t  data_1,data_2,data_3,data_4,data_5;
	if((data_cnt==0)&&(data==0x0A))data_cnt=1;
	else if(data_cnt==1){data_1=data;data_cnt=2;}
	else if(data_cnt==2){data_2=data;data_cnt=3;}
	else if(data_cnt==3){data_3=data;data_cnt=4;}
	else if(data_cnt==4){data_4=data;data_cnt=5;}
	else if(data_cnt==5){data_5=data;data_cnt=0;}
	else data_cnt=0;
	OpenMV_data_0 = data_1;
	
	if(OpenMV_data_0!=9)
	{
		int mode;
		mode=dis_group[1][dis_target_ref];
//		if(KEY_2!=0)mode=KEY_2;
		if(OpenMV_data_0==mode)//只接收对应模式的数据
		{
			OpenMV_data_1 = data_3;
			OpenMV_data_1<<=8;
			OpenMV_data_1 |=data_2;
			OpenMV_data_2 = data_5;
			OpenMV_data_2<<=8;
			OpenMV_data_2 |=data_4;
			
			
			if(mode==1)//黑色方块
			{
				OpenMV_data_1=-OpenMV_data_1/240.0*hight;
				OpenMV_data_2=OpenMV_data_2/240.0*hight;
			}
			
			
			
			
			
			
			
//			//OpenMV换算为实际距离,普通镜头
//			OpenMV_data_1=(OpenMV_data_1-60)/120.0*hight;
//			OpenMV_data_2=(OpenMV_data_2-80)/160.0*hight*4.0/3.0;
//			//OpenMV换算为实际距离,广角镜头
//			OpenMV_data_1=(OpenMV_data_1-60)/120.0*hight*5.0/3.0;
//			OpenMV_data_2=(OpenMV_data_2-80)/160.0*hight*5.0/3.0*4.0/3.0;
		}
	}
	else
	{
		OpenMV_data_1=0;
		OpenMV_data_2=0;
	}
}

