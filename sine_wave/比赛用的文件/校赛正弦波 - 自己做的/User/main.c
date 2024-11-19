#include "Tim.h"
#include "delay.h"
#include "sys.h"
#include "dma.h"
#include "dac.h"
#include "key.h"
#include "lcd.h"
#include "dac.h"
#include "math.h"


//输入显示屏的字符串变量,必须是char类型不能是unsigned char型
u16 SineWave[100]; 
#define pi  3.1415926535
u16 nums=100;
signed int fre=10;//频率,有符号整形
u8 fre_flag = 0;//频率变化标志位
u16 arr;
u8 key_val,key_old,key_down;
u8 key_mode=0;//初始状态为10hz模式


void creat_sin(uint16_t maxval,uint16_t samples)//发生正弦波
{
	uint8_t i;
	float digltal_value;//数字量存储到SineWave_Value数组中
	float w = (2*pi) / samples;// 周期/采样点个数=采样频率
//	if(maxval<=((samples / 2))) return;//采样过多，需要示波器验证结果
	for(i = 0; i< samples;i++){
		digltal_value = maxval*sin(w*i)+maxval;//向上偏移半个幅值。使SineWave_Value数组中的数字量为正
		/*下列代码含义需验证*/
		if(digltal_value > 4095)
		digltal_value = 4095;/*上限限定*/
		SineWave[i] = digltal_value;//接受数字量
	}
}



void output_sin_wave(uint16_t arr,u16 nums)
{
	creat_sin(2048,nums);
	wave_enable(arr);
}


void Key_Proc()
{
	key_val = KEY_Scan(0);
	key_down = key_val&(key_old^key_val);//按下
	switch(key_down)
	{
		case MODE:LCD_Clear(WHITE);if(++key_mode==4)key_mode=0;
		break;
		case KEY_ADD:fre_flag = 1;
		switch(key_mode)
		{
			case 0:if(++fre==10000)
			fre = 10000;
			break;

			case 1:
			fre += 10; 
			if(fre==10000)
			fre = 10000;
			break;

			case 2:
			fre += 100; 
			if(fre==10000)
			fre = 10000;
			break;
			
			case 3:
			fre += 1000; 
			if(fre==10000)
			fre = 10000;
			break;
		}
		break;
		case KEY_REDUCE:fre_flag = 1;
		switch(key_mode)
		{
			case 0:if(--fre==10000)
			fre = 10000;
			break;

			case 1:
			fre -= 10; 
			if(fre==10000)
			fre = 10000;
			break;

			case 2:
			fre -= 100; 
			if(fre==10000)
			fre = 10000;
			break;
			
			case 3:
			fre -= 1000; 
			if(fre==10000)
			fre = 10000;
			break;
		}
	break;
}
	switch(key_mode)
	{
			case 0:LCD_ShowString(30,50,200,16,16,"Fre_var:1hz");break;
			case 1:LCD_ShowString(30,50,200,16,16,"Fre_var:10hz");break;
			case 2:LCD_ShowString(30,50,200,16,16,"Fre_var:100hz");break;
			case 3:LCD_ShowString(30,50,200,16,16,"Fre_var:1000hz");break;
	}
	if(fre>10000)//设置频率上下限
	{
		fre = 10000;
	}
	if(fre<10)
	{
		fre = 10;
	}
	LCD_ShowxNum(40,70,fre,16,16,0);//显示频率
	LCD_ShowString(16,70,200,16,16,"fre");
	if(fre_flag)
	{
		if(fre<=2500)
		{	
			nums = 100;
			arr = 10000/fre -1;
		}
		else if(fre>2500)
		{
			arr = 4-1;
			nums = 250000/fre;
		}
		output_sin_wave(arr,nums);
		fre_flag =0;
	}
}

int main(void)
{
	
	delay_init();
	
	LCD_Init();
	DAC_Config();
	KEY_Init();
	DMA_Config();
	POINT_COLOR=RED;//设置字体为红色
	output_sin_wave(1000-1,100);//初始状态10hz
	while(1){
	Key_Proc();
	}
}
