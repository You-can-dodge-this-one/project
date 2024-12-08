#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"


#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读取按键1
#define KEY_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键3(WK_UP) 

 

#define KEY_ADD 	1	//KEY0按下
#define KEY_REDUCE	  2	//KEY1按下
#define MODE   3	//KEY_UP按下(即WK_UP/KEY_UP)


void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数					    
#endif
