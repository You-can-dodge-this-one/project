#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f4xx.h"

//按键状态宏定义
#define KEY_DOWN    1
#define KEY_UP      0

extern u8 KEY_1,KEY_2,KEY_3;
extern u8 KEY_1_last,KEY_2_last,KEY_3_last;

void KEY_Init(void);		//按键引脚初始化函数
u8   KEY_Scan_1(u8 mode);		//按键检测函数
u8   KEY_Scan_2(u8 mode);		//按键检测函数
u8   KEY_Scan_3(u8 mode);		//按键检测函数
#endif

