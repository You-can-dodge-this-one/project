#ifndef __TCS_H
#define __TCS_H


#include "sys.h"
#include "stm32f10x.h"
//#include "delay.h"
#include <stdio.h>
//#include "cfg.h"


#define white  	 1  //��
#define black  	 2    //��
#define yellow   3    //��
#define red    	 4    //��
#define blue     5   //��
#define other    6	  //�������


#define s0 PEout(4)// PB5
#define led_xm PEout(10)// PB5

#define s1 PEout(7)// PE5	
#define s2 PEout(6)// PB5
#define s3 PEout(8)// PE5	
#define led PEout(10)// PB5//���ˣ�2019,09.29
#define out PEin(5)// PE5	

void xm_TCS230_WhiteBalance(u16 pColor[3]);


u16 xm_colourjudge(u16 pRGB[3]); //������ɫ�����ж�ʶ����ɫ


#endif
