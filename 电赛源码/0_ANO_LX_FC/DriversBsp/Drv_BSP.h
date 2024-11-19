#ifndef _DRVBSP_H_
#define _DRVBSP_H_
#include "SysConfig.h"
#include "Drv_Sys.h"
#include "ANO_LX.h"

typedef struct
{
	u8 sig_mode; //0==null,1==ppm,2==sbus
	//
	s16 ppm_ch[9];
	//
	s16 sbus_ch[16];
	u8 sbus_flag;
	//
	u16 signal_fre;
	u8 no_signal;
	u8 fail_safe;
	_rc_ch_un rc_ch;
	u16 signal_cnt_tmp;
	u8 rc_in_mode_tmp;
} _rc_input_st;

//==数据声明
extern _rc_input_st rc_in;

u8 All_Init(void);//加入外设初始化函数

void DrvRcInputInit(void);//输入协议初始化选择
void DrvPpmGetOneCh(u16 data);//获取一个通道的pwm，ppm将20ms划分为10个通道的pwm
void DrvSbusGetOneByte(u8 data);//获取一个通道的pwm，sbus将划分为15个通道的pwm
void DrvRcInputTask(float dT_s);//遥控器输入信号处理

#endif
