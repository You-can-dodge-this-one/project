#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H

//==引用
#include "SysConfig.h"

//==定义/声明

typedef struct
{
	//
	u8 of_update_cnt;  //光流数据更新计数。
	u8 alt_update_cnt; //高度数据更新计数。
	//
	u8 link_sta; //连接状态：0，未连接。1，已连接。
	u8 work_sta; //工作状态：0，异常。1，正常
	//
	u8 of_quality;//光流的质量越大越好
	//原始
	u8 of0_sta;//光流的状态
	s8 of0_dx;//x轴的移动速度
	s8 of0_dy;//y轴的移动速度
	//融合光流的信息
	u8 of1_sta;//光流的状态
	s16 of1_dx;//x轴的移动速度
	s16 of1_dy;//y轴的移动速度
	//惯性融合后的光流信息
	u8 of2_sta;//光流的状态
	s16 of2_dx;//x轴移速
	s16 of2_dy;//y轴移速
	s16 of2_dx_fix;//修正后的x轴移速
	s16 of2_dy_fix;//修正后的y轴移速
	s16 intergral_x;//x轴的速度积分值
	s16 intergral_y;//y轴的速度积分值
	//
	u32 of_alt_cm;//高度信息
	//
	float quaternion[4];//姿态信息，四元数
	//静态
	s16 acc_data_x;//x加速度
	s16 acc_data_y;//y加速度
	s16 acc_data_z;//z加速度
	//动态
	s16 gyr_data_x;//x加速度
	s16 gyr_data_y;//y加速度
	s16 gyr_data_z;//z加速度

} _ano_of_st;//光流结构体

//飞控状态

//==数据声明
extern _ano_of_st ano_of;
//==函数声明
//static
static void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_Check_State(float dT_s);
#endif
