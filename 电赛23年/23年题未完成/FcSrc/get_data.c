//////////////////////////////////////////////////////////////////////
/*
获取数据用于后级pid处理和实时帧控制
*/
//////////////////////////////////////////////////////////////////////

#include "get_data.h"
#include "ANO_LX.h"
#include "stm32f4xx.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "mode.h"

double rol,pit,yaw,yaw_part,yaw_last,yaw_round;//imu输出的欧拉角，单位度
int16_t vel_x,vel_y;//速度，单位cm/s
int32_t hight;//高度,单位cm
int32_t dis_x,dis_y,dis_x___,dis_y___;//累积的距离,单位cm
int32_t dx_fix_intergral,dy_fix_intergral;//x轴y轴上积分数据


int32_t dis_xmu,dis_ymu;
void get_data(void)//获取各种数据
{
//数据来自ANO_LX.h凌霄接收IMU数据
	rol = (fc_att.st_data.rol_x100)/100.0;
	pit = (fc_att.st_data.pit_x100)/100.0;
	
	yaw_part = fc_att.st_data.yaw_x100;
	if((yaw_last>9000)&&(yaw_part<-9000)) yaw_round++;
	if((yaw_last<-9000)&&(yaw_part>9000)) yaw_round--;
	yaw = yaw_round*360 + yaw_part/100.0;
	yaw_last=yaw_part;
	yaw_fix=yaw-yaw_zore;//修正yaw角

////数据来自光流数据融合后
//	vel_x = ano_of.of2_dx_fix;
//	vel_y = ano_of.of2_dy_fix;	
	////////////////////////////////////////////////////
	if(ano_of.of2_dx_fix<200&&ano_of.of2_dx_fix>-200)//修正后的x轴y轴速度适用于积分计算 cm/s	
		dx_fix_intergral += ano_of.of2_dx_fix;//对x积分结果进行累计得到x轴偏移的位移
	if(ano_of.of2_dy_fix<200&&ano_of.of2_dy_fix>-200)	
		dy_fix_intergral += ano_of.of2_dy_fix;
//	////////////////////////////////////////////////////

	
	//光流修正的，坐标系为右手打枪坐标系，要注意方向
	/*下面进行坐标系变换*/
	dis_x = (int32_t) dx_fix_intergral/1247;//实际累积误差与光流的累积误差成一个倍数
	dis_y = -(int32_t) dy_fix_intergral/1247;//
	
	//dis_x_zore_fix雷达偏移修正坐标
		
//数据来自ANO_LX.h凌霄接收IMU数据，很抽象用不了
	vel_x = fc_vel.st_data.vel_x;//来自imu的x速度
	vel_y = fc_vel.st_data.vel_y;//来自imu的y速度

	hight = fc_hight.st_data.alt_add;//来自imu的高度	
}

void set_dis_zero(void)//设置初态都为0
{
	dx_fix_intergral=0;//初态x轴的积分
	dy_fix_intergral=0;//初态y轴的积分
	dis_x_zore_fix=0;//初态雷达修正的光流x轴的原点
	dis_y_zore_fix=0;//初态雷达修正的光流y轴的原点
}
