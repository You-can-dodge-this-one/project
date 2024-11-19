//////////////////////////////////////////////////////////////////////
/*
模式选择
*/
//////////////////////////////////////////////////////////////////////

#include "mode.h"

int my_mode;//飞行模式
int32_t hight_target,dis_x_target,dis_y_target,yaw_target;//目标高度，目标x轴距离，目标y轴距离 ，目标yaw角
int32_t dis_x_zore_fix = 0,dis_y_zore_fix = 0;//雷达修正后的光流坐标的原点
int32_t dis_x_fix,dis_y_fix,dis_heigh_fix;//当前光流坐标系下的坐标
double yaw_zore,yaw_fix;
int keep_hight_flag,keep_yaw_flag,keep_dis_x_flag,keep_dis_y_flag,keep_OpenMV_data_1,keep_OpenMV_data_2,keep_radar_x_flag,keep_radar_y_flag;//标志位
int dis_target_ref,dis_group[2][20]={{0,1,2,3,4,5,6,7,8,9,10,11,12},{1,0,2,0,0,0,2,0,0,0,2,3,0}};//一组坐标组，一组OpenMV模式，2023

void mode_select(int mode)//选择飞行模式，改变标志位
{
	keep_hight_flag=0;keep_yaw_flag=0;keep_dis_x_flag=0;keep_dis_y_flag=0;keep_OpenMV_data_1=0;keep_OpenMV_data_2=0;keep_radar_x_flag=0;keep_radar_y_flag=0;
	switch(mode)
	{
		case 0:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=0;keep_dis_y_flag=0;}break;//定高定点都不用
		case 1:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=1;keep_dis_y_flag=1;}break;//定高定点起飞，定点降落
		case 2:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=1;keep_dis_y_flag=1;}break;//xy定高定角度
		case 3:{keep_hight_flag=1;keep_yaw_flag=0;keep_OpenMV_data_1=1;keep_OpenMV_data_2=1;}break;//OpenMV定高定角度
		case 4:{keep_hight_flag=1;keep_yaw_flag=0;keep_radar_x_flag=1;keep_radar_y_flag=1;}break;//激光雷达xy定高定角度
		default :break;
	}
}

void dis_target_select(int dis_target_ref)
{
	int mode=dis_group[0][dis_target_ref];
	switch(mode)
	{
		//起飞点为坐标原点        dis_x_target
		//2023
		case 0:{dis_y_target=0; dis_y_target=0;}break;//
		case 1:{dis_y_target=4;dis_x_target=0;}break;//第一个目标点
		case 2:{dis_y_target=1;dis_x_target=4;}break;
		case 3:{dis_y_target=1;dis_x_target=1;}break;
		case 4:{dis_y_target=2;dis_x_target=1;}break;
		case 5:{dis_y_target=2;dis_x_target=4;}break;
		case 6:{dis_y_target=3;dis_x_target=4;}break;
		case 7:{dis_y_target=3;dis_x_target=1;}break;
		case 8:{dis_y_target=4;dis_x_target=1;}break;
		case 9:{dis_y_target=4;dis_x_target=4;}break;
		case 10:{dis_y_target=5;dis_x_target=4;}break;
		case 11:{dis_y_target=5;dis_x_target=0;}break;
		case 12:{dis_y_target=0;dis_x_target=0;}break;
	}
	dis_x_target*=80;dis_y_target*=80;
}
