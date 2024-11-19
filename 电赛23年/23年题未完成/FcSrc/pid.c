//////////////////////////////////////////////////////////////////////
/*
������PID
*/
//////////////////////////////////////////////////////////////////////

#include "pid.h"
#include "control.h"
#include "get_data.h"
#include "mode.h"
#include "my_uart.h"
#include "mode.h"
#include "mbotLinuxUsart.h"
#include <math.h>

double KP_x=0.6,KI_x=0.1,KD_x=0,x_intrgrate_max=20;//����pid����
double KP_y=0.6,KI_y=0,KD_y=0,y_intrgrate_max=20;//����pid����
double KP_MV=0.6,KI_MV=0,KD_MV=15;//OpenMV��pid����
double KP_radar=0.8,KI_radar=0,KD_radar=15;//�����״�pid����


int Limit_max_int(int input,int max)//�޷�����
{
	if(input > max)
	{
		input = max;
	}
	else if(input < -max)
	{
		input = -max;
	}
	return input;
}

void keep_hight(int32_t hight_target,int32_t hight)//�߶Ȼ�
{
	my_give_vel_z = (hight_target-hight)*3;
	my_give_vel_z = Limit_max_int(my_give_vel_z,30);
}

void keep_dis_x(double KP,double KD,double KI,int32_t dis_x_target,int32_t dis_x,int32_t max)//x�ỷ
{
	int32_t error,KP_out;
	int32_t KI_out,KD_out,total_out;
	static int32_t dis_intrgrate,error_last;
	
	error = dis_x_target-dis_x;//Ŀ��ֵ��ȥ��ǰֵ
	KP_out = KP*error;//���б�������
	
	KD_out = (error-error_last)*KD;//����΢������
	if(error_last!=error) error_last = error;//�����ϴ�������ڻ��ֻ���
	
	dis_intrgrate += error;//����
	KI_out=Limit_max_int(dis_intrgrate,x_intrgrate_max); //ki֮ǰû���õ�
	
	total_out = KP_out+KI_out+KD_out;
//	total_out = KP_out;
	total_out = Limit_max_int(total_out,max);
	
	my_give_vel_x = total_out;
}

void keep_dis_y(double KP,double KD,double KI,int32_t dis_y_target,int32_t dis_y,int32_t max)//y�ỷ
{
	int32_t error,KP_out;
	int32_t KI_out,KD_out,total_out;
	static int32_t dis_intrgrate,error_last;
	
	error = dis_y_target-dis_y;
	KP_out = KP*error;
	
	KD_out = (error-error_last)*KD;
	if(error_last!=error) error_last = error;
	
	dis_intrgrate += error;
	KI_out=Limit_max_int(KI_out,y_intrgrate_max);
	
//	total_out = KP_out+KI_out+KD_out;
	total_out = KP_out;
	total_out = Limit_max_int(total_out,max);
	
	my_give_vel_y = total_out;
}


void keep_yaw(int32_t yaw_target,int32_t yaw)//yaw��8
{
	my_give_vel_yaw = (yaw_target-yaw)*4;
	my_give_vel_yaw = Limit_max_int(my_give_vel_yaw,120);
}

void PID(void)//ң������ֵ��PID����
{
	//����ϴ����õ�����
	my_give_vel_x=0;
	my_give_vel_y=0;
	my_give_vel_z=0;
	my_give_vel_yaw=0;
	
	if(keep_hight_flag)		keep_hight(hight_target,hight);
	
	if(keep_dis_x_flag)	  keep_dis_x(KP_x,KD_x,KI_x,dis_x_target,dis_x,30);
//	if(keep_OpenMV_data_1)keep_dis_x(KP_MV,KD_MV,KI_MV,0,OpenMV_data_1,20);
//	if(keep_radar_x_flag) keep_dis_x(KP_radar,KD_radar,KI_radar,dis_x_target,radar_data_1,20);
		
	if(keep_dis_y_flag)	  keep_dis_y(KP_y,KD_y,KI_y,dis_y_target,dis_y,30);
//	if(keep_OpenMV_data_2)keep_dis_y(KP_MV,KD_MV,KI_MV,0,OpenMV_data_2,20);
//	if(keep_radar_y_flag) keep_dis_y(KP_radar,KD_radar,KI_radar,dis_y_target,radar_data_2,20);
		
	if(keep_yaw_flag)			keep_yaw(yaw_target,yaw_fix);
}
