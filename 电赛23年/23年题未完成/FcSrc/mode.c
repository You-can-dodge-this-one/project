//////////////////////////////////////////////////////////////////////
/*
ģʽѡ��
*/
//////////////////////////////////////////////////////////////////////

#include "mode.h"

int my_mode;//����ģʽ
int32_t hight_target,dis_x_target,dis_y_target,yaw_target;//Ŀ��߶ȣ�Ŀ��x����룬Ŀ��y����� ��Ŀ��yaw��
int32_t dis_x_zore_fix = 0,dis_y_zore_fix = 0;//�״�������Ĺ��������ԭ��
int32_t dis_x_fix,dis_y_fix,dis_heigh_fix;//��ǰ��������ϵ�µ�����
double yaw_zore,yaw_fix;
int keep_hight_flag,keep_yaw_flag,keep_dis_x_flag,keep_dis_y_flag,keep_OpenMV_data_1,keep_OpenMV_data_2,keep_radar_x_flag,keep_radar_y_flag;//��־λ
int dis_target_ref,dis_group[2][20]={{0,1,2,3,4,5,6,7,8,9,10,11,12},{1,0,2,0,0,0,2,0,0,0,2,3,0}};//һ�������飬һ��OpenMVģʽ��2023

void mode_select(int mode)//ѡ�����ģʽ���ı��־λ
{
	keep_hight_flag=0;keep_yaw_flag=0;keep_dis_x_flag=0;keep_dis_y_flag=0;keep_OpenMV_data_1=0;keep_OpenMV_data_2=0;keep_radar_x_flag=0;keep_radar_y_flag=0;
	switch(mode)
	{
		case 0:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=0;keep_dis_y_flag=0;}break;//���߶��㶼����
		case 1:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=1;keep_dis_y_flag=1;}break;//���߶�����ɣ����㽵��
		case 2:{keep_hight_flag=1;keep_yaw_flag=0;keep_dis_x_flag=1;keep_dis_y_flag=1;}break;//xy���߶��Ƕ�
		case 3:{keep_hight_flag=1;keep_yaw_flag=0;keep_OpenMV_data_1=1;keep_OpenMV_data_2=1;}break;//OpenMV���߶��Ƕ�
		case 4:{keep_hight_flag=1;keep_yaw_flag=0;keep_radar_x_flag=1;keep_radar_y_flag=1;}break;//�����״�xy���߶��Ƕ�
		default :break;
	}
}

void dis_target_select(int dis_target_ref)
{
	int mode=dis_group[0][dis_target_ref];
	switch(mode)
	{
		//��ɵ�Ϊ����ԭ��        dis_x_target
		//2023
		case 0:{dis_y_target=0; dis_y_target=0;}break;//
		case 1:{dis_y_target=4;dis_x_target=0;}break;//��һ��Ŀ���
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
