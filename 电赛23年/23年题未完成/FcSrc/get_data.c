//////////////////////////////////////////////////////////////////////
/*
��ȡ�������ں�pid�����ʵʱ֡����
*/
//////////////////////////////////////////////////////////////////////

#include "get_data.h"
#include "ANO_LX.h"
#include "stm32f4xx.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "mode.h"

double rol,pit,yaw,yaw_part,yaw_last,yaw_round;//imu�����ŷ���ǣ���λ��
int16_t vel_x,vel_y;//�ٶȣ���λcm/s
int32_t hight;//�߶�,��λcm
int32_t dis_x,dis_y,dis_x___,dis_y___;//�ۻ��ľ���,��λcm
int32_t dx_fix_intergral,dy_fix_intergral;//x��y���ϻ�������


int32_t dis_xmu,dis_ymu;
void get_data(void)//��ȡ��������
{
//��������ANO_LX.h��������IMU����
	rol = (fc_att.st_data.rol_x100)/100.0;
	pit = (fc_att.st_data.pit_x100)/100.0;
	
	yaw_part = fc_att.st_data.yaw_x100;
	if((yaw_last>9000)&&(yaw_part<-9000)) yaw_round++;
	if((yaw_last<-9000)&&(yaw_part>9000)) yaw_round--;
	yaw = yaw_round*360 + yaw_part/100.0;
	yaw_last=yaw_part;
	yaw_fix=yaw-yaw_zore;//����yaw��

////�������Թ��������ںϺ�
//	vel_x = ano_of.of2_dx_fix;
//	vel_y = ano_of.of2_dy_fix;	
	////////////////////////////////////////////////////
	if(ano_of.of2_dx_fix<200&&ano_of.of2_dx_fix>-200)//�������x��y���ٶ������ڻ��ּ��� cm/s	
		dx_fix_intergral += ano_of.of2_dx_fix;//��x���ֽ�������ۼƵõ�x��ƫ�Ƶ�λ��
	if(ano_of.of2_dy_fix<200&&ano_of.of2_dy_fix>-200)	
		dy_fix_intergral += ano_of.of2_dy_fix;
//	////////////////////////////////////////////////////

	
	//���������ģ�����ϵΪ���ִ�ǹ����ϵ��Ҫע�ⷽ��
	/*�����������ϵ�任*/
	dis_x = (int32_t) dx_fix_intergral/1247;//ʵ���ۻ������������ۻ�����һ������
	dis_y = -(int32_t) dy_fix_intergral/1247;//
	
	//dis_x_zore_fix�״�ƫ����������
		
//��������ANO_LX.h��������IMU���ݣ��ܳ����ò���
	vel_x = fc_vel.st_data.vel_x;//����imu��x�ٶ�
	vel_y = fc_vel.st_data.vel_y;//����imu��y�ٶ�

	hight = fc_hight.st_data.alt_add;//����imu�ĸ߶�	
}

void set_dis_zero(void)//���ó�̬��Ϊ0
{
	dx_fix_intergral=0;//��̬x��Ļ���
	dy_fix_intergral=0;//��̬y��Ļ���
	dis_x_zore_fix=0;//��̬�״������Ĺ���x���ԭ��
	dis_y_zore_fix=0;//��̬�״������Ĺ���y���ԭ��
}
