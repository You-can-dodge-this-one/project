//////////////////////////////////////////////////////////////////////
/*
�ײ����
*/
//////////////////////////////////////////////////////////////////////

#include "control.h"
int my_code_control_flag;//�Ƿ�ʹ���Զ����������ң������ı�־λ
int my_give_vel_x,my_give_vel_y,my_give_vel_z,my_give_vel_yaw;//�Զ����������ң������
int Task_time_dly_cnt_ms,time_cnt_ms;//��ͨ��ʱ
unsigned int  mission_step;//����

void all_data_init(void)//���������ȫ�����ݵĳ�ʼ��
{
	Task_time_dly_cnt_ms=0;
	time_cnt_ms=0;
}

void User_Task_Delay(int ms)//�����н�����ʱ����
{
	if(Task_time_dly_cnt_ms<ms){Task_time_dly_cnt_ms+=20;}
	else{Task_time_dly_cnt_ms=0;mission_step++;}			
}

int square_wave(int amplitude)//��Ծ����������10s
{
	int out;
	if(time_cnt_ms/5000%2==0)out=amplitude;
	if(time_cnt_ms/5000%2==1)out=0;
	return out;
}

int triangualr_wave(int amplitude)//���Ǻ���������10s
{
	int out;
	if(time_cnt_ms/5000%2==0)out=time_cnt_ms%5000/5000.0*amplitude;
	if(time_cnt_ms/5000%2==1)out=(1-time_cnt_ms%5000/5000.0)*amplitude;
	return out;
}
