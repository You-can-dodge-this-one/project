#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H

//==����
#include "SysConfig.h"

//==����/����

typedef struct
{
	//
	u8 of_update_cnt;  //�������ݸ��¼�����
	u8 alt_update_cnt; //�߶����ݸ��¼�����
	//
	u8 link_sta; //����״̬��0��δ���ӡ�1�������ӡ�
	u8 work_sta; //����״̬��0���쳣��1������
	//
	u8 of_quality;//����������Խ��Խ��
	//ԭʼ
	u8 of0_sta;//������״̬
	s8 of0_dx;//x����ƶ��ٶ�
	s8 of0_dy;//y����ƶ��ٶ�
	//�ںϹ�������Ϣ
	u8 of1_sta;//������״̬
	s16 of1_dx;//x����ƶ��ٶ�
	s16 of1_dy;//y����ƶ��ٶ�
	//�����ںϺ�Ĺ�����Ϣ
	u8 of2_sta;//������״̬
	s16 of2_dx;//x������
	s16 of2_dy;//y������
	s16 of2_dx_fix;//�������x������
	s16 of2_dy_fix;//�������y������
	s16 intergral_x;//x����ٶȻ���ֵ
	s16 intergral_y;//y����ٶȻ���ֵ
	//
	u32 of_alt_cm;//�߶���Ϣ
	//
	float quaternion[4];//��̬��Ϣ����Ԫ��
	//��̬
	s16 acc_data_x;//x���ٶ�
	s16 acc_data_y;//y���ٶ�
	s16 acc_data_z;//z���ٶ�
	//��̬
	s16 gyr_data_x;//x���ٶ�
	s16 gyr_data_y;//y���ٶ�
	s16 gyr_data_z;//z���ٶ�

} _ano_of_st;//�����ṹ��

//�ɿ�״̬

//==��������
extern _ano_of_st ano_of;
//==��������
//static
static void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_Check_State(float dT_s);
#endif
