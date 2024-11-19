#ifndef __GET_DATA_H
#define __GET_DATA_H

#include "stdint.h"

extern double rol,pit,yaw,yaw_part,yaw_last,yaw_round;//imu�����ŷ���ǣ���λ��
extern int16_t vel_x,vel_y,vel_z;//�ٶȣ���λcm/s
extern int32_t hight;//�߶�,��λcm
extern int32_t dis_x,dis_y;//�ۻ��ľ���,��λcm

void get_data(void);
int square_wave(int amplitude);
int triangualr_wave(int amplitude);
void set_dis_zero(void);

#endif
