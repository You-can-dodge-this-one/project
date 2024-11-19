#ifndef __GET_DATA_H
#define __GET_DATA_H

#include "stdint.h"

extern double rol,pit,yaw,yaw_part,yaw_last,yaw_round;//imu输出的欧拉角，单位度
extern int16_t vel_x,vel_y,vel_z;//速度，单位cm/s
extern int32_t hight;//高度,单位cm
extern int32_t dis_x,dis_y;//累积的距离,单位cm

void get_data(void);
int square_wave(int amplitude);
int triangualr_wave(int amplitude);
void set_dis_zero(void);

#endif
