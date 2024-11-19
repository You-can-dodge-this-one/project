#ifndef __PID_H
#define __PID_H

#include "stdint.h"

int Limit_max_int(int input,int max);
void keep_hight(int32_t hight_target,int32_t hight);
void keep_dis_x(double KP,double KD,double KI,int32_t dis_x_target,int32_t dis_x,int32_t max);
void keep_dis_y(double KP,double KD,double KI,int32_t dis_y_target,int32_t dis_y,int32_t max);
void keep_yaw(int32_t yaw_target,int32_t yaw);
void PID(void);

#endif
