#ifndef __MODE_H
#define __MODE_H

#include "stdint.h"
#include "control.h"

extern int my_mode;//飞行模式
extern int32_t hight_target,dis_x_target,dis_y_target,yaw_target;//数据目标
extern int32_t dis_x_zore_fix,dis_y_zore_fix;//雷达修正后的光流坐标的原点
extern int32_t dis_x_fix,dis_y_fix,dis_heigh_fix;//当前光流坐标系下的坐标
extern double yaw_zore,yaw_fix;
extern int dis_target_ref,dis_group[2][20];

extern int keep_hight_flag,keep_yaw_flag,keep_dis_x_flag,keep_dis_y_flag,keep_OpenMV_data_1,keep_OpenMV_data_2,keep_radar_x_flag,keep_radar_y_flag;//标志位

void mode_select(int mode);
void dis_target_select(int mode);

#endif
