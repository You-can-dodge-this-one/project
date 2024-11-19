#ifndef __CONTROL_H
#define __CONTROL_H

extern int my_code_control_flag;
extern int my_give_vel_x,my_give_vel_y,my_give_vel_z,my_give_vel_yaw;
extern int Task_time_dly_cnt_ms,time_cnt_ms;
extern unsigned int  mission_step;

void all_data_init(void);
void User_Task_Delay(int ms);
int square_wave(int amplitude);
int triangualr_wave(int amplitude);

#endif
