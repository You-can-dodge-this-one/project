#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "stm32f4xx.h"
extern int16_t pole_y,pole_z;

extern u8 openmv_type;
extern int16_t pole_x,pole_y;
void UserTask_OneKeyCmd(void);
void UserTask_task_one(void);
void UserTask_task_two(void);
void UserTask_task_three(void);
void UserTask_task_four(void);
void uart2_openmv(void);

void start_dis_front(void);
void start_dis_tail(void);


#endif
