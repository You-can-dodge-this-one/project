#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"
#include "stm32f4xx.h"

void UserTask_OneKeyCmd(void);
void UserTask_task_one(void);
void UserTask_task_two(void);
void UserTask_task_three(void);
void UserTask_task_four(void);
void uart2_openmv(void);

void start_dis_front(void);
void start_dis_tail(void);


#endif
