#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "SysConfig.h"
typedef struct
{
void(*task_func)(void);
uint16_t rate_hz;
uint16_t interval_ticks;
uint32_t last_run;
}sched_task_t;

extern int cnt;
extern int Send_fire_flag;
extern int wait_time;
extern int servo_flag;

void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif

