#ifndef _DRV_BEEP_H
#define _DRV_BEEP_H

#include "Drv_Sys.h"

extern int beep_flag;

void beep_init(void);
void beep_bsp(unsigned int flag);
void beep_delay(void);
#endif
