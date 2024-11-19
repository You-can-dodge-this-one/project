#ifndef __DRV_RC_INPUT_H
#define __DRV_RC_INPUT_H

#include "SysConfig.h"

/*每一毫秒触发一次，接受遥控器发送的数据，执行系统的飞行任务*/
void DrvRcPpmInit(void);
void DrvRcSbusInit(void);
void PPM_IRQH(void);
void Sbus_IRQH(void);

#endif
