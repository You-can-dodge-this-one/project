#ifndef __MT_UART_H
#define __MT_UART_H

#include "stdint.h"

extern int16_t OpenMV_data_0,OpenMV_data_1,OpenMV_data_2;
extern int16_t OpenMV_help_fix_x,OpenMV_help_fix_y,new_fire_x,new_fire_y,fire_x,fire_y;
extern unsigned char car_data_0,car_data_1;

void MY_urst1_Receive(uint8_t data);
void MY_urst2_Send(int to_OpenMV_data_0);
void MY_urst2_Receive(uint8_t data);
void MY_urst1_car_Receive(uint8_t data);
void MY_urst1_Send_xy(void);
void MY_urst1_Send_radar(void);

#endif
