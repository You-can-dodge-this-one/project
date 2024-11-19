#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//测试发送变量
extern short to_radar_data_1;
extern short to_radar_data_2;
extern short to_radar_data_3;
extern unsigned char to_radar_data_4;
extern int radar_wait_time;
extern int radar_data_1_original,radar_data_2_original;

//测试接收变量
extern int radar_data_1;
extern int radar_data_2;
extern unsigned char radar_data_3;

//从linux接收并解析数据到参数地址中
extern int usartReceiveOneData(void);   
//封装数据，调用USART1_Send_String将数据发送给linux
extern void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag); 
//发送指定字符数组的函数
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
