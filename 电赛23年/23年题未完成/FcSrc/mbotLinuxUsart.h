#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//���Է��ͱ���
extern short to_radar_data_1;
extern short to_radar_data_2;
extern short to_radar_data_3;
extern unsigned char to_radar_data_4;
extern int radar_wait_time;
extern int radar_data_1_original,radar_data_2_original;

//���Խ��ձ���
extern int radar_data_1;
extern int radar_data_2;
extern unsigned char radar_data_3;

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(void);   
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
extern void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag); 
//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
