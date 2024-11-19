#include "NiMing_UpperConputer.h"
#include "Drv_Uart.h"

//��SysConfig.h�ж���
//#define BYTE0(dwTemp)   (*(char *)(&dwTemp))       //�������ݲ��
//#define BYTE1(dwTemp)   (*((char *)(&dwTemp)+1))
//#define BYTE2(dwTemp)   (*((char *)(&dwTemp)+2))
//#define BYTE3(dwTemp)   (*((char *)(&dwTemp)+3))

uint8_t DataSend[30];

void ANODT_SendF4(int Data_A,int Data_B,int Data_C,int Data_D)                //ʹ����  ANODT_SendF1(data);  data��СΪ16λ
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xAA;//֡ͷ
    DataSend[_cnt++] = 0xFF;//Ŀ���ַ���㲥oxFF
    DataSend[_cnt++] = 0xF1;//������
    DataSend[_cnt++] = 0x08;   //���ݳ���
    DataSend[_cnt++] = BYTE0(Data_A);//�������
    DataSend[_cnt++] = BYTE1(Data_A);//�������
    DataSend[_cnt++] = BYTE0(Data_B);//�������
    DataSend[_cnt++] = BYTE1(Data_B);//�������
    DataSend[_cnt++] = BYTE0(Data_C);//�������
    DataSend[_cnt++] = BYTE1(Data_C);//�������
    DataSend[_cnt++] = BYTE0(Data_D);//�������
    DataSend[_cnt++] = BYTE1(Data_D);//�������
    uint8_t sc = 0;
    uint8_t ac = 0;
    for(uint8_t i=0;i<DataSend[3]+4;i++)
    {
        sc +=DataSend[i];
        ac +=sc;
    }
    DataSend[_cnt++] = sc;//��У��λ
    DataSend[_cnt++] = ac;//���� У��λ

    DrvUart1SendBuf(DataSend,DataSend[3]+6);
}

void ANODT_SendF1(int Data_A)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xAA;//֡ͷ
    DataSend[_cnt++] = 0xFF;//Ŀ���ַ���㲥oxFF
    DataSend[_cnt++] = 0xF1;//������
    DataSend[_cnt++] = 0x02;   //���ݳ���
    DataSend[_cnt++] = BYTE0(Data_A);//�������
    DataSend[_cnt++] = BYTE1(Data_A);//�������
    uint8_t sc = 0;
    uint8_t ac = 0;
    for(uint8_t i=0;i<DataSend[3]+4;i++)
    {
        sc +=DataSend[i];
        ac +=sc;
    }
    DataSend[_cnt++] = sc;//��У��λ
    DataSend[_cnt++] = ac;//���� У��λ

    DrvUart1SendBuf(DataSend,DataSend[3]+6);
}

