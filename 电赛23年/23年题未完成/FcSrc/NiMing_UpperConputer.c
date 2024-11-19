#include "NiMing_UpperConputer.h"
#include "Drv_Uart.h"

//在SysConfig.h有定义
//#define BYTE0(dwTemp)   (*(char *)(&dwTemp))       //定义数据拆分
//#define BYTE1(dwTemp)   (*((char *)(&dwTemp)+1))
//#define BYTE2(dwTemp)   (*((char *)(&dwTemp)+2))
//#define BYTE3(dwTemp)   (*((char *)(&dwTemp)+3))

uint8_t DataSend[30];

void ANODT_SendF4(int Data_A,int Data_B,int Data_C,int Data_D)                //使用例  ANODT_SendF1(data);  data大小为16位
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xAA;//帧头
    DataSend[_cnt++] = 0xFF;//目标地址，广播oxFF
    DataSend[_cnt++] = 0xF1;//功能码
    DataSend[_cnt++] = 0x08;   //数据长度
    DataSend[_cnt++] = BYTE0(Data_A);//填充数据
    DataSend[_cnt++] = BYTE1(Data_A);//填充数据
    DataSend[_cnt++] = BYTE0(Data_B);//填充数据
    DataSend[_cnt++] = BYTE1(Data_B);//填充数据
    DataSend[_cnt++] = BYTE0(Data_C);//填充数据
    DataSend[_cnt++] = BYTE1(Data_C);//填充数据
    DataSend[_cnt++] = BYTE0(Data_D);//填充数据
    DataSend[_cnt++] = BYTE1(Data_D);//填充数据
    uint8_t sc = 0;
    uint8_t ac = 0;
    for(uint8_t i=0;i<DataSend[3]+4;i++)
    {
        sc +=DataSend[i];
        ac +=sc;
    }
    DataSend[_cnt++] = sc;//和校验位
    DataSend[_cnt++] = ac;//附加 校验位

    DrvUart1SendBuf(DataSend,DataSend[3]+6);
}

void ANODT_SendF1(int Data_A)
{
    uint8_t _cnt = 0;
    DataSend[_cnt++] = 0xAA;//帧头
    DataSend[_cnt++] = 0xFF;//目标地址，广播oxFF
    DataSend[_cnt++] = 0xF1;//功能码
    DataSend[_cnt++] = 0x02;   //数据长度
    DataSend[_cnt++] = BYTE0(Data_A);//填充数据
    DataSend[_cnt++] = BYTE1(Data_A);//填充数据
    uint8_t sc = 0;
    uint8_t ac = 0;
    for(uint8_t i=0;i<DataSend[3]+4;i++)
    {
        sc +=DataSend[i];
        ac +=sc;
    }
    DataSend[_cnt++] = sc;//和校验位
    DataSend[_cnt++] = ac;//附加 校验位

    DrvUart1SendBuf(DataSend,DataSend[3]+6);
}

