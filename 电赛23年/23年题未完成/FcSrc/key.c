#include "KEY.h"
#include "Drv_Sys.h"

u8 KEY_1,KEY_2,KEY_3;
u8 KEY_1_last,KEY_2_last,KEY_3_last;

/**
  * @name   KEY_Init
  * @brief  ������ʼ��
  * @param  None
  * @retval None
  */
void KEY_Init(void)
{
    //��ʼ���ṹ�� GPIO_InitStruct(ȡ��һ�����������)
    GPIO_InitTypeDef   GPIO_InitStruct;
    //��ʱ�ӣ�һ�㿪ʱ��Ҫ�ŵ�ǰ���λ�ã�Ȼ���������������������Щ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_5;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_4;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
  * @name   KEY_Scan
  * @brief  ����ɨ��
  * @param  mode��1��֧�ְ�����������������֧�ְ�������
  * @retval u8
  */
u8 KEY_Scan_1(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);

    //�������
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //ȷ����������
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //��¼��ΰ���
        return KEY_DOWN;    //���ذ��±�־��1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //����û���»����ɿ�
       key_up = 1;    
    }
    return KEY_UP;              //����û���±�־��0
}

u8 KEY_Scan_2(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);

    //�������
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //ȷ����������
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //��¼��ΰ���
        return KEY_DOWN;    //���ذ��±�־��1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //����û���»����ɿ�
       key_up = 1;    
    }
    return KEY_UP;              //����û���±�־��0
}

u8 KEY_Scan_3(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);

    //�������
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //ȷ����������
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //��¼��ΰ���
        return KEY_DOWN;    //���ذ��±�־��1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //����û���»����ɿ�
       key_up = 1;    
    }
    return KEY_UP;              //����û���±�־��0
}

