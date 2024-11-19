#include "KEY.h"
#include "Drv_Sys.h"

u8 KEY_1,KEY_2,KEY_3;
u8 KEY_1_last,KEY_2_last,KEY_3_last;

/**
  * @name   KEY_Init
  * @brief  按键初始化
  * @param  None
  * @retval None
  */
void KEY_Init(void)
{
    //初始化结构体 GPIO_InitStruct(取的一个随机的名字)
    GPIO_InitTypeDef   GPIO_InitStruct;
    //打开时钟（一般开时钟要放到前面的位置，然后再是设置上拉，输出这些）
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
  * @brief  按键扫描
  * @param  mode：1：支持按键长按；其他：不支持按键长按
  * @retval u8
  */
u8 KEY_Scan_1(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6);

    //按键检测
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //确定按键按下
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //记录这次按下
        return KEY_DOWN;    //返回按下标志，1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //按键没按下或者松开
       key_up = 1;    
    }
    return KEY_UP;              //返回没按下标志，0
}

u8 KEY_Scan_2(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5);

    //按键检测
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //确定按键按下
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //记录这次按下
        return KEY_DOWN;    //返回按下标志，1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //按键没按下或者松开
       key_up = 1;    
    }
    return KEY_UP;              //返回没按下标志，0
}

u8 KEY_Scan_3(u8 mode)
{
    static u8 key_up = 1;
    u8 KEY1 = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);

    //按键检测
    if(key_up&&(KEY1==0))
    {
			key_up++;
        //确定按键按下
      if((key_up==2)&&(KEY1==0))
      {
        key_up = 0;         //记录这次按下
        return KEY_DOWN;    //返回按下标志，1
      }
    }
		else if((key_up==0)&&(KEY1==1)){        //按键没按下或者松开
       key_up = 1;    
    }
    return KEY_UP;              //返回没按下标志，0
}

