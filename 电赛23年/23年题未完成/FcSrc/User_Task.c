#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "control.h"
#include "mode.h"
#include "get_data.h"
#include "mbotLinuxUsart.h"
#include "Ano_Scheduler.h"
#include "Drv_led.h"
#include "my_uart.h"
int find_fire_flag=0;

int task_see;

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_land_f = 1, one_key_mission_f = 0;
        //
        //判断第6通道拨杆位置 1700<CH_6<2200，右拨杆向下
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =
                    //执行一键降落
                    OneKey_Land();
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
	//判断第6通道拨杆位置 800<CH_6<1200，右拨杆向上
//		if((rc_in.rc_ch.st_data.ch_[ch_6_aux2]>800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1200))//////////////////////////////////////////////
		//一切都部署完成，准备开始任务
		if((rc_in.rc_ch.st_data.ch_[ch_6_aux2]>800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1200))//////////////////////////////////////////////
		{
			//还没有执行
			if(one_key_mission_f ==0)
			{
				//标记已经执行
				one_key_mission_f = 1;
				//开始流程
				mission_step = 1;
			}
		}
		else//未部署完成等待部署完成
		{
			//复位标记，以便再次执行
			one_key_mission_f = 0;		
		}
		//执行任务
		if(one_key_mission_f==1)
		{
			switch(mission_step)
			{
				///////////////////////////////////////////mdoe 0试飞///////////////////////////////////////////
				case 0:{Task_time_dly_cnt_ms = 0;}break;//不知道写这个意思是啥，一旦满足任务条件，就开始执行case 1了
				//切换程控模式，当拨动6拨杆时，程序已经进入程控模式，在这里LX_Change_Mode(2)返回值是1  //////
				//定点模式，定点
				case 1:{mission_step += LX_Change_Mode(2);}break;/////////////////////////
				//解锁和初始化
				case 2:{all_data_init();mode_select(0);mission_step += FC_Unlock();}break;
				case 3:{User_Task_Delay(2000);}break;
				//起飞
				case 4:
					{
						set_dis_zero();//坐标初始化
						dis_x_target=120;//初始化目标x
						dis_y_target=0;//初始化目标y
						hight_target=50;//定高
						yaw_target=yaw;//定yaw角度
						mode_select(0);//定高起飞
						
						User_Task_Delay(5000);
					}break;//跳出之后进入获取当前数据进入实时控制帧，起飞
				case 5:{mission_step=10;}break;//此时已经开始起飞准备飞往目标点位
				case 10:{
					mode_select(1);//定高+定点
//					dis_target_ref=1;//到达第一个任务点
					time_cnt_ms=0;
					mission_step++;}break;
				case 11:{
//				dis_target_select(dis_target_ref);
					if(((dis_x>=110)&&(dis_x<=130))&&((dis_y>=-10)&&(dis_y<=10)))
						{my_give_vel_x=0;my_give_vel_y=0;User_Task_Delay(1000);}
				
					mission_step++;
				}break;//等待到达目标点降落
				case 12:{				
							 mission_step++;
//					if(dis_x_target-radar_data_1>-10&&dis_x_target-radar_data_1<10&&dis_y_target-radar_data_2>-10&&dis_y_target-radar_data_2<10)cnt++;else cnt=0;//到达目标
//					if(cnt>=25){mission_step++;cnt=0;}
				}break;
				case 13:{
					mission_step++;//进入下一步
				}break;
				//定点降落
				case 14:{mode_select(1);hight_target=10;if(hight<20)mission_step=999;}break;
				//用于处理灭火的
//				case 20:{task_see=20;
////					mode_select(3);beep_bsp(1);
////					if(OpenMV_data_1>-5&&OpenMV_data_1<5&&OpenMV_data_2>-5&&OpenMV_data_2<5) cnt++;else cnt=0;
////					if(cnt>=25){mission_step++;cnt=0;}
//					mode_select(4);//选择飞行模式
//					dis_x_target=fire_x;dis_y_target=fire_y;
//					if(find_fire_flag==0&&(fire_x-radar_data_1>-10&&fire_x-radar_data_1<10&&fire_y-radar_data_2>-10&&fire_y-radar_data_2<10))cnt++;else cnt=0;//到达目标
//					if(cnt>=25){mission_step++;cnt=0;}
//				}break;
//				
//				
//				case 21:{task_see=21;Send_fire_flag=1;
//					hight_target=100;
//				if(hight<110&&hight>90){mission_step++;}break;
//				}
//				case 22:{User_Task_Delay(2000);}break;
//				//执行舵机
//				case 23:{servo_flag=1;User_Task_Delay(1500);}break;
//				case 24:{task_see=23;new_fire_x=radar_data_1;new_fire_y=radar_data_2;Send_fire_flag=0;mode_select(4);hight_target=150;find_fire_flag=1;mission_step=11;}break;
//				
//				
				///////////////////////////////////////////基础试飞///////////////////////////////////////////
				//一键降落
				case 999:{mode_select(0);OneKey_Land();}break;
				default:break;
			}
		}
		else
		{
			mission_step = 0;
		}
	
    ////////////////////////////////////////////////////////////////////////
}
