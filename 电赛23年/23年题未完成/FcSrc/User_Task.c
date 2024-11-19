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
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_land_f = 1, one_key_mission_f = 0;
        //
        //�жϵ�6ͨ������λ�� 1700<CH_6<2200���Ҳ�������
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 2200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =
                    //ִ��һ������
                    OneKey_Land();
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
	//�жϵ�6ͨ������λ�� 800<CH_6<1200���Ҳ�������
//		if((rc_in.rc_ch.st_data.ch_[ch_6_aux2]>800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1200))//////////////////////////////////////////////
		//һ�ж�������ɣ�׼����ʼ����
		if((rc_in.rc_ch.st_data.ch_[ch_6_aux2]>800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1200))//////////////////////////////////////////////
		{
			//��û��ִ��
			if(one_key_mission_f ==0)
			{
				//����Ѿ�ִ��
				one_key_mission_f = 1;
				//��ʼ����
				mission_step = 1;
			}
		}
		else//δ������ɵȴ��������
		{
			//��λ��ǣ��Ա��ٴ�ִ��
			one_key_mission_f = 0;		
		}
		//ִ������
		if(one_key_mission_f==1)
		{
			switch(mission_step)
			{
				///////////////////////////////////////////mdoe 0�Է�///////////////////////////////////////////
				case 0:{Task_time_dly_cnt_ms = 0;}break;//��֪��д�����˼��ɶ��һ�����������������Ϳ�ʼִ��case 1��
				//�л��̿�ģʽ��������6����ʱ�������Ѿ�����̿�ģʽ��������LX_Change_Mode(2)����ֵ��1  //////
				//����ģʽ������
				case 1:{mission_step += LX_Change_Mode(2);}break;/////////////////////////
				//�����ͳ�ʼ��
				case 2:{all_data_init();mode_select(0);mission_step += FC_Unlock();}break;
				case 3:{User_Task_Delay(2000);}break;
				//���
				case 4:
					{
						set_dis_zero();//�����ʼ��
						dis_x_target=120;//��ʼ��Ŀ��x
						dis_y_target=0;//��ʼ��Ŀ��y
						hight_target=50;//����
						yaw_target=yaw;//��yaw�Ƕ�
						mode_select(0);//�������
						
						User_Task_Delay(5000);
					}break;//����֮������ȡ��ǰ���ݽ���ʵʱ����֡�����
				case 5:{mission_step=10;}break;//��ʱ�Ѿ���ʼ���׼������Ŀ���λ
				case 10:{
					mode_select(1);//����+����
//					dis_target_ref=1;//�����һ�������
					time_cnt_ms=0;
					mission_step++;}break;
				case 11:{
//				dis_target_select(dis_target_ref);
					if(((dis_x>=110)&&(dis_x<=130))&&((dis_y>=-10)&&(dis_y<=10)))
						{my_give_vel_x=0;my_give_vel_y=0;User_Task_Delay(1000);}
				
					mission_step++;
				}break;//�ȴ�����Ŀ��㽵��
				case 12:{				
							 mission_step++;
//					if(dis_x_target-radar_data_1>-10&&dis_x_target-radar_data_1<10&&dis_y_target-radar_data_2>-10&&dis_y_target-radar_data_2<10)cnt++;else cnt=0;//����Ŀ��
//					if(cnt>=25){mission_step++;cnt=0;}
				}break;
				case 13:{
					mission_step++;//������һ��
				}break;
				//���㽵��
				case 14:{mode_select(1);hight_target=10;if(hight<20)mission_step=999;}break;
				//���ڴ�������
//				case 20:{task_see=20;
////					mode_select(3);beep_bsp(1);
////					if(OpenMV_data_1>-5&&OpenMV_data_1<5&&OpenMV_data_2>-5&&OpenMV_data_2<5) cnt++;else cnt=0;
////					if(cnt>=25){mission_step++;cnt=0;}
//					mode_select(4);//ѡ�����ģʽ
//					dis_x_target=fire_x;dis_y_target=fire_y;
//					if(find_fire_flag==0&&(fire_x-radar_data_1>-10&&fire_x-radar_data_1<10&&fire_y-radar_data_2>-10&&fire_y-radar_data_2<10))cnt++;else cnt=0;//����Ŀ��
//					if(cnt>=25){mission_step++;cnt=0;}
//				}break;
//				
//				
//				case 21:{task_see=21;Send_fire_flag=1;
//					hight_target=100;
//				if(hight<110&&hight>90){mission_step++;}break;
//				}
//				case 22:{User_Task_Delay(2000);}break;
//				//ִ�ж��
//				case 23:{servo_flag=1;User_Task_Delay(1500);}break;
//				case 24:{task_see=23;new_fire_x=radar_data_1;new_fire_y=radar_data_2;Send_fire_flag=0;mode_select(4);hight_target=150;find_fire_flag=1;mission_step=11;}break;
//				
//				
				///////////////////////////////////////////�����Է�///////////////////////////////////////////
				//һ������
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
