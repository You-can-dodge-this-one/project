#include "User_Task.h"
#include "Drv_RcIn.h"
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"                  // Device header

#include "Drv_Uart.h"


#include "stm32f4xx_gpio.h"

#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "main.h"

#include "stdio.h"

#include <string.h>
#include <math.h>//�������Ǻ���
#include "main.h"

#include "User_Task.h"
#include <stdlib.h>//�������Ǻ���
#include "Drv_Sys.h"

#include "Drv_Uart.h"
#include "I2C_Init.h"


extern float sum_x_aa;
extern u8 m1_step,m2_step,m3_step,m4_step;
extern u8 rx_buff_com1[20];
extern int dis_front_cm;
extern int dis_tail_cm;
char flag=0x00;


u8 start_trig_flag=0;
int cont_t2_echo=0;
int cont_t2_echo_tail=0;

struct point //Ŀ��
{
	float x;
	float y;
	float z;
};
													
struct point sow_piont[]={ 
													{0.83,0.98,150},//1��λ��
													//A3							A2								 A1            		A4           		 A5            		  A6                
													{0.98,1.17,150},{1.02,1.71,150},{1.0,2.30,150},{1.0,2.23,100},{1.09,1.73,100},{1.09,1.25,100}, 
													//��ǽ��                                		��ͷ
													{1.08,0.2,100},{2.58,0.47,100},//w1,w2 ��
													//B6							B5						 		 B4           		B1             	 B2              		B3                
													{2.2,1.56,100},{2.2,1.97,100},{2.3,2.3,100},{2.32,2.47,150},{2.3,1.94,150},{2.28,1.3,150},
													//����ǽ���е�                              ��ͷ
													{2.86,1.27,150},
													//C3							C2								 C1     		      C4            	 C5              		C6                
													{3.1,1.38,150},{3.16,1.85,150},{3.23,2.32,150},{3.2,2.3,100},{3.3,1.85,100},{3.3,1.4,100},
													//��ǽ��                                    ��ͷ
													{3.05,0.2,100},{4.5,0.4,100},//w3��w4
													//D6							D5						 		 D4   	         	  D1	             	 D2 	             D3             
													{4.4,1.4,100},{4.4,2.0,100},{4.4,2.55,100},{4.4,2.55,150},{4.4,2.0,150},{4.32,1.45,150},
													{4.58,3.17,150}//�ؼ�����
};
//struct point1 //������
//{
//	float x;
//	float y;
//	float z;
//};              
//struct point fix_piont[4]={{0.94,0.45,1.0},{1.9,0.3,1.5},{3.27,0.56,1.0},{3.9,0.32,1.5}};
//u8 w1=0;
//u8 fix_flag = 0;//���������������־λ







int sow_add[27]={28,27,20,19,26,25,18,14,10,8,4,3,7,9,3,7,24,23,16,6,2,1,5,11,15,22};
#define state_num 18
	
float home_x=4.1;
float home_y=3.2;
	
int wait_flag;
int standard_yaw,standard_yaw1,target_yaw,now_standard_yaw;

//////////////////////////////////////////////////////////////////////////////////////////////
//follow_line()
//////////////////////////////////////////////////////////////////////////////////////////
int16_t pole_x=0,pole_y=0,pole_z=0,pole_w=0,pole_w_middle=0;
int16_t line_x=0,line_y=0,line_sita=0;
int time_dly_cnt_ms_xm=0,time_dly_cnt_ms=0;
int16_t x_position=0,y_position=0,y_position_last=0;
char c, type;
double plant_x_init,plant_y_init,plant_z,act_x=4.19,act_y=4.46;
double plant_x,plant_y;
float error_x=0,error_y=0;
int add_n=0;
	
int steering_pwmval=1500;
	
	
float altitude_set=120,p_altitude=1.0,speed_z_axis=0,speed_x_axis=0,speed_y_axis=0,p_longitude=10.0;//p_longitudeԭ����ֵΪ10
	
float altitude_set_1=100;

int16_t speed_x_axis_16=0,speed_y_axis_16=0,speed_z_axis_16=0,yaw_16=0,p_speed_y_axis_16=0,d_speed_y_axis_16=0,speed_yaw_axis_16=0;										

u8 mission_step=0,near_alarm_flag=0,far_alarm_flag=0,near_alarm_count=0;

u8 far_alarm_count=0,first_a0_voice=1,cnt_20ms_no_signal=0,flag_45=0;

float cons_kp=0.08,cons_kd=0.2;

int i,cnt_20=0,trun_on_time=0;

u8 period_dis=0,basic_step=0,light_flag=0,s_i_xm=0,Send_Count_xm=0;

u8 DataScope_OutPut_Buffer_xm11[10]={0};

u8 index_sow_i=0;
float tar_temp_x,tar_temp_x,tar_temp_x;
float dis_sonic=0;

unsigned char return_flag=0;//��ȡopenmv�ķ���ֵ������ҵ�����Դͷ�򷵻�1���ҵ���ɽ����ͷ���2
unsigned char done_flag=0;//���������ɱ�־λ


u8 position_1_order=0x00;
u8 position_2_order=0x00;
u8 tx1_buf_xm[6]={0};

int box_down_flag=0x00;
int box_count=0,result_target=0;

int down_count=0x00;
u8 	down_flag=0x00;
unsigned char fire_flag=0x00;

int	up_count=0x00;
u8 	up_flag=0x00;

int record_cnt=0x00,record_arry[10]={0},start_iden_flag=0x00,m2_time_cnt=0x00;//;
int time;
int distance_x,distance_y;//����

unsigned char transition=0x00,old_transition=0xff;;
u8 fly_mode=0,ds_flag=0;;
/*******************************************************************************/
u8 openmv_type=0;//0��openmv1��1��openmv2
u8 now_mission_step = 0,last_mission_step = 0;//�ϴ�����

void cmd_send1(int16_t speed_x_axis_16,int16_t speed_y_axis_16,int16_t speed_z_axis_16,int16_t yaw_16)
{
	rt_tar.st_data.rol = 0;
	rt_tar.st_data.pit = 0;
	rt_tar.st_data.thr = 0; //����ģʽ0������ģʽ0ʱʧ�أ����Ź�����ܣ���һ���Ե�����λ������
			//������ʵʱXYZ-YAW�����ٶ�����
	switch(fly_mode)
	{
		case 0://��ͷ����
		{
			rt_tar.st_data.yaw_dps=yaw_16;// yaw_16;
			rt_tar.st_data.vel_x =speed_x_axis_16;//speed_x_axis_16;
			rt_tar.st_data.vel_y =speed_y_axis_16;//
			rt_tar.st_data.vel_z =speed_z_axis_16;
		}break;
		case 1://��ͷ����
		{
			rt_tar.st_data.yaw_dps=yaw_16;// yaw_16;
			rt_tar.st_data.vel_x =-speed_x_axis_16;//speed_x_axis_16;
			rt_tar.st_data.vel_y =-speed_y_axis_16;//
			rt_tar.st_data.vel_z =speed_z_axis_16;
		}break;
		default :break;
	}
	dt.fun[0x41].WTS = 1; //��Ҫ����rt_tar���ݡ�
}

void task1_UserTask_OneKeyCmd(void);

void stop_basket(void)
{
	steering_pwmval=1500;//high mean low speed
	TIM_SetCompare1(TIM4,steering_pwmval);
	
	up_count=0;
	down_count=0;
	up_flag=0;
	down_flag=0;
}




void up_basket(void)
{
	if(up_flag==00)
	{
		up_flag=0x01;
		steering_pwmval=1500-200;//high mean low speed
		TIM_SetCompare1(TIM4,steering_pwmval);
	}
	else
	{
		if(up_count>=2900)
		{
			steering_pwmval=1500;//high mean low speed
			TIM_SetCompare1(TIM4,steering_pwmval);
		}
	}
}


void down_basket(void)
{
	if(down_flag==00)
	{
		down_flag=0x01;
		steering_pwmval=1500+200;//high mean low speed
		TIM_SetCompare1(TIM4,steering_pwmval);
	}
	else
	{
		if(down_count>=2900)
		{
			steering_pwmval=1500;//high mean low speed
			TIM_SetCompare1(TIM4,steering_pwmval);
		}
	}	
}



///////////////////////////////////
void sow_seed(void)//led����˸����
{
	GPIO_SetBits(GPIOD,GPIO_Pin_15);//GPIOF9,F10���øߣ�����
	light_flag=1;
}

void calculate_shortest_rotation_angle(int target)//����yaw����
{
	int now = three_yaw_xm;//��ǰ�Ƕ�
	// �淶��ƫ���ǵ�-180�㵽180��ķ�Χ��
	// ����ǶȲ�
	int diff = target - now;

	// ȷ���ǶȲ���-180�㵽180��ķ�Χ��
	if (diff > 180) {
			diff -= 360;
	} else if (diff < -180.0) {
			diff += 360;
	}
	diff = (-diff/2);
	if((diff>-20)&&(diff<20))
	{
		if(diff>10)diff=7;
		if(diff<-10)diff=-7;
	}
	yaw_16 = diff;
//	cmd_send1(0,0,0,diff);
}

/*---------------------uwb������ڴ���---------------------*/

void uwb_adj(float w_x,float w_y,float w_z)//uwb��������
{
	dis_sonic=sum_x_aa;		
	speed_x_axis=p_longitude*(w_x-plant_x);//PIDX�����
	speed_x_axis_16=(int16_t) speed_x_axis;
	if(speed_x_axis_16>7)
	{
		speed_x_axis_16=7;
	}
	if(speed_x_axis_16<-7)
	{
		speed_x_axis_16=-7;
	}
	
	if(transition>=2&&transition<=6)//aǽ������Ҫ��ֹײǽ��Զ��ǽ
	{
		//����1�׾��룬�����������Ǳ��ԭ��δɨ�赽ǽ
		if(dis_front_cm<=100)
		{
			if(dis_front_cm<42)//����aǽ����
			{
				speed_x_axis_16=-7;
			}
			else if(dis_front_cm>52)//����aǽ��Զ
			{
				speed_x_axis_16=7;
			}
	
		}
		
	}
	else if (transition>=11&&transition<=14)//bǽ������Ҫ��ֹײǽ��Զ��ǽ
	{
		
		
		if(dis_tail_cm<=100)
		{
			if(dis_tail_cm<42)//����bǽ����
			{
				speed_x_axis_16=7;
			}
			else if(dis_tail_cm>52)//����bǽ��Զ
			{
				speed_x_axis_16=-7;
			}
	
		}
		
		
		
	}
	
	else if (transition>=16&&transition<=21)//cǽ������Ҫ��ֹײǽ��Զ��ǽ
	{
		//����1�׾��룬�����������Ǳ��ԭ��δɨ�赽ǽ
		if(dis_front_cm<=100)
		{
			if(dis_front_cm<42)//����cǽ����
			{
				speed_x_axis_16=-7;
			}
			else if(dis_front_cm>52)//����cǽ��Զ
			{
				speed_x_axis_16=7;
			}
	
		}
		
		
	}
	
	else if (transition>=25&&transition<=29)//dǽ������Ҫ��ֹײǽ��Զ��ǽ
	{
		//����1�׾��룬�����������Ǳ��ԭ��δɨ�赽ǽ
		if(dis_tail_cm<=100)
		{
			if(dis_tail_cm<42)//����bǽ����
			{
				speed_x_axis_16=7;
			}
			else if(dis_tail_cm>52)//����bǽ��Զ
			{
				speed_x_axis_16=-7;
			}
	
		}
		
		
	}
	
	
	if((speed_x_axis_16<5)&&(speed_x_axis_16>0))
	{
		speed_x_axis_16=5;
	}
	
	if((speed_x_axis_16<0)&&(speed_x_axis_16>-5))
	{
		speed_x_axis_16=-5;
	}
	
	speed_y_axis=p_longitude*(w_y-plant_y);//PIDY�����  act_y-plant_y
	speed_y_axis_16=(int16_t) speed_y_axis;
	if(speed_y_axis_16>7)
	{
		speed_y_axis_16=7;
	}
	if(speed_y_axis_16<-7)
	{
		speed_y_axis_16=-7;
	}
	
	
	if((speed_y_axis_16<5)&&(speed_y_axis_16>0))
	{
		speed_y_axis_16=5;
	}
	
	if((speed_y_axis_16<0)&&(speed_y_axis_16>-5))
	{
		speed_y_axis_16=-5;
	}

	speed_z_axis=p_altitude*(w_z-alt_add_xm);//PID�߶ȿ���
	speed_z_axis_16=(int16_t) speed_z_axis;
	
	target_yaw = now_standard_yaw;//��ǰyaw
	calculate_shortest_rotation_angle(target_yaw);//��ΪĿ��yaw��
	//�任����ϵ//��ȫ�ֱ���
	cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
	if(time_dly_cnt_ms_xm%300==0)
	{
		if(fly_mode==0)
		{
			if((speed_x_axis_16>0)&&(speed_y_axis_16>0))//���ֶ���
		{
			LxStringSend(1,"ǰ��+1,����+1");
			sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
			speak_len1=strlen(buf1);
			speak_context((u8*)buf1,speak_len1);
		}
		else if((speed_x_axis_16>0)&&(speed_y_axis_16<0))
		{
			LxStringSend(1,"ǰ��+1,����+1");
			sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
			speak_len1=strlen(buf1);
			speak_context((u8*)buf1,speak_len1);
		}
		else if((speed_x_axis_16<0)&&(speed_y_axis_16>0))
		{
			LxStringSend(1,"����+1,����+1");
			sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
			speak_len1=strlen(buf1);
			speak_context((u8*)buf1,speak_len1);
		}
		else if((speed_x_axis_16<0)&&(speed_y_axis_16<0))
		{
			LxStringSend(1,"����+1,����+1");
			sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
			speak_len1=strlen(buf1);
			speak_context((u8*)buf1,speak_len1);
		}
			
		}
		else
		{
			
				if((speed_x_axis_16>0)&&(speed_y_axis_16>0))//���ֶ���
			{
				LxStringSend(1,"����+1,����+1");
				sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
				
			}
			else if((speed_x_axis_16>0)&&(speed_y_axis_16<0))
			{
				 LxStringSend(1,"����+1,����+1");
				sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
			}
			else if((speed_x_axis_16<0)&&(speed_y_axis_16>0))
			{
				LxStringSend(1,"ǰ��+1,����+1");
				sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
				
			}
			else if((speed_x_axis_16<0)&&(speed_y_axis_16<0))
			{
				 LxStringSend(1,"ǰ��+1,����+1");
				sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
			}
			
			
			
			
			
			
			
			
			
			
		}
		
	}
	
//	cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
}

/*---------------------uwb������ڴ������---------------------*/

/*---------------------openmv������ڴ���---------------------*/
void openmv_adj(float o_x,float o_y,float o_z)
{
	speed_x_axis = (120-o_y);
	speed_x_axis_16 = (int16_t)speed_x_axis;
	if(speed_x_axis_16>10)
	{
		speed_x_axis_16=5;
	}
	if(speed_x_axis_16<-10)
	{
		speed_x_axis_16=-5;
	}
	
	if((speed_x_axis_16<5)&&(speed_x_axis_16>0))
	{
		speed_x_axis_16=2;
	}
	
	if((speed_x_axis_16<0)&&(speed_x_axis_16>-5))
	{
		speed_x_axis_16=-2;
	}
	
	speed_y_axis = (160-o_x);
	speed_y_axis_16 = (int16_t)speed_y_axis;
	
	if(speed_y_axis_16>10)
	{
		speed_y_axis_16=5;
	}
	if(speed_y_axis_16<-10)
	{
		speed_y_axis_16=-5;
	}
	
	if((speed_y_axis_16<5)&&(speed_y_axis_16>0))
	{
		speed_y_axis_16=2;
	}
	
	if((speed_y_axis_16<0)&&(speed_y_axis_16>-5))
	{
		speed_y_axis_16=-2;
	}
	
	speed_z_axis = (o_z-alt_add_xm);
	speed_z_axis_16 = (int16_t)speed_z_axis_16;
	
	target_yaw = now_standard_yaw;//Ŀ��yawΪ��ǰyaw
	calculate_shortest_rotation_angle(target_yaw);//��ΪĿ��yaw��
	//�任����ϵ//��ȫ�ֱ���
	cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
	
	if((speed_x_axis_16>0)&&(speed_y_axis_16>0))//���ֶ���
	{
		LxStringSend(1,"ǰ��+1,����+1");
		sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
		speak_len1=strlen(buf1);
		speak_context((u8*)buf1,speak_len1);
	}
	else if((speed_x_axis_16>0)&&(speed_y_axis_16<0))
	{
		LxStringSend(1,"ǰ��+1,����+1");
		sprintf(buf1,"[s9]ǰ��");//\[b1] ��ӭ���٣����
		speak_len1=strlen(buf1);
		speak_context((u8*)buf1,speak_len1);
	}
	else if((speed_x_axis_16<0)&&(speed_y_axis_16>0))
	{
		LxStringSend(1,"����+1,����+1");
		sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
		speak_len1=strlen(buf1);
		speak_context((u8*)buf1,speak_len1);
	}
	else if((speed_x_axis_16<0)&&(speed_y_axis_16<0))
	{
		LxStringSend(1,"����+1,����+1");
		sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
		speak_len1=strlen(buf1);
		speak_context((u8*)buf1,speak_len1);
	}
}
/*---------------------openmv������ڴ������---------------------*/
	

int find_target(float target_x,float target_y,float target_z)
{
	
	char ds_buf[30]={0};
	if(return_flag == 2||return_flag==1)
	{
		if(return_flag ==2)
		{					
			if((plant_x>=(line_x-0.05f))&&(plant_x<=(line_x+0.05f))&&(plant_y>(line_y-0.05f))&&(plant_y<=(line_y+0.05f)))
			{
				mission_step = 16;
				return 1;
			}
			else
			{
				openmv_adj(line_x,line_y,altitude_set);//���˻����½����꣬�߶���1.8m
				return 0;
			}
		}
		else
		{
			if((plant_x>=(pole_x-0.05f))&&(plant_x<=(pole_x+0.05f))&&(plant_y>(pole_y-0.05f))&&(plant_y<=(pole_y+0.05f)))//���˻��ҵ���Դ������1��
			{
				return 1;
			}
			else
			{
				openmv_adj(pole_x,pole_y,altitude_set);//���˻��һ�Դ���߶ȱ�����1.8m
				return 0;
			}
		}
	}	
	else
	{
		if (((plant_x>=(target_x-0.2f))&&(plant_x<=(target_x+0.2f)))&&((plant_y>(target_y-0.2f))&&(plant_y<=(target_y+0.2f))))//uwb��������ԭ����15->20;
		{
				sprintf(buf1,"[s9]�����һ������");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
				
				
				if(transition!=old_transition)
				{ 
					sprintf(ds_buf,"�����%d������",transition);
					LxStringSend(1,ds_buf);
					old_transition=transition;
				}
				
				return 1;
		}
		else
		 {
			uwb_adj(target_x,target_y,target_z);
			ds_flag=1;
			return 0;
		 }	
	 }		 
}

/////////////////////////////////////////////////////////////////////////////////////////////
//��������
/////////////////////////////////////////////////////////////////////////////////////////

void uart3_camera_command_analyse(void)
{
	if( rx_end_flag==0x01)
	{
		
		int n2 = sscanf((char*)rx_buff,"$KT%c, %lf,%lf, %lf ", &type,  &plant_x_init, &plant_y_init, &plant_z);

//		plant_x =-0.1716*pow(plant_x_init,3)+0.9039*pow(plant_x_init,2)-0.1098*plant_x_init+0.3109;
//		
//		plant_y =0.01032*pow(plant_y_init,2)+0.9243*plant_y_init+0.07365;
		
		plant_x =plant_x_init;
		
		plant_y =plant_y_init;
		rx_end_flag=0;
		first_frame_end=0;
	}	
}	

void uart2_openmv(void)
{
	if(rx_end_flag==0x01)
	{		
		if(rx_buff[1]==0xf0)//ploe
		{
			pole_x=(rx_buff[3]<<8|rx_buff[4]);//���ֵ���������x
			pole_y=(rx_buff[5]<<8|rx_buff[6]);//���ֵ���������y
			if(done_flag==0x00)	
			{
				return_flag = 0x01;
			}//����м�⵽���־ͷ���1
			else 
			{
				return_flag = 0;
			}
		}
		else if(rx_buff[1]==0xf1)//line
		{
			line_x=(rx_buff[3]<<8|rx_buff[4]);//��ɵ����������x
			line_y=(rx_buff[5]<<8|rx_buff[6]);//��ɵ����������y
//			if(transition==11) return_flag = 2;
//			else return_flag = 0;
		}
		rx_end_flag=0;
		first_frame_end=0;
	}			
}


void start_dis_front(void)
{
	steering_pd12=0;
	steering_pd12=1;
	
	MyDelayUs(50);
	
	steering_pd12=0;
	
	start_trig_flag=1;
	
	cont_t2_echo=0;
	
//	end_dis_flag=0;
	
	
	
}


void start_dis_tail(void)
{
	steering_pd13=0;
	steering_pd13=1;
	
	MyDelayUs(50);
	
	steering_pd13=0;
	
	start_trig_flag=1;
	
	cont_t2_echo_tail=0;
	
//	end_dis_flag=0;
	
	
	
}











/*--------------------------------------------��ҵ1----------------------------------------------------------*/

void UserTask_task_one(void)
{
	if(!flag)
	{
		TIM_SetCompare1(TIM4,1500);
		flag=1;
		m1_step=1;
		fly_mode = 0;//����ģʽ
		standard_yaw = three_yaw_xm;//��ȡ��׼yaw
		now_standard_yaw = standard_yaw;//��ǰ���ĸ�Ŀ��yaw
		if(standard_yaw>=0)//��ȡ��תyaw
			standard_yaw1 = standard_yaw-180;
		else
			standard_yaw1 = standard_yaw+180;
		
	}
	switch(m1_step)
	{
		case 0:
			//reset
			if(rx_end_flag_com1)
			{
				rx_end_flag_com1=0;
				position_1_order= rx_buff_com1[3];
				position_2_order= rx_buff_com1[4];
				
				tx1_buf_xm[0]=0xa5;
				tx1_buf_xm[1]=0x5a;
				tx1_buf_xm[2]=0x03;
				tx1_buf_xm[3]=0x00;
				tx1_buf_xm[4]=0xfd;
				
				DrvUart1SendBuf(tx1_buf_xm,5);
				
				if(position_1_order==1&&position_2_order==1) m1_step=0x01;
				
				sprintf(buf1,"[s8]׼������ɣ������ѽ���");//�ɿؽ���
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
			}
			break;
		case 1:
			//reset
			task1_UserTask_OneKeyCmd();
			break;
		default:
			break;
     }
}

/*--------------------------------------------��ҵ1��ǽ���------------------------------------------------------*/



/*--------------------------------------------��ʼ������ҵ-------------------------------------------------------*/
void task1_UserTask_OneKeyCmd(void)
{
    //һ�����/��������
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;//һ����ɣ�һ�������־λ
    static u8 last_switch=0,now_switch=0;

/*-----------------------------ң�����ź������޸ģ�����-----------------------------*/
	
    //�ж���ң���źŲ�ִ��
    if (rc_in.no_signal == 0)//�յ��źű�־λΪ0
    {
			  //�жϵ�6ͨ������λ�� 800<CH_6<1200 1000  c�ſ��ز�����1��
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =  OneKey_Land(); //ִ��һ������  
            }
			now_switch=1;
		
			if(last_switch!=now_switch)
			{
				last_switch=now_switch;
				sprintf(buf1,"[x1] sound201 ");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
				
				LxStringSend(1,"1��");
			}
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
			
			//�жϵ�6ͨ������λ�� 1300<CH_6<1700  1500   c�ſ��ز�����2��
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {	
			if (one_key_takeoff_f == 0)
			{	
				one_key_mission_f = 0;
				mission_step = 1;
				
				now_switch=2;
			
				if(last_switch!=now_switch)
				{
					last_switch=now_switch;
					sprintf(buf1,"[x1] sound203 ");//\[b1] ��ӭ���٣����
					speak_len1=strlen(buf1);
					speak_context((u8*)buf1,speak_len1);
					
					LxStringSend(1,"2��");
					first_a0_voice=1;
					
					find_pole_flag=0;
					first_frame_end=0;
					second_frame_end=0;
			
					cmd_send1(0,0,0,0);
				}	
			}						
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
    
	//�жϵ�6ͨ������λ�� 1700<CH_6<2200  c�ſ��ز���������
		if(rc_in.rc_ch.st_data.ch_[ch_6_aux2]>1700 && rc_in.rc_ch.st_data.ch_[ch_6_aux2]<2200)
		{
			//��û��ִ��
			if(one_key_mission_f ==0)
			{
				//����Ѿ�ִ��
				one_key_mission_f = 1;
				//��ʼ����
				mission_step = 1;
				//���Բ�������			
			}
			
			now_switch=3;
			last_switch=3;
		}
		else
		{
			//��λ��ǣ��Ա��ٴ�ִ��
			one_key_mission_f = 0;		
		}
		
/*--------------------------------�����޸ģ�����------------------------------*/
		
/*----------------------------����������뿪ʼ--------------------------------*/
		
		if(one_key_mission_f==1)
		{
			static int time_dly_cnt_ms;
			switch(mission_step)
			{
				case 0:
				{
					time_dly_cnt_ms = 0;
					first_a0_voice=1;
				}
				break;
				case 1:
				{
					mission_step=mission_step+1;
					first_a0_voice=1;
					cmd_send1(0,0,0,0);
				}
				break;
				case 2://����
					mission_step += FC_Unlock();
					if(first_a0_voice==1)
					{
						sprintf(buf1,"[s8]����");//�ɿؽ���
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
						
						LxStringSend(1,"����");
						
						first_a0_voice=0;
					}
					first_a0_voice=1;
					break;
				case 3://��2��
					if(time_dly_cnt_ms<2000)
					{
						time_dly_cnt_ms+=20;//ms
						if (time_dly_cnt_ms<1000&&time_dly_cnt_ms>100)
						{
							cmd_send1(0,0,0,0);
						}
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
						first_a0_voice=1;
					}
					break;
				case 4: //ģʽ�л�
					mission_step += LX_Change_Mode(2);
				    time_dly_cnt_ms = 0;
					if(first_a0_voice==1)
					{
						sprintf(buf1,"[s9]ģʽ2��z������");//\[b1] ��ӭ���٣����
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
						LxStringSend(1,"ģʽ2��z������");
						first_a0_voice=0;
					}
					first_a0_voice=1;			
					break;
				case 5://��2��
					if(time_dly_cnt_ms<2000)
					{
						time_dly_cnt_ms+=20;//ms
						
						if (time_dly_cnt_ms<1000&&time_dly_cnt_ms>100)
						{
							cmd_send1(0,0,0,0);
						}
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
						first_a0_voice=1;
					}					
					break;
				case 6://һ�����		
					mission_step += OneKey_Takeoff(100);
					time_dly_cnt_ms = 0;
					if(first_a0_voice==1)
					{
						sprintf(buf1,"[s9]һ�����");//\[b1] ��ӭ���٣����
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
						LxStringSend(1,"һ�����");
					}
					break;
				case 7://���˻���ɳ�ʼ��
					if((time_dly_cnt_ms<5000))//&&(!((three_yaw_xm-standard_yaw<5)&&(three_yaw_xm-standard_yaw>-5))))
					{
						time_dly_cnt_ms+=20;//ms
						if(time_dly_cnt_ms%100==0)
						{
							speed_y_axis_16=0;
							speed_x_axis_16=0;
							
							target_yaw = standard_yaw;//����yaw��У׼
							speed_z_axis=p_altitude*(altitude_set-alt_add_xm);//PID�߶ȿ���
							speed_z_axis_16=(int16_t) speed_z_axis;
							calculate_shortest_rotation_angle(target_yaw);//�������Ӧ��yaw
							cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
						}
					}
					else
					{
						sprintf(buf1,"[s9]�뿪��ɵ�");//\[b1] ��ӭ���٣����
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
						LxStringSend(1,"�뿪��ɵ�");
					
						time_dly_cnt_ms = 0;
						mission_step += 1;
						first_a0_voice=1;		
						transition=0;
					}					
				break;
		    case 8://��2��	
				wait_flag+=20;//2s	
				time_dly_cnt_ms_xm+=20;//ms
				if(time_dly_cnt_ms_xm>=1000)
				{
					time_dly_cnt_ms_xm=0;
				}
				
				uart3_camera_command_analyse();//��ȡ��ǰλ��
				
				//��Ŀ��ִ������
				uart2_openmv();
//				if(transition<7){
					if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
					else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
//				}
				if(0==result_target)wait_flag=0;
				if(1==result_target&&wait_flag>=800)
				{
					time_dly_cnt_ms = 0;
					
					time_dly_cnt_ms_xm=0;
					time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
					if(first_a0_voice==1)
					{
						LxStringSend(1,"�����x������");
						first_a0_voice=0;		
					}
					
				
					if(return_flag==1&&wait_flag>=1600)
					{
						wait_flag=0;
						LxStringSend(1,"ǰ���Ż��");
						sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
					}
					if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
					{
//						if(transition!=7)
							transition+=1;//������һ��Ŀ��
							first_a0_voice=1;
						if(transition == 7)
						{ 
							 mission_step = 9;//ǰ��w1
							 first_a0_voice=1;		
							
						}
						
						LxStringSend(1,"ǰ����һ��ת�۵�");
						wait_flag=0;
					}
				}
				break;	
			
			case 9://����ת��
				wait_flag+=20;//2s	
				time_dly_cnt_ms_xm+=20;//ms
				if(time_dly_cnt_ms_xm>=1000)
				{
					time_dly_cnt_ms_xm=0;
				}
				
				uart3_camera_command_analyse();//��ȡ��ǰλ��
				
				//��Ŀ��ִ������
				uart2_openmv();
				if(transition<9)
				{
								if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
								else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
								
								
						if(0==result_target)wait_flag=0;
						if(1==result_target&&wait_flag>=800)
						{
								time_dly_cnt_ms = 0;
//								first_a0_voice=1;
								time_dly_cnt_ms_xm=0;
								time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
								
								if(first_a0_voice==1)
								{
									LxStringSend(1,"�����x������");
									first_a0_voice=0;		
								}
						
								if(return_flag==1&&wait_flag>=1600)
								{
										wait_flag=0;
										LxStringSend(1,"ǰ���Ż��");
										sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
										speak_len1=strlen(buf1);
										speak_context((u8*)buf1,speak_len1);
								}
								if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
								{
//                            if(transition!=9)
														transition+=1;
														first_a0_voice=1;
												
															if(transition==8)
																{
																	LxStringSend(1,"ǰ����һ��ת�۵�");
																}
																if(transition==9)
																{
																	LxStringSend(1,"׼����ͷ");
																}
														wait_flag=0;
										}
						}
		}
                    
                
                
                else//ִ�е�ͷ����?
                {
                    
                    
                    
                    
                    
                    
//                        target_yaw = standard_yaw1;
//                        now_standard_yaw = standard_yaw1;
//                        if(!((-10<three_yaw_xm-target_yaw)&&(three_yaw_xm-target_yaw<10)))//���з�ת�Ƕȹ���
//                        {
//                                speed_x_axis_16 = 0;
//                                speed_y_axis_16 = 0;
//                                altitude_set = 100;//����
//                                speed_z_axis=p_altitude*(altitude_set-alt_add_xm);//PID�߶ȿ���
//                                speed_z_axis_16=(int16_t) speed_z_axis;
//                                calculate_shortest_rotation_angle(target_yaw);//��ΪĿ��yaw��
//                                //�任����ϵ//��ȫ�ֱ���
//                                cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
//                        }
//                        else//�ȴ��Ƕ���ɷ�ת 
//                        {    
//                            fly_mode = 1;//�任����ϵ
//                            mission_step+=1;//ѰB
//													 first_a0_voice=1;	
//														LxStringSend(1,"��ͷ��ϣ�׼��ȥbǽ");
//														wait_flag=0;
//                        }


													 mission_step+=1;//ѰB
													 first_a0_voice=1;	
													 LxStringSend(1,"����Ҫ��ͷ��׼��ȥbǽ");
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                }
			break;		 
			case 10:  //
				{
					wait_flag+=20;//2s	
					time_dly_cnt_ms_xm+=20;//ms
					if(time_dly_cnt_ms_xm>=1000)
					{
						time_dly_cnt_ms_xm=0;
					}
					
					uart3_camera_command_analyse();//��ȡ��ǰλ��
					
					//��Ŀ��ִ������
					uart2_openmv();
//					if(transition<15){
						if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
						else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
//					}
					if(0==result_target)wait_flag=0;
					if(1==result_target&&wait_flag>=800)
					{
						time_dly_cnt_ms = 0;
//						first_a0_voice=1;
						time_dly_cnt_ms_xm=0;
						time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
						
						if(first_a0_voice==1)
						{
							LxStringSend(1,"�����x������");
							first_a0_voice=0;		
						}
					
						if(return_flag==1&&wait_flag>=1600)
						{
							wait_flag=0;
							LxStringSend(1,"ǰ���Ż��");
							sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
							speak_len1=strlen(buf1);
							speak_context((u8*)buf1,speak_len1);
						}
						if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
						{
//							if(transition!=15)
								transition+=1;//������һ��Ŀ��
								first_a0_voice=1;
							if(transition == 15) 
							{
								mission_step = 11;//ǰ��w1
								first_a0_voice=1;
							}
							LxStringSend(1,"ǰ����һ��ת�۵�");
							wait_flag=0;
						}
					}	
				}
		   break;
			
			
				case 11:
				wait_flag+=20;//2s	
				time_dly_cnt_ms_xm+=20;//ms
				if(time_dly_cnt_ms_xm>=1000)
				{
					time_dly_cnt_ms_xm=0;
				}
				
				uart3_camera_command_analyse();//��ȡ��ǰλ��
				
				//��Ŀ��ִ������
				uart2_openmv();
				if(transition<16)
                    {
                            if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
                            else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
                            if(0==result_target)wait_flag=0;
                        if(1==result_target&&wait_flag>=800)
                        {
                            time_dly_cnt_ms = 0;
                            
                            time_dly_cnt_ms_xm=0;
                            time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
                            
                           if(first_a0_voice==1)
													{
														LxStringSend(1,"�����x������");
														first_a0_voice=0;		
													}
                        
                            if(return_flag==1&&wait_flag>=1600)
                            {
                                wait_flag=0;
                                LxStringSend(1,"ǰ���Ż��");
                                sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
                                speak_len1=strlen(buf1);
                                speak_context((u8*)buf1,speak_len1);
                            }
                            if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
                            {
                                        transition+=1;
																				first_a0_voice=1;
                                    
                                        LxStringSend(1,"׼����ͷ");
                                        wait_flag=0;
                                }
                        }
                }
                    
                
                
                else//ִ�е�ͷ����?
                {
                    
                    
                    
                    
                    
                    
//                        target_yaw = standard_yaw;
//                        now_standard_yaw = standard_yaw;
//                        if(!((-10<three_yaw_xm-target_yaw)&&(three_yaw_xm-target_yaw<10)))//���з�ת�Ƕȹ���
//                        {
//                                speed_x_axis_16 = 0;
//                                speed_y_axis_16 = 0;
//                                altitude_set = 150;//����
//                                speed_z_axis=p_altitude*(altitude_set-alt_add_xm);//PID�߶ȿ���
//                                speed_z_axis_16=(int16_t) speed_z_axis;
//                                calculate_shortest_rotation_angle(target_yaw);//��ΪĿ��yaw��
//                                //�任����ϵ//��ȫ�ֱ���
//                                cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
//                        }
//                        else//�ȴ��Ƕ���ɷ�ת 
//                        {    
//                            fly_mode = 0;//�任����ϵ
//                            mission_step+=1;//ѰC
//														first_a0_voice=1;
//														 LxStringSend(1,"��ͷ��ϣ�׼��ȥcǽ");
//														wait_flag=0;
//                        }

													 mission_step+=1;//Ѱc
													 first_a0_voice=1;	
													 LxStringSend(1,"����Ҫ��ͷ��׼��ȥcǽ");
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                }
					break;			
/*---------------------�ָ��߶�1.8���ָ�ǰһ��Ѳ�ߵص�--------------------*/		
		    case 12:
				{
					wait_flag+=20;//2s	
					time_dly_cnt_ms_xm+=20;//ms
					if(time_dly_cnt_ms_xm>=1000)
					{
						time_dly_cnt_ms_xm=0;
					}
					
					uart3_camera_command_analyse();//��ȡ��ǰλ��
					
					//��Ŀ��ִ������
					uart2_openmv();
					if(transition<22){
						if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
						else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
					}
					if(0==result_target)wait_flag=0;
					if(1==result_target&&wait_flag>=800)
					{
						time_dly_cnt_ms = 0;
						
						time_dly_cnt_ms_xm=0;
						time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
						
						if(first_a0_voice==1)
						{
							LxStringSend(1,"�����x������");
							first_a0_voice=0;		
						}
					
						if(return_flag==1&&wait_flag>=1600)
						{
							wait_flag=0;
							LxStringSend(1,"ǰ���Ż��");
							sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
							speak_len1=strlen(buf1);
							speak_context((u8*)buf1,speak_len1);
						}
						if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
						{
							if(transition!=22)
							{
								transition+=1;//������һ��Ŀ��
								first_a0_voice=1;
							}
								
							if(transition == 22)
							{
								 mission_step = 13;//ǰ��w1
								first_a0_voice=1;
							}
								
							LxStringSend(1,"ǰ����һ��ת�۵�");
							wait_flag=0;
						}
					}	
				}
				break;	
				
				
				case 13:{
				wait_flag+=20;//2s	
				time_dly_cnt_ms_xm+=20;//ms
				if(time_dly_cnt_ms_xm>=1000)
				{
					time_dly_cnt_ms_xm=0;
				}
				
				uart3_camera_command_analyse();//��ȡ��ǰλ��
				
				//��Ŀ��ִ������
				uart2_openmv();
				if(transition<24)
          {
								if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
								else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
								if(0==result_target)wait_flag=0;
								if(1==result_target&&wait_flag>=800)
								{
										time_dly_cnt_ms = 0;
									 
										time_dly_cnt_ms_xm=0;
										time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
										
									 if(first_a0_voice==1)
									{
										LxStringSend(1,"�����x������");
										first_a0_voice=0;		
									}
								
										if(return_flag==1&&wait_flag>=1600)
										{
												wait_flag=0;
												LxStringSend(1,"ǰ���Ż��");
												sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
												speak_len1=strlen(buf1);
												speak_context((u8*)buf1,speak_len1);
										}
										if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
										{
																transition+=1;
																first_a0_voice=1;
																if(transition==23)
																{
																	LxStringSend(1,"ǰ����һ��ת�۵�");
																}
																if(transition==24)
																{
																	LxStringSend(1,"׼����ͷ������dǽ");
																}
																
																wait_flag=0;
															 
										 } 
										
								}
						}
                    
                
                
                else//ִ�е�ͷ����?
                {
                    
                    
                    
                    
                    
                    
//                        target_yaw = standard_yaw1;
//                        now_standard_yaw = standard_yaw1;
//                        if(!((-10<three_yaw_xm-target_yaw)&&(three_yaw_xm-target_yaw<10)))//���з�ת�Ƕȹ���
//                        {
//                                speed_x_axis_16 = 0;
//                                speed_y_axis_16 = 0;
//                                altitude_set = 150;//����
//                                speed_z_axis=p_altitude*(altitude_set-alt_add_xm);//PID�߶ȿ���
//                                speed_z_axis_16=(int16_t) speed_z_axis;
//                                calculate_shortest_rotation_angle(target_yaw);//��ΪĿ��yaw��
//                                //�任����ϵ//��ȫ�ֱ���
//                                cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
//                        }
//                        else//�ȴ��Ƕ���ɷ�ת 
//                        {    
//                            fly_mode = 0;//�任����ϵ
//                            mission_step+=1;//ѰC
//													  first_a0_voice=1;
//														LxStringSend(1,"��ͷ��ϣ�׼��ȥdǽ");
//														wait_flag=0;
//                        }
													 mission_step+=1;//ѰB
													 first_a0_voice=1;	
													 LxStringSend(1,"����Ҫ��ͷ��׼��ȥdǽ");
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                }
				
				
				}break;
				case 14:{
					wait_flag+=20;//2s	
					time_dly_cnt_ms_xm+=20;//ms
					if(time_dly_cnt_ms_xm>=1000)
					{
						time_dly_cnt_ms_xm=0;
					}
					
					uart3_camera_command_analyse();//��ȡ��ǰλ��
					
					//��Ŀ��ִ������
					uart2_openmv();
					if(transition<30){
						if(return_flag == 0x01) result_target = find_target(pole_x,pole_y,altitude_set);
						else result_target=find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);//����Ŀ���
					}
					if(0==result_target)wait_flag=0;
					if(1==result_target&&wait_flag>=800)
					{
						time_dly_cnt_ms = 0;
//						first_a0_voice=1;
						time_dly_cnt_ms_xm=0;
						time=0;//��Ϊ���µ�20msѰ��λ�õı�־λ
						
						if(first_a0_voice==1)
						{
							LxStringSend(1,"�����x������");
							first_a0_voice=0;		
						}
					
						if(return_flag==1&&wait_flag>=1600)
						{
							wait_flag=0;
							LxStringSend(1,"ǰ���Ż��");
							sprintf(buf1,"[s9]ǰ���Ż��");//\[b1] ��ӭ���٣����
							speak_len1=strlen(buf1);
							speak_context((u8*)buf1,speak_len1);
						}
						if(wait_flag>=1600&&!return_flag)//�ȴ�ʱ��Ϊ1.6s��openmvû�з����κ�������������Ѳ��
						{
							if(transition!=30)
								transition+=1;//������һ��Ŀ��
							first_a0_voice=1;
							if(transition == 30) 
							{
								 mission_step = 25;//ǰ���ؼ�
								first_a0_voice=1;
							}
							LxStringSend(1,"ǰ����һ��ת�۵�");
							wait_flag=0;
						}
					}	
				
				}break;
/*----------------------------------------Ŀǰ�߶�1.8�ף�׼���ؼҽ���---------------------------*/
	
		    case 25:
			
				//��2��
					time_dly_cnt_ms_xm+=20;//ms
					if(time_dly_cnt_ms_xm>=10000)
					{
						time_dly_cnt_ms_xm=0;
					}
					
					uart3_camera_command_analyse();
					
					if ((plant_x>=sow_piont[transition].x-0.15f)&&(plant_x<=sow_piont[transition].x+0.15f)&&(plant_y>sow_piont[transition].y-0.15f)&&(plant_y<=sow_piont[transition].y+0.15f))
					{
						sow_seed();
						index_sow_i=0;
						
						sprintf(buf1,"[s9]�����𽵵㣬׼��z��0��");//\[b1] ��ӭ���٣����
						speak_len1=strlen(buf1);
						speak_context((u8*)buf1,speak_len1);
						
						if(first_a0_voice==1)
						{
							LxStringSend(1,"�����𽵵㣬׼��z��0��");//LxStringSend(1,"�����x������");
							first_a0_voice=0;		
						}
						
						
						time_dly_cnt_ms = 0;
						mission_step += 1;
						first_a0_voice=1;
						time_dly_cnt_ms_xm=0;
					 }
					 else
					 {
							find_target(sow_piont[transition].x,sow_piont[transition].y,sow_piont[transition].z);
						 
						 
						target_yaw = now_standard_yaw;
						calculate_shortest_rotation_angle(target_yaw);	
					}			

				
			
			break;	
			case 26://z��0��
				{
					time_dly_cnt_ms_xm+=20;//ms		
					
					
					if(time_dly_cnt_ms_xm%100==0) //����ÿ20ms����һ�Σ�100/20=5��ʵ��ʱ�䣺5*20=100ms=0.1s
					{
						//uart2_openmv();
						//ֹͣ��
						speed_x_axis_16=0;
						speed_y_axis_16=0;
						
						speed_z_axis=0;//int16_t
					
						speed_z_axis_16=(int16_t) speed_z_axis;
						
						target_yaw = now_standard_yaw;
						calculate_shortest_rotation_angle(target_yaw);
						cmd_send1(speed_x_axis_16,speed_y_axis_16,speed_z_axis_16,yaw_16);
					}

					if(time_dly_cnt_ms_xm>=1000)
					{
						LxStringSend(1,"z��0��,׼��һ������");
					
						time_dly_cnt_ms = 0;
						mission_step += 1;
						first_a0_voice=1;
						time_dly_cnt_ms_xm=0;
					}	
					break;						
				}
			
		case 27://ִ��һ������
			OneKey_Land();
			
			time_dly_cnt_ms = 0;
			mission_step += 1;
					
			if(first_a0_voice==1)
			{
				sprintf(buf1,"[s9]����");//\[b1] ��ӭ���٣����
				speak_len1=strlen(buf1);
				speak_context((u8*)buf1,speak_len1);
				
				LxStringSend(1,"����");
				
				first_a0_voice=0;
			}
			transition = 0;
			break;			
	case 28:
		{
			//ִ��һ������
			
				time_dly_cnt_ms+=20;//ms
				first_a0_voice=0;
		}
		break;
		case 29:
		{
			time_dly_cnt_ms+=20;//ms
		}
		break;				
		default:
			break;
	}
		}
	}
	else
	{
		mission_step = 0;
	}
		
}
    ////////////////////////////////////////////////////////////////////////
