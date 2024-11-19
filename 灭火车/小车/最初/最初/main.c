#include "ultrasonic.h"
#include "action.h"
#include "motor.h"
#include "key.h"

bit candle_fire_4=0x01;// 1 is fire on ; 0 is extinguish
bit candle_fire_5=0x00;//  1 is fire on ; 0 is extinguish
bit candle_fire_6=0x00;////  1 is fire on ; 0 is extinguish
bit candle_fire_7=0x01;////  1 is fire on ; 0 is extinguish
bit candle_fire_8=0x01;////  1 is fire on ; 0 is extinguish
sbit Fan = P0^0;
sbit LeftFire = P0^2;
sbit RightFire = P0^3;

#define Fan_On {Fan = 0;}
#define Fan_Off {Fan = 1;}

unsigned int left_300;
unsigned int blow=800;//吹风时间
unsigned char d;

void main()
{
	LED = 1;
	Timer0Init();//左轮PWM
	Timer1Init();//右轮PWM
	Ultrasonic_Init();
	while(1)
	{ 
		keyscan();
//		while(1){k();}
		if(start==1)
		{
			front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
			front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
			front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
			front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
			right_dis= Ultrasonic_Get_Right();
			left_dis= Ultrasonic_Get_Left();
			right_dis= Ultrasonic_Get_Right();
			left_dis= Ultrasonic_Get_Left();
			right_dis= Ultrasonic_Get_Right();
			left_dis= Ultrasonic_Get_Left();
			
		
//		//测试直行
//		
////			while(1){forward();}
//			
//					//			//测试转向
//					d=4;
//					while(d)
//					{
//					car_stop();
//					delay_ms(50);
//					car_stop();
//					turn_right_90();
//					car_stop();
//					delay_ms(1000);
//					d--;
//					}
//					while(1){k();}
					//
		
		
			goto aaa;
		//////////////////
			aaa:
			if(candle_fire_4==0x00&&candle_fire_5==0x00&&candle_fire_6==0x00&&candle_fire_7==0x00&&candle_fire_8==0x00)
			{
				car_stop();
				break;	
			}
		//////////////////
			
//开机出发，巡右墙，在离墙180毫米处停下
//并在3#街道停下来，离墙180毫米停。
// now potion is cornre 3 of  room 2,next action  walk alone righr wall
		///////////////////
			bbb:
			do
			{
				Circulation_wall_right();				
			}
			while(front_dis >480);
		//////////////////

// now potion is cornre 4 of room 2,next action  turn left
			do
			{
				Circulation_wall_right();
			}
			while(front_dis >190);
			car_stop();
			delay_ms(50);
			turn_left_90();
			
			
			//直接屏蔽掉右方超声波检测引起的误判
			car_stop();
//			i=0;
//			do
//			{
//			  car_stop();
//			}
//			while(i<2);
			
// 判断3号房有火吗
		//火全部在第4号房间    		模式1   7  8火 			
			if(candle_fire_4==0x00&&candle_fire_5==0x00&&candle_fire_6==0x00)
			{
				//火在4号房间
/////////////						
//这时小车4号大街尽端，准备进入4号房间，不去3号房间，
///////////////因为3号房间刚刚判断过，没有火源
/////////////////
				i = 0;
				do
				{
					Circulation_wall_right();
				}
				while(i<95);							
				car_stop();
				delay_ms(50); 
				turn_left_90();				
/////////////						
//这时小车2号大街终端，4号房间大门处，车头已经对准4#房间大门。准备进入4号房间
				fire_8_fire_7_is_fire_on:						
				if (candle_fire_8==0x01)
				{
					_left_dis=170;
					//4号房间的火源是8号，至于7号是否也着火，下面会在灭完8号火后进行判断
					do
					{
						Circulation_wall_left();//   PID调整距离前进  保持中心距离 
					}
					while(front_dis >190);
			// now potion is cornre 4 of room 2,next action  turn left
					car_stop();
					car_stop();
					delay_ms(50);
					do
					{
						Fan_On;
						delay_ms(blow);//1100
						break;
					}
					while(LeftFire==0||RightFire==0);   /////为0火是点燃状态 不跳出循环 让它不断的吹
					delay_ms(500);
					//害怕死灰复燃，停2秒，再次检测，
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					delay_ms(1000);
					
					//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					Fan_Off;
					delay_ms(500);
					car_stop();
					_left_dis=180;
				}
				else         
				{
					_left_dis=170;
					//4号房间的火源在7号
					do
					{
						Circulation_wall_left();
					}
					while(front_dis >190);
				// now potion is cornre 4 of room 2,next action  turn left
					car_stop();
				}
				if(candle_fire_7==0x01)
				{
					delay_ms(50);
					//准备前往7号蜡烛，先把车头右转90度，巡左墙
					turn_right_90();
					do
					{
						Circulation_wall_left();
					}
					while(front_dis >170);
	// now potion is cornre 4 of room 2,next action turn left
					car_stop();
	//now position is fire 1,and next step put off fire 1		
					car_stop();
					do
					{ 
						Fan_On;
						delay_ms(blow);//1100
						break;
					}
					while(LeftFire==0||RightFire==0);
					delay_ms(500);
					//害怕死灰复燃，停2秒，再次检测，
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					delay_ms(1000);
					//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					Fan_Off;
					delay_ms(500);
			//now position is fire 1,and next turn back				
//			  find_fire_turn_back_right();
			//now position is fire 1,and next turn left 			
					car_stop();
					_right_dis =170;
	//灭火结束，回家，原地掉头180度，巡右墙，左转90度，车头对准大门
					turn_right_180();  
					do
					{
						Circulation_wall_right();
					}
					while(front_dis >210);
					car_stop();
					delay_ms(50);
					turn_left_90();
				}
				else
				{
					_right_dis =170;
					delay_ms(150);
					//回家，即原地180度掉头，车头对准大门
					turn_right_180();
				}
				car_stop();
//小车目前处于4号房间4号拐角，车头对准大门，巡右墙出4号房间大门。
//准备驶出4#房间，并在3#街道停下来，离墙180毫米停。
////////////////////
				do
				{
					Circulation_wall_right();
				}
				while(front_dis>485);///qian 485
				do
				{
					Circulation_wall_right();
				}
				while(front_dis >200);
				car_stop();
				delay_ms(50);
				turn_right_90();
				car_stop();						
			/////////////						
//这时小车4号房间大门口，准备进入4#大街
///////////////
				i = 0;
				do
				{
					Circulation_wall_left();
				}
				while(i<=120);//115
				car_stop();
				delay_ms(50);
				turn_right_90_small();
				delay_ms(30);
//				delay_ms(50);
				car_stop();
/////////////				
//这时小车4#大街，车头对准出发点，准备巡左墙回家
///////////////
				_right_dis = 160;//改变右边pid
				do	
				{
					Circulation_wall_right();
				}
				while(front_dis >600);		
				ccc:
				do	
				{
					Circulation_wall_right();
				}
				while(front_dis >550);		
				do
				{
					Circulation_wall_right();
				}
				while(front_dis >395);						
				
				car_stop();
				car_stop();
				
				front_dis= Ultrasonic_Get_Front();
				front_dis= Ultrasonic_Get_Front();
				front_dis= Ultrasonic_Get_Front();
				front_dis= Ultrasonic_Get_Front();
				
				if(Ultrasonic_Get_Front() <= 395)
				{
					car_stop();
					car_stop();
				}
				
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				
				if(kalmanfilter(Ultrasonic_Get_Front(),1) <= 395)
				{
					car_stop();
					car_stop();
				}
				
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
				
				if(kalmanfilter(Ultrasonic_Get_Front(),1) <= 395)
				{
					car_stop();
					car_stop();
				}
				
				if(kalmanfilter(Ultrasonic_Get_Front(),1) > 395)
				{
					front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
					front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
					goto ccc;
				}
				
				
				else if(kalmanfilter(Ultrasonic_Get_Front(),1) <= 395)
				{
					car_stop();
					car_stop();
					delay_ms(100);
					car_stop();
					car_stop();
					while(1);
				}
				
				car_stop();
				car_stop();
				delay_ms(100);
				car_stop();
				car_stop();
				while(1);	
			}
			goto wrong_judge_have_left_wall;					
//////////////////////////
//3#房间有火，至于4号房间是否有火，回来的路上到了6#蜡烛在判断，
//先行前往3号房间
//目前小车位置是在4号大街尽头，车头朝向6号蜡烛
//这时小车处于3#大街，巡右墙走，并扫描左墙，稳定发现左墙
///////////////					
			wrong_judge_have_left_wall:			
			i=0;
			do
			{
				Circulation_wall_right();
//				right_dis= Ultrasonic_Get_Right();
			}
			while(i<=20);
			do
			{
				Circulation_wall_right();
//				right_dis= Ultrasonic_Get_Right();
			}
			while(right_dis<=410);
			////////////////
			car_stop();
			car_stop();
			i=0;
			do
			{
				Circulation_wall_left();
			}
			while(i<60);//50
			car_stop();
			delay_ms(50);
			car_stop();
			if (candle_fire_6==0x01)//6号蜡烛有火，前往灭火
			{
				do
				{
					Fan_On;
					delay_ms(blow);//1100
					break;
				}
				while(LeftFire==0||RightFire==0);
				delay_ms(500);
				//害怕死灰复燃，停2秒，再次检测，
				while(LeftFire==0||RightFire==0)
				{
					Fan_On;
					delay_ms(500);//1000
					break;
				}
				delay_ms(1000);
				//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
				while(LeftFire==0||RightFire==0)
				{
					Fan_On;
					delay_ms(500);//1000
					break;
				}
				Fan_Off;
			//	delay_ms(1000);
				delay_ms(500);
				car_stop();
			}
			//6号蜡烛已经灭完火了，在判断5号，4号蜡烛是否着火
			if(candle_fire_5==0x01||candle_fire_4==0x01)
				//5号，6号蜡烛是否着火
			{
				delay_ms(150);
				//前往4号或是5号着火点灭火，此时车况是3#大街尽头，
				//车头朝向6#蜡烛
				turn_right_90();
				car_stop();
				delay_ms(50);
				_left_dis=180;//改变左pid
				if(candle_fire_5==0x01)//前往灭5号蜡烛的火
				{
					i=0;
					do
					{
//							forward();
						Circulation_wall_left();
					}while(i<20);
					do
					{
						Circulation_wall_left();
					}while(front_dis >190);
					car_stop();
					car_stop();
					delay_ms(50);
//					_left_dis=180;//改变左pid
					do
					{
						Fan_On;
						delay_ms(blow);//1100
						break;
					}
					while(LeftFire==0||RightFire==0);
					delay_ms(500);
					//害怕死灰复燃，停2秒，再次检测，
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					delay_ms(1000);
					//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					Fan_Off;
					delay_ms(500);
			//now position is fire 1,and next turn back				
			//now position is fire 1,and next turn left 			
					car_stop();
//判断4#蜡烛是否着火，着火就去4#蜡烛处灭火，不着火，返回
//目前车处于5号蜡烛处，车头正对5#蜡烛
					if (candle_fire_4==0x01)
//右转90度，巡左墙，去3#房间的3号拐角，依次4号蜡烛，灭火返回值6号蜡烛处
					{
						delay_ms(50);
						turn_right_90();
/////////////			
//这时小车处于5#着火点，准备离开着火点。巡左墙走，
//并在前墙170毫米处停下
///////////////							
						do
						{
							Circulation_wall_left();
						}
						while(front_dis >210);
						car_stop();
						delay_ms(50);
						turn_right_90();
						_left_dis=175;
/////////////			
//这时小车处于3#房间拐角3，准备离去4#着火点。巡左墙走，
//并在前墙210毫米处停下
///////////////						
						do
						{
							Circulation_wall_left();
						}
						while(front_dis >170);
						car_stop();
						//灭火开始						
//						find_fire_turn_left();
		//now position is fire 1,and next step put off  fire 1		
						car_stop();
						do
						{
							Fan_On;
							delay_ms(blow);//1100
							break;
						}
						while(LeftFire==0||RightFire==0);
						delay_ms(500);
						//害怕死灰复燃，停2秒，再次检测，
						while(LeftFire==0||RightFire==0)
						{
							Fan_On;
							delay_ms(500);//1000
							break;
						}
						delay_ms(1000);
						//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
						while(LeftFire==0||RightFire==0)
						{
							Fan_On;
							delay_ms(500);//1000
							break;
						}
						Fan_Off;
						delay_ms(500);
				
						car_stop();/////////原始程序回家方式、
						turn_right_90();
						
						goto fire4_go_to_fire6_NO2;										
									
					}
					else//180度掉头，巡右墙，回到6号蜡烛处
					{
						
						delay_ms(50);
						turn_right_180();
						do
						{
							Circulation_wall_right();
						}
						while(front_dis >190);
						car_stop();
						turn_left_90();
						goto street_3_end_car_forward_up;
						//////////6号处，车头背对火焰
					}
				}
				else///////////////这表明 4号有火，五号没火/////////////////
				{
					front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
					front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
					front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
					i=0;
					do
					{
						Circulation_wall_left();
					}while(i<20);
					do
					{
						Circulation_wall_left();
					}
					while(front_dis >=190);
					car_stop();
					_left_dis=175;//此时在5号火处，准备到四号火
					delay_ms(20);
					turn_right_90();
					do
					{
						Circulation_wall_left();
					}
					while(front_dis >=210);
					car_stop();
					delay_ms(20);
					turn_right_90();
					do
					{
						Circulation_wall_left();
					}
					while(front_dis >=170);//5号火
					car_stop();
					delay_ms(50);
					do
					{
						Fan_On;
						delay_ms(blow);//1100
						break;
					}
					while(LeftFire==0||RightFire==0);
					delay_ms(500);
					//害怕死灰复燃，停2秒，再次检测，
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000
						break;
					}
					delay_ms(1000);
					//害怕死灰复燃，停0.5秒，再次检测，保证吹灭
					while(LeftFire==0||RightFire==0)
					{
						Fan_On;
						delay_ms(500);//1000 
						break;
					}
					
					Fan_Off;
					delay_ms(500);
				
					car_stop();
					turn_right_90();
				goto fire4_go_to_fire6_NO2;
				}
			}
			else
			//蜡烛都没有着火，准备返回到4号房间大门口，
			//在门口中央判断是否需要进入4号房间灭火
			{
				delay_ms(50);
				//返回到4号房间大门口，在门口中央判断
				//是否需要进入4号房间灭火
				//此时车况是3#大街尽头，车头朝向6#蜡烛
				turn_right_180();
				goto street_3_end_car_forward_up;
			}
			fire4_go_to_fire6_NO2:
			i = 0;
			do
			{
				Circulation_wall_left();			
			}
			while(i<=150);
			do
			{
				Circulation_wall_left();
				//adjustDirection_1(1700,1300);
				front_dis = Ultrasonic_Get_Front();
			}
			while(front_dis >170);
			car_stop();
			delay_ms(50);
			turn_left_90_low();
			do
			{
				Circulation_wall_right();
			}
			while(front_dis >180);
			car_stop();
			delay_ms(150);
			turn_left_90();
			car_stop();
			//从六号到8号
		street_3_end_car_forward_up:						
			left_300=0x00;
			do
			{
				Circulation_wall_right();
				//
				left_dis= Ultrasonic_Get_Left();	
				//
				if(left_dis<400)
				{
 					left_300++;
				}
				else
				{
					left_300=0x00;
				}
			}
			while(left_dis>300||left_300<=1);//front_dis > 580||left_dis <600
/////////////						
//这时小车处于3#大街，发现左墙，准备巡左墙走，并在前墙170毫米处停下
			do
			{
				Circulation_wall_right();
			}
			while(right_dis<=400);
//			car_stop();
//			car_stop();
			i = 0;
			do
			{
				Circulation_wall_left();
//				forward();
			}
			while(i<=55);//35
			if(candle_fire_7==0x01||candle_fire_8==0x01)
			{
				car_stop();
				car_stop();
				delay_ms(50);
				turn_right_90();
				delay_ms(5);
				car_stop();
				car_stop();
				delay_ms(50);
				goto fire_8_fire_7_is_fire_on;
			}
			else
			{
					/////////////						
//这时小车4号房间大门口，准备进入4#大街
//////////////
				i = 0;
				do
				{
					Circulation_wall_left();
				}
				while(i<=115);	//115	与车速有关90			
				// now potion is middle of door of room 2,next action isturn right 90	
				car_stop();
				delay_ms(50);
				turn_right_90_small();
				delay_ms(30);
				
				_right_dis = 160;
		/////////////						
		//这时小车4#大街，车头对准出发点，准备巡左墙回家
		///////////////
				do
				{
					Circulation_wall_right();
				}
				while(front_dis>600);		
				do
				{
					Circulation_wall_right();
				}
				while(front_dis>550);		
				do
				{
					Circulation_wall_right();
				}
				while(front_dis>395);						
				car_stop();	
				car_stop();					
				while(1);
			}
		}
	}
}