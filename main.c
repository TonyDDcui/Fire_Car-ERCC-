#include "ultrasonic.h"
#include "action.h"
#include "motor.h"
#include "key.h"
#include "stdlib.h"

sbit Fan = P0^0;
sbit LeftFire = P0^2;
sbit RightFire = P0^3;

#define Fan_On {Fan = 0;}
#define Fan_Off {Fan = 1;}
#define lim_dis 240

int dis_car[3] = {0,0,0};
int car_road = 0;
volatile  int time = 0;
volatile int run_time = 0;
int sure_step = 0;
unsigned char task = 0;

unsigned char prevent_time = 0;
unsigned int left_300 = 0;
unsigned int right_900 = 0;
unsigned int pwm_num = 0;
unsigned int room_2_pwm_num = 0;
unsigned int left_time=0;

void put_off(int blow_time);
void get_pos(void);
int make_sure_dis(void);
void make_car_road(void);
void cor_dis(unsigned char dir);


/******************主函数***************
**************实现功能移动灭火**********/
void main()
{
	LED = 1;//亮灯指示
	Timer0Init();//左轮PWM
	Timer1Init();//右轮PWM
	Ultrasonic_Init();
	
	while(1)
	{ 
		keyscan();//按键检测
		for(sure_step = 0;sure_step<3;sure_step++)
    {
			get_pos();
      dis_car[sure_step] = front_dis;
    }
		
		if(start==1 && make_sure_dis() && user_push == 1)
		{
program_begin:
			car_road = fire_pos[0]*4 + fire_pos[1]*3 + fire_pos[2]*2 + fire_pos[3]*0;
			do								   
			{	
				Circulation_wall_right();								
			}
			while(front_dis > 185);		
				
			if(fire_pos[3] == 1)	
			{
				put_off(1500);

				if(car_road == 0)	goto only_fire1_is_on_back_home;
			}
 				
			car_stop();
			
			turn_left_90_small();//
			car_stop();
			
			if(car_road != 2)
			{
					do
					{
						Circulation_wall_right();	
						cor_dis(1);
					}
					while(front_dis > 210);//180   
					
					car_stop();
					
					turn_left_90_small();//左转90°进去一号房间和二号房间的中间过道
				
					do{Circulation_wall_right();Circulation_wall_right();}while(right_dis<300);
					do{Circulation_wall_right();Circulation_wall_right();}while(right_dis<300);
					do{Circulation_wall_right();Circulation_wall_right();}while(right_dis<300);
					
					i=0;
					do{forward();}while(i<54);
					
					turn_right_90();//右转90°进入4街道
					car_stop();
					
					do
					{
						forward();
						get_pos();
					}while(left_dis > 240);
					
					do	
					{
						Circulation_wall_left();//寻左墙保持18cm
						cor_dis(0);
					}while(front_dis >230);//180,210-5.20

					car_stop();	//停在3火拐角，向西
					
					if(car_road == 3 || car_road == 5)
					{
						turn_left_90();
						car_stop();					
						put_off(1500);
					}	
					else
					{
							turn_right_90();//右转90°
							car_stop();
							front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
							front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
							front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
							front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
							
							do
							{
								Circulation_wall_left();
								cor_dis(0);
							}while(front_dis >185);//前距离20cm，190-5.20     190可以使用 稍微近一些  
						
							car_stop();
							turn_right_90();
							car_stop();

							do
							{
								Circulation_wall_left();
								cor_dis(0);
							}while(front_dis >185);
				
							car_stop();
							delay_ms(100);
							
							turn_left_90();
							car_stop();
							
							put_off(1500);
						
							turn_left_90();
							car_stop();
						
							do
							{
								Circulation_wall_right();
								cor_dis(1);
							}while(front_dis >210);//170,190-5.20     190可以使用 稍微近一些---------------改  3.24  195 还是有点近   198
							car_stop();
							turn_left_90();
							car_stop();
							do
							{
								Circulation_wall_right();
								cor_dis(1);
							}while(front_dis >185);  //180可以走 稍微远    改  170太近了  -------------------改  3.24  175  178有点近   180可但还需要调节//185
							car_stop();
							if(car_road == 7 || car_road == 9)	put_off(1500);
						}
						turn_left_90();	
						goto only_fire_1_fire_9_fire_3_is_on_back_home;
			}

			else
			{
					get_pos();
					i=0;
					do								   //一号房间
					{
						Circulation_wall_right();
					}
					while(i<=35);
					//为了防止误判，发现一号房间门口 执行两次。因为调试过程中在火焰1处应为前方超声测距错误，测成514毫米，实际应为1200毫米
					//，导致满足while(front_dis > 550||left_dis <600)条件，是的单片机本在火焰1误认为在一号房间门口，所以写两次
					//希望前方超声测距不要猜测错了。
							
					delay_ms(100);
						
					left_dis= Ultrasonic_Get_Left();//左侧检查
					right_dis= Ultrasonic_Get_Right();//右侧检查
					
					do								   //进入一号房间前——寻找右墙
					{
						Circulation_wall_right();
						left_dis= Ultrasonic_Get_Left();
					}
					while(front_dis > 560||left_dis <600);  //front_dis=590
					
					car_stop();
					//now position is middle gate of room 1,and next action turn left 
					delay_ms(50);		
					turn_left_90_more_small();
					goto goto_fire_2;
			}
/////////////////
//
//在着火点3位置，准备驶出2#房间，并在3#街道停下来，离墙180毫米停。
////////////////////						
					// now potion is cornre 3 of  room 2,next action  walk alone righr wall
only_fire_1_fire_9_fire_3_is_on_back_home:												
						do
						{
							Circulation_wall_right();
						}while(front_dis >410);
					// now potion is cornre 4 of room 2,next action  turn left
						car_stop();
						front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
						front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
						front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
						front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
						do
						{						    
//						 forward();
							Circulation_wall_right();
						 
						}while(front_dis >188);  //9.28改
						car_stop();
						turn_left_90();
						car_stop();
					// now potion is  door of room 2,and meanwhile at the street 3,next action   walk alone righr wall
/////////////						
//这时小车处于3#大街，巡右墙走，并扫描左墙，稳定发现左墙
///////////////

						left_300=0x00;
						do
						{
							Circulation_wall_right();
							left_dis= Ultrasonic_Get_Left();
							
							if(left_dis<300)
							{
								left_300++;
							}
							else
							{
								left_300=0x00;
							}
						}
						while(left_dis>300||left_300<=3);//front_dis > 580||left_dis <600
						
						
						
/////////////						
//这时小车处于3#大街，发现左墙，准备巡左墙走，并在前墙170毫米处停下
///////////////						
	
						do							
						{
							Circulation_wall_left();
						}while(front_dis >180);   //200可以使用   可以调大一些  ------------改210  3.24
						car_stop();
						turn_right_90_small();
						
            //前往6/9/5号房间            
            if(car_road == 6 || car_road == 9 || car_road == 5)	goto car_room_1;
/////////////						
//这时小车处于2#大街尽头，准备巡左墙走，并在前墙170毫米处停下
///////////////							
						do
						{
							Circulation_wall_left();
							if(left_dis > lim_dis) 
							{
									do
									{
											adjustDirection_1(1350,1350);
											get_pos();
									}while(left_dis > lim_dis);
							}
						}while(front_dis >180);//原数据180
						car_stop();
						
						turn_right_90_more_small();
						
						goto only_fire_9_fire_1_is_on_back_home;

/////////////						
//这时小车处于1#着火点，准备巡左墙走，并在前墙580毫米处停下，并掉头，为下次启动做准备
///////////////	
only_fire_9_fire_1_is_on_back_home:	

						do
						{
							Circulation_wall_right();		
						}
						while(front_dis > 550);
						
						do
						{
							Circulation_wall_right();			
						}
						while(front_dis > 500); 
						
						do
						{
							Circulation_wall_right();			
						}
						while(front_dis > 450); 
						
						do
						{
							Circulation_wall_right();			
						}
						while(front_dis > 430); 
                        
						car_stop();
						while(rand_pushed != 1)	keyscan();
						turn_right_180();
						make_car_road();
						goto program_begin;
						
car_room_1:
		run_time = 0;
		do
		{
			Circulation_wall_left();
			if(left_dis > lim_dis) 
			{
					do
					{
							adjustDirection_1(1350,1350);
							get_pos();
					}while(left_dis > lim_dis);
			}
			else run_time++;
		}while(run_time <= 130);
		
		car_stop();
		turn_right_90_small();
		
		do
		{
			get_pos();
			forward();
			if(front_dis < 240)
			{
				do
				{
					get_pos();
					adjustDirection_1(1700,1300);
				}while(front_dis < 240);
			}
		}while(right_dis > 200);
		
		goto goto_fire_2;

			
goto_fire_2:									
				do
				{
					Circulation_wall_right();
				}
				while(front_dis > 220);//改为20-5.19，23-5.20   230  250    230  210合适
				delay_ms(50);//-------------------
			//now position is corner 1 of room 1,and next action turn left
				car_stop();
				turn_left_90();							//左转			
						
				//now position is corner 1 of room 1,and next action wanlk alone with right wall
				do
				{
					Circulation_wall_right();	//寻右墙	
					cor_dis(1);
				}while(front_dis > 210);//原数据185改160-5.18，19-5.20,21-5.25

				put_off(1500);
				
				turn_right_180();//右转有180°不足——已解决
				
				i=0;
				do
				{
					Circulation_wall_left();	//寻左墙
					cor_dis(0);
				}
				while(left_dis < 240 && i < 24);//i原数据为12改为-24，5-18//390
				
				do
				{
					Circulation_wall_left();	//寻左墙	
					cor_dis(0);
				}
				while(front_dis > 200);
				car_stop();
				turn_right_90_small();			

				do
				{
					//front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);//卡尔曼滤波
					Circulation_wall_left();//寻左墙		
				}
				while(front_dis > 230);//驶出到1号房间出口1#街区处前墙19cm   23cm合适
				car_stop();
						
				turn_right_90();//-----------------回家
				
				do
				{
					Circulation_wall_left();	//寻左墙	
					cor_dis(0);
				}
				while(front_dis > 180);//前方20cm处//240
				turn_right_90_more_small();
				goto only_fire_9_fire_1_is_on_back_home;//灭蜡烛1和蜡烛9后回家
				

						
only_fire1_is_on_back_home:	
				turn_right_180();
				goto only_fire_9_fire_1_is_on_back_home; 
		}		
	}
}


void put_off(int blow_time)
{
    while(time < blow_time)
    {
        car_stop();
        Fan_On;
			  car_stop();
        time++;
        delay_ms(1);
    }
    
    Fan_Off;
    time = 0;
}

void get_pos(void)
{
	right_dis = Ultrasonic_Get_Right();
	front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
	left_dis = Ultrasonic_Get_Left();
}

int make_sure_dis(void)
{
    if((dis_car[0] + dis_car[1])/2 <= dis_car[2] + 5 && (dis_car[0] + dis_car[1])/2 >= dis_car[2] - 5)  return 1;
    else 
    {
        dis_car[0] = 0;
        dis_car[1] = 0;
        dis_car[2] = 0;
        return 0;
    }
}

void make_car_road(void)
{
	task = rand()%10;
	switch(task)
	{
		case 0:
			fire_pos[3] = 1;
			fire_pos[2] = 0;
			fire_pos[1] = 0;
			fire_pos[0] = 0;
		break;
			
		case 2:
			fire_pos[3] = 0;
			fire_pos[2] = 1;
			fire_pos[1] = 0;
			fire_pos[0] = 0;
		break;
								
		case 3:
			fire_pos[3] = 0;
			fire_pos[2] = 0;
			fire_pos[1] = 1;
			fire_pos[0] = 0;
		break;
											
		case 4:
			fire_pos[3] = 0;
			fire_pos[2] = 0;
			fire_pos[1] = 0;
			fire_pos[0] = 1;
		break;
														
		case 5:
			fire_pos[3] = 1;
			fire_pos[2] = 1;
			fire_pos[1] = 0;
			fire_pos[0] = 0;
		break;
																	
		case 6:
			fire_pos[3] = 1;
			fire_pos[2] = 0;
			fire_pos[1] = 1;
			fire_pos[0] = 0;
		break;
																				
		case 7:
			fire_pos[3] = 1;
			fire_pos[2] = 0;
			fire_pos[1] = 0;
			fire_pos[0] = 1;
		break;
																							
		case 8:
			fire_pos[3] = 0;
			fire_pos[2] = 1;
			fire_pos[1] = 1;
			fire_pos[0] = 0;
		break;
																										
		case 9:
			fire_pos[3] = 0;
			fire_pos[2] = 1;
			fire_pos[1] = 0;
			fire_pos[0] = 1;
		break;
																													
		case 1:
			fire_pos[3] = 0;
			fire_pos[2] = 0;
			fire_pos[1] = 1;
			fire_pos[0] = 1;
		break;
		
		default:
			fire_pos[3] = 1;
			fire_pos[2] = 1;
			fire_pos[1] = 1;
			fire_pos[0] = 1;
	}
}


//0:左边
//1:右边
void cor_dis(unsigned char dir)
{
	switch(dir)
	{
		case 0:
			if(left_dis > lim_dis)
			{
				do{
					adjustDirection_1(1350,1350);
					get_pos();
				}while(left_dis > lim_dis);
			}		
		break;
	
		case 1:
			if(right_dis > lim_dis)
			{
				do{
					adjustDirection_1(1650,1650);
					get_pos();
				}while(right_dis > lim_dis);
			}
		break;
	}
}
			