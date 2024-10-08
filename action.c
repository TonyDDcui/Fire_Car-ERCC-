#include "action.h"

volatile unsigned int last_dis = 0;
volatile unsigned int left_dis = 0;
volatile unsigned int right_dis = 0;
volatile unsigned int front_dis = 0;
volatile unsigned int speed_right = 0;
volatile unsigned int speed_left = 0;

float xdata time_i = 0;
float xdata error=0x00;
float xdata error_i=0x00;
float xdata sum_err=0x00;
float xdata error_dis=0x00;
float xdata error_dir=0x00;

float xdata kp_dis=5.2; //5.2
float xdata kp_dir=5.0;

float xdata kp_dis_big=2.5; //2.3 -- 2.5 -- 2.3
float xdata ki_dis_big=2.3; //2.0 -- 2.3 -- 2.5
float xdata kp_dir_big=8.0; //7.0 -- 8.0 -- 7.0

float xdata kp_dis_mid=3.0;//3.0*1.5;2.7-2.4
float xdata kp_dir_mid=30.7;//1.5*1.5;   34-30还不错//30.7

float xdata kp_dis_samll=3.9;//1.9;//2.2有点猛;//1.8*1.5;1.5-1.2-1.0//1.9
float xdata kp_dir_small=40.0;//1.9;//2.2有点猛;//1.8*1.5;1.5-1.2-1.0//40

#define right_wall_dis 150
#define left_wall_dis 160


typedef struct Kalm_parameter
{
	float p_mid;
	float p_last;
	float p_now;
	float Q;
	float kg;
	float R;
	float Sm;
	float x_last;
	float x_mid;
}
Kalm_par;

Kalm_par kalm_front = {0.0, 0.0, 0.0, 0.018, 0.0, 0.0542, 0.0, 0.0, 0.0};
Kalm_par xdata kalm_left = {0.0, 0.0, 0.0, 0.001, 0.0, 0.012, 0.0, 0.0, 0.0};
Kalm_par xdata kalm_right = {0.0, 0.0, 0.0, 0.001, 0.0, 0.012, 0.0, 0.0, 0.0};

int kalmanfilter(int z_measure, int position)//卡尔曼滤波
{
	switch(position)
	{
		case 1:
			kalm_front.x_mid = kalm_front.x_last;
			kalm_front.p_mid = kalm_front.p_last + kalm_front.Q;
			kalm_front.kg = kalm_front.p_mid + kalm_front.Q;
			kalm_front.Sm = (int)kalm_front.x_mid + kalm_front.kg * (z_measure - kalm_front.x_mid);
			kalm_front.p_now = (1-kalm_front.kg) * kalm_front.p_mid;
			kalm_front.p_last = kalm_front.p_now;
			kalm_front.x_last = kalm_front.Sm;
			break;
		case 2:
			kalm_left.x_mid = kalm_left.x_last;
			kalm_left.p_mid = kalm_left.p_last + kalm_left.Q;
			kalm_left.kg = kalm_left.p_mid + kalm_left.Q;
			kalm_left.Sm = (int)kalm_left.x_mid + kalm_left.kg * (z_measure - kalm_left.x_mid);
			kalm_left.p_now = (1-kalm_left.kg) * kalm_left.p_mid;
			kalm_left.p_last = kalm_left.p_now;
			kalm_left.x_last = kalm_left.Sm;
			break;
		case 3:
			kalm_right.x_mid = kalm_right.x_last;
			kalm_right.p_mid = kalm_right.p_last + kalm_right.Q;
			kalm_right.kg = kalm_right.p_mid + kalm_right.Q;
			kalm_right.Sm = (int)kalm_right.x_mid + kalm_right.kg * (z_measure - kalm_right.x_mid);
			kalm_right.p_now = (1-kalm_right.kg) * kalm_right.p_mid;
			kalm_right.p_last = kalm_right.p_now;
			kalm_right.x_last = kalm_right.Sm;
			break;
		default:
			break;
	}
	if(1 == position) 
		return kalm_front.x_last;
	else if(2 == position) 
		return kalm_left.x_last;
	else if(3 == position) 
		return kalm_right.x_last;
	
	return 0;
}


void adjustDirection_1(unsigned int LeftSpeed,unsigned int RightSpeed)
{
	Right_H_Speed = (int)(RightSpeed * 0.89);
    Right_L_Speed = 18330 - Right_H_Speed;
	
	Left_H_Speed = (int)(LeftSpeed * 0.89);
	Left_L_Speed = 18330 - Left_H_Speed;
}
void Circulation_wall_right()
{
	 last_dis=right_dis;
	 front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
	 right_dis = Ultrasonic_Get_Right();
	 //left_dis = Ultrasonic_Get_Left();
	
//	 front_dis= Ultrasonic_Get_Front();
//	 right_dis= Ultrasonic_Get_Right();
//	 left_dis= Ultrasonic_Get_Left();
	
	 if(right_dis > 420)//右边不靠墙时扫描左边与墙的距离
	 {
		 speed_right=1300;
		 speed_left=1700;
		 adjustDirection_1(speed_left,speed_right);
	 }
//--------------大于170--------------10	 
	 else if(right_dis >right_wall_dis)//turn right
	 {
		 error_dis=(right_dis-right_wall_dis)*kp_dis;// kp_dis=5.0
/////////////////////////////大于220-------------------------------
//小车离中心线大于50mm，有位置环pid，有方向环pid
/////////////////////////				 
		 if(right_dis-right_wall_dis>50)
		 {
			 kp_dis=kp_dis_samll;//kp_dis_samll=1.9
			 kp_dir=kp_dir_small;//kp_dir_small=40.0
//------------------------------------------------------------------			 
			 error_dis=(right_dis-right_wall_dis)*kp_dis;//-------------10.2
			 error_dir=(last_dis-right_dis)*kp_dir;	
//			 speed_right=1300+error_dis;
			 if((last_dis-right_dis)>0)
				 {
					 //现朝右墙远离中心，要使靠近中心线
					  speed_right=1300+error_dis-error_dir;
					  if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
					 speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);
				 }
				 else
				 {
					 //现朝左远离中心线，使靠近中心线
					 speed_right=1300+error_dis-error_dir;
					 if(speed_right>=1500)
					 {
						 speed_right=1450;
					 }
					 speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);
				 }
		 }
		 else
		 {
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////			 
			 error_dis=(right_dis-right_wall_dis)*kp_dis;
			 
			 if(right_dis-right_wall_dis>30)
			 {
				   kp_dis=kp_dis_mid;//kp_dis_mid=2.0
				   kp_dir=kp_dir_mid;//kp_dir_mid=30.7
				  
				  error_dis=(right_dis-right_wall_dis)*kp_dis;
				  error_dir=(last_dis-right_dis)*kp_dir;
				 
				 if((last_dis-right_dis)>0)
				 {
					 //靠近中心线
					 if(error_dir>=0.9*error_dis)
					 {
						 error_dir=0.9*error_dis;
					 }
					  speed_right=1300+error_dis-error_dir;
					  if(speed_right>=1500)
					 {
						 speed_right=1450;
					 }
					 speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);
				 }
				 else
				 {
					 //远离中心线
					 speed_right=1300+error_dis-error_dir;
					 if(speed_right>=1500)
					 {
						 speed_right=1450;
					 }
					 speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);
				 }
			 }
////////////////////////////
//小车离中心线小于30mm，大于10mm，有位置环pid，有方向环pid
/////////////////////////	
			 else
			 {
				kp_dis=kp_dis_big;//kp_dis_big=2.3
				kp_dir=kp_dir_big;//kp_dir_big=7.0
				error_dis=(right_dis-right_wall_dis)*kp_dis;
				error_dir=(last_dis-right_dis)*kp_dir;
				error=(right_dis-right_wall_dis);
				if(15>=error&&time_i<=5)				 
				{
					if(time_i<4)
						 sum_err=sum_err+error*0.3;
					else
						 sum_err=(sum_err+error)*0.7;
					
					error_i=sum_err*ki_dis_big;//ki_dis_big=2.0
						
					time_i++;
				}
				else if (15<error||time_i>5)
				{
					sum_err=0;
					error_i=0;					  
					time_i=0;
				}
				if((last_dis-right_dis)>0)//朝右(鼓励适当右)
				{
					//右转是大环境，这里离目标线很近，需要调整车头方向为左拐，防止超调 
					speed_left=1700+error_dis-error_dir*0.3+error_i;
					if(speed_left<=1500)
					{
						 speed_left=1550;
					}
					  speed_right=1300;
					 adjustDirection_1((int)speed_left,1300);
				 }
				 else if((last_dis-right_dis)<0)//朝左 error_dir<0
				 {
					 //远离中心线
					 speed_right=1300+error_dis-error_dir+error_i;
					 if(speed_right>=1500)
					 {
						 speed_right=1450;
					 }
					  speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);					
				 }
				 else //正向
				 {
					//远离中心线
					speed_right=1300+error_dis+error_i;
					if(speed_right>=1500)
					{
						speed_right=1450;
					}
					 speed_left=1700;
					adjustDirection_1(1700,(int)speed_right);					
				 }
			 }
		 }
	 }
/////////////////////////////
//-------------小车离墙距离小于170mm
/////////////////////////	 
	 else if(right_dis <right_wall_dis)//turn left
	 {
		error_dis=(right_wall_dis-right_dis)*kp_dis;
		if(right_wall_dis-right_dis>50)
		{
/////////////////////////////
//小车离中心线大于50mm，只有位置环pid，没有方向环pid
/////////////////////////		 
			kp_dis=kp_dis_samll;
			kp_dir=kp_dir_small;
			 error_dis=(right_wall_dis-right_dis)*kp_dis;
			error_dir=(last_dis-right_dis)*kp_dir;
			if((last_dis-right_dis)>0)
			{
				//远近中心线
				kp_dis=kp_dis_big;
				kp_dir=kp_dir_big;
				speed_left=1700-error_dis-error_dir;
				if(speed_left<=1500)
				{
					speed_left=1550;
				}
				speed_right=1300;
				adjustDirection_1((int)speed_left,1300);
			}
			else
			{
				//靠近中心线
				if(-(error_dir)>=0.9*error_dis)
				{
					error_dir=-0.9*error_dis;
				}
				speed_left=1700-error_dis-error_dir;
				if(speed_left<=1500)
				{
					speed_left=1550;
				}
				speed_right=1300;
				adjustDirection_1((int)speed_left,1300);
			}
		}
		else
		{
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////				 
			if(right_wall_dis-right_dis>30)
			{
				kp_dis=kp_dis_mid;
				kp_dir=kp_dir_mid;
				error_dis=(right_wall_dis-right_dis)*kp_dis;
				error_dir=(last_dis-right_dis)*kp_dir;
				if((last_dis-right_dis)>0)
				{
					//远近中心线
					kp_dis=kp_dis_big;
					kp_dir=kp_dir_big;
					speed_left=1700-error_dis-error_dir;
					if(speed_left<=1500)
					{
						speed_left=1550;
					}
					speed_right=1300;
					adjustDirection_1((int)speed_left,1300);
				}
				else
				{
					//靠近中心线
					if(-(error_dir)>=0.9*error_dis)
					{
						error_dir=-0.9*error_dis;
					}
					speed_left=1700-error_dis-error_dir;
					if(speed_left<=1500)
					{
						speed_left=1550;
					}
					speed_right=1300;
					adjustDirection_1((int)speed_left,1300);
				}
			}
 /////////////////////////////
//小车离中心线小于30mm，大于10mm，有位置环pid，有方向环pid
/////////////////////////
			else
			{
				kp_dis=kp_dis_big;
				kp_dir=kp_dir_big;
				error_dis=(right_wall_dis-right_dis)*kp_dis;
				error_dir=(last_dis-right_dis)*kp_dir;
				error=(right_wall_dis-right_dis);				 
				if(15>=error&&time_i<=5)				 
				{
					if(time_i<3)
						 sum_err=sum_err+error*0.3;
					else
						 sum_err=(sum_err+error)*0.6;
					error_i=sum_err*ki_dis_big;						 
					time_i++;
				}
				else if (15<error||time_i>5)
				{
				sum_err=0;
				error_i=0;					  
				time_i=0;
				}
				if((last_dis-right_dis)>0)//朝右
				{
					//远近中心线
					
					kp_dis=kp_dis_big;
					kp_dir=kp_dir_big;
					 speed_left=1700-error_dis-error_dir-error_i;
					if(speed_left<=1500)
					{
						 speed_left=1550;
					}
					 speed_right=1300;
					adjustDirection_1((int)speed_left,1300);
				}
				else if((last_dis-right_dis)<0)//朝左(适当鼓励)
				{
					//靠近中心线
					//左转是大环境，这里离目标线很近，需要调整车头方向为右拐，防止超调
					speed_right=1300-error_dis-error_dir*0.3-error_i;
					if(speed_right>=1500)
					{
						speed_right=1450;
					}
					speed_left=1700;
					adjustDirection_1(1700,(int)speed_right);
				}
				else //正向
				{
					//让朝右
						 
					speed_left=1700-error_dis-error_i;
					if(speed_left<=1500)
					{
						 speed_left=1550;
					}
					 speed_right=1300;
					adjustDirection_1((int)speed_left,1300);					
				}
			}
		 }
	 }
	 else if(right_dis ==right_wall_dis)
	 {
		 adjustDirection_1(1700,1300);		 	 
	 }
}

/*
void Circulation_wall_left(void)
{	
	 last_dis=left_dis;
	 front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
	 right_dis= Ultrasonic_Get_Right();
	 left_dis= Ultrasonic_Get_Left();
//	 front_dis= Ultrasonic_Get_Front();
//	 left_dis= Ultrasonic_Get_Left();
//	 right_dis= Ultrasonic_Get_Right();
	
	
	if(left_dis > 320)//右边不靠墙时扫描左边与墙的距离
	 {		 
		 speed_right=1300;
		 speed_left=1700;
		 adjustDirection_1(speed_left,speed_right);	
	 }
	 
/////////////////////////////
//--------------------小车离墙距离小于172mm
/////////////////////////	
	 else if(left_dis <=170)//turn right
	 {
		 error_dis=(170-left_dis)*kp_dis;
/////////////////////////////
//小车离中心线大于50mm，只有位置环pid，没有方向环pid
/////////////////////////			 
		 if(170-left_dis>50)
		 {
			 kp_dis=kp_dis_samll;
			 kp_dir=kp_dir_small;
			 error_dir=0.0;
			 error_dis=(170-left_dis)*kp_dis;
			 speed_right=1300+error_dis;
			  if(speed_right>=1500)
			 {
				 speed_right=1450;
			 }
		    speed_left=1700;
			  adjustDirection_1(speed_left,(int)speed_right);
		 }
		 else
		 {
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////	
			 if(170-left_dis>30)
			 {
				 kp_dis=kp_dis_mid;
				 kp_dir=kp_dir_mid;
				 error_dis=(170-left_dis)*kp_dis;
				 error_dir=(last_dis-left_dis)*kp_dir;
				 if((last_dis-left_dis)>0)//--------------朝左
				 {
					 //远离中心线
					speed_right=1300+error_dis+error_dir;
					  if(speed_right>=1500)
					 {
						 speed_right=1450;
						  
					 }
					 speed_left=1700;
					 adjustDirection_1(speed_left,(int)speed_right);
				 }
				 else                  //--------------朝右
				 {
					 //靠近中心线
					  if(-(error_dir)>=0.9*error_dis)
					 {
						 error_dir=-0.9*error_dis;
					 }
					  speed_right=1300+error_dis+error_dir;
					  if(speed_right>=1500)
					 {
						 speed_right=1450;
						
					 }
					 speed_left=1700;
					 adjustDirection_1(speed_left,(int)speed_right);
				 }
			 }
			 else
			 {
/////////////////////////////
//小车离中心线小于30mm，有位置环pid，有方向环pid
/////////////////////////					 
				 
					  kp_dis=kp_dis_big;
					  kp_dir=kp_dir_big;
					  error_dis=(170-left_dis)*kp_dis;
					  error_dir=(last_dis-left_dis)*kp_dir;
					 if((last_dis-left_dis)>0)  //--------------朝左
					 {
						 //远离中心线
						  speed_right=1300+error_dis+error_dir;
						  if(speed_right>=1500)
						  {
							 speed_right=1450;
						  }
						  speed_left=1700;
						  adjustDirection_1(speed_left,(int)speed_right);
					 }
					 else if((last_dis-left_dis)<0)      //--------------朝右适当鼓励dir-
					 {
						 //靠近中心线
						 //转中的转
						  //右转是大环境，这里离目标线很近，需要调整车头左拐，防止超调
						speed_left=1700+error_dis+error_dir*0.5;  //---适当鼓励朝右
						if(speed_left<=1500)
							
						{
							speed_left=1550;
						}
						speed_right=1300;
						adjustDirection_1((int)speed_left,speed_right);
					 }
				    else //正向
					 {
						 //远离中心线
						 	 
						 speed_right=1300+error_dis;
						 if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
						 speed_left=1700;
						 adjustDirection_1(speed_left,(int)speed_right);					
					 }
			 }
		 }
	 }
/////////////////////////////
//小车离墙距离大于172mm
/////////////////////////	 
	 else//turn left
	 {
		 error_dis=(left_dis-170)*kp_dis;
/////////////////////////////
//小车离中心线大于50mm，只有位置环pid，没有方向环pid
/////////////////////////		 
		 if(left_dis-170>50)
		 {
			 kp_dis=kp_dis_samll;
			 kp_dir=kp_dir_small;
			 error_dis=(left_dis-170)*kp_dis;
			 error_dir=0.0;
			 speed_left=1700-error_dis;
			 if(speed_left<=1500)
			 {
				 speed_left=1550;
			 }
			  speed_right=1300;
			 adjustDirection_1((int)speed_left,speed_right);
		 }
		 else
		 {
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////			
			 if(left_dis-170>30)
			 {
				   kp_dis=kp_dis_mid;
				   kp_dir=kp_dir_mid;
				 error_dis=(left_dis-170)*kp_dis;
				 error_dir=(last_dis-left_dis)*kp_dir;
				 
				 if((last_dis-left_dis)>0)//---------朝左
				 {
					 //靠近中心线
					  if(error_dir>=0.9*error_dis)
					 {
						 error_dir=0.9*error_dis;
					 }
					 speed_left=1700-error_dis+error_dir;
					  if(speed_left<=1500)
					 {
						 speed_left=1550;
					 }
					  speed_right=1300;
					 adjustDirection_1((int)speed_left,speed_right);
				 }
				 else       //---------朝右
				 {
					 //远近中心线
					speed_left=1700-error_dis+error_dir*0.8;
					  if(speed_left<=1500)
					 {
						 speed_left=1550;
					 }
					  speed_right=1300;
					 adjustDirection_1((int)speed_left,speed_right);
				 }
			 }
			 else
			 {
				 kp_dis=kp_dis_big;
				  kp_dir=kp_dir_big;
				 error_dis=(left_dis-170)*kp_dis;
/////////////////////////////
//小车离中心线小于30mm，--10mm，有位置环pid，有方向环pid
/////////////////////////
				
					error_dir=(last_dis-left_dis)*kp_dir;
					 if((last_dis-left_dis)>0)//---------朝左适当鼓励
					 {
						 //靠近中心线
						 //左转是大环境，这里离目标线很近，需要调整车头右拐，防止超调
						 speed_right=1300-error_dis+error_dir*0.6;
						  if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
						  speed_left=1700;
						 adjustDirection_1(speed_left,(int)speed_right);
					 }
				else if((last_dis-left_dis)<0) //---------朝右
					 {
						 //远近中心线
						speed_left=1700-error_dis+error_dir*0.9;
						  if(speed_left<=1500)
						 {
							 speed_left=1550;
						 }
						  speed_right=1300;
						 adjustDirection_1((int)speed_left,speed_right);
					 }
				
				 else //正向
					 {
						 //远离中心线
						 	 
						 speed_left=1700-error_dis;
						 if(speed_left<=1500)
						 {
							 speed_left=1550;
						 }
						  speed_right=1300;
						 adjustDirection_1((int)speed_left,speed_right);				
					 }
			 }	
		 }
	 }
}
*/

void car_stop()
{
	Left_H_Speed = 1335;
    Left_L_Speed = 18330 - 1335;
    Right_H_Speed = 1335;
    Right_L_Speed = 18330 - 1335;
}
void forward()
{
	adjustDirection_1(1700,1300);
}
void turn_left_90()
{
	adjustDirection_1(1350,1350);
	delay_ms(440);//延时500ms即可 ——原数据360改为540——5.18，400-5.19，420-5.20
}
void turn_left_90_small()
{
	adjustDirection_1(1350,1350);
	delay_ms(400);//延时500ms即可 ——原数据400改为600——5.18，500-5.19，480-5.20
}
void turn_left_90_more_small()
{
	adjustDirection_1(1400,1400);
	delay_ms(360);//延时500ms即可 ——原数据400改为600——5.18，500-5.19，480-5.20
}

void turn_right_90_more_small()
{
	adjustDirection_1(1600,1600);
	delay_ms(320);
}

void turn_left_90_big()
{
	adjustDirection_1(1350,1350);
	delay_ms(500);//延时500ms即可 ——原数据400改为600——5.18，500-5.19，480-5.20
}
void turn_right_90()
{
	adjustDirection_1(1650,1650);
	delay_ms(420);//延时500ms即可 ——原数据360改为540——5.18，400-5.19，420-5.20
}

void turn_right_90_small()
{
	adjustDirection_1(1650,1650);
	delay_ms(400);//延时500ms即可 ——原数据360改为540——5.18，400-5.19，420-5.20
}
void turn_right_90_big()
{
	adjustDirection_1(1650,1650);
	delay_ms(450);//延时500ms即可 ——原数据400改为600——5.18，500-5.19，480-5.20
}

void turn_right_back_90()//回家转90；
{
	adjustDirection_1(1650,1650);
	delay_ms(480);
}

void turn_right_180()
{
	adjustDirection_1(1650,1650); //之前是1650 和 1650   
	delay_ms(800);//延时500ms即可 ——原数据760，改为延迟1080-5.17，1060-5.18，840-5.20，880-5.26      840不行   500不行    300不行   延时1000角度开始接近    1250不行  980 1050不行  860  880  950   1020  1065  1085 1150
}//1130  1110  1120  1140  1010   1200 
//890   920  950   870  840   760  740  720  710有用  730稍微小点   740改赛道稍小  750还是同样问题  765


void Circulation_wall_left(void)
{	
	 last_dis=left_dis;
	 front_dis= kalmanfilter(Ultrasonic_Get_Front(),1);
	 //right_dis = Ultrasonic_Get_Right();
	 left_dis = Ultrasonic_Get_Left();
	
	
	if(left_dis > 420)//右边不靠墙时扫描左边与墙的距离
	 {
		 left_dis = Ultrasonic_Get_Left();
//		  forwardOneStep();
//		  forwardOneStep();
//		  forwardOneStep();
//		  forwardOneStep();
//		  forwardOneStep(); 
		 
		 
		 
		 speed_right=1300;
		 speed_left=1700;
		 adjustDirection_1(speed_left,speed_right);
//		 sprintf(buf1,"直行");
//		
//		speak_len1=strlen(( const char *)buf1);
//		speak_context((u8*)buf1,speak_len1);	
	 }
/////////////////////////////
//--------------------小车离墙距离小于172mm
/////////////////////////	
	  else if(left_dis <=left_wall_dis)//turn right
	 {
		 error_dis=(left_wall_dis-left_dis)*kp_dis;
/////////////////////////////
//小车离中心线大于50mm，只有位置环pid，没有方向环pid
/////////////////////////			 
		 if(left_wall_dis-left_dis>50)
		 {
			 kp_dis=kp_dis_samll;
			 kp_dir=kp_dir_small;
			 error_dir=0.0;
			 error_dis=(left_wall_dis-left_dis)*kp_dis;
			 speed_right=1300+error_dis;
			  if(speed_right>=1500)
				 {
					 speed_right=1450;
					
				 }
		    speed_left=1700;
			adjustDirection_1(speed_left,(int)speed_right);
//			sprintf(buf1,"右转");
//			speak_len1=strlen(( const char *)buf1);
//			speak_context((u8*)buf1,speak_len1);	
			 
		 }
		 else
		 {
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////	
			 if(left_wall_dis-left_dis>30)
			 {
				 kp_dis=kp_dis_mid;
				 kp_dir=kp_dir_mid;
				  error_dis=(left_wall_dis-left_dis)*kp_dis;
				 error_dir=(last_dis-left_dis)*kp_dir;
				 if((last_dis-left_dis)>0)//--------------朝左
				 {
					 //远离中心线
					speed_right=1300+error_dis+error_dir;
					  if(speed_right>=1500)
					 {
						 speed_right=1450;
						  
					 }
					 speed_left=1700;
					 adjustDirection_1(1700,(int)speed_right);
				 }
				 else                  //--------------朝右
				 {
					 //靠近中心线
					  if(-(error_dir)>=0.9*error_dis)
					 {
						 error_dir=-0.9*error_dis;
					 }

					  speed_right=1300+error_dis+error_dir;
					  if(speed_right>=1500)
					 {
						 speed_right=1450;
						
					 }
					   speed_left=1700;
					 adjustDirection_1(speed_left,(int)speed_right);
				 }
//				 sprintf(buf1,"右转");
//				speak_len1=strlen(( const char *)buf1);
//				speak_context((u8*)buf1,speak_len1);	
			 }
			 else
			 {
/////////////////////////////
//小车离中心线小于30mm，有位置环pid，有方向环pid
/////////////////////////					 
				 
					  kp_dis=kp_dis_big;
					  kp_dir=kp_dir_big;
					  error_dis=(left_wall_dis-left_dis)*kp_dis;
					  error_dir=(last_dis-left_dis)*kp_dir;
					 if((last_dis-left_dis)>0)  //--------------朝左
					 {
						 //远离中心线
						speed_right=1300+error_dis+error_dir;
						  if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
						  speed_left=1700;
						 adjustDirection_1(1700,(int)speed_right);
//						  sprintf(buf1,"右转");
//						speak_len1=strlen(( const char *)buf1);
//						speak_context((u8*)buf1,speak_len1);	
					 }
					 else if((last_dis-left_dis)<0)      //--------------朝右适当鼓励dir-
					 {
						 //靠近中心线
	//					  if(-(error_dir)>=0.9*error_dis)
	//					 {
	//						 error_dir=-0.9*error_dis;
	//					 }
	//					  speed_right=1300+error_dis+error_dir;
	//					  if(speed_right>=1500)
	//					 {
	//						 speed_right=1450;
	//					 }
	//					 adjustDirection_1(1700,(int)speed_right);
						 //转中的转
						  //右转是大环境，这里离目标线很近，需要调整车头左拐，防止超调
						 
						speed_left=1700+error_dis+error_dir*0.6;  //---适当鼓励朝右
						  if(speed_left<=1500)
						 {
							 speed_left=1550;
						 }
						 speed_right=1300;
						 adjustDirection_1((int)speed_left,1300);
					 
						 
					 }
				 else //正向
					 {
						 //远离中心线
						 	 
						 speed_right=1300+error_dis;
						 if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
						  speed_left=1700;
						 adjustDirection_1(1700,(int)speed_right);					
					 }
				 
				
			 }
			
				 
		 }
		 
//		 sprintf(buf1,"F=%d,L=%d,s_l=1700,s_r=%d\n",front_dis,left_dis,(int)speed_right);
//		speak_len1=strlen(( const char *)buf1);
//		for( s_i = 0 ; s_i < speak_len1; s_i++) 
//		{
//	//		while((USART3->SR&0X40)==0);  
//	//		USART3->DR = DataScope_OutPut_Buffer2[s_i]; 
//			while((USART1->SR&0X40)==0);  
//			USART1->DR = buf1[s_i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
//		}

			
	 }
/////////////////////////////
//小车离墙距离大于172mm
/////////////////////////	 
	 else//turn left
	 {
		 error_dis=(left_dis-left_wall_dis)*kp_dis;
/////////////////////////////
//小车离中心线大于50mm，只有位置环pid，没有方向环pid
/////////////////////////		 
		 if(left_dis-left_wall_dis>50)
		 {
			 kp_dis=kp_dis_samll;
			 kp_dir=kp_dir_small;
			 error_dis=(left_dis-left_wall_dis)*kp_dis;
			 error_dir=0.0;
			 speed_left=1700-error_dis;
			 if(speed_left<=1500)
			 {
				 speed_left=1550;
			 }
			  speed_right=1300;
			 adjustDirection_1((int)speed_left,1300);
			 
//			 sprintf(buf1,"左转");
//			speak_len1=strlen(( const char *)buf1);
//			speak_context((u8*)buf1,speak_len1);
			 
		 }
		 else
		 {
/////////////////////////////
//小车离中心线小于50mm，大于30mm，有位置环pid，有方向环pid
/////////////////////////			
			 if(left_dis-left_wall_dis>30)
			 {
				   kp_dis=kp_dis_mid;
				   kp_dir=kp_dir_mid;
				 error_dis=(left_dis-left_wall_dis)*kp_dis;
				 error_dir=(last_dis-left_dis)*kp_dir;
				 
				 if((last_dis-left_dis)>0)//---------朝左
				 {
					 //靠近中心线
					  if(error_dir>=0.9*error_dis)
					 {
						 error_dir=0.9*error_dis;
					 }
					 speed_left=1700-error_dis+error_dir;
					  if(speed_left<=1500)
					 {
						 speed_left=1550;
					 }
					  speed_right=1300;
					 adjustDirection_1((int)speed_left,1300);

					  
				 }
				 else       //---------朝右
				 {
					 //远近中心线
					speed_left=1700-error_dis+error_dir*0.8;
					  if(speed_left<=1500)
					 {
						 speed_left=1550;
					 }
					  speed_right=1300;
					 adjustDirection_1((int)speed_left,1300);
				 }
//				 sprintf(buf1,"左转");
//				speak_len1=strlen(( const char *)buf1);
//				speak_context((u8*)buf1,speak_len1);
			 }
			 else
			 {
	
				 
				 kp_dis=kp_dis_big;
				  kp_dir=kp_dir_big;
				 error_dis=(left_dis-left_wall_dis)*kp_dis;
/////////////////////////////
//小车离中心线小于30mm，--10mm，有位置环pid，有方向环pid
/////////////////////////
				
					error_dir=(last_dis-left_dis)*kp_dir;
					 if((last_dis-left_dis)>0)//---------朝左适当鼓励
					 {
						 //靠近中心线
						 
	
						 //左转是大环境，这里离目标线很近，需要调整车头右拐，防止超调
						 speed_right=1300-error_dis+error_dir*0.6;
						  if(speed_right>=1500)
						 {
							 speed_right=1450;
						 }
						  speed_left=1700;
						 adjustDirection_1(1700,(int)speed_right);
						 
//						 sprintf(buf1,"右转");
//						speak_len1=strlen(( const char *)buf1);
//						speak_context((u8*)buf1,speak_len1);

						  
					 }
				else if((last_dis-left_dis)<0) //---------朝右
					 {
						 //远近中心线
						speed_left=1700-error_dis+error_dir*0.9;
						  if(speed_left<=1500)
						 {
							 speed_left=1550;
						 }
						  speed_right=1300;
						 adjustDirection_1((int)speed_left,1300);
//						 sprintf(buf1,"左转");
//						speak_len1=strlen(( const char *)buf1);
//						speak_context((u8*)buf1,speak_len1);
					 }
				
				 else //正向
					 {
						 //远离中心线
						 	 
						 speed_left=1700-error_dis;
						 if(speed_left<=1500)
						 {
							 speed_left=1550;
						 }
						  speed_right=1300;
						 adjustDirection_1((int)speed_left,1300);				
					 }
			 }	 
		 }
	 }
}

