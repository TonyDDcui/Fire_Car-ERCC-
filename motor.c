#include "motor.h"

sbit Lefe_Motor = P1^1;
sbit Right_Motor = P1^3;


/*1代表需要装载高电平初值 2代表需要装载低电平初值*/
unsigned char Load_Left_flag = 1; 
unsigned char Load_Right_flag = 1;

unsigned char flag_100ms = 5;

volatile unsigned int i = 0;

unsigned int Left_H_Speed = 1335;
unsigned int Left_L_Speed = 18330 - 1335;

unsigned int Right_H_Speed = 1335;
unsigned int Right_L_Speed = 18330 - 1335;


void Timer1_Load_RightMotor(unsigned int RightspeedVal)
{
	RightspeedVal = 65536 - RightspeedVal;
	
	TH1 = RightspeedVal >> 8;	
	TL1 = RightspeedVal & 0xff;	

	TR1 = 1;		//启动定时器
}


void Timer0_Load_LeftMotor(unsigned int LeftspeedVal)
{
	LeftspeedVal = 65536 - LeftspeedVal;
	
	TH0 = LeftspeedVal >> 8;	
	TL0 = LeftspeedVal & 0xff;	

	TR0 = 1;		//启动定时器
}



void Timer0() interrupt 1
{
	TR0 = 0;//定时器0停止计时
		
	switch(Load_Left_flag)
	{
		case(1):
		{
			Lefe_Motor = 1;
			Load_Left_flag = 2;
			Timer0_Load_LeftMotor(Left_H_Speed);
			i++;
			break;
		}
		case(2):
		{
			Lefe_Motor = 0;
			Load_Left_flag = 1;
			Timer0_Load_LeftMotor(Left_L_Speed);
			break;
		}
		default:break;
	}
}

void Timer1() interrupt 3
{
	TR1 = 0;//定时器1停止计时
	
	switch(Load_Right_flag)
	{
		case(1):
		{
			Right_Motor = 1;
			Load_Right_flag = 2;
			Timer1_Load_RightMotor(Right_H_Speed);
			break;
		}
		case(2):
		{
			Right_Motor = 0;
			Load_Right_flag = 1;
			Timer1_Load_RightMotor(Right_L_Speed);
			break;
		}
		default:break;
	}
}

void Timer0Init()		//定时器0初始化 10us
{
	TMOD |= 0x01;	//设置定时器0模式
	TH0 = 0XFF;		//设置定时初值
	TL0=  0XF6;		//设置定时初值
	TF0 = 0;		//清除TF1标志
	TR0 = 1;		//定时器1开始计时
	ET0 = 1;        //打开定时器0中断
	EA  = 1;        //打开总中断
}

void Timer1Init()      //定时器1初始化 20us
{
	TMOD |= 0x10;	//设置定时器1模式
	TH1 = 0XFF;        //设置定时初值
	TL1 = 0XEC;        //设置定时初值
	TF1 = 0;
	TR1 = 1;
	ET1 = 1;
	EA  = 1;
}

