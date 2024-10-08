#include "reg52.h"
#include "Ultrasonic.h"

//unsigned char buffer1[10];
//unsigned char buffer2[10];
//unsigned char buffer3[10];

void Ultrasonic_Init(void) //超声波定时器2初始化
{

	T2CON = 0;			
	T2MOD = 0;	
	TH2 = 0;
	TL2 = 0;	
	TR2 = 0;		
  TF2 = 0;
}
unsigned int Ultrasonic_Get_Front()//得到一次右超声波测距模块的距离
{
	unsigned int time = 0;
	unsigned int distance = 0;
	
	TH2 = 0;
	TL2 = 0;	
	
	Trig_Front = 1; //发送trig信号
	somenop;somenop;
	Trig_Front = 0; //发送trig信号完毕
	while(Echo_Front==0);
	TR2=1;//开启计数
	while(TF2==0&&Echo_Front==1);//当Echo为1计数并等待
	TR2=0;//关闭计数
	if(Echo_Front==0)
	{
		Echo_Front = 1;
		time=TH2;
		time=time<<8;
		time=time|TL2;
		
		distance=(unsigned int)(time*0.17);
		
		//distance= distance * 10;
		if(distance >= 990)
		{
			distance = 990;
		}
	}
	
	else if(TF2==1)
	{
		TF2 = 0;
		
		distance = 990;
	}
	
	return distance;
}
unsigned int Ultrasonic_Get_Left()//得到一次左超声波测距模块的距离
{
	unsigned int time = 0;
	unsigned int distance = 0;
	
	TF2 = 0;
	TH2 = 0;
	TL2 = 0;	
	
	Trig_Left = 1; //发送trig信号
	somenop;somenop;
	Trig_Left = 0; //发送trig信号完毕
	while(Echo_Left==0);
	TR2=1;//开启计数
	while(TF2==0&&Echo_Left==1);//当Echo为1计数并等待
	TR2=0;//关闭计数
	if(Echo_Left==0)
	{
		Echo_Left = 1;
		time=TH2;
		time=time<<8;
		time=time|TL2;
		
		distance=(unsigned int)(time*0.17);
		//distance= distance * 10;
		if(distance >= 990)
		{
			distance = 990;
		}
	}
	
	else if(TF2==1)
	{
		TF2 = 0;
		distance = 990;
	}
	
	return distance;
}
unsigned int Ultrasonic_Get_Right()//得到一次右超声波测距模块的距离
{
	unsigned int time = 0;
	unsigned int distance = 0;
	
	TH2 = 0;
	TL2 = 0;	
	
	Trig_Right = 1; //发送trig信号
	somenop;somenop;
	Trig_Right = 0; //发送trig信号完毕
	while(Echo_Right==0);
	TR2=1;//开启计数
	while(TF2==0&&Echo_Right==1);//当Echo为1计数并等待
	TR2=0;//关闭计数
	if(Echo_Right==0)
	{
		Echo_Right = 1;
		time=TH2;
		time=time<<8;
		time=time|TL2;
		
		distance=(unsigned int)(time*0.17);
		//distance= distance * 10;
		if(distance >= 990)
		{
			distance = 990;
		}
	}
	
	else if(TF2==1)
	{
		TF2 = 0;
		distance = 990;
	}
	
	return distance;
}
void delay_ms(unsigned int z)    
{
	 unsigned int i,j;
	 for(j = z;j > 0; j--)
	 for(i = 112;i > 0; i--);
}
