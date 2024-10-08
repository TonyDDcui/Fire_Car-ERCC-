#include "key.h"
#include "Ultrasonic.h"
#include "stdlib.h"


sbit Key1 = P1^0;
sbit Key2 = P0^7;
sbit Key3 = P0^6;
sbit Key4 = P0^5;
sbit Key5 = P0^4;
sbit Key6 = P0^3;


volatile unsigned char start = 0;
int fire_pos[4]={0,0,0,0};
int user_push = 0;
unsigned char rand_pushed = 0;


void keyscan(void)
{
	if(Key1==0)
	{
		delay_ms(5);
		if(Key1==0)
		{
			start = 1;
		}
		while(Key1==0);
	}
	
	if(Key2==0)
	{
		delay_ms(50);
		while(Key2==0);
		delay_ms(50);
		if(fire_pos[0] == 1) fire_pos[0] = 0;
		else fire_pos[0] = 1;
		user_push = 1;
	}
	
		if(Key3==0)
	{
		delay_ms(50);
		while(Key3==0);
		delay_ms(50);
		if(fire_pos[1] == 1) fire_pos[1] = 0;
		else fire_pos[1] = 1;
		user_push = 1;
	}
	
		if(Key4==0)
	{
		delay_ms(50);
		while(Key4==0);
		delay_ms(50);
		if(fire_pos[2] == 1) fire_pos[2] = 0;
		else fire_pos[2] = 1;
		user_push = 1;
	}
	
	if(Key5==0)
	{
		delay_ms(50);
		while(Key5==0);
		delay_ms(50);
		if(fire_pos[3] == 1) fire_pos[3] = 0;
		else fire_pos[3] = 1;
		user_push = 1;
	}
	
	if(Key6==0)
	{
		delay_ms(50);
		while(Key6==0);
		delay_ms(50);
		if(rand_pushed == 1) rand_pushed = 0;
		else rand_pushed = 1;
	}
}

