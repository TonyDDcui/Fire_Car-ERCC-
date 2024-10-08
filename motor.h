#ifndef __MOTOR_H
#define __MPTOR_H


#include "reg52.h"

#define uchar unsigned char
#define uint unsigned int
	

extern unsigned int Left_H_Speed;
extern unsigned int Left_L_Speed;

extern unsigned int Right_H_Speed;
extern unsigned int Right_L_Speed;

extern unsigned int i;

void Timer0Init();
void Timer1Init();
void Timer0_Load_LeftMotor(unsigned int LeftspeedVal);
void Timer1_Load_RightMotor(unsigned int RightspeedVal);


#endif