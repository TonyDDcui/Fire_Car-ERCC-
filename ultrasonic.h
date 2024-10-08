#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "reg52.h"
#include "stdio.h"
#include "intrins.h"
#define somenop {_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}

sbit Echo_Left = P2^7;//接受端
sbit Trig_Left = P2^6;//控制端
sbit Echo_Right = P2^5;//接收端
sbit Trig_Right = P2^4;//控制端
sbit Echo_Front = P2^3;//接收端
sbit Trig_Front = P2^2;//控制端

extern unsigned char buffer1[10];
extern unsigned char buffer2[10];
extern unsigned char buffer3[10];

void delay_ms(unsigned int z);
void Ultrasonic_Init(void);
unsigned int Ultrasonic_Get_Front();
unsigned int Ultrasonic_Get_Left();
unsigned int Ultrasonic_Get_Right();
#endif
