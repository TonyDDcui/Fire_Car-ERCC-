#ifndef __KEY_H
#define __KEY_H

#include "reg52.h"
#include "Ultrasonic.h"

sbit LED = P1^2;

void keyscan(void);

extern unsigned char  start;
extern int fire_pos[4];
extern int user_push;
extern unsigned char rand_pushed;

#endif