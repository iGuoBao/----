#ifndef __ACTION_H
#define __ACTION_H
#include "main.h"
#include "port.h"

void turn_left(void);
void turn_right(void);
void turn_around(void);
void forward(int data);
void forward_delay(int delay_20ms, int speed);
int count_zeros_8bit(uint8_t num);
void route(char Road[50]);
void forward_begin(void);
void Action_EnableMotionGuard(uint8_t enable);
void Action_ResetMotionFault(void);
uint8_t Action_HasMotionFault(void);
#endif
