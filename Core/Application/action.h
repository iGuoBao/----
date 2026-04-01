#ifndef __ACTION_H
#define __ACTION_H
#include "main.h"
#include "port.h"





void turn_left(void);
void turn_right(void);
void turn_around(void);
void forward(int data);
int count_zeros_8bit(uint8_t num);
void route(char Road[50]);
void forward_begin(void);
#endif

