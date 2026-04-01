#ifndef __ACTION_H
#define __ACTION_H
#include "main.h"
#include "port.h"

// define 低速speed
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 80
#define LOW_SPEED_LAST_3 120
#define LOW_SPEED_LAST_MORE 150

// turn pid mpu6050_pid_reset(1.80, 0.5f, 0.2f, 50, 3000);
#define MPU6050_TURN_P              1.80f
#define MPU6050_TURN_I              0.5f
#define MPU6050_TURN_D              0.2f
#define MPU6050_TURN_MAX_OUTPUT     50
#define MPU6050_TURN_MAX_INTEGRAL   3000






void turn_left(void);
void turn_right(void);
void turn_around(void);
void forward(int data);
int count_zeros_8bit(uint8_t num);
void route(char Road[50]);
void forward_begin(void);
#endif

