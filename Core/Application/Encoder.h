#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"

extern  int16_t encoderLeft, encoderRight;
extern int sevenway_data;
extern float x_data, y_data, z_data;
extern int32_t encoder_total_l;  // 累计脉冲总数
extern int32_t encoder_total_r;  // 累计脉冲总数
extern uint8_t remote_data[9];
extern uint32_t tim6_at;
extern int left_speed,right_speed;
extern uint8_t receive_flag,seven_ff;

void Encoder_Init(void);
void Encoder_Get(void);
void turn(uint8_t *rx_data);
void StartRotation(int turns, int speed,uint8_t left_or_right);
void mpu_0p(void);
#endif

