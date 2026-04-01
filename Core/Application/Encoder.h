#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"

extern  int16_t encoderLeft, encoderRight;
extern int sevenway_data;
extern float x_data, y_data, z_data;
extern int32_t encoder_total_l;  // �ۼ���������
extern int32_t encoder_total_r;  // �ۼ���������
extern uint8_t remote_data[9];
extern uint32_t tim6_at;
extern int left_speed,right_speed;
extern uint8_t receive_flag,seven_ff;

extern int16_t encoder_left_speed_mm_s, encoder_right_speed_mm_s; // 存储左右轮的速度 mm/s
extern int16_t encoder_speed_mm_s;                           // 存储小车的线速度 mm/s
extern int16_t encoder_left_angular_speed_deg_s, encoder_right_angular_speed_deg_s; // 左右轮角速度 deg/s
extern int16_t encoder_angular_speed_deg_s; // 小车角速度 deg/s
extern float encoder_angular_speed_rad_s; // 小车角速度 rad/s

void Encoder_Init(void);
void Encoder_Get(void);
void turn(uint8_t *rx_data);
void StartRotation(int turns, int speed,uint8_t left_or_right);
void mpu_0p(void);
#endif

