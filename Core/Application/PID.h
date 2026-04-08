#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "port.h"

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float prev_input;       
    float derivative_filtered; 
    float output_limit;     
    float integral_limit;   
    float filter_coef;     
} PIDController;
extern int left_motor, right_motor;
extern float target_angle;


float PID_Update(PIDController* pid, float setpoint, float input);
void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, 
             float output_lim, float integ_lim, float filter_coef);
void motor_pid_init(void);
void motor_pid_control(int set_left,int set_right);
void mpu6050_turn_angle(float target,uint8_t way);
void motor_speed_set(int set_left, int set_right);
void mpu6050_turn_angle_oneway(float target,int Direction);
void seven_line_pid_init(void);
void seven_line_pid_control(int speed);
void length_pid_init(void);
void length_control(int left,int right);
void mpu6050_pid_control(int speed,float target);
void target_angle_set(float data);
void mpu6050_sevenway_init(void);
void mpu6050_sevenway_control(int speed);
float Normalization(float data);
void OLED_ShowDynamicData(int16_t target, int16_t deviation) ;
float Normalization(float data);


void mpu6050_pid_reset(float kp,float ki,float kd,float output,float inte);
#endif

