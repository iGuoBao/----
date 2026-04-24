#ifndef CAR_TYPE_H
#define CAR_TYPE_H
// 队伍3
// #define CLAW_CAR_3_1     // 没有海绵的爪子
#define CLAW_CAR_3_2   // 有海绵的爪子
// #define LOADER_CAR_3_3
// #define BASE_CAR_3_4

// 队伍4
// #define CLAW_CAR_4_1
// #define CLAW_CAR_4_2
// #define LOADER_CAR_4_3
// #define BASE_CAR_4_4

#if defined(CLAW_CAR_3_1)

//  #define NEW_SERVO_0 // 升降
 #define NEW_SERVO_1 // 爪子

// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM      1150  // 爪子开 pwm
#define SERVO_CLOSE_PWM     1550 // 爪子关 pwm

#define SERVO_LIFT_PWM      2100  // 升降升 pwm
#define SERVO_RETRACT_PWM   900  // 升降降 pwm
#define SERVO_LIFT_PWM_t    1800
#define SERVO_RETRACT_PWM_d 1200
#define SERVO_STOP_PWM      1500  // 升降停 pwm
#define SERVO_dt_TIME    0 // d 正常的升降 时间
#define SERVO_DT_TIME    0 // D 正常的升降 时间
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 85
#define LOW_SPEED_LAST_2 95
#define LOW_SPEED_LAST_3 100
#define LOW_SPEED_LAST_MORE 110
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 1.90f
#define MPU6050_TURN_I 0.3f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 130
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

// START 后退参数
#define BACKWARD_SPEED -44
#define BACKWARD_TIME_20ms 85
// END 后退参数

#elif defined(CLAW_CAR_3_2)

//  #define NEW_SERVO_0 // 升降
 #define NEW_SERVO_1 // 爪子

// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM      1040  // 爪子开 pwm
#define SERVO_CLOSE_PWM     1830 // 爪子关 pwm

#define SERVO_LIFT_PWM      2100  // 升降升 pwm
#define SERVO_RETRACT_PWM   900  // 升降降 pwm
#define SERVO_LIFT_PWM_t    1800
#define SERVO_RETRACT_PWM_d 1200
#define SERVO_STOP_PWM      1500  // 升降停 pwm

#define SERVO_dt_TIME    1 // d 正常的升降 时间
#define SERVO_DT_TIME    2 // D 正常的升降 时间
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 85
#define LOW_SPEED_LAST_2 90
#define LOW_SPEED_LAST_3 100
#define LOW_SPEED_LAST_MORE 110
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 1.90f
#define MPU6050_TURN_I 0.3f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 80
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

// START 后退参数
#define BACKWARD_SPEED -44
#define BACKWARD_TIME_20ms 79
// END 后退参数

#elif defined(LOADER_CAR_3_3)
//  #define NEW_SERVO_0 // 升降
 #define NEW_SERVO_1 // 爪子

// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM      1050  // 爪子开 pwm
#define SERVO_CLOSE_PWM     1700 // 爪子关 pwm

#define SERVO_LIFT_PWM      2349  // 升降升 pwm
#define SERVO_RETRACT_PWM   914  // 升降降 pwm
#define SERVO_STOP_PWM      1500  // 升降停 pwm
#define SERVO_dt_TIME    1 // d 正常的升降 时间
#define SERVO_DT_TIME    2 // D 正常的升降 时间
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 100
#define LOW_SPEED_LAST_3 140
#define LOW_SPEED_LAST_MORE 160
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 2.20f
#define MPU6050_TURN_I 0.3f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 85
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

// START 后退参数
#define BACKWARD_SPEED -44
#define BACKWARD_TIME_20ms 83
// END 后退参数

#elif defined(BASE_CAR_3_4)
#define WHEEL_BASE_MM 250
#define AXLE_LENGTH_MM 250
//  #define NEW_SERVO_0 // 升降
 #define NEW_SERVO_1 // 爪子

// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM      1050  // 爪子开 pwm
#define SERVO_CLOSE_PWM     1700 // 爪子关 pwm

#define SERVO_LIFT_PWM      2349  // 升降升 pwm
#define SERVO_RETRACT_PWM   914  // 升降降 pwm
#define SERVO_STOP_PWM      1500  // 升降停 pwm
#define SERVO_dt_TIME    1 // d 正常的升降 时间
#define SERVO_DT_TIME    2 // D 正常的升降 时间
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 90
#define LOW_SPEED_LAST_3 100
#define LOW_SPEED_LAST_MORE 110
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 1.90f
#define MPU6050_TURN_I 0.3f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 85
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

// START 后退参数
#define BACKWARD_SPEED -44
#define BACKWARD_TIME_20ms 83
// END 后退参数

#endif
#endif
