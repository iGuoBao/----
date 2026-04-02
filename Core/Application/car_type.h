#ifndef CAR_TYPE_H
#define CAR_TYPE_H
/**
 *  @iGuoBao
 *  @date 2025.09.30
 *  @note :
 *    根据define 编译不同的代码
 *    定义小车类型
 *
 *      1
 *      CLAW_CAR_1_1 : 1队1号爪车
 *      CLAW_CAR_1_2 : 1队2号爪车
 *      FORK_CAR_1_3 : 1队3号叉车
 *      BASE_CAR_1_4 : 1队4号底盘
 *
 *      2
 *      CLAW_CAR_2_1 : 2队1号爪车
 *      CLAW_CAR_2_2 : 2队2号爪车
 *      FORK_CAR_2_3 : 2队3号叉车
 *      BASE_CAR_2_4 : 2队4号底盘
 *
 *      3
 *      CLAW_CAR_3_1 : 3队1号爪车
 *      CLAW_CAR_3_2 : 3队2号爪车
 *      FORK_CAR_3_3 : 3队3号叉车
 *      BASE_CAR_3_4 : 3队4号底盘
 *
 *      4
 *      CLAW_CAR_4_1 : 4队1号爪车
 *      CLAW_CAR_4_2 : 4队2号爪车
 *      FORK_CAR_4_3 : 4队3号叉车
 *      BASE_CAR_4_4 : 4队4号底盘
 */

// 队伍1
// #define CLAW_CAR_1_1
// #define CLAW_CAR_1_2
// #define FORK_CAR_1_3
// #define BASE_CAR_1_4

// 队伍2
// #define CLAW_CAR_2_1
// #define CLAW_CAR_2_2
// #define FORK_CAR_2_3
// #define BASE_CAR_2_4

// 队伍3
#define CLAW_CAR_3_1
// #define CLAW_CAR_3_2
// #define FORK_CAR_3_3
// #define BASE_CAR_3_4

// 队伍4
// #define CLAW_CAR_4_1
// #define CLAW_CAR_4_2
// #define FORK_CAR_4_3
// #define BASE_CAR_4_4

#if defined(CLAW_CAR_3_1)

// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM 1250  // 爪子开 pwm
#define SERVO_CLOSE_PWM 1600 // 爪子关 pwm
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 110
#define LOW_SPEED_LAST_3 130
#define LOW_SPEED_LAST_MORE 150
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 2.00f
#define MPU6050_TURN_I 0.5f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 50
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

#elif defined(CLAW_CAR_3_2)
// START 小车基础物理参数
#define WHEEL_BASE_MM 252          // 两轮间距 mm
#define AXLE_LENGTH_MM 140         // 轴长度 mm
#define WHEEL_CIRCUMFERENCE_MM 233 // 轮子周长 mm
#define WHEEL_PULSES_PER_REV 1560  // 轮子一周的脉冲数
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM 1250        // 爪子开 pwm
#define SERVO_CLOSE_PWM 1600       // 爪子关 pwm
// END 舵机参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 110
#define LOW_SPEED_LAST_3 130
#define LOW_SPEED_LAST_MORE 150
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 2.00f
#define MPU6050_TURN_I 0.5f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 50
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

#elif defined(FORK_CAR_3_3)
// START 小车基础物理参数
#define WHEEL_BASE_MM 218
#define AXLE_LENGTH_MM 140
// END 小车基础物理参数

// START 循迹速度设置
#define LOW_SPEED_LAST_1 80
#define LOW_SPEED_LAST_2 110
#define LOW_SPEED_LAST_3 130
#define LOW_SPEED_LAST_MORE 150
// END 循迹速度设置

// START MPU6050转向PID参数
#define MPU6050_TURN_P 2.00f
#define MPU6050_TURN_I 0.5f
#define MPU6050_TURN_D 0.2f
#define MPU6050_TURN_MAX_OUTPUT 50
#define MPU6050_TURN_MAX_INTEGRAL 3000
// END MPU6050转向PID参数

#elif defined(BASE_CAR_3_4)
#define WHEEL_BASE_MM 250
#define AXLE_LENGTH_MM 250
#endif

#endif
