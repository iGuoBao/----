#ifndef CAR_TYPE_H
#define CAR_TYPE_H
/**
 *  @iGuoBao
 *  @date 2025.09.30
 *  @note :
 *    根据define 编译不同的代码
 *    定义小车类型
 *    
 *    CLAW_CAR_1 : 爪车1
 *    CLAW_CAR_2 : 爪车2
 *    FORK_CAR_1 : 叉车1
 *    BASE_CAR_1 : 底盘车1
 */
#define CLAW_CAR_1 
// #define CLAW_CAR_2 
// #define FORK_CAR_1
//  #define BASE_CAR_1

// 轮子直径65 mm 因此周长约为 208 mm
// 轮子周长 mm
#define WHEEL_CIRCUMFERENCE_MM 233

// 轮子一周的脉冲数
#define WHEEL_PULSES_PER_REV 1560   


#if defined(CLAW_CAR_1)

// START 小车基础物理参数
#define WHEEL_BASE_MM 252      // 两轮间距 mm
#define AXLE_LENGTH_MM 140     // 轴长度 mm
// END 小车基础物理参数

// START 舵机参数
#define SERVO_OPEN_PWM 1250     // 爪子开 pwm
#define SERVO_CLOSE_PWM 1600    // 爪子关 pwm

// END 舵机参数

// START PID参数
// 转向PID参数
#define TURN_PID_KP 3.0f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.0f
#define TURN_PID_OUTPUT_LIMIT 600.0f
#define TURN_PID_INTEGRAL_LIMIT 100.0f
#define TURN_PID_FILTER_COEF 0.8f
#define TURN_PID_DT 0.02f
// 左轮速度环PID参数
#define LEFT_SPEED_PID_KP 1.2f
#define LEFT_SPEED_PID_KI 0.00f
#define LEFT_SPEED_PID_KD 0.00f
#define LEFT_SPEED_PID_OUTPUT_LIMIT 600.0f
#define LEFT_SPEED_PID_INTEGRAL_LIMIT 400.0f
#define LEFT_SPEED_PID_FILTER_COEF 0.8f
#define LEFT_SPEED_PID_DT 0.02f
// 右轮速度环PID参数
#define RIGHT_SPEED_PID_KP 1.2f
#define RIGHT_SPEED_PID_KI 0.00f
#define RIGHT_SPEED_PID_KD 0.00f
#define RIGHT_SPEED_PID_OUTPUT_LIMIT 600.0f
#define RIGHT_SPEED_PID_INTEGRAL_LIMIT 400.0f
#define RIGHT_SPEED_PID_FILTER_COEF 0.8f
#define RIGHT_SPEED_PID_DT 0.02f
// 七路循迹PID参数
#define SEVENWAY_PID_KP 0.5f
#define SEVENWAY_PID_KI 0.0f
#define SEVENWAY_PID_KD 0.1f
#define SEVENWAY_PID_OUTPUT_LIMIT 30.0f
#define SEVENWAY_PID_INTEGRAL_LIMIT 50.0f
#define SEVENWAY_PID_FILTER_COEF 0.95f
#define SEVENWAY_PID_DT 0.02f
// END PID参数

#elif defined(CLAW_CAR_2)
// START 小车基础物理参数
#define WHEEL_BASE_MM 252      // 两轮间距 mm
#define AXLE_LENGTH_MM 140     // 轴长度 mm
// END 小车基础物理参数

// START PID参数
// 转向PID参数
#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.1f
#define TURN_PID_KD 0.05f
#define TURN_PID_OUTPUT_LIMIT 200.0f
#define TURN_PID_INTEGRAL_LIMIT 100.0f
#define TURN_PID_FILTER_COEF 0.8f
#define TURN_PID_DT 0.02f
// 左轮速度环PID参数
#define LEFT_SPEED_PID_KP 0.8f
#define LEFT_SPEED_PID_KI 0.05f
#define LEFT_SPEED_PID_KD 0.02f
#define LEFT_SPEED_PID_OUTPUT_LIMIT 600.0f
#define LEFT_SPEED_PID_INTEGRAL_LIMIT 400.0f
#define LEFT_SPEED_PID_FILTER_COEF 0.8f
#define LEFT_SPEED_PID_DT 0.02f
// 右轮速度环PID参数
#define RIGHT_SPEED_PID_KP 0.8f
#define RIGHT_SPEED_PID_KI 0.05f
#define RIGHT_SPEED_PID_KD 0.02f
#define RIGHT_SPEED_PID_OUTPUT_LIMIT 600.0f
#define RIGHT_SPEED_PID_INTEGRAL_LIMIT 400.0f
#define RIGHT_SPEED_PID_FILTER_COEF 0.8f
#define RIGHT_SPEED_PID_DT 0.02f
// END PID参数
#elif defined(FORK_CAR_1)
#define WHEEL_BASE_MM 218       
#define AXLE_LENGTH_MM 140
#elif defined(BASE_CAR_1)
#define WHEEL_BASE_MM 250
#define AXLE_LENGTH_MM 250
#endif




#endif
