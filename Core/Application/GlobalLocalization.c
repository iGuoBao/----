#include "GlobalLocalization.h"
#include "imu901.h"
#include "usart.h"
#include "Encoder.h"
#include <string.h>
#include <math.h>

// 串口句柄声明
extern UART_HandleTypeDef huart1;


// 模块静态状态
static GlobalPose_t s_pose;

// Yaw 更新阈值（20ms周期）：变化小于此值忽略
#define YAW_UPDATE_THRESHOLD_DEG  0.00f
// 上一次采样的 MPU yaw（度）
static float s_last_mpu_yaw = 0.0f;

// 十字路口检测与对齐
#define CROSSROAD_BITMAP_ALL_WHITE 0x00
#define CROSSROAD_BITMAP_MIDDLE5_MASK 0x3E  // bit1~bit5
#define CROSSROAD_ALIGN_GRID_MM 400
#define CROSSROAD_YAW_TOLERANCE_DEG 10.0f
static bool s_crossroad_active = false;
static int32_t s_crossroad_x_mm = 0;
static int32_t s_crossroad_y_mm = 0;

// 辅助：把角度裁剪到 [-180, 180)
static float normalize_yaw(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void GlobalLoc_Init()
{
    float  _pitch, _roll, _yaw;
    float _yaw_sum = 0.0f;

    s_pose.x_mm = GLOBAL_INIT_X_MM;
    s_pose.y_mm = GLOBAL_INIT_Y_MM;
    s_pose.x_grid = GLOBAL_INIT_X_GRID;
    s_pose.y_grid = GLOBAL_INIT_Y_GRID;
    s_pose.abs_yaw = GLOBAL_INIT_YAW_DEG;
    s_pose.yaw = 0;

    // 初始化 yaw 过滤器初值
    s_last_mpu_yaw = s_pose.yaw;
}

/**
 * @iGuoBao
 * @attention 没什么用，保留接口
 */
void GlobalLoc_ResetPose(int32_t x_mm, int32_t y_mm, float yaw_deg)
{
    s_pose.x_mm = x_mm;
    s_pose.y_mm = y_mm;
    s_pose.x_grid = x_mm / GLOBAL_GRID_SIZE_MM;
    s_pose.y_grid = y_mm / GLOBAL_GRID_SIZE_MM;
    s_pose.yaw = yaw_deg;
}

/**
 * @iGuoBao
 * @date 2025.10.12
 * @note :
 *  20ms 调用一次。
 * @note :
 *  1. 使用 mpu_dmp_get_data 获取当前绝对航向，转化为相对航向
 *  2. 使用 Encoder_Speed_mm_s 获取当前线速度（全局变量 encoder_speed_mm_s）
 *  3. 计算当前位移并更新位姿 包括 xy和相对yaw
 *  4. 向前x+ 为正，向左y+为正，yaw逆时针为正
 * @date 2025.10.23
 * @note : 增加航向角低通滤波，避免拐弯后角度突变
 */
void GlobalLoc_Periodic(void)
{
    float _yaw, _pitch, _roll;

    // START 计算小车角度yaw
    // mpu_dmp_get_data(&_pitch, &_roll, &_yaw);
    _pitch = attitude.pitch;
    _roll  = attitude.roll;
    _yaw   = normalize_yaw(attitude.yaw + s_pose.abs_yaw); // 转为相对航向
    // _yaw = attitude.yaw;

    // END 计算小车角度yaw
    
    // START 计算线速度和线速度相关数据
    // Encoder_Speed_mm_s(); 
    Encoder_Get(); // 更新 encoder_speed_mm_s, encoder_left_speed_mm_s, encoder_right_speed_mm_s, encoder_angular_speed_deg_s, encoder_angular_speed_rad_s
    s_pose.linear_velocity_mm_s = encoder_speed_mm_s;
    s_pose.left_wheel_velocity_mm_s = encoder_left_speed_mm_s;
    s_pose.right_wheel_velocity_mm_s = encoder_right_speed_mm_s;
    s_pose.angular_velocity_deg_s = encoder_angular_speed_deg_s;
    s_pose.angular_velocity_rad_s = encoder_angular_speed_rad_s;
    // END   计算线速度和线速度相关数据

    // START 处理七路传感器数据
    s_pose.seven_data = SevenWay_Read();    // 前7位有效数据位置 如果白线则对应位置0 否则为1
    s_pose.seven_data = seven_ff; // 直接使用全局变量seven_ff，避免调用函数可能带来的延迟和不一致问题
    // END   处理七路传感器数据

    // START 更新s_pose
    // float current_mpu_yaw = normalize_yaw(_yaw);
    // float yaw_diff = normalize_yaw(current_mpu_yaw - s_last_mpu_yaw);

    // if (fabsf(yaw_diff) >= YAW_UPDATE_THRESHOLD_DEG) {
    //     // 只有当变化足够大时才更新 yaw，抛弃噪声抖动
    //     s_pose.yaw = normalize_yaw(s_pose.yaw + yaw_diff);
    //     s_last_mpu_yaw = current_mpu_yaw;
    // }
    // 若变化小于阈值，则保持上次 yaw 不变

    // s_pose.yaw = _yaw;  // 直接使用当前计算的相对航向，忽略滤波（如果需要滤波可以改为上面注释的方式）
    s_pose.yaw = _yaw;  
    s_pose.pitch = _pitch;
    s_pose.roll  = _roll;

    // 结合编码器与航向角更新全局位置（毫米）：只更新 x_mm,y_mm
    // 20ms 周期，线速度为 mm/s
    const float dt_s = 20.0f / 1000.0f;
    float delta_s_mm = (float)s_pose.linear_velocity_mm_s * dt_s;  // 一次周期前进距离

    // yaw 为相对航向（度），转换为弧度
    float yaw_rad = s_pose.yaw * (3.14159265358979323846f / 180.0f);
    s_pose.x_mm += (int32_t)(delta_s_mm * cosf(yaw_rad));
    s_pose.y_mm += (int32_t)(delta_s_mm * sinf(yaw_rad));

    // 更新grid坐标
    s_pose.x_grid = s_pose.x_mm / GLOBAL_GRID_SIZE_MM;
    s_pose.y_grid = s_pose.y_mm / GLOBAL_GRID_SIZE_MM;

    // 十字路口识别与对齐
    bool is_crossroad = ((s_pose.seven_data & CROSSROAD_BITMAP_MIDDLE5_MASK) == CROSSROAD_BITMAP_ALL_WHITE);
    if (is_crossroad && !s_crossroad_active) {
        s_crossroad_active = true;

        // 记录十字路口坐标（对齐前）
        s_crossroad_x_mm = s_pose.x_mm;
        s_crossroad_y_mm = s_pose.y_mm;

        float abs_yaw = fabsf(s_pose.yaw);

        if (fabsf(abs_yaw - 0.0f) <= CROSSROAD_YAW_TOLERANCE_DEG || fabsf(abs_yaw - 180.0f) <= CROSSROAD_YAW_TOLERANCE_DEG) {
            // 车身朝向0/180度，按X坐标网格对齐
            int32_t aligned_x = (int32_t)(roundf((float)s_pose.x_mm / CROSSROAD_ALIGN_GRID_MM) * CROSSROAD_ALIGN_GRID_MM);
            s_pose.x_mm = aligned_x;
            s_crossroad_x_mm = aligned_x;
        } else if (fabsf(abs_yaw - 90.0f) <= CROSSROAD_YAW_TOLERANCE_DEG) {
            // 车身朝向90度，按Y坐标网格对齐
            int32_t aligned_y = (int32_t)(roundf((float)s_pose.y_mm / CROSSROAD_ALIGN_GRID_MM) * CROSSROAD_ALIGN_GRID_MM);
            s_pose.y_mm = aligned_y;
            s_crossroad_y_mm = aligned_y;
        } else {
            // 非纯 0/90 时，按最近网格对齐（可选双轴）
            int32_t aligned_x = (int32_t)(roundf((float)s_pose.x_mm / CROSSROAD_ALIGN_GRID_MM) * CROSSROAD_ALIGN_GRID_MM);
            int32_t aligned_y = (int32_t)(roundf((float)s_pose.y_mm / CROSSROAD_ALIGN_GRID_MM) * CROSSROAD_ALIGN_GRID_MM);
            s_pose.x_mm = aligned_x;
            s_pose.y_mm = aligned_y;
            s_crossroad_x_mm = aligned_x;
            s_crossroad_y_mm = aligned_y;
        }

        // 更新grid坐标
        s_pose.x_grid = s_pose.x_mm / GLOBAL_GRID_SIZE_MM;
        s_pose.y_grid = s_pose.y_mm / GLOBAL_GRID_SIZE_MM;
    } else if (!is_crossroad) {
        s_crossroad_active = false;
    }

    // 现在x_mm/y_mm是世界坐标毫米值；x_grid/y_grid已更新
    // END 更新s_pose
}

GlobalPose_t GlobalLoc_GetPose(void)
{
    return s_pose;
}


