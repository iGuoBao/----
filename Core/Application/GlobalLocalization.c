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

// 十字路口检测与栅格定位
#define CROSSROAD_BITMAP_ALL_WHITE 0x00
#define CROSSROAD_BITMAP_MIDDLE5_MASK 0x3E  // bit1~bit5
static bool s_crossroad_active = false;
static bool s_crossroad_anchor_ready = false;

typedef struct
{
    int8_t from_x;
    int8_t from_y;
    int8_t to_x;
    int8_t to_y;
} CrossroadSkipEdgeRule_t;

static const CrossroadSkipEdgeRule_t s_crossroad_skip_edges[] = {
    {1, 5, 1, 6}, {1, 7, 1, 6}, {2, 5, 2, 6}, {2, 7, 2, 6},
    {1, 5, 1, 4}, {1, 3, 1, 4}, {2, 5, 2, 4}, {2, 3, 2, 4},
    {6, 5, 6, 6}, {6, 7, 6, 6}, {7, 5, 7, 6}, {7, 7, 7, 6},
    {6, 5, 6, 4}, {6, 3, 6, 4}, {7, 5, 7, 4}, {7, 3, 7, 4},
};

// 辅助：把角度裁剪到 [-180, 180)
static float normalize_yaw(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int32_t mm_to_grid_nearest(int32_t mm, int32_t max_grid)
{
    int32_t grid = (int32_t)lroundf((float)mm / (float)GLOBAL_GRID_SIZE_MM);
    return clamp_i32(grid, 0, max_grid);
}

static float abs_angle_diff(float a, float b)
{
    return fabsf(normalize_yaw(a - b));
}

static void yaw_to_grid_step(float yaw_deg, int32_t *step_x, int32_t *step_y)
{
    static const float yaw_candidates[4] = {0.0f, 90.0f, 180.0f, -90.0f};
    static const int8_t step_candidates[4][2] = {
        {1, 0},
        {0, 1},
        {-1, 0},
        {0, -1},
    };

    float min_diff = 1e9f;
    uint8_t min_idx = 0;

    if (step_x == NULL || step_y == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float diff = abs_angle_diff(yaw_deg, yaw_candidates[i]);
        if (diff < min_diff)
        {
            min_diff = diff;
            min_idx = i;
        }
    }

    // 如果角度与候选方向的差异过大（例如 > 45度），可以选择不进行步进，避免误判
    if (min_diff > 20.0f)
    {
        *step_x = 0;
        *step_y = 0;
    }
    else
    {
        *step_x = step_candidates[min_idx][0];
        *step_y = step_candidates[min_idx][1];
    }
}

static bool is_crossroad_skip_edge(int32_t from_x, int32_t from_y, int32_t to_x, int32_t to_y)
{
    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_crossroad_skip_edges) / sizeof(s_crossroad_skip_edges[0])); i++)
    {
        const CrossroadSkipEdgeRule_t *rule = &s_crossroad_skip_edges[i];
        if (rule->from_x == from_x &&
            rule->from_y == from_y &&
            rule->to_x == to_x &&
            rule->to_y == to_y)
        {
            return true;
        }
    }

    return false;
}

void GlobalLoc_Init()
{
    s_pose.x_mm = GLOBAL_INIT_X_MM;
    s_pose.y_mm = GLOBAL_INIT_Y_MM;
    s_pose.x_grid = GLOBAL_INIT_X_GRID;
    s_pose.y_grid = GLOBAL_INIT_Y_GRID;
    s_pose.abs_yaw = GLOBAL_INIT_YAW_DEG;
    s_pose.yaw = 0;

    s_crossroad_active = false;
    s_crossroad_anchor_ready = false;

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
    s_pose.x_grid = mm_to_grid_nearest(x_mm, GLOBAL_MAP_X_GRID - 1);
    s_pose.y_grid = mm_to_grid_nearest(y_mm, GLOBAL_MAP_Y_GRID - 1);
    s_pose.yaw = yaw_deg;
    s_crossroad_active = false;
    s_crossroad_anchor_ready = false;
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
    s_pose.seven_data = seven_ff;
    // END   处理七路传感器数据

    // START 更新s_pose
    s_pose.yaw = _yaw;
    s_pose.pitch = _pitch;
    s_pose.roll  = _roll;

    // 十字路口触发定位：不再用编码器积分 x/y
    bool is_crossroad = ((s_pose.seven_data & CROSSROAD_BITMAP_MIDDLE5_MASK) == CROSSROAD_BITMAP_ALL_WHITE);
    if (is_crossroad && !s_crossroad_active)
    {
        s_crossroad_active = true;

        if (1)
        {
            int32_t step_x = 0;
            int32_t step_y = 0;
            int32_t step_count = 1;

            // 最小策略：只在前进时推进到下一个预期格点。
            if (s_pose.linear_velocity_mm_s > 0)
            {
                yaw_to_grid_step(s_pose.yaw, &step_x, &step_y);
                s_pose.x_grid = clamp_i32(s_pose.x_grid + step_x * step_count, 0, GLOBAL_MAP_X_GRID - 1);
                s_pose.y_grid = clamp_i32(s_pose.y_grid + step_y * step_count, 0, GLOBAL_MAP_Y_GRID - 1);
            }
        }

        // 十字事件后直接对齐到栅格毫米坐标
        s_pose.x_mm = s_pose.x_grid * GLOBAL_GRID_SIZE_MM;
        s_pose.y_mm = s_pose.y_grid * GLOBAL_GRID_SIZE_MM;
    }
    else if (!is_crossroad)
    {
        s_crossroad_active = false;
    }

    // 现在 x_mm/y_mm 由栅格坐标直接映射得到
    // END 更新s_pose
}

GlobalPose_t GlobalLoc_GetPose(void)
{
    return s_pose;
}


