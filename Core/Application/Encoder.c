#include "Encoder.h"
#include "GlobalLocalization.h"
#include "car_type.h"
#include "loader_strategy.h"
#include <stdlib.h>
int16_t encoderLeft, encoderRight;
int sevenway_data;
int32_t encoder_total_l; 
int32_t encoder_total_r; 

int left_speed, right_speed;
float x_data, y_data, z_data;
int32_t target_pulses = 0; 
uint32_t tim6_at;
uint8_t remote_data[9] = {0};
uint8_t tx_data[6];
uint8_t receive_flag, seven_ff;


int16_t encoder_left_speed_mm_s, encoder_right_speed_mm_s; // 存储左右轮的速度 mm/s
int16_t encoder_speed_mm_s;                           // 存储小车的线速度 mm/s
int16_t encoder_left_angular_speed_deg_s, encoder_right_angular_speed_deg_s; // 左右轮角速度 deg/s
int16_t encoder_angular_speed_deg_s; // 小车角速度 deg/s
float encoder_angular_speed_rad_s; // 小车角速度 rad/s


void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // ����������ģʽ
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // ����������ģʽ
    HAL_TIM_Base_Start_IT(&htim2);                  // �������������ж�
    HAL_TIM_Base_Start_IT(&htim3);                  // �������������ж�
    HAL_TIM_Base_Start_IT(&htim6);                  // ������ʱ�����ж�
    tim6_at = 0;
    receive_flag = 0;
    left_speed = 0;
    right_speed = 0;
}
void int16_to_3uint8(int16_t value, uint8_t *sign, uint8_t *high, uint8_t *low)
{
    // 1. ��ȡ����λ��0��ʾ����1��ʾ����
    *sign = (value >> 15) & 0x01;

    // 2. ���������ľ���ֵ��ȡ���룩
    int16_t abs_value = (*sign) ? -(value + 1) : value;

    // 3. �����7λ�͵�8λ
    *high = (abs_value >> 8) & 0x7F; // ȡ��7λ������0x7F���˷���λ��
    *low = abs_value & 0xFF;         // ȡ��8λ
}
// ������int16_t����uint8_t����
void pack_int16_array(int16_t *src, uint8_t *dest)
{
    for (int i = 0; i < 2; i++)
    {
        int16_to_3uint8(src[i],
                        &dest[i * 3],
                        &dest[i * 3 + 1],
                        &dest[i * 3 + 2]);
    }
}

void uint8_to_bin_str(uint8_t value, char *buffer)
{
    for (int i = 0; i < 8; i++)
    {
        // �Ӹ�λ����λ���μ��ÿһλ�Ƿ�Ϊ1
        buffer[i] = (value & (0x80 >> i)) ? '1' : '0'; // 0x80��Ӧ������10000000
    }
    buffer[8] = '\0'; // �����ַ���������
}
void turn(uint8_t *rx_data)
{
    int16_t nums[2] = {encoderLeft, encoderRight};
    pack_int16_array(nums, rx_data);
}
#define PI 3.1415926
void Encoder_Get(void)
{

    encoderLeft = __HAL_TIM_GetCounter(&htim2); // ��ȡ����ֵ
    // encoder_total_l += encoderLeft;
    __HAL_TIM_SetCounter(&htim2, 0); // ��ռ���ֵ

    encoderRight = __HAL_TIM_GetCounter(&htim3); // ��ȡ����ֵ
    // encoder_total_r += encoderRight;
    __HAL_TIM_SetCounter(&htim3, 0); // ��ռ���ֵ

    #define ENCODER_INTERVAL_MS 20 // 编码器速度计算的时间间隔，单位毫秒

        // 计算每脉冲对应的毫米数
    float mm_per_pulse = (float)WHEEL_CIRCUMFERENCE_MM / (float)WHEEL_PULSES_PER_REV;
    // 速度换算因子：从每 ENCODER_INTERVAL_MS 毫秒的脉冲数 -> mm/s
    float speed_factor = 1000.0f / ENCODER_INTERVAL_MS;
    /* 计算左右轮速度（mm/s），使用 32 位中间结果防止溢出 */
    float left_speed_f = (float)encoderLeft * mm_per_pulse * speed_factor;
    float right_speed_f = (float)encoderRight * mm_per_pulse * speed_factor;
    right_speed_f = -right_speed_f; // 右轮编码器方向相反，取负值
    /* 截断为 int16_t 存储（保持原有接口），同时保存角速度 */
    if (left_speed_f > (float)INT16_MAX) left_speed_f = (float)INT16_MAX;
    if (left_speed_f < (float)INT16_MIN) left_speed_f = (float)INT16_MIN;
    if (right_speed_f > (float)INT16_MAX) right_speed_f = (float)INT16_MAX;
    if (right_speed_f < (float)INT16_MIN) right_speed_f = (float)INT16_MIN;
    encoder_left_speed_mm_s = (int16_t)left_speed_f;
    encoder_right_speed_mm_s = (int16_t)right_speed_f;
    
    /* 计算小车线速度 mm/s（两轮平均，沿X轴前进方向） */
    encoder_speed_mm_s = (int16_t)(((int32_t)encoder_left_speed_mm_s + (int32_t)encoder_right_speed_mm_s) / 2);

    // 计算左右轮角速度 deg/s（单轮自转速度）
    float left_angular_speed_f = ((float)encoder_left_speed_mm_s / (float)WHEEL_CIRCUMFERENCE_MM) * 360.0f;
    float right_angular_speed_f = ((float)encoder_right_speed_mm_s / (float)WHEEL_CIRCUMFERENCE_MM) * 360.0f;
    encoder_left_angular_speed_deg_s = (int16_t)left_angular_speed_f;
    encoder_right_angular_speed_deg_s = (int16_t)right_angular_speed_f;
    
    // 小车角速度 deg/s（绕Z轴，ROS标准：逆时针为正）
    // 公式: ω = (v_right - v_left) / wheel_base
    // 右轮快 → 左转 → Yaw增大（逆时针）→ 角速度为正
    float angular_speed_f = ((float)encoder_right_speed_mm_s - (float)encoder_left_speed_mm_s) / (float)WHEEL_BASE_MM * (180.0f / 3.1415926f);
    encoder_angular_speed_deg_s = (int16_t)angular_speed_f;
    encoder_angular_speed_rad_s = ((float)encoder_right_speed_mm_s - (float)encoder_left_speed_mm_s) / (float)WHEEL_BASE_MM;

}

void StartRotation(int turns, int speed, uint8_t left_or_right)
{
    const uint16_t PULSES_PER_REV = 1;
    target_pulses = turns * PULSES_PER_REV;
    MG513_Set(0, 0);
    if (left_or_right == 1)
    {
        encoder_total_l = 0;
        while (abs(encoder_total_l) < abs(target_pulses))
            motor_pid_control(speed * turns, 0);
        MG513_SetLeft(0);
    }
    else if (left_or_right == 2)
    {
        encoder_total_r = 0;
        while (abs(encoder_total_r) < abs(target_pulses))
            motor_pid_control(0, speed * turns);
        MG513_SetRight(0);
    }
    else
    {
        encoder_total_r = 0;
        encoder_total_l = 0;
        while ((abs(encoder_total_l) + abs(encoder_total_r)) / 2 < abs(target_pulses))
            motor_pid_control(speed * turns, speed * turns);
        MG513_Set(0, 0);
    }
}

float k_data;
float max_data;
static float z;
float prev = 0;
void mpu_0p(void)
{

    float curr_duct = 0;
    int js = 0;
    float data = 0;
    while (1)
    {
        if (receive_flag)
        {
            if (fabsf(prev - z) < 0.0001 && js > 50)
                break;
            receive_flag = 0;
            curr_duct = curr_duct + prev - z;
            prev = z;
            js++;
        }

        OLED_ShowFloatNum(0, 48, curr_duct / js, 1, 5, OLED_8X16);
    }
    k_data = curr_duct / js;
    max_data = curr_duct;
    // tim6_at=0;
}

float offset;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim2) // ������
    {
    }
    else if (htim == &htim3) // ������
    {
    }
    else if (htim == &htim6) // ����ת��20ms
    {
        // Encoder_Get();
        // mpu_dmp_get_data(&x_data, &y_data, &z);
        GlobalLoc_Periodic();
        GlobalPose_t pose = GlobalLoc_GetPose();
        z = pose.yaw;
        tim6_at++;
        z_data = Normalization(z);
        motor_pid_control(left_speed, right_speed);
        prev = z;
        if (!receive_flag)
            receive_flag = 1;
        
        static uint16_t oled_flag = 0;
        static uint16_t oled_clear_flag = 0;
        if (oled_flag++ < 15)
            return;
        char k[40] = "";
        oled_flag = 0;

        if (!is_start_successful())
        {
            return;
        }
        // // 显示坐标
        // sprintf(k, "X:%4d Y:%4d", pose.x_mm, pose.y_mm);
        // OLED_ShowString(0, 0, k, OLED_8X16);
        // // 显示航向
        // sprintf(k, "Yaw:%06.1f", pose.yaw);
        // OLED_ShowString(0, 16, k, OLED_8X16);
        // 显示grid
        sprintf(k, "Grid:%2d,%2d", pose.x_grid, pose.y_grid);
        OLED_ShowString(0, 32, k, OLED_8X16);
        // 显示编码器速度 左右
        // sprintf(k, "L:%3d R:%3d", encoder_left_speed_mm_s, encoder_right_speed_mm_s);
        // OLED_ShowString(0, 48, k, OLED_8X16);
        // 显示七路传感器
        sprintf(k, "S:%d", pose.seven_data);
        OLED_ShowString(0, 48, k, OLED_8X16);
        // LoaderStrategyState_t status = LoaderStrategy_GetState();
        // sprintf(k, "Cmd:%d", status);
        // OLED_ShowString(0, 48, k, OLED_8X16);

        OLED_Update();

        if (oled_clear_flag++<3)
            return;
        OLED_Clear();
        oled_clear_flag = 0;
    }
}
