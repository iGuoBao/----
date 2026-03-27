#include "Encoder.h"
#include <stdlib.h>
int16_t encoderLeft, encoderRight;
int sevenway_data;
int32_t encoder_total_l; // �ۼ���������
int32_t encoder_total_r; // �ۼ���������
int left_speed, right_speed;
float x_data, y_data, z_data;
int32_t target_pulses = 0; // Ŀ��������
uint32_t tim6_at;
uint8_t remote_data[9] = {0};
char k[20] = "";
uint8_t tx_data[6];
uint8_t receive_flag, seven_ff;
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
        Encoder_Get();
        mpu_dmp_get_data(&x_data, &y_data, &z);
        tim6_at++;
        // z=0.9*z+0.1*fter;
        if (fabsf(prev - z) < 0.15f)
            offset = offset + prev - z;
        z_data = Normalization(z + offset);
        motor_pid_control(left_speed, right_speed);


        if (!receive_flag)
            receive_flag = 1;

        
        sprintf(k, "%d  %d  ", encoderLeft, -encoderRight);
        OLED_ShowString(0, 0, k, OLED_8X16);
        OLED_ShowFloatNum(0, 16, z_data, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 32, target_angle, 3, 3, OLED_8X16);
        OLED_ShowFloatNum(0, 48, prev - z, 1, 3, OLED_8X16);

        OLED_Update();
        prev = z;
    }
}
