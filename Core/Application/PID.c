#include "PID.h"
#include "math.h"
#include <stdlib.h>
int left_motor, right_motor;
float target_angle;

char k11[20];
int data_1;

float Normalization(float data)
{
    float new_angle = data;
    new_angle = fmodf(new_angle, 360.0f);
    if (new_angle > 180.0f)
        new_angle -= 360.0f;
    else if (new_angle <= -180.0f)
        new_angle += 360.0f;
    return new_angle;
}

float constrain(float constrained, float max_up, float max_down)
{
    if (constrained > max_up)
        return max_up;
    if (constrained < max_down)
        return max_down;
    return constrained;
}

int sign(float num)
{
    if (num > 0)
    {
        return 1;
    }
    else if (num < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
float PID_Update(PIDController *pid, float setpoint, float input)
{
    const float dt = 0.02f; // �̶�20ms����

    // 1. ������
    double error = setpoint - input;

    // 2. ����������������ͣ�
    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->integral_limit), -pid->integral_limit);

    // 3. �Ľ���΢�����������仯�ʣ������趨ֵͻ��Ӱ�죩
    float derivative = (input - pid->prev_input) / dt;

    // 4. ΢�����ͨ�˲���һ���ͺ��˲���
    pid->derivative_filtered = pid->filter_coef * pid->derivative_filtered + (1 - pid->filter_coef) * derivative;

    // 5. �������
    float output = pid->Kp * error + pid->Ki * pid->integral - pid->Kd * pid->derivative_filtered; // ע�����

    // 6. ����޷��������ʹ���
    if (output > pid->output_limit)
    {
        output = pid->output_limit;
        pid->integral -= error * dt; // Clamping������
    }
    else if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
        pid->integral -= error * dt;
    }

    // 7. ״̬����
    pid->prev_error = error;
    pid->prev_input = input;

    return output;
}

// ��ʼ��PID����
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd,
              float output_lim, float integ_lim, float filter_coef)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_lim;
    pid->integral_limit = integ_lim;
    pid->filter_coef = filter_coef;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_input = 0;
    pid->derivative_filtered = 0;
}

PIDController motor_left;
PIDController motor_right;
void motor_pid_init()
{
    PID_Init(&motor_left, 3.2, 0.5f, 0.3f, 800, 400, 0.02f);
    PID_Init(&motor_right, 3, 0.5f, 0.3f, 800, 400, 0.02f);
}

void motor_pid_control(int set_left, int set_right)
{
    //		PID_Update(&motor_left, set_left, encoderLeft);
    //	  PID_Update(&motor_right, set_right, -encoderRight);

    left_motor = PID_Update(&motor_left, set_left, encoderLeft);
    right_motor = PID_Update(&motor_right, set_right, -encoderRight);

    MG513_Set((int)(left_motor * 2.85), (int)(right_motor * 2.85));
}

void motor_speed_set(int set_left, int set_right)
{
    left_speed = set_left;
    right_speed = set_right;
}
PIDController seven_line;

void seven_line_pid_init()
{
#if KT
    PID_Init(&seven_line, sevenway_pid_p, sevenway_pid_i, sevenway_pid_d, 500);
#else
    PID_Init(&seven_line, 5, 0.01, 4, 1000, 500, 0.02);
#endif
}

void seven_line_pid_control(int speed)
{
#if KT
    int left;
    int right;
    int out;
    float data;
    if (count_zeros_8bit(seven_ff) > 4)
        return;
    //	if(sevenway_data<200&&sevenway_data>-200)out=0;
    //	else if(sevenway_data>=200) out=sevenway_data-200;
    //	else if(sevenway_data<=-200) out=sevenway_data+200;
    PID_Update(&seven_line, 0, sevenway_data / 20);
    data = PID_Update(&seven_line, 0, sevenway_data / 20);
    char k1[20] = "";
    // if(fabsf(data)>speed*0.6)data=sign(data)*speed*0.6;
    left_speed = speed - data;
    right_speed = speed + data;
//    motor_pid_control(left, right);
// MG513_Set(left*2.85,right*2.85);
//	sprintf(k1, "%d %d %f ", left,right,seven_line.integral);
//  OLED_ShowString(1, 32, k1, OLED_8X16);
#else
    int left;
    int right;
    float data = PID_Update(&seven_line, 0, sevenway_data * 0.2);
    char k1[20] = "";
    left = speed + data;
    right = speed - data;
    motor_pid_control(left, right);
#endif
}
PIDController length_pid;
void length_pid_init()
{
    PID_Init(&length_pid, 2, 0, 0, 1000, 500, 0.02f);
    encoder_total_l = 0;
    encoder_total_r = 0;
}
void length_pid_control(int left, int right)
{
    int l;
    int r;
    PID_Update(&length_pid, left, encoderLeft);
    PID_Update(&length_pid, right, encoderRight);
    l = PID_Update(&length_pid, left, encoderLeft);
    r = PID_Update(&length_pid, right, encoderRight);
    motor_pid_control(l, r);
}

void length_control(int left, int right)
{
    encoder_total_l = 0;
    encoder_total_r = 0;
    while (1)
    {
        if ((abs(left) - abs(encoder_total_l)) <= 0 && (abs(right) - abs(encoder_total_r)) <= 0)
            break;
        length_pid_control(left, right);
    }
    MG513_Set(0, 0);
}

PIDController mpu6050_pid;

void mpu6050_pid_control(int speed, float target)
{

    //   mpu_dmp_get_data(&x_data, &y_data, &z_data);
    //	if(error>180)error=360-error;
    //	if(error<-180)error=error+360;
    // PID_Update(&mpu6050_pid, target_angle,z_data);

    // z_data = z + fmin(tim6_at / 50, 15) + 0.49f; //��Ư����

    //	  target=target_angle-sevenway_data*0.8;//����7·ƫ�����ı�Ŀ��Ƕ�
    //	  if(target==0)target_angle=target;
    //	  if(sevenway_data>=15||sevenway_data<=-15)target_angle=target_angle-sevenway_data*0.01;
    // target_angle_set(-sevenway_data*0.05);
    mpu6050_pid.Kp = 2 + fmax(speed - 100, 0) * 0.02;
    float error = Normalization(target - z_data);
    data_1 = (int)PID_Update(&mpu6050_pid, 0, -error);

    left_speed = speed - data_1;
    right_speed = speed + data_1;
}

void mpu6050_turn_angle(float target, uint8_t way)
{
    float error = Normalization(target - z_data);
    data_1 = (int)PID_Update(&mpu6050_pid, 0, -error);
    if (data_1 > 0)
    {

        left_speed = -data_1; //*0.9f;//*(1-sevenway_data*0.02*way);
        right_speed = data_1; //*1.2f;//*(1+sevenway_data*0.01*way);
    }
    else
    {
        left_speed = -data_1; //*1.5f;//*(1+sevenway_data*0.01f*way);
        right_speed = data_1; //*0.7f;//*(1-sevenway_data*0.01f*way);
    }
}

void mpu6050_turn_angle_oneway(float target, int Direction)
{
    float error = Normalization(target - z_data);
    data_1 = (int)PID_Update(&mpu6050_pid, 0, -error);
    switch (Direction)
    {
    case 1:
        motor_speed_set(data_1, 0);
        break;
    case 2:
        motor_speed_set(-data_1, 0);
        break;
    case 3:
        motor_speed_set(0, data_1);
        break;
    case 4:
        motor_speed_set(0, -data_1);
        break;
    }
}

void mpu6050_pid_reset(float kp, float ki, float kd, float output, float inte)
{
    PID_Init(&mpu6050_pid, kp, ki, kd, output, inte, 0.02f);
}

PIDController ms_c;
void target_angle_set(float data)
{
    float new_angle = target_angle + data;
    // ��ȷ��һ����[-180��, 180��)
    new_angle = fmodf(new_angle, 360.0f);
    if (new_angle > 180.0f)
        new_angle -= 360.0f;
    else if (new_angle <= -180.0f)
        new_angle += 360.0f;
    target_angle = new_angle;
    mpu6050_sevenway_init();
}

static float prev_z = 0.0f;
static float modify = 0.0f;
static float filtered_sevenway = 0;
static uint32_t t = 0;

#define se 1
void mpu6050_sevenway_init()
{
    // 200: PID_Init(&ms_c, 0.25f, 0.03f, 0.05f, 45.0f, 3000.0f, 0.02f); // ��������PID
// PID_Init(&ms_c, 0.8f, 0.06f, 0.05f, 45.0f, 3000.0f, 0.02f);//120
#if se
    PID_Init(&ms_c, 1.2f, 0.01f, 0.25f, 45.0f, 100.0f, 0.02f); // 120
#else
    PID_Init(&ms_c, 0.02f, 0.00f, 0.025f, 0.1f, 1000.0f, 0.02f); // 120
#endif
}
static float delta = 0;
static float data__l;
static float target;
static float prev_sevenway = 0.0f;
static int prev_motion_dir = 0;
static uint32_t mpu6050_ctrl_tick20ms = 0;
#define SEVENWAY_CENTER_ALPHA 0.12f
#define SEVENWAY_CENTER_GAIN 1.20f
// Ѳ�߿��ƺ�����20ms���ڵ��ã�
void mpu6050_sevenway_control(int speed)
{

#if se
    int speed_abs;
    int motion_dir;
    float sevenway_for_control;

    target = target_angle;
    error_calculate();

    speed_abs = abs(speed);
    motion_dir = (speed >= 0) ? 1 : -1;
    sevenway_for_control = (float)(motion_dir * sevenway_data) * SEVENWAY_CENTER_GAIN;

    if (prev_motion_dir != 0 && prev_motion_dir != motion_dir)
    {
        delta = 0.0f;
        ms_c.integral = 0.0f;
        ms_c.prev_error = 0.0f;
        ms_c.prev_input = 0.0f;
        ms_c.derivative_filtered = 0.0f;
    }
    prev_motion_dir = motion_dir;

    // 提高七路误差进入控制环的速度，增强“回中”能力。
    delta = SEVENWAY_CENTER_ALPHA * sevenway_for_control + (1.0f - SEVENWAY_CENTER_ALPHA) * delta;
    if (speed_abs > 70)
    {
        ms_c.output_limit = 25.0f;
        if (fabsf(delta) >= 24)
            ms_c.Kp = 0.5f;
        else if (fabsf(delta) >= 19)
            ms_c.Kp = 0.3f;
        else if (fabsf(delta) >= 9)
            ms_c.Kp = 0.25f;
        else
            ms_c.Kp = 0.25f;
    }
    else
    {
        ms_c.output_limit = 34.0f;
        ms_c.Kp = 0.9f;
        ms_c.Kd = 0.15f;
    }
    data__l = PID_Update(&ms_c, 0, delta);
    target = Normalization(target + data__l);
    if (receive_flag)
    {
        mpu6050_ctrl_tick20ms++;
        receive_flag = 0;
        mpu6050_pid_control(speed, target);
    }
#else
    if (receive_flag)
    {
        int motion_dir = (speed >= 0) ? 1 : -1;
        mpu6050_ctrl_tick20ms++;
        receive_flag = 0;
        error_calculate();
        delta = 0.05f * (float)(motion_dir * sevenway_data) + 0.95f * delta;
        data__l = PID_Update(&ms_c, 0, delta);
        target_angle = Normalization(target_angle + data__l);
    }
    mpu6050_pid_control(speed, target_angle);
#endif
}

uint32_t mpu6050_get_ctrl_tick20ms(void)
{
    return mpu6050_ctrl_tick20ms;
}

void mpu6050_reset_ctrl_tick20ms(void)
{
    mpu6050_ctrl_tick20ms = 0;
}

#define HISTORY_SIZE 128                    // ��ʷ���ݻ�������С������Ļ����һ�£�
static int16_t history[HISTORY_SIZE] = {0}; // �洢ƫ��ֵ����ʷ����
static uint8_t data_index = 0;              // ��������λ��
static float max_value = 10.0f;             // �������ֵ���Զ�������

/**
 * @brief  ��ʾĿ��ֵ��ƫ��Ķ�̬����
 * @param  target Ŀ��ֵ����Χ��-1000~1000��
 * @param  deviation ƫ��ֵ����Χ��-1000~1000��
 * @retval ��
 */
void OLED_ShowDynamicData(int16_t target, int16_t deviation)
{
    // 1. ���������ʾ���򣨱���������������
    OLED_ClearArea(0, 16, 128, 48);

    // 2. ������ʷ����
    history[data_index] = deviation;
    data_index = (data_index + 1) % HISTORY_SIZE;

    // 3. �Զ��������᷶Χ�����ͺ��˲���
    static float prev_max = 10.0f;
    float current_max = fabsf(deviation * 1.2f); // ��20%����
    max_value = (current_max > prev_max) ? current_max : prev_max * 0.95f;
    prev_max = max_value;

    // 4. ����Ŀ��ο��ߣ���ɫ��
    uint8_t target_y = 16 + (int)((target / max_value) * 24);
    target_y = (target_y > 63) ? 63 : (target_y < 16) ? 16
                                                      : target_y;
    for (int x = 0; x < 128; x++)
    {
        OLED_DrawPoint(x, target_y); // ���ƺ��ߣ�����Ϸ�ɫģʽ��
    }

    // 5. ���ƶ�̬���ߣ���ɫ��
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        int idx = (data_index + i) % HISTORY_SIZE;
        float ratio = history[idx] / max_value;
        uint8_t y = 40 - (ratio * 24); // 16~40����Ϊ��������
        y = (y < 16) ? 16 : (y > 40) ? 40
                                     : y;
        OLED_DrawPoint(i, y); // �������ݵ�
    }

    // 6. ʵʱ��ֵ��ʾ�����Ͻǣ�
    char str[16];
    snprintf(str, sizeof(str), "T:%2d", target);
    OLED_ShowString(80, 0, str, OLED_8X16);
    snprintf(str, sizeof(str), "D:%2d", deviation);
    OLED_ShowString(80, 16, str, OLED_8X16);

    // 7. �ֲ�ˢ�£���������������
    OLED_UpdateArea(0, 16, 128, 48);
}
