#include "action.h"
#include "math.h"

int count_zeros_8bit(uint8_t num)
{
    if (num == 0)
        return 8; // ȫ0ֱ�ӷ���8
    int count = 0;
    for (int i = 0; i < 8; i++)
    {
        if (!(num & (1U << i)))
            count++; // ����iλ�Ƿ�Ϊ0
    }
    return count;
}

void turn_angle(float angle)
{
    uint32_t t = tim6_at;
    //    while(tim6_at - t < 30)
    //    {
    //        mpu6050_pid_control(0, target_angle);
    //    }
    mpu6050_pid_reset(2.1, 0.0f, 0.2f, 120, 3000);

    // float radian = 0;

    target_angle = Normalization(target_angle + angle);
    t = tim6_at;
    //    if(angle != 179)
    //    {
    //        while(tim6_at - t < (110 + 0.5 * (fmax(fabsf(angle) - 90, 0))))
    //        {
    //            mpu6050_pid_control(0, target_angle);
    //        }
    //    }
    //    else
    //    {
    error_calculate();
    //    while(tim6_at - t < (100 + 0.5 * (fmax(fabsf(angle) - 90, 0))))
    //    {
    //        if(receive_flag){receive_flag=0;mpu6050_turn_angle(target_angle, 1);}
    //    }
    while (1)
    {
         // 多次验证在误差里
        static uint16_t flag = 0;
        if (fabsf(Normalization(target_angle - z_data)) < 5)
        {
            flag++;
        }

        if (flag >= 10000){
            flag = 0;
            break;
        }



        if (receive_flag)
        {
            receive_flag = 0;
            mpu6050_turn_angle(target_angle, 1);
        }
    }
    //    }
    mpu6050_pid_reset(2, 0.01f, 0.02f, 200, 3000);
    motor_pid_init();
    motor_speed_set(0, 0);
}

void turn_left()
{
    // left_reset();
    mpu6050_sevenway_init();
    turn_angle(90.0f);

    //	  error_calculate();
    //    turn_angle(90-sevenway_data);
}

// ����дһ��mpu_pid������pֵ��С����֤��һ����������60�ٶ�Ѳ��
void turn_right()
{
    mpu6050_sevenway_init();
    turn_angle(-90.0f);

    //	  error_calculate();
    //    turn_angle(-90-sevenway_data);
}
void turn_around()
{
#if 1
    mpu6050_sevenway_init();
    turn_angle(-180.0f);

#else
    mpu6050_pid_reset(2, 0, 0.5, 100, 3000);
    uint32_t t = tim6_at;
    float radian = 0;
    while (tim6_at - t < 30)
    {
        mpu6050_pid_control(0, target_angle);
    }
    target_angle_set(-90);
    while (1)
    {
        mpu6050_turn_angle_oneway(target_angle, 3);

        if (fabsf(Normalization(target_angle - z_data)) < 5)
            break;
    }
    target_angle_set(-90);
    while (1)
    {
        mpu6050_turn_angle_oneway(target_angle, 2);

        if (fabsf(Normalization(target_angle - z_data)) < 5)
            break;
    }
    mpu6050_pid_reset(2, 0.01f, 0.02f, 200, 3000);
    motor_speed_set(0, 0);
#endif
}

#define CROSSROAD_MASK 0x00 // ������3��4��5Ϊ�м���·

void forward(int data)
{
    uint8_t state = 0;
    uint8_t current_state = 0;
    uint8_t flag = 0;
    int speed = 0;
    int back_flag = 0;

    // 如果data是负数 等会就倒车
    if (data <0){
        back_flag = 1;
        data = -data;
    }

    flag = 0;
    while (1)
    {
        if (count_zeros_8bit(seven_ff) >= 5)
            current_state = 1;
        else
            current_state = 0;
        if ((state ^ current_state) & 0xff)
        {
            state = current_state;
            flag++;
        }
        if (data * 2 - 1 == flag)
            break;

        if (data == 1 || data == 2)
        {
            speed = 80;
        }
        else if (flag / 2 + 1 >= data * 0.85)
        {
            speed = 90;
        }
        else
        {
            speed = 100; // 200
        }

        if (back_flag) speed = -speed;
        mpu6050_sevenway_control(speed);
    }
    // motor_speed_set(-50, -50);

    //		flag=0;
    //
    //    while(1)
    //    {
    //        //mpu6050_pid_control(-100, target_angle);
    //			  if(count_zeros_8bit(seven_ff) >= 5)current_state = 1;
    //        else current_state = 0;
    //			  if((state ^ current_state) & 0xff)
    //        {
    //            state = current_state;
    //            flag++;
    //        }
    //				if(flag==2)break;
    //    }
    motor_pid_init();
    motor_speed_set(0, 0);
}

void forward_begin()
{
    uint32_t t1 = 0;
    while (1)
    {
        if (receive_flag)
        {
            receive_flag = 0;
            t1++;
        }
        if (t1 > 120)
            break;
    }
}

void forward_slow()
{
    motor_speed_set(70, 70);
    delay_jx(1300);
    motor_speed_set(0, 0);
}

void wait()
{
    motor_speed_set(0, 0);
    delay_jx(1000);
}

void back(int len)
{
    encoder_total_l = encoder_total_r = 0;

    while (1)
    {
        if (receive_flag)
        {
            receive_flag = 0;
            encoder_total_l += encoderLeft;
            encoder_total_r += encoderRight;
            mpu6050_pid_control(-120, target_angle);
            if (fabsf((encoder_total_l - encoder_total_r) / 2.0f) > len){
                break;
            }
                
        }
    }
}

void route(char Road[50])
{
    uint8_t i = 0;
    while (1)
    {

        switch (Road[i])
        {
        case '1':
            forward(1);
            break;
        case '2':
            forward(2);
            break;
        case '3':
            forward(3);
            break;
        case '4':
            forward(4);
            break;
        case '5':
            forward(5);
            break;
        case '6':
            forward(6);
            break;
        case '7':
            forward(7);
            break;
        case '8':
            forward(8);
            break;
        case '9':
            forward(9);
            break;
        case 'L':
            turn_left();
            break;
        case 'R':
            turn_right();
            break;
        case 'A':
            turn_around();
            break;
        case 'T':
            Servos_up(3);
            break;
        case 't':
            Servos_up(1);
            break;
        case 'd':
            Servos_down(1); // 0004
            break;
        case 'D':
            Servos_down(3); // 0004
            break;
        case 'H':
            Servos_close(1800);
            break;
        case 'K':
            Servos_close(1650);
            break;
        case 'O':
            Servos_open(1250);
            break;
        case 'f':
            forward_slow();
            break;
        case 'b':
            // back(600);
            // forward(-1);
            motor_speed_set(-80, -80);
            delay_jx(1000);
            motor_speed_set(0, 0);
            break;
        case 'w':
            wait();
            break;
        case 'S':
            motor_pid_init();
            while (1)
            {
                mpu6050_pid_control(0, target_angle);
            }
            break;
        default:
            break;
        }
        i++;
    }
}
