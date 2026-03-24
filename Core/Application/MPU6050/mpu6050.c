#include "mpu6050.h"
#include "math.h"
#include "main.h"
#include "i2c.h"
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Init(void)
{
    uint8_t res;
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);	//复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);	//唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
    MPU_Set_Rate(200);						//设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	//I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	//关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	//INT引脚低电平有效
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res == MPU_ADDR) //器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01);	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);	//加速度与陀螺仪都工作
        MPU_Set_Rate(100);						//设置采样率为50Hz
    }
    else return 1;
    return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if(lpf >= 188)data = 1;
    else if(lpf >= 98)data = 2;
    else if(lpf >= 42)data = 3;
    else if(lpf >= 20)data = 4;
    else if(lpf >= 10)data = 5;
    else data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate > 1000)rate = 1000;
    if(rate < 4)rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);	//设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short* gx, short* gy, short* gz)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if(res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short* ax, short* ay, short* az)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if(res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;;
}
//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 res;
    res = HAL_I2C_Mem_Write(&hi2c1, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    return res;
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 res;
    res = HAL_I2C_Mem_Read(&hi2c1, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    return res;
}

//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//其他,错误代码
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    u8 res;
    res = HAL_I2C_Mem_Write(&hi2c1, (MPU_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xfff);
    return res;
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    HAL_I2C_Mem_Read(&hi2c1, (MPU_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 0xfff);
    return res;
}






// 陀螺仪原始值转角速度(°/s)
void MPU_Convert_Gyro(short gRaw, float* deg_s, float* rad_s)
{
    const float scale = 16.4f; // ±2000dps灵敏度[3](@ref)
    *deg_s = gRaw / scale;
    *rad_s = *deg_s * 0.0174533f; // °/s转rad/s[1](@ref)
}

// 加速度计原始值转加速度(m/s2)
void MPU_Convert_Accel(short aRaw, float* m_s2)
{
    const float scale = 16384.0f; // ±2g灵敏度[3](@ref)
    *m_s2 = (aRaw / scale) * 9.80665f;
}

// 加速度计解算俯仰角(pitch)和横滚角(roll)
void MPU_Calculate_Angles(float ax, float ay, float az, float* pitch, float* roll)
{
    *pitch = atan2(ay, sqrt(ax * ax + az * az)) * 57.29578f; // rad转°
    *roll  = atan2(ax, sqrt(ay * ay + az * az)) * 57.29578f;
}

// 陀螺仪积分计算偏航角(yaw)

void MPU_Integrate_Yaw(float gyro_z_rad, float dt)
{
    z_data += gyro_z_rad * dt; // dt为采样间隔
    z_data = fmod(z_data, 6.283185307f); // 限制在0-360°[5](@ref)
}



// 自动校准函数
void MPU_Calibrate(MPU_Calib_t* calib, uint16_t samples)
{
    int32_t gSum[3] = {0}, aSum[3] = {0};

    for(int i = 0; i < samples; i++)
    {
        short gx, gy, gz, ax, ay, az;
        MPU_Get_Gyroscope(&gx, &gy, &gz);
        MPU_Get_Accelerometer(&ax, &ay, &az);

        gSum[0] += gx;
        gSum[1] += gy;
        gSum[2] += gz;
        aSum[0] += ax;
        aSum[1] += ay;
        aSum[2] += az - 16384; // Z轴理论值1g[4](@ref)
        HAL_Delay(10);
    }

    for(int i = 0; i < 3; i++)
    {
        calib->gyro_offset[i] = gSum[i] / samples;
        calib->accel_offset[i] = aSum[i] / samples;
    }
}

// 应用校准数据
void MPU_Apply_Calib(MPU_Calib_t* calib, short* gRaw, short* aRaw)
{
    gRaw[0] -= calib->gyro_offset[0];
    gRaw[1] -= calib->gyro_offset[1];
    gRaw[2] -= calib->gyro_offset[2];

    aRaw[0] -= calib->accel_offset[0];
    aRaw[1] -= calib->accel_offset[1];
    aRaw[2] -= calib->accel_offset[2];
}




float Filter_MovingAvg(MovingAverage_t* filter, float new_val)
{
    filter->buffer[filter->index] = new_val;
    filter->index = (filter->index + 1) % FILTER_SIZE;

    float sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++)
        sum += filter->buffer[i];
    return sum / FILTER_SIZE;
}

// 一阶滞后滤波
float Filter_FirstOrder(float prev, float new_val, float alpha)
{
    return alpha * new_val + (1 - alpha) * prev;
}


MPU_Calib_t mpu_calib;
MovingAverage_t gyro_filter[3], accel_filter[3];


void mpu_get()
{
    short gRaw[3], aRaw[3], prev[3] = {0};
    MPU_Get_Gyroscope(&gRaw[0], &gRaw[1], &gRaw[2]);
    MPU_Get_Accelerometer(&aRaw[0], &aRaw[1], &aRaw[2]);
    MPU_Apply_Calib(&mpu_calib, gRaw, aRaw);

    // 数据转换
    float gyro_deg[3], gyro_rad[3], accel_m_s2[3];
    for(int i = 0; i < 3; i++)
    {
        MPU_Convert_Gyro(gRaw[i], &gyro_deg[i], &gyro_rad[i]);
        MPU_Convert_Accel(aRaw[i], &accel_m_s2[i]);

        // 应用滤波
        gyro_deg[i] = Filter_MovingAvg(&gyro_filter[i], gyro_deg[i]);
        accel_m_s2[i] = Filter_FirstOrder(prev[i], accel_m_s2[i], 0.2);
        prev[i] = accel_m_s2[i];
    }

    // 姿态解算
    float pitch, roll;
    MPU_Calculate_Angles(accel_m_s2[0], accel_m_s2[1], accel_m_s2[2], &x_data, &y_data);
    MPU_Integrate_Yaw(gyro_rad[2], 0.02f); // 假设dt=20ms
}

