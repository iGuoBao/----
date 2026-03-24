#include "mpu.h"
#include "math.h"




void Get_Sensor_Data(float *yaw, float *pitch, float *roll) {
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    
    // 读取FIFO数据（需在中断中调用）
    if (dmp_read_fifo(NULL, NULL, quat, &timestamp, &sensors, &more) == 0) {
        if (sensors & INV_WXYZ_QUAT) {  // 校验四元数有效性[7](@ref)
            // 转换为浮点数（Q30格式转换）
            float q0 = quat[0] / 1073741824.0f;
            float q1 = quat[1] / 1073741824.0f;
            float q2 = quat[2] / 1073741824.0f;
            float q3 = quat[3] / 1073741824.0f;
            
            // 四元数转欧拉角
            *yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 57.3f;
            *pitch = asin(2*(q0*q2 - q3*q1)) * 57.3f;
            *roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.3f;
        }
    }
}