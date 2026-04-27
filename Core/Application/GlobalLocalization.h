#ifndef GLOBAL_LOCALIZATION_H
#define GLOBAL_LOCALIZATION_H

#include <stdint.h>
#include <stdbool.h>

// 坐标系参考  ROS/REP-103 标准
// X轴：前进方向为正
// Y轴：左手方向为正
// 航向角yaw：逆时针为正

// 默认初始位置和朝向
#define GLOBAL_INIT_X_MM  (int32_t)180  	// 默认x起始位置（毫米）
#define GLOBAL_INIT_Y_MM  (int32_t)2000  	// 默认y起始位置（毫米）
#define GLOBAL_INIT_X_GRID GLOBAL_INIT_X_MM / 400
#define GLOBAL_INIT_Y_GRID GLOBAL_INIT_Y_MM / 400
#define GLOBAL_INIT_YAW_DEG  (-90.0f) 		// 0度表示出发点方向 逆时针yaw+
// #if defined(LOADER_CAR_3_3)
// #define GLOBAL_INIT_YAW_DEG  (-90.0f) 		// 0度表示出发点方向 逆时针yaw+
// #else
// #define GLOBAL_INIT_YAW_DEG  (0.0f) 		// 0度表示出发点方向 逆时针yaw+
// #endif
// 地图配置 
#define GLOBAL_MAP_X_GRID    8 					// 场地x格子数量
#define GLOBAL_MAP_Y_GRID    10					// 场地y格子数量
#define GLOBAL_MAP_X_MM 	(int32_t)3200  		// 场地高度（毫米） 
#define GLOBAL_MAP_Y_MM 	(int32_t)4000 		// 场地宽度（毫米）
#define GLOBAL_GRID_SIZE_MM 400  // 每个格子400mm

/**
 * @iGuoBao
 * @date 2025.10.12
 * @note
 *     关于相对航向yaw和绝对航向abs_yaw设立的考虑：
 * 		比赛场地是正方形或长方形，但是实际的朝向不一定是完全对齐坐标轴。
 * 		因此需要一个参考航向（相对航向），这个参考航向可以在初始化时设置。
 * 		二维地图xy建立在起始点朝向上。
 */

typedef struct {
	int32_t x_mm; // 毫米为单位的 x 坐标
	int32_t y_mm; // 毫米为单位的 y 坐标
	int32_t x_grid; // 网格坐标x
	int32_t y_grid; // 网格坐标y
	int32_t field_width_mm;  // 场地宽度（毫米）
	int32_t field_height_mm; // 场地高度（毫米）

	float   pitch;  // 俯仰角，度为单位
	float   roll;   // 横滚角，度为单位
	float 	yaw;    // 相对航向，度为单位（-180 ~ 180） 0度出发点方向作为  到时候不同车要进行统一
	float 	abs_yaw; // 绝对航向，度为单位（-180 ~ 180）   将会作为偏值
	int16_t linear_velocity_mm_s; // 线速度 mm/s
	int16_t left_wheel_velocity_mm_s; // 左轮速度 mm/s
	int16_t right_wheel_velocity_mm_s; // 右轮速度 mm/s
	int16_t angular_velocity_deg_s; // 角速度 deg/s
	float   angular_velocity_rad_s; // 角速度 rad/s
	uint8_t seven_data; // 7路传感器数据变量
} GlobalPose_t;  // 4*6 + 8 + 2*4 + 4 + 4 = 52字节

typedef enum {
	GLOBAL_LOC_EXCEPTION_NONE = 0u,
	GLOBAL_LOC_EXCEPTION_YAW_GRID_MISMATCH = (1u << 0),
	GLOBAL_LOC_EXCEPTION_CROSSROAD_STUCK = (1u << 1),
	GLOBAL_LOC_EXCEPTION_LOW_SPEED_CROSSROAD = (1u << 2),
} GlobalLoc_ExceptionFlag_t;

// 初始化模块。参数为场地尺寸（毫米）
void GlobalLoc_Init(void);

// 显式重置位置（单位：毫米，度）
void GlobalLoc_ResetPose(int32_t x_mm, int32_t y_mm, float yaw_deg);

// 周期调用的定位任务
void GlobalLoc_Periodic(void);

// 获取当前估计位姿（值拷贝）
GlobalPose_t GlobalLoc_GetPose(void);

// 获取当前异常情况
uint8_t GlobalLoc_GetException(void);

// 清除异常标志（mask=0xFF 清除全部）
void GlobalLoc_ClearException(uint8_t mask);

#endif // GLOBAL_LOCALIZATION_H
