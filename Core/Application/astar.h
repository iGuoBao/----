#ifndef ASTAR_H__
#define ASTAR_H__

/**
 * @file    astar.h
 * @iGuoBao
 * @date    2025.10.21
 * @note    A* 路径规划算法 - 独立模块
 */

#include "main.h"
#include <stdint.h>
#include "GlobalLocalization.h" // 使用 GlobalPose_t 和地图配置

// #define ASTAR_DEBUG

/* ==================== 地图配置 ==================== */
// 使用 GlobalLocalization.h 中定义的地图参数
#define ASTAR_MAP_WIDTH     GLOBAL_MAP_X_GRID    // 格子
#define ASTAR_MAP_HEIGHT    GLOBAL_MAP_Y_GRID    // 格子
#define ASTAR_MAP_WIDTH_MM  GLOBAL_MAP_X_MM      // 场地大小mm
#define ASTAR_MAP_HEIGHT_MM GLOBAL_MAP_Y_MM      // 场地大小mm

#define MAX_PATH_POINTS 256  // 路径点最大数量

// 计算格子尺寸
#define ASTAR_GRID_SIZE_X_MM  (ASTAR_MAP_WIDTH_MM / ASTAR_MAP_WIDTH)    
#define ASTAR_GRID_SIZE_Y_MM  (ASTAR_MAP_HEIGHT_MM / ASTAR_MAP_HEIGHT)  

/* ==================== 方向定义（对应 astar.c s_directions 索引） */
#define ASTAR_DIR_X_PLUS   0    // (1, 0)
#define ASTAR_DIR_Y_PLUS   1    // (0, 1)
#define ASTAR_DIR_X_MINUS  2    // (-1, 0)
#define ASTAR_DIR_Y_MINUS  3    // (0, -1)
#define ASTAR_DIR_XY_PLUS  4    // (1, 1)
#define ASTAR_DIR_XY_MINUS 5    // (1, -1)
#define ASTAR_DIR_NXY_MINUS 6   // (-1, -1)
#define ASTAR_DIR_NXY_PLUS 7    // (-1, 1)

/* ==================== 代价定义 ==================== */
#define ASTAR_COST_STRAIGHT  10.0f    // 直线移动代价
#define ASTAR_COST_DIAGONAL  14.0f    // 对角线移动代价 (√2 ≈ 1.414)
#define ASTAR_OBSTACLE_COST  255      // 障碍物标记值
#define ASTAR_FREE_COST      0        // 空闲格子标记值

// 墙体惩罚配置（多层递减）
#define ASTAR_WALL_PENALTY_LAYERS   3       // 惩罚层数（1-3层）
#define ASTAR_WALL_PENALTY_BASE     60      // 第1层基础惩罚（靠障碍物最近）
#define ASTAR_WALL_PENALTY_DECAY    0.5f    // 每层递减系数（0.5表示每层减半）

/* ==================== 路径点结构 ==================== */
// 单个路径点 4字节
typedef struct {
    int16_t x_mm;               // X坐标(毫米) 
    int16_t y_mm;               // Y坐标(毫米) 
} PathPoint_t; // 4字节

/* ==================== 路径结构 ==================== */
// 路径结构体
typedef struct {
    uint16_t point_count;                   // 路径点数量（改为uint16_t，支持最多65535个点）
    uint16_t current_index;                 // 当前追踪的路径点索引（改为uint16_t）
    uint8_t is_valid;                       // 路径是否有效 (0=无效, 1=有效)
    uint8_t is_finished;                    // 路径是否完成 (0=未完成, 1=完成)
    PathPoint_t points[MAX_PATH_POINTS];    // 路径点数组
} Path_t;

/* ==================== 网格点结构 ==================== */
typedef struct {
    int8_t x;  // 网格 X 坐标 (0~31)
    int8_t y;  // 网格 Y 坐标 (0~31)
    uint8_t padding[2]; // 填充字节以对齐结构体大小为4字节
} AStar_GridPoint_t; // 4字节

/* ==================== A* 节点结构（优化版，节省内存）==================== */
typedef struct {
    int8_t parent_x;        // 父节点 X 坐标
    int8_t parent_y;        // 父节点 Y 坐标
    uint8_t flags;           // bit0=in_open_list, bit1=in_closed_list
    uint8_t padding;    // 填充字节以对齐结构体大小为8字节
    float g;                 // 从起点到当前点的实际代价
} AStar_Node_t;  // 8字节

/* ==================== 地图结构 ==================== */
typedef struct {
    uint8_t is_initialized;                                // 地图是否已初始化
    uint8_t padding[7];                                    // 填充
    uint8_t grid[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH];       // 64kb栅格地图 (0=空闲, 255=障碍物)
    AStar_Node_t nodes[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH]; // 每个格子对应的A*节点信息
} AStar_Map_t;  // 约9KB（32x32地图）
// 栅格地图32*32      地图精度94mm/格子  足够
// 1+7+32*32 + 8*32*32 = 9,216字节 约等于9KB
// 假设栅格地图48*48   地图精度63mm/格子
// 1+7 + 48*48 + 8*48*48 = 20,865字节  约等于20KB
// 假设栅格地图64*64   地图精度47mm/格子
// 1+7 + 64*64 + 8*64*64 = 36,865字节  约等于36KB
// 栅格地图n*m
// 1+7 + n*m + 8*n*m = 1+7 + 9*n*m = 8 + 9*n*m 字节

/* ==================== 公共函数声明 ==================== */

/**
 * @brief 初始化 A* 地图
 * @note 清空地图，所有格子设为空闲
 */
void AStar_Init(void);

/**
 * @brief 设置障碍物（单个格子）
 * @param grid_x 网格 X 坐标 (0~31)
 * @param grid_y 网格 Y 坐标 (0~31)
 */
void AStar_SetObstacle(int16_t grid_x, int16_t grid_y);

/**
 * @brief 设置障碍物（矩形区域）- 网格坐标版本
 * @param x1 起始网格 X 坐标 (0~31)
 * @param y1 起始网格 Y 坐标 (0~31)
 * @param x2 结束网格 X 坐标 (0~31)
 * @param y2 结束网格 Y 坐标 (0~31)
 * @note 参数为网格坐标，不是世界坐标(mm)
 */
void AStar_SetObstacleRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

/**
 * @brief 设置障碍物（矩形区域）- 世界坐标版本
 * @param x1_mm 起始世界 X 坐标 (mm)
 * @param y1_mm 起始世界 Y 坐标 (mm)
 * @param x2_mm 结束世界 X 坐标 (mm)
 * @param y2_mm 结束世界 Y 坐标 (mm)
 * @note 参数为世界坐标(mm)，会自动转换为网格坐标
 */
void AStar_SetObstacleRect_mm(int16_t x1_mm, int16_t y1_mm, int16_t x2_mm, int16_t y2_mm);

/**
 * @brief 设置障碍物（矩形区域，对角线连接）- 世界坐标版本
 * @param x1_mm 
 * @param y1_mm 
 * @param x2_mm 
 * @param y2_mm 
 * @note 参数为世界坐标(mm)，会自动转换为网格坐标
 *       专门应对 斜着的矩形障碍物
 */
void AStar_SetObstacleRect_mm_diagonal(int16_t x1_mm, int16_t y1_mm, int16_t x2_mm, int16_t y2_mm);
/**
 * @brief 清除障碍物（单个格子）
 * @param grid_x 网格 X 坐标
 * @param grid_y 网格 Y 坐标
 */
void AStar_ClearObstacle(int16_t grid_x, int16_t grid_y);

/**
 * @brief 清空整个地图
 */
void AStar_ClearMap(void);

/**
 * @brief A* 路径搜索（核心函数）
 * @param start_mm 起点世界坐标 (mm)
 * @param goal_mm 终点世界坐标 (mm)
 * @param path 输出路径指针
 * @return 1=成功找到路径, 0=失败
 * @note 输出的路径已经是世界坐标(mm)，可直接用于路径追踪
 */
uint8_t AStar_FindPath(PathPoint_t start_mm, PathPoint_t goal_mm, Path_t *path);

/**
 * @brief 获取地图指针（调试用）
 * @return 地图指针
 */
AStar_Map_t* AStar_GetMap(void);


/**
 * @brief 打印地图到串口1（调试用）
 * @note  打印格式：hex
 *        直接从x=0,y=0开始逐行打印
 *        先x++，再y++
 *        直接打印每个栅格的16进制值
 */
void AStar_PrintMap(void);

/**
 * @brief 重置地图到初始固定障碍物的地图快照
 * @note  1. 清空所有障碍物
 *        2. 再调用固定好的障碍物设置函数
 */
void AStar_ResetToInitialObstacles(void);

/**
 * @brief 设置单向边阻断
 * @param grid_x 起点格子X
 * @param grid_y 起点格子Y
 * @param direction 方向编码 ASTAR_DIR_*
 * @param blocked 1=禁止从该点朝该方向移动, 0=允许
 */
void AStar_SetEdgeBlocked(int16_t grid_x, int16_t grid_y, uint8_t direction, uint8_t blocked);

/**
 * @brief 设置从一个节点到另一个节点的边阻断（自动推断方向）
 * @param x1 起点X
 * @param y1 起点Y
 * @param x2 终点X
 * @param y2 终点Y
 * @param blocked 1=禁止该转移, 0=允许
 */
void AStar_SetEdgeBlockedTo(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t blocked);

/**
 * @brief 应用墙体惩罚（多层递减惩罚机制）
 * @note  调用此函数后，靠近墙体的格子会根据距离应用不同的惩罚代价：
 *        - 第1层（紧邻障碍物）：ASTAR_WALL_PENALTY_BASE (60)
 *        - 第2层：ASTAR_WALL_PENALTY_BASE * DECAY (30)
 *        - 第3层：ASTAR_WALL_PENALTY_BASE * DECAY^2 (15)
 *        这样小车会优先选择远离墙体的路径，但必要时仍可通过
 *        建议在设置完所有障碍物后调用
 */
void AStar_ApplyWallPenalty(void);

#endif  // ASTAR_H__
