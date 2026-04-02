/**
 * @file    astar.c
 * @iGuoBao
 * @date    2025.10.21
 * @note    A* 路径规划算法实现 - 内存优化版
 *          - 优化节点结构，减少50%内存占用
 *          - 32×32网格（约94mm/格），RAM占用约15KB
 *          - STM32F103RC (64KB RAM) 安全可用
 */

#include "astar.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ==================== 私有宏定义 ==================== */
#define FLAG_IN_OPEN_LIST   0x01
#define FLAG_IN_CLOSED_LIST 0x02

/* ==================== 私有变量 ==================== */
static AStar_Map_t s_map;                           // 全局地图
static uint8_t s_edge_block[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH]; // 每个格子边阻断掩码，bit0..bit7对应方向
static AStar_GridPoint_t s_open_list[512];          // 开放列表（固定大小）
static uint16_t s_open_count = 0;                   // 开放列表中的节点数量

/* ==================== 私有函数声明 ==================== */
static float heuristic(AStar_GridPoint_t a, AStar_GridPoint_t b);
static uint8_t is_valid_grid(int16_t x, int16_t y);
static uint8_t is_obstacle(int16_t x, int16_t y);
static uint8_t is_edge_blocked(int16_t x, int16_t y, uint8_t direction);
static void set_edge_blocked(int16_t x, int16_t y, uint8_t direction, uint8_t blocked);
static void add_to_open_list(AStar_GridPoint_t point);
static AStar_GridPoint_t get_lowest_f_node(AStar_GridPoint_t goal);
static void reconstruct_path(AStar_GridPoint_t start, AStar_GridPoint_t goal, Path_t *path);
static void grid_to_world(AStar_GridPoint_t grid, PathPoint_t *world);
static void world_to_grid(PathPoint_t world, AStar_GridPoint_t *grid);
static void AStar_ClearObstacles(void);
static void AStar_SetStaticObstacles(void);

/* ==================== 8方向移动 ====================
 * 注意：grid[row][col] 其中 row=Y轴索引(向左), col=X轴索引(向前)
 * GridPoint中：.x对应X轴(向前)，.y对应Y轴(向左)
 * s_directions[i][0]加到x上 → 影响列索引 → X轴移动
 * s_directions[i][1]加到y上 → 影响行索引 → Y轴移动
 */
static const int16_t s_directions[4][2] = {
    {1, 0},   // X+方向：向前（列+1，行不变）
    {0, 1},   // Y+方向：向左（列不变，行+1）
    {-1, 0},  // X-方向：向后（列-1，行不变）
    {0, -1}   // Y-方向：向右（列不变，行-1）
};

static const uint8_t s_opposite_direction[4] = {
    ASTAR_DIR_X_MINUS,    // 0 -> 2
    ASTAR_DIR_Y_MINUS,    // 1 -> 3
    ASTAR_DIR_X_PLUS,     // 2 -> 0
    ASTAR_DIR_Y_PLUS      // 3 -> 1
};

/* ==================== 公共函数实现 ==================== */

/**
 * @brief 初始化 A* 地图
 */
void AStar_Init(void)
{
    memset(&s_map, 0, sizeof(AStar_Map_t));
    memset(&s_edge_block, 0, sizeof(s_edge_block)); // 清除所有边阻断
    
    // 清空地图，所有格子设为空闲
    for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++) {
        for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++) {
            s_map.grid[y][x] = ASTAR_FREE_COST;
        }
    }

    AStar_ResetToInitialObstacles();
    
    s_map.is_initialized = 1;
}

/**
 * @brief 设置障碍物（单个格子）
 */
void AStar_SetObstacle(int16_t grid_x, int16_t grid_y)
{
    if (!is_valid_grid(grid_x, grid_y)) {
        return;
    }
    s_map.grid[grid_y][grid_x] = ASTAR_OBSTACLE_COST;
}

/**
 * @brief 设置障碍物（矩形区域）- 网格坐标版本
 */
void AStar_SetObstacleRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    for (int16_t y = y1; y <= y2; y++) {
        for (int16_t x = x1; x <= x2; x++) {
            AStar_SetObstacle(x, y);
        }
    }
}

/**
 * @brief 设置障碍物（矩形区域）- 世界坐标版本
 */
void AStar_SetObstacleRect_mm(int16_t x1_mm, int16_t y1_mm, int16_t x2_mm, int16_t y2_mm)
{
    // 转换世界坐标到网格坐标
    PathPoint_t p1 = {x1_mm, y1_mm};
    PathPoint_t p2 = {x2_mm, y2_mm};
    AStar_GridPoint_t g1, g2;
    
    world_to_grid(p1, &g1);
    world_to_grid(p2, &g2);
    
    // 调用网格坐标版本
    AStar_SetObstacleRect(g1.x, g1.y, g2.x, g2.y);
}

/**
 * @brief 设置障碍物 处理输入是斜着的矩形
 * @note 
 *          因为有时候小车认为的障碍物 在地图上是斜着的矩形
 *          需要把斜着的矩形 转换为多个网格坐标的障碍物 
 *          输入的两个点 代表矩形的对角点
 *          优化策略：
 *          - 只有当矩形覆盖格子面积 >= 50% 时才标记为障碍物
 *          - 保证至少标记一个格子（选择相交面积最大的）
 * @param x1_mm 矩形一个点
 * @param y1_mm 矩形一个点
 * @param x2_mm 矩形另一个点
 * @param y2_mm 矩形另一个点
 */
void AStar_SetObstacleRect_mm_diagonal(int16_t x1_mm, int16_t y1_mm, int16_t x2_mm, int16_t y2_mm)
{
    // 确保矩形坐标顺序正确（左下到右上）
    int16_t rect_x_min = (x1_mm < x2_mm) ? x1_mm : x2_mm;
    int16_t rect_x_max = (x1_mm > x2_mm) ? x1_mm : x2_mm;
    int16_t rect_y_min = (y1_mm < y2_mm) ? y1_mm : y2_mm;
    int16_t rect_y_max = (y1_mm > y2_mm) ? y1_mm : y2_mm;

    // 计算矩形可能覆盖的网格范围
    PathPoint_t p_min = {rect_x_min, rect_y_min};
    PathPoint_t p_max = {rect_x_max, rect_y_max};
    AStar_GridPoint_t g_min, g_max;
    world_to_grid(p_min, &g_min);
    world_to_grid(p_max, &g_max);

    // 计算每个格子的物理尺寸
    float grid_width_mm = (float)ASTAR_MAP_WIDTH_MM / ASTAR_MAP_WIDTH;
    float grid_height_mm = (float)ASTAR_MAP_HEIGHT_MM / ASTAR_MAP_HEIGHT;
    float grid_area = grid_width_mm * grid_height_mm;
    float threshold_area = grid_area * 0.5f; // 50% 阈值

    // 记录所有相交面积
    float max_intersection_area = 0.0f;
    int16_t max_grid_x = -1;
    int16_t max_grid_y = -1;

    // 遍历可能相交的所有格子
    for (int16_t gy = g_min.y; gy <= g_max.y; gy++) {
        for (int16_t gx = g_min.x; gx <= g_max.x; gx++) {
            if (!is_valid_grid(gx, gy)) {
                continue;
            }

            // 计算当前格子的世界坐标边界
            float cell_x_min = gx * grid_width_mm;
            float cell_x_max = (gx + 1) * grid_width_mm;
            float cell_y_min = gy * grid_height_mm;
            float cell_y_max = (gy + 1) * grid_height_mm;

            // 计算矩形与格子的相交区域
            float intersect_x_min = (rect_x_min > cell_x_min) ? rect_x_min : cell_x_min;
            float intersect_x_max = (rect_x_max < cell_x_max) ? rect_x_max : cell_x_max;
            float intersect_y_min = (rect_y_min > cell_y_min) ? rect_y_min : cell_y_min;
            float intersect_y_max = (rect_y_max < cell_y_max) ? rect_y_max : cell_y_max;

            // 检查是否有相交
            if (intersect_x_min < intersect_x_max && intersect_y_min < intersect_y_max) {
                // 计算相交面积
                float intersect_area = (intersect_x_max - intersect_x_min) * 
                                       (intersect_y_max - intersect_y_min);

                // 更新最大相交面积
                if (intersect_area > max_intersection_area) {
                    max_intersection_area = intersect_area;
                    max_grid_x = gx;
                    max_grid_y = gy;
                }

                // 如果相交面积超过 50%，标记为障碍物
                if (intersect_area >= threshold_area) {
                    AStar_SetObstacle(gx, gy);
                }
            }
        }
    }

    // 确保至少标记一个格子（面积最大的）
    if (max_grid_x >= 0 && max_grid_y >= 0) {
        AStar_SetObstacle(max_grid_x, max_grid_y);
    }
}

/**
 * @brief 清除障碍物
 */
void AStar_ClearObstacle(int16_t grid_x, int16_t grid_y)
{
    if (!is_valid_grid(grid_x, grid_y)) {
        return;
    }
    s_map.grid[grid_y][grid_x] = ASTAR_FREE_COST;
}

/**
 * @brief 清空整个地图
 */
void AStar_ClearMap(void)
{
    for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++) {
        for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++) {
            s_map.grid[y][x] = ASTAR_FREE_COST;
        }
    }
}


/**
 * @brief A* 路径搜索核心函数
 * @return 1=成功找到路径,  0=失败
 */
uint8_t AStar_FindPath(PathPoint_t start_mm, PathPoint_t goal_mm, Path_t *path)
{
    if (path == NULL || !s_map.is_initialized) {
        return 0;
    }
    
    // 转换为网格坐标
    AStar_GridPoint_t start, goal;
    world_to_grid(start_mm, &start);
    world_to_grid(goal_mm, &goal);
    
    // 检查起点和终点是否有效
    // 会是潜在隐患，未来可能有小车进入从而被比较为障碍物 然后小车因为被判断在障碍物里而直接return 0
    if (is_obstacle(start.x, start.y) || is_obstacle(goal.x, goal.y)) {
        #ifdef ASTAR_DEBUG
        char __buf[48];
        // sofERROR
        sprintf(__buf, "sofEROR");
        OLED_Clear();
        OLED_ShowString(0, 0, __buf, OLED_8X16);
        OLED_Update();
        HAL_Delay(3000);
        #endif
        return 0;  // 起点或终点是障碍物
    }
    
    // 初始化所有节点
    for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++) {
        for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++) {
            s_map.nodes[y][x].parent_x = -1;
            s_map.nodes[y][x].parent_y = -1;
            s_map.nodes[y][x].g = 1e9f;  // 无穷大
            s_map.nodes[y][x].flags = 0;
        }
    }
    
    // 初始化起点
    s_map.nodes[start.y][start.x].g = 0.0f;
    
    // 清空开放列表并添加起点
    s_open_count = 0;
    add_to_open_list(start);
    
    // A* 主循环
    while (s_open_count > 0) {
        // 从开放列表中取出 f 值最小的节点
        AStar_GridPoint_t current = get_lowest_f_node(goal);
        
        // 如果到达终点
        if (current.x == goal.x && current.y == goal.y) {
            reconstruct_path(start, goal, path);
            return 1;
        }
        
        // 将当前节点加入关闭列表
        s_map.nodes[current.y][current.x].flags |= FLAG_IN_CLOSED_LIST;
        
// 遍历4个方向的邻居（不走对角）
        for (uint8_t i = 0; i < 4; i++) {
            int16_t nx = current.x + s_directions[i][0];
            int16_t ny = current.y + s_directions[i][1];
            
            // 检查邻居是否有效
            if (!is_valid_grid(nx, ny) || is_obstacle(nx, ny)) {
                continue;
            }

            // 检查该边是否被设置为阻断（按方向）
            if (is_edge_blocked(current.x, current.y, i)) {
                continue;
            }
            
            // 如果已在关闭列表中，跳过
            if (s_map.nodes[ny][nx].flags & FLAG_IN_CLOSED_LIST) {
                continue;
            }
            
            // 计算移动代价（只有直线4方向）
            float move_cost = ASTAR_COST_STRAIGHT;
            
            // 添加格子本身的代价（墙体惩罚）
            float grid_cost_weight = s_map.grid[ny][nx] / 10.0f;  // 将代价转换为权重系数
            float tentative_g = s_map.nodes[current.y][current.x].g + move_cost + grid_cost_weight;
            
            // 如果找到更好的路径
            if (tentative_g < s_map.nodes[ny][nx].g) {
                s_map.nodes[ny][nx].parent_x = current.x;
                s_map.nodes[ny][nx].parent_y = current.y;
                s_map.nodes[ny][nx].g = tentative_g;
                
                // 如果不在开放列表中，加入
                if (!(s_map.nodes[ny][nx].flags & FLAG_IN_OPEN_LIST)) {
                    AStar_GridPoint_t neighbor = {nx, ny};
                    add_to_open_list(neighbor);
                }
            }
        }
    }
    
    // 未找到路径
    return 0;
}

/**
 * @brief 获取地图指针
 */
AStar_Map_t* AStar_GetMap(void)
{
    return &s_map;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 启发式函数（对角距离）
 * @note 允许8方向移动时，对角距离更准确
 */
static float heuristic(AStar_GridPoint_t a, AStar_GridPoint_t b)
{
    int16_t dx = abs(a.x - b.x);
    int16_t dy = abs(a.y - b.y);

    // 仅4方向可行，使用曼哈顿距离
    return ASTAR_COST_STRAIGHT * (dx + dy);
}

/**
 * @brief 检查网格坐标是否有效
 */
static uint8_t is_valid_grid(int16_t x, int16_t y)
{
    return (x >= 0 && x < ASTAR_MAP_WIDTH && y >= 0 && y < ASTAR_MAP_HEIGHT);
}

/**
 * @brief 检查是否是障碍物
 */
static uint8_t is_obstacle(int16_t x, int16_t y)
{
    if (!is_valid_grid(x, y)) {
        return 1;  // 出界视为障碍物
    }
    return (s_map.grid[y][x] == ASTAR_OBSTACLE_COST);
}

/**
 * @brief 添加到开放列表
 */
static void add_to_open_list(AStar_GridPoint_t point)
{
    if (s_open_count >= 512) {
        return;  // 列表已满
    }
    
    s_open_list[s_open_count] = point;
    s_map.nodes[point.y][point.x].flags |= FLAG_IN_OPEN_LIST;
    s_open_count++;
}

/**
 * @brief 从开放列表中取出 f 值最小的节点
 * @note 动态计算 f = g + h，不存储
 */
static AStar_GridPoint_t get_lowest_f_node(AStar_GridPoint_t goal)
{
    if (s_open_count == 0) {
        AStar_GridPoint_t invalid = {-1, -1};
        return invalid;
    }
    
    // 找到 f 值最小的节点
    uint16_t min_idx = 0;
    AStar_GridPoint_t p0 = s_open_list[0];
    float min_f = s_map.nodes[p0.y][p0.x].g + heuristic(p0, goal);
    
    for (uint16_t i = 1; i < s_open_count; i++) {
        AStar_GridPoint_t p = s_open_list[i];
        float f = s_map.nodes[p.y][p.x].g + heuristic(p, goal);
        if (f < min_f) {
            min_f = f;
            min_idx = i;
        }
    }
    
    // 从开放列表中移除
    AStar_GridPoint_t result = s_open_list[min_idx];
    s_map.nodes[result.y][result.x].flags &= ~FLAG_IN_OPEN_LIST;
    
    // 用最后一个元素填补空缺
    s_open_list[min_idx] = s_open_list[s_open_count - 1];
    s_open_count--;
    
    return result;
}

/**
 * @brief 重建路径（从终点回溯到起点）
 * @note 输出世界坐标 x_mm, y_mm
 */
static void reconstruct_path(AStar_GridPoint_t start, AStar_GridPoint_t goal, Path_t *path)
{
    memset(path, 0, sizeof(Path_t));
    
    // 从终点回溯到起点
    AStar_GridPoint_t current = goal;
    AStar_GridPoint_t temp_path[MAX_PATH_POINTS];
    uint16_t count = 0;
    
    while (current.x != start.x || current.y != start.y) {
        if (count >= MAX_PATH_POINTS) {
            break;  // 路径太长
        }
        
        temp_path[count] = current;
        count++;
        
        // 回溯到父节点
        int16_t parent_x = s_map.nodes[current.y][current.x].parent_x;
        int16_t parent_y = s_map.nodes[current.y][current.x].parent_y;
        
        // 防止死循环
        if (parent_x == -1 || parent_y == -1) {
            break;
        }
        
        current.x = parent_x;
        current.y = parent_y;
    }
    
    // 添加起点
    if (count < MAX_PATH_POINTS) {
        temp_path[count] = start;
        count++;
    }
    
    // 反转路径（从起点到终点）并转换为世界坐标
    for (uint16_t i = 0; i < count; i++) {
        AStar_GridPoint_t grid = temp_path[count - 1 - i];
        grid_to_world(grid, &path->points[i]);
    }
    
    path->point_count = count;
    path->current_index = 0;
    path->is_valid = 1;
    path->is_finished = 0;
}

/**
 * @brief 网格坐标转世界坐标
 * @note 转换到格子中心点
 */
static void grid_to_world(AStar_GridPoint_t grid, PathPoint_t *world)
{
    // 更精确的计算，转换为int16_t
    world->x_mm = (int16_t)((grid.x * ASTAR_MAP_WIDTH_MM) / ASTAR_MAP_WIDTH + 
                             ASTAR_MAP_WIDTH_MM / (2 * ASTAR_MAP_WIDTH));
    world->y_mm = (int16_t)((grid.y * ASTAR_MAP_HEIGHT_MM) / ASTAR_MAP_HEIGHT + 
                             ASTAR_MAP_HEIGHT_MM / (2 * ASTAR_MAP_HEIGHT));
}

/**
 * @brief 世界坐标转网格坐标
 * @note world.x_mm对应X轴(向前) → grid.x
 *       world.y_mm对应Y轴(向左) → grid.y
 */
static void world_to_grid(PathPoint_t world, AStar_GridPoint_t *grid)
{
    // world.x_mm → grid.x (X轴，向前)
    grid->x = (int16_t)((world.x_mm * ASTAR_MAP_WIDTH) / ASTAR_MAP_WIDTH_MM);
    // world.y_mm → grid.y (Y轴，向左)
    grid->y = (int16_t)((world.y_mm * ASTAR_MAP_HEIGHT) / ASTAR_MAP_HEIGHT_MM);
    
    // 限制在有效范围内
    if (grid->x < 0) grid->x = 0;
    if (grid->x >= ASTAR_MAP_WIDTH) grid->x = ASTAR_MAP_WIDTH - 1;
    if (grid->y < 0) grid->y = 0;
    if (grid->y >= ASTAR_MAP_HEIGHT) grid->y = ASTAR_MAP_HEIGHT - 1;
}

/**
 * @brief 打印地图到串口1（调试用）
 * @note  数组布局：s_map.grid[row][col]，row=Y轴索引(0~47)，col=X轴索引(0~47)
 *        物理坐标：X轴向前，Y轴向左（ROS标准）
 *        
 *        打印顺序：外循环X轴(0~47)，内循环Y轴(0~47)
 *        每行代表固定X值，Y从0到47
 *        第1行：X=0, Y=0~47 → grid[0][0], grid[1][0], ..., grid[47][0] + 换行
 *        第2行：X=1, Y=0~47 → grid[0][1], grid[1][1], ..., grid[47][1] + 换行
 *        ...
 *        第48行：X=47, Y=0~47 → grid[0][47], grid[1][47], ..., grid[47][47] + 换行
 */
void AStar_PrintMap(void)
{
    // 遍历X轴（列索引，向前方向，0到47）
    for (int16_t col = 0; col < ASTAR_MAP_WIDTH; col++) {
        // 遍历Y轴（行索引，向左方向，0到47）
        for (int16_t row = 0; row < ASTAR_MAP_HEIGHT; row++) {
            uint8_t value = s_map.grid[row][col];  // grid[Y索引][X索引]
            HAL_UART_Transmit(&huart1, &value, 1, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)"\n", 1, HAL_MAX_DELAY);
    }
}

/**
 * @iGuoBao
 * @brief 重新回到初始永久障碍物地图
 * @note  1. 先清空所有
 *        2. 再调用固定好的障碍物设置函数
 */
void AStar_ResetToInitialObstacles(void)
{
    AStar_ClearObstacles();
    AStar_SetStaticObstacles();
    // AStar_ApplyWallPenalty();
}

/**
 * @iGuoBao
 * @brief 清空所有障碍物
 * @note  用于重置地图
 */
static void AStar_ClearObstacles(void)
{
    for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++) {
        for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++) {
            s_map.grid[y][x] = ASTAR_FREE_COST;
        }
    }
}

/**
 * @brief 应用墙体惩罚（多层递减惩罚机制）
 * @note  使用多层扩散算法，根据到最近障碍物的距离应用递减惩罚
 *        层数越多，惩罚越低，形成平滑的避障区域
 */
void AStar_ApplyWallPenalty(void)
{
    // 8方向偏移量（包括对角线）
    static const int16_t offsets[8][2] = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1},
        {1, 1}, {1, -1}, {-1, -1}, {-1, 1}
    };
    
    // 多层扩散：从障碍物向外扩散多层
    for (uint8_t layer = 1; layer <= ASTAR_WALL_PENALTY_LAYERS; layer++) {
        // 计算当前层的惩罚代价（递减）
        float decay_factor = 1.0f;
        for (uint8_t i = 1; i < layer; i++) {
            decay_factor *= ASTAR_WALL_PENALTY_DECAY;
        }
        uint8_t current_penalty = (uint8_t)(ASTAR_WALL_PENALTY_BASE * decay_factor);
        
        // 如果惩罚值太小（<5），跳过这一层
        if (current_penalty < 5) {
            break;
        }
        
        // 遍历所有格子，寻找需要标记的目标
        for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++) {
            for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++) {
                // 检查当前格子是否是上一层的标记（或障碍物）
                uint8_t is_source = 0;
                if (layer == 1) {
                    // 第1层：从障碍物扩散
                    is_source = (s_map.grid[y][x] == ASTAR_OBSTACLE_COST);
                } else {
                    // 后续层：从上一层的惩罚格子扩散
                    uint8_t prev_penalty = (uint8_t)(ASTAR_WALL_PENALTY_BASE * decay_factor / ASTAR_WALL_PENALTY_DECAY);
                    is_source = (s_map.grid[y][x] == prev_penalty || s_map.grid[y][x] == ASTAR_OBSTACLE_COST);
                }
                
                if (is_source) {
                    // 检查8个方向的邻居
                    for (uint8_t i = 0; i < 8; i++) {
                        int16_t nx = x + offsets[i][0];
                        int16_t ny = y + offsets[i][1];
                        
                        // 确保邻居在有效范围内
                        if (is_valid_grid(nx, ny)) {
                            // 只对空闲格子应用惩罚，不覆盖已有的障碍物或更高惩罚
                            if (s_map.grid[ny][nx] == ASTAR_FREE_COST) {
                                s_map.grid[ny][nx] = current_penalty;
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 *  @brief 静态设置设定的障碍物
 *  @note  根据赛道 写死障碍物位置 一个个加
 */
static uint8_t is_edge_blocked(int16_t x, int16_t y, uint8_t direction)
{
    if (!is_valid_grid(x, y) || direction >= 8) {
        return 1;
    }
    return (s_edge_block[y][x] & (1u << direction)) != 0;
}

static void set_edge_blocked(int16_t x, int16_t y, uint8_t direction, uint8_t blocked)
{
    if (!is_valid_grid(x, y) || direction >= 8) {
        return;
    }
    if (blocked) {
        s_edge_block[y][x] |= (1u << direction);
    } else {
        s_edge_block[y][x] &= ~(1u << direction);
    }
}

void AStar_SetEdgeBlocked(int16_t grid_x, int16_t grid_y, uint8_t direction, uint8_t blocked)
{
    set_edge_blocked(grid_x, grid_y, direction, blocked);
}

void AStar_SetEdgeBlockedTo(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t blocked)
{
    if (!is_valid_grid(x1, y1) || !is_valid_grid(x2, y2)) {
        return;
    }

    int8_t dx = x2 - x1;
    int8_t dy = y2 - y1;
    uint8_t dir = 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        if (s_directions[i][0] == dx && s_directions[i][1] == dy) {
            dir = i;
            break;
        }
    }

    if (dir == 0xFF) {
        return;
    }

    set_edge_blocked(x1, y1, dir, blocked);
    if (!blocked) {
        set_edge_blocked(x2, y2, s_opposite_direction[dir], 0);
    }
}

static void AStar_SetStaticObstacles(void)
{  
    // 赛道标准ros 坐标系：X轴向前，Y轴向左 地图右下角是0,0
    // 示例 例如己方满仓环位置是2,5 己方三分区4，9
    // 单边封锁
    // 示例 AStar_SetEdgeBlockedTo(10, 10, 11, 10, 1);
     // 封锁 (10,10) 到 (11,10) 的单边通行 第五个形参1就是阻止
    // 启动坡道右侧
    AStar_SetEdgeBlockedTo(1, 4, 2, 4, 1);
    AStar_SetEdgeBlockedTo(2, 4, 1, 4, 1);
    AStar_SetEdgeBlockedTo(2, 4, 3, 4, 1);
    AStar_SetEdgeBlockedTo(3, 4, 2, 4, 1);
}
