#ifndef LOADER_STRATEGY_H
#define LOADER_STRATEGY_H

#include "main.h"
#include "astar.h"
#include "translate_route_cmd.h"

#define LOADER_STRATEGY_MAX_PATROL_POINTS 64
#define LOADER_STRATEGY_NON_PATROL_PENALTY_DEFAULT 50
#define LOADER_STRATEGY_MIN_PATROL_STEP_DISTANCE_DEFAULT 2

typedef enum
{
    LOADER_STRATEGY_STATE_PATROL = 0,
    LOADER_STRATEGY_STATE_RETURN_SCORE = 1
} LoaderStrategyState_t;

typedef enum
{
    LOADER_RECOVERY_STATE_IDLE = 0,
    LOADER_RECOVERY_STATE_NAVIGATE = 1,
    LOADER_RECOVERY_STATE_RECOVER_LOCALIZE = 2,
    LOADER_RECOVERY_STATE_RECOVER_TASK = 3,
    LOADER_RECOVERY_STATE_ABORT = 4
} LoaderRecoveryState_t;

/**
 * @brief 初始化铲车策略状态机。
 */
void LoaderStrategy_Init(void);

/**
 * @brief 添加得分区目标栅格点（最多 2 个）。
 * @note 回得分区时会自动选择离当前位置更近的点。
 * @return 1=成功, 0=参数非法或超过上限
 */
uint8_t LoaderStrategy_SetScorePoint(int16_t score_x, int16_t score_y);

/**
 * @brief 设置巡逻路径点列表（按顺序循环）。
 * @return 1=成功, 0=参数非法
 */
uint8_t LoaderStrategy_SetPatrolPoints(const AStar_GridPoint_t *points, uint8_t count);

/**
 * @brief 设置每巡逻多少个点后回得分区。
 * @note points=0 时会自动按 1 处理。
 */
void LoaderStrategy_SetPatrolPointsBeforeReturn(uint8_t points);

/**
 * @brief 兼容旧接口：语义等同于 LoaderStrategy_SetPatrolPointsBeforeReturn。
 */
void LoaderStrategy_SetPatrolRoundsBeforeReturn(uint8_t rounds);

/**
 * @brief 设置巡逻点最小步进距离（曼哈顿距离）。
 * @note 算法会尽量满足该距离；若无可选点会自动放宽约束。
 */
void LoaderStrategy_SetPatrolMinStepDistance(uint8_t min_distance);

/**
 * @brief 设置非巡逻区附加代价。
 * @param penalty 0=关闭该功能，推荐范围 20~120
 */
void LoaderStrategy_SetNonPatrolPenalty(uint8_t penalty);

/**
 * @brief 开关定位异常优先恢复。
 */
void LoaderStrategy_EnableLocalizationRecovery(uint8_t enable);

/**
 * @brief 设置临时障碍衰减周期（策略循环次数）。
 * @note cycles=0 时按 1 处理。
 */
void LoaderStrategy_SetTempObstacleDecayCycles(uint8_t cycles);

/**
 * @brief 获取当前策略状态。
 */
LoaderStrategyState_t LoaderStrategy_GetState(void);

/**
 * @brief 获取最近一次路径转译状态。
 */
TranslateRouteCmd_Status_t LoaderStrategy_GetLastTranslateStatus(void);

/**
 * @brief 获取最近一次恢复阶段。
 */
LoaderRecoveryState_t LoaderStrategy_GetLastRecoveryState(void);

/**
 * @brief 执行一步策略（一次到单个目标点的运行）。
 * @return 1=成功, 0=失败
 */
uint8_t LoaderStrategy_RunOnce(void);

/**
 * @brief 循环运行策略（阻塞）。
 */
void LoaderStrategy_RunLoop(void);

#endif // LOADER_STRATEGY_H
