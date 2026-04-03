#ifndef TRANSLATE_ROUTE_CMD_H
#define TRANSLATE_ROUTE_CMD_H

#include "main.h"

#define TRANSLATE_ROUTE_CMD_MAX_SPECIAL_EDGES 32

typedef enum {
    TRANSLATE_ROUTE_CMD_OK = 0,
    TRANSLATE_ROUTE_CMD_ERR_PARAM,
    TRANSLATE_ROUTE_CMD_ERR_NO_PATH,
    TRANSLATE_ROUTE_CMD_ERR_YAW,
    TRANSLATE_ROUTE_CMD_ERR_STEP,
    TRANSLATE_ROUTE_CMD_ERR_BUFFER,
    TRANSLATE_ROUTE_CMD_ERR_ZERO_FORWARD,
    TRANSLATE_ROUTE_CMD_ERR_RULE_FULL
} TranslateRouteCmd_Status_t;

typedef enum {
    TRANSLATE_ROUTE_INTENT_NONE = 0,        // 无特殊意图，正常行驶
    TRANSLATE_ROUTE_INTENT_PICK_CUBE = 1,   // 拾取方块
    TRANSLATE_ROUTE_INTENT_PICK_RING = 2,   // 拾取圆环
    TRANSLATE_ROUTE_INTENT_PLACE_RING = 3,  // 放置圆环
    TRANSLATE_ROUTE_INTENT_PLACE_CUBE = 4   // 放置方块
} TranslateRouteCmd_Intent_t;

/**
 * @brief 从当前定位位姿出发，生成到目标栅格的命令串。
 * @param goal_x 目标栅格 x
 * @param goal_y 目标栅格 y
 * @param out_cmd 输出命令串缓冲区
 * @param out_cmd_size 缓冲区大小（含 '\0'）
 * @return 状态码
 * @note 输出命令仅包含 L/R/A/1~9，不自动追加 S。
 */
TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateToGoal(int16_t goal_x,
                                                            int16_t goal_y,
                                                            char *out_cmd,
                                                            uint16_t out_cmd_size);

/**
 * @brief 从指定起点与航向出发，生成到目标栅格的命令串（便于离线测试）。
 */
TranslateRouteCmd_Status_t TranslateRouteCmd_Generate(int16_t start_x,
                                                      int16_t start_y,
                                                      float start_yaw_deg,
                                                      int16_t goal_x,
                                                      int16_t goal_y,
                                                      char *out_cmd,
                                                      uint16_t out_cmd_size);

/**
 * @brief 从当前定位位姿出发，生成到目标栅格的命令串，并根据意图追加尾部动作。
 */
TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateToGoalWithIntent(int16_t goal_x,
                                                                      int16_t goal_y,
                                                                      TranslateRouteCmd_Intent_t intent,
                                                                      char *out_cmd,
                                                                      uint16_t out_cmd_size);

/**
 * @brief 从指定起点与航向出发，生成命令串，并根据意图追加尾部动作（便于离线测试）。
 */
TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateWithIntent(int16_t start_x,
                                                                int16_t start_y,
                                                                float start_yaw_deg,
                                                                int16_t goal_x,
                                                                int16_t goal_y,
                                                                TranslateRouteCmd_Intent_t intent,
                                                                char *out_cmd,
                                                                uint16_t out_cmd_size);

/**
 * @brief 添加特殊边的前进计数补偿。
 * @param from_x 起点格子 x
 * @param from_y 起点格子 y
 * @param to_x   终点格子 x
 * @param to_y   终点格子 y
 * @param route_units 该边对应的前进计数，允许 0（表示不计入 route 前进数）
 * @return 状态码
 * @note 规则是有向边，若反向也特殊，需要再添加一次。
 */
TranslateRouteCmd_Status_t TranslateRouteCmd_AddEdgeRouteUnits(int16_t from_x,
                                                               int16_t from_y,
                                                               int16_t to_x,
                                                               int16_t to_y,
                                                               uint8_t route_units);

/**
 * @brief 设置赛道的前进计数补偿规则
 * @note
 *   斜坡上有些没十字路口的，做标记
 */
void TranslateRouteCmd_SetSpecialEdgeRules(void);

/**
 * @brief 清空所有特殊边补偿规则。
 */
void TranslateRouteCmd_ClearEdgeRouteUnits(void);

/**
 * @brief 获取状态码对应文本，便于日志打印。
 */
const char *TranslateRouteCmd_StatusString(TranslateRouteCmd_Status_t status);

#endif
