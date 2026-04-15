#include "shovel_strategy.h"

#include "action.h"
#include "GlobalLocalization.h"

#include <string.h>

typedef struct
{
    AStar_GridPoint_t patrol_points[SHOVEL_STRATEGY_MAX_PATROL_POINTS];
    uint8_t patrol_count;
    uint8_t patrol_points_before_return;
    uint8_t patrol_points_done_since_return;
    uint8_t min_patrol_step_distance;
    int8_t last_patrol_index;
    uint32_t rng_state;

    AStar_GridPoint_t score_point;
    uint8_t score_point_valid;
    uint8_t non_patrol_penalty;

    ShovelStrategyState_t state;
    TranslateRouteCmd_Status_t last_translate_status;
} ShovelStrategyContext_t;

static ShovelStrategyContext_t s_ctx;

static uint8_t is_valid_grid(int16_t x, int16_t y)
{
    return (x >= 0 && x < ASTAR_MAP_WIDTH && y >= 0 && y < ASTAR_MAP_HEIGHT);
}

static uint16_t abs_diff_i16(int16_t a, int16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t patrol_manhattan_distance(uint8_t idx_a, uint8_t idx_b)
{
    int16_t ax = (int16_t)s_ctx.patrol_points[idx_a].x;
    int16_t ay = (int16_t)s_ctx.patrol_points[idx_a].y;
    int16_t bx = (int16_t)s_ctx.patrol_points[idx_b].x;
    int16_t by = (int16_t)s_ctx.patrol_points[idx_b].y;

    return (uint16_t)(abs_diff_i16(ax, bx) + abs_diff_i16(ay, by));
}

static uint32_t rng_next_u32(void)
{
    uint32_t x = s_ctx.rng_state;

    if (x == 0)
    {
        x = HAL_GetTick() ^ 0x9E3779B9u;
        if (x == 0)
        {
            x = 0xA341316Cu;
        }
    }

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;

    s_ctx.rng_state = x;
    return x;
}

static uint32_t rng_next_bounded(uint32_t upper_exclusive)
{
    if (upper_exclusive == 0)
    {
        return 0;
    }

    return rng_next_u32() % upper_exclusive;
}

static void mix_rng_entropy(void)
{
    GlobalPose_t pose = GlobalLoc_GetPose();
    int32_t yaw10 = (int32_t)(pose.yaw * 10.0f);
    uint32_t mix = 0;

    mix ^= (uint32_t)HAL_GetTick();
    mix ^= (uint32_t)SysTick->VAL;
    mix ^= (uint32_t)pose.x_mm;
    mix ^= ((uint32_t)pose.y_mm << 11) | ((uint32_t)pose.y_mm >> 21);
    mix ^= ((uint32_t)yaw10 << 7) | ((uint32_t)yaw10 >> 25);
    mix ^= 0x9E3779B9u;

    s_ctx.rng_state ^= mix;

    if (s_ctx.rng_state == 0)
    {
        s_ctx.rng_state = 0xA341316Cu;
    }
}

static uint8_t is_candidate_far_enough(uint8_t idx)
{
    if (s_ctx.last_patrol_index < 0 || s_ctx.min_patrol_step_distance == 0)
    {
        return 1;
    }

    return (patrol_manhattan_distance((uint8_t)s_ctx.last_patrol_index, idx) >=
            (uint16_t)s_ctx.min_patrol_step_distance);
}

static int16_t select_next_patrol_index(void)
{
    uint8_t fallback_idx = 0xFFu;

    if (s_ctx.patrol_count == 0)
    {
        return -1;
    }

    if (s_ctx.patrol_count == 1)
    {
        return 0;
    }

    mix_rng_entropy();

    for (uint8_t attempts = 0; attempts < (uint8_t)(s_ctx.patrol_count * 2U); attempts++)
    {
        uint8_t idx = (uint8_t)rng_next_bounded(s_ctx.patrol_count);

        if (s_ctx.last_patrol_index >= 0 && idx == (uint8_t)s_ctx.last_patrol_index)
        {
            continue;
        }

        if (is_candidate_far_enough(idx))
        {
            return (int16_t)idx;
        }

        if (fallback_idx == 0xFFu)
        {
            fallback_idx = idx;
        }
    }

    if (fallback_idx != 0xFFu)
    {
        return (int16_t)fallback_idx;
    }

    {
        uint8_t idx = (uint8_t)rng_next_bounded(s_ctx.patrol_count);
        if (s_ctx.last_patrol_index >= 0 && idx == (uint8_t)s_ctx.last_patrol_index)
        {
            idx = (uint8_t)((idx + 1U) % s_ctx.patrol_count);
        }
        return (int16_t)idx;
    }
}

static uint8_t is_preferred_grid(int16_t x, int16_t y)
{
    for (uint8_t i = 0; i < s_ctx.patrol_count; i++)
    {
        if ((int16_t)s_ctx.patrol_points[i].x == x && (int16_t)s_ctx.patrol_points[i].y == y)
        {
            return 1;
        }
    }

    if (s_ctx.score_point_valid &&
        (int16_t)s_ctx.score_point.x == x &&
        (int16_t)s_ctx.score_point.y == y)
    {
        return 1;
    }

    return 0;
}

static void apply_non_patrol_penalty(AStar_Map_t *map)
{
    if (map == NULL || s_ctx.non_patrol_penalty == 0)
    {
        return;
    }

    for (int16_t y = 0; y < ASTAR_MAP_HEIGHT; y++)
    {
        for (int16_t x = 0; x < ASTAR_MAP_WIDTH; x++)
        {
            if (map->grid[y][x] == ASTAR_OBSTACLE_COST)
            {
                continue;
            }

            if (is_preferred_grid(x, y))
            {
                continue;
            }

            uint16_t raised = (uint16_t)map->grid[y][x] + (uint16_t)s_ctx.non_patrol_penalty;
            if (raised >= ASTAR_OBSTACLE_COST)
            {
                raised = (uint16_t)(ASTAR_OBSTACLE_COST - 1);
            }
            map->grid[y][x] = (uint8_t)raised;
        }
    }
}

static uint8_t execute_to_goal(AStar_GridPoint_t goal)
{
    char cmd[50];
    char route_cmd[50];
    AStar_Map_t *map = AStar_GetMap();
    uint8_t grid_backup[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH];
    uint16_t cmd_len = 0;

    if (map == NULL)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        return 0;
    }

    memcpy(grid_backup, map->grid, sizeof(grid_backup));
    apply_non_patrol_penalty(map);

    memset(cmd, 0, sizeof(cmd));
    // 如果巡逻点
    if (s_ctx.state == SHOVEL_STRATEGY_STATE_PATROL)
    {
        s_ctx.last_translate_status = TranslateRouteCmd_GenerateToGoalWithIntent((int16_t)goal.x,
                                                                      (int16_t)goal.y,
                                                                      TRANSLATE_ROUTE_INTENT_NONE,
                                                                      cmd,
                                                                      sizeof(cmd));
    }
    else
    {
        // 就带尾巴  放方块
        s_ctx.last_translate_status = TranslateRouteCmd_GenerateToGoalWithIntent((int16_t)goal.x,
                                                                      (int16_t)goal.y,
                                                                      TRANSLATE_ROUTE_INTENT_PLACE_CUBE,
                                                                      cmd,
                                                                      sizeof(cmd));
    }
    memcpy(map->grid, grid_backup, sizeof(grid_backup));

    if (s_ctx.last_translate_status != TRANSLATE_ROUTE_CMD_OK)
    {
        return 0;
    }

    if (cmd[0] == '\0')
    {
        return 1;
    }

    while (cmd_len < (uint16_t)(sizeof(cmd) - 1U) && cmd[cmd_len] != '\0')
    {
        cmd_len++;
    }

    if (cmd_len >= (uint16_t)(sizeof(cmd) - 1U) && cmd[cmd_len] != '\0')
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_BUFFER;
        return 0;
    }

    memset(route_cmd, 0, sizeof(route_cmd));
    memcpy(route_cmd, cmd, cmd_len);
    route_cmd[cmd_len] = '\0';

    // debug 第一行显示目标点 第二行显示转译的命令
    char debug_line[50];
    OLED_ClearArea(0, 0, 128, 16);
    sprintf(debug_line, "Goal:(%d,%d)", (int16_t)goal.x, (int16_t)goal.y);
    OLED_ShowString(0, 0, debug_line, OLED_8X16);
    OLED_ClearArea(0, 16, 128, 16);
    memset(debug_line, 0, sizeof(debug_line));
    sprintf(debug_line, "%s", route_cmd);
    OLED_ShowString(0, 16, debug_line, OLED_8X16);
    OLED_Update();
    delay_20ms(50);
    route(route_cmd);
    return 1;
}

void ShovelStrategy_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.patrol_points_before_return = 2;
    s_ctx.min_patrol_step_distance = SHOVEL_STRATEGY_MIN_PATROL_STEP_DISTANCE_DEFAULT;
    s_ctx.last_patrol_index = -1;
    s_ctx.rng_state = HAL_GetTick() ^ SysTick->VAL ^ 0x9E3779B9u;
    if (s_ctx.rng_state == 0)
    {
        s_ctx.rng_state = 0xA341316Cu;
    }
    s_ctx.non_patrol_penalty = SHOVEL_STRATEGY_NON_PATROL_PENALTY_DEFAULT;
    s_ctx.state = SHOVEL_STRATEGY_STATE_PATROL;
    s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_OK;
}

uint8_t ShovelStrategy_SetScorePoint(int16_t score_x, int16_t score_y)
{
    if (!is_valid_grid(score_x, score_y))
    {
        return 0;
    }

    s_ctx.score_point.x = (int8_t)score_x;
    s_ctx.score_point.y = (int8_t)score_y;
    s_ctx.score_point_valid = 1;
    return 1;
}

uint8_t ShovelStrategy_SetPatrolPoints(const AStar_GridPoint_t *points, uint8_t count)
{
    if (points == NULL || count == 0 || count > SHOVEL_STRATEGY_MAX_PATROL_POINTS)
    {
        return 0;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        if (!is_valid_grid((int16_t)points[i].x, (int16_t)points[i].y))
        {
            return 0;
        }

        s_ctx.patrol_points[i] = points[i];
    }

    s_ctx.patrol_count = count;
    s_ctx.last_patrol_index = -1;
    s_ctx.patrol_points_done_since_return = 0;
    s_ctx.state = SHOVEL_STRATEGY_STATE_PATROL;
    return 1;
}

void ShovelStrategy_SetPatrolPointsBeforeReturn(uint8_t points)
{
    if (points == 0)
    {
        points = 1;
    }

    s_ctx.patrol_points_before_return = points;
}

void ShovelStrategy_SetPatrolRoundsBeforeReturn(uint8_t rounds)
{
    ShovelStrategy_SetPatrolPointsBeforeReturn(rounds);
}

void ShovelStrategy_SetPatrolMinStepDistance(uint8_t min_distance)
{
    s_ctx.min_patrol_step_distance = min_distance;
}

void ShovelStrategy_SetNonPatrolPenalty(uint8_t penalty)
{
    s_ctx.non_patrol_penalty = penalty;
}

ShovelStrategyState_t ShovelStrategy_GetState(void)
{
    return s_ctx.state;
}

TranslateRouteCmd_Status_t ShovelStrategy_GetLastTranslateStatus(void)
{
    return s_ctx.last_translate_status;
}

uint8_t ShovelStrategy_RunOnce(void)
{
    int16_t next_index;

    if (!s_ctx.score_point_valid || s_ctx.patrol_count == 0)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        return 0;
    }

    if (s_ctx.state == SHOVEL_STRATEGY_STATE_PATROL)
    {
        next_index = select_next_patrol_index();
        if (next_index < 0 || next_index >= s_ctx.patrol_count)
        {
            s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
            return 0;
        }

        if (!execute_to_goal(s_ctx.patrol_points[next_index]))
        {
            return 0;
        }

        s_ctx.last_patrol_index = (int8_t)next_index;

        if (s_ctx.patrol_points_done_since_return < 0xFFu)
        {
            s_ctx.patrol_points_done_since_return++;
        }

        if (s_ctx.patrol_points_done_since_return >= s_ctx.patrol_points_before_return)
        {
            s_ctx.state = SHOVEL_STRATEGY_STATE_RETURN_SCORE;
        }

        return 1;
    }

    if (!execute_to_goal(s_ctx.score_point))
    {
        return 0;
    }

    s_ctx.patrol_points_done_since_return = 0;
    s_ctx.last_patrol_index = -1;
    s_ctx.state = SHOVEL_STRATEGY_STATE_PATROL;
    return 1;
}

void ShovelStrategy_RunLoop(void)
{
    while (1)
    {
        if (!ShovelStrategy_RunOnce())
        {
            motor_speed_set(0, 0);
            delay_20ms(5);
            continue;
        }

        delay_20ms(2);
    }
}

