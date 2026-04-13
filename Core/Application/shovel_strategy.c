#include "shovel_strategy.h"

#include "action.h"

#include <string.h>

typedef struct
{
    AStar_GridPoint_t patrol_points[SHOVEL_STRATEGY_MAX_PATROL_POINTS];
    uint8_t patrol_count;
    uint8_t patrol_index;
    uint8_t patrol_rounds_before_return;
    uint8_t patrol_rounds_done;

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
    AStar_Map_t *map = AStar_GetMap();
    uint8_t grid_backup[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH];

    if (map == NULL)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        return 0;
    }

    memcpy(grid_backup, map->grid, sizeof(grid_backup));
    apply_non_patrol_penalty(map);

    memset(cmd, 0, sizeof(cmd));
    s_ctx.last_translate_status = TranslateRouteCmd_GenerateToGoal((int16_t)goal.x,
                                                                   (int16_t)goal.y,
                                                                   cmd,
                                                                   (uint16_t)sizeof(cmd));
    memcpy(map->grid, grid_backup, sizeof(grid_backup));

    if (s_ctx.last_translate_status != TRANSLATE_ROUTE_CMD_OK)
    {
        return 0;
    }

    if (cmd[0] == '\0')
    {
        return 1;
    }

    route(cmd);
    return 1;
}

void ShovelStrategy_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.patrol_rounds_before_return = 2;
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
    s_ctx.patrol_index = 0;
    s_ctx.patrol_rounds_done = 0;
    s_ctx.state = SHOVEL_STRATEGY_STATE_PATROL;
    return 1;
}

void ShovelStrategy_SetPatrolRoundsBeforeReturn(uint8_t rounds)
{
    if (rounds == 0)
    {
        rounds = 1;
    }

    s_ctx.patrol_rounds_before_return = rounds;
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
    if (!s_ctx.score_point_valid || s_ctx.patrol_count == 0)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        return 0;
    }

    if (s_ctx.state == SHOVEL_STRATEGY_STATE_PATROL)
    {
        if (!execute_to_goal(s_ctx.patrol_points[s_ctx.patrol_index]))
        {
            return 0;
        }

        s_ctx.patrol_index++;
        if (s_ctx.patrol_index >= s_ctx.patrol_count)
        {
            s_ctx.patrol_index = 0;
            s_ctx.patrol_rounds_done++;

            if (s_ctx.patrol_rounds_done >= s_ctx.patrol_rounds_before_return)
            {
                s_ctx.state = SHOVEL_STRATEGY_STATE_RETURN_SCORE;
            }
        }

        return 1;
    }

    if (!execute_to_goal(s_ctx.score_point))
    {
        return 0;
    }

    s_ctx.patrol_rounds_done = 0;
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