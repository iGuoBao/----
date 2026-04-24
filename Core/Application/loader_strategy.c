#include "loader_strategy.h"

#include "action.h"
#include "GlobalLocalization.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define LOADER_RECOVERY_MAX_RETRY 3u
#define LOADER_RECOVERY_DRIFT_MARK_AFTER 2u
#define LOADER_STRATEGY_MAX_SCORE_POINTS 2u
#define LOADER_STRATEGY_DEBUG_OLED 1u

typedef struct
{
    AStar_GridPoint_t patrol_points[LOADER_STRATEGY_MAX_PATROL_POINTS];
    uint8_t patrol_count;
    uint8_t patrol_points_before_return;
    uint8_t patrol_points_done_since_return;
    uint8_t min_patrol_step_distance;
    int8_t last_patrol_index;
    uint32_t rng_state;

    AStar_GridPoint_t score_points[LOADER_STRATEGY_MAX_SCORE_POINTS];
    uint8_t score_point_count;
    uint8_t non_patrol_penalty;

    LoaderStrategyState_t state;
    TranslateRouteCmd_Status_t last_translate_status;
} LoaderStrategyContext_t;

static LoaderStrategyContext_t s_ctx;

#if LOADER_STRATEGY_DEBUG_OLED
static void loader_strategy_debug_oled(const char *tag, int16_t a, int16_t b, int16_t c)
{
    static uint32_t s_last_debug_tick = 0;

    GlobalPose_t pose;
    char line0[17] = {0};
    char line1[17] = {0};
    char line2[17] = {0};
    char line3[17] = {0};


    pose = GlobalLoc_GetPose();

    snprintf(line0, sizeof(line0), "LS:%s", tag);
    snprintf(line1, sizeof(line1), "st%d tr%d", (int)s_ctx.state, (int)s_ctx.last_translate_status);
    snprintf(line2, sizeof(line2), "a%d b%d c%d", (int)a, (int)b, (int)c);
    snprintf(line3, sizeof(line3), "g%d,%d", (int)pose.x_grid, (int)pose.y_grid);

    OLED_Clear();
    OLED_ShowString(0, 0, line0, OLED_8X16);
    OLED_ShowString(0, 16, line1, OLED_8X16);
    OLED_ShowString(0, 32, line2, OLED_8X16);
    OLED_ShowString(0, 48, line3, OLED_8X16);
    delay_20ms(150);
    OLED_Update();
}
#else
static void loader_strategy_debug_oled(const char *tag, int16_t a, int16_t b, int16_t c)
{
    (void)tag;
    (void)a;
    (void)b;
    (void)c;
}
#endif

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint8_t is_valid_grid(int16_t x, int16_t y)
{
    return (x >= 0 && x < ASTAR_MAP_WIDTH && y >= 0 && y < ASTAR_MAP_HEIGHT);
}

static uint16_t abs_diff_i16(int16_t a, int16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static int16_t mm_to_grid_nearest_i32(int32_t mm, int16_t max_grid)
{
    int32_t grid = (mm + (GLOBAL_GRID_SIZE_MM / 2)) / GLOBAL_GRID_SIZE_MM;
    return clamp_i16((int16_t)grid, 0, max_grid);
}

static void get_pose_grid_now(const GlobalPose_t *pose, int16_t *grid_x, int16_t *grid_y)
{
    if (pose == NULL || grid_x == NULL || grid_y == NULL)
    {
        return;
    }

    *grid_x = mm_to_grid_nearest_i32(pose->x_mm, (int16_t)(ASTAR_MAP_WIDTH - 1));
    *grid_y = mm_to_grid_nearest_i32(pose->y_mm, (int16_t)(ASTAR_MAP_HEIGHT - 1));
}

static float normalize_yaw(float deg)
{
    while (deg >= 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float abs_angle_diff(float a, float b)
{
    return fabsf(normalize_yaw(a - b));
}

static void yaw_to_forward_step(float yaw_deg, int8_t *dx, int8_t *dy)
{
    float candidates[4] = {0.0f, 90.0f, 180.0f, -90.0f};
    int8_t step_dx[4] = {1, 0, -1, 0};
    int8_t step_dy[4] = {0, 1, 0, -1};
    uint8_t best = 0;
    float best_diff = 1e9f;

    if (dx == NULL || dy == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float diff = abs_angle_diff(yaw_deg, candidates[i]);
        if (diff < best_diff)
        {
            best_diff = diff;
            best = i;
        }
    }

    *dx = step_dx[best];
    *dy = step_dy[best];
}

static void mark_front_obstacle_from_pose(void)
{
    GlobalPose_t pose = GlobalLoc_GetPose();
    int16_t now_x = 0;
    int16_t now_y = 0;
    int8_t dx = 0;
    int8_t dy = 0;
    int16_t obstacle_x;
    int16_t obstacle_y;

    get_pose_grid_now(&pose, &now_x, &now_y);
    yaw_to_forward_step(pose.yaw, &dx, &dy);

    obstacle_x = (int16_t)(now_x + dx);
    obstacle_y = (int16_t)(now_y + dy);

    if (!is_valid_grid(obstacle_x, obstacle_y))
    {
        return;
    }

    AStar_SetObstacle(obstacle_x, obstacle_y);
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
    uint32_t mix = 0;

    mix ^= (uint32_t)HAL_GetTick();
    mix ^= (uint32_t)SysTick->VAL;
    mix ^= ((uint32_t)pose.x_mm << 11) | ((uint32_t)pose.x_mm >> 21);
    mix ^= ((uint32_t)pose.y_mm << 7) | ((uint32_t)pose.y_mm >> 25);
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

    for (uint8_t i = 0; i < s_ctx.score_point_count; i++)
    {
        if ((int16_t)s_ctx.score_points[i].x == x &&
            (int16_t)s_ctx.score_points[i].y == y)
        {
            return 1;
        }
    }

    return 0;
}

static uint8_t select_nearest_score_point(AStar_GridPoint_t *out_goal)
{
    GlobalPose_t pose;
    int16_t now_x = 0;
    int16_t now_y = 0;
    uint8_t best_idx = 0;
    uint16_t best_dist = 0xFFFFu;

    if (out_goal == NULL || s_ctx.score_point_count == 0)
    {
        return 0;
    }

    if (s_ctx.score_point_count == 1)
    {
        *out_goal = s_ctx.score_points[0];
        return 1;
    }

    pose = GlobalLoc_GetPose();
    get_pose_grid_now(&pose, &now_x, &now_y);

    for (uint8_t i = 0; i < s_ctx.score_point_count; i++)
    {
        uint16_t dist = (uint16_t)(abs_diff_i16(now_x, (int16_t)s_ctx.score_points[i].x) +
                                   abs_diff_i16(now_y, (int16_t)s_ctx.score_points[i].y));
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
    }

    *out_goal = s_ctx.score_points[best_idx];
    return 1;
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

static TranslateRouteCmd_Status_t generate_route_cmd(AStar_GridPoint_t goal,
                                                      char *out_cmd,
                                                      uint16_t out_cmd_size)
{
    GlobalPose_t pose = GlobalLoc_GetPose();
    int16_t start_x = 0;
    int16_t start_y = 0;
    AStar_Map_t *map = AStar_GetMap();
    uint8_t grid_backup[ASTAR_MAP_HEIGHT][ASTAR_MAP_WIDTH];
    TranslateRouteCmd_Intent_t intent;

    if (map == NULL || out_cmd == NULL || out_cmd_size == 0)
    {
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    get_pose_grid_now(&pose, &start_x, &start_y);

    memcpy(grid_backup, map->grid, sizeof(grid_backup));
    apply_non_patrol_penalty(map);

    intent = (s_ctx.state == LOADER_STRATEGY_STATE_PATROL)
                 ? TRANSLATE_ROUTE_INTENT_NONE
                 : TRANSLATE_ROUTE_INTENT_PLACE_CUBE;

    s_ctx.last_translate_status = TranslateRouteCmd_GenerateWithIntent(start_x,
                                                                       start_y,
                                                                       pose.yaw,
                                                                       (int16_t)goal.x,
                                                                       (int16_t)goal.y,
                                                                       intent,
                                                                       out_cmd,
                                                                       out_cmd_size);

    memcpy(map->grid, grid_backup, sizeof(grid_backup));
    return s_ctx.last_translate_status;
}

static uint8_t run_route_once(AStar_GridPoint_t goal,
                              uint8_t *motion_fault,
                              uint8_t *goal_reached)
{
    char cmd[50];

    if (motion_fault == NULL || goal_reached == NULL)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        return 0;
    }

    *motion_fault = 0;
    *goal_reached = 0;

    memset(cmd, 0, sizeof(cmd));
    if (generate_route_cmd(goal, cmd, sizeof(cmd)) != TRANSLATE_ROUTE_CMD_OK)
    {
        loader_strategy_debug_oled("TRF", (int16_t)goal.x, (int16_t)goal.y, (int16_t)s_ctx.last_translate_status);
        return 0;
    }

    if (cmd[0] == '\0')
    {
        *goal_reached = 1;
        return 1;
    }

    Action_ResetMotionFault();
    Action_EnableMotionGuard(1u);
    route(cmd);
    Action_EnableMotionGuard(0u);

    *motion_fault = Action_HasMotionFault() ? 1u : 0u;
    if (*motion_fault)
    {
        loader_strategy_debug_oled("MOT", (int16_t)goal.x, (int16_t)goal.y, 0);
    }

    if (!(*motion_fault))
    {
        *goal_reached = 1;
    }

    return 1;
}

static uint8_t recovery_backoff_one_step(void)
{
    Action_ResetMotionFault();
    Action_EnableMotionGuard(1u);
    forward(-1);
    Action_EnableMotionGuard(0u);

    if (Action_HasMotionFault())
    {
        Action_ResetMotionFault();
        return 0;
    }

    delay_20ms(5);
    return 1;
}

static uint8_t execute_to_goal(AStar_GridPoint_t goal)
{
    for (uint8_t attempt = 0; attempt < LOADER_RECOVERY_MAX_RETRY; attempt++)
    {
        uint8_t motion_fault = 0;
        uint8_t goal_reached = 0;

        if (!run_route_once(goal, &motion_fault, &goal_reached))
        {
            loader_strategy_debug_oled("RRF", (int16_t)goal.x, (int16_t)goal.y, (int16_t)attempt);
            return 0;
        }

        if (goal_reached)
        {
            return 1;
        }

        motor_speed_set(0, 0);

        if (motion_fault)
        {
            // 明确静态阻挡：立刻标记前方格子，下一轮重规划避开。
            mark_front_obstacle_from_pose();

            if (!recovery_backoff_one_step())
            {
                loader_strategy_debug_oled("BOF", (int16_t)goal.x, (int16_t)goal.y, (int16_t)attempt);
                return 0;
            }
        }
        else if ((uint8_t)(attempt + 1u) >= LOADER_RECOVERY_DRIFT_MARK_AFTER)
        {
            // 连续漂移恢复失败后，升级为疑似静态阻挡，避免重复走同一路径。
            mark_front_obstacle_from_pose();
        }
    }

    loader_strategy_debug_oled("RTX", (int16_t)goal.x, (int16_t)goal.y, LOADER_RECOVERY_MAX_RETRY);

    return 0;
}

void LoaderStrategy_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.patrol_points_before_return = 2;
    s_ctx.min_patrol_step_distance = LOADER_STRATEGY_MIN_PATROL_STEP_DISTANCE_DEFAULT;
    s_ctx.last_patrol_index = -1;
    s_ctx.rng_state = HAL_GetTick() ^ SysTick->VAL ^ 0x9E3779B9u;
    if (s_ctx.rng_state == 0)
    {
        s_ctx.rng_state = 0xA341316Cu;
    }
    s_ctx.non_patrol_penalty = LOADER_STRATEGY_NON_PATROL_PENALTY_DEFAULT;
    s_ctx.state = LOADER_STRATEGY_STATE_PATROL;
    s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_OK;
}

uint8_t LoaderStrategy_SetScorePoint(int16_t score_x, int16_t score_y)
{
    if (!is_valid_grid(score_x, score_y))
    {
        return 0;
    }

    for (uint8_t i = 0; i < s_ctx.score_point_count; i++)
    {
        if ((int16_t)s_ctx.score_points[i].x == score_x &&
            (int16_t)s_ctx.score_points[i].y == score_y)
        {
            return 1;
        }
    }

    if (s_ctx.score_point_count >= LOADER_STRATEGY_MAX_SCORE_POINTS)
    {
        return 0;
    }

    s_ctx.score_points[s_ctx.score_point_count].x = (int8_t)score_x;
    s_ctx.score_points[s_ctx.score_point_count].y = (int8_t)score_y;
    s_ctx.score_point_count++;
    return 1;
}

uint8_t LoaderStrategy_SetPatrolPoints(const AStar_GridPoint_t *points, uint8_t count)
{
    if (points == NULL || count == 0 || count > LOADER_STRATEGY_MAX_PATROL_POINTS)
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
    s_ctx.state = LOADER_STRATEGY_STATE_PATROL;
    return 1;
}

void LoaderStrategy_SetPatrolPointsBeforeReturn(uint8_t points)
{
    if (points == 0)
    {
        points = 1;
    }

    s_ctx.patrol_points_before_return = points;
}

void LoaderStrategy_SetPatrolRoundsBeforeReturn(uint8_t rounds)
{
    LoaderStrategy_SetPatrolPointsBeforeReturn(rounds);
}

void LoaderStrategy_SetPatrolMinStepDistance(uint8_t min_distance)
{
    s_ctx.min_patrol_step_distance = min_distance;
}

void LoaderStrategy_SetNonPatrolPenalty(uint8_t penalty)
{
    s_ctx.non_patrol_penalty = penalty;
}

LoaderStrategyState_t LoaderStrategy_GetState(void)
{
    return s_ctx.state;
}

TranslateRouteCmd_Status_t LoaderStrategy_GetLastTranslateStatus(void)
{
    return s_ctx.last_translate_status;
}


uint8_t LoaderStrategy_RunOnce(void)
{
    int16_t next_index;
    AStar_GridPoint_t return_goal;

    if (s_ctx.score_point_count == 0 || s_ctx.patrol_count == 0)
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        loader_strategy_debug_oled("PAR", (int16_t)s_ctx.score_point_count, (int16_t)s_ctx.patrol_count, 0);
        return 0;
    }

    if (s_ctx.state == LOADER_STRATEGY_STATE_PATROL)
    {
        next_index = select_next_patrol_index();
        if (next_index < 0 || next_index >= s_ctx.patrol_count)
        {
            s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
            loader_strategy_debug_oled("IDX", next_index, (int16_t)s_ctx.patrol_count, 0);
            return 0;
        }

        if (!execute_to_goal(s_ctx.patrol_points[next_index]))
        {
            loader_strategy_debug_oled("PAT", (int16_t)s_ctx.patrol_points[next_index].x,
                                      (int16_t)s_ctx.patrol_points[next_index].y,
                                      next_index);
            return 0;
        }

        s_ctx.last_patrol_index = (int8_t)next_index;

        if (s_ctx.patrol_points_done_since_return < 0xFFu)
        {
            s_ctx.patrol_points_done_since_return++;
        }

        if (s_ctx.patrol_points_done_since_return >= s_ctx.patrol_points_before_return)
        {
            s_ctx.state = LOADER_STRATEGY_STATE_RETURN_SCORE;
        }

        return 1;
    }

    if (!select_nearest_score_point(&return_goal))
    {
        s_ctx.last_translate_status = TRANSLATE_ROUTE_CMD_ERR_PARAM;
        loader_strategy_debug_oled("NSP", (int16_t)s_ctx.score_point_count, 0, 0);
        return 0;
    }

    if (!execute_to_goal(return_goal))
    {
        loader_strategy_debug_oled("SCR", (int16_t)return_goal.x, (int16_t)return_goal.y, 0);
        return 0;
    }

    s_ctx.patrol_points_done_since_return = 0;
    s_ctx.last_patrol_index = -1;
    s_ctx.state = LOADER_STRATEGY_STATE_PATROL;
    return 1;
}

void LoaderStrategy_RunLoop(void)
{
    while (1)
    {
        if (!LoaderStrategy_RunOnce())
        {
            motor_speed_set(0, 0);
            delay_20ms(5);
            continue;
        }

        delay_20ms(2);
    }
}

