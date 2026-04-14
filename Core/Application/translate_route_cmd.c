#include "translate_route_cmd.h"

#include "GlobalLocalization.h"
#include "astar.h"

#include <math.h>
#include <string.h>

typedef enum
{
    HEADING_X_PLUS = 0,
    HEADING_Y_PLUS = 1,
    HEADING_X_MINUS = 2,
    HEADING_Y_MINUS = 3
} Heading_t;

typedef struct
{
    int8_t from_x;
    int8_t from_y;
    int8_t to_x;
    int8_t to_y;
    uint8_t route_units;
    uint8_t in_use;
} SpecialEdgeRule_t;

static SpecialEdgeRule_t s_special_edge_rules[TRANSLATE_ROUTE_CMD_MAX_SPECIAL_EDGES];

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

static uint8_t is_valid_grid(int16_t x, int16_t y)
{
    return (x >= 0 && x < ASTAR_MAP_WIDTH && y >= 0 && y < ASTAR_MAP_HEIGHT);
}

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

static int16_t mm_to_grid_x(int16_t x_mm)
{
    int32_t numerator = (int32_t)x_mm * ASTAR_MAP_WIDTH + (ASTAR_MAP_WIDTH_MM / 2);
    int16_t x_grid = (int16_t)(numerator / ASTAR_MAP_WIDTH_MM);
    return clamp_i16(x_grid, 0, ASTAR_MAP_WIDTH - 1);
}

static int16_t mm_to_grid_y(int16_t y_mm)
{
    int32_t numerator = (int32_t)y_mm * ASTAR_MAP_HEIGHT + (ASTAR_MAP_HEIGHT_MM / 2);
    int16_t y_grid = (int16_t)(numerator / ASTAR_MAP_HEIGHT_MM);
    return clamp_i16(y_grid, 0, ASTAR_MAP_HEIGHT - 1);
}

static uint8_t yaw_to_heading(float yaw_deg, Heading_t *heading)
{
    float candidates[4] = {0.0f, 90.0f, 180.0f, -90.0f};
    Heading_t mapped[4] = {HEADING_X_PLUS, HEADING_Y_PLUS, HEADING_X_MINUS, HEADING_Y_MINUS};

    float min_diff = 1e9f;
    uint8_t min_index = 0;

    if (heading == NULL)
    {
        return 0;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        float diff = abs_angle_diff(yaw_deg, candidates[i]);
        if (diff < min_diff)
        {
            min_diff = diff;
            min_index = i;
        }
    }

    *heading = mapped[min_index];
    return 1;
}

static uint8_t step_to_heading(int16_t dx, int16_t dy, Heading_t *heading)
{
    if (heading == NULL)
    {
        return 0;
    }

    if (dx == 1 && dy == 0)
    {
        *heading = HEADING_X_PLUS;
        return 1;
    }
    if (dx == 0 && dy == 1)
    {
        *heading = HEADING_Y_PLUS;
        return 1;
    }
    if (dx == -1 && dy == 0)
    {
        *heading = HEADING_X_MINUS;
        return 1;
    }
    if (dx == 0 && dy == -1)
    {
        *heading = HEADING_Y_MINUS;
        return 1;
    }

    return 0;
}

static TranslateRouteCmd_Status_t append_char(char *out_cmd,
                                              uint16_t out_cmd_size,
                                              uint16_t *out_len,
                                              char c)
{
    if (out_cmd == NULL || out_len == NULL || out_cmd_size == 0)
    {
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    if ((uint32_t)(*out_len) + 1U >= out_cmd_size)
    {
        return TRANSLATE_ROUTE_CMD_ERR_BUFFER;
    }

    out_cmd[*out_len] = c;
    (*out_len)++;
    out_cmd[*out_len] = '\0';

    return TRANSLATE_ROUTE_CMD_OK;
}

static TranslateRouteCmd_Status_t append_turn_cmds(char *out_cmd,
                                                   uint16_t out_cmd_size,
                                                   uint16_t *out_len,
                                                   Heading_t current,
                                                   Heading_t target)
{
    uint8_t diff = (uint8_t)((target - current + 4) % 4);

    if (diff == 0)
    {
        return TRANSLATE_ROUTE_CMD_OK;
    }
    if (diff == 1)
    {
        return append_char(out_cmd, out_cmd_size, out_len, 'L');
    }
    if (diff == 3)
    {
        return append_char(out_cmd, out_cmd_size, out_len, 'R');
    }

    return append_char(out_cmd, out_cmd_size, out_len, 'A');
}

static TranslateRouteCmd_Status_t append_forward_units(char *out_cmd,
                                                       uint16_t out_cmd_size,
                                                       uint16_t *out_len,
                                                       uint16_t forward_units)
{
    TranslateRouteCmd_Status_t status;

    if (forward_units == 0)
    {
        return TRANSLATE_ROUTE_CMD_ERR_ZERO_FORWARD;
    }

    while (forward_units >= 9)
    {
        status = append_char(out_cmd, out_cmd_size, out_len, '9');
        if (status != TRANSLATE_ROUTE_CMD_OK)
        {
            return status;
        }
        forward_units -= 9;
    }

    if (forward_units > 0)
    {
        status = append_char(out_cmd, out_cmd_size, out_len, (char)('0' + forward_units));
        if (status != TRANSLATE_ROUTE_CMD_OK)
        {
            return status;
        }
    }

    return TRANSLATE_ROUTE_CMD_OK;
}

static TranslateRouteCmd_Status_t append_intent_tail(char *out_cmd,
                                                     uint16_t out_cmd_size,
                                                     uint16_t *out_len,
                                                     TranslateRouteCmd_Intent_t intent)
{
    const char *tail = NULL;

    switch (intent)
    {
    case TRANSLATE_ROUTE_INTENT_NONE:
        return TRANSLATE_ROUTE_CMD_OK;
    case TRANSLATE_ROUTE_INTENT_PICK_CUBE:
    case TRANSLATE_ROUTE_INTENT_PICK_RING:
        tail = "K";
        break;
    case TRANSLATE_ROUTE_INTENT_PLACE_RING:
        tail = "tfObd";
        break;
    case TRANSLATE_ROUTE_INTENT_PLACE_CUBE:
        tail = "Op";
        break;
    default:
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    for (uint16_t i = 0; tail[i] != '\0'; i++)
    {
        TranslateRouteCmd_Status_t status = append_char(out_cmd, out_cmd_size, out_len, tail[i]);
        if (status != TRANSLATE_ROUTE_CMD_OK)
        {
            return status;
        }
    }

    return TRANSLATE_ROUTE_CMD_OK;
}

static uint8_t get_edge_route_units(int16_t from_x,
                                    int16_t from_y,
                                    int16_t to_x,
                                    int16_t to_y)
{
    for (uint8_t i = 0; i < TRANSLATE_ROUTE_CMD_MAX_SPECIAL_EDGES; i++)
    {
        const SpecialEdgeRule_t *rule = &s_special_edge_rules[i];
        if (!rule->in_use)
        {
            continue;
        }

        if (rule->from_x == from_x &&
            rule->from_y == from_y &&
            rule->to_x == to_x &&
            rule->to_y == to_y)
        {
            return rule->route_units;
        }
    }

    return 1;
}

void TranslateRouteCmd_SetSpecialEdgeRules(void)
{
    // 示例：设置 (1,1) -> (1,2) 的前进计数为 0（表示不计入 route 前进数）
    // TranslateRouteCmd_AddEdgeRouteUnits(1, 1, 1, 2, 0);

    // 己方斜坡左侧
    TranslateRouteCmd_AddEdgeRouteUnits(1, 5, 1, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(1, 7, 1, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(2, 5, 2, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(2, 7, 2, 6, 0);
    // 己方斜坡右侧
    TranslateRouteCmd_AddEdgeRouteUnits(1, 5, 1, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(1, 3, 1, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(2, 5, 2, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(2, 3, 2, 4, 0);
    // 对方斜坡左侧
    TranslateRouteCmd_AddEdgeRouteUnits(6, 5, 6, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(6, 7, 6, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(7, 5, 7, 6, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(7, 7, 7, 6, 0);
    // 对方斜坡右侧
    TranslateRouteCmd_AddEdgeRouteUnits(6, 5, 6, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(6, 3, 6, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(7, 5, 7, 4, 0);
    TranslateRouteCmd_AddEdgeRouteUnits(7, 3, 7, 4, 0);
}

void TranslateRouteCmd_ClearEdgeRouteUnits(void)
{
    memset(s_special_edge_rules, 0, sizeof(s_special_edge_rules));
}

TranslateRouteCmd_Status_t TranslateRouteCmd_AddEdgeRouteUnits(int16_t from_x,
                                                               int16_t from_y,
                                                               int16_t to_x,
                                                               int16_t to_y,
                                                               uint8_t route_units)
{
    if (!is_valid_grid(from_x, from_y) || !is_valid_grid(to_x, to_y))
    {
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    for (uint8_t i = 0; i < TRANSLATE_ROUTE_CMD_MAX_SPECIAL_EDGES; i++)
    {
        SpecialEdgeRule_t *rule = &s_special_edge_rules[i];

        if (rule->in_use &&
            rule->from_x == from_x &&
            rule->from_y == from_y &&
            rule->to_x == to_x &&
            rule->to_y == to_y)
        {
            rule->route_units = route_units;
            return TRANSLATE_ROUTE_CMD_OK;
        }
    }

    for (uint8_t i = 0; i < TRANSLATE_ROUTE_CMD_MAX_SPECIAL_EDGES; i++)
    {
        SpecialEdgeRule_t *rule = &s_special_edge_rules[i];

        if (!rule->in_use)
        {
            rule->from_x = (int8_t)from_x;
            rule->from_y = (int8_t)from_y;
            rule->to_x = (int8_t)to_x;
            rule->to_y = (int8_t)to_y;
            rule->route_units = route_units;
            rule->in_use = 1;
            return TRANSLATE_ROUTE_CMD_OK;
        }
    }

    return TRANSLATE_ROUTE_CMD_ERR_RULE_FULL;
}

TranslateRouteCmd_Status_t TranslateRouteCmd_Generate(int16_t start_x,
                                                      int16_t start_y,
                                                      float start_yaw_deg,
                                                      int16_t goal_x,
                                                      int16_t goal_y,
                                                      char *out_cmd,
                                                      uint16_t out_cmd_size)
{
    Path_t path;
    AStar_GridPoint_t start;
    AStar_GridPoint_t goal;
    Heading_t current_heading;
    uint16_t out_len = 0;

    if (out_cmd == NULL || out_cmd_size == 0)
    {
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    out_cmd[0] = '\0';

    if (!is_valid_grid(start_x, start_y) || !is_valid_grid(goal_x, goal_y))
    {
        return TRANSLATE_ROUTE_CMD_ERR_PARAM;
    }

    if (!yaw_to_heading(start_yaw_deg, &current_heading))
    {
        return TRANSLATE_ROUTE_CMD_ERR_YAW;
    }

    if (start_x == goal_x && start_y == goal_y)
    {
        return TRANSLATE_ROUTE_CMD_OK;
    }

    start.x = (int8_t)start_x;
    start.y = (int8_t)start_y;
    goal.x = (int8_t)goal_x;
    goal.y = (int8_t)goal_y;

    if (!AStar_FindPath(start, goal, &path) || !path.is_valid || path.point_count < 2)
    {
        return TRANSLATE_ROUTE_CMD_ERR_NO_PATH;
    }

    uint16_t i = 1;
    while (i < path.point_count)
    {
        TranslateRouteCmd_Status_t status;
        Heading_t segment_heading;

        while (i < path.point_count)
        {
            int16_t from_x = mm_to_grid_x(path.points[i - 1].x_mm);
            int16_t from_y = mm_to_grid_y(path.points[i - 1].y_mm);
            int16_t to_x = mm_to_grid_x(path.points[i].x_mm);
            int16_t to_y = mm_to_grid_y(path.points[i].y_mm);
            int16_t dx = (int16_t)(to_x - from_x);
            int16_t dy = (int16_t)(to_y - from_y);

            if (!step_to_heading(dx, dy, &segment_heading))
            {
                return TRANSLATE_ROUTE_CMD_ERR_STEP;
            }
            break;
        }

        status = append_turn_cmds(out_cmd, out_cmd_size, &out_len, current_heading, segment_heading);
        if (status != TRANSLATE_ROUTE_CMD_OK)
        {
            return status;
        }

        current_heading = segment_heading;

        uint16_t forward_units = 0;
        while (i < path.point_count)
        {
            Heading_t this_heading;
            int16_t from_x = mm_to_grid_x(path.points[i - 1].x_mm);
            int16_t from_y = mm_to_grid_y(path.points[i - 1].y_mm);
            int16_t to_x = mm_to_grid_x(path.points[i].x_mm);
            int16_t to_y = mm_to_grid_y(path.points[i].y_mm);
            int16_t dx = (int16_t)(to_x - from_x);
            int16_t dy = (int16_t)(to_y - from_y);

            if (!step_to_heading(dx, dy, &this_heading))
            {
                return TRANSLATE_ROUTE_CMD_ERR_STEP;
            }

            if (this_heading != segment_heading)
            {
                break;
            }

            forward_units += get_edge_route_units(from_x, from_y, to_x, to_y);
            i++;
        }

        status = append_forward_units(out_cmd, out_cmd_size, &out_len, forward_units);
        if (status != TRANSLATE_ROUTE_CMD_OK)
        {
            return status;
        }
    }

    return TRANSLATE_ROUTE_CMD_OK;
}

TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateToGoal(int16_t goal_x,
                                                            int16_t goal_y,
                                                            char *out_cmd,
                                                            uint16_t out_cmd_size)
{
    GlobalPose_t pose = GlobalLoc_GetPose();

    return TranslateRouteCmd_Generate((int16_t)pose.x_grid,
                                      (int16_t)pose.y_grid,
                                      pose.yaw,
                                      goal_x,
                                      goal_y,
                                      out_cmd,
                                      out_cmd_size);
}

TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateWithIntent(int16_t start_x,
                                                                int16_t start_y,
                                                                float start_yaw_deg,
                                                                int16_t goal_x,
                                                                int16_t goal_y,
                                                                TranslateRouteCmd_Intent_t intent,
                                                                char *out_cmd,
                                                                uint16_t out_cmd_size)
{
    TranslateRouteCmd_Status_t status = TranslateRouteCmd_Generate(start_x,
                                                                   start_y,
                                                                   start_yaw_deg,
                                                                   goal_x,
                                                                   goal_y,
                                                                   out_cmd,
                                                                   out_cmd_size);
    uint16_t out_len;

    if (status != TRANSLATE_ROUTE_CMD_OK)
    {
        return status;
    }

    out_len = (uint16_t)strlen(out_cmd);
    return append_intent_tail(out_cmd, out_cmd_size, &out_len, intent);
}

TranslateRouteCmd_Status_t TranslateRouteCmd_GenerateToGoalWithIntent(int16_t goal_x,
                                                                      int16_t goal_y,
                                                                      TranslateRouteCmd_Intent_t intent,
                                                                      char *out_cmd,
                                                                      uint16_t out_cmd_size)
{
    TranslateRouteCmd_Status_t status = TranslateRouteCmd_GenerateToGoal(goal_x,
                                                                          goal_y,
                                                                          out_cmd,
                                                                          out_cmd_size);
    uint16_t out_len;

    if (status != TRANSLATE_ROUTE_CMD_OK)
    {
        return status;
    }

    out_len = (uint16_t)strlen(out_cmd);
    return append_intent_tail(out_cmd, out_cmd_size, &out_len, intent);
}

const char *TranslateRouteCmd_StatusString(TranslateRouteCmd_Status_t status)
{
    switch (status)
    {
    case TRANSLATE_ROUTE_CMD_OK:
        return "OK";
    case TRANSLATE_ROUTE_CMD_ERR_PARAM:
        return "ERR_PARAM";
    case TRANSLATE_ROUTE_CMD_ERR_NO_PATH:
        return "ERR_NO_PATH";
    case TRANSLATE_ROUTE_CMD_ERR_YAW:
        return "ERR_YAW";
    case TRANSLATE_ROUTE_CMD_ERR_STEP:
        return "ERR_STEP";
    case TRANSLATE_ROUTE_CMD_ERR_BUFFER:
        return "ERR_BUFFER";
    case TRANSLATE_ROUTE_CMD_ERR_ZERO_FORWARD:
        return "ERR_ZERO_FORWARD";
    case TRANSLATE_ROUTE_CMD_ERR_RULE_FULL:
        return "ERR_RULE_FULL";
    default:
        return "ERR_UNKNOWN";
    }
}
