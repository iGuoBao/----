#include "plan.h"
#include "action.h"
#include "shovel_strategy.h"


/**
 * @brief 策略 到左侧对方区域满分环，去待机区等待，推走方块 然后回去，尝试左侧拿走2分
 *         双车策略
 */
void plan_a()
{
    // 在左侧对方区域满分环
    route("wO1dfw1KL2R1L1RtwfObd");
    // 去待机区
    route("Rbw");
    // 在待机区等待
    for (int i = 0; i < 8; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2bw");
    
    // 尝试右侧拿走2分
    // route("TRfDKtbK");
    route("tRwwwfODwwKTwwwbwOdww");

    route("S");
}


/**
 * @brief 策略 到右侧对方区域阻碍对方满分，等待100s，推走方块 然后回去，尝试右侧抓走对方满分环，尝试右侧拿走2分
 *         双车策略
 */
void plan_b()
{
    // 在右侧对方区域阻碍对方满分
    route("1R1ww1KL2R1AbO");
    // 等待100s
    for (int i = 0; i < 3; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2bw");
    // 尝试右侧抓走对方满分环
    // route("T1RfDKTbO");
    route("tRwwwfODwKTwwwbwOww");
    // 尝试右侧拿走2分
    // route("AfDKTbORd");
    route("LLwwwfODwwKTwwwbwOdww");
    route("S");
}

/**
 * @brief 策略 到右侧对方区域阻碍对方满分，等待100s，推走方块 然后回去，尝试右侧抓走对方满分环，尝试右侧拿走2分
 *         4车策略
 */
void plan_4_a()
{
    // 在左侧对方区域满分环
    route("wwwOdf1L1K2R1L1RtwfObd");
    // 去待机区
    route("Rbw");
    // 在待机区等待
    for (int i = 0; i < 8; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2bw");
    
    // 尝试右侧拿走2分
    // route("TRfDKtbK");
    route("tRwwwfODwwKTwwwbwOdww");

    route("S");
}

void plan_4_b()
{
    // 在右侧对方区域阻碍对方满分
    route("f1R1ww1KL1R1AbO");
    // 等待100s
    for (int i = 0; i < 3; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2bw");
    // 尝试右侧抓走对方满分环
    // route("T1RfDKTbO");
    route("tRwwwfODwKTwwwbwOww");
    // 尝试右侧拿走2分
    // route("AfDKTbORd");
    route("LLwwwfODwwKTwwwbwOdww");
    route("S");
}

void plan_shovel_patrol_loop(void)
{
    static const AStar_GridPoint_t patrol_points[] = {
        {1, 1, {0, 0}},
        {1, 2, {0, 0}},
        {1, 3, {0, 0}},
        {1, 7, {0, 0}},
        {1, 8, {0, 0}},
        {1, 9, {0, 0}},
        
        {2, 2, {0, 0}},
        {2, 3, {0, 0}},
        {2, 7, {0, 0}},
        {2, 8, {0, 0}},

        {3, 1, {0, 0}},
        {3, 2, {0, 0}},
        {3, 3, {0, 0}},
        {3, 4, {0, 0}},
        {3, 5, {0, 0}},
        {3, 6, {0, 0}},
        {3, 7, {0, 0}},
        {3, 8, {0, 0}},
        {3, 9, {0, 0}},

        {4, 2, {0, 0}},
        {4, 3, {0, 0}},
        {4, 4, {0, 0}},
        {4, 6, {0, 0}},
        {4, 7, {0, 0}},
        {4, 8, {0, 0}},

        {5, 1, {0, 0}},
        {5, 2, {0, 0}},
        {5, 3, {0, 0}},
        {5, 4, {0, 0}},
        {5, 5, {0, 0}},
        {5, 6, {0, 0}},
        {5, 7, {0, 0}},
        {5, 8, {0, 0}},
        {5, 9, {0, 0}},

        {6, 2, {0, 0}},
        {6, 3, {0, 0}},
        {6, 7, {0, 0}},
        {6, 8, {0, 0}},

        {7, 1, {0, 0}},
        {7, 2, {0, 0}},
        {7, 3, {0, 0}},
        {7, 7, {0, 0}},
        {7, 8, {0, 0}},
        {7, 9, {0, 0}},
    };

    ShovelStrategy_Init();
    ShovelStrategy_SetPatrolRoundsBeforeReturn(2);
    ShovelStrategy_SetNonPatrolPenalty(80);

    if (!ShovelStrategy_SetScorePoint(5, 1))
    {
        return;
    }
    if (!ShovelStrategy_SetScorePoint(5, 9))
    {
        return;
    }

    if (!ShovelStrategy_SetPatrolPoints(patrol_points, (uint8_t)(sizeof(patrol_points) / sizeof(patrol_points[0]))))
    {
        return;
    }

    ShovelStrategy_RunLoop();
}