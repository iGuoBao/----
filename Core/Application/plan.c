#include "plan.h"
#include "action.h"
#include "shovel_strategy.h"

// 去左侧的
void plan_4_a()
{
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);

    // 适配角度
    // 特调 下基地车
    forward_delay(75,40);
    delay_20ms(50);
    forward(1);
    // 抓取圆环
    route("LOw1Kw");
    // 特调 下坡 期望变速
    forward_delay(50,80);
    forward_delay(200,40);
    // 满分前
    route("1R1L1R");
    // 得分
    route("twwwfObd");

    // 在左侧对方区域满分环
    // route("wwOwdwf1L1Kw2R1L1RtwwwfObd");
    // 去待机区
    route("Rbw");
    // 在待机区等待
    for (int i = 0; i < 8; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2wwbbw1");
    
    // 尝试右侧拿走2分
    // route("TRfDKtbK");
    route("tRwwwfODwwwwwKTwwwbwODww");

    route("S");
}

// 去右侧的
void plan_4_b()
{

    delay_20ms(50);
    delay_20ms(50);
    // 适配角度
    // 特调 下基地车
    // forward_delay(100,40);
    forward(1);
    // 特调下坡 变速
    // route("L");
    // forward_delay(100,50);
    // 特调下坡 直接倒车
    route("L");
    forward(-1);
    route("A");

    // 去对方得分区
    route("1L1R1AbO");

    // 在右侧对方区域阻碍对方满分
    // route("f1R1ww1KL1R1AbO");
    // 等待100s
    for (int i = 0; i < 8; i++)
    {
        delay_20ms(50);
    }
    // 推走方块 然后回去
    route("2wwbw");
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
    ShovelStrategy_SetPatrolPointsBeforeReturn(2);
    ShovelStrategy_SetPatrolMinStepDistance(2);
    ShovelStrategy_SetNonPatrolPenalty(200);

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

void test_servo_loop(void)
{
    route("OwwwwKwwwwtwwwwdwwwS");
}

