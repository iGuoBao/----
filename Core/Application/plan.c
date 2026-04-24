#include "plan.h"
#include "action.h"
#include "loader_strategy.h"

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
    // route("twwwfObd");
    // 从1到4
    route("T");
    delay_20ms(150);
    route("Z");
    delay_20ms(100);
    route("fOb");

    // 从4到1
    route("D");
    delay_20ms(120);
    route("Z");
    delay_20ms(100);

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
    // route("tRwwwfODwwwKwwTwwwbwODw");
    // 从1到4
    route("T");
    delay_20ms(150);
    route("Z");
    delay_20ms(100);
    route("ORf");
    // 从4到2
    route("D");
    delay_20ms(53);
    route("Z");
    delay_20ms(100);
    route("Kww");
    // 从2到4
    route("T");
    delay_20ms(105);
    route("Z");
    delay_20ms(100);
    route("b");

    route("S");
}

// 去右侧的
void plan_4_b()
{

    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    // 适配角度
    // 特调 下基地车
    forward_delay(25,30);
    delay_20ms(50);
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
    route("2wwbbw1");
    // 尝试右侧抓走对方满分环
    // route("tRwwwfODwKTwwwbwOww");
    // 从1到4
    route("T");
    delay_20ms(150);
    route("Z");
    delay_20ms(100);
    route("RwfO");
    // 从4到3
    route("D");
    delay_20ms(32);
    route("Z");
    delay_20ms(100);
    route("Kww");
    // 从3到4
    route("T");
    delay_20ms(67);
    route("Z");
    delay_20ms(100);
    route("bwOww");


    // 尝试右侧拿走2分
    route("LLwwwfO");
    // 从4到2
    route("D");
    delay_20ms(53);
    route("Z");
    delay_20ms(100);
    route("Kww");
    // 从2到4
    route("T");
    delay_20ms(105);
    route("Z");
    delay_20ms(100);
    route("b");
    route("S");
}

void plan_loader_patrol_loop(void)
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

    LoaderStrategy_Init();
    LoaderStrategy_SetPatrolPointsBeforeReturn(2);
    LoaderStrategy_SetPatrolMinStepDistance(2);
    LoaderStrategy_SetNonPatrolPenalty(200);

    if (!LoaderStrategy_SetScorePoint(5, 1))
    {
        return;
    }
    if (!LoaderStrategy_SetScorePoint(5, 9))
    {
        return;
    }

    if (!LoaderStrategy_SetPatrolPoints(patrol_points, (uint8_t)(sizeof(patrol_points) / sizeof(patrol_points[0]))))
    {
        return;
    }

    LoaderStrategy_RunLoop();
}

void test_servo_loop(void)
{
    route("OwwwwKwwwwtwwwwdwwwS");
}

void base_car_startup_strategy_run(void)
{
    delay_20ms(75);
    motor_speed_set(80, 62); // 155   120
    delay_20ms(62);
    motor_speed_set(-7, -5);
    delay_20ms(300);
    motor_speed_set(0, 0);
    while (1);
}

