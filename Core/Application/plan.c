#include "plan.h"
#include "action.h"


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