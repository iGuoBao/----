#include "startup_strategy.h"


#define STARTUP_CMD_TEXT "START"
#define STARTUP_CMD_LEN 5u

typedef void (*StartupRunFunc_t)(void);

typedef struct {
    const char *name;
    StartupRunFunc_t run;
} StartupStrategy_t;

static uint8_t s_nrf_ready = 0;
static uint8_t s_button_latched[BUTTON_COUNT] = {0};
static uint8_t should_start = 0u;

static void startup_run_plan_4_a(void)
{
    plan_4_a();
}

static void startup_run_plan_4_b(void)
{
    plan_4_b();
}

static void startup_run_loader_reset_loop(void)
{
    route("R1L1");
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    delay_20ms(50);
    GlobalLoc_ResetPose(400, 1200, 0);
    plan_loader_patrol_loop();
}

static void startup_test_servo_loop(void)
{
    // 推走方块 然后回去
    route("O2wwbbw1");
    // 尝试右侧抓走对方满分环
    // route("tRwwwfODwKTwwwbwOww");
    // 从1到4
    route("T");
    delay_20ms(SERVO_1to4_TIME);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    route("RwfO");
    // 从4到3
    route("D");
    delay_20ms(SERVO_4to3_TIME);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    route("Kww");
    // 从3到4
    route("T");
    delay_20ms(SERVO_3to4_TIME);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    route("bwOww");


    // 尝试右侧拿走2分
    route("LLwwwfO");
    // 从4到2
    route("D");
    delay_20ms(SERVO_4to2_TIME);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    route("Kww");
    // 从2到4
    route("T");
    delay_20ms(SERVO_2to4_TIME);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    route("b");
    route("S");
}

static void startup_test_servo_pwm(void)
{
    /**
     * @brief
     * 先标定预期的四个高度
     * 1 底部
     * 2 能抓2分的高度
     * 3 能抓满分的高度
     * 4 高处，不会被圆柱挡住的高度
     * 
     * 预期 能够准确到达四个高度
     * 
     * 从1到4
     * 从4到1
     * 从1到4
     * 
     * 从4到3
     * 从3到4
     * 
     * 从4到2
     * 从2到4
     * 
     * 2和3 基本都是从4过渡的 尽可能保证相同
     */
    // 从1到4
    route("T");
    delay_20ms(150);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    // 从4到1
    route("D");
    delay_20ms(120);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    // 从4到1
    route("T");
    delay_20ms(150);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);

    // 从4到3
    route("D");
    delay_20ms(32);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);

    // 从3到4
    route("T");
    delay_20ms(67);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);

    // 从4到2
    route("D");
    delay_20ms(53);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);

    // 从2到4
    route("T");
    delay_20ms(105);
    route("Z");
    delay_20ms(SERVO_DELAY_TIME);
    
}

static const StartupStrategy_t g_startup_strategies[] = {
    {"R1 C GO L", startup_run_plan_4_a},
    {"R1 C GO R", startup_run_plan_4_b},
    {"R1 L GO LOOP", startup_run_loader_reset_loop},
    {"TEST SERVO", startup_test_servo_loop},
    {"TEST SERVO PWM", startup_test_servo_pwm},
};

#define STARTUP_STRATEGY_COUNT ((uint8_t)(sizeof(g_startup_strategies) / sizeof(g_startup_strategies[0])))

static uint8_t startup_button_pressed_once(Button_t button)
{
    if (Button_IsPressed(button))
    {
        if (s_button_latched[button] == 0u)
        {
            s_button_latched[button] = 1u;
            return 1u;
        }
    }
    else
    {
        s_button_latched[button] = 0u;
    }

    return 0u;
}

static void startup_show_strategy(uint8_t strategy_index)
{
    char index_text[6];

    index_text[0] = (char)('1' + strategy_index);
    index_text[1] = '/';
    index_text[2] = (char)('0' + STARTUP_STRATEGY_COUNT);
    index_text[3] = '\0';

    OLED_Clear();
    
    OLED_ShowString(0, 0, "Select Strategy", OLED_8X16);
    OLED_ShowString(0, 16, "Index:", OLED_8X16);
    OLED_ShowString(56, 16, index_text, OLED_8X16);
    OLED_ShowString(0, 32, (char *)g_startup_strategies[strategy_index].name, OLED_8X16);

    if (s_nrf_ready)
    {
        OLED_ShowString(0, 48, "B3 START NRF ON", OLED_8X16);
    }
    else
    {
        OLED_ShowString(0, 48, "B3 START NRF OFF", OLED_8X16);
    }

    OLED_Update();
}

static void startup_nrf_init(void)
{
    s_nrf_ready = (NRF24L01_Check() == 0u) ? 1u : 0u;
    if (s_nrf_ready)
    {
        NRF24L01_RX_Mode();
    }
}

static uint8_t startup_nrf_has_start_cmd(void)
{
    uint8_t rx_payload[RX_PLOAD_WIDTH];

    if (!s_nrf_ready)
    {
        return 0u;
    }

    if (NRF24L01_RxPacket(rx_payload) != 0u)
    {
        return 0u;
    }

    if (memcmp(rx_payload, STARTUP_CMD_TEXT, STARTUP_CMD_LEN) == 0)
    {
        return 1u;
    }

    return 0u;
}

static void startup_nrf_send_start_cmd_if_base(void)
{
#if defined(BASE_CAR_3_4) || defined(BASE_CAR_4_4)
    uint8_t tx_payload[TX_PLOAD_WIDTH] = {0};

    if (!s_nrf_ready)
    {
        return;
    }

    memcpy(tx_payload, STARTUP_CMD_TEXT, STARTUP_CMD_LEN);

    NRF24L01_TX_Mode();
    NRF24L01_TxPacket(tx_payload);
    NRF24L01_TxPacket(tx_payload);
    NRF24L01_TxPacket(tx_payload);
    NRF24L01_RX_Mode();
    base_car_startup_strategy_run();
#endif
}

void startup_strategy_run(void)
{
    uint8_t strategy_index = 0u;

    startup_nrf_init();
    startup_show_strategy(strategy_index);

    while (!should_start)
    {
        if (startup_button_pressed_once(BUTTON_PC0))
        {
            strategy_index = (strategy_index == 0u) ? (uint8_t)(STARTUP_STRATEGY_COUNT - 1u) : (uint8_t)(strategy_index - 1u);
            startup_show_strategy(strategy_index);
        }
        else if (startup_button_pressed_once(BUTTON_PC1))
        {
            strategy_index = (uint8_t)((strategy_index + 1u) % STARTUP_STRATEGY_COUNT);
            startup_show_strategy(strategy_index);
        }
        else if (startup_button_pressed_once(BUTTON_PC3))
        {
            startup_nrf_send_start_cmd_if_base();
            should_start = 1u;
            
        }
        else if (startup_nrf_has_start_cmd())
        {
            should_start = 1u;
        }

        delay_20ms(1);
    }

    OLED_ShowString(0, 0, "Starting", OLED_8X16);
    OLED_ShowString(0, 16, (char *)g_startup_strategies[strategy_index].name, OLED_8X16);

    g_startup_strategies[strategy_index].run();
}

uint8_t is_start_successful(void)
{
    return should_start;
}
