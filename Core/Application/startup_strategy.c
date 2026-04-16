#include "startup_strategy.h"

#include <string.h>

#include "GlobalLocalization.h"
#include "NRF24L01.h"
#include "OLED.h"
#include "action.h"
#include "button.h"
#include "car_type.h"
#include "plan.h"

#define STARTUP_CMD_TEXT "START"
#define STARTUP_CMD_LEN 5u

typedef void (*StartupRunFunc_t)(void);

typedef struct {
    const char *name;
    StartupRunFunc_t run;
} StartupStrategy_t;

static uint8_t s_nrf_ready = 0;
static uint8_t s_button_latched[BUTTON_COUNT] = {0};

static void startup_run_plan_4_a(void)
{
    plan_4_a();
}

static void startup_run_plan_4_b(void)
{
    plan_4_b();
}

static void startup_run_shovel_reset_loop(void)
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
    plan_shovel_patrol_loop();
}

static void startup_run_shovel_loop(void)
{
    plan_shovel_patrol_loop();
}

static void startup_test_servo_loop(void)
{
    route("OwKwtwwwwdwwwS");
}

static const StartupStrategy_t g_startup_strategies[] = {
    {"PLAN 4 A", startup_run_plan_4_a},
    {"PLAN 4 B", startup_run_plan_4_b},
    {"SHOVEL RST", startup_run_shovel_reset_loop},
    {"SHOVEL LP", startup_run_shovel_loop},
    {"TEST SERVO", startup_test_servo_loop},
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
    delay_20ms(75);
    motor_speed_set(100, 62); // 155   120
    delay_20ms(62);
    motor_speed_set(-7, -5);
    delay_20ms(300);
    motor_speed_set(0, 0);
    while (1);
#endif
}

void startup_strategy_run(void)
{
    uint8_t strategy_index = 0u;
    uint8_t should_start = 0u;

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

    OLED_Clear();
    OLED_ShowString(0, 0, "Starting", OLED_8X16);
    OLED_ShowString(0, 16, (char *)g_startup_strategies[strategy_index].name, OLED_8X16);
    OLED_Update();

    g_startup_strategies[strategy_index].run();
}
