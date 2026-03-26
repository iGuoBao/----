#include "button.h"
#include "main.h" // 包含 HAL_GPIO_ReadPin
#include "Delay.h" // 包含 Delay_ms

// 假定按键接地，使用上拉输入，按下为低。
static const GPIO_TypeDef* BUTTON_PORT[BUTTON_COUNT] = {
    GPIOC,
    GPIOC,
    GPIOC,
    GPIOC
};

static const uint16_t BUTTON_PIN[BUTTON_COUNT] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3
};

#define BUTTON_ACTIVE_STATE GPIO_PIN_RESET

void Button_Init(void)
{
    // GPIO 已由 MX_GPIO_Init 初始化，这里留空用于兼容调用。
}

uint8_t Button_ReadRaw(Button_t btn)
{
    if (btn >= BUTTON_COUNT) return 0;
    return HAL_GPIO_ReadPin((GPIO_TypeDef*)BUTTON_PORT[btn], BUTTON_PIN[btn]);
}

uint8_t Button_IsPressed(Button_t btn)
{
    if (btn >= BUTTON_COUNT) return 0;
    uint8_t state1 = Button_ReadRaw(btn);
    Delay_ms(10); // 简单的去抖延时
    uint8_t state2 = Button_ReadRaw(btn);
    return (state1 == state2) && (state1 == BUTTON_ACTIVE_STATE);
}

uint8_t Button_ReadAll(void)
{
    uint8_t state = 0;
    for (int i = 0; i < BUTTON_COUNT; ++i) {
        if (Button_IsPressed((Button_t)i)) {
            state |= (1u << i);
        }
    }
    return state;
}
