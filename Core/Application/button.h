#ifndef __BUTTON_H
#define __BUTTON_H

#include "stdint.h"
#include "stm32f1xx_hal.h"

typedef enum {
    BUTTON_PC0 = 0,
    BUTTON_PC1,
    BUTTON_PC2,
    BUTTON_PC3,
    BUTTON_COUNT
} Button_t;

uint8_t Button_ReadRaw(Button_t btn);       // 直接读取引脚
uint8_t Button_IsPressed(Button_t btn);    // 去抖后判断按下
uint8_t Button_ReadAll(void);              // 返回低4位按钮状态

#endif // __BUTTON_H
