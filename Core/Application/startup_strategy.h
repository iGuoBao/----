#ifndef STARTUP_STRATEGY_H
#define STARTUP_STRATEGY_H

#include <string.h>

#include "GlobalLocalization.h"
#include "NRF24L01.h"
#include "OLED.h"
#include "action.h"
#include "button.h"
#include "car_type.h"
#include "plan.h"

void startup_strategy_run(void);
uint8_t is_start_successful(void);

#endif
