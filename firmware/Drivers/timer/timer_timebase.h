#ifndef TIMER_TIMEBASE_H
#define TIMER_TIMEBASE_H

#include <stdint.h>
#include "stm32f446xx.h"

void timer2_timebase_init(void);
uint32_t micros(void);

#endif
