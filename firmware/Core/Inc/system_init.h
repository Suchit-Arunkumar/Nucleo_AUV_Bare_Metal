#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "stm32f4xx.h"

extern volatile uint32_t g_tick;

void system_clock_init(void);
void systick_init(void);
void delay_ms(uint32_t ms);

#endif
