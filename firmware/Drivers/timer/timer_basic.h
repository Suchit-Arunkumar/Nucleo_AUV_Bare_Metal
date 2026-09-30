#ifndef TIMER_BASIC_H
#define TIMER_BASIC_H

#include <stdint.h>
#include "stm32f446xx.h"

void tim7_init(void);

// TIM7 tick probe: toggled once per 50 Hz tick by TIM7_IRQHandler, so the pin
// carries a 25 Hz square wave -- scope it to see tick rate and ISR jitter.
// PB10 (Arduino D6): nothing else in this firmware configures PB10, and it has
// no debug/JTAG role. Not LD2: LD2 is hard-wired to PA5, which is SPI1_SCK.
#define TICK_PROBE_PORT  GPIOB
#define TICK_PROBE_PIN   10U

#endif
