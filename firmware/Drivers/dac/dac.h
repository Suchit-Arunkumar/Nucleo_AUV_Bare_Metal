#ifndef DAC_H
#define DAC_H

#include "stm32f446xx.h"
#include <stdint.h>

void dac_init(void);
void dac_set(uint16_t value);

#endif
