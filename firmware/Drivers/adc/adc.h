#ifndef ADC_H
#define ADC_H

#include "stm32f446xx.h"
#include <stdint.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel);

#endif
