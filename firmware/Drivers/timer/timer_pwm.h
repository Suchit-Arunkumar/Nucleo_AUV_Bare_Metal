#ifndef TIMER_PWM_H
#define TIMER_PWM_H

#include <stdint.h>
#include "stm32f446xx.h"

void timer3_pwm_init(void);
void pwm_set_us(uint8_t channel, uint16_t us);

#endif
