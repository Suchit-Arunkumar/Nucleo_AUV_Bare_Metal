#ifndef GPIO_H
#define GPIO_H

#include "stm32f4xx.h"

void gpio_init(GPIO_TypeDef *port, uint8_t pin);
void gpio_set(GPIO_TypeDef *port, uint8_t pin);
void gpio_clear(GPIO_TypeDef *port, uint8_t pin);
void gpio_toggle(GPIO_TypeDef *port, uint8_t pin);

#endif
