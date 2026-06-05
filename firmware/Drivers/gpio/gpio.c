#include "gpio.h"

void gpio_init(GPIO_TypeDef *port, uint8_t pin)
{
    if(port == GPIOA)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    if(port == GPIOB)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    if(port == GPIOC)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    port->MODER &= ~(3U << (pin * 2));
    port->MODER |=  (1U << (pin * 2));
}

void gpio_set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1U << pin);
}

void gpio_clear(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1U << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, uint8_t pin)
{
    port->ODR ^= (1U << pin);
}
