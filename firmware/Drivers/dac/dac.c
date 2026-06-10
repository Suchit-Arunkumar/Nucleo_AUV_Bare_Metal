#include "dac.h"

void dac_init(void)
{
    // 1. Enable DAC clock in RCC APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // 2. Enable GPIOA clock in RCC AHB1ENR (if not already enabled)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// 3. Configure PA4 as analog mode (MODER bits 9:8 = 11)
    GPIOA->MODER |= (3U << (2*4));

	// 4. Enable DAC Channel 1 (EN1 bit in DAC->CR)
    DAC->CR|= DAC_CR_EN1;

	// 5. Disable output buffer (BOFF1 bit in DAC->CR) — reduces glitches
    DAC->CR |= DAC_CR_BOFF1;


}

void dac_set(uint16_t value)
{
    // 1. Write value into DAC->DHR12R1 (bits 11:0, right-aligned)
	DAC->DHR12R1 = value;
}
