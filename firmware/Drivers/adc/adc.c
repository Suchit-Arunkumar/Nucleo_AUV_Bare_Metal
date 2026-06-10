#include "adc.h"

void adc_init(void)
{
    // 1. Enable ADC1 clock in RCC APB2ENR
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// 2. Enable GPIOA clock in RCC AHB1ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// 3. Configure PA0 as analog mode (MODER bits 1:0 = 11)
    GPIOA->MODER |= (3U << (2*0));

    // 4. Enable ADC1 (ADON bit in ADC1->CR2)
    ADC1->CR2 |= ADC_CR2_ADON;

}

uint16_t adc_read(uint8_t channel)
{
    // 1. Set the channel in ADC1->SQR3 (bits 4:0)
	ADC1->SQR3 = (channel << 0);

    // 2. Set sequence length to 1 in ADC1->SQR1 (L bits = 0000)
	ADC1->SQR1 &= ~(0xF << 20);

    // 3. Start conversion by setting SWSTART bit in ADC1->CR2
	ADC1->CR2 |= ADC_CR2_SWSTART;

    // 4. Poll EOC flag in ADC1->SR until set
	while(!(ADC1->SR & ADC_SR_EOC));

    // 5. Read and return ADC1->DR
	return(ADC1->DR);
}
