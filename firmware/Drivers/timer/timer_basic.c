#include "timer_basic.h"



void tim7_init(void)
{
    // 1. Enable TIM7 clock in RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // 2. Set PSC to 1799
    //    (90MHz / (1799+1) = 50kHz timer clock)
	TIM7->PSC = 1799;

    // 3. Set ARR to 999
    //    (50kHz / (999+1) = 50Hz overflow)
	TIM7->ARR = 999;

    // 4. Enable Update Interrupt in DIER (UIE bit)
	TIM7->DIER |= TIM_DIER_UIE;

    // 5. Set NVIC priority for TIM7_IRQn to 0
	NVIC_SetPriority(TIM7_IRQn, 0);

    // 6. Enable TIM7_IRQn in NVIC
	NVIC_EnableIRQ(TIM7_IRQn);

    // 7. Enable the timer by setting CEN in CR1
	TIM7->CR1 |= TIM_CR1_CEN;
}
