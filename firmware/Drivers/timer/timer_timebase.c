#include "timer_timebase.h"

void timer2_timebase_init(void)
{
    // 1. Enable TIM2 clock in RCC_APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 2. Set PSC to get 1MHz tick (90MHz / 90 = 1MHz, so PSC = 89)
	TIM2->PSC &= ~(0xFFFFU << 0);
	TIM2->PSC |= (89U << 0);

    // 3. Set ARR to max value (TIM2 is 32-bit, so 0xFFFFFFFF)
	TIM2->ARR &= ~(0xFFFFFFFFU);
	TIM2->ARR |= (0xFFFFFFFFU);

    // 4. Set CR1 - clear counter direction (up), set CEN to start counter
	TIM2->CR1 |= TIM_CR1_CEN;


}

uint32_t micros(void){

	return(TIM2->CNT);

}
