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

    // 4. Load PSC now. PSC is preloaded: a written value only takes effect at
    //    the next update event, and with ARR = 0xFFFFFFFF counting at 90 MHz
    //    that is 47.7 s away. Until then micros() ran 90x fast. UG forces
    //    the update immediately (and zeroes CNT). Found on the first
    //    hardware run; the FreeRTOS tree had already fixed the same line.
	TIM2->EGR = TIM_EGR_UG;

    // 5. Set CR1 - clear counter direction (up), set CEN to start counter
	TIM2->CR1 |= TIM_CR1_CEN;


}

uint32_t micros(void){

	return(TIM2->CNT);

}
