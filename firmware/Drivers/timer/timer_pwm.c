#include "timer_pwm.h"

void timer3_pwm_init(void)
{
    // 1. Enable TIM3 clock in RCC_APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 2. Enable GPIOA clock, configure PA6 as AF2 (TIM3_CH1)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER &= ~(3U << (6*2));
	GPIOA->MODER |= (2U << (6*2));
	GPIOA->AFR[0] |= (2 << (6 * 4));



    // 3. Set PSC = 89 (90MHz / 90 = 1MHz tick)
	TIM3->PSC = (89U << 0);

    // 4. Set ARR = 19999 (1MHz / 20000 = 50Hz)
	TIM3->ARR = (19999);

    // 5. Set CCMR1 - configure CH1 as PWM mode 1 (OC1M = 110)
	TIM3->CCMR1 |= (6U << 4);

    // 6. Enable CCMR1 output compare preload (OC1PE bit)
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    // 7. Set CCR1 = 1500 (1500us = neutral position)
	TIM3->CCR1 = 1500;

    // 8. Enable capture compare output in CCER (CC1E bit)
	TIM3->CCER |= TIM_CCER_CC1E;

    // 9. Set CR1 - enable auto reload preload (ARPE bit), then set CEN
	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void pwm_set_us(uint8_t channel, uint16_t us){

	TIM3->CCR1 = us;

}
