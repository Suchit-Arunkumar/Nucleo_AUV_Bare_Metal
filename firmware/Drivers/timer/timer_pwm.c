#include "timer_pwm.h"

// =============================================================================
// timer_pwm.c -- see timer_pwm.h for the channel/pin map and timer choice.
// =============================================================================

#define PWM_TIMER_CLK_HZ  90000000U                        // APB1 timer clock
#define PWM_TICK_HZ       1000000U                         // 1 tick = 1 us
#define PWM_PSC           (PWM_TIMER_CLK_HZ / PWM_TICK_HZ - 1U)   // 89

_Static_assert(PWM_TIMER_CLK_HZ % PWM_TICK_HZ == 0U, "timer clock is not a whole number of MHz");
_Static_assert(PWM_PERIOD_US - 1U <= 0xFFFFU, "ARR must fit a 16-bit timer");
_Static_assert(PWM_MAX_US < PWM_PERIOD_US, "max pulse must be shorter than the frame");

// channel index -> capture/compare register. Order is the thruster order.
static volatile uint32_t * const pwm_ccr[PWM_NUM_CHANNELS] = {
    &TIM3->CCR1,  &TIM3->CCR2,  &TIM3->CCR3,  &TIM3->CCR4,    // T1-T4
    &TIM4->CCR1,  &TIM4->CCR2,                                 // T5-T6
    &TIM12->CCR1, &TIM12->CCR2,                                // T7-T8
};

// -----------------------------------------------------------------------------
// Put one pin into alternate-function mode. Every field is cleared before it is
// set: these ports are shared with SPI1, I2C1, USART and the OLED, and OR-ing an
// AF number onto a field that already holds one is exactly how PA6 ended up at
// AF7 (AF5 | AF2). AFR is written before MODER so the pin never passes through
// the wrong AF on its way into AF mode.
// -----------------------------------------------------------------------------
static void pin_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    uint32_t afr_shift = (pin & 7U) * 4U;
    uint32_t two_bit   = pin * 2U;

    port->AFR[pin >> 3] &= ~(0xFU << afr_shift);
    port->AFR[pin >> 3] |=  (af   << afr_shift);

    port->OTYPER  &= ~(1U << pin);            // push-pull
    port->OSPEEDR &= ~(3U << two_bit);        // low speed: 50 Hz edges, least EMI
    port->PUPDR   &= ~(3U << two_bit);
    port->PUPDR   |=  (2U << two_bit);        // pull-down: line idles low, never floats

    port->MODER   &= ~(3U << two_bit);
    port->MODER   |=  (2U << two_bit);        // AF mode last
}

// -----------------------------------------------------------------------------
// Configure one timer for n_ch channels (2 or 4) of edge-aligned PWM mode 1,
// start it with every CCR at 0 (output held low), and leave it running.
// -----------------------------------------------------------------------------
static void pwm_timer_start(TIM_TypeDef *tim, uint32_t n_ch)
{
    // PWM mode 1 (OCxM = 110) with CCR preload (OCxPE), output direction
    // (CCxS = 00). The same bit pattern sits at bit 0 for CH1/CH3 and bit 8
    // for CH2/CH4.
    const uint32_t oc = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    tim->CR1   = 0U;                          // stopped, up-counting, edge-aligned
    tim->PSC   = PWM_PSC;
    tim->ARR   = PWM_PERIOD_US - 1U;

    tim->CCMR1 = oc | (oc << 8);              // CH1 + CH2
    tim->CCR1  = 0U;
    tim->CCR2  = 0U;
    uint32_t ccer = TIM_CCER_CC1E | TIM_CCER_CC2E;

    if (n_ch == 4U) {                         // TIM12 has no CH3/CH4: leave its
        tim->CCMR2 = oc | (oc << 8);          // reserved CCMR2/CCR3/CCR4 alone
        tim->CCR3  = 0U;
        tim->CCR4  = 0U;
        ccer |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    }
    tim->CCER  = ccer;                        // active-high outputs enabled

    tim->CR1   = TIM_CR1_ARPE;
    tim->EGR   = TIM_EGR_UG;                  // load PSC/ARR/CCR shadows now; without
                                              // this the first period runs at PSC = 0
    tim->SR    = 0U;                          // drop the UIF that UG just set
    tim->CR1  |= TIM_CR1_CEN;
}

void pwm_init(void)
{
    // 1. Clocks. The read-backs give the enable the cycles it needs before
    //    the first register access (ES0298, "delay after an RCC peripheral
    //    clock enabling").
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM12EN;
    (void)RCC->APB1ENR;

    // 2. Timers running, every output held low (CCR = 0).
    pwm_timer_start(TIM3,  4U);
    pwm_timer_start(TIM4,  2U);
    pwm_timer_start(TIM12, 2U);

    // 3. Hand the pins to the timers. Because every output is currently low,
    //    switching a pin mid-period cannot produce a short partial pulse.
    pin_af(GPIOC,  6U, 2U);   // TIM3_CH1   T1
    pin_af(GPIOB,  5U, 2U);   // TIM3_CH2   T2
    pin_af(GPIOC,  8U, 2U);   // TIM3_CH3   T3
    pin_af(GPIOC,  9U, 2U);   // TIM3_CH4   T4
    pin_af(GPIOB,  6U, 2U);   // TIM4_CH1   T5
    pin_af(GPIOB,  7U, 2U);   // TIM4_CH2   T6
    pin_af(GPIOB, 14U, 9U);   // TIM12_CH1  T7
    pin_af(GPIOB, 15U, 9U);   // TIM12_CH2  T8

    // 4. Neutral on every channel. CCR is preloaded, so each timer's next
    //    period starts with a full, clean 1500 us pulse.
    for (uint8_t ch = 0U; ch < PWM_NUM_CHANNELS; ch++) {
        pwm_set_us(ch, (uint16_t)PWM_NEUTRAL_US);
    }
}

void pwm_set_us(uint8_t channel, uint16_t us)
{
    if (channel >= PWM_NUM_CHANNELS) {
        return;
    }
    if (us < PWM_MIN_US) {
        us = (uint16_t)PWM_MIN_US;
    } else if (us > PWM_MAX_US) {
        us = (uint16_t)PWM_MAX_US;
    }
    *pwm_ccr[channel] = us;
}
