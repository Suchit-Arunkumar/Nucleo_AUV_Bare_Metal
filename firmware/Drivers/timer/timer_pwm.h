#ifndef TIMER_PWM_H
#define TIMER_PWM_H

// =============================================================================
// timer_pwm.h -- 8-channel 50 Hz ESC PWM for the STM32F446RE (LQFP64)
//
// Channel map (channel index = thruster index in control_loop.c, T1..T8):
//
//   ch  thruster  timer  chan   pin   AF   Nucleo header (UM1724, 64-pin)
//   --  --------  -----  ----   ----  ---  ------------------------------
//   0   T1 vert   TIM3   CH1    PC6   2    CN10-4
//   1   T2 vert   TIM3   CH2    PB5   2    CN10-29  (Arduino D4)
//   2   T3 vert   TIM3   CH3    PC8   2    CN10-2
//   3   T4 vert   TIM3   CH4    PC9   2    CN10-1
//   4   T5 horiz  TIM4   CH1    PB6   2    CN10-17  (Arduino D10)
//   5   T6 horiz  TIM4   CH2    PB7   2    CN7-21
//   6   T7 horiz  TIM12  CH1    PB14  9    CN10-28
//   7   T8 horiz  TIM12  CH2    PB15  9    CN10-26
//
// Why three timers and not two: on this package every other output pin of
// TIM3/TIM4/TIM1 is already taken or not bonded out.
//   TIM3  CH1 PA6 = SPI1_MISO, PB4 = NJTRST        -> PC6
//         CH2 PA7 = SPI1_MOSI, PC7 = OLED RES      -> PB5
//         CH3 PB0 = SD card CS                     -> PC8
//         CH4                                      -> PC9
//   TIM4  CH3/CH4 on PB8/PB9 = I2C1 (Bar30); PD12-15 not on LQFP64
//         -> only CH1/CH2 usable
//   TIM1  CH1 PA8 = OLED DC, CH2/CH3 PA9/PA10 = USART1 (Pi link);
//         PE9-14 not on LQFP64 -> only CH4 (PA11) usable
//   TIM12 CH1/CH2 on PB14/PB15, both free -> covers the last two
//
// TIM3, TIM4 and TIM12 are all on APB1: timer clock = 2 x 45 MHz = 90 MHz
// (PPRE1 = /4, DCKCFGR.TIMPRE = 0). One prescaler/period serves all three.
// TIM2 (micros timebase) and TIM7 (50 Hz control ISR) are not touched.
// =============================================================================

#include <stdint.h>
#include "stm32f446xx.h"

#define PWM_NUM_CHANNELS  8U

#define PWM_PERIOD_US     20000U   // 50 Hz frame
#define PWM_NEUTRAL_US    1500U
#define PWM_MIN_US        1000U    // hard driver limits; control_loop.c
#define PWM_MAX_US        2000U    // clamps tighter (1100..1900) itself

// Bring up TIM3, TIM4, TIM12 and all eight pins. Every channel leaves this
// function outputting 1500 us neutral, with no runt pulse on any pin.
void pwm_init(void);

// Set one channel's pulse width. channel is 0..7 (see map above).
// Out-of-range channels are ignored; us is clamped to PWM_MIN_US..PWM_MAX_US
// (a value >= PWM_PERIOD_US would otherwise hold the ESC line high).
// Takes effect at the start of that timer's next 20 ms period (CCR preload).
// Safe to call from the TIM7 ISR: each call is a single register store.
void pwm_set_us(uint8_t channel, uint16_t us);

// Legacy name, still called from main.c. Identical to pwm_init(); rename the
// call in main.c when convenient and delete this.
static inline void timer3_pwm_init(void) { pwm_init(); }

#endif
