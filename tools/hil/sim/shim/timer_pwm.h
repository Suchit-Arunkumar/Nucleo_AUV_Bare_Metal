/* Host build only: the control loop calls pwm_set_us(); the simulator
 * records the values instead of driving timers. */
#ifndef SIM_TIMER_PWM_H
#define SIM_TIMER_PWM_H

#include <stdint.h>

void pwm_set_us(uint8_t channel, uint16_t us);

#endif
