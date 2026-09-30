/* Host build only: stands in for the CMSIS device header so the firmware's
 * protocol and control-loop sources compile on a PC. Nothing here touches
 * hardware. The simulator is single-threaded, so the interrupt-masking
 * intrinsics used by ring_buffer.c are no-ops. */
#ifndef SIM_STM32F446XX_H
#define SIM_STM32F446XX_H

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void)        { return 0; }
static inline void     __set_PRIMASK(uint32_t v)  { (void)v; }
static inline void     __disable_irq(void)        { }
static inline void     __enable_irq(void)         { }

#endif
