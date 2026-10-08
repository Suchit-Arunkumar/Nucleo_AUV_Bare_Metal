#ifndef BENCH_H
#define BENCH_H

/*
 * BENCH_HIL - measurements for tools/hil/hil_test.py on a bare Nucleo.
 *
 * 1: DWT timing of the TIM7 ISR and the main loop, MSP stack painting, PWM measured on PA15 by TIM2_CH1 input
 *   capture, an MPU-6050 sanity read on I2C1, the reset cause, and two
 *   commands carried in TYPE_PID frames ("BENCH:HANG", "BENCH:SIG1"/"SIG0").
 *   Results are printed in the 500 ms status cadence.
 *
 *   BENCH:SIG1 drives the ESC outputs off neutral and BENCH:HANG stops the
 *   watchdog refresh. Never flash a BENCH_HIL build to a vehicle with ESCs.
 *
 * 0 (default): none of it is compiled. The default build is the vehicle
 *   build; the bench build is opt-in: -DBENCH_HIL=1 -DLINK_PORT_STLINK=1.
 */
#ifndef BENCH_HIL
#define BENCH_HIL 0
#endif

#include <stdint.h>
#include <stdbool.h>

#if BENCH_HIL

void bench_stack_paint(void);              /* first thing in main()             */
void bench_init(void);                     /* after timer2_timebase_init()      */
void bench_mpu_init(void);                 /* after i2c1_init()                 */
void bench_tim7_entry(void);               /* TIM7_IRQHandler, first statement  */
void bench_tim7_exit(void);                /* TIM7_IRQHandler, last statement   */
void bench_loop_top(void);                 /* top of the main loop              */
void bench_tx_begin(void);
void bench_tx_end(void);
void bench_status(void);                   /* with the 500 ms status line       */
void bench_mpu_poll(uint32_t now_ms);      /* main loop, staggered from status  */
void bench_pid_frame(const uint8_t *payload); /* packet.c, valid TYPE_PID frame */
bool bench_hang_requested(void);

#else

#define bench_stack_paint()      ((void)0)
#define bench_init()             ((void)0)
#define bench_mpu_init()         ((void)0)
#define bench_tim7_entry()       ((void)0)
#define bench_tim7_exit()        ((void)0)
#define bench_loop_top()         ((void)0)
#define bench_tx_begin()         ((void)0)
#define bench_tx_end()           ((void)0)
#define bench_status()           ((void)0)
#define bench_mpu_poll(t)        ((void)0)
#define bench_pid_frame(p)       ((void)0)
#define bench_hang_requested()   (false)

#endif

#endif /* BENCH_H */
