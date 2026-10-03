/*
 * bench.c - measurements for the hardware-in-the-loop test on a bare Nucleo.
 * See bench.h. The laptop side is tools/hil/hil_test.py.
 *
 * Every number is measured by the chip: the DWT cycle counter (PM0214) runs
 * at HCLK, so 180 counts = 1 us, and TIM2 counts 1 us ticks. Lines printed,
 * one per 500 ms status window, in rotation:
 *
 *   perf  t7=<min>-<max> isr=<max>/<avg> loop=<max>/<avg> tx=<max>   (cycles)
 *   stack peak=<bytes> reserve=<bytes>
 *   mpu   who=0x68 n=<cycles> err=<n> stale=<n> i2cerr=<n> t14=<us>   or "mpu absent"
 *   pwm   n=<pulses> hi=<min>-<max> per=<min>-<max> sig=<0|1>       (us, PA15)
 *
 * With the PWM signature on, the pwm line is printed every window instead.
 */
#include "bench.h"

#if BENCH_HIL

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "system_init.h"
#include "timer_pwm.h"
#include "timer_timebase.h"
#include "i2c.h"

static inline uint32_t cyc(void) { return DWT->CYCCNT; }

typedef struct
{
    uint32_t n, min, max;
    uint64_t sum;
} Stat;

static void stat_reset(volatile Stat *s) { s->n = 0; s->min = 0xFFFFFFFFUL; s->max = 0; s->sum = 0; }

static inline void stat_add(volatile Stat *s, uint32_t v)
{
    s->n++;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
}

static void snap(const volatile Stat *src, Stat *dst)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    dst->n = src->n; dst->min = src->min; dst->max = src->max; dst->sum = src->sum;
    __set_PRIMASK(primask);
    if (dst->n == 0U) dst->min = 0;
}

static uint32_t avg(const Stat *s) { return s->n ? (uint32_t)(s->sum / s->n) : 0U; }

/* ---- TIM7 ISR: period (entry to entry) and duration ---------------------- */
static volatile uint32_t t7_entry, t7_prev;
static volatile uint8_t  t7_have_prev;
static volatile Stat     st_t7, st_isr;

void bench_tim7_entry(void)
{
    uint32_t now = cyc();
    if (t7_have_prev) stat_add(&st_t7, now - t7_prev);
    t7_prev = now;
    t7_entry = now;
    t7_have_prev = 1U;
}

static volatile uint8_t pwm_signature;

void bench_tim7_exit(void)
{
    if (pwm_signature)
    {
        /* Channel k at 1100 + 100k us, overriding the control loop's write
         * this tick: a jumper on any ESC pin then identifies its channel. */
        for (uint8_t ch = 0; ch < 8U; ch++)
        {
            pwm_set_us(ch, (uint16_t)(1100U + 100U * ch));
        }
    }
    stat_add(&st_isr, cyc() - t7_entry);
}

/* ---- main loop ------------------------------------------------------------ */
static uint32_t loop_prev, tx_start;
static uint8_t  loop_have_prev;
static Stat     st_loop, st_tx;                 /* main loop only */

void bench_loop_top(void)
{
    uint32_t now = cyc();
    if (loop_have_prev) stat_add(&st_loop, now - loop_prev);
    loop_prev = now;
    loop_have_prev = 1U;
}

void bench_tx_begin(void) { tx_start = cyc(); }
void bench_tx_end(void)   { stat_add(&st_tx, cyc() - tx_start); }

/* ---- stack painting -------------------------------------------------------
 * main() runs on the MSP from _estack down. Everything below the current SP
 * down to 2 KB under the top is unused at this point (the heap grows up from
 * _end, far below), so paint it and later find the lowest word touched. */
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;
#define PAINT       0xA5A5A5A5UL
#define PAINT_SPAN  2048U

static uint32_t *paint_base;

void bench_stack_paint(void)
{
    uint32_t top = (uint32_t)&_estack;
    paint_base   = (uint32_t *)(top - PAINT_SPAN);
    uint32_t *sp = (uint32_t *)(__get_MSP() - 64U);
    for (uint32_t *p = paint_base; p < sp; p++)
    {
        *p = PAINT;
    }
}

static uint32_t stack_peak(void)
{
    uint32_t *p   = paint_base;
    uint32_t *top = (uint32_t *)&_estack;
    while ((p < top) && (*p == PAINT)) p++;
    return (uint32_t)top - (uint32_t)p;
}

/* ---- PWM on PA15: TIM2_CH1 input capture, both edges ---------------------- */
static volatile uint32_t cap_rise;
static volatile uint8_t  cap_have_rise;
static volatile Stat     st_hi, st_per;

void TIM2_IRQHandler(void)
{
    uint32_t sr = TIM2->SR;
    if (sr & TIM_SR_CC1IF)
    {
        uint32_t c    = TIM2->CCR1;               /* clears CC1IF */
        uint32_t high = GPIOA->IDR & (1U << 15);
        if (high)
        {
            if (cap_have_rise) stat_add(&st_per, c - cap_rise);
            cap_rise = c;
            cap_have_rise = 1U;
        }
        else if (cap_have_rise)
        {
            stat_add(&st_hi, c - cap_rise);
        }
    }
    if (sr & TIM_SR_CC1OF) TIM2->SR = ~TIM_SR_CC1OF;
}

static void cap_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    stat_reset(&st_hi);
    stat_reset(&st_per);
    cap_have_rise = 0U;
    __set_PRIMASK(primask);
}

void bench_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    stat_reset(&st_t7);  stat_reset(&st_isr);
    stat_reset(&st_loop); stat_reset(&st_tx);
    stat_reset(&st_hi);  stat_reset(&st_per);

    /* PA15 resets as JTDI: AF0 with a pull-up. Clear both, then AF1. */
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER  &= ~(3U << (2 * 15));
    GPIOA->MODER  |=  (2U << (2 * 15));
    GPIOA->PUPDR  &= ~(3U << (2 * 15));
    GPIOA->PUPDR  |=  (2U << (2 * 15));          /* pull-down: open = low */
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (1U   << ((15 - 8) * 4));

    /* TIM2 is already the free-running 1 MHz micros() counter; capture only
     * copies CNT into CCR1 and leaves the count alone. */
    TIM2->CCER  &= ~TIM_CCER_CC1E;
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC);
    TIM2->CCMR1 |=  (1U << TIM_CCMR1_CC1S_Pos) | (3U << TIM_CCMR1_IC1F_Pos);
    TIM2->CCER  |=  TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E;
    TIM2->SR     = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
    TIM2->DIER  |=  TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM2_IRQn, 2);              /* below TIM7 (0), USART2 (1) */
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* ---- MPU-6050 on I2C1 ------------------------------------------------------ */
#define MPU_ADDR  0x68U
static uint8_t  mpu_present, mpu_who;
static uint32_t mpu_n, mpu_err, mpu_stale, mpu_i2cerr, mpu_t14_max;
static uint32_t mpu_next;

static int mpu_reg(uint8_t reg, uint8_t *buf, uint8_t n)
{
    if (i2c_write(MPU_ADDR, &reg, 1U) != 0) return -1;
    if (I2C1->SR1 & I2C_SR1_RXNE) mpu_stale++;   /* byte left over from the last read */
    return i2c_read(MPU_ADDR, buf, n);
}

static int16_t be16(const uint8_t *b) { return (int16_t)(((uint16_t)b[0] << 8) | b[1]); }

static bool accel_ok(const uint8_t *b)
{
    int64_t x = be16(&b[0]), y = be16(&b[2]), z = be16(&b[4]);
    int64_t m2 = x * x + y * y + z * z;
    return (m2 >= 9830LL * 9830LL) && (m2 <= 22938LL * 22938LL);   /* 0.6 .. 1.4 g */
}

static bool temp_ok(const uint8_t *b)
{
    int32_t raw = be16(b);                                           /* T = raw/340 + 36.53 */
    return (raw > -47 * 340) && (raw < 33 * 340);
}

void bench_mpu_init(void)
{
    uint8_t wake[2] = { 0x6BU, 0x00U };
    if (i2c_write(MPU_ADDR, wake, 2U) == 0)
    {
        delay_ms(50);
        if (mpu_reg(0x75U, &mpu_who, 1U) == 0)
        {
            mpu_present = 1U;
        }
    }
    printf(mpu_present ? "MPU 0x%02X\r\n" : "MPU ABSENT\r\n", mpu_who);
    mpu_next = g_tick + 250U;                   /* half a window off the status line */
}

void bench_mpu_poll(uint32_t now_ms)
{
    if (!mpu_present || (int32_t)(now_ms - mpu_next) < 0) return;
    mpu_next += 500U;

    uint8_t b[14];
    uint8_t who = 0;

    if (mpu_reg(0x75U, &who, 1U) != 0)       mpu_i2cerr++;
    else if (who != mpu_who)                  mpu_err++;

    if (mpu_reg(0x41U, b, 2U) != 0)           mpu_i2cerr++;
    else if (!temp_ok(b))                     mpu_err++;

    if (mpu_reg(0x3BU, b, 6U) != 0)           mpu_i2cerr++;
    else if (!accel_ok(b))                    mpu_err++;

    uint32_t t0 = micros();
    if (mpu_reg(0x3BU, b, 14U) != 0)          mpu_i2cerr++;
    else
    {
        uint32_t dt = micros() - t0;
        if (dt > mpu_t14_max) mpu_t14_max = dt;
        if (!accel_ok(b) || !temp_ok(&b[6]))  mpu_err++;
    }
    mpu_n++;
}

/* ---- bench commands over TYPE_PID frames -------------------------------- */
static volatile uint8_t hang_req;

void bench_pid_frame(const uint8_t *payload)
{
    if (memcmp(payload, "BENCH:HANG", 10) == 0)      hang_req = 1U;
    else if (memcmp(payload, "BENCH:SIG1", 10) == 0) { pwm_signature = 1U; cap_reset(); }
    else if (memcmp(payload, "BENCH:SIG0", 10) == 0) { pwm_signature = 0U; cap_reset(); }
}

bool bench_hang_requested(void) { return hang_req != 0U; }

/* ---- status output -------------------------------------------------------- */
static void print_pwm(void)
{
    Stat hi, per;
    snap(&st_hi, &hi);
    snap(&st_per, &per);
    printf("pwm n=%lu hi=%lu-%lu per=%lu-%lu sig=%u\r\n",
           (unsigned long)hi.n, (unsigned long)hi.min, (unsigned long)hi.max,
           (unsigned long)per.min, (unsigned long)per.max, (unsigned)pwm_signature);
    cap_reset();
}

void bench_status(void)
{
    static uint8_t k;

    if (pwm_signature)
    {
        print_pwm();
        return;
    }

    switch (k++ & 3U)
    {
        case 0:
        {
            Stat t7, isr;
            snap(&st_t7, &t7);
            snap(&st_isr, &isr);
            printf("perf t7=%lu-%lu isr=%lu/%lu loop=%lu/%lu tx=%lu\r\n",
                   (unsigned long)t7.min, (unsigned long)t7.max,
                   (unsigned long)isr.max, (unsigned long)avg(&isr),
                   (unsigned long)st_loop.max, (unsigned long)avg(&st_loop),
                   (unsigned long)st_tx.max);
            break;
        }
        case 1:
            printf("stack peak=%lu reserve=%lu\r\n",
                   (unsigned long)stack_peak(), (unsigned long)(uint32_t)&_Min_Stack_Size);
            break;
        case 2:
            if (mpu_present)
                printf("mpu who=0x%02X n=%lu err=%lu stale=%lu i2cerr=%lu t14=%lu\r\n",
                       mpu_who, (unsigned long)mpu_n, (unsigned long)mpu_err,
                       (unsigned long)mpu_stale, (unsigned long)mpu_i2cerr,
                       (unsigned long)mpu_t14_max);
            else
                printf("mpu absent\r\n");
            break;
        default:
            print_pwm();
            break;
    }
}

#endif /* BENCH_HIL */
