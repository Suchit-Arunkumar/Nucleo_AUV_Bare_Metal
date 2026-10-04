#include "iwdg.h"
#include "system_init.h"   // g_tick, for the bounded wait

/* T = (RLR+1) * PRESCALER / f_LSI
 * PR=3 -> /32, RLR=500
 * f_LSI = 47 kHz (max): T = 341 ms  <- kick interval designed against this
 * f_LSI = 32 kHz (typ): T = 501 ms
 * f_LSI = 17 kHz (min): T = 943 ms  <- worst-case detection latency
 */
#define IWDG_PR_VALUE    3U
#define IWDG_RLR_VALUE   500U
#define IWDG_CFG_WAIT_MS 50U    // PVU/RVU clear in ~5 LSI cycles: < 0.3 ms

/*
 * Order matters, and the old one hung on silicon.
 *
 * The previous sequence unlocked the registers, wrote PR and then spun on
 * SR.PVU with no timeout, and only started the watchdog at the very end.
 * On the first hardware run (tools/hil) boot stopped right here, every
 * time, with no reset: PR/RLR are written into the LSI clock domain, and
 * the update did not complete while the watchdog was not yet running, so
 * PVU never cleared and the never-started watchdog could not rescue it.
 *
 * This is the order ST's own HAL_IWDG_Init uses: start the counter (which
 * also forces the LSI on), then unlock, write PR and RLR, wait for both
 * update flags to clear, and reload. While the new values propagate the
 * reset defaults apply (/4, RLR 0xFFF: at least 348 ms at the 47 kHz LSI
 * extreme), far longer than this function takes. The wait is bounded, so
 * a failure is reported instead of hanging boot.
 *
 * Returns 0 on success, -1 if the update flags did not clear in time (the
 * watchdog then runs with whatever values did latch, default or new).
 */
int iwdg_init(void)
{
    IWDG->KR = 0xCCCCU;                 // start; LSI is enabled by hardware
    IWDG->KR = 0x5555U;                 // unlock PR and RLR
    IWDG->PR  = IWDG_PR_VALUE;
    IWDG->RLR = IWDG_RLR_VALUE;

    uint32_t start = g_tick;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU))
    {
        if ((uint32_t)(g_tick - start) > IWDG_CFG_WAIT_MS)
        {
            IWDG->KR = 0xAAAAU;
            return -1;
        }
    }

    IWDG->KR = 0xAAAAU;                 // reload with the new RLR; relocks
    return 0;
}

void iwdg_kick(void)
{
    IWDG->KR = 0xAAAAU;
}
