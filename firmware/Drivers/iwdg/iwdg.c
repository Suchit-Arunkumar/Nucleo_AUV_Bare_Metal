#include "iwdg.h"

/* T = (RLR+1) * PRESCALER / f_LSI
 * PR=3 -> /32, RLR=500
 * f_LSI = 47 kHz (max): T = 341 ms  <- kick interval designed against this
 * f_LSI = 32 kHz (typ): T = 501 ms
 * f_LSI = 17 kHz (min): T = 943 ms  <- worst-case detection latency
 */
#define IWDG_PR_VALUE   3U
#define IWDG_RLR_VALUE  500U

void iwdg_init(void)
{
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}
    IWDG->KR = 0x5555U;
    IWDG->PR = IWDG_PR_VALUE;
    while (IWDG->SR & IWDG_SR_PVU) {}
    IWDG->RLR = IWDG_RLR_VALUE;
    while (IWDG->SR & IWDG_SR_RVU) {}
    IWDG->KR = 0xCCCCU;
}

void iwdg_kick(void)
{
    IWDG->KR = 0xAAAAU;
}
