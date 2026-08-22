#include "system_init.h"

#define SYSCLK_HZ 180000000U

volatile uint32_t g_tick = 0;

void system_clock_init(void){

	/* 1. HSE bypass, enable, wait for stable */
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	while(!(RCC->CR & RCC_CR_HSERDY ))
	{

	}

	/* 2. PWR clock — PWR is on APB1, must be clocked before its registers
	      are writable (over-drive lives in PWR->CR / PWR->CSR) */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* 3. PWR->CR VOS is left at its reset value (Scale 1) — not written.
	      See report. */

	/* 4. Flash configuration for 180 MHz — 5 wait states */
	FLASH->ACR =
	    FLASH_ACR_ICEN |
	    FLASH_ACR_DCEN |
	    FLASH_ACR_PRFTEN |
	    FLASH_ACR_LATENCY_5WS;

	/* 5. PLL Configuration
	      HSE = 8 MHz
	      PLLM = 8
	      PLLN = 360
	      PLLP = 2   (PLLP field = 0)

	      SYSCLK = 180 MHz

	      Clear-then-set: PLLCFGR resets to 0x24003010, so |= alone merges
	      new fields into the reset values instead of replacing them. */
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
	                  RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
	RCC->PLLCFGR |=  (8U   << RCC_PLLCFGR_PLLM_Pos) |
	                 (360U << RCC_PLLCFGR_PLLN_Pos) |
	                 (0U   << RCC_PLLCFGR_PLLP_Pos) |
	                 RCC_PLLCFGR_PLLSRC_HSE;

	/* 6. Enable PLL, wait for lock */
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}

	/* 7. Over-drive — required above 168 MHz. Enabled after PLL lock but
	      before the SYSCLK switch, since the switch puts the core at 180. */
	PWR->CR |= PWR_CR_ODEN;
	while (!(PWR->CSR & PWR_CSR_ODRDY)) {}
	PWR->CR |= PWR_CR_ODSWEN;
	while (!(PWR->CSR & PWR_CSR_ODSWRDY)) {}

	/* 8. Bus prescalers before the switch: APB1 45 MHz, APB2 90 MHz */
	RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	/* 9. Switch SYSCLK to PLL, confirm via SWS read-back */
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

}

void systick_init(void)
{
	/* 10. 1 ms tick from HCLK */
	SysTick->LOAD = (SYSCLK_HZ / 1000U) - 1U;
	SysTick->VAL  = 0U;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
	                SysTick_CTRL_TICKINT_Msk   |
	                SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = g_tick;

    while((g_tick - start) < ms);
}
