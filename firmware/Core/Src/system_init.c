#include "system_init.h"

volatile uint32_t g_tick = 0;

void system_clock_init(void){

	//Enable HSE
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	//Wait for HSE to become stable
	while(!(RCC->CR & RCC_CR_HSERDY ))
	{

	}

	/* Flash configuration for 180 MHz */
	    FLASH->ACR =
	        FLASH_ACR_ICEN |
	        FLASH_ACR_DCEN |
	        FLASH_ACR_PRFTEN |
	        FLASH_ACR_LATENCY_5WS;

	    /* PLL Configuration
	       HSE = 8 MHz
	       PLLM = 8
	       PLLN = 360
	       PLLP = 2

	       SYSCLK = 180 MHz
	    */

	// m = 8
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
	// n = 360
	RCC->PLLCFGR |= (360 << RCC_PLLCFGR_PLLN_Pos);
	// p = 2
	RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);
	// src = hse
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;



	//enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	// APB1 at 45Mhz
	RCC->CFGR = RCC_CFGR_PPRE1_DIV4;
	// APBH2 at 90Mhz
	RCC->CFGR = RCC_CFGR_PPRE2_DIV2;

	// Switch cpu clk source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{

	}

}

void systick_init(void)
{
    SysTick->LOAD = 180000 - 1;

    SysTick->VAL = 0;


    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = g_tick;

    while((g_tick - start) < ms);
}
