#include "iwdg.h"

void iwdg_init(void)
{
    // 1. Unlock PR and RLR by writing 0x5555 to KR
	IWDG->KR = (0x5555 << 0);

    // 2. Set prescaler in PR (divide LSI by 32 → 1kHz tick)
    IWDG->PR = (3 << 0);

	// 3. Set RLR reload value for ~500ms timeout
    IWDG->RLR = (500 << 0);

	// 4. Start IWDG by writing 0xCCCC to KR
    IWDG->KR = (0xCCCC << 0);

}

void iwdg_kick(void)
{
    // 1. Write 0xAAAA to KR to reload counter
	IWDG->KR = (0xAAAA << 0);
}
