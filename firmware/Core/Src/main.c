#include <stdio.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"

int main(void)
{
    system_clock_init();

    systick_init();

    gpio_init(GPIOA, 5);

    uart2_init();
    printf("BOOT OK\r\n");



    while(1)
    {
        gpio_toggle(GPIOA, 5);
        printf("tick=%lu\r\n", g_tick);
        delay_ms(500);
    }
}
