#include <stdio.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"


int main(void)
{
    system_clock_init();

    systick_init();

    gpio_init(GPIOA, 5);

    uart2_init();
    printf("BOOT OK\r\n");

    // initialize USART1 with DMA RX and IDLE interrupt
    uart1_init();

    //  print confirmation message over UART2 that UART1 is up
    uart2_write_str("UART1 initialized\r\n");


    while(1)
    {
        gpio_toggle(GPIOA, 5);
        printf("tick=%lu\r\n", g_tick);
        delay_ms(500);
    }
}
