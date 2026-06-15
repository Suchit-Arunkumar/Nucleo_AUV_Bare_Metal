#include <stdio.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"
#include "spi.h"
#include "oled.h"
#include "sd_card.h"
#include "timer_basic.h"
#include "control_loop.h"
#include "adc.h"
#include "dac.h"


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

    spi1_init();
    SD_Status sd_status = sd_init();
    if (sd_status == SD_OK) {
        printf("SD OK\r\n");
    }
    else {
        printf("SD FAIL\r\n");
    }

    oled_init();
    oled_draw_string(0, 0, "ROV OK");
    oled_update();

    adc_init();
    dac_init();

    tim7_init();
    control_loop_init();

    while(1)
    {
        // 1. Read shared variable safely with IRQ guard
        __disable_irq();
        // read link_ok or any other shared state here
        bool local_link = link_ok;

        __enable_irq();

        // 2. Print status over UART every 500ms using g_tick
        //    (non-blocking — compare g_tick, don't use delay_ms)
        static uint32_t last_print = 0;
        if (g_tick - last_print >= 500) {
            last_print = g_tick;
            printf("tick=%lu link=%d\r\n", g_tick, local_link);
        }
    }

}
