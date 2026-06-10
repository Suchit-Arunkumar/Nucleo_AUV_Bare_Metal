#include <stdio.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"
#include "spi.h"
#include "oled.h"
#include "sd_card.h"


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

    while(1)
    {
        gpio_toggle(GPIOA, 5);
        printf("tick=%lu\r\n", g_tick);
        delay_ms(500);
    }
}
