#include <stdio.h>
#include <string.h>
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
#include "struct.h"
#include "packet.h"
#include "sd_logger.h"
#include "crc_hw.h"
#include "i2c.h"
#include "bar30.h"
#include "timer_pwm.h"
#include "timer_timebase.h"
#include "iwdg.h"

static CommandPayload cmd;


int main(void)
{
    // 1. Configure system clocks (180 MHz PLL)
    system_clock_init();

    // 2. Start 1 ms system tick
    systick_init();

    // 3. Initialize GPIO
    gpio_init(GPIOA, 5);

    // 4. Initialize UART2 for debug prints
    uart2_init();
    printf("BOOT OK\r\n");

    // 5. Initialize CRC peripheral
    crc_init();

    // 6. Initialize ADC
    adc_init();

    // 7. Initialize DAC
    dac_init();

    // 8. Initialize SPI
    spi1_init();

    // 9. Initialize SD card (slow startup)
    SD_Status sd_status = sd_init();
    if (sd_status == SD_OK)
    {
        printf("SD OK\r\n");
    }
    else
    {
        printf("SD FAIL\r\n");
    }

    // 10. Initialize I2C
    i2c1_init();

    // 11. Initialize Bar30 pressure sensor
    bar30_init();

    // 12. Initialize OLED display
    oled_init();
    oled_draw_string(0, 0, "ROV OK");
    oled_update();

    // 13. initialize USART1 with DMA RX and IDLE interrupt
    uart1_init();

    // print confirmation message over UART2 that UART1 is up
    uart2_write_str("UART1 initialized\r\n");

    // 14. Initialize thruster PWM outputs (neutral)
    timer3_pwm_init();

    // 15. Initialize microsecond timebase
    timer2_timebase_init();

    // 16. Initialize control loop state
    control_loop_init();

    // 17. Start 50 Hz control loop timer ISR
    tim7_init();

    // 18. Arm watchdog LAST
    iwdg_init();

    while (1)
    {
        // 1. Read shared variable safely with IRQ guard
        __disable_irq();

        // read link_ok or any other shared state here
        bool local_link = link_ok;

        __enable_irq();

        // 2. Print status over UART every 500ms using g_tick
        //    (non-blocking — compare g_tick, don't use delay_ms)
        static uint32_t last_print = 0;

        if (packet_parse_cmd(&cmd))
        {
            float pose[6] =
            {
                cmd.current_x,
                cmd.current_y,
                cmd.current_z,
                cmd.current_roll,
                cmd.current_pitch,
                cmd.current_yaw
            };

            float target[6] =
            {
                cmd.target_x,
                cmd.target_y,
                cmd.target_z,
                cmd.target_roll,
                cmd.target_pitch,
                cmd.target_yaw
            };

            cmd_update(
                pose,
                target,
                cmd.armed
            );
        }

        if (g_tick - last_print >= 500)
        {
            last_print = g_tick;
            printf("tick=%lu link=%d\r\n", g_tick, local_link);
        }

        // Send telemetry at 50 Hz

        uint8_t telem_pending_local;

        __disable_irq();
        telem_pending_local = telem_pending;
        telem_pending = 0;
        __enable_irq();

        if (telem_pending_local)
        {
            TelemetryPayload tp;
            uint8_t tx_buf[PACKET_SIZE];

            memset(&tp, 0, sizeof(tp));

            tp.depth_m     = cmd.current_z;
            tp.raw_depth_m = cmd.current_z;

            control_loop_get_pwm(tp.esc_pwm, 8);

            tp.armed   = control_loop_get_armed();
            tp.link_ok = control_loop_get_link();

            tp.sat_flags = 0;

            packet_build_telemetry(&tp, tx_buf);

            uart1_write_buf(tx_buf, PACKET_SIZE);
        }

        // 3. Check log_pending flag and write log record

        uint8_t pending;

        __disable_irq();
        pending = log_pending;
        log_pending = 0;
        __enable_irq();

        if (pending)
        {
            LogRecord rec;

            rec.depth_m = cmd.current_z;

            rec.roll_deg  = cmd.current_roll;
            rec.pitch_deg = cmd.current_pitch;
            rec.yaw_deg   = cmd.current_yaw;

            control_loop_get_pwm(rec.pwm, 8);

            rec.armed = control_loop_get_armed();

            rec.link_ok = control_loop_get_link();

            rec.crc16 = crc_compute(
                (uint8_t *)&rec,
                sizeof(LogRecord) - sizeof(rec.crc16)
            );

            sd_logger_write(&rec);
        }

        // Feed watchdog
        iwdg_kick();
    }
}
