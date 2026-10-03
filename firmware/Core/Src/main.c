#include <stdio.h>
#include <string.h>
#include "system_init.h"
#include "gpio.h"
#include "uart.h"
#include "uart_packet.h"
#include "link.h"
#include "spi.h"
#include "oled.h"
#include "sd_card.h"
#include "timer_basic.h"
#include "control_loop.h"
#include "adc.h"
#include "struct.h"
#include "packet.h"
#include "ring_buffer.h"
#include "sd_logger.h"
#include "crc_hw.h"
#include "i2c.h"
#include "bar30.h"
#include "timer_pwm.h"
#include "timer_timebase.h"
#include "iwdg.h"
#include "bench.h"

static CommandPayload cmd;

// Why the last reset happened (RCC_CSR, RM0390 6.3.21), then clear the flags
// so the next boot reports only its own cause. Power-on sets POR, PIN and BOR
// together; a watchdog reset sets IWDG and PIN.
static void reset_cause_report(void)
{
    uint32_t csr = RCC->CSR;

    printf("RESET:%s%s%s%s%s%s%s\r\n",
           (csr & RCC_CSR_LPWRRSTF) ? " LPWR" : "",
           (csr & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
           (csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
           (csr & RCC_CSR_SFTRSTF)  ? " SOFT" : "",
           (csr & RCC_CSR_PORRSTF)  ? " POR"  : "",
           (csr & RCC_CSR_PINRSTF)  ? " PIN"  : "",
           (csr & RCC_CSR_BORRSTF)  ? " BOR"  : "");

    RCC->CSR |= RCC_CSR_RMVF;
}


int main(void)
{
    // 0. BENCH_HIL: paint the unused stack so its high-water mark can be read
    bench_stack_paint();

    // 1. Configure system clocks (180 MHz PLL)
    system_clock_init();

    // 2. Start 1 ms system tick
    systick_init();

    // 2a. Thruster PWM outputs to neutral before anything that can block:
    //     sd_init() and bar30_init() poll with no timeout, and until this
    //     runs the ESC signal pins are floating inputs.
    timer3_pwm_init();

    // 3. TIM7 tick probe output (PB10 / Arduino D6)
    gpio_init(TICK_PROBE_PORT, TICK_PROBE_PIN);

    // 4. Initialize UART2 for debug prints
    uart2_init();
    printf("BOOT OK\r\n");
    reset_cause_report();

    // 5. Initialize CRC peripheral
    crc_init();

    // 6. Initialize ADC
    adc_init();

    // 8. Initialize SPI
    spi1_init();

    // 9. Initialize SD card (slow startup)
    SD_Status sd_status = sd_init();
    if (sd_status == SD_OK)
    {
        sd_logger_init();
        printf("SD OK\r\n");
    }
    else
    {
        printf("SD FAIL\r\n");
    }

    // 10. Initialize I2C
    i2c1_init();

    // 11. Initialize Bar30 pressure sensor (fails cleanly if absent)
    if (bar30_init() == 0)
    {
        printf("BAR30 OK\r\n");
    }
    else
    {
        printf("BAR30 FAIL\r\n");
    }

    // 11a. BENCH_HIL: an MPU-6050 on the same bus, if one is fitted
    bench_mpu_init();

    // 12. Initialize OLED display
    oled_init();
    oled_draw_string(0, 0, "ROV OK");
    oled_update();

    // 13. Start the Pi link: USART2 over the ST-LINK USB cable, or USART1
    //     with DMA RX on PA9/PA10 (LINK_PORT_STLINK in link.h)
    link_init();
    printf("LINK %s\r\n", link_port_name());

    // 15. Initialize microsecond timebase
    timer2_timebase_init();

    // 15a. BENCH_HIL: DWT cycle counter, PA15 capture on TIM2_CH1
    bench_init();

    // 16. Initialize control loop state
    control_loop_init();

    // 17. Start 50 Hz control loop timer ISR
    tim7_init();

    // 18. Arm watchdog LAST
    iwdg_init();
    printf("BOOT DONE\r\n");

    while (1)
    {
        bench_loop_top();

        // BENCH_HIL "BENCH:HANG": stop here, as a hung main loop would. The
        // TIM7 ISR keeps running the control loop, so this also shows that
        // a live timer interrupt does not keep the watchdog fed.
        if (bench_hang_requested())
        {
            printf("BENCH HANG\r\n");
            for (;;)
            {
            }
        }

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
            printf("tick=%lu link=%d rxdrop=%lu\r\n", g_tick, local_link, rx_dropped_count());
            bench_status();
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

            // esc_pwm sits in a packed struct, so its address is not
            // guaranteed 2-byte aligned; fill an aligned local and copy
            // the bytes, rather than hand a uint16_t* to an odd address.
            uint16_t pwm_aligned[8];
            control_loop_get_pwm(pwm_aligned, 8);
            memcpy(tp.esc_pwm, pwm_aligned, sizeof(pwm_aligned));

            tp.armed   = control_loop_get_armed();
            tp.link_ok = control_loop_get_link();

            tp.sat_flags = 0;

            packet_build_telemetry(&tp, tx_buf);

            bench_tx_begin();
            link_send(tx_buf, PACKET_SIZE);
            bench_tx_end();
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

            rec.timestamp_ms = g_tick;

            rec.depth_m = cmd.current_z;

            rec.roll_deg  = cmd.current_roll;
            rec.pitch_deg = cmd.current_pitch;
            rec.yaw_deg   = cmd.current_yaw;

            uint16_t rec_pwm[8];                  // same reason as above
            control_loop_get_pwm(rec_pwm, 8);
            memcpy(rec.pwm, rec_pwm, sizeof(rec_pwm));

            rec.armed = control_loop_get_armed();

            rec.link_ok = control_loop_get_link();

            rec.crc16 = crc_compute(
                (uint8_t *)&rec,
                sizeof(LogRecord) - sizeof(rec.crc16)
            );

            sd_logger_write(&rec);
        }

        // Write a partly filled log block if it has waited too long
        sd_logger_poll(g_tick);

        // BENCH_HIL: one MPU-6050 read cycle every 500 ms, offset from the status line
        bench_mpu_poll(g_tick);

        // Feed watchdog
        iwdg_kick();
    }
}
