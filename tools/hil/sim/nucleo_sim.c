// =============================================================================
// nucleo_sim.c -- host-side stand-in for the Nucleo, for developing and
// checking tools/hil/hil_test.py without a board.
//
// What is real: the firmware's own packet.c (framing, CRC-16, parser),
// ring_buffer.c and control_loop.c (PID, allocation, PWM mapping, 500 ms
// command-timeout failsafe), compiled for the host from firmware/.
//
// What is modelled: main.c's main loop (mirrored below -- keep it in sync),
// SysTick as wall-clock milliseconds, TIM7 as a 20 ms step of that clock,
// and the ST-LINK virtual COM port as a pseudo-terminal. The console text is
// the same strings main.c prints on a bare board (no SD card, no Bar30).
//
// Not modelled: UART bit timing, USB latency, the hardware itself. A pass
// here says the test script and the firmware's protocol logic agree; only
// a run on the board says the board works.
//
// Usage:   ./nucleo_sim            prints the pty path, e.g. /dev/pts/3
//          kill -USR1 <pid>        simulated RESET button: reboot, reprint boot log
//          BENCH:HANG in a PID frame: prints BENCH HANG, "resets" 600 ms later
// =============================================================================
#define _XOPEN_SOURCE 700
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "packet.h"
#include "ring_buffer.h"
#include "control_loop.h"
#include "bench.h"

volatile uint32_t g_tick;              // declared extern in control_loop.h
static uint16_t   pwm_out[8];
void pwm_set_us(uint8_t ch, uint16_t us) { if (ch < 8) pwm_out[ch] = us; }

static int                   master_fd = -1;
static volatile sig_atomic_t reset_requested = 0;
static uint64_t              boot_ms;
static unsigned long         tx_dropped;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

// UART TX: never block. With no one reading the port, bytes are lost, as
// they would be on a real UART with nothing attached.
static void port_write(const void *buf, size_t len)
{
    ssize_t n = write(master_fd, buf, len);
    if (n < (ssize_t)len) tx_dropped++;
}

static void port_printf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n > 0) port_write(line, (size_t)n < sizeof line ? (size_t)n : sizeof line - 1);
}

static void on_usr1(int sig) { (void)sig; reset_requested = 1; }

// ---- BENCH_HIL stand-ins. packet.c calls bench_pid_frame() for TYPE_PID
// frames; the real one lives in firmware/Core/Src/bench.c. Measurements are
// modelled with fixed plausible values: the simulator checks the script's
// handling of the lines, not the numbers.
static int         sim_hang, sim_sig;
static const char *reset_cause = " POR PIN BOR";

void bench_pid_frame(const uint8_t *payload)
{
    if (memcmp(payload, "BENCH:HANG", 10) == 0)      sim_hang = 1;
    else if (memcmp(payload, "BENCH:SIG1", 10) == 0) sim_sig = 1;
    else if (memcmp(payload, "BENCH:SIG0", 10) == 0) sim_sig = 0;
}

static void sim_bench_status(void)
{
    static unsigned k;
    if (sim_sig) { port_printf("pwm n=25 hi=1100-1100 per=20000-20000 sig=1\r\n"); return; }
    switch (k++ & 3u) {
    case 0: port_printf("perf t7=3599964-3600036 isr=5400/4100 loop=2160000/36000 tx=970000\r\n"); break;
    case 1: port_printf("stack peak=412 reserve=1024\r\n"); break;
    case 2: port_printf("mpu who=0x68 n=%lu err=0 stale=0 i2cerr=0 t14=1708 ewho=0 etmp=0 eacc=0 e14=0 ax=-312 ay=180 az=16210 traw=-4120\r\n", (unsigned long)(g_tick / 500u)); break;
    default: port_printf("pwm n=0 hi=0-0 per=0-0 sig=0\r\n"); break;
    }
}

// ---- mirrors main.c from reset to BOOT DONE, on a board with nothing attached
static CommandPayload cmd;

static void boot(void)
{
    boot_ms = now_ms();
    g_tick = 0;
    memset(&cmd, 0, sizeof cmd);
    while (rx_avail()) rx_eat(rx_avail());
    port_printf("BOOT OK\r\n");
    port_printf("RESET:%s\r\n", reset_cause);
    reset_cause = " PIN";
    port_printf("SD FAIL\r\n");
    port_printf("BAR30 FAIL\r\n");
    port_printf("MPU 0x68 pwr=0x00\r\n");
    port_printf("LINK USART2 ST-LINK VCP\r\n");
    control_loop_init();
    port_printf("TIMERS OK\r\n");
    port_printf("BOOT DONE\r\n");
}

int main(void)
{
    master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd < 0 || grantpt(master_fd) || unlockpt(master_fd)) { perror("pty"); return 1; }
    const char *slave = ptsname(master_fd);

    // Put the slave side in raw mode and hold it open: no echo, no CR/LF
    // translation, and reads on the master don't fail while no client is
    // attached.
    int slave_fd = open(slave, O_RDWR | O_NOCTTY);
    struct termios t;
    tcgetattr(slave_fd, &t);
    cfmakeraw(&t);
    tcsetattr(slave_fd, TCSANOW, &t);
    fcntl(master_fd, F_SETFL, fcntl(master_fd, F_GETFL) | O_NONBLOCK);

    signal(SIGUSR1, on_usr1);
    printf("PTY %s\n", slave);
    printf("PID %d  (kill -USR1 %d = press RESET)\n", (int)getpid(), (int)getpid());
    fflush(stdout);

    boot();
    uint32_t last_tim7 = 0;
    uint32_t last_print = 0;
    bool     telem_pending_local = false;

    for (;;)
    {
        if (reset_requested) { reset_requested = 0; last_tim7 = 0; last_print = 0; boot(); }

        if (sim_hang)                                     // main loop stops; IWDG fires
        {
            port_printf("BENCH HANG\r\n");
            struct timespec wd = { 0, 600000000 };        // ~ the typical-LSI timeout
            nanosleep(&wd, NULL);
            sim_hang = 0; sim_sig = 0;
            reset_cause = " IWDG PIN";
            last_tim7 = 0; last_print = 0;
            boot();
            continue;
        }

        // ---- "interrupts": SysTick, UART RX, TIM7 --------------------------
        g_tick = (uint32_t)(now_ms() - boot_ms);

        uint8_t in[256];
        ssize_t n = read(master_fd, in, sizeof in);
        if (n > 0) rx_write(in, (uint16_t)n);               // USART2_IRQHandler

        while ((uint32_t)(g_tick - last_tim7) >= 20u)       // TIM7_IRQHandler
        {
            last_tim7 += 20u;
            telem_pending_local = true;                      // telem_pending = 1
            control_loop_tick();
        }

        // ---- main loop body, as in main.c ---------------------------------
        bool local_link = link_ok;

        if (packet_parse_cmd(&cmd))
        {
            float pose[6]   = { cmd.current_x, cmd.current_y, cmd.current_z,
                                cmd.current_roll, cmd.current_pitch, cmd.current_yaw };
            float target[6] = { cmd.target_x, cmd.target_y, cmd.target_z,
                                cmd.target_roll, cmd.target_pitch, cmd.target_yaw };
            cmd_update(pose, target, cmd.armed);
        }

        if (g_tick - last_print >= 500)
        {
            last_print = g_tick;
            port_printf("tick=%lu link=%d rxdrop=%lu\r\n",
                        (unsigned long)g_tick, local_link, (unsigned long)rx_dropped_count());
            sim_bench_status();
        }

        if (telem_pending_local)
        {
            telem_pending_local = false;
            TelemetryPayload tp;
            uint8_t tx_buf[PACKET_SIZE];
            memset(&tp, 0, sizeof tp);
            tp.depth_m     = cmd.current_z;
            tp.raw_depth_m = cmd.current_z;
            uint16_t pwm_aligned[8];                      // as main.c: esc_pwm is packed
            control_loop_get_pwm(pwm_aligned, 8);
            memcpy(tp.esc_pwm, pwm_aligned, sizeof(pwm_aligned));
            tp.armed   = control_loop_get_armed();
            tp.link_ok = control_loop_get_link();
            tp.sat_flags = 0;
            packet_build_telemetry(&tp, tx_buf);
            port_write(tx_buf, PACKET_SIZE);
        }

        struct timespec nap = { 0, 200000 };                 // 0.2 ms
        nanosleep(&nap, NULL);
    }
}
