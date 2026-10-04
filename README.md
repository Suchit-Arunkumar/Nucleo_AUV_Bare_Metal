# Nucleo_AUV_Bare_Metal

Register-level firmware for an 8-thruster AUV control node on the STM32F446RE
(NUCLEO-F446RE), written without HAL or LL: a 180 MHz clock tree with
over-drive, eight ESC PWM channels across three timers, a CRC-16 binary link to a
Raspberry Pi, SPI SD logging, an I2C pressure sensor, an SPI OLED, a 50 Hz
6-DOF control loop and an independent watchdog. Every peripheral access in
`Drivers/`, `Devices/` and `Protocol/` is a direct register write traceable to
RM0390, PM0214 or DS10693.

**Verified on a NUCLEO-F446RE**: 14 of 14 hardware-in-the-loop tests pass, with
0.34 µs worst control-tick jitter and 0 CRC errors over a 60 s, 50 Hz soak.
Bring-up found three defects that compiling never showed. Measured results are in
[Key figures](#key-figures) and
[Hardware-in-the-loop verification](#hardware-in-the-loop-verification).

**Scope.** The STM32 control path for Team Tiburon's AUV (SAUVC 2026), developed
and verified on a NUCLEO-F446RE bench rig; not yet integrated on the vehicle.
At SAUVC 2026 these jobs were split between a Raspberry Pi (VN-200 IMU,
Wayfinder DVL) and the team's RP2350 (Pico 2) firmware (all 8 ESCs and
thrusters, Bar30 depth, SD logging, TFT display), which speaks the same wire protocol
([pico-protocol](https://github.com/Suchit-Arunkumar/pico-protocol)). The control
law here (PID, 6×8 thrust allocation, command-timeout failsafe) is ported from
that firmware's tuning sketch. The STM32 Bar30, SD and OLED drivers are
separate register-level implementations, exercised on the bench only with
those parts absent.

---

## Contents

1. [Status](#status)
2. [System overview](#system-overview)
3. [Pin map](#pin-map)
4. [Boot sequence](#boot-sequence)
5. [Clock tree](#clock-tree)
6. [Timing and interrupts](#timing-and-interrupts)
7. [Thruster PWM](#thruster-pwm)
8. [Serial link to the Pi](#serial-link-to-the-pi)
9. [Control loop](#control-loop)
10. [SD card logging](#sd-card-logging)
11. [Independent watchdog](#independent-watchdog)
12. [Hardware-in-the-loop verification](#hardware-in-the-loop-verification)
13. [Key figures](#key-figures)
14. [Engineering log](#engineering-log)
15. [Off-target verification](#off-target-verification)
16. [Build](#build)
17. [Repository layout](#repository-layout)
18. [References](#references)

Open issues are listed in [TODO.md](TODO.md).

---

## Status

| | |
|---|---|
| Builds at `-O2` | Yes: 0 errors, 2 warnings (arm-none-eabi-gcc 13.2.1), both in code paths not yet used |
| Verified on hardware | **Yes, on a bare NUCLEO-F446RE**: 14 / 14 HIL tests, 2026-10-04 ([reports](tools/hil/reports/)) |
| Exercised on hardware | Clock tree, SysTick, TIM2/3/4/7/12, USART2 + RX interrupt, I2C1 (against an MPU-6050), IWDG, RCC reset flags, the protocol, control loop and failsafe |
| Not exercised on hardware | USART1 + DMA on PA9/PA10, SD card writes, the OLED, the Bar30, ESCs and thrusters |
| Vehicle integration | Not yet: bench-verified only. On the SAUVC 2026 vehicle the Pi ran the VN-200 and DVL, and the RP2350 (Pico 2) firmware drove all 8 ESCs and ran the Bar30, SD logging and display, on the same wire protocol |

---

## System overview

```
                    ┌──────────────────────── STM32F446RE @ 180 MHz ────────────────────────┐
  Raspberry Pi ◄──► │ USART1 115200  DMA2 S2 circular RX + IDLE IRQ → ring buffer → parser  │
  (fused nav state, │   62-byte frames, CRC-16/IBM-3740 (same framing as pico-protocol)     │
   PID gains)       │   bench build: USART2 over the ST-LINK USB cable, RXNE IRQ instead    │
                    │                                                                        │
                    │ TIM7 50 Hz ISR ── PID ── 6-DOF allocation (B⁺) ── slew limit ──┐       │
                    │                                                                ▼       │
  8 × ESC ◄──────── │ TIM3 CH1-4, TIM4 CH1-2, TIM12 CH1-2   50 Hz, 1 µs resolution           │
                    │                                                                        │
  SD card ◄──────── │ SPI1 ── 12 log records per 512-byte block                              │
  SSD1306 OLED ◄─── │ SPI1 (shared bus, separate chip-selects)                               │
  Bar30 (MS5837) ◄─ │ I2C1 100 kHz, 5 ms timeouts                                            │
  ST-LINK VCP ◄──── │ USART2 printf console (and the link, in the bench build)               │
                    │ IWDG kicked from the main loop only                                    │
                    └────────────────────────────────────────────────────────────────────────┘
```

**Data flow per 20 ms tick.** TIM7 fires, sets `log_pending` and `telem_pending`,
toggles the tick probe pin, and runs `control_loop_tick()`: command-timeout
check, PID on six axes, thrust allocation through the pseudo-inverse of the
thruster geometry matrix, then slew-limited PWM writes to eight channels. The
main loop parses incoming frames from the Pi, sends one telemetry frame, builds
and writes a log record, services the logger's flush timer, and kicks the
watchdog.

---

## Pin map

The pin state after the complete boot sequence, as each init function leaves
it. Header positions are from UM1724 (Nucleo-64 user manual); check them against
your board revision before wiring.

| Pin | Mode | Function | Header | Configured by |
|---|---|---|---|---|
| PA0 | Analog | ADC1_IN0 (driver present, never read) | A0 | `adc_init()` |
| PA2 | AF7 | USART2_TX: console, and the Pi link in the bench build | ST-LINK | `uart2_init()` |
| PA3 | AF7 | USART2_RX: the Pi link in the bench build | ST-LINK | `uart2_init()`, `link_init()` |
| PA4 | Output | OLED chip-select | A2 | `spi1_init()` |
| PA5 | AF5 | SPI1_SCK (also drives LD2 through SB21) | D13 | `spi1_init()` |
| PA6 | AF5, pull-up | SPI1_MISO; pulled up so "no card" reads 0xFF | D12 | `spi1_init()` |
| PA7 | AF5 | SPI1_MOSI | D11 | `spi1_init()` |
| PA8 | Output | OLED data/command | D7 | `oled_init()` |
| PA9 | AF7 | USART1_TX to the Pi (vehicle build only) | D8 | `link_init()` → `uart1_init()` |
| PA10 | AF7 | USART1_RX from the Pi (vehicle build only) | D2 | `link_init()` → `uart1_init()` |
| PA13 / PA14 | AF0 | SWDIO / SWCLK (reset default, untouched) | — | — |
| PA15 | AF1, pull-down | TIM2_CH1 input capture (bench build only) | CN7-17 | `bench_init()` |
| PB0 | Output | SD card chip-select | A3 | `spi1_init()` |
| PB5 | AF2 | TIM3_CH2, thruster T2 | D4 | `pwm_init()` |
| PB6 | AF2 | TIM4_CH1, thruster T5 | D10 | `pwm_init()` |
| PB7 | AF2 | TIM4_CH2, thruster T6 | CN7-21 | `pwm_init()` |
| PB8 | AF4, open-drain, pull-up | I2C1_SCL: Bar30, and the MPU-6050 on the bench | D15 | `i2c1_init()` |
| PB9 | AF4, open-drain, pull-up | I2C1_SDA | D14 | `i2c1_init()` |
| PB10 | Output | TIM7 tick probe, 25 Hz square wave | D6 | `main()` step 3 |
| PB14 | AF9 | TIM12_CH1, thruster T7 | CN10-28 | `pwm_init()` |
| PB15 | AF9 | TIM12_CH2, thruster T8 | CN10-26 | `pwm_init()` |
| PC6 | AF2 | TIM3_CH1, thruster T1 | CN10-4 | `pwm_init()` |
| PC7 | Output | OLED reset | D9 | `oled_init()` |
| PC8 | AF2 | TIM3_CH3, thruster T3 | CN10-2 | `pwm_init()` |
| PC9 | AF2 | TIM3_CH4, thruster T4 | CN10-1 | `pwm_init()` |
| PH0 | — | HSE bypass input, 8 MHz from the ST-LINK MCO | — | `system_clock_init()` |

**Rules the drivers follow.** Every alternate-function field is cleared before
it is set, never OR'd in on its own, so the order the init functions run in
cannot corrupt a shared port. No init touches a pin it doesn't own. PB3 and PB4
(SWO and NJTRST) are deliberately left alone. PA15 resets as JTDI with a
pull-up; the bench build clears both before taking it, which is safe because
the debugger uses SWD only.

---

## Boot sequence

From `main()`. The order is deliberate, and the reason is given where it matters.

| Step | Call | Why it is here |
|---|---|---|
| 0 | `bench_stack_paint()` | Bench build: fills unused stack with a pattern so the high-water mark can be read later |
| 1 | `system_clock_init()` | 180 MHz with over-drive; every later timing constant assumes it |
| 2 | `systick_init()` | 1 ms `g_tick`; `delay_ms()` depends on it |
| 2a | `timer3_pwm_init()` | ESC lines to 1500 µs neutral **before** anything that can block. Until this runs they are floating inputs |
| 3 | `gpio_init(PB10)` | Tick probe output |
| 4 | `uart2_init()` | Debug console, so later steps can report; then prints `BOOT OK` and the reset cause from `RCC_CSR` |
| 5 | `crc_init()` | Hardware CRC unit, used for log-record CRCs |
| 6 | `adc_init()` | PA0 analog; no reads yet |
| 8 | `spi1_init()` | Shared OLED/SD bus and both chip-selects, deselected; MISO pulled up |
| 9 | `sd_init()`, `sd_logger_init()` | Prints `SD OK` / `SD FAIL`. The logger is only initialised if the card came up |
| 10 | `i2c1_init()` | 100 kHz, CCR 225, TRISE 46, internal pull-ups |
| 11 | `bar30_init()` | Prints `BAR30 OK` / `BAR30 FAIL`. Each I2C wait gives up after 5 ms or on NACK |
| 11a | `bench_mpu_init()` | Bench build: wakes an MPU-6050 if fitted, reads `WHO_AM_I` and `PWR_MGMT_1` back |
| 12 | `oled_init()` | Reset pulse timed with `delay_ms()` |
| 13 | `link_init()` | Pi link: USART2 over the ST-LINK USB (bench build) or USART1 with DMA RX (vehicle build); prints `LINK <port>` |
| 15 | `timer2_timebase_init()` | 32-bit 1 MHz free-running counter for `micros()`; `EGR.UG` loads the prescaler immediately |
| 15a | `bench_init()` | Bench build: DWT cycle counter, TIM2_CH1 capture on PA15 |
| 16 | `control_loop_init()` | State reset, all channels neutral |
| 17 | `tim7_init()` | 50 Hz control ISR starts; prints `TIMERS OK` |
| 18 | `iwdg_init()` | Armed last, so slow init can't trip it; then prints `BOOT DONE` |

On a bare board, measured: 62–95 ms from `BOOT OK` to `BOOT DONE`. The console
then shows a status line every 500 ms, `tick=<ms> link=<0|1> rxdrop=<bytes>`,
followed in the bench build by one of four rotating measurement lines
(`perf`, `stack`, `mpu`, `pwm`).

```
BOOT OK
RESET: PIN
SD FAIL
BAR30 FAIL
MPU 0x68 pwr=0x00
LINK USART2 ST-LINK VCP
TIMERS OK
BOOT DONE
tick=500 link=0 rxdrop=0
```

---

## Clock tree

HSE 8 MHz (bypass, from the ST-LINK MCO) → PLL → 180 MHz SYSCLK.

| Stage | Value | Result |
|---|---|---|
| PLLM | 8 | VCO input 1 MHz |
| PLLN | 360 | VCO output 360 MHz |
| PLLP | `0b00` (encodes /2) | SYSCLK 180 MHz |
| HPRE | /1 | AHB 180 MHz |
| PPRE1 | /4 | APB1 45 MHz (maximum); APB1 timers 90 MHz |
| PPRE2 | /2 | APB2 90 MHz (maximum); APB2 timers 180 MHz |
| `FLASH->ACR` | 5 wait states, ART prefetch + I/D cache | RM0390 table: 150 < HCLK ≤ 180 MHz at 2.7–3.6 V |

180 MHz is above the 168 MHz ceiling of the normal regulator mode, so
over-drive is enabled after PLL lock and before the SYSCLK switch:

```
APB1ENR.PWREN → PWR_CR.ODEN → PWR_CSR.ODRDY → PWR_CR.ODSWEN → PWR_CSR.ODSWRDY
```

Timer clocks double when their APB prescaler is not 1 (`RCC_DCKCFGR.TIMPRE` is
left at 0). That is why TIM2/3/4/7/12 run from 90 MHz while APB1 itself is
45 MHz.

`PWR_CR.VOS` is relied on at its reset value (Scale 1). A warm reset or
bootloader path that leaves VOS at Scale 2 would turn the `ODRDY` poll into a
silent infinite hang; ST's reference sequence writes VOS explicitly.

PLLQ and PLLR stay at reset (4 and 2). With a 360 MHz VCO, PLLQ gives 90 MHz,
not the 48 MHz USB OTG needs, so USB can't be clocked from the main PLL.

**Measured.** Over 9.5 s the board's SysTick count matched the laptop clock to
0.05–0.22 % across runs (the limit of USB timestamp jitter; a wrong PLL setting
shows up as tens of percent). Console text at 115,200 baud is clean, and a
62-byte frame takes 5.292 ms to send against 5.295 ms theoretical, so USART2's
BRR, and with it the 45 MHz APB1 clock, is right to within 0.06 %.

References: RM0390 §6.3.2 (PLLCFGR), §6.3.3 (CFGR), PWR chapter, §3.4.1 (ACR).

---

## Timing and interrupts

| Source | Rate | Priority | Does | Measured |
|---|---|---|---|---|
| SysTick | 1 kHz | 0 (reset default) | `g_tick++` | status lines exactly 500 ms of board time apart |
| TIM7 update | 50 Hz | 0 | Control loop, `log_pending`, `telem_pending`, tick probe | period 19,999.66–20,000.32 µs; body 8.3 µs avg, 45.6 µs max |
| USART1 IDLE | per burst | 1 | Vehicle build: copies new DMA bytes into the ring buffer | not exercised |
| USART2 RXNE | per byte | 1 | Bench build: pushes each received byte into the ring buffer | 0 dropped bytes in a 60 s soak |
| TIM2 CC1 | per edge | 2 | Bench build: PWM input capture on PA15 | 200 pulses, 0 µs error |
| TIM3 / TIM4 / TIM12 | 50 Hz PWM | — | Hardware only, no interrupts | 20,000 µs period on all 8 pins |
| TIM2 | 1 MHz free-running | — | `micros()`, capture timebase | — |

TIM7 uses PSC 1799 and ARR 999: 90 MHz / 1800 / 1000 = 50 Hz, so the control
loop's `dt = 0.02 s` is exact by construction. PB10 toggles once per tick, so a
scope on it shows the tick rate directly.

**Jitter.** The worst period measured over a full run, ISR entry to ISR entry
by the DWT cycle counter, is 0.34 µs (61 core cycles) from nominal. That is
consistent with TIM7 occasionally waiting for SysTick, which shares priority 0.
The worst ISR body, 45.6 µs, is 0.23 % of the tick.

**Main loop.** 4 µs per pass on average; the worst pass measured is 17.77 ms.
Blocking console output dominates that figure: one telemetry frame (5.29 ms)
plus the status line and one measurement line, all sent from the main loop at
115,200 baud in the same pass. It is still 19× inside the watchdog's 341 ms
shortest timeout.

---

## Thruster PWM

Eight channels, one per thruster, from `Drivers/timer/timer_pwm.c`.

| Channel | Thruster | Timer | Pin | AF |
|---|---|---|---|---|
| 0 | T1 vertical | TIM3 CH1 | PC6 | 2 |
| 1 | T2 vertical | TIM3 CH2 | PB5 | 2 |
| 2 | T3 vertical | TIM3 CH3 | PC8 | 2 |
| 3 | T4 vertical | TIM3 CH4 | PC9 | 2 |
| 4 | T5 horizontal | TIM4 CH1 | PB6 | 2 |
| 5 | T6 horizontal | TIM4 CH2 | PB7 | 2 |
| 6 | T7 horizontal | TIM12 CH1 | PB14 | 9 |
| 7 | T8 horizontal | TIM12 CH2 | PB15 | 9 |

**Why three timers.** On the 64-pin package, every other output pin these
timers could use is already taken or doesn't exist:

- TIM3_CH1's alternatives are PA6 (SPI1_MISO) and PB4 (NJTRST).
- TIM3_CH2's are PA7 (SPI1_MOSI) and PC7 (OLED reset).
- TIM3_CH3's is PB0 (SD chip-select).
- TIM4_CH3/CH4 are only on PB8/PB9 (I2C1) or on port D, which isn't bonded out.
- TIM1's channels are on PA8 (OLED DC), PA9/PA10 (USART1) and port E, which
  isn't bonded out.

So no pair of timers can reach eight free pins. TIM3, TIM4 and TIM12 all sit on
the 90 MHz APB1 timer clock, so one configuration serves all three: PSC 89
(1 µs ticks), ARR 19999 (20 ms frame), PWM mode 1 with CCR preload.

**Start-up without runt pulses.**
1. Each timer starts with every CCR at 0, so its outputs are held low.
2. `EGR.UG` loads the prescaler before the first period.
3. The pins switch to alternate-function mode only after that, with pull-downs.
4. 1500 µs is written through the preload register, so every pin's first real
   pulse is a full neutral pulse. A pin can't be caught mid-period and emit a
   short one.

**`pwm_set_us(channel, us)`** writes one CCR register through a lookup table.
It ignores channels above 7 and clamps to 1000–2000 µs. A value at or above the
20 ms period would otherwise hold the ESC line high. The control loop already
limits itself to 1100–1900 µs, so the clamp is a backstop. It's safe to call
from the TIM7 ISR because each call is a single register store.

**Measured.** In the bench build, channel *k* is driven at 1,100 + 100*k* µs and
a jumper from PA15 (TIM2_CH1 input capture, both edges, 1 µs ticks) is moved
across the eight pins. Every pin showed exactly its own channel's width over
25 pulses (200 pulses in all, 0 µs deviation) and a 20,000 µs period, then
returned to 1,500 µs when the signature was switched off
([report](tools/hil/reports/2026-10-04_0ed4c05.md)). The capture shares the
clock tree with the timers it measures, so it checks PSC, ARR, CCR and pin
routing exactly; the absolute clock rate is checked separately against the
laptop.

---

## Serial link to the Pi

115200 baud, framed identically to
[pico-protocol](https://github.com/Suchit-Arunkumar/pico-protocol), the
RP2350 firmware used on the competition vehicle, so the Pi-side parser accepts
frames from either board. Telemetry goes out once per 50 Hz tick. Commands are
accepted at any rate.

**Which UART.** `LINK_PORT_STLINK` in `Drivers/uart/link.h` selects it at build
time:

| Setting | UART | Receive | Use |
|---|---|---|---|
| `1` (default) | USART2, the ST-LINK virtual COM port on the USB cable | RXNE interrupt, one byte at a time | Bench testing with a laptop, no adapter ([`tools/hil/`](tools/hil/)) |
| `0` | USART1 on PA9/PA10 | DMA2 Stream 2 circular + IDLE interrupt | The vehicle, wired to the Pi |

In the bench build the `printf` console shares USART2 with the protocol. Lines and
frames are both written whole from the main loop, never interleaved, and console
text never contains `0xAA`, so the host separates them by parsing frames and
treating everything else as text.

```
[ 0xAA ][ 0x55 ][ LEN=56 ][ TYPE ][ ...... PAYLOAD, 56 bytes ...... ][ CRC_HI ][ CRC_LO ]
   0       1        2        3       4                          59       60        61
```

| Field | Value |
|---|---|
| Frame length | 62 bytes, fixed |
| Payload | packed little-endian structs from `struct.h`, 56 bytes for every type |
| TYPE | `0x01` TELEMETRY (to Pi), `0x02` CMD (from Pi), `0x03` PID gains (from Pi) |
| CRC | CRC-16/IBM-3740: poly `0x1021`, init `0xFFFF`, no reflection, no final XOR; check value of `"123456789"` is `0x29B1` |
| CRC covers | LEN, TYPE, payload (58 bytes); not the sync bytes |
| CRC byte order | big-endian (`CRC_HI` first) |

The CRC is computed in software. The F446's hardware CRC unit only implements
CRC-32 (poly `0x04C11DB7`), and 58 bytes of bitwise CRC-16 twice per 20 ms is a
few microseconds at 180 MHz. The hardware unit is still used for SD log records.

**Receive path.** Both UARTs feed the same 256-byte software ring buffer.
- **USART1:** DMA2 Stream 2 (channel 4) writes bytes into a 256-byte circular
  buffer, and the IDLE-line interrupt copies whatever arrived since the last IDLE
  into the ring buffer. The IDLE flag is cleared by reading SR, then DR.
- **USART2:** the RXNE interrupt pushes each byte straight in.

The ring buffer has one writer (the interrupt) and one reader (the main loop). Its
count is updated with interrupts masked on the reader side. A full buffer drops
new bytes and counts them (`rxdrop` on the console) rather than overwriting
unread data. The parser in `packet_parse_cmd()` then checks each candidate frame
in turn:

1. sync bytes
2. `LEN == 56` and a known TYPE
3. CRC

On any failure it drops one byte and tries again, so a corrupted or misaligned
stream costs at most one frame of resync. A coincidental `0xAA 0x55` inside a
payload is rejected by the LEN/TYPE/CRC checks. Valid frames that aren't CMD are
consumed whole.

**Layout pinning.** `packet.h` asserts the size of each payload struct and the
offset of every field at compile time, with the same values as
`pico_protocol.h`. A reordered or resized field fails the build instead of
producing CRC-valid frames that decode to wrong values.

**Measured on hardware** (bench build, USART2): 3,002 frames over 60 s with 0
CRC errors and 0 dropped bytes; 3,002 / 3,002 random float32 values returned
bit-exact; 146 / 146 frames recovered through junk, decoy headers and split
writes; no value from a bad-CRC frame ever used. The vehicle path, USART1 with
DMA on PA9/PA10, has not been run on hardware.

**Limitations** (tracked in [TODO.md](TODO.md)):

- PID frames are recognised and discarded; gains are compile-time constants.
- Telemetry `raw_depth_m` currently echoes the commanded depth, not the Bar30
  reading, because `bar30_read()` is not yet called.
- Telemetry is sent with a blocking write from the main loop: 5.292 ms of
  every 20 ms tick at 115,200 baud, measured.
- There is no protocol version byte.

---

## Control loop

TIM7's 50 Hz ISR runs PID on six axes (surge, sway, heave, roll, pitch, yaw),
then 6-DOF thrust allocation via the pseudo-inverse `B⁺` of the 6×8 thruster
geometry matrix. It renormalises the vertical, horizontal-translation and yaw
groups separately so no group saturates the others, maps thrust to PWM with a
deadzone and a 50 µs-per-tick slew limit, and runs a 500 ms command-timeout
failsafe. Yaw error is wrapped to ±180°.

**Gains not yet tuned.** The structure above is complete and timed on hardware;
`kp`, `ki`, `kd`, `kff` and `FF_OFFSET` are zero, so every axis outputs zero and
all thrusters hold neutral. Tuning needs the vehicle in water.

**Measured.** The whole tick (timeout check, PID, allocation, PWM writes) runs
in 8.3 µs on average and 45.6 µs at worst. The 500 ms command-timeout failsafe
disarmed the vehicle in 3 / 3 runs, 535–542 ms after the last command as seen
from the laptop; the board detects it on the first 20 ms tick past 500 ms, and
the rest is USB latency.

---

## SD card logging

One `LogRecord` per 20 ms tick, 40 bytes: timestamp, depth, roll/pitch/yaw, eight
PWM values, armed, link state and a CRC.

**On-card format**, starting at block 100 (blocks 0–99 are left for the boot
sector and partition table):

```
block N:  [ record 0 ][ record 1 ] ... [ record 11 ][ 32 bytes of zero ]
             40 B        40 B              40 B
```

Twelve records are buffered in RAM and written as one 512-byte block, about 4
card writes a second instead of 50. Unused slots are all zero and fail the
record CRC check, so a reader validates every slot rather than trusting a count.

**Flushing.** A partly filled block is written once its oldest record is
500 ms old. At 50 Hz a block fills in 240 ms, so this only happens if records
stop arriving. The same block is rewritten in place when it fills, so nothing
that reached the card is ever dropped. If a full block fails to write, it is
kept and retried on the next record. A failing timed flush backs off 500 ms
rather than retrying on every loop pass.

**Trade-off.** Up to 11 records (220 ms) exist only in RAM at any moment and are
lost on power loss. The buffer is static: the previous implementation put
512 bytes on a 1 KB stack for every write.

**Card support.** SDHC/SDXC only. `sd_write_block()` sends block addresses; the
byte addressing SDSC cards need is not implemented.

---

## Independent watchdog

`PR = 3` (/32), `RLR = 500`, clocked from the LSI.

The LSI on the F446 is an uncalibrated RC oscillator, so the timeout is a
**range**, not a value:

```
T = (RLR + 1) × prescaler / f_LSI  =  16,032 / f_LSI
```

| f_LSI | Timeout | Meaning |
|---|---|---|
| 47 kHz (max) | **341 ms** | Shortest timeout: the kick budget |
| 32 kHz (typ) | 501 ms | Nominal |
| 28.5 kHz (**this chip, measured**) | **562 ms** | 6 / 6 deliberate hangs reset in 561–563 ms |
| 17 kHz (min) | **943 ms** | Longest: worst-case detection latency |

The kick interval is designed against 341 ms, not 501 ms. The worst main-loop
pass measured is 17.77 ms, a 19× margin. The safety bound, how long a hung MCU
can sit before reset, is 943 ms plus reset and re-init.

`iwdg_kick()` is called from the **main loop**, deliberately not from the TIM7
ISR. A timer ISR fires whether or not the main loop is making progress, so
kicking from it would only prove the timer is alive. The HIL test proves this:
it stops the main loop with TIM7 still running the control loop, and the board
resets every time, with `RCC_CSR` reporting `IWDG` on the next boot.

**Init sequence**, in ST's HAL order: `KR = 0xCCCC` (start; forces the LSI on)
→ `KR = 0x5555` (unlock) → `PR` → `RLR` → wait until `SR.PVU` and `SR.RVU` clear,
bounded at 50 ms → `KR = 0xAAAA` (reload). The previous order wrote PR and spun
on PVU before starting the watchdog, and on hardware PVU never cleared: boot
hung forever with nothing to reset it. That was the first defect the HIL run
found.

---

## Hardware-in-the-loop verification

### The rig

```
  Laptop (plays the Pi)                    NUCLEO-F446RE
  ┌────────────────────┐   ST-LINK USB    ┌──────────────────────────────────────┐
  │ hil_test.py        │◄────────────────►│ USART2: console + 62-byte frames     │
  │  sends CMD frames  │   one cable:     │ TIM2_CH1 capture on PA15 ◄── jumper ─┼─► any ESC pin
  │  checks telemetry  │   flash, power,  │ I2C1 PB8/PB9 ◄──────────────────────►│ MPU-6050
  │  writes a report   │   serial         │ DWT cycle counter: timing            │
  └────────────────────┘                  └──────────────────────────────────────┘
```

No Raspberry Pi, scope, logic analyser or USB-serial adapter. The laptop sends
the same command frames the Pi would. The board measures itself: the DWT
cycle counter times the ISR and the main loop at 5.6 ns resolution, and
TIM2 input capture on PA15 measures any ESC output at 1 µs resolution. The
`BENCH_HIL` build flag carries this instrumentation; `-DBENCH_HIL=0` removes it
for the vehicle.

### Results

Firmware `5a2af27` unless noted, 2026-10-04. Full reports:
[`5a2af27`](tools/hil/reports/2026-10-04_5a2af27.md) (final firmware),
[`0ed4c05`](tools/hil/reports/2026-10-04_0ed4c05.md) (PWM pin sweep; same PWM
code). Host-measured times include Windows USB-serial latency.

| Test | Result | Measured |
|---|---|---|
| Boot with nothing attached | PASS | `BOOT DONE` 95 ms after `BOOT OK`; SD and Bar30 absence reported, not hung; reset cause `PIN` |
| Clock rate | PASS | board 9,500 ms vs laptop 9,521 ms (−0.22 %, limit ±1 %); status lines exactly 500 ms apart |
| Telemetry, link idle | PASS | 249 frames in 5 s (49.8 Hz), 0 bad CRC; disarmed, link down, all ESCs at 1,500 µs |
| Command round trip | PASS | link up 44 ms after the first command; 150 / 150 values exact; 38 ms median, 55 ms max |
| Arm flag | PASS | armed in 50 / 50 frames while commanded; ESCs stay at 1,500 µs (gains are zero) |
| Junk and split frames | PASS | 146 / 146 frames recovered, 0 values never sent |
| Corrupted frames | PASS | 0 bad-CRC values used; link dropped 528 ms after the last good frame |
| Command-timeout failsafe | PASS | 3 / 3 runs: disarmed at 542, 537, 535 ms |
| Soak, 60 s both ways | PASS | 3,002 frames at 50.03 Hz; 0 CRC errors, 0 ring-buffer drops, 0 link drops, 0 resets |
| Timing | PASS | tick 19,999.66–20,000.32 µs; ISR 8.3 µs avg / 45.6 µs max; main loop 17.77 ms worst; TX 5.292 ms |
| Stack | PASS | 872 B peak of 1,024 B reserved |
| I2C vs MPU-6050 | PASS | `PWR_MGMT_1` = 0x00 after wake; 187 cycles × 4 reads, 0 errors; 14-byte read 1,572 µs |
| Eight PWM outputs | PASS (`0ed4c05`) | T1–T8 at 1,100–1,800 µs exactly, 25 pulses each; period 20,000 µs on all 8 |
| Watchdog | PASS | 3 / 3 hangs reset in 563, 562, 563 ms; reset cause `IWDG PIN`; LSI ≈ 28.5 kHz |

The PWM test drives channel *k* at 1,100 + 100*k* µs, so a pin carrying the
wrong channel fails. It checks the routing, not only that a pulse exists. The
watchdog test stops the main loop while TIM7 keeps running the control loop,
which shows that a live timer interrupt does not keep the watchdog fed.

### Defects found on hardware

Three defects were found on the first two runs, each in code that had compiled
cleanly and passed every off-target check. The first run failed 13 of 14 tests;
the next two runs were 13 / 14 and 14 / 14.

| Defect | How it showed | Root cause | Fix |
|---|---|---|---|
| Boot hung in `iwdg_init()` | Console stopped after `LINK`, on every reset, with no watchdog reset | PR written, then an unbounded spin on `SR.PVU` *before* the watchdog was started; the update never completed | ST's order: start, unlock, write PR/RLR, bounded wait, reload. `0ed4c05` |
| TIM2 prescaler never loaded | Found while bringing up the capture; would have made every capture and `micros()` reading 90× fast | PSC is preloaded and only latches on an update event, 2³² counts (47.7 s) away | `EGR.UG` after writing PSC. `0ed4c05` |
| `i2c_write()` dropped the last byte of multi-byte writes | MPU-6050 test: exactly 2 implausible values per cycle over 187 cycles, 0 bus errors | STOP set right after the last byte entered DR, without waiting for BTF; the wake command `{0x6B, 0x00}` lost its `0x00` and the sensor stayed asleep | Wait for BTF before STOP; `PWR_MGMT_1` read back to prove it. `5a2af27` |

### Running it

```
pip install pyserial
python tools/hil/hil_test.py          # press RESET when prompted; move the PA15 jumper 8 times
```

Wiring, test descriptions and the host simulator are in
[`tools/hil/README.md`](tools/hil/README.md).

---

## Key figures

Every number below is either measured on a NUCLEO-F446RE by
[`tools/hil/hil_test.py`](tools/hil/), with the report committed in
[`tools/hil/reports/`](tools/hil/reports/), or read from a build output. The
source column says which.

| | Figure | Source |
|---|---|---|
| **Scope** | | |
| Hand-written firmware | 2,245 lines of C (cloc), 843 lines of comments; no HAL/LL (ST startup, `system_stm32f4xx.c`, CMSIS excluded) | cloc |
| Peripherals driven at register level | 17 blocks: RCC, PWR, FLASH, GPIO, USART1, USART2, DMA2, SPI1, I2C1, TIM2, TIM3, TIM4, TIM7, TIM12, IWDG, CRC, ADC1; plus SysTick, NVIC and DWT in the core | source |
| Thruster outputs | 8 PWM channels on 3 timers (TIM3, TIM4, TIM12), 1 µs resolution, 50 Hz | source |
| Wire protocol | 62-byte frames, CRC-16/IBM-3740, byte-identical to the vehicle's RP2350 firmware | off-target test |
| Defects found and fixed, each with root cause and commit | 28 (3 of them found on hardware) | [Engineering log](#engineering-log) |
| Test harness | 14 automated HIL tests, ~6 min per run; host simulator for development without a board | `tools/hil/` |
| **Timing, measured on silicon** | | |
| Control-tick period (TIM7, 50 Hz) | 19,999.66–20,000.32 µs over the whole run: worst 0.34 µs off nominal (17 ppm) | DWT, report `5a2af27` |
| Control ISR (PID + 6×8 allocation + 8 PWM writes) | 8.3 µs average, 45.6 µs worst: 0.23 % of the 20 ms tick | DWT, report `5a2af27` |
| Main-loop pass | 4 µs average, 17.77 ms worst: 19× inside the watchdog's 341 ms shortest timeout | DWT, report `5a2af27` |
| UART baud rate | 62-byte frame sent in 5.292 ms vs 5.295 ms theoretical at 115,200 baud (−0.06 %) | DWT, report `5a2af27` |
| System clock vs host clock | within 0.22 % over 9.5 s (limit 1 %; USB timestamp jitter dominates) | report `5a2af27` |
| **Link and failsafe, measured on silicon** | | |
| Soak, 60 s at 50 Hz both ways | 3,002 frames, 0 CRC errors, 0 dropped bytes, 0 link drops, 0 resets | report `5a2af27` |
| Data integrity | 3,002 / 3,002 random float32 values returned bit-exact | report `5a2af27` |
| Parser robustness | 146 / 146 frames recovered through random junk, decoy headers and split writes | report `5a2af27` |
| Command-timeout failsafe (500 ms) | 3 / 3 runs disarmed; 535–542 ms host-measured incl. USB latency | report `5a2af27` |
| Corrupted-frame rejection | 0 bad-CRC values ever acted on; link dropped 528 ms after the last good frame | report `5a2af27` |
| Host → board → host round trip | 38 ms median, 53 ms p95 (Windows USB-serial latency included) | report `5a2af27` |
| **Outputs and peripherals, measured on silicon** | | |
| PWM outputs | 8 / 8 pins carry their own channel; 200 pulses captured, 0 µs error at 1 µs resolution; 20,000 µs period on every pin | TIM2 capture, report `0ed4c05` |
| I2C against a real slave (MPU-6050) | 748 transfers (1, 2, 6 and 14 bytes), 0 errors, 0 bus errors; 1.047 g at rest, 32.3 °C | report `5a2af27` |
| I2C bus rate | 14-byte burst read in 1,572 µs vs ~1,550 µs computed for 100 kHz | report `5a2af27` |
| Watchdog | 6 / 6 deliberate main-loop hangs reset in 561–563 ms; reset cause read back as IWDG | reports `0ed4c05`, `5a2af27` |
| This chip's LSI oscillator | ≈ 28.5 kHz, derived from the watchdog timeout (datasheet range 17–47 kHz) | report `5a2af27` |
| Boot to main loop, nothing attached | 62–95 ms, SD and Bar30 absence handled without hanging | reports |
| **Memory** | | |
| Flash, vehicle build | 16,472 B, 3.1 % of 512 KiB | `arm-none-eabi-size` |
| Flash, bench build (with test hooks) | 20,192 B, 3.9 % | `arm-none-eabi-size` |
| Static RAM, vehicle build | 4,384 B, 3.3 % of 128 KiB | `arm-none-eabi-size` |
| Stack high-water mark | 872 B peak (painted MSP), against a 1,024 B linker reservation | report `5a2af27` |
| SD write amplification | 1 block write per 12 log records: 250 writes for 3,000 records, vs 3,000 before | off-target test |
| Compiler warnings at `-O2` | 2, both marking known gaps ([TODO.md](TODO.md)) | build |

---

## Engineering log

28 defects, each with its effect, root cause and the commit that fixed it.
24 were found by auditing the code against the reference manual and this
repository's own documentation, 1 by the compiler, and 3 on hardware. Every
commit message records what broke, why, and what changed.

### Found on hardware

| Defect | Effect | Fix |
|---|---|---|
| `iwdg_init()` spun on `SR.PVU` before starting the watchdog | Boot hung after `LINK`, every time, with no reset | `0ed4c05` |
| TIM2 PSC written but never loaded (no `EGR.UG`) | `micros()` and PWM capture 90× fast for the first 47.7 s after boot | `0ed4c05` |
| `i2c_write()` set STOP before BTF | Last byte of every multi-byte write dropped; MPU-6050 never left sleep | `5a2af27` |

### Found by the compiler

| Defect | Effect | Fix |
|---|---|---|
| `control_loop_get_pwm()` handed `&tp.esc_pwm`, a member of a packed struct | Unaligned `uint16_t*`: undefined behaviour (`-Waddress-of-packed-member`) | `8996036` |

### Bug class: read-modify-write onto a field that isn't zero

The most repeated mistake in this codebase: `|=` applied to a multi-bit field
without clearing it first, or plain `=` wiping fields set by an earlier write.

| Instance | Effect | Fix |
|---|---|---|
| `RCC->PLLCFGR \|=` onto reset value `0x24003010` | PLLM 24 / PLLN 488, chip at ~81 MHz, APB1 overclocked | `ba3acde` |
| `SysTick->CTRL` written three times with `=` | `TICKINT = 0`: `g_tick` never advanced, boot hung at step 11 | `ba3acde` |
| PA6 AFR: SPI1 writes AF5, PWM ORs AF2 | `5 \| 2 = 7`: PA6 at AF7, SPI MISO and TIM3_CH1 both dead | `875d596` |
| PA9/PA10 AFR OR'd without clearing | Latent; correct only from reset | `3b13a8c` |
| SPI1 (PA5–7) and I2C1 (PB8/9) AFR OR'd without clearing | Latent, same shape as PA6 | `f52b4a3` |
| `I2C1->CR2 \|= 45` onto the FREQ field | Latent; correct only from reset | `c46dd76` |

### Pin ownership

| Defect | Fix |
|---|---|
| OLED reset/DC on PA2/PA3 turned USART2 TX/RX into GPIO; debug console died mid-boot | `817c453` |
| PA4 set as DAC output, then taken as OLED chip-select; DAC left enabled on a digital output | `c2bfcf0` |
| Heartbeat toggled PA5 through ODR while PA5 was SPI1_SCK in AF mode; no effect | `dd08328` |
| TIM3_CH1 on PA6, shared with SPI1_MISO | `875d596` |

### Functional

| Defect | Fix |
|---|---|
| Frames were 64 bytes with hardware CRC-32; the Pi and RP2350 expect 62 bytes with CRC-16/IBM-3740 | `2ee1292` |
| `pwm_set_us()` ignored its channel; 7 of 8 thrusters never driven | `875d596` |
| USART1 transmitter never enabled (`CR1.TE`) | `3b13a8c` |
| `sd_logger_init()` never called; first log write aimed at block 0 (the MBR) | `2e39208` |
| `LogRecord.timestamp_ms` never assigned; stack garbage went into the CRC | `2e39208` |
| Every 20 ms record written as a full 512-byte block, blocking, from a 512-byte stack buffer | `98792ff` |
| ESC pins floating until after SD and Bar30 init, both of which can hang | `3245126` |
| Every I2C wait unbounded with no NACK check: with no Bar30, boot hung before the watchdog was armed | `c46dd76` |
| SPI MISO floated with no SD card, so `sd_init()` read noise | `c46dd76` |
| RX ring buffer count updated from an ISR and the main loop with no guard; no overflow check | `e7b6d70` |
| `telem_pending` never set, so telemetry was never sent | `b8afa51` |
| Link only on USART1, which the Nucleo doesn't route: no way to test the protocol with just the board | `dc75bce` |
| OLED reset delay was a NOP loop, about 2 ms instead of 10 ms at 180 MHz | `1ab6e90` |
| Over-drive never enabled, CFGR prescalers wiped, LSI not started | `ba3acde` |

---

## Off-target verification

Checks made without hardware during development, which the HIL runs above then
confirmed on silicon where they overlap.

| Claim | How it was checked | Result |
|---|---|---|
| Builds with the pinned flags | Full compile and link, arm-none-eabi-gcc 13.2.1, at every commit from `c2bfcf0` on, in both link builds | 0 errors, 6 warnings throughout |
| Frames are byte-identical to pico-protocol | Telemetry frame built by `packet_build_telemetry()` compared with pico-protocol's C encoder and decoded by its Python parser | Identical, all fields decode |
| Parser resyncs and rejects corruption | CMD frame from pico-protocol's Python encoder, placed after junk and a PID frame; single-bit flip | Parsed; bit flip rejected |
| CRC-16 parameters | Check value of `"123456789"` | `0x29B1`, the published value |
| PWM registers and routing | `pwm_init()` run on an emulated Cortex-M4 against simulated TIM/GPIO/RCC registers, at `-O0`, `-O2`, `-Os` | PSC/ARR/CCMR/CCER correct; each channel writes exactly one CCR; clamps hold |
| Pin ownership after boot | Real init functions, `link_init()` included, run in `main()` order against simulated registers | Every claimed pin in the state shown in [Pin map](#pin-map), pull-ups included |
| I2C never hangs | `i2c_write`/`i2c_read`/`bar30_init` against simulated I2C registers and a simulated millisecond clock | NACK exits before the timeout; stuck bus times out and the peripheral is reconfigured; absent Bar30 fails in one transfer |
| RX ring buffer | Overflow, underflow and 7000 bytes through the ring; disassembly of `rx_eat()` | Drops counted, no underflow, order kept; both stores between `cpsid i` and the PRIMASK restore |
| USART2 link | `USART2_IRQHandler` fed a CMD frame one byte per interrupt after junk and a decoy sync pair | `packet_parse_cmd()` recovers it; USART2 has TE/RE/RXNEIE/UE, IRQ at priority 1 |
| SD block packing | Logger run against a simulated card: 60 s at 50 Hz, partial flush, write failure, dead card | 3000 records → 250 writes, in order; partial block completed in place; failures retried; no loss beyond the documented case |
| Empty log slots fail the CRC | CRC-32/MPEG-2 model of `crc_hw.c`, itself checked against the published check value | Zero slot stores `0x0000`, CRC is `0xC868`: rejected |
| Stack cost of logging | `-fstack-usage` | `sd_logger_write()` 528 → 24 bytes |
| HIL script | Run against a host simulator built from the firmware's own protocol and control-loop sources, and against three deliberately broken builds of it | Passes on the correct build; each broken build fails the tests aimed at its bug |

**Not verified anywhere yet:**

- USART1 with DMA on PA9/PA10 (the vehicle link), SD card writes to a real
  card, the OLED, the Bar30, and anything with ESCs or thrusters attached.
- The project's pinned 11.3.1 toolchain. Builds and warning counts are from 13.2.1.
- Alternate-function numbers against a primary reading of DS10693 Table 11
  (all 8 PWM pins and the I2C, USART2 and capture pins are confirmed on
  hardware; the SPI and USART1 pins are not).
- The emulated register-level harnesses used during development are not in
  the repository; the HIL script and its simulator are.

---

## Build

**Toolchain (pinned for the project)**

```
arm-none-eabi-gcc (GNU Tools for STM32 11.3.rel1.20230912-1600) 11.3.1 20220712
```

**Flags**

```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
-std=gnu11 -O2 -g3 -DSTM32F446xx -DHSE_VALUE=8000000U -Wall -Wdouble-promotion
-ffunction-sections -fdata-sections --specs=nano.specs
LD: -T STM32F446RETX_FLASH.ld -Wl,--gc-sections --specs=nosys.specs -lc -lm
```

**Build options**

| Define | Default | Effect |
|---|---|---|
| `LINK_PORT_STLINK` | `1` | Pi link on USART2 over the ST-LINK USB cable (bench). `0`: USART1 on PA9/PA10 with DMA (vehicle) |
| `BENCH_HIL` | `1` | HIL instrumentation in `Core/Src/bench.c`: DWT timing, stack painting, PA15 capture, MPU-6050 checks, `BENCH:` commands. `0`: none of it compiled |

**The vehicle build is `-DLINK_PORT_STLINK=0 -DBENCH_HIL=0`.** The bench build
can drive the ESC outputs off neutral on command and stop the watchdog
refresh; it must never be flashed to a vehicle with ESCs attached. In CubeIDE:
Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor.

**Size at `-O2`** (arm-none-eabi-gcc 13.2.1; 11.3.1 will differ slightly)

| Build | text | data | bss | Flash (text + data) | Static RAM (data + bss) |
|---|---|---|---|---|---|
| Vehicle (`LINK_PORT_STLINK=0`, `BENCH_HIL=0`) | 16,372 | 100 | 4,284 | 16,472 B (3.1 % of 512 KiB) | 4,384 B (3.3 % of 128 KiB) |
| Bench (defaults) | 20,092 | 100 | 4,536 | 20,192 B (3.9 %) | 4,636 B (3.5 %) |

The RAM figures include the linker's 1,536-byte heap and stack reservation
(`_Min_Heap_Size 0x200`, `_Min_Stack_Size 0x400`) and the logger's 512-byte
block buffer. The measured stack peak is 872 B; the stack can grow past the
reservation into free RAM, but the reservation should be raised (TODO.md).

Two warnings remain at `-O2`, both deliberate markers of unfinished work:
`TEMP` set but unused in `bar30.c` (second-order temperature compensation not
implemented) and `r7` set but unused in `sd_card.c` (the CMD8 check pattern is
read and not verified).

> The CubeIDE **Release** configuration has never been set up for this
> project: it has only the stock include paths and still defines
> `USE_HAL_DRIVER`. Use the Debug configuration or the command line above.

---

## Repository layout

```
firmware/
├── STM32F446RETX_FLASH.ld / _RAM.ld   linker scripts
├── Core/
│   ├── Inc/        bench.h  control_loop.h  main.h  stm32f4xx_it.h  system_init.h
│   ├── Src/        main.c              boot sequence and main loop
│   │               system_init.c       clock tree, over-drive, SysTick, delay_ms
│   │               control_loop.c      PID, allocation, PWM mapping, failsafe
│   │               stm32f4xx_it.c      SysTick and TIM7 handlers
│   │               bench.c             BENCH_HIL instrumentation (DWT, capture, MPU-6050)
│   │               syscalls.c  sysmem.c  system_stm32f4xx.c*
│   └── Startup/    startup_stm32f446retx.s*
├── Drivers/
│   ├── timer/      timer_pwm (8-ch ESC PWM)  timer_basic (TIM7 tick)  timer_timebase (TIM2 µs)
│   ├── uart/       uart (USART2 console)  uart_packet (USART1 + DMA RX)  link (link port select)
│   ├── spi/  i2c/  gpio/  iwdg/  crc_hw/  adc/  dac/
├── Devices/
│   ├── bar30/      MS5837 pressure sensor
│   └── oled/       SSD1306 over SPI
└── Protocol/
    ├── packet.c/.h        frame build/parse, CRC-16
    ├── struct.h           payload layouts
    ├── ring_buffer.c/.h   UART RX buffer (ISR writer, main-loop reader)
    └── sd_card/           sd_card (SPI SD driver)  sd_logger (block-packed logging)
tools/
└── hil/
    ├── hil_test.py        hardware-in-the-loop test over the ST-LINK USB port
    ├── reports/           committed reports from hardware runs (the evidence for this README)
    └── sim/               host simulator built from the firmware's own sources
```

`*` ST-generated.

---

## References

- **RM0390**: STM32F446xx reference manual
- **PM0214**: Cortex-M4 programming manual
- **DS10693**: STM32F446xC/E datasheet (alternate functions, LSI characteristics)
- **UM1724**: STM32 Nucleo-64 boards user manual (header pinout, solder bridges)
- **pico-protocol**: the RP2350 link implementation this board's framing matches
- **PM0214 §DWT** and **RM0390 §IWDG, §I2C master transmitter (EV8_2), §RCC_CSR**: the sections the hardware fixes rest on
