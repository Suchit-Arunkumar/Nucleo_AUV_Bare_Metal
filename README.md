# Nucleo_AUV_Bare_Metal

Register-level firmware for an AUV control node on the STM32F446RE
(NUCLEO-F446RE), written without HAL or LL. It brings up the clock tree,
eight ESC PWM channels, a DMA-driven binary link to a Raspberry Pi, an SD
card logger, an I2C pressure sensor, an SPI OLED, a 50 Hz control loop and an
independent watchdog. Every peripheral access in `Drivers/`, `Devices/` and
`Protocol/` is a direct register write traceable to RM0390 (reference manual),
PM0214 (Cortex-M4 programming manual) or DS10693 (datasheet).

Built as groundwork for Team Tiburon's AUV (SAUVC 2026). **It has never been
deployed on the vehicle**, which runs separate firmware on an RP2350. This
repository is a driver stack and a study of the silicon, not flight software,
and it says so wherever that matters.

**Scope note:** `Core/Src/system_stm32f4xx.c` and the startup file are
ST-generated. `SystemInit()` runs before `main()` and enables the FPU through
`SCB->CPACR`. Everything after that, the clock tree included, is in
`system_init.c` and the driver modules.

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
12. [Engineering log](#engineering-log)
13. [Verification](#verification)
14. [Build](#build)
15. [Repository layout](#repository-layout)
16. [References](#references)

Open issues are listed in [TODO.md](TODO.md).

---

## Status

| | |
|---|---|
| Builds at `-O2` | Yes: 0 errors, 6 warnings, none in clock, watchdog, PWM, protocol or logging code |
| Verified on hardware | **No** |
| Deployed on a vehicle | **No** |
| Off-target verification | Protocol framing, PWM register setup, pin ownership after boot and SD block packing, run on an emulated Cortex-M4 during development ([details](#verification)); these harnesses are not yet in the repository |

The six warnings are `B_forward` unused (`control_loop.c`), `TEMP` set but unused
(`bar30.c`), `write_data` unused (`oled.c`), `r7` set but unused (`sd_card.c`)
and two `-Waddress-of-packed-member` in `main.c`.

The firmware compiles and links. It has not been confirmed to reach the main
loop on hardware: `bar30_init()` and `sd_init()` sit on I2C and SPI status
polls with no timeout, so a device that doesn't respond stops boot there. The
ESC outputs are brought to neutral before either of them runs, so a hang there
leaves the thrusters at a defined idle signal rather than floating.

---

## System overview

```
                    ┌──────────────────────── STM32F446RE @ 180 MHz ────────────────────────┐
  Raspberry Pi ◄──► │ USART1 115200  DMA2 S2 circular RX + IDLE IRQ → ring buffer → parser  │
  (fused nav state, │   62-byte frames, CRC-16/IBM-3740 (same framing as pico-protocol)     │
   PID gains)       │                                                                        │
                    │ TIM7 50 Hz ISR ── PID ── 6-DOF allocation (B⁺) ── slew limit ──┐       │
                    │                                                                ▼       │
  8 × ESC ◄──────── │ TIM3 CH1-4, TIM4 CH1-2, TIM12 CH1-2   50 Hz, 1 µs resolution           │
                    │                                                                        │
  SD card ◄──────── │ SPI1 ── 12 log records per 512-byte block                              │
  SSD1306 OLED ◄─── │ SPI1 (shared bus, separate chip-selects)                               │
  Bar30 (MS5837) ◄─ │ I2C1 100 kHz                                                           │
  ST-LINK VCP ◄──── │ USART2 printf console                                                  │
                    │ IWDG kicked from the main loop only                                    │
                    └────────────────────────────────────────────────────────────────────────┘
```

**Data flow per 20 ms tick.** TIM7 fires, sets `log_pending`, toggles the tick
probe pin, and runs `control_loop_tick()`: command-timeout check, PID on six
axes, thrust allocation through the pseudo-inverse of the thruster geometry
matrix, then slew-limited PWM writes to eight channels. The main loop parses
incoming frames from the Pi, builds and writes a log record, services the
logger's flush timer, and kicks the watchdog.

---

## Pin map

The pin state after the complete boot sequence, as each init function leaves
it. Header positions are from UM1724 (Nucleo-64 user manual); check them against
your board revision before wiring.

| Pin | Mode | Function | Header | Configured by |
|---|---|---|---|---|
| PA0 | Analog | ADC1_IN0 (driver present, never read) | A0 | `adc_init()` |
| PA2 | AF7 | USART2_TX, debug console via ST-LINK VCP | ST-LINK | `uart2_init()` |
| PA3 | AF7 | USART2_RX | ST-LINK | `uart2_init()` |
| PA4 | Output | OLED chip-select | A2 | `spi1_init()` |
| PA5 | AF5 | SPI1_SCK (also drives LD2 through SB21) | D13 | `spi1_init()` |
| PA6 | AF5 | SPI1_MISO | D12 | `spi1_init()` |
| PA7 | AF5 | SPI1_MOSI | D11 | `spi1_init()` |
| PA8 | Output | OLED data/command | D7 | `oled_init()` |
| PA9 | AF7 | USART1_TX to the Pi | D8 | `uart1_init()` |
| PA10 | AF7 | USART1_RX from the Pi | D2 | `uart1_init()` |
| PA13 / PA14 | AF0 | SWDIO / SWCLK (reset default, untouched) | — | — |
| PB0 | Output | SD card chip-select | A3 | `spi1_init()` |
| PB5 | AF2 | TIM3_CH2, thruster T2 | D4 | `pwm_init()` |
| PB6 | AF2 | TIM4_CH1, thruster T5 | D10 | `pwm_init()` |
| PB7 | AF2 | TIM4_CH2, thruster T6 | CN7-21 | `pwm_init()` |
| PB8 | AF4, open-drain | I2C1_SCL, Bar30 | D15 | `i2c1_init()` |
| PB9 | AF4, open-drain | I2C1_SDA, Bar30 | D14 | `i2c1_init()` |
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
(SWO and NJTRST) are deliberately left alone.

---

## Boot sequence

From `main()`. The order is deliberate, and the reason is given where it matters.

| Step | Call | Why it is here |
|---|---|---|
| 1 | `system_clock_init()` | 180 MHz with over-drive; every later timing constant assumes it |
| 2 | `systick_init()` | 1 ms `g_tick`; `delay_ms()` depends on it |
| 2a | `timer3_pwm_init()` | ESC lines to 1500 µs neutral **before** anything that can block. Until this runs they are floating inputs |
| 3 | `gpio_init(PB10)` | Tick probe output |
| 4 | `uart2_init()` | Debug console, so later steps can report |
| 5 | `crc_init()` | Hardware CRC unit, used for log-record CRCs |
| 6 | `adc_init()` | PA0 analog; no reads yet |
| 8 | `spi1_init()` | Shared OLED/SD bus and both chip-selects, deselected |
| 9 | `sd_init()`, `sd_logger_init()` | Logger only initialised if the card came up; otherwise it never touches the card |
| 10 | `i2c1_init()` | 100 kHz, CCR 225, TRISE 46 |
| 11 | `bar30_init()` | Can hang if the sensor doesn't ACK (no timeout) |
| 12 | `oled_init()` | Reset pulse timed with `delay_ms()` |
| 13 | `uart1_init()` | Link to the Pi: DMA RX, IDLE interrupt, TX enabled |
| 15 | `timer2_timebase_init()` | 32-bit 1 MHz free-running counter for `micros()` |
| 16 | `control_loop_init()` | State reset, all channels neutral |
| 17 | `tim7_init()` | 50 Hz control ISR starts |
| 18 | `iwdg_init()` | Armed last, so slow init can't trip it |

Step 7 (DAC init) was removed; the DAC is not used and its pin belongs to the
OLED chip-select.

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

References: RM0390 §6.3.2 (PLLCFGR), §6.3.3 (CFGR), PWR chapter, §3.4.1 (ACR).

---

## Timing and interrupts

| Source | Rate | Priority | Does |
|---|---|---|---|
| SysTick | 1 kHz | 0 (reset default) | `g_tick++` |
| TIM7 update | 50 Hz | 0 | Control loop, `log_pending`, tick probe |
| USART1 IDLE | per burst | 1 | Copies new DMA bytes into the ring buffer |
| TIM3 / TIM4 / TIM12 | 50 Hz PWM | — | Hardware only, no interrupts |
| TIM2 | 1 MHz free-running | — | `micros()`, no interrupts |

TIM7 uses PSC 1799 and ARR 999: 90 MHz / 1800 / 1000 = 50 Hz, so the control
loop's `dt = 0.02 s` is exact by construction. PB10 toggles once per tick, so a
scope on it shows the tick rate and ISR entry jitter directly.

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

---

## Serial link to the Pi

USART1 at 115200 baud, framed identically to
[pico-protocol](https://github.com/Suchit-Arunkumar/pico-protocol), the
RP2350 firmware the vehicle actually runs, so the Pi-side parser accepts
frames from either board.

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

**Receive path.** DMA2 Stream 2 (channel 4) writes USART1 bytes into a 256-byte
circular buffer. The IDLE-line interrupt copies whatever arrived since the last
IDLE into a software ring buffer. The IDLE flag is cleared by reading SR, then
DR. The parser in `packet_parse_cmd()` then checks each candidate frame in turn:

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

**Limitations** (tracked in [TODO.md](TODO.md)):

- Telemetry transmit is built and framed correctly but never sent:
  `telem_pending` is never set to 1.
- PID frames are recognised and discarded; gains are compile-time constants.
- Telemetry `raw_depth_m` currently echoes the commanded depth, not the Bar30
  reading, because `bar30_read()` is not yet called.
- There is no protocol version byte.

---

## Control loop

TIM7's 50 Hz ISR runs PID on six axes (surge, sway, heave, roll, pitch, yaw),
then 6-DOF thrust allocation via the pseudo-inverse `B⁺` of the 6×8 thruster
geometry matrix. It renormalises the vertical, horizontal-translation and yaw
groups separately so no group saturates the others, maps thrust to PWM with a
deadzone and a 50 µs-per-tick slew limit, and runs a 500 ms command-timeout
failsafe. Yaw error is wrapped to ±180°.

**The structure is complete; the gains are not.** `kp`, `ki`, `kd`, `kff` and
`FF_OFFSET` are all zero, so every axis outputs zero and all thrusters sit at
neutral. This is scaffolding awaiting tuning, not a tuned controller.

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
| 17 kHz (min) | **943 ms** | Longest: worst-case detection latency |

The kick interval is designed against 341 ms, not 501 ms. The safety bound, how
long a hung MCU can sit before reset, is 943 ms plus reset and re-init.

`iwdg_kick()` is called from the **main loop**, deliberately not from the TIM7
ISR. A timer ISR fires whether or not the main loop is making progress, so
kicking from it would only prove the timer is alive.

Init sequence: `RCC_CSR.LSION` → `LSIRDY` → `KR = 0x5555` → `PR` → poll `SR.PVU`
→ `RLR` → poll `SR.RVU` → `KR = 0xCCCC`. `PR` and `RLR` live in the LSI clock
domain, so writing them back to back without the `PVU`/`RVU` waits leaves their
contents indeterminate for thousands of core cycles.

The main loop's worst-case iteration time has **not been measured**. A blocking
SD block write is the largest known contributor.

---

## Engineering log

Defects found by auditing the code against the reference manual and against
this repository's own documentation, with the commit that fixed each. Every
commit message records what broke, why, and what changed.

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
| OLED reset delay was a NOP loop, about 2 ms instead of 10 ms at 180 MHz | `1ab6e90` |
| Over-drive never enabled, CFGR prescalers wiped, LSI not started | `ba3acde` |

---

## Verification

What has been checked, how, and what has not. Hardware testing is still the
biggest gap.

| Claim | How it was checked | Result |
|---|---|---|
| Builds with the pinned flags | Full compile and link, arm-none-eabi-gcc 13.2.1, at every commit from `c2bfcf0` to `875d596` | 0 errors, 6 warnings throughout |
| Frames are byte-identical to pico-protocol | Telemetry frame built by `packet_build_telemetry()` compared with pico-protocol's C encoder and decoded by its Python parser | Identical, all fields decode |
| Parser resyncs and rejects corruption | CMD frame from pico-protocol's Python encoder, placed after junk and a PID frame; single-bit flip | Parsed; bit flip rejected |
| CRC-16 parameters | Check value of `"123456789"` | `0x29B1`, the published value |
| PWM registers and routing | `pwm_init()` run on an emulated Cortex-M4 against simulated TIM/GPIO/RCC registers, at `-O0`, `-O2`, `-Os` | PSC/ARR/CCMR/CCER correct; each channel writes exactly one CCR; clamps hold |
| Pin ownership after boot | Real init functions run in `main()` order against simulated registers | All 23 claimed pins in the state shown in [Pin map](#pin-map) |
| SD block packing | Logger run against a simulated card: 60 s at 50 Hz, partial flush, write failure, dead card | 3000 records → 250 writes, in order; partial block completed in place; failures retried; no loss beyond the documented case |
| Empty log slots fail the CRC | CRC-32/MPEG-2 model of `crc_hw.c`, itself checked against the published check value | Zero slot stores `0x0000`, CRC is `0xC868`: rejected |
| Stack cost of logging | `-fstack-usage` | `sd_logger_write()` 528 → 24 bytes |

**Not verified:**

- Anything on silicon: waveforms on the PWM pins, a live link to the Pi, SD
  writes to a real card, the OLED, the Bar30.
- The project's pinned 11.3.1 toolchain. Warning counts above are from 13.2.1.
- Worst-case main-loop time and peak stack use.
- Alternate-function numbers against a primary reading of DS10693 Table 11.
- None of the off-target harnesses are committed yet (see [TODO.md](TODO.md)).

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

**Size at `-O2`** (arm-none-eabi-gcc 13.2.1; 11.3.1 will differ slightly)

```
   text	   data	    bss	    dec	    hex
  15556	    100	   4284	  19940	   4de4
```

| | Bytes | |
|---|---|---|
| Flash (`text + data`) | 15,656 | 15.3 KiB of 512 KiB |
| Static RAM (`data + bss`) | 4,384 | 4.3 KiB of 128 KiB |

The RAM figure includes the linker's 1,536-byte heap and stack reservation
(`_Min_Heap_Size 0x200`, `_Min_Stack_Size 0x400`) and the logger's 512-byte
block buffer.

> The CubeIDE **Release** configuration has never been set up for this
> project: it has only the stock include paths and still defines
> `USE_HAL_DRIVER`. Use the Debug configuration or the command line above.

---

## Repository layout

```
firmware/
├── STM32F446RETX_FLASH.ld / _RAM.ld   linker scripts
├── Core/
│   ├── Inc/        control_loop.h  main.h  stm32f4xx_it.h  system_init.h
│   ├── Src/        main.c              boot sequence and main loop
│   │               system_init.c       clock tree, over-drive, SysTick, delay_ms
│   │               control_loop.c      PID, allocation, PWM mapping, failsafe
│   │               stm32f4xx_it.c      SysTick and TIM7 handlers
│   │               syscalls.c  sysmem.c  system_stm32f4xx.c*
│   └── Startup/    startup_stm32f446retx.s*
├── Drivers/
│   ├── timer/      timer_pwm (8-ch ESC PWM)  timer_basic (TIM7 tick)  timer_timebase (TIM2 µs)
│   ├── uart/       uart (USART2 console)  uart_packet (USART1 + DMA RX)
│   ├── spi/  i2c/  gpio/  iwdg/  crc_hw/  adc/  dac/
├── Devices/
│   ├── bar30/      MS5837 pressure sensor
│   └── oled/       SSD1306 over SPI
└── Protocol/
    ├── packet.c/.h        frame build/parse, CRC-16
    ├── struct.h           payload layouts
    ├── ring_buffer.c/.h   USART1 RX buffer
    └── sd_card/           sd_card (SPI SD driver)  sd_logger (block-packed logging)
```

`*` ST-generated.

---

## References

- **RM0390**: STM32F446xx reference manual
- **PM0214**: Cortex-M4 programming manual
- **DS10693**: STM32F446xC/E datasheet (alternate functions, LSI characteristics)
- **UM1724**: STM32 Nucleo-64 boards user manual (header pinout, solder bridges)
- **pico-protocol**: the RP2350 link implementation this board's framing matches
