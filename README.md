# Nucleo_AUV_Bare_Metal

Bare-metal peripheral driver stack for the STM32F446RE (Nucleo-F446RE), written
without HAL or LL. Every peripheral register access in `Drivers/`, `Devices/`,
and `Protocol/` is direct and derived from RM0390 and PM0214.

**One exception, stated up front:** `Core/Src/system_stm32f4xx.c` is
ST-generated and is not mine. Its `SystemInit()` is called from
`startup_stm32f446retx.s` before `main()` and enables the FPU via `SCB->CPACR`.
Everything else in the boot path — clock tree, SysTick, all peripheral init —
is hand-written in `system_init.c` and the driver modules.

Built as groundwork for an AUV control node for Team Tiburon (SAUVC 2026).
**This firmware has never been deployed on the vehicle** — the AUV runs separate
firmware on an RP2350. This is a driver stack and a study of the silicon, not
flight software.

---

## Status

| | |
|---|---|
| Builds at `-O2` | Yes. Six warnings, all benign, none in the clock or watchdog paths |
| Verified on hardware | **No** |
| Deployed on a vehicle | **No** |

The firmware compiles and links. It has not been confirmed to reach the main
loop on hardware. `bar30_init()` calls `i2c_write()`, which polls `SB` and
`ADDR` with no timeout — if the Bar30 does not ACK, boot blocks there. The same
pattern exists in `sd_init()`'s SPI polls.

Outstanding warnings: `B_forward` unused, `TEMP` set-but-unused in `bar30.c`,
`write_data` unused in `oled.c`, `r7` set-but-unused in `sd_card.c`, and two
`-Waddress-of-packed-member` in `main.c`.

---

## Build

**Toolchain**

```
arm-none-eabi-gcc (GNU Tools for STM32 11.3.rel1.20230912-1600) 11.3.1 20220712
```

**Flags**

```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
-std=gnu11 -O2 -g3 -DSTM32F446xx -Wall
-ffunction-sections -fdata-sections --specs=nano.specs
LD: -T STM32F446RETX_FLASH.ld -Wl,--gc-sections --specs=nosys.specs -lc -lm
```

**Size at `-O2`**

```
   text	   data	    bss	    dec	    hex
  12156	    100	   3756	  16012	   3e8c
```

| | Bytes | |
|---|---|---|
| Flash (`text + data`) | **12,256** | 11.97 KiB of 512 KiB |
| Static RAM (`data + bss`) | 3,856 | 3.77 KiB of 128 KiB |

Of that RAM figure, 1,536 bytes are the linker-reserved heap+stack region
(`_Min_Heap_Size 0x200`, `_Min_Stack_Size 0x400`) — already counted inside the
`bss` column, not additional to it. Peak stack usage within that region is
**unmeasured**; no stack painting is implemented.

> The CubeIDE **Release** configuration has never been set up to build this
> project. It carries only the five stock include paths, none of the driver
> paths, and still defines `USE_HAL_DRIVER`. All builds are the Debug config or
> the pinned command line above.

---

## Clock tree

HSE 8 MHz (bypass, from the ST-LINK MCO) → PLL → 180 MHz SYSCLK.

| Stage | Value | Result |
|---|---|---|
| PLLM | 8 | VCO input 1 MHz |
| PLLN | 360 | VCO output 360 MHz |
| PLLP | `0b00` (encodes /2) | SYSCLK 180 MHz |
| HPRE | /1 | AHB 180 MHz |
| PPRE1 | /4 | APB1 45 MHz (max) |
| PPRE2 | /2 | APB2 90 MHz (max) |
| `FLASH->ACR` | 5 wait states | RM0390 table, 150 < HCLK ≤ 180 MHz @ 2.7–3.6 V |

180 MHz requires the over-drive regulator mode, enabled after PLL lock and
before the SYSCLK switch:

```
APB1ENR.PWREN → PWR_CR.ODEN → PWR_CSR.ODRDY → PWR_CR.ODSWEN → PWR_CSR.ODSWRDY
```

`PWR_CR.VOS` is relied upon at its reset value (Scale 1). Known fragility: a
warm reset or bootloader path leaving VOS at Scale 2 turns the `ODRDY` poll into
a silent infinite hang. ST's reference sequence writes VOS explicitly.

PLLQ and PLLR stay at reset (4 and 2). With a 360 MHz VCO, PLLQ yields 90 MHz —
not the 48 MHz USB OTG requires, so this VCO cannot clock USB from the main PLL.

References: RM0390 §6.3.2 (PLLCFGR), §6.3.3 (CFGR), PWR chapter, §3.4.1 (ACR).

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
| 47 kHz (max) | **341 ms** | Shortest timeout — the kick budget |
| 32 kHz (typ) | 501 ms | Nominal |
| 17 kHz (min) | **943 ms** | Longest — worst-case detection latency |

The kick interval is designed against 341 ms, not 501 ms. The safety bound —
how long a hung MCU can sit before reset — is 943 ms plus reset and re-init.

`iwdg_kick()` is called from the **main loop**, deliberately not from the TIM7
ISR. A timer ISR fires regardless of whether the main loop is making forward
progress, so kicking from it would produce a watchdog that only proves the timer
peripheral is alive.

Init sequence: `RCC_CSR.LSION` → `LSIRDY` → `KR = 0x5555` → `PR` → poll
`SR.PVU` → `RLR` → poll `SR.RVU` → `KR = 0xCCCC`. The `PVU`/`RVU` waits exist
because `PR` and `RLR` live in the LSI clock domain; at 180 MHz core against a
~32 kHz LSI, writing them back to back without waiting leaves the register
contents indeterminate for thousands of core cycles.

The kick interval is **unmeasured** — main-loop worst-case iteration time has
never been instrumented.

---

## Peripheral drivers

**Exercised at runtime**

| Peripheral | Pins | Notes |
|---|---|---|
| USART2 | PA2 / PA3, AF7 | Debug output — see pin conflicts |
| USART1 | PA9 TX / PA10 RX, AF7 | DMA2 Stream 2, CHSEL 4, circular RX, IDLE-line detect |
| I2C1 | PB8 / PB9, AF4 | Open-drain, 100 kHz, CCR 225, TRISE 46 |
| SPI1 | PA5 SCK / PA6 MISO / PA7 MOSI, AF5 | Shared bus: OLED (CS on PA4) and SD card (CS on PB0) |
| TIM3_CH1 | PA6 | 50 Hz PWM, single channel only |
| TIM7 | — | 50 Hz control-loop tick |
| CRC | — | Hardware CRC32 over packet payloads |
| IWDG | — | See above |

**Present but never called at runtime**

| Driver | Status |
|---|---|
| ADC1 | `adc_read()` — zero call sites |
| DAC1 | `dac_set()` — zero call sites |
| TIM2 (µs timebase) | `micros()` — zero call sites |
| Bar30 / MS5837 | `bar30_read()` — zero call sites; only `bar30_init()` runs |

---

## Serial protocol

64-byte fixed frames, `0xAA 0x55` sync, length byte, message ID, payload,
hardware CRC32. Receive is DMA-driven with IDLE-line detection into a circular
buffer; the IDLE flag is cleared by the `SR`-then-`DR` read sequence.

`_Static_assert(sizeof(TelemetryPayload) == 56)` guards payload layout.

**Transmit is non-functional.** `uart1_init()` sets RE, IDLEIE, DMAR, and UE but
never `USART_CR1_TE` — the transmitter is disabled. Separately, `telem_pending`
is only ever set to zero (its initializer and one assignment in `main.c`) and
never to one, so the telemetry branch in `main.c` is unreachable. Receive works;
transmit does not.

---

## Control loop

TIM7 drives a 50 Hz ISR containing PID, 6-DOF thrust allocation via a
pseudo-inverse `B` matrix, PWM mapping with a slew limiter, and a command
timeout failsafe.

**The structure is complete; the gains are not.** `kp`, `ki`, `kd`, `kff`, and
`FF_OFFSET` are all-zero arrays, so `computePID()` returns zero on every axis,
allocation produces zeros, and `thrust_to_pwm()` returns neutral. This is
control-loop scaffolding awaiting tuning, not a tuned controller. `I_CLAMP` and
`U_MAX` are non-zero but only clamp values that are already zero.

`B_forward` is defined and never referenced — only `B_pinv` is used.

---

## Known defects

Found by auditing this repository against its own documentation.

**A recurring bug class: read-modify-write onto a non-zero field.** Three
instances, all the same mistake — `|=` applied without first clearing the field:

1. `RCC->PLLCFGR |= ...` onto the register's non-zero reset value
   (`0x24003010`), giving `16 | 8 = 24` and `192 | 360 = 488`. The chip ran at
   ~81 MHz while every downstream constant assumed 180 MHz. **Fixed.**
2. `SysTick->CTRL` written three times with `=`, each overwriting the last,
   leaving `TICKINT = 0` and `CLKSOURCE = 0`. `g_tick` never incremented,
   `delay_ms()` never returned, boot hung at step 11 of 18. **Fixed.**
3. PA6's alternate-function field: `spi1_init()` writes AF5, then
   `timer3_pwm_init()` ORs in AF2 without clearing. `5 | 2 = 7`. PA6 ends at
   **AF7 — neither SPI1_MISO nor TIM3_CH1.** Both peripherals lose the pin.
   **Open.**

**Other open defects**

- **PA4** is claimed by DAC1 (analog) and then by SPI1 as OLED chip-select
  (output). SPI clears the field first, so it wins cleanly — DAC1's output pin
  is gone.
- **PA2/PA3** are reprogrammed from AF7 to GPIO output by the OLED reset/DC
  init, killing the debug UART from that point onward.
- **PA5** is toggled as a heartbeat LED via `GPIOA->ODR` while in AF mode.
  Writing ODR on an AF pin does nothing; the LED is dead.
- **`pwm_set_us(uint8_t channel, uint16_t us)` ignores `channel`** and always
  writes `TIM3->CCR1`. Only one PWM channel exists.
- **`sd_logger_init()` has zero call sites**, so `current_block` stays at its
  initializer of 0 and the first log write targets block 0 — the card's MBR.
  The `current_block = 100` that would skip the partition table lives inside
  the function that is never called.
- **`LogRecord.timestamp_ms` is never assigned.** `LogRecord rec;` is an
  uninitialized stack object and the CRC is computed over every byte including
  that garbage. `crc_compute()` returns `uint32_t` into a `uint16_t` field —
  silent truncation.
- **Ring buffer is unsynchronized.** `rx_write()` runs in `USART1_IRQHandler`,
  `rx_eat()` runs in the main loop, and `rx_head` / `rx_tail` / `rx_count` are
  non-`volatile` and unguarded. `rx_head` is masked with `% RX_BUF_SIZE` so the
  array is never indexed out of range — but `rx_count` increments unbounded past
  `RX_BUF_SIZE` and head silently laps tail. `rx_eat()` can underflow
  `rx_count`.
- **I2C and SPI poll flags without timeouts**, so an unresponsive device blocks
  boot indefinitely.
- **`Protocol/struct.c` is a zero-byte file** that is compiled and linked.

---

## What this repository is for

The value here is not that the firmware flies — it does not. It is that every
register write is one derived from the reference manual and accountable,
including the ones that were wrong. The three-instance bug class above was found
by auditing my own code, and the fix for each is in the commit history with the
reasoning attached.

---

## Layout

```
firmware/
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
├── Core/
│   ├── Inc/       control_loop.h  main.h  stm32f4xx_it.h  system_init.h
│   ├── Src/       control_loop.c  main.c  stm32f4xx_it.c  syscalls.c
│   │              sysmem.c  system_init.c  system_stm32f4xx.c*
│   └── Startup/   startup_stm32f446retx.s
├── Drivers/
│   ├── adc/  crc_hw/  dac/  gpio/  i2c/  iwdg/  spi/  uart/
│   └── timer/     timer_basic  timer_pwm  timer_timebase
├── Devices/
│   ├── bar30/     bar30.c/.h
│   └── oled/      oled.c/.h
└── Protocol/
    ├── packet.c/.h  ring_buffer.c/.h  struct.c/.h
    └── sd_card/   sd_card.c/.h  sd_logger.c/.h
```

`*` ST-generated, not hand-written. See the note at the top of this file.

---

## References

- **RM0390** — STM32F446 reference manual
- **PM0214** — Cortex-M4 programming manual
- **DS10693** — STM32F446xC/E datasheet (LSI characteristics, alternate functions)
