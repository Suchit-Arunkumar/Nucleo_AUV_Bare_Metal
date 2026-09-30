# Known limitations

Open items in this repository, stated explicitly rather than left for a reader
to discover. Fixed defects are recorded in the README's engineering log and in
the commit history, not here.

## Verification gaps

- **Not yet run on hardware.** Every timing constant — `I2C1->CR2 = 45`,
  `CCR = 225`, `TRISE = 46`, PWM `PSC = 89` on TIM3/TIM4/TIM12,
  `TIM7->PSC = 1799`, the USART `BRR` values — assumes the 180 MHz clock tree
  that `ba3acde` made correct. `tools/hil/hil_test.py` checks the clock rate,
  boot, the link, the parser and the failsafe on a bare Nucleo. Out of its reach:
  PWM waveforms (needs a scope), USART1 on PA9/PA10 (needs a USB-serial adapter
  or a Pi), and SD, Bar30 and OLED (need the parts).
- **The emulated register-level harnesses are not in the repository.**
  Protocol framing (byte-compared against pico-protocol), PWM register setup,
  pin ownership after boot, I2C timeouts, the ring buffer, the USART2 link and
  SD block packing were run on an emulated Cortex-M4 against simulated
  peripheral registers during development. The HIL script and its host
  simulator are committed; these harnesses are not.
- **The HIL simulator mirrors `main.c` by hand.** `tools/hil/sim/nucleo_sim.c`
  compiles the real protocol and control-loop sources, but its main loop is a
  copy of `main.c`'s and has to be kept in step with it.
- **Warning counts are from arm-none-eabi-gcc 13.2.1,** not the pinned
  11.3.1 toolchain.
- **Main-loop iteration time unmeasured.** The IWDG kick interval is designed
  against a 341 ms fast-extreme timeout, but worst-case loop period has never
  been instrumented. Known contributors: one blocking SD block write about four
  times a second (`sd_write_block()` can poll busy up to 100,000 times), and the
  blocking 62-byte telemetry send, about 5.4 ms every tick.
  *Experiment:* enable DWT CYCCNT (PM0214), sample it at the top of each
  main-loop iteration, accumulate min/max/mean over 1000 iterations, convert
  at 180 MHz. PB10 gives the TIM7 tick on a scope for the ISR side.
- **Peak stack usage unmeasured.** No stack painting. The linker reserves
  1,024 bytes of stack; the actual high-water mark is unknown.

## Configuration risks

- **`PWR_CR.VOS` is relied on at its reset value (Scale 1)** rather than
  written explicitly. If any path leaves VOS at Scale 2 — bootloader, warm
  reset, debugger reload — the `ODRDY` poll becomes a silent infinite hang.
- **CubeIDE Release configuration has never been set up.** It has only the
  stock include paths and still defines `USE_HAL_DRIVER`.
- **ESC pins float from reset until PWM init.** PWM init runs right after
  `systick_init()`, but the pins are floating inputs through
  `system_clock_init()`. External pull-downs on the eight ESC signal lines
  would close that window.
- **The default build puts the Pi link on USART2** (ST-LINK USB) for bench
  testing. The vehicle needs `-DLINK_PORT_STLINK=0`; forgetting it leaves the
  Pi's UART silent.

## Open defects

Link to the Pi:

- **PID frames are ignored.** `TYPE_PID` is recognised and skipped; gains are
  compile-time constants.
- **Telemetry `raw_depth_m` echoes the commanded depth,** not the Bar30,
  because `bar30_read()` is never called.
- **No protocol version byte.** A payload layout change that keeps `TYPE` and
  `LEN` passes the CRC and decodes to wrong values.
- **USART overruns are not counted.** A byte lost to an overrun (ORE) is not in
  `rxdrop`; it only shows up as a frame failing its CRC. At 115200 baud the
  receive interrupt has about 87 µs per byte, and TIM7 (priority 0) can hold it
  off for the length of `control_loop_tick()`. That hold-off time hasn't been
  measured.

I2C:

- **`i2c_read()` uses one sequence for every length.** RM0390 gives different
  ACK/POS/BTF handling for 1-, 2- and 3+-byte master receives. The Bar30 PROM
  reads are 2 bytes. This has not been exercised against a real sensor.
- **`bar30_read()` ignores transfer errors.** `bar30_init()` now checks them,
  but `bar30_read()`, which is never called yet, would compute a depth from a
  failed read.

SPI and SD card:

- **SPI status polls have no timeout.** `spi_transmit()`/`spi_transfer()` wait
  on TXE, RXNE and BSY. As bus master these complete whether or not a slave is
  present, so a missing card can't hang them, but a misconfigured SPI peripheral
  would.
- **SDHC/SDXC only.** `sd_write_block()` sends block addresses; byte
  addressing for SDSC cards is commented out, not implemented.
- **`LogRecord.crc16` holds only the low 16 bits** of the 32-bit hardware CRC.
- **Up to 11 log records (220 ms) live only in RAM** and are lost on power
  loss — the cost of packing 12 records per block.

Same bug class as the fixed AFR writes, currently harmless:

- `SPI1->CR1 |= (3U << 3)` ORs into the BR field without clearing it. The
  field is zero from reset and written once, so the result is correct today.

Dead code:

- **ADC1:** `adc_init()` still runs and puts PA0 in analog mode; `adc_read()`
  has no callers.
- **DAC1:** `Drivers/dac/` has no callers. Removing the folder also needs its
  include path removed from `.cproject`.
- **`micros()`** (TIM2) and **`uart1_write_byte()`** have no callers.
- **`Protocol/struct.c`** is a zero-byte file that is compiled and linked.
- **`B_forward`** is defined and never referenced.

Cosmetic:

- `struct.h` comments are stale: `TelemetryPayload` says `reserved` is 1 byte
  (it is 5) and gives the CMD Python format string as `'<12f3B5s'` (the
  struct is `'<12f2B6s'`).
- `main.c` step comments run 2, 2a, 3 … 6, 8: step 7 (DAC) was removed and
  PWM init was moved to 2a without renumbering.
- Six `-O2` warnings: `B_forward` unused, `TEMP` set-but-unused in
  `bar30.c`, `write_data` unused in `oled.c`, `r7` set-but-unused in
  `sd_card.c`, and two `-Waddress-of-packed-member` in `main.c`.

## Control loop

- **All PID gains are zero.** `kp`, `ki`, `kd`, `kff` and `FF_OFFSET` are
  all-zero arrays; every axis outputs zero and every thruster sits at neutral.
  This is scaffolding awaiting tuning.
