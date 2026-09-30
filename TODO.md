# Known limitations

Open items in this repository, stated explicitly rather than left for a reader
to discover. Fixed defects are recorded in the README's engineering log and in
the commit history, not here.

## Verification gaps

- **Never run on hardware since the clock fix.** Every timing constant —
  `I2C1->CR2 = 45`, `CCR = 225`, `TRISE = 46`, PWM `PSC = 89` on
  TIM3/TIM4/TIM12, `TIM7->PSC = 1799`, USART1 `BRR` from a 90 MHz APB2 — assumes
  the 180 MHz clock tree that `ba3acde` made correct. None has been confirmed
  on silicon: no PWM waveforms scoped, no live link to the Pi, no writes to a
  real SD card.
- **Off-target harnesses are not in the repository.** Protocol framing
  (byte-compared against pico-protocol), PWM register setup, pin ownership
  after boot, and SD block packing were run on an emulated Cortex-M4 against
  simulated peripheral registers during development. Committing them with a
  script that runs under `qemu-arm` would make those results reproducible.
- **Warning counts are from arm-none-eabi-gcc 13.2.1,** not the pinned
  11.3.1 toolchain.
- **Main-loop iteration time unmeasured.** The IWDG kick interval is designed
  against a 341 ms fast-extreme timeout, but worst-case loop period has never
  been instrumented. The largest known contributor is one blocking SD block
  write, now about four times a second instead of fifty. `sd_write_block()`
  can poll the card's busy state up to 100,000 times per write.
  *Experiment:* enable DWT CYCCNT (PM0214), sample it at the top of each
  main-loop iteration, accumulate min/max/mean over 1000 iterations, convert
  at 180 MHz. CYCCNT wraps every ~23.9 s, far longer than one iteration.
  PB10 already gives the TIM7 tick on a scope for the ISR side.
- **Peak stack usage unmeasured.** No stack painting. The linker reserves
  1,024 bytes of stack; the logger no longer takes 512 of them per write, but
  the actual high-water mark is unknown.

## Configuration risks

- **`PWR_CR.VOS` is relied on at its reset value (Scale 1)** rather than
  written explicitly. If any path leaves VOS at Scale 2 — bootloader, warm
  reset, debugger reload — the `ODRDY` poll becomes a silent infinite hang.
- **CubeIDE Release configuration has never been set up.** It has only the
  stock include paths and still defines `USE_HAL_DRIVER`.
- **ESC pins float from reset until PWM init.** PWM init now runs right after
  `systick_init()`, but the pins are floating inputs through
  `system_clock_init()`. External pull-downs on the eight ESC signal lines
  would close that window.

## Open defects

Link to the Pi:

- **`telem_pending` is never set to 1.** Telemetry frames are built and framed
  correctly (62 bytes, CRC-16, matching pico-protocol) but the branch in
  `main.c` that sends them is unreachable. Setting it once per TIM7 tick would
  send at 50 Hz: 62 bytes at 115200 baud is about 5.4 ms of the 20 ms frame,
  and `uart1_write_buf()` busy-waits for all of it.
- **PID frames are ignored.** `TYPE_PID` is recognised and skipped; gains are
  compile-time constants.
- **Telemetry `raw_depth_m` echoes the commanded depth,** not the Bar30,
  because `bar30_read()` is never called.
- **No protocol version byte.** A payload layout change that keeps `TYPE` and
  `LEN` passes the CRC and decodes to wrong values.
- **Ring buffer is unsynchronised.** `rx_write()` runs in `USART1_IRQHandler`,
  `rx_eat()` in the main loop; `rx_head` / `rx_tail` / `rx_count` are
  non-`volatile` and unguarded. `rx_count` can drift when the interrupt lands
  inside `rx_eat()`'s read-modify-write, and `rx_write()` never checks for
  free space, so a burst over 256 bytes overwrites unread data.

Blocking and timeouts:

- **I2C and SPI poll status flags with no timeout.** An unresponsive device
  blocks boot indefinitely; `bar30_init()` and `sd_init()` are both on this
  path.

SD card:

- **SDHC/SDXC only.** `sd_write_block()` sends block addresses; byte
  addressing for SDSC cards is commented out, not implemented.
- **`LogRecord.crc16` holds only the low 16 bits** of the 32-bit hardware CRC.
- **Up to 11 log records (220 ms) live only in RAM** and are lost on power
  loss — the cost of packing 12 records per block.

Same bug class as the fixed AFR writes, currently harmless:

- `SPI1->CR1 |= (3U << 3)` (BR field) and `I2C1->CR2 |= 45` (FREQ field) OR
  into multi-bit fields without clearing them. Both fields are zero from
  reset and written once, so the result is correct today.

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
