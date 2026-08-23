# Known limitations

Open items in this repository, stated explicitly rather than left for a reader
to discover.

## Verification gaps

- **No host-side unit tests.** Nothing in this repository is testable off
  target. The packet framing, CRC, ring buffer, and thrust allocation are all
  pure functions that could be compiled and tested natively; none are.
- **Never verified on hardware since the clock fix.** The firmware previously
  ran at ~81 MHz due to a PLLCFGR read-modify-write bug. Every downstream
  timing constant — `I2C1->CR2 = 45`, `CCR = 225`, `TRISE = 46`,
  `TIM3->PSC = 89`, `TIM7->PSC = 1799` — was written assuming a 45 MHz APB1
  that only became true after the fix. They should now be correct. This is
  unconfirmed on silicon.
- **CRC-32 between this board and the Pico firmware is almost certainly
  incompatible, not merely unverified.** The STM32 hardware CRC peripheral
  (`crc_hw.c`) is fixed to the Ethernet polynomial with MSB-first input,
  init `0xFFFFFFFF`, no input/output reflection, and no final XOR. The
  common software CRC-32 (zlib and most libraries) reflects both input and
  output and applies a final XOR of `0xFFFFFFFF`. These produce different
  results on identical bytes unless the Pico side was deliberately written
  to match the STM32 variant bit-for-bit. `crc_hw.c` also zero-pads the
  trailing 1-3 bytes of the 58-byte payload into a full 32-bit word before
  feeding the peripheral, which the software side would also need to
  replicate exactly. Given USART1 TX is disabled (see Open defects), the
  two boards have never exchanged a packet, so this has never been caught
  in practice.
- **Main-loop iteration time unmeasured.** The IWDG kick interval is designed
  against a 341 ms fast-extreme timeout, but worst-case loop period has never
  been instrumented, so the margin is asserted rather than measured.
  *Experiment:* enable DWT CYCCNT (PM0214), sample the counter at the top of
  each main-loop iteration, accumulate min/max/mean over 1000 iterations,
  convert to ms at 180 MHz. CYCCNT is 32-bit and wraps every ~23.9 s at
  180 MHz, which is far longer than one iteration, so wrap handling is not
  needed for per-iteration deltas.
- **Peak stack usage unmeasured.** No stack painting. The linker reserves
  1,536 bytes of heap+stack; actual high-water mark is unknown.

## Configuration risks

- **`PWR_CR.VOS` is relied upon at its reset value (Scale 1)** rather than
  written explicitly. If any path leaves VOS at Scale 2 — bootloader, warm
  reset, debugger reload — the `ODRDY` poll becomes a silent infinite hang
  with no diagnostic.
- **CubeIDE Release configuration has never been set up.** It carries only the
  five stock include paths, none of the driver paths, and still defines
  `USE_HAL_DRIVER`. Only the Debug config and the pinned command line in the
  README have ever built this project.

## Open defects

Pin conflicts:

- **PA6 ends at AF7 — neither SPI1_MISO (AF5) nor TIM3_CH1 (AF2).**
  `timer3_pwm_init()` ORs AF2 into a field already holding AF5 without
  clearing it: `5 | 2 = 7`. Both peripherals lose the pin. SPI1 MISO is
  non-functional from that point, which means SD card reads return garbage.
- **PA4** is configured as DAC1 analog output and then reclaimed by SPI1 as
  the OLED chip-select. DAC1 has no output pin.
- **PA2/PA3** are reprogrammed from USART2 AF7 to GPIO output by the OLED
  reset/DC init. The debug UART dies partway through boot.
- **PA5** is toggled as a heartbeat LED via `GPIOA->ODR` while the pin is in
  AF mode for SPI1_SCK. Writing ODR on an AF pin has no effect.

Functional:

- **`pwm_set_us(uint8_t channel, uint16_t us)` ignores `channel`** and always
  writes `TIM3->CCR1`. Only one PWM channel is configured.
- **`sd_logger_init()` has zero call sites.** `current_block` stays at its
  initializer of 0, so the first log write targets block 0 — the card's MBR.
- **`LogRecord.timestamp_ms` is never assigned.** The record is an
  uninitialized stack object and the CRC is computed over the garbage value.
  `crc_compute()` returns `uint32_t` into a `uint16_t` field.
- **`USART_CR1_TE` is never set for USART1.** The transmitter is disabled;
  `uart1_write_buf()` writes into a dead DR.
- **`telem_pending` is never set to 1.** The telemetry branch in `main.c` is
  unreachable.
- **Ring buffer is unsynchronized.** `rx_write()` runs in `USART1_IRQHandler`,
  `rx_eat()` in the main loop; `rx_head` / `rx_tail` / `rx_count` are
  non-`volatile` and unguarded. `rx_head` is masked so the array is never
  indexed out of range, but `rx_count` increments unbounded past
  `RX_BUF_SIZE` and head silently laps tail. `rx_eat()` can underflow
  `rx_count`.
- **I2C and SPI poll status flags with no timeout.** An unresponsive device
  blocks boot indefinitely. `bar30_init()` and `sd_init()` are both on this
  path.

Cosmetic:

- **`Protocol/struct.c` is a zero-byte file** that is compiled and linked.
- **`B_forward` is defined and never referenced.**
- Six `-O2` warnings remain: `B_forward` unused, `TEMP` set-but-unused in
  `bar30.c`, `write_data` unused in `oled.c`, `r7` set-but-unused in
  `sd_card.c`, and two `-Waddress-of-packed-member` in `main.c`.

## Control loop

- **All PID gains are zero.** `kp`, `ki`, `kd`, `kff`, and `FF_OFFSET` are
  all-zero arrays. The control loop computes and allocates zeros; thrusters
  never leave neutral. This is scaffolding awaiting tuning.
