# Known limitations

Open items in this repository, stated explicitly rather than left for a reader
to discover. Fixed defects are recorded in the README's engineering log and in
the commit history, not here.

## Verification gaps

Run on hardware 2026-10-04 (14 / 14) and again on 2026-10-08 after the review
fixes (12 / 12, PWM and I2C skipped); reports in `tools/hil/reports/`. What
those runs could not reach:

- **USART1 with DMA on PA9/PA10**, the vehicle's Pi link. The HIL link runs over
  USART2. Testing it needs a jumper from PA9 to PA10 and a loopback mode, as the
  FreeRTOS tree has, or a USB-serial adapter.
- **SD card, OLED and Bar30**: not fitted. SD block packing is verified only
  against a simulated card.
- **Anything with ESCs or thrusters attached.**
- **The emulated register-level harnesses are not in the repository.** Protocol
  framing, PWM register setup, pin ownership, I2C timeouts, the ring buffer and
  SD block packing were checked on an emulated Cortex-M4 during development.
  The HIL script and its host simulator are committed; these harnesses are not.
- **The HIL simulator mirrors `main.c` by hand.** `tools/hil/sim/nucleo_sim.c`
  compiles the real protocol and control-loop sources, but its main loop is a
  copy of `main.c`'s and has to be kept in step with it.
- **Warning counts and sizes:** README figures are from arm-none-eabi-gcc
  13.2.1; the 2026-10-08 HIL build used the pinned 11.3.1 (text 17748 /
  data 100 / bss 5568, same three warnings).
- **The overrun counter is not checked by the HIL suite.** `hil_test.py` ignores
  the `ore=` status field, and no test provokes an overrun, so the counter is
  built and printed but not exercised.

Measured, no longer open: worst main-loop pass 17.77 ms (2026-10-04,
GCC 13.2.1, MPU test included) and 8.60 ms (2026-10-08, GCC 11.3.1, no
MPU-6050 fitted); peak stack 872 B (of the then 1,024 B reservation) and
736 B (of 2,048 B since 4aecf75). All far inside the 341 ms watchdog floor.

## Configuration risks

- **The over-drive polls (`ODRDY`, `ODSWRDY`) have no timeout.** VOS is now
  written explicitly, which removes the known way they could hang, but a
  hardware fault there would still stop boot with no message.
- **CubeIDE Release configuration has never been set up.** It has only the
  stock include paths and still defines `USE_HAL_DRIVER`.
- **ESC pins float from reset until PWM init.** PWM init runs right after
  `systick_init()`, but the pins are floating inputs through
  `system_clock_init()`. External pull-downs on the eight ESC signal lines
  would close that window.
- **CubeIDE has no separate Bench configuration.** The default build is now
  the vehicle build; the bench build needs `BENCH_HIL=1` and
  `LINK_PORT_STLINK=1` added by hand. A dedicated build configuration would make
  switching less error-prone.
- **Blocking console output sets the worst main-loop pass.** 17.77 ms measured,
  mostly the 62-byte telemetry frame (5.29 ms) plus status lines at 115,200
  baud in the same pass. Harmless against the watchdog; a DMA TX path would
  remove it.

## Open defects

Link to the Pi:

- **PID frames are ignored.** `TYPE_PID` is recognised and skipped; gains are
  compile-time constants.
- **Telemetry `raw_depth_m` echoes the commanded depth,** not the Bar30,
  because `bar30_read()` is never called.
- **The CMD version byte is not checked.** Telemetry carries
  `PROTOCOL_VERSION` (1), and CMD has the same byte, but the parser accepts any
  value, as pico-protocol does. Whether to drop or log a mismatch is a vehicle
  decision.
- **USART overruns are counted but not yet observed on hardware.** The status
  line's `ore=` field counts ORE events on either link UART (USART1 notices
  them at the next IDLE, since EIE is off). The worst-case TIM7 hold-off is
  45.6 µs against about 87 µs per byte at 115200 baud, so none are expected;
  a HIL run with the new field would confirm it.

I2C:

- **`i2c_read()` uses one sequence for every length.** RM0390 gives different
  ACK/POS/BTF handling for 1-, 2- and 3+-byte master receives. Against an
  MPU-6050 it completed 748 reads of 1, 2, 6 and 14 bytes with 0 errors and 0
  stale bytes, but with only the 45 µs TIM7 ISR able to interrupt it. A longer
  interruption at the wrong byte would ACK one byte too many. The FreeRTOS tree
  carries `i2c_read_rm()` with the RM0390 sequences; port it if the I2C code is
  ever called with longer-running interrupts enabled.
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

Dead code:

- **ADC1:** `adc_init()` still runs and puts PA0 in analog mode; `adc_read()`
  has no callers.
- **DAC1:** `Drivers/dac/` has no callers. Removing the folder also needs its
  include path removed from `.cproject`.
- **`uart1_write_byte()`** has no callers. (`micros()` is now used by the
  bench build's MPU timing.)

Cosmetic:

- `main.c` step comments run 2, 2a, 3 … 6, 8: step 7 (DAC) was removed and
  PWM init was moved to 2a without renumbering.
- Two `-O2` warnings in the vehicle build: `TEMP` set-but-unused in `bar30.c`
  and `r7` set-but-unused in `sd_card.c` (see README, Build). The bench build
  adds a deliberate `#warning` from `bench.c`.

## Control loop

- **All PID gains are zero.** `kp`, `ki`, `kd`, `kff` and `FF_OFFSET` are
  all-zero arrays; every axis outputs zero and every thruster sits at neutral.
  This is scaffolding awaiting tuning.
