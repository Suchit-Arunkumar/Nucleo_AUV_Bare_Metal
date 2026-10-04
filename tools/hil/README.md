# Hardware-in-the-loop test

`hil_test.py` runs the firmware on a real NUCLEO-F446RE. A laptop stands in for
the Raspberry Pi: it talks to the board over the same USB cable used for
flashing, sends the Pi's 62-byte command frames, and checks everything the board
sends back. With the `BENCH_HIL` build (the default) the board also measures
itself, with the DWT cycle counter and TIM2 input capture, and reports the
numbers on its console.

Last run: **14 / 14 pass**, 2026-10-04, reports in [`reports/`](reports/).

**Wiring.** Only the USB cable is required. Two optional additions unlock two
more tests; each test SKIPs without its wiring.

| Part | Connection | Test it enables |
|---|---|---|
| MPU-6050 (GY-521 / HW-123 board) | VCC → 3V3, GND → GND, SCL → D15 (PB8), SDA → D14 (PB9) | I2C driver against a real slave |
| One jumper wire | PA15 (CN7 pin 17) → each ESC pin in turn, when prompted | Eight PWM outputs |

Out of reach: the SD card, the Bar30, the OLED, and USART1 on PA9/PA10.

## How the link reaches the laptop

The firmware's `LINK_PORT_STLINK` setting (in `firmware/Drivers/uart/link.h`)
decides which UART carries the Pi protocol:

| `LINK_PORT_STLINK` | Link UART | Use |
|---|---|---|
| `1` (default) | USART2: the ST-LINK virtual COM port on the USB cable | Bench testing with this script |
| `0` | USART1 on PA9/PA10, DMA receive | The vehicle, wired to the Pi |

In the default build, the console (`printf`) and the protocol share the USB port.
The firmware writes text lines and frames whole from its main loop, so they never
interleave. The script separates them the same way the firmware's parser does:
anything that isn't a valid frame is treated as text.

## Running it

1. Flash the firmware as usual from STM32CubeIDE. The default build has
   `LINK_PORT_STLINK = 1`.
2. Close anything holding the board's serial port, such as CubeIDE's serial
   console or a terminal program. If you flashed through a debug session, either
   end it or press Resume, so the core isn't sitting halted at `main()`.
3. Run:
   ```
   pip install pyserial
   python tools/hil/hil_test.py
   ```
   The script finds the ST-LINK port by its USB vendor ID. If it can't, pass
   `--port COM5` (Windows) or `--port /dev/ttyACM0` (Linux / macOS).
4. When prompted, press the black **RESET** button (B2) so the script can watch
   the board boot. Use `--no-reset` to skip that test.

5. When prompted, move the PA15 jumper to each of the eight ESC pins and press
   Enter (`s` skips a pin; `--no-pwm` skips the test). The watchdog test then
   resets the board three times on its own (`--no-iwdg` skips it).

A run takes about 6 minutes, including a 60 s soak (`--soak N` to change it,
`--soak 0` to skip). If boot never reaches the main loop, the script stops after
the first test and prints the last console lines instead of failing every test
for the same reason. It exits with 0 only if every test passes, and writes
`tools/hil/hil_report.md`; reports from runs worth keeping go in `reports/`,
named by date and firmware commit.

## What each test shows

| Test | Pass means |
|---|---|
| Boot on a bare board | Console text is readable, so the 180 MHz clock tree and UART baud are right. `BOOT DONE` arrives, so the I2C and SD timeouts let boot finish with no sensors and the watchdog gets armed |
| Console heartbeat and clock rate | The main loop is alive. The board's millisecond counter matches the laptop's clock to within 1 %, so SysTick really is 1 kHz. No resets |
| Telemetry at 50 Hz, link idle | TIM7 runs at 50 Hz, every frame the board builds has a valid CRC, and with no commands it reports disarmed, link down, 1500 µs on all ESCs |
| Link up and data round trip | A random float sent in a command comes back bit-exact in telemetry, and only after it was sent. This measures round-trip latency |
| Arm flag round trip | The armed bit reaches the control loop and comes back. ESC outputs stay at 1500 µs, which is correct with all gains at zero |
| Resync through junk and split frames | The board's parser recovers from garbage bytes, decoy `0xAA 0x55` pairs and frames split across writes, with no data errors |
| Corrupted frames rejected | Frames with a damaged CRC are never used, and the link times out as if they weren't there |
| Command-timeout failsafe | When commands stop, the board disarms and drops the link roughly 500 ms later, measured over three runs |
| Soak | Sustained 50 Hz traffic both ways with no CRC errors, no ring-buffer drops (the board's `rxdrop` counter), no link drops and no resets |
| Timing | TIM7 period and jitter, control ISR duration, worst main-loop pass against the watchdog's 341 ms floor, and the USART2 baud rate from the time one frame takes to send, all from the DWT cycle counter |
| Stack | The painted stack's high-water mark is below the linker's reservation |
| I2C vs MPU-6050 | `PWR_MGMT_1` reads back 0x00 after the wake write; 1-, 2-, 6- and 14-byte reads every 500 ms with no implausible values, bus errors or stale bytes; 14-byte read time matches 100 kHz |
| Eight PWM outputs | With channel *k* driven at 1,100 + 100*k* µs, each pin shows its own width and a 20,000 µs period, measured by TIM2_CH1 capture on PA15 |
| Watchdog | `BENCH:HANG` stops the main loop with TIM7 still running; the board resets inside the LSI window (341–943 ms) and reports `IWDG` as the cause, three times |

## Developing without a board: the simulator

`sim/` builds a stand-in for the Nucleo on Linux, macOS or WSL. It compiles the
firmware's **own** `packet.c`, `ring_buffer.c` and `control_loop.c` for the PC,
and serves them on a pseudo-terminal:

```
make -C tools/hil/sim
./tools/hil/sim/nucleo_sim                  # prints e.g. "PTY /dev/pts/3" and its PID
python tools/hil/hil_test.py --port /dev/pts/3
kill -USR1 <pid>                            # the simulated RESET button, when prompted
```

The main loop in `nucleo_sim.c` mirrors `firmware/Core/Src/main.c` and has to be
kept in step with it by hand. The simulator doesn't model UART timing, USB latency
or any hardware, so passing against it shows that the script and the firmware's
protocol logic agree. Only a run on the board shows the board works.

**How the script itself was checked.** It passes all 9 tests against the
simulator. It was also run against three deliberately broken builds of the
simulator:

- telemetry never sent
- the failsafe timeout changed to 1500 ms
- echoed data corrupted

It failed each one, in the tests aimed at that bug.
