# Hardware-in-the-loop test report

- Date: 2026-10-04 23:36
- Board: NUCLEO-F446RE, link over ST-LINK USB (COM12); optional MPU-6050 on I2C1 and a PA15 jumper
- Firmware commit: `1c32d43`
- Host: Windows 11, Python 3.14.3
- Board -> laptop CRC errors, whole run: 1

| Test | Result | What it shows |
|---|---|---|
| Boot on a bare board | **PASS** | clock, console, and every init step through the watchdog complete with no sensors attached |
| Console heartbeat and clock rate | **PASS** | main loop alive; SysTick runs at 1 kHz, i.e. the 180 MHz clock tree is right; no resets |
| Telemetry at 50 Hz, link idle | **PASS** | TIM7 at 50 Hz; frames framed and CRC'd correctly by the board; idle state is safe |
| Link up and data round trip | **PASS** | board receives, parses and acts on CMD frames; data survives laptop -> board -> laptop exactly |
| Arm flag round trip | **PASS** | the armed bit in CMD reaches the control loop and comes back |
| Resync through junk and split frames | **PASS** | the board's parser recovers from garbage, decoy sync bytes and frames split across writes |
| Corrupted frames rejected | **PASS** | the board's CRC check rejects damaged frames: nothing from them is used, and the link times out |
| Command-timeout failsafe | **PASS** | if the Pi goes silent, the board disarms and drops the link within the 500 ms timeout |
| Soak, 60 s at 50 Hz both ways | **PASS** | no dropped bytes, CRC errors, link drops or resets under sustained traffic |
| Timing on silicon: tick, ISR, main loop | **PASS** | TIM7 period and jitter, control ISR cost, worst main-loop pass vs the watchdog, UART baud from TX time |
| Stack high-water mark | **PASS** | painted MSP: deepest use since boot, across every test so far |
| I2C driver against a real slave (MPU-6050) | **FAIL** | 1-, 2-, 6- and 14-byte register reads, checked for plausibility, every 500 ms since boot |
| Eight PWM outputs, measured by the chip | **PASS** | each ESC pin carries its own channel, 1 us resolution, 20 ms frame (TIM2_CH1 capture on PA15) |
| Watchdog catches a hung main loop | **PASS** | with TIM7 still running the control loop, a stalled main loop is reset inside the LSI window |

## Boot on a bare board: PASS

- PASS: 'BOOT OK' received (console readable: clock tree and USART2 baud correct)
- info: console: BOOT OK
- info: console: RESET: PIN
- info: console: SD FAIL
- info: console: BAR30 FAIL
- info: console: MPU 0x68
- info: console: LINK USART2 ST-LINK VCP
- info: console: TIMERS OK
- info: console: BOOT DONE
- PASS: 'BOOT DONE' received: boot ran through iwdg_init() without hanging
- info: BOOT OK -> BOOT DONE: 62 ms (host-measured)
- PASS: SD init reported a result instead of hanging
- PASS: Bar30 init reported a result instead of hanging
- PASS: link running on USART2 (ST-LINK VCP)
- PASS: reset cause reported: 'RESET: PIN'

## Console heartbeat and clock rate: PASS

- PASS: 20 status lines in 10 s (expect one per 500 ms)
- PASS: g_tick strictly increasing (no watchdog or other reset)
- PASS: line spacing 500-500 ms of board time (expect 500-501)
- PASS: board clock vs laptop clock: 9500 ms vs 9495 ms (+0.05 %, limit +/-1 %)
- info: ratio carries a few ms of USB timestamp jitter; a wrong PLL setting shows up as tens of percent

## Telemetry at 50 Hz, link idle: PASS

- PASS: 250 frames in 5 s = 50.00 Hz (expect 50)
- PASS: 0 frames with a bad CRC
- PASS: link_ok = 0 with no commands
- PASS: armed = 0 with no commands
- PASS: all 8 ESC values at 1500 us

## Link up and data round trip: PASS

- PASS: telemetry shows link_ok = 1 after commands start
- info: first CMD sent -> first link_ok=1 telemetry: 46 ms
- PASS: 150/150 telemetry depth values are exact float32 copies of a CMD value we sent
- PASS: 0 depth values that were never sent
- PASS: link_ok stays 1 while commands flow at 50 Hz
- info: round trip (CMD sent -> its value back in telemetry): median 39 ms, p95 53 ms, max 56 ms
- PASS: console status line reports link=1

## Arm flag round trip: PASS

- PASS: armed = 1 in all 50 frames while commanded
- PASS: ESC values stay at 1500 us (all PID gains are zero, so zero thrust is the correct output)
- PASS: armed = 0 again once commanded off

## Resync through junk and split frames: PASS

- PASS: link_ok stayed 1 across 144 frames
- PASS: 144/144 depth values still track the commands
- PASS: 0 depth values that were never sent

## Corrupted frames rejected: PASS

- PASS: no value from a bad-CRC frame ever appears in telemetry
- PASS: link_ok drops to 0 while only bad frames arrive
- PASS: dropped 544 ms after the last good frame (timeout is 500 ms)

## Command-timeout failsafe: PASS

- PASS: run 1: link_ok went to 0 after commands stopped
- PASS: run 1: armed = 0 in the same frame
- PASS: run 2: link_ok went to 0 after commands stopped
- PASS: run 2: armed = 0 in the same frame
- PASS: run 3: link_ok went to 0 after commands stopped
- PASS: run 3: armed = 0 in the same frame
- PASS: failsafe after 545, 524, 537 ms (limit 500 ms + one 20 ms tick + USB latency)

## Soak, 60 s at 50 Hz both ways: PASS

- PASS: 3001 telemetry frames = 50.02 Hz
- PASS: 0 CRC errors board -> laptop
- PASS: link never dropped
- PASS: 3001/3001 depth values echo sent commands
- PASS: status lines kept arriving
- PASS: no reset during the soak
- PASS: board rx ring buffer drops: 0 (unchanged throughout)

## Timing on silicon: tick, ISR, main loop: PASS

- PASS: perf line received (BENCH_HIL build)
- PASS: TIM7 period 19999.87..20000.13 us since boot (expect 20000; worst 0.13 us off)
- PASS: TIM7 ISR incl. control_loop_tick(): avg 8.4 us, max 45.8 us (0.23 % of the tick)
- PASS: main loop: avg 4 us, worst pass 13.43 ms; the watchdog's shortest timeout is 341 ms (25x margin)
- PASS: 62-byte telemetry send blocks 5.292 ms (61 byte times at 115200 = 5.30 ms): the USART2 baud rate, measured

## Stack high-water mark: PASS

- PASS: stack line received
- PASS: peak 852 B of the 1024 B the linker reserves (83 %)

## I2C driver against a real slave (MPU-6050): FAIL

- PASS: 187 read cycles (4 transfers each) since boot, WHO_AM_I 0x68
- FAIL: 374 implausible values (WHO_AM_I changed, temperature or |accel| out of range)
- PASS: 0 bus errors (NACK or timeout)
- PASS: 0 stale bytes left in DR by a previous read
- PASS: slowest 14-byte burst 1572 us (100 kHz floor ~1.7 ms: CCR = 225 is right)

## Eight PWM outputs, measured by the chip: PASS

- PASS: T1 PC6: capture line
- PASS: T1 PC6: high 1100..1100 us over 25 pulses (expect 1100: this channel, no other)
- PASS: T1 PC6: period 20000..20000 us
- PASS: T2 PB5: capture line
- PASS: T2 PB5: high 1200..1200 us over 25 pulses (expect 1200: this channel, no other)
- PASS: T2 PB5: period 20000..20000 us
- PASS: T3 PC8: capture line
- PASS: T3 PC8: high 1300..1300 us over 25 pulses (expect 1300: this channel, no other)
- PASS: T3 PC8: period 20000..20000 us
- PASS: T4 PC9: capture line
- PASS: T4 PC9: high 1400..1400 us over 25 pulses (expect 1400: this channel, no other)
- PASS: T4 PC9: period 20000..20000 us
- PASS: T5 PB6: capture line
- PASS: T5 PB6: high 1500..1500 us over 25 pulses (expect 1500: this channel, no other)
- PASS: T5 PB6: period 20000..20000 us
- PASS: T6 PB7: capture line
- PASS: T6 PB7: high 1600..1600 us over 25 pulses (expect 1600: this channel, no other)
- PASS: T6 PB7: period 20000..20000 us
- PASS: T7 PB14: capture line
- PASS: T7 PB14: high 1700..1700 us over 25 pulses (expect 1700: this channel, no other)
- PASS: T7 PB14: period 20000..20000 us
- PASS: T8 PB15: capture line
- PASS: T8 PB15: high 1800..1800 us over 25 pulses (expect 1800: this channel, no other)
- PASS: T8 PB15: period 20000..20000 us
- PASS: signature off: last pin back at neutral, 1500-1500 us

## Watchdog catches a hung main loop: PASS

- PASS: run 1: main loop stopped on command
- PASS: run 1: board reset itself
- PASS: run 1: reset cause 'RESET: IWDG PIN'
- PASS: run 2: main loop stopped on command
- PASS: run 2: board reset itself
- PASS: run 2: reset cause 'RESET: IWDG PIN'
- PASS: run 3: main loop stopped on command
- PASS: run 3: board reset itself
- PASS: run 3: reset cause 'RESET: IWDG PIN'
- PASS: stall -> reset 563, 563, 561 ms (window 341..943 ms from the LSI spec, plus a few ms of boot)
- info: implied LSI on this chip: 28.5 kHz (spec 17..47 kHz, typical 32)
