# Nucleo AUV Bare-Metal Firmware

Register-level firmware for an AUV control board, running on an STM32F446RE (Nucleo-64) with zero HAL and zero RTOS. Every peripheral driver is written directly against CMSIS headers.

## Overview

This firmware handles sensor acquisition, closed-loop attitude/depth control, thruster PWM output, telemetry, and onboard data logging for an underwater vehicle, entirely from scratch at the register level. STM32CubeIDE is used only as an editor, flasher, and debugger — no code generation, no HAL, no LL libraries.

- **MCU:** STM32F446RE @ 180MHz (HSE bypass + PLL, M=8 N=360 P=2)
- **APB1:** 45MHz &nbsp;|&nbsp; **APB2:** 90MHz
- **Toolchain:** arm-none-eabi-gcc 11.3.rel1 (CubeIDE 1.14.0)
- **Flash footprint:** ~15.5KB

## Features

- 50Hz ISR-driven control loop (TIM7) running PID + thrust allocation + failsafe
- Full peripheral driver set written against raw registers: GPIO, UART (polled + DMA/IDLE), I2C, SPI, ADC, DAC, timers (timebase/PWM/basic), CRC, IWDG
- DMA-driven UART RX with IDLE-line detection and a circular ring buffer, avoiding per-byte interrupts
- Custom binary packet protocol (0xAA/0x55 framing) with hardware CRC32 validation for telemetry and command exchange with a companion computer
- PWM motor/ESC control at 50Hz across multiple timer channels
- Bar30 (MS5837) depth sensor driver over I2C with full pressure/temperature compensation
- SSD1306 OLED status display over SPI
- SD card data logging over SPI for post-mission analysis
- Hardware watchdog (IWDG) kicked from the control loop ISR, with flash-backed PID gain persistence
- Fixed, dependency-ordered peripheral init sequence (debug UART first, control-loop timer last)

## Hardware & Toolchain

| Item | Detail |
|---|---|
| Board | Nucleo-F446RE |
| Toolchain | arm-none-eabi-gcc 11.3.rel1 (CubeIDE 1.14.0) |
| Clock | 180MHz via HSE bypass + PLL (M=8, N=360, P=2) |
| APB1 | 45MHz |
| APB2 | 90MHz |

## Peripheral Map

| Peripheral | Pins | Role |
|---|---|---|
| USART2 | PA2/PA3 | Debug printf via ST-Link VCP — initialized first always |
| USART1 | PA9/PA10 | Companion-computer packet protocol, DMA RX + IDLE ISR |
| SPI1 | PA5/PA6/PA7 | SSD1306 OLED + SD card, two independent CS lines |
| I2C1 | PB8/PB9 | Bar30 MS5837 depth sensor, AF4, open-drain, 100kHz |
| ADC1 | PA0 | Analog input, 12-bit single conversion |
| DAC1 | PA4 | Signal generation / loopback test |
| TIM2 | — | 32-bit microsecond timebase |
| TIM3/TIM4 | — | 50Hz PWM output for ESC control |
| TIM7 | — | 50Hz basic timer ISR — control loop trigger, init last |
| DMA2 Stream2 Ch4 | — | USART1 RX circular buffer |
| Hardware CRC | — | CRC32 packet validation |
| IWDG | — | Watchdog, kicked from TIM7 ISR |
| Flash | — | PID gain persistence |

## Packet Protocol

| Field | Value |
|---|---|
| Header | 0xAA 0x55 |
| Length | 1 byte |
| MSG ID 0x01 | Telemetry: STM32 → companion computer |
| MSG ID 0x02 | Command: companion computer → STM32 |
| CRC | Hardware CRC32 unit (uint32_t) |
| Ring buffer API | `rx_avail()`, `rx_peek()`, `rx_eat()`, `rx_write()` |

## Peripheral Init Order

Fixed and dependency-ordered — debug UART must come first so later init failures are visible, and the control-loop timer must come last since its ISR depends on every other peripheral being ready.

```
system_clock_init() -> systick_init() -> gpio_init()
  -> uart2_init()   // debug printf first
  -> uart1_init()   // DMA + IDLE ISR
  -> i2c1_init() -> bar30_init()
  -> spi_init() -> ssd1306_init() -> sd_card_init()
  -> adc_init() -> dac_init()
  -> tim2_init()    // microsecond timebase before PWM
  -> tim3_init()    // PWM
  -> crc_init() -> iwdg_init()
  -> tim7_init()    // control-loop ISR, always last
```

## File Structure

```
firmware/
├── Core/
│   ├── Inc/    main.h, system_init.h, control_loop.h
│   └── Src/    main.c, system_init.c, stm32f4xx_it.c, control_loop.c
├── Drivers/
│   ├── gpio/
│   ├── uart/       uart.c/.h, uart_packet.c/.h
│   ├── i2c/
│   ├── spi/
│   ├── adc/
│   ├── dac/
│   ├── timer/      timer_timebase, timer_pwm, timer_basic
│   ├── dma/
│   ├── crc_hw/
│   ├── flash/
│   └── iwdg/
├── Devices/
│   ├── bar30/      depth sensor driver
│   ├── ssd1306/    OLED driver
│   └── sd_card/    sd_card.c/.h, sd_logger.c/.h
└── Protocol/       ring_buffer.c/.h, packet.c/.h
```

## Building

Import into STM32CubeIDE as an existing project. Any new folder under `Devices/` or `Drivers/` must have build inclusion checked: right-click the folder in Project Explorer → Resource Configurations → Exclude from Build → leave **unchecked**, or the linker won't see it.

Include paths (Properties → C/C++ Build → Settings → MCU GCC Compiler → Include paths):

```
../Core/Inc ../Protocol ../Drivers/gpio
../Drivers/uart ../Drivers/i2c ../Drivers/spi
../Drivers/adc ../Drivers/dac ../Drivers/timer
../Drivers/dma ../Drivers/crc_hw ../Drivers/flash
../Drivers/iwdg ../Devices/bar30 ../Devices/ssd1306
../Devices/sd_card
```

## Notes

- No HAL, no LL drivers — `USE_HAL_DRIVER` is removed from preprocessor defines and `HAL_IncTick()` is removed from `SysTick_Handler`.
- All flag-polling loops (I2C, SPI, UART) are direct register polls; production hardening would add timeout counters to prevent a stuck bus hanging the MCU indefinitely.
- I2C is configured open-drain per the spec, since push-pull would cause bus contention with the depth sensor.
- PID gains run hardcoded on the MCU — no runtime tuning message in the current packet protocol.
