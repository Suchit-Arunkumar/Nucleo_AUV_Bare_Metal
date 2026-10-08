#ifndef LINK_H
#define LINK_H

#include <stdint.h>

// Which UART carries the Pi link (62-byte frames, see packet.h).
//
//   LINK_PORT_STLINK = 1   USART2: the Nucleo's ST-LINK USB virtual COM port.
//                          Bench testing with nothing but a laptop and the USB
//                          cable (tools/hil/hil_test.py). The printf console
//                          shares the port: text lines and frames are both
//                          written whole from the main loop, never interleaved,
//                          and the host parser skips text between frames.
//
//   LINK_PORT_STLINK = 0   USART1 on PA9/PA10 with DMA receive: the wiring to
//                          the Raspberry Pi on the vehicle.
//
// Defaults to 0, the vehicle. Build with -DLINK_PORT_STLINK=1 for the bench.
#ifndef LINK_PORT_STLINK
#define LINK_PORT_STLINK 0
#endif

// Start receiving on the link UART. With LINK_PORT_STLINK, uart2_init() must
// already have run (it does, as the console).
void link_init(void);

// Blocking send of one frame.
void link_send(const uint8_t *buf, uint16_t len);

// For the boot log.
const char *link_port_name(void);

#endif
