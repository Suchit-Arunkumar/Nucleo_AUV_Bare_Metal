#include <stdint.h>

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define RX_BUF_SIZE 256

// Called from UART RX interrupt context only. Bytes that arrive while the
// buffer is full are dropped and counted.
void rx_write(uint8_t *data, uint16_t len);

// Called from the main loop only.
uint16_t rx_avail(void);
uint8_t  rx_peek(uint16_t offset);
void     rx_eat(uint16_t len);

// Total bytes dropped because the buffer was full, since boot.
uint32_t rx_dropped_count(void);

// Called from UART interrupt context when the UART reports an overrun (ORE):
// a byte was lost in the peripheral before it reached the buffer, so it is
// not in rx_dropped_count(). Total since boot.
void     rx_note_overrun(void);
uint32_t rx_overrun_count(void);

#endif
