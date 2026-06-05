#ifndef UART_PACKET_H
#define UART_PACKET_H

#include <stdint.h>
#include "ring_buffer.h"

#define DMA_BUF_SIZE 256

void uart1_init(void);
void uart1_write_buf(uint8_t *buf, uint16_t len);

#endif
