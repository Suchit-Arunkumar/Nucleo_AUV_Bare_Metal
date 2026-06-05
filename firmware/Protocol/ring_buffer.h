#include <stdint.h>

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define RX_BUF_SIZE 256

void rx_write(uint8_t *data, uint16_t len);
uint16_t rx_avail(void);
uint8_t rx_peek(uint16_t offset);
void rx_eat(uint16_t len);

#endif
