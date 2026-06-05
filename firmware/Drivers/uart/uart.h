#ifndef UART_UART_H_
#define UART_UART_H_

#include <stdint.h>

void uart2_init(void);

void uart2_write_byte(uint8_t b);

void uart2_write_buf(uint8_t *buf, uint16_t len);

void uart2_write_str(const char *s);

#endif
