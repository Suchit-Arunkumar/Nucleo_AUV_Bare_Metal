#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void    spi1_init(void);
void    spi_transmit(uint8_t data);
uint8_t spi_receive(void);
uint8_t spi_transfer(uint8_t data);
void    spi_select_oled(void);
void    spi_deselect_oled(void);
void    spi_select_sd(void);
void    spi_deselect_sd(void);

#endif
