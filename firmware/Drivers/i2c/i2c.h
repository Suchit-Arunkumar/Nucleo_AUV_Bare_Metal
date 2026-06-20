
#ifndef I2C_h
#define I2C_h

#include <stdint.h>
#include "stm32f446xx.h"

// declare i2c1_init()
void i2c1_init();
// declare i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
void i2c_write(uint8_t addr, uint8_t *data, uint8_t len);
// declare i2c_read(uint8_t addr, uint8_t *buf, uint8_t len)
void i2c_read(uint8_t addr, uint8_t *buf, uint8_t len);

#endif
