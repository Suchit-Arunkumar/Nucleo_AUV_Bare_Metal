
#ifndef I2C_h
#define I2C_h

#include <stdint.h>
#include "stm32f446xx.h"

void i2c1_init(void);

// Both return 0 on success, -1 if the slave NACKs or any step times out
// (I2C_TIMEOUT_MS per flag). On failure the transfer is ended with STOP and
// the peripheral is reset if the bus is still busy.
int i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
int i2c_read(uint8_t addr, uint8_t *buf, uint8_t len);

#endif
