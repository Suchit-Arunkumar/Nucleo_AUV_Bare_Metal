#ifndef BAR30_H
#define BAR30_H

#include <stm32f446xx.h>
#include <stdint.h>


// Reset the sensor and read its calibration PROM. Returns 0 on success, -1 if
// any I2C transfer fails (sensor absent or not responding).
int bar30_init(void);

// declare bar30_read()  -- returns float (depth in meters)
float bar30_read(void);

#endif
