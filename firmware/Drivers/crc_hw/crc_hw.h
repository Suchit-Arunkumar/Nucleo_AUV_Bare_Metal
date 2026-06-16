#ifndef CRC_HW_H
#define CRC_HW_H

#include "stm32f446xx.h"
#include <stdint.h>

void crc_init(void);

uint32_t crc_compute(const uint8_t *data, uint32_t length);


#endif
