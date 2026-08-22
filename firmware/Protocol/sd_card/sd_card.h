#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>

typedef enum { SD_OK = 0, SD_FAIL = 1 } SD_Status;

SD_Status sd_init(void);

SD_Status sd_read_block(uint32_t block_addr, uint8_t *buf);

SD_Status sd_write_block(uint32_t block_addr, const uint8_t *data);

#endif
