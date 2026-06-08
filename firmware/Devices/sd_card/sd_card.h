#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>

typedef enum { SD_OK = 0, SD_FAIL = 1 } SD_Status;

SD_Status sd_init(void);

#endif
