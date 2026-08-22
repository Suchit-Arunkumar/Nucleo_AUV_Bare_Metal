#include "sd_logger.h"
#include "sd_card.h"

#include <string.h>

static uint32_t current_block = 0;

void sd_logger_init(void)
{
    // Skip boot sector / partition table area
    current_block = 100;
}

SD_Status sd_logger_write(const LogRecord *record)
{
    uint8_t buf[512];

    // Clear entire sector
    memset(buf, 0, sizeof(buf));

    // Copy record into start of sector
    memcpy(buf, record, sizeof(LogRecord));

    // Write sector to SD card
    SD_Status status = sd_write_block(current_block, buf);

    // Advance only if write succeeded
    if (status == SD_OK)
    {
        current_block++;
    }

    return status;
}
