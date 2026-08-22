// sd_logger.h

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "sd_card.h"


typedef struct __attribute__((packed))
{
    uint32_t timestamp_ms;   // SysTick timestamp

    float depth_m;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    uint16_t pwm[8];         // 8 thrusters

    uint8_t armed;           // 0 = disarmed, 1 = armed
    uint8_t link_ok;         // 0 = link lost, 1 = link healthy

    uint16_t crc16;          // record integrity check

} LogRecord;

// Initialize logger and reset sector counter
void sd_logger_init(void);

// Write one log record to SD card
SD_Status sd_logger_write(const LogRecord *record);

#endif
