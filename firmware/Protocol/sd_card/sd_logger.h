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

// -----------------------------------------------------------------------------
// On-card format, starting at SD_LOG_FIRST_BLOCK, one 512-byte block at a time:
//
//   bytes   0..479   up to 12 LogRecords (40 bytes each), oldest first
//   bytes 480..511   zero
//
// Unused record slots are all-zero. A reader should accept a slot only if its
// crc16 checks out; an all-zero slot does not. A block can be written while
// partly full (timeout flush) and later rewritten in place with more records,
// so the card never loses a record it already held.
// -----------------------------------------------------------------------------
#define SD_LOG_BLOCK_SIZE         512U
#define SD_LOG_FIRST_BLOCK        100U   // skip boot sector / partition table
#define SD_LOG_RECORDS_PER_BLOCK  (SD_LOG_BLOCK_SIZE / sizeof(LogRecord))
#define SD_LOG_FLUSH_TIMEOUT_MS   500U   // > 12 x 20 ms, so it never fires at 50 Hz

// Reset the logger to SD_LOG_FIRST_BLOCK with an empty buffer. Nothing touches
// the card until this has been called.
void sd_logger_init(void);

// Append one record to the RAM block buffer. The card is written only when the
// block fills (every 12th call), so most calls return without any SD traffic.
// Returns SD_FAIL if the logger is not initialised, or if a full block could
// not be written (the buffered records are kept and retried; this record is
// dropped).
SD_Status sd_logger_write(const LogRecord *record);

// Call every main-loop pass. Writes a partly filled block once its oldest
// unwritten record is SD_LOG_FLUSH_TIMEOUT_MS old, so records cannot sit in
// RAM indefinitely if the record rate drops. A failed attempt is retried one
// timeout later, not on every pass.
void sd_logger_poll(uint32_t now_ms);

// Write whatever is buffered now, full block or not.
SD_Status sd_logger_flush(void);

#endif
