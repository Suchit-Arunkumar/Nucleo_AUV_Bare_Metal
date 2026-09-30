#include "sd_logger.h"
#include "sd_card.h"

#include <stdbool.h>
#include <string.h>

_Static_assert(sizeof(LogRecord) == 40U,
               "LogRecord size changed: records per block and the on-card format change with it");
_Static_assert(SD_LOG_RECORDS_PER_BLOCK >= 1U, "LogRecord no longer fits in a block");

// Static, not on the stack: the old per-call 512-byte stack buffer was half
// of the 1 KB the linker script reserves for the whole stack.
static uint8_t  block_buf[SD_LOG_BLOCK_SIZE];

static uint32_t current_block;    // where block_buf goes on the card
static uint32_t n_buffered;       // records in block_buf
static uint32_t n_on_card;        // how many of those the card already has
static uint32_t flush_due_from;   // g_tick the timeout counts from
static bool     ready = false;    // false until sd_logger_init()

static void start_next_block(void)
{
    current_block++;
    n_buffered = 0;
    n_on_card  = 0;
    memset(block_buf, 0, sizeof(block_buf));
}

void sd_logger_init(void)
{
    memset(block_buf, 0, sizeof(block_buf));
    current_block  = SD_LOG_FIRST_BLOCK;
    n_buffered     = 0;
    n_on_card      = 0;
    flush_due_from = 0;
    ready          = true;
}

SD_Status sd_logger_flush(void)
{
    if (!ready)
        return SD_FAIL;

    if (n_buffered == n_on_card)
        return SD_OK;                       // nothing the card doesn't have

    SD_Status status = sd_write_block(current_block, block_buf);
    if (status != SD_OK)
        return status;                      // keep everything; retried later

    if (n_buffered == SD_LOG_RECORDS_PER_BLOCK)
        start_next_block();
    else
        n_on_card = n_buffered;             // partial: rewritten in place when full

    return SD_OK;
}

SD_Status sd_logger_write(const LogRecord *record)
{
    if (!ready)
        return SD_FAIL;

    // Buffer still full means the last full-block write failed. Retry it
    // before accepting anything new.
    if (n_buffered == SD_LOG_RECORDS_PER_BLOCK)
    {
        if (sd_logger_flush() != SD_OK)
            return SD_FAIL;                 // card still failing: drop this record
    }

    if (n_buffered == n_on_card)
        flush_due_from = record->timestamp_ms;   // oldest record the card lacks

    memcpy(&block_buf[n_buffered * sizeof(LogRecord)], record, sizeof(LogRecord));
    n_buffered++;

    if (n_buffered == SD_LOG_RECORDS_PER_BLOCK)
        return sd_logger_flush();

    return SD_OK;
}

void sd_logger_poll(uint32_t now_ms)
{
    if (!ready || n_buffered == n_on_card)
        return;

    if ((uint32_t)(now_ms - flush_due_from) < SD_LOG_FLUSH_TIMEOUT_MS)
        return;

    if (sd_logger_flush() != SD_OK)
        flush_due_from = now_ms;            // back off one timeout, don't hammer a dead card
}
