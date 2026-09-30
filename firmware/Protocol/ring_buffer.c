#include "ring_buffer.h"
#include "stm32f446xx.h"   // __get_PRIMASK / __disable_irq / __set_PRIMASK

// One writer and one reader:
//   rx_write() runs only in UART RX interrupt context
//   rx_avail() / rx_peek() / rx_eat() run only in the main loop
// rx_head is touched only by the writer, rx_tail only by the reader.
// rx_count is shared, so it is volatile and the reader's read-modify-write
// of it runs with interrupts masked. The writer can't be interrupted by the
// reader, so its increment needs no guard.

static uint8_t           rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head    = 0;   // where new data is written
static volatile uint16_t rx_tail    = 0;   // where data is read from
static volatile uint16_t rx_count   = 0;   // bytes available
static volatile uint32_t rx_dropped = 0;   // bytes lost because the buffer was full


void rx_write(uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		// Full: drop the new byte rather than overwrite unread data. The
		// parser resyncs on the next sync bytes; overwriting would corrupt
		// a frame it might already be part-way through.
		if (rx_count >= RX_BUF_SIZE)
		{
			rx_dropped++;
			continue;
		}

		rx_buf[rx_head] = data[i];
		rx_head = (uint16_t)((rx_head + 1U) % RX_BUF_SIZE);
		rx_count++;
	}
}

uint16_t rx_avail(void)
{
	return rx_count;   // single 16-bit load: atomic on Cortex-M4
}

uint8_t rx_peek(uint16_t offset)
{
	return rx_buf[(rx_tail + offset) % RX_BUF_SIZE];
}

void rx_eat(uint16_t len)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if (len > rx_count)
		len = rx_count;             // never let the count underflow

	rx_tail  = (uint16_t)((rx_tail + len) % RX_BUF_SIZE);
	rx_count = (uint16_t)(rx_count - len);

	__set_PRIMASK(primask);         // restore, don't blindly re-enable
}

uint32_t rx_dropped_count(void)
{
	return rx_dropped;
}
