#include "ring_buffer.h"

// internal buffer
static uint8_t  rx_buf[RX_BUF_SIZE];

// head — where new data is written
static uint16_t rx_head = 0;

// tail — where data is read from
static uint16_t rx_tail = 0;

// number of bytes available
static uint16_t rx_count = 0;


void rx_write(uint8_t *data, uint16_t len)
{
    	// 1. loop through len bytes

        // 2. write data[i] into rx_buf at rx_head position

        // 3. increment rx_head, wrap around using modulo RX_BUF_SIZE

        // 4. increment rx_count

		for(int i = 0; i < len; i++){

			rx_buf[rx_head] = data[i];
			rx_head++;
			rx_head = (rx_head % RX_BUF_SIZE);
			rx_count++;

		}


}

uint16_t rx_avail(void)
{
    // 1. return rx_count
	return(rx_count);
}

uint8_t rx_peek(uint16_t offset)
{
    // 1. return byte at position (rx_tail + offset) % RX_BUF_SIZE
	return rx_buf[(rx_tail + offset) % RX_BUF_SIZE];
}

void rx_eat(uint16_t len)
{
    // 1. move rx_tail forward by len, wrap using modulo RX_BUF_SIZE

    // 2. subtract len from rx_count


	rx_tail += len;
	rx_tail = rx_tail % RX_BUF_SIZE;

	rx_count = (rx_count - len);
}
