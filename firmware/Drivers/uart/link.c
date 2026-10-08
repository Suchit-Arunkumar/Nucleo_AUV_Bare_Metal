#include "link.h"
#include "stm32f446xx.h"
#include "ring_buffer.h"
#include "uart.h"
#include "uart_packet.h"

#if LINK_PORT_STLINK

void link_init(void)
{
	// USART2 is already configured by uart2_init() as the printf console:
	// PA2/PA3 AF7, 115200 baud, TE and UE set. Add the receiver and the
	// RXNE interrupt. Same priority as USART1 would use: below TIM7.
	USART2->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);
}

void link_send(const uint8_t *buf, uint16_t len)
{
	uart2_write_buf((uint8_t *)buf, len);
}

const char *link_port_name(void)
{
	return "USART2 ST-LINK VCP";
}

// One interrupt per received byte: at 115200 baud that is at most ~11.5k/s,
// and 50 Hz command frames are ~3.1k/s. The byte goes straight into the same
// ring buffer USART1's DMA path fills, so the parser is identical either way.
void USART2_IRQHandler(void)
{
	uint32_t sr = USART2->SR;

	if (sr & USART_SR_ORE)
		rx_note_overrun();   // a byte was lost before DR could be read

	if (sr & (USART_SR_RXNE | USART_SR_ORE))
	{
		// Reading DR after SR clears RXNE and any ORE/NE/FE with it.
		uint8_t b = (uint8_t)USART2->DR;
		rx_write(&b, 1);
	}
}

#else

void link_init(void)
{
	uart1_init();
}

void link_send(const uint8_t *buf, uint16_t len)
{
	uart1_write_buf((uint8_t *)buf, len);
}

const char *link_port_name(void)
{
	return "USART1 PA9/PA10";
}

#endif
