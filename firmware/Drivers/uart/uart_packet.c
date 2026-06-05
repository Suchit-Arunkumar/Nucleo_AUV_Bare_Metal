#include "uart_packet.h"
#include "stm32f446xx.h"
#include "ring_buffer.h"

#define APB2CLK 90000000U
#define UART1_BR 115200U

// DMA receive buffer — DMA writes directly into this
static uint8_t dma_rx_buf[DMA_BUF_SIZE];

// track last DMA position to detect new bytes
static uint16_t last_dma_pos = 0;

void uart1_init(void)
{
    // 1. enable GPIOA clock in RCC AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. configure PA9 as alternate function mode (MODER = 10)
	GPIOA->MODER &= ~(3U << (2*9));
	GPIOA->MODER |=  (2U << (2*9));

    // 3. configure PA10 as alternate function mode (MODER = 10)
	GPIOA->MODER &= ~(3U << (2*10));
	GPIOA->MODER |=  (2U << (2*10));

    // 4. set PA9 alternate function to AF7 (USART1 TX) in AFRH
    //    note: PA9 is pin 9, AFRH is AFR[1], position = (9-8)*4 = 4
	GPIOA->AFR[1] |= (7 << 4);

    // 5. set PA10 alternate function to AF7 (USART1 RX) in AFRH
    //    note: PA10 is pin 10, position = (10-8)*4 = 8
	GPIOA->AFR[1] |= (7 << 8);

    // 6. enable USART1 clock in RCC APB2ENR
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 7. set baud rate in USART1 BRR (APB2 = 90MHz, target = 115200)
	USART1->BRR = ((APB2CLK + UART1_BR/2)/UART1_BR);

    // 8. enable receiver (RE bit) in USART1 CR1
	USART1->CR1 |= USART_CR1_RE;

    // 9. enable IDLE line interrupt (IDLEIE bit) in USART1 CR1
	USART1->CR1 |= USART_CR1_IDLEIE;

    // 10. enable DMA RX request (DMAR bit) in USART1 CR3
	USART1->CR3 |= USART_CR3_DMAR;

    // 11. enable USART1 (UE bit) in USART1 CR1
	USART1->CR1 |= USART_CR1_UE;

	// 12. enable DMA2 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// 13. configure DMA2 Stream2 for USART1 RX

	    // disable stream first before configuring
	    DMA2_Stream2->CR &= ~DMA_SxCR_EN;

	    // wait until stream is disabled
	    while(DMA2_Stream2->CR & DMA_SxCR_EN);

	    // set peripheral address — where DMA reads from
	    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;

	    // set memory address — where DMA writes to
	    DMA2_Stream2->M0AR = (uint32_t)dma_rx_buf;

	    // set number of data items
	    DMA2_Stream2->NDTR = DMA_BUF_SIZE;

	    // set channel 4 (CHSEL bits 27:25 = 100)
	    DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL;
	    DMA2_Stream2->CR |= (4U << DMA_SxCR_CHSEL_Pos);

	    // enable circular mode
	    DMA2_Stream2->CR |= DMA_SxCR_CIRC;

	    // enable memory increment
	    DMA2_Stream2->CR |= DMA_SxCR_MINC;

	    // set transfer direction — peripheral to memory
	    DMA2_Stream2->CR &= ~DMA_SxCR_DIR;

	    // enable the stream
	    DMA2_Stream2->CR |= DMA_SxCR_EN;


	 // 14. enable USART1 interrupt in NVIC
	 NVIC_SetPriority(USART1_IRQn, 1);
	 NVIC_EnableIRQ(USART1_IRQn);

}



void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_IDLE)
    {
        volatile uint32_t dummy;

        /* Clear IDLE flag */
        dummy = USART1->SR;
        dummy = USART1->DR;

        /* Current DMA position */
        uint16_t current_pos =
            DMA_BUF_SIZE -
            DMA2_Stream2->NDTR;

        /* No wraparound */
        if(current_pos > last_dma_pos)
        {
            rx_write(
                &dma_rx_buf[last_dma_pos],
                current_pos - last_dma_pos
            );
        }
        /* DMA wrapped around */
        else if(current_pos < last_dma_pos)
        {
            rx_write(
                &dma_rx_buf[last_dma_pos],
                DMA_BUF_SIZE - last_dma_pos
            );

            rx_write(
                &dma_rx_buf[0],
                current_pos
            );
        }

        last_dma_pos = current_pos;

        (void)dummy;
    }
}


void uart1_write_buf(uint8_t *buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++)
    {
        while(!(USART1->SR & USART_SR_TXE))
        {
        }

        USART1->DR = buf[i];
    }
}

void uart1_write_byte(uint8_t b)
{
    while(!(USART1->SR & USART_SR_TXE))
    {
    }

    USART1->DR = b;
}
