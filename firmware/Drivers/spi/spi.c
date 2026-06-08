#include "spi.h"
#include "stm32f446xx.h"

void spi1_init(void)
{
    // 1. enable GPIOA clock in RCC AHB1ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. enable GPIOB clock in RCC AHB1ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 3. enable SPI1 clock in RCC APB2ENR
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 4. configure PA5 (SCK) as alternate function mode (MODER = 10)
    GPIOA->MODER &= ~(3U << (2*5));
    GPIOA->MODER |=  (2U << (2*5));

    // 5. configure PA6 (MISO) as alternate function mode (MODER = 10)
    GPIOA->MODER &= ~(3U << (2*6));
    GPIOA->MODER |=  (2U << (2*6));

    // 6. configure PA7 (MOSI) as alternate function mode (MODER = 10)
    GPIOA->MODER &= ~(3U << (2*7));
    GPIOA->MODER |=  (2U << (2*7));

    // 7. set PA5, PA6, PA7 to high speed in OSPEEDR
    GPIOA->OSPEEDR |= (3U << (2*5));
    GPIOA->OSPEEDR |= (3U << (2*6));
    GPIOA->OSPEEDR |= (3U << (2*7));

    // 8. set PA5 to AF5 in AFR[0]
    GPIOA->AFR[0] |= (5U << 20);

    // 9. set PA6 to AF5 in AFR[0]
    GPIOA->AFR[0] |= (5U << 24);

    // 10. set PA7 to AF5 in AFR[0]
    GPIOA->AFR[0] |= (5U << 28);

    // 11. configure PA4 (CS_OLED) as GPIO output (MODER = 01)
    GPIOA->MODER &= ~(3U << (2*4));
    GPIOA->MODER |=  (1U << (2*4));

    // 12. configure PB0 (CS_SD) as GPIO output (MODER = 01)
    GPIOB->MODER &= ~(3U << (2*0));
    GPIOB->MODER |=  (1U << (2*0));

    // 13. deselect both CS pins high using BSRR
    GPIOA->BSRR = (1U << 4);   // PA4 high — OLED deselected
    GPIOB->BSRR = (1U << 0);   // PB0 high — SD deselected

    // 14. configure SPI_CR1
    SPI1->CR1 |= SPI_CR1_SSM;      // software slave management
    SPI1->CR1 |= SPI_CR1_SSI;      // internal NSS high, prevents MODF
    SPI1->CR1 |= (3U << 3);        // BR[2:0] = 011 → fPCLK/16
    SPI1->CR1 |= SPI_CR1_MSTR;     // master mode

    // 15. enable SPI1 by setting SPE bit in CR1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_transmit(uint8_t data)
{
    // 1. wait until TXE flag is set in SR (TX buffer empty)
    while(!(SPI1->SR & SPI_SR_TXE));

    // 2. write data to DR as 8-bit (cast DR pointer to uint8_t*)
    *((__IO uint8_t*)&SPI1->DR) = data;

    // 3. wait until TXE flag is set again (shift register drained)
    while(!(SPI1->SR & SPI_SR_TXE));

    // 4. wait until BSY flag clears (last bit fully clocked out)
    while(SPI1->SR & SPI_SR_BSY);

    // 5. dummy read DR to clear RXNE and prevent OVR flag
    (void)(*((__IO uint8_t*)&SPI1->DR));
}

uint8_t spi_transfer(uint8_t data)
{
    // 1. wait until TXE flag is set in SR
    while(!(SPI1->SR & SPI_SR_TXE));

    // 2. write data to DR as 8-bit
    *((__IO uint8_t*)&SPI1->DR) = data;

    // 3. wait until RXNE flag is set in SR (RX buffer has data)
    while(!(SPI1->SR & SPI_SR_RXNE));

    // 4. return DR (received byte)
    return (uint8_t)SPI1->DR;
}

uint8_t spi_receive(void)
{
    // 1. call spi_transfer with 0xFF dummy byte and return result
    return spi_transfer(0xFF);
}

void spi_select_oled(void)
{
    // 1. pull PA4 low using BSRR (bits 16+n clear the pin)
    GPIOA->BSRR = (1U << (4 + 16));
}

void spi_deselect_oled(void)
{
    // 1. pull PA4 high using BSRR (bits 0+n set the pin)
    GPIOA->BSRR = (1U << 4);
}

void spi_select_sd(void)
{
    // 1. pull PB0 low using BSRR
    GPIOB->BSRR = (1U << (0 + 16));
}

void spi_deselect_sd(void)
{
    // 1. pull PB0 high using BSRR
    GPIOB->BSRR = (1U << 0);
}
