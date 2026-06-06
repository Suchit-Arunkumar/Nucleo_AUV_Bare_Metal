#include <stdio.h>
#include "stm32f446xx.h"
#include "i2c.h"

#define APB1CLK_MHZ  45U

void i2c1_init(void)
{
    // 1. enable GPIOB clock in RCC AHB1ENR
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 2. configure PB8 as alternate function mode (MODER = 10)
	GPIOB->MODER &= ~(3 << (2*8));
	GPIOB->MODER |=  (2 << (2*8));

    // 3. configure PB9 as alternate function mode (MODER = 10)
	GPIOB->MODER &= ~(3 << (2*9));
	GPIOB->MODER |=  (2 << (2*9));

    // 4. set PB8 and PB9 to open-drain in OTYPER
	GPIOB->OTYPER |= GPIO_OTYPER_OT8;
	GPIOB->OTYPER |= GPIO_OTYPER_OT9;

    // 5. set PB8 AF4 in AFRH (AFR[1])
	// 6. set PB9 AF4 in AFRH (AFR[1])
	GPIOB->AFR[1] |= (4 << 0);
	GPIOB->AFR[1] |= (4 << 4);

    // 7. enable I2C1 clock in RCC APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // 8. reset I2C1 via CR1 SWRST bit, then clear it
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

    // 9. set CR2 with APB1 frequency in MHz (45)
	I2C1->CR2 |= 45;

    // 10. set CCR = 225 for 100kHz standard mode
	I2C1->CCR = 225 ;

    // 11. set TRISE = 46
	I2C1->TRISE = 46;

    // 12. enable I2C1 via PE bit in CR1
	I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    // 1. generate START condition
	I2C1->CR1 |= I2C_CR1_START;

    // 2. wait for SB flag in SR1
	while(!(I2C1->SR1 & I2C_SR1_SB));

	// 3. send slave address with write bit (addr << 1 | 0)
	I2C1->DR = (addr << 1) | 0;

	// 4. wait for ADDR flag in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

	// 6. loop len times:
	//    a. wait for TXE flag
	//    b. write byte to DR

	for(int i = 0; i < len; i++){

		while(!(I2C1->SR1 & I2C_SR1_TXE));
		I2C1->DR = data[i] ;

	}

	// 7. STOP
	I2C1->CR1 |= I2C_CR1_STOP;
}

void i2c_read(uint8_t addr, uint8_t *buf, uint8_t len)
{
    // 1. generate START condition
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;

    // 2. wait for SB flag in SR1
	while(!(I2C1->SR1 & I2C_SR1_SB));

    // 3. send slave address with read bit (addr << 1 | 1)
    //    also enable ACK in CR1 before sending address
	I2C1->DR = (addr << 1) | 1;

    // 4. wait for ADDR flag in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR));

    // 5. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

    // 6. loop len times:
    //    a. if this is the last byte, disable ACK and generate STOP
    //    b. wait for RXNE flag in SR1
    //    c. read byte from DR into buf[i]
	for(int i = 0; i < len; i++){
	    if(i == (len - 1)){
	        I2C1->CR1 &= ~I2C_CR1_ACK;
	        I2C1->CR1 |= I2C_CR1_STOP;
	    }
	    while(!(I2C1->SR1 & I2C_SR1_RXNE));
	    buf[i] = I2C1->DR;
	}

}
