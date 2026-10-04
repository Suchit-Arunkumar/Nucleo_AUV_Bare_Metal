#include <stdio.h>
#include "stm32f446xx.h"
#include "i2c.h"
#include "system_init.h"   // g_tick, for timeouts

#define APB1CLK_MHZ     45U
#define I2C_TIMEOUT_MS  5U    // per flag wait; a 100 kHz byte takes ~0.1 ms

// Peripheral part of the setup: reset, timing, enable. Also used to recover
// the peripheral after a failed transfer leaves it holding the bus.
static void i2c1_periph_config(void)
{
    // reset I2C1 via CR1 SWRST bit, then clear it
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

    // CR2.FREQ = APB1 frequency in MHz (45). Plain write: the field is
    // zero after SWRST, and |= would merge with anything left in it.
	I2C1->CR2 = APB1CLK_MHZ;

    // CCR = 225 for 100 kHz standard mode
	I2C1->CCR = 225;

    // TRISE = 46
	I2C1->TRISE = 46;

    // enable I2C1 via PE bit in CR1
	I2C1->CR1 |= I2C_CR1_PE;
}

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

    // 4a. internal pull-ups on PB8/PB9 (PUPDR = 01). The Nucleo has no I2C
    //     pull-ups; with nothing attached the lines would float and a START
    //     might never complete. With pull-ups an absent device is a clean
    //     NACK. The Bar30's own pull-ups sit in parallel when it is fitted.
	GPIOB->PUPDR &= ~((3U << (2*8)) | (3U << (2*9)));
	GPIOB->PUPDR |=   (1U << (2*8)) | (1U << (2*9));

    // 5-6. set PB8 and PB9 to AF4 in AFRH (AFR[1]), clearing each 4-bit
    //      field first so no earlier AF value can merge in
	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |=   (4U   << 0) | (4U   << 4);

    // 7. enable I2C1 clock in RCC APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // 8-12. reset, timing, enable
	i2c1_periph_config();
}

// Wait for an SR1 flag. Returns 0 when it sets, -1 on NACK (AF) or timeout.
// Every wait in this driver goes through here: an absent or stuck device can
// no longer hang the caller.
static int i2c_wait_sr1(uint32_t flag)
{
	uint32_t start = g_tick;

	while (!(I2C1->SR1 & flag))
	{
		if (I2C1->SR1 & I2C_SR1_AF)
			return -1;                                   // slave did not ACK
		if ((uint32_t)(g_tick - start) > I2C_TIMEOUT_MS)
			return -1;                                   // bus or device stuck
	}
	return 0;
}

// End a failed transfer: clear AF, request STOP, and if the bus is still busy
// afterwards, reset the peripheral so the next transfer starts clean.
static int i2c_abort(void)
{
	I2C1->SR1 &= ~I2C_SR1_AF;
	I2C1->CR1 |= I2C_CR1_STOP;

	uint32_t start = g_tick;
	while ((I2C1->CR1 & I2C_CR1_STOP) && (uint32_t)(g_tick - start) <= 1U)
	{
	}

	if (I2C1->SR2 & I2C_SR2_BUSY)
		i2c1_periph_config();

	return -1;
}

int i2c_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    // 1. generate START condition, wait for SB
	I2C1->CR1 |= I2C_CR1_START;
	if (i2c_wait_sr1(I2C_SR1_SB)) return i2c_abort();

	// 2. send slave address with write bit, wait for ADDR (AF = no device)
	I2C1->DR = (uint8_t)(addr << 1);
	if (i2c_wait_sr1(I2C_SR1_ADDR)) return i2c_abort();

	// 3. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

	// 4. for each byte: wait for TXE, write it to DR
	for (int i = 0; i < len; i++)
	{
		if (i2c_wait_sr1(I2C_SR1_TXE)) return i2c_abort();
		I2C1->DR = data[i];
	}

	// 5. wait for BTF, then STOP. With the last byte just written to DR,
	//    the byte before it is still in the shift register; a STOP set now
	//    ends the transfer after that byte and the last one is never sent
	//    (RM0390 master transmitter, EV8_2: STOP only once TxE and BTF are
	//    both set). Single-byte writes got away with it, because their byte
	//    leaves DR for the shift register at once. The MPU-6050 wake
	//    command {0x6B, 0x00} lost its 0x00 on the first hardware run, and
	//    the sensor stayed asleep reading zero.
	if (i2c_wait_sr1(I2C_SR1_BTF)) return i2c_abort();
	I2C1->CR1 |= I2C_CR1_STOP;
	return 0;
}

int i2c_read(uint8_t addr, uint8_t *buf, uint8_t len)
{
    // 1. enable ACK, generate START condition, wait for SB
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	if (i2c_wait_sr1(I2C_SR1_SB)) return i2c_abort();

    // 2. send slave address with read bit, wait for ADDR
	I2C1->DR = (uint8_t)((addr << 1) | 1U);
	if (i2c_wait_sr1(I2C_SR1_ADDR)) return i2c_abort();

    // 3. clear ADDR flag by reading SR1 then SR2
	(void)I2C1->SR1;
	(void)I2C1->SR2;

    // 4. for each byte: before the last one, disable ACK and request STOP;
    //    wait for RXNE, read DR
	for (int i = 0; i < len; i++)
	{
	    if (i == (len - 1))
	    {
	        I2C1->CR1 &= ~I2C_CR1_ACK;
	        I2C1->CR1 |= I2C_CR1_STOP;
	    }
	    if (i2c_wait_sr1(I2C_SR1_RXNE)) return i2c_abort();
	    buf[i] = I2C1->DR;
	}

	return 0;
}
