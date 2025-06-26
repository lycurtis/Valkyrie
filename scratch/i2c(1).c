#include "i2c.h"

// Include your STM32 register headers here
#include "stm32f4xx.h"

/*
 * SCL: PB8
 * SDA: PB9
 * */

#define AC_GPIOB_OTYPER_OT8 (1U<<8)
#define AC_GPIOB_OTYPER_OT9 (1U<<9)
#define I2C_100KHZ 80
#define SD_MODE_MAX_RISE_TIME 17

void I2C_Init(void) {
	// Enable clock access for GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Configure GPIO pins for I2C (Alternate Function, Open Drain)
	// Set  PB8 and PB9 mode to alternate function [10] for I2C1
	GPIOB->MODER |= (1U<<17);
	GPIOB->MODER &= ~(1U<<16);

	GPIOB->MODER |= (1U<<19);
	GPIOB->MODER &= ~(1U<<18);

	GPIOB->OTYPER |= AC_GPIOB_OTYPER_OT8; // Output open drain pin 8
	GPIOB->OTYPER |= AC_GPIOB_OTYPER_OT9; // Output open drain pin 9

	// Enable Pulll-up [01] for PB8 and PB9
	GPIOB->PUPDR &= ~(1U<<17);
	GPIOB->PUPDR |= (1U<<16);

	GPIOB->PUPDR &= ~(1U<<19);
	GPIOB->PUPDR |= (1U<<18);

    // Configure RCC clock to I2C1 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure I2C timing registers
	// 1. Enter RESET Mode then immediately come out of RESET
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	// 2. Set clock Frequency to 16 MHz
	I2C1->CR2 |= (1U<<4); //FREQ[5:0] ==> (1U<<4) = 0b0100 = 16

	// 3. Set I2C to standard mode (100 kHz)
	/*
	 * CCR = (Peripheral Clock) / (2 * I2C Speed)
	 * CCR = 16 MHz / (2 * 100 kHz) = 80
	 *
	 * TRISE = (Maximum rise time/ T_PCLK1) + 1
	 * TRISE = (1000 ns / 62.5 ns) + 1
	 * TRISE = 16 + 1 = 17
	 * */
	I2C1->CCR = I2C_100KHZ;

	// 4. Set rise time
	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	// 5. Enable I2C peripheral
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_byteRead(char saddr, char maddr, char* data){

	volatile int tmp;

	// check busy flag in I2C_SR2
	while(I2C1->SR2 & I2C_SR2_BUSY){} //if busy then wait in while loop

	// Generate start condition
	I2C1->CR1 |= I2C_CR1_START;

	//if start bit is not set, wait/stuck in a while loop until start condition has been generated
	while(!(I2C1->SR1 & I2C_SR1_SB)){} // wait for SB (Start bit) flag to set in SR1

	// Transmit slave address + Write
	I2C1->DR = saddr << I2C_SR1_ADDR;

	// Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// Clear ADDR flag
	tmp = I2C1->SR2;

	// Send memory address
	I2C1->DR = maddr;

	// Wait until transmitter empty
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// Generate restart
	I2C1->CR1 |= I2C_CR1_START;

	// Wait until start flag is set
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// Transmit slave address + Read
	I2C1->DR = saddr << 1 | 1;

	// Wait until ADDR flag is set
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// Disable Acknowledge
	I2C1->CR1 &= ~I2C_CR1_ACK;

	// Clear ADDR flag
	tmp = I2C1->SR2;

	// Generate stop after data received
	I2C1->CR1 |= I2C_CR1_STOP;

	// Wait until RXNE flag is set
	while(!(I2C1->SR1 & I2C_SR1_RXNE)){}

	// Read data from DR
	*data++ = I2C1->DR;
}

void I2C1_burstRead(char saddr, char maddr, int n, char* data){

	volatile int temp;

	// wait until bus no busy
	while(I2C1->SR2 & I2C_SR2_BUSY){} //if busy then wait in while loop

	// Generate start condition
	I2C1->CR1 |= I2C_CR1_START;

	// wait until start flag is set
	while(!(I2C1->SR1 & I2C_SR1_SB)){} // wait for SB (Start bit) flag to set in SR1

	// Transmit slave address + Write
	I2C1->DR = saddr << I2C_SR1_ADDR; //saddr << 1

	// Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// Clear ADDR flag
	temp = I2C1->SR2;

	// Wait until transmitter empty
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// Send memory address
	I2C1->DR = maddr;

	// Wait until transmitter empty
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// Generate restart
	I2C1->CR1 |= I2C_CR1_START;

	// Wait until start flag is set
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// Transmit slave address + Read
	I2C1->DR = saddr << 1 | 1;

	// Wait until ADDR flag is set
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// Clear ADDR flag
	tmp = I2C1->SR2;

	// Enable Acknowledge
	I2C1->CR1 |= I2C_CR1_ACK;

	while(n>0U){
		// if 1 byte
		if(n == 1U){
			// Disable Acknowledge
			I2C1->CR1 &= ~I2C_CR1_ACK;

			// Generate stop after data received
			I2C1->CR1 |= I2C_CR1_STOP;

			// Wait until RXNE flag is set
			while(!(I2C1->SR1 & I2C_SR1_RXNE)){}

			// Read data from DR
			*data++ = I2C1->DR;
			break;
		}
		else{
			// Wait until RXNE flag is set
			while(!(I2C1->SR1 & I2C_SR1_RXNE)){}

			// Read data from DR
			*data++ = I2C1->DR;

			n--;
		}
	}

}

void I2C1_burstWrite(char saddr, char maddr, int n, char* data){

	volatile int temp;

	/* Wait until bus not busy */
	while (I2C1->SR2 & (I2C_SR2_BUSY)){}

	/* Generate start */
	I2C1->CR1 |= I2C_CR1_START;

	/* Wait until start flag is set */
	while (!(I2C1->SR1 & (I2C_SR1_SB))){}

	/* Transmit slave address */
	I2C1->DR = saddr << 1;

	/* Wait until Addr flag is set */
	while (!(I2C1->SR1 & (I2C_SR1_ADDR))){}

	/* Clear Addr flag */
	temp = I2C1->SR2;

	/* Wait until data register empty */
	while (!(I2C1->SR1 & (I2C_SR1_TXE))){}

	/* Send memory address */
	I2C1->DR = maddr;

	for (int i = 0; i < n; i++) {

		/* Wait until data register empty */
		while (!(I2C1->SR1 & (I2C_SR1_TXE))){}

		/* Transmit memory address */
		I2C1->DR = *data++;
	}

	/* Wait until transfer finished */
	while (!(I2C1->SR1 & (I2C_SR1_BTF))){}

	/* Generate stop */
	I2C1->CR1 |= I2C_CR1_STOP;
}

 BTF (Byte Transfer Finished) flag is set (optional but recommended)
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
	// 8. Generate a STOP condition (set the STOP bit in CR1)
	I2C1->CR1 |= I2C_CR1_STOP;
}
