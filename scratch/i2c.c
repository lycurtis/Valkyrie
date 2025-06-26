/*
 * i2c.c
 *
 *  Created on: Jun 23, 2025
 *      Author: cly
 */

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

void I2C_Write(uint8_t dev_addr, uint8_t *data, uint16_t len) {
    // Implement I2C write sequence (START, SEND ADDR, WRITE DATA, STOP)

	volatile uint32_t temp;

	// 1. Generate a START condition (set the START bit in CR1)
	I2C1->CR1 |= I2C_CR1_START;

	// 2. Wait until the SB (Start Bit) flag is set in SR1 (indicates start has been generated)
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// 3. Send the slave address with the write bit (LSB = 0)
	I2C1->DR = dev_addr << 1;

	// 4. Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// 5. Clear the ADDR flag by reading SR1 followed by SR2 (this is required)
	temp = I2C1->SR1;
	temp = I2C1->SR2;

	// 6. Loop through the data array:
	//    For each byte:
	//    a. Write the byte to the DR (Data Register)
	//    b. Wait until the TXE (Transmit Data Register Empty) flag is set (means you can send the next byte)
	for(int i = 0; i < len; i++){
		I2C1->DR = data[i];
		while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	}

	// 7. After all bytes are sent, wait until BTF (Byte Transfer Finished) flag is set (optional but recommended)
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}
	// 8. Generate a STOP condition (set the STOP bit in CR1)
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_Read(uint8_t dev_addr, uint8_t *data, uint16_t len) {
    // Implement I2C read sequence (START, SEND ADDR, READ DATA, STOP)
	volatile uint32_t temp;

    // 1. Generate a START condition by setting the START bit in CR1
	I2C1->CR1 |= I2C_CR1_START;

    // 2. Wait until the SB (Start Bit) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

    // 3. Send the slave address with the READ bit (LSB = 1)
	I2C1->DR = dev_addr;

    // 4. Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

    // 5. Clear the ADDR flag by reading SR1 then SR2 (required)
	temp = I2C1->SR1;
	temp = I2C1->SR2;

    // 6. If only ONE byte to read:
	if(len == 1){
		//    a. Clear the ACK bit in CR1 (NACK after the next byte)
		I2C1->CR1 &= ~I2C_CR1_ACK;

		//    b. Generate the STOP condition immediately
		I2C1->CR1 |= I2C_CR1_STOP;

		//    c. Wait until RXNE (Receive Buffer Not Empty) flag is set
		while(!(I2C1->SR1 & I2C_SR1_RXNE)){}

		//    d. Read data from DR
		*data = I2C1->DR;
	}
	else{
		// 7. If MORE than one byte:
		//    a. For all bytes except the last two:
		for(int i = 0; i < len; i++){
			while(!(I2C1->SR1 & I2C_SR1_RXNE)){} // Wait until RXNE is set

			// if second to last byte
			if(i == (len - 2)){
				//    b. When only two bytes remain:
				//       - Clear ACK bit (prepare NACK for second-to-last byte)
				//       - Generate STOP condition
				//       - Read remaining two bytes when RXNE is set
				I2C1->CR1 &= ~I2C_CR1_ACK;
				I2C1->CR1 |= I2C_CR1_STOP;
			}
			//READ DATA FROM DR INTO BUFFER
			data[i] = I2C1->DR;
		}
	}

    // 8. Make sure to re-enable ACK after the transfer if you disabled it
	I2C1->CR1 |= I2C_CR1_ACK;
}

void I2C_Mem_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    // Send register address, then write data

	volatile uint32_t temp;

	// 1. Generate a START condition (set the START bit in CR1)
	I2C1->CR1 |= I2C_CR1_START;

	// 2. Wait until the SB (Start Bit) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// 3. Send the slave address with the write bit (LSB = 0)
	I2C1->DR = dev_addr << 1;

	// 4. Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// 5. Clear the ADDR flag by reading SR1 followed by SR2
	// To clear this flag (which is required to continue the transfer):
	temp = I2C1->SR1; // Read SR1
	temp = I2C1->SR2; // Then immediately after read SR2

	// 6. Send the memory/register address (mem_addr)
	I2C1->DR = reg_addr;

	// 7. Wait until the TXE (Transmit Data Register Empty) flag is set
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// 8. Loop through the data array:
	//    For each byte:
	//    a. Write the byte to the DR register
	//    b. Wait until TXE is set
	for(int i = 0; i < len; i++){
		I2C1->DR = data[i]; //a. Write the current byte to the DR register

		while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	}

	// 9. After all bytes are sent, wait until BTF (Byte Transfer Finished) flag is set (optional but recommended)
	while(!(I2C1->SR1 & I2C_SR1_BTF)){}

	// 10. Generate a STOP condition (set the STOP bit in CR1)
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_Mem_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    // Send register address, then read data
	/*
	 * [START] → [Slave Address + Write] → [Register Address] →
	 * → [REPEATED START] → [Slave Address + Read] → [Read Data] → [STOP]
	 * */
	/*
	 * When you use Mem_Read (Memory/Register Read) on devices like the MPU6050,
	 * you need to:
		Tell the device which internal memory/register you want to read from.
		Then actually read the data from that register.
	 */
	volatile uint32_t temp;

	// 1. Generate a START condition (set START in CR1)
	I2C1->CR1 |= I2C_CR1_START;

	// 2. Wait until the SB (Start Bit) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// 3. Send the slave address with the WRITE bit (LSB = 0)
	I2C1->DR = dev_addr << 1;

	// 4. Wait until the ADDR (Address Sent) flag is set in SR1
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// 5. Clear the ADDR flag by reading SR1 then SR2
	temp = I2C1->SR1;
	temp = I2C1->SR2;

	// 6. Send the memory/register address (mem_addr)
	I2C1->DR = reg_addr;

	// 7. Wait until TXE (Transmit Buffer Empty) flag is set
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	// 8. Generate a REPEATED START condition (set START in CR1 again)
	I2C1->CR1 |= I2C_CR1_START;

	// 9. Wait until SB flag is set again
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	// 10. Send the slave address with the READ bit (LSB = 1)
	I2C1->DR = (dev_addr << 1) | 1;

	// 11. Wait until ADDR flag is set
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}

	// 12. Clear the ADDR flag by reading SR1 and SR2
	temp = I2C1->SR1;
	temp = I2C1->SR2;

	// 13. If ONE byte to read:
	//     a. Clear ACK bit (prepare NACK after this byte)
	//     b. Generate STOP condition
	//     c. Wait for RXNE flag
	//     d. Read the byte from DR
	if(len == 1){
		I2C1->CR1 &= ~I2C_CR1_ACK; // a. Clear ACK bit
		I2C1->CR1 |= I2C_CR1_STOP; // b. Generate stop condition
		while(!(I2C1->SR1 & I2C_SR1_RXNE)){} // c. Wait for RXNE flag
		*data = I2C1->DR; // d. read the byte from DR

		//Re-enable ACK for future communications
		I2C1->CR1 |= I2C_CR1_ACK;
	}
	else{ // 14. If MORE than one byte:
		// a. For all bytes except the last two:
		//    - Wait for RXNE flag
		//    - Read data from DR into buffer
		//     b. When two bytes remain:
		//        - Clear ACK bit
		//        - Generate STOP condition
		//        - Read remaining two bytes as RXNE becomes set
		for(int i = 0; i < len; i++){
			if(i == (len - 2)){
				// Prepare to NACK the second to last byte
				I2C1->CR1 &= ~I2C_CR1_ACK;

				//Send STOP condition
				I2C1->CR1 |= I2C_CR1_STOP;
			}

			while(!(I2C1->SR1 & I2C_SR1_RXNE)){} // Wait for RXNE flag

			//READ THE DATA
			data[i] = I2C1->DR;
		}

		// 15. After transfer, re-enable ACK if it was disabled
		I2C1->CR1 |= I2C_CR1_ACK;
	}
}
