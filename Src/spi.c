/*
 * spi.c
 *
 *  Created on: Jul 17, 2025
 *      Author: cly
 */

#include "spi.h"
#include "stm32f4xx.h"

/*
 * PA5 SPI1_SCK
 * PA6 SPI1_MISO
 * PA7 SPI1_MOSI
 * PA9 SS/CS (Slave Select/Chip Select) pin of any choice
 * */
void SPI1_GPIO_Init(void){
	// enable clock access to GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Set PA5, PA6, and PA7 to alternate function mode
	GPIOA->MODER |= (1U<<11);
	GPIOA->MODER &= ~(1U<<10); // PA5

	GPIOA->MODER |= (1U<<13);
	GPIOA->MODER &= ~(1U<<12); // PA6

	GPIOA->MODER |= (1U<<15);
	GPIOA->MODER &= ~(1U<<14); // PA7

	// Set PA9 as an output pin
	GPIOA->MODER &= ~(1U<<19);
	GPIOA->MODER |= (1U << 18);

	// Set PA5, PA6, and PA7 alternate function type to SPI1 (AF05)->0101
	GPIOA->AFR[0] |= (1U<<20); // Recall: AFR[0] corresponds to the lower pins
	GPIOA->AFR[0] &= ~(1U<<21); // lower pins are pins [0:7] while upper pins [8:15]
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23); // PA5 -> AFRL5

	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27); // PA6 -> AFRL6

	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31); // PA7 -> AFRL7

}

void SPI1_Config(void){
	// Configure BR (baud rate control): fpclk/4 = 001
	SPI1->CR1 &= ~(1U<<5);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 |= (1U<<3);

	// Set CPOL (clock polarity) to 1 and CPHA (clock phase) to 1
	SPI1->CR1 |= SPI_CR1_CPOL; // CK to 1 when Idle
	SPI1->CR1 |= SPI_CR1_CPHA; // The second clock transition is the first data capture edge

	// Enable full duplex using by setting RXONLY (bit 10) to 0
	SPI1->CR1 &= ~SPI_CR1_RXONLY;

	// Set MSB transmitted first by disabling LSBFIRST (bit 7) to 0
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

	// Set to Master configuration (STM32 is master, external device is slave)
	SPI1->CR1 |= SPI_CR1_MSTR;

	// Set DFF (Data Frame Format) 0: 8-bit DDF; 1: 16-bit DFF
	SPI1->CR1 &= ~SPI_CR1_DFF; // 8-bit DFF

	// SSM (software slave management) and SSI (internal slave select) to 1
	SPI1->CR1 |= SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_SSI;

	// Enable the SPI peripheral by setting SPE (bit 6) to 1
	SPI1->CR1 |= SPI_CR1_SPE; // Peripheral enabled
}

void SPI1_Transmit(uint8_t *data, uint32_t size){
	uint32_t i=0;
//	uint8_t temp; // dummy variable for reading

	while(i < size){
		// Wait until TXE is set 1 (Buffer is empty)
		while(!(SPI1->SR & SPI_SR_TXE)){} // waits if buffer is not empty

		// Write the data to the DR (data register)
		SPI1->DR = data[i];
		i++;
	}
	// Wait until TXE is set
	while(!(SPI1->SR & SPI_SR_TXE)){} // waits if buffer is not empty

	// Wait for BUSY flag to reset
	while((SPI1->SR & SPI_SR_BSY)){} // while SPI is busy we wait

	/* Clear OVR (overrun) flag
	From reference manual: "Clearing the OVR bit is done by a read access
	to the SPI_DR register followed by a read access to the SPI_SR register */
//	temp = SPI1->DR;
//	temp = SPI1->SR; // ALTNERATIVE METHOD BELOW
	(void)SPI1->DR;
	(void)SPI1->SR;
}

void SPI1_Receive(uint8_t *data, uint32_t size){
	while(size){
		// Send dummy data
		SPI1->DR = 0;

		// wait for the status register to set RXNE flag to notify
		// Representing that data is ready to be received
		while(!(SPI1->SR & SPI_SR_RXNE)){} // waiting until RX buffer is not empty

		// Read data from DR (data register)
		*data++ = (SPI1->DR);
		size--;
	}
}

void CS_Enable(void){
	GPIOA->ODR &= ~(1U<<9); // Set the bit to 0 (low) to enable
}

void CS_Disable(void){
	GPIOA->ODR |= (1U<<9); // Set the bit to 1 (high) to disable
}
