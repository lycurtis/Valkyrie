/*
 * spi.h
 *
 *  Created on: Jul 17, 2025
 *      Author: cly
 */

//#ifndef SPI_H_
//#define SPI_H_
//
//
//
//#endif /* SPI_H_ */

#pragma once

#include <stdint.h>

void SPI1_GPIO_Init(void);
void SPI1_Config(void);
void SPI1_Transmit(uint8_t *data, uint32_t size);
void SPI1_Receive(uint8_t *data, uint32_t size);
void CS_Enable(void);
void CS_Disable(void);

