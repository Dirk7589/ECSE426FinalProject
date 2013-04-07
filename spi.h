/**
*@file spi.h
*@author Dirk Dubois, Alain Slak
*@date March 22nd, 2013
*@brief A header file that contains the SPI generic drivers
*/

#ifndef __SPI_H
#define __SPI_H

/**@brief A function that initializes SPI1
*@retval None
*/
void initSPI(void);

/**
*@brief A function that starts a DMA transfer using the buffers provided, on SPI1 using the passed chip select port
*@param[inout] rx A pointer to the receive buffer in Bytes
*@param[in] tx A pointer to the transmit buffer in Bytes
*@param[in] bufferSize The size of the rx and tx buffer
*@param[in] csPort The GPIO port that has the CS line for SPI1
*@param[in] csPin The GPIO pin that the CS line is connected to for SPI1
*@retval None
*/
void SPI_DMA_Transfer(const uint8_t* rx, const uint8_t* tx, const uint8_t bufferSize, GPIO_TypeDef* csPort, uint16_t csPin);
#endif
