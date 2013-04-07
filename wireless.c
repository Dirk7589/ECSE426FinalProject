/**
*@file wireless.c
*@author Dirk Dubois, Alain Slak
*@date March 20th, 2013
*@brief A set of functions to interface with the ez430-CC2500
*
*/

/*Includes*/
#include <stdint.h>
#include "stm32f4xx.h"
#include "spi.h"
#include "wireless.h"
#include "common.h"

/*Defines*/
#define DUMMY_BYTE 0x00


/**
*@brief A function to setup and initialize wireless communication
*@param[inout] None
*@retval None
*/
void initWireless(void){
  strobeCommand[0] = SIDLE|SINGLEBYTE_WR; //Set for receive mode
  SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
  while(dmaFromWirelessFlag);
  osMutexRelease(dmaId);
 
   SPI_DMA_Transfer(rxWirelessInit, txWirelessInit, WIRELESS_BUFFER_INIT_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
   //Wait for the wireless INIT to finish
	while(dmaFromWirelessFlag);
	osMutexRelease(dmaId);//Clear Mutex
  
  txWirelessInit[0] = 0x00 | MULTIPLEBYTE_RD;
	SPI_DMA_Transfer(rxWirelessInit, txWirelessInit, WIRELESS_BUFFER_INIT_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
  //Wait for the wireless INIT to finish
	while(dmaFromWirelessFlag);
	osMutexRelease(dmaId);//Clear Mutex
	
	strobeCommand[0] = SIDLE|SINGLEBYTE_WR; //Set for receive mode
  SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
  while(dmaFromWirelessFlag);
  osMutexRelease(dmaId);
}

/**
*@brief A function to write to a specified register on the wireless board
*@param[in] pBuffer The data to be sent to the device
*@param[in] writeAddr The address of the register to send data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessSend(uint8_t* pBuffer, uint8_t writeAddr, uint16_t numOfBytes){
	uint8_t addr;
	
	// If more than 1 byte, enable burst
	if (numOfBytes > 0x01) {
		writeAddr |= (uint8_t)MULTIPLEBYTE_WR;
	}
	// Set chip select Low at the start of the transmission
	WIRELESS_CS_LOW();
	
	// NumOfBytes = 0 represents a strobe
	// Send the address to where the data is to be sent
	wirelessSendByte(writeAddr);
	// Send the data
	while (numOfBytes >= 0x01) {
		addr = wirelessSendByte(*pBuffer);
		numOfBytes--;
		pBuffer++;
	}
	// Set chip select back to high at end of transmission
	WIRELESS_CS_HIGH();
}

/**
*@brief A function that writes a byte for the Wireless module over SPI
*@param[in] data The data to be written to the wireless device
*@retval uint8_t
*/
uint8_t wirelessSendByte(uint8_t data){
	__IO uint32_t WIRELESSTimeout = WIRELESS_FLAG_TIMEOUT;
	
	// While flag is reset
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
// 	{
// 		if ((WIRELESSTimeout--) == 0) 
// 			while(1);
// 	}
	
	// Send the data on the SPI line
	SPI_I2S_SendData(SPI1, data);
	
	// While flag is reset
	WIRELESSTimeout = WIRELESS_FLAG_TIMEOUT;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
// 	{
// 		if ((WIRELESSTimeout--) == 0) 
// 			while(1);
// 	}
	
	// Return status byte from the wireless peripheral
	return (uint8_t) SPI_I2S_ReceiveData(SPI1);
}

/**
*@brief A function to read from a specified resgister on the wireless board
*@param[inout] pBuffer The data to be read to the device
*@param[in] readAddr The address of the register to read data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval The received value
*/
void wirelessRead(uint8_t* pBuffer, uint8_t readAddr, uint16_t numOfBytes){
	// If more than 1 byte, enable burst
	if (numOfBytes > 0x01) {
		readAddr |= (uint8_t)(MULTIPLEBYTE_RD);
	} else {
		readAddr |= (uint8_t)SINGLEBYTE_RD;
		}
	// Set chip select Low at the start of the transmission
	WIRELESS_CS_LOW();
	
	// NumOfBytes = 0 represents a status register read
	if (numOfBytes == 0x00) {
		readAddr = readAddr | MULTIPLEBYTE_RD;
		*pBuffer = wirelessSendByte(readAddr);	
		*pBuffer = wirelessSendByte(DUMMY_BYTE);
	} 
	else {
		// Send the address to where the data is to be sent
		*pBuffer = wirelessSendByte(readAddr);
		// Send the data
		while (numOfBytes > 0x00) {
			*pBuffer = wirelessSendByte(DUMMY_BYTE);
			numOfBytes--;
			pBuffer++;
		}
	}
	// Set chip select back to high at end of transmission
	WIRELESS_CS_HIGH();
}



/**
  * @brief  Receive packets wirelessly
  * @param  pitchBuffer - array holding values for the pitch angle
	*					rollBuffer 	- array holding values for the roll angle
	*					buffLength	- length of the two above arrays
	*					packet			- received packet, returned by reference
  * @retval None
  */
void wirelessRX(uint8_t* packet) {
	uint8_t garbage = 0x00;
	uint8_t data = 0x00;
	
	// Strobe RX
	wirelessSend(&garbage, SRX, 0);
	osDelay(50);
	
	// Initialize data to 0
	data = 0x00;
	uint32_t t = 0;
	uint32_t i = 0;
// 	wirelessRead(&data, PKTSTATUS, 0);
// 	// Loop while the CRC fails and the sync word has not been found
// 	while ((data & 0x80 != 0x80) && (data & 0x08 != 0x08)) {
// 		wirelessRead(&data, PKTSTATUS, 0);
// 	}
// 	wirelessRead(&data, RXBYTES, 0);
	// Loop while bytes in RX fifo are less than the packet length
	
	while ((data < (uint8_t)SMARTRF_SETTING_PKTLEN + 2))  {
		//t++;
		// Read RXBYTES register
		wirelessRead(&data, RXBYTES, 0);
		// Mask
		data = data & 0x7F;								
	}	
	
	// Strobe IDLE
	wirelessSend(&garbage, SIDLE, 0);	
	osDelay(50);
	// State of the wireless transceiver, 01 = IDLE
	wirelessRead(&data, MARCSTATE, 0);
	
	if (data != 0x01) {
		while (1);
	}
	
	//for(i = 0; i < 5; i++)
	wirelessRead(packet, RXFIFO_BURST, WIRELESS_BUFFER_SIZE);	// read FIFO in burst
	
	// Check if the checksum test passed
// 	if (packet[4] & 0x80 == 0x80) {
// 	  // CRC passed
// 		data = 0x05;
// 	} else {
// 		while (1);
// 	}
		
	// Strobe SFRX - Flush the RX FIFO
	wirelessSend(&garbage, SFRX, 0);
	osDelay(50);
	
	// Read RX FIFOBYTES register to check if it has been flushed
	wirelessRead(&data, RXBYTES, 0);	// READ RXBYTES REG
	
	if ((data & 0x7F) != 0x00) {
		while (1);
	}
	
	// Check to see if tranceiver is in 01 = IDLE
	wirelessRead(&data, MARCSTATE, 0);
	if (data != 0x01) {
		while (1);
	}
}	

/**
  * @brief  Send packets wirelessly
  * @param  pitchBuffer - array holding values for the pitch angle
	*					rollBuffer 	- array holding values for the roll angle
	*					buffLength	- length of the two above arrays
  * @retval None
  */
void wirelessTX(uint8_t pitch, uint8_t roll) {
	uint8_t garbage = 0x00;
	uint32_t i;
	uint8_t data = 0x00;
	uint8_t TXbuffer[4] = {0x00, 0x00, 0x00, 0x00};
	
	// Strobe IDLE
	wirelessSend(&garbage, SIDLE, 0);
	osDelay(50);
	
	// State of the wireless transceiver, 01 = IDLE
	wirelessRead(&data, MARCSTATE, 0);
	if (data != 0x01) {
		while (1);
	}
	
	// Strobe SFTX - Flush the TX FIFO
	wirelessSend(&garbage, SFTX, 0);
	osDelay(50);
	
	// Read TX FIFOBYTES register to check if it has been flushed
	wirelessRead(&data, TXBYTES, 0);
	if ((data & 0x7F) != 0x00) {
		while (1);
	}
	
	TXbuffer[0] = 0x03;
	// Loop over the 2 sent buffers and store their values in their respective TXbuffer positions
//	for (i = 0; i < buffLength; i++) {	
	
	TXbuffer[1] = SMARTRF_SETTING_ADDR;
	TXbuffer[2] = pitch;
	TXbuffer[3] = roll;
	
	// Send the TXBuffer to the transceiver
	wirelessSend(TXbuffer, TXFIFO_BURST, 4);
	osDelay(50);
		
	// Check to see how many bytes have been put into the TX FIFO
	wirelessRead(&data, TXBYTES, 0);	
	//data = data & 0x7F;
	// If TXFIF_NUMBYTES = 3, then enable STX 
	if (data == 0x04) {
		wirelessRead(&data, MARCSTATE, 0);
		wirelessSend(&garbage, STX, 0);	
	}else
		while(1);
		// Wait for tranceiver to go back into IDLE
		//SPI_WAIT();
	osDelay(50);
	
	// Check to see if tranceiver is in 01 = IDLE
	wirelessRead(&data, MARCSTATE, 0);
	if (data != 0x01) {
		while (1);
	}
}
