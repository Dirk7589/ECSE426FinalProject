/**
*@file wireless.h
*@author Dirk Dubois, Alain Slak
*@date March 20th, 2013
*@brief A set of functions to interface with the ez430-CC2500
*
*/

#ifndef __WIRELESS_H
#define __WIRELESS_H

#define WIRELESS_CS_LOW()       			  	 GPIO_ResetBits(WIRELESS_CS_PORT, WIRELESS_CS_PIN)
#define WIRELESS_CS_HIGH()      					 GPIO_SetBits(WIRELESS_CS_PORT, WIRELESS_CS_PIN)

#define WIRELESS_FLAG_TIMEOUT         ((uint32_t)0x1000)

//defines
#define SMARTRF_SETTING_IOCFG2 0x29
#define SMARTRF_SETTING_IOCFG1 0x2E
#define SMARTRF_SETTING_IOCFG0 0x06
#define SMARTRF_SETTING_FIFOTHR 0x07
#define SMARTRF_SETTING_SYNC1 0xA3
#define SMARTRF_SETTING_SYNC0 0xD9
#define SMARTRF_SETTING_PKTLEN 0x04	//used to be A
#define SMARTRF_SETTING_PKTCTRL1 0x0E // Changed so that there is an appended status to the payload and address checking based on the ADDR value with broadcasting
#define SMARTRF_SETTING_PKTCTRL0 0x04 // CRC enabled and variable packet length enabled, so packet length must be defined as the first byte of the payload (payload length doesn't include this byte)
#define SMARTRF_SETTING_ADDR 0x10
#define SMARTRF_SETTING_CHANNR 0x08 //Set the channel
#define SMARTRF_SETTING_FSCTRL1 0x0C
#define SMARTRF_SETTING_FSCTRL0 0x00
#define SMARTRF_SETTING_FREQ2 0x5D
#define SMARTRF_SETTING_FREQ1 0x93
#define SMARTRF_SETTING_FREQ0 0xB1
#define SMARTRF_SETTING_MDMCFG4 0x0E
#define SMARTRF_SETTING_MDMCFG3 0x3B
#define SMARTRF_SETTING_MDMCFG2 0x73 
#define SMARTRF_SETTING_MDMCFG1 0x42
#define SMARTRF_SETTING_MDMCFG0 0xF8
#define SMARTRF_SETTING_DEVIATN 0x00 // ***
#define SMARTRF_SETTING_MCSM2 0x07
#define SMARTRF_SETTING_MCSM1 0x30
#define SMARTRF_SETTING_MCSM0 0x18
#define SMARTRF_SETTING_FOCCFG 0x1D
#define SMARTRF_SETTING_BSCFG 0x1C
#define SMARTRF_SETTING_AGCCTRL2 0xC7
#define SMARTRF_SETTING_AGCCTRL1 0x00 // ***
#define SMARTRF_SETTING_AGCCTRL0 0xB0
#define SMARTRF_SETTING_WOREVT1 0x87
#define SMARTRF_SETTING_WOREVT0 0x6B
#define SMARTRF_SETTING_WORCTRL 0xF8
#define SMARTRF_SETTING_FREND1 0xB6
#define SMARTRF_SETTING_FREND0 0x10
#define SMARTRF_SETTING_FSCAL3 0xEA
#define SMARTRF_SETTING_FSCAL2 0x0A
#define SMARTRF_SETTING_FSCAL1 0x00
#define SMARTRF_SETTING_FSCAL0 0x19
#define SMARTRF_SETTING_RCCTRL1 0x41
#define SMARTRF_SETTING_RCCTRL0 0x00
#define SMARTRF_SETTING_FSTEST 0x59
#define SMARTRF_SETTING_PTEST 0x7F
#define SMARTRF_SETTING_AGCTEST 0x3F
#define SMARTRF_SETTING_TEST2 0x88
#define SMARTRF_SETTING_TEST1 0x31
#define SMARTRF_SETTING_TEST0 0x0B

/*Status registers*/
#define PARTNUM 0x30
#define VERSION 0x31
#define MARCSTATE 0x35 /**<Main Radio Control State Machine State*/
#define TXBYTES 0x3A /**<Underflow and number of bytes in the TX FIFO (needs to be ORed with MULTIPLEBYTE_RD)*/
#define RXBYTES 0x3B /**<Overflow and number of bytes in the RX FIFO (needs to be ORed with MULTIPLEBYTE_RD)*/
#define PKTSTATUS 0x38 /**<Current GDOx Status and Packet Status*/

#define RXFIFO_SINGLE 0xBF
#define RXFIFO_BURST 0xFF
#define TXFIFO_SINGLE 0x3F
#define TXFIFO_BURST 0x7F

#define RX_OVERFLOW 0x6F /**<Status value if an overflow has occured*/
#define RX_RDY 0x1F

#define TX_RDY 0x2F
#define TX_UNDERFLOW 0x7F

#define WIRELESS_MASK 0x70
#define WIRELESS_IDLE 0x00
#define WIRELESS_RX 0x10
#define WIRELESS_TX 0x20

/*Command strobes*/
#define SRES 0x30 /**<Reset chip*/
#define SNOP 0x3D /**<Get the chip status*/
#define SFXTXON 0x31 /**<Enable and calibrate frequency synthesizer*/
#define SXOFF 0x32 /**<Turn off crystal oscillator*/
#define SCAL 0x33 /**<Calibrate frequency synthesizer and turn it off*/
#define SRX 0x34 /**<Enter receive mode*/
#define STX 0x35 /**<Enter transmit mode*/
#define SIDLE 0x36 /**<Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable*/
#define SWOR 0x38 /**<Start automatic RX polling sequence*/
#define SPWD 0x39 /**<Enter power down mode when CSn goes high*/
#define SFRX 0x3A /**<The strobe command to flush the RX FIFO*/
#define SFTX 0x3B /**<The strobe command to flush the TX FIFO*/

/*Wireless SPI defines*/
#define WIRELESS_BUFFER_SIZE 7
#define WIRELESS_BUFFER_INIT_SIZE 48
#define WIRELESS_CS_PIN GPIO_Pin_15 /**<Select pin4 for wireless SPI1 CSn*/
#define WIRELESS_CS_PORT GPIOA /**<Select portA for wireless SPI CSn*/

/*Wireless Command defines*/
#define SINGLEBYTE_WR 0x00
#define MULTIPLEBYTE_WR 0x40
#define SINGLEBYTE_RD 0x80
#define MULTIPLEBYTE_RD 0xC0


/**
*@brief A function to setup and initialize wireless communication
*@param[inout] None
*@retval None
*/
void initWireless(void);

/**
*@brief A function to write to a specified register on the wireless board
*@param[in] pBuffer The data to be sent to the device
*@param[in] writeAddr The address of the register to send data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessSend(uint8_t* pBuffer, uint8_t writeAddr, uint16_t numOfBytes);

/**
*@brief A function that writes a byte for the Wireless module over SPI
*@param[in] data The data to be written to the wireless device
*@retval uint8_t
*/
uint8_t wirelessSendByte(uint8_t data);

/**
*@brief A function to read from a specified resgister on the wireless board
*@param[inout] pBuffer The data to be read to the device
*@param[in] readAddr The address of the register to read data to
*@param[in] numOfBytes The number of bytes to be sent
*@retval None
*/
void wirelessRead(uint8_t* pBuffer, uint8_t readAddr, uint16_t numOfBytes);

/**
  * @brief  Send packets wirelessly
  * @param  pitchBuffer - array holding values for the pitch angle
	*					rollBuffer 	- array holding values for the roll angle
	*					buffLength	- length of the two above arrays
  * @retval None
  */

void wirelessTX(uint8_t pitch, uint8_t roll);

/**
  * @brief  Receive packets wirelessly
  * @param  pitchBuffer - array holding values for the pitch angle
	*					rollBuffer 	- array holding values for the roll angle
	*					buffLength	- length of the two above arrays
	*					packet			- received packet, returned by reference
  * @retval None
  */
void wirelessRX(uint8_t* packet);

#endif
