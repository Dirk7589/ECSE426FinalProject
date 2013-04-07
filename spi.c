/**
*@file spi.c
*@author Dirk Dubois, Alain Slak
*@date March 22nd, 2013
*@brief A c file that contains the SPI generic drivers
*/

/*Includes*/
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "spi.h"
#include "common.h"

void initSPI(void){
	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOA, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure GPIO PIN for Wireless Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_Cmd(SPI1, DISABLE);
  /* SPI configuration -------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
}

/**
*@brief A function that starts a DMA transfer using the buffers provided, on SPI1 using the passed chip select port
*@param[inout] rx A pointer to the receive buffer in Bytes
*@param[in] tx A pointer to the transmit buffer in Bytes
*@param[in] bufferSize The size of the rx and tx buffer
*@param[in] csPort The GPIO port that has the CS line for SPI1
*@param[in] csPin The GPIO pin that the CS line is connected to for SPI1
*@retval None
*/
void SPI_DMA_Transfer(const uint8_t* rx, const uint8_t* tx, const uint8_t bufferSize, GPIO_TypeDef* csPort, uint8_t csPin){
	osMutexWait(dmaId, osWaitForever);//Check that DMA is avaible using mutex

	if((csPort == GPIOE) && (csPin == 0x0008)){
		dmaFromAccFlag = 1; 
	}
	
	if((csPort == WIRELESS_CS_PORT) && (csPin == WIRELESS_CS_PIN)){
		dmaFromWirelessFlag = 1;
	}
	
	//Configure DMA
	DMA2_Stream0->NDTR = bufferSize;
	DMA2_Stream0->M0AR = (uint32_t)rx;

	DMA2_Stream3->NDTR = bufferSize;
	DMA2_Stream3->M0AR = (uint32_t)tx;

	GPIO_ResetBits(csPort, csPin);	//lower CS line

	//Enable DMA
	DMA_Cmd(DMA2_Stream0, ENABLE); // RX
	DMA_Cmd(DMA2_Stream3, ENABLE); // TX
	
}
