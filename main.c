 /**
*@file main.c
*@author Dirk Dubois, Alain Slak
*@date February 21th, 2013
*@brief 
*
*/

/*Includes*/
#include <stdint.h>
#include <arm_math.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "init.h"
#include "initACC.h"
#include "moving_average.h"
#include "temp.h"
#include "access.h"
#include "common.h"
#include "wireless.h"
#include "spi.h"

/*Defines */
#define DEBUG 0
#define TRANSMIT_WIRELESS 0
#define USER_BTN 0x0001 /*!<Defines the bit location of the user button*/
#define THRESHOLD_ANGLE 10


/*Global Variables*/
uint8_t sampleACCFlag = 0x01; /**<A flag variable for sampling, restricted to a value of 0 or 1*/
uint8_t buttonState = 1; /**<A variable that represents the current state of the button*/
uint8_t dmaFlag = 0x02; /**<A flag variable that represent the DMA flag*/
uint8_t wirelessFlag = 0x04; /**<A flag variable that represents the wireless flag*/
uint8_t wirelessRdy = 0x08; 
//uint8_t dmaInitFlag = 0;
uint8_t LEDState = 0; //Led state variable
uint8_t orientationMatch = 0;
uint8_t LEDCounter = 0;

uint8_t tx[7] = {0x29|0x40|0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Transmission buffer for ACC for DMA*/
uint8_t rx[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Receive buffer for ACC for DMA*/

uint8_t const* txptr = &tx[0];
uint8_t* rxptr = &rx[0];
int8_t wirelessAngles[2] = {0,0};

int8_t swapped = 0;

//Declare global variables externed in common.h
int8_t txWireless[WIRELESS_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Transmission buffer for Wireless for DMA*/
int8_t rxWireless[WIRELESS_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Receive buffer for Wireless for DMA*/

uint8_t wirelessRx[WIRELESS_BUFFER_SIZE];

uint8_t txWirelessInit[WIRELESS_BUFFER_INIT_SIZE] = {0x00|MULTIPLEBYTE_WR, SMARTRF_SETTING_IOCFG2, SMARTRF_SETTING_IOCFG1,
			SMARTRF_SETTING_IOCFG0, SMARTRF_SETTING_FIFOTHR, SMARTRF_SETTING_SYNC1, SMARTRF_SETTING_SYNC0,
			SMARTRF_SETTING_PKTLEN,SMARTRF_SETTING_PKTCTRL1, SMARTRF_SETTING_PKTCTRL0, SMARTRF_SETTING_ADDR,
			SMARTRF_SETTING_CHANNR, SMARTRF_SETTING_FSCTRL1, SMARTRF_SETTING_FSCTRL0, SMARTRF_SETTING_FREQ2,
			SMARTRF_SETTING_FREQ1, SMARTRF_SETTING_FREQ0, SMARTRF_SETTING_MDMCFG4, SMARTRF_SETTING_MDMCFG3,
			SMARTRF_SETTING_MDMCFG2,SMARTRF_SETTING_MDMCFG1, SMARTRF_SETTING_MDMCFG0, SMARTRF_SETTING_DEVIATN,
			SMARTRF_SETTING_MCSM2,SMARTRF_SETTING_MCSM1, SMARTRF_SETTING_MCSM0,SMARTRF_SETTING_FOCCFG, SMARTRF_SETTING_BSCFG,
			SMARTRF_SETTING_AGCCTRL2, SMARTRF_SETTING_AGCCTRL1, SMARTRF_SETTING_AGCCTRL0, SMARTRF_SETTING_WOREVT1, 
			SMARTRF_SETTING_WOREVT0,SMARTRF_SETTING_WORCTRL, SMARTRF_SETTING_FREND1,SMARTRF_SETTING_FREND0, SMARTRF_SETTING_FSCAL3,
			SMARTRF_SETTING_FSCAL2,SMARTRF_SETTING_FSCAL1, SMARTRF_SETTING_FSCAL0, SMARTRF_SETTING_RCCTRL1, SMARTRF_SETTING_RCCTRL0,
			SMARTRF_SETTING_FSTEST, SMARTRF_SETTING_PTEST, SMARTRF_SETTING_AGCTEST, SMARTRF_SETTING_TEST2, SMARTRF_SETTING_TEST1, 
			SMARTRF_SETTING_TEST0}; /**<Transmission buffer for Wireless for DMA*/

uint8_t rxWirelessInit[WIRELESS_BUFFER_INIT_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
																										 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
																										 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t strobeCommand[1] = {0x00};
uint8_t status[1] = {0x00};

float accCorrectedValues[3];
float wirelessAccValues[3] = {0,0,0};
float angles[2];
int32_t accValues[3];

uint8_t dmaFromAccFlag = 0; /**<A flag variable that represents whether or not DMA was called from the accelerometer thread*/
uint8_t dmaFromWirelessFlag = 0; /**<A flag variable that represents whether or not DMA was called from the wireless thread*/

//Define semaphores for global variable externed in common.h
osSemaphoreDef(accCorrectedValues)
osSemaphoreId accId;

osSemaphoreDef(wirelessAngles)
osSemaphoreId wirelessAccId;

osSemaphoreDef(txWireless)
osSemaphoreId txId;

osSemaphoreDef(rxWireless)
osSemaphoreId rxId;

//Define Mutexes

osMutexDef(dmaMutex)
osMutexId dmaId;
/*Function Prototypes*/

/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void);

/**
*@brief A function that flashes the LEDs if the pitch and roll positions are the same
*@retval None
*/
void displayPitchRoll(void);

/*!
 @brief Thread to perform the accelerometer data processing
 @param argument Unused
 */
void accelerometerThread(void const * argument);
void wirelessThread(void const * argument);

//Thread structure for above thread
osThreadDef(accelerometerThread, osPriorityNormal, 1, 0);
osThreadDef(wirelessThread, osPriorityNormal, 1, 0);

osThreadId aThread; //Accelerometer thread ID
osThreadId wThread; //Wireless thread ID


/**
*@brief The main function that creates the processing threads and displays the UI
*@retval An int
*/
int main (void) {	

	#if !DEBUG
	//Create necessary semaphores
	accId = osSemaphoreCreate(osSemaphore(accCorrectedValues), 1);
	//Create mutex
	dmaId = osMutexCreate(osMutex(dmaMutex));
	
	initIO(); //Enable LEDs and button
	initTim3(); //Enable Tim3 at 100Hz
	initACC(); //Enable the accelerometer
	initDMA(); //Enable DMA for the accelerometer
	initEXTIButton(); //Enable button interrupts via exti1
	initSPI(); //Enable SPI for wireless
	initWireless(); //Configure the wireless module
	wirelessRead(rxWirelessInit,0x00,WIRELESS_BUFFER_INIT_SIZE);
	// Start threads
	
	aThread = osThreadCreate(osThread(accelerometerThread), NULL);
	wThread = osThreadCreate(osThread(wirelessThread), NULL);

	displayUI(); //Main display function
	
	#else
	
	initIO(); //Enable LEDs and button
// 	initTim3(); //Enable Tim3 at 100Hz
// 	initACC(); //Enable the accelerometer
// 	initDMA(); //Enable DMA for the accelerometer
// 	initEXTIButton(); //Enable button interrupts via exti1
	initSPI(); //Enable SPI for wireless
	initWireless(); //Configure the wireless module
	
// 	aThread = osThreadCreate(osThread(accelerometerThread), NULL);
// 	wThread = osThreadCreate(osThread(wirelessThread2), NULL);

// 	displayUI(); //Main display function
	
	uint8_t packet[WIRELESS_BUFFER_SIZE] = {0,0,0,0,0,0};
	uint8_t pitch = 101;
	uint8_t roll = 43;
	
	wirelessRead(rxWirelessInit,0x00,WIRELESS_BUFFER_INIT_SIZE);
	
	#if TRANSMIT_WIRELESS
	
	int counter = 0;
	while(1) {
		osDelay(10);
		counter = (counter + 1) % 1000;
		if(counter == 0)
		{
			pitch = (pitch + 1) % 180;
			roll = (roll + 1) % 180;
		}
		wirelessTX(pitch, roll);
	}
	
	#else
	
	while(1) {
		osDelay(10);
		wirelessRX(packet);
		//packet[0] = 0;
		//packet[1] = 0;
		//packet[2] = 0;
	}
	
	#endif
	
	
	
	
	#endif
}
	

void accelerometerThread(void const * argument){
  uint8_t i = 0; //Counting variables
	
	//Create structures for moving average filter
	AVERAGE_DATA_TYPEDEF dataX;
	AVERAGE_DATA_TYPEDEF dataY;
	AVERAGE_DATA_TYPEDEF dataZ;
	
	//Intialize structures for moving average filter
	movingAverageInit(&dataX);
	movingAverageInit(&dataY);
	movingAverageInit(&dataZ);
	
	//Real-time processing of data
	while(1){
		
		osSignalWait(sampleACCFlag, osWaitForever ); //Wait to sample
		osSemaphoreWait(accId, osWaitForever); //Have exclusive access to accelerometer values
		
		SPI_DMA_Transfer(rx, tx, 7, GPIOE, (uint16_t)0x0008); //Start transfer for the LIS302DL
		
		osSignalWait(dmaFlag, osWaitForever); //Wait for DMA to finish
		
		int32_t* out = &accValues[0];
		//Scale the values from DMA to the actual values
		for(i=0; i<0x03; i++)
		{
			*out =(int32_t)(18 *  (int8_t)rx[2*i +1]); //Copy out of rx buffer
			out++;
		}		

		osMutexRelease(dmaId);//Clear Mutex
		
		//Filter ACC values
		calibrateACC(accValues, accCorrectedValues); //Calibrate the accelerometer	
		
		accCorrectedValues[0] = movingAverage(accCorrectedValues[0], &dataX);
		accCorrectedValues[1] = movingAverage(accCorrectedValues[1], &dataY);
		accCorrectedValues[2] = movingAverage(accCorrectedValues[2], &dataZ);
		
		osSemaphoreRelease(accId); //Release exclusive access
		osSignalSet(wThread, wirelessFlag); //Set wireless signal
	}
}

// void wirelessThread2(void const * argument)
// {
// 	float tempACCValues[3];
// 	float anglesTemp[2];
// 	int8_t pitch;
// 	int8_t roll;
// 	uint8_t packet[WIRELESS_BUFFER_SIZE] = {0,0,0,0,0,0};

// 	wirelessRead(rxWirelessInit,0x00,WIRELESS_BUFFER_INIT_SIZE);
// 	
// 	while(1){
// 		osSignalWait(wirelessFlag, osWaitForever);
// 		
// 		switch(buttonState){
// 			case 0:
// 				osDelay(10);
// 			
// 				getACCValues(tempACCValues); //Get current ACC values
// 				
// 				toAngle(tempACCValues, anglesTemp); //Convert to pitch and roll to send
// 				//Cast to signed integer to transmit
//  				pitch = (int8_t)anglesTemp[0];			
//  				roll = (int8_t)anglesTemp[1];			
// 			
// 				wirelessTX(pitch, roll);
// 			case 1:
// 				
// 				osDelay(10);
// 				wirelessRX(packet);
// 			
// 				osSemaphoreWait(wirelessAccId, osWaitForever);
// 				wirelessAngles[0] = packet[3];			//Roll
// 				osSemaphoreRelease(wirelessAccId);
// 		
// 				osSemaphoreWait(wirelessAccId, osWaitForever);
// 				wirelessAngles[1] = packet[2];			//Pitch
// 				osSemaphoreRelease(wirelessAccId);
// 		}
// 	}
// }


void wirelessThread(void const * argument){
	
	float tempACCValues[3];
	float anglesTemp[2];
	int8_t anglesTransmit[2];
	//uint8_t receive = SRX|SINGLEBYTE_WR;
	
	while(1) {
		osSignalWait(wirelessFlag, osWaitForever);
		
		strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
		SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
		osSignalWait(dmaFlag, osWaitForever);
		osMutexRelease(dmaId);
	
		while((status[0] & WIRELESS_MASK) != WIRELESS_IDLE){
			strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
			SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
			osSignalWait(dmaFlag, osWaitForever);
			osMutexRelease(dmaId);
		}
		
		switch(buttonState) {
			// RX mode
			case 0:
				//Flush RX FIFO
				strobeCommand[0] = SFRX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				//Strobe RX
				strobeCommand[0] = SRX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				//Wait for IDLE state
				strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
			
				while((status[0] & WIRELESS_MASK) != WIRELESS_IDLE){
					strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
					osSignalWait(dmaFlag, osWaitForever);
					osMutexRelease(dmaId);
				}
				
				//Read RX FIFO
				txWireless[0] = RXFIFO_BURST;
				SPI_DMA_Transfer(rxWireless, txWireless, WIRELESS_BUFFER_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				if((int8_t)rxWireless[3] >= -90 && (int8_t)rxWireless[3] <= 90)
				{
						osSemaphoreWait(wirelessAccId, osWaitForever);
						wirelessAngles[0] = rxWireless[3];
						osSemaphoreRelease(wirelessAccId);
				}
				
				if((int8_t)rxWireless[4] >= -90 && (int8_t)rxWireless[4] <= 90)
				{
						osSemaphoreWait(wirelessAccId, osWaitForever);
						wirelessAngles[1] = rxWireless[4];
						osSemaphoreRelease(wirelessAccId);
				}
				
				//Flush RX FIFO
				strobeCommand[0] = SFRX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				break;
			// TX mode
			case 1:
				//Flush TX FIFO
				strobeCommand[0] = SFTX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				//Wait for IDLE state
				strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
			
				while((status[0] & WIRELESS_MASK) != WIRELESS_IDLE){
					strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
					osSignalWait(dmaFlag, osWaitForever);
					osMutexRelease(dmaId);
				}
				
				//Get current ACC values
				getACCValues(tempACCValues); //Get current ACC values
				
				toAngle(tempACCValues, anglesTemp); //Convert to pitch and roll to send
				//Cast to signed integer to transmit
 				anglesTransmit[0] = (int8_t)anglesTemp[0];
 				anglesTransmit[1] = (int8_t)anglesTemp[1];
				
				//Prepare TX buffer to transmit
				txWireless[0] = TXFIFO_BURST;
				txWireless[1] = SMARTRF_SETTING_PKTLEN;
				txWireless[2] = SMARTRF_SETTING_ADDR;
				txWireless[3] = anglesTransmit[0];
				txWireless[4] = anglesTransmit[1];
				
				//Load TX FIFO
				SPI_DMA_Transfer(rxWireless, txWireless, 5, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				//Strobe TX
				strobeCommand[0] = STX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				
				//Wait until goes back to idle
				strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
			
				while((status[0] & WIRELESS_MASK) != WIRELESS_IDLE){
					strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
					osSignalWait(dmaFlag, osWaitForever);
					osMutexRelease(dmaId);
				}
				
				//Strobe clear TX FIFO
				strobeCommand[0] = SFTX|SINGLEBYTE_WR; //Set for receive mode
				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
				osSignalWait(dmaFlag, osWaitForever);
				osMutexRelease(dmaId);
				break;
			default:
				break;
			}
		}
		
	}
	
	
	
	
	
	
	
	
	
// 	while(1){
// 		osSignalWait(wirelessFlag, osWaitForever);
// 		
// 		if(swapped == 0)
// 		{
// 			if(buttonState == 0)
// 			{
// 				strobeCommand[0] = SFRX;
// 				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 				
// 				strobeCommand[0] = SIDLE|SINGLEBYTE_WR; //Set for receive mode
// 				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 				
// 				strobeCommand[0] = SRX|SINGLEBYTE_WR; //Set for receive mode
// 				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);				
// 			}
// 			else
// 			{
// 				strobeCommand[0] = SIDLE|SINGLEBYTE_WR; //Set for receive mode
// 				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 			}
// 			swapped = 1 - swapped;
// 		}
// 			
// 			
// 		switch(buttonState){
// 			case 0:
// 				strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
// 				SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 			
// 				while((status[0] | 0x70) != 0x00){
// 					strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId);
// 					
// // 					if(swapped == 0)
// // 					{
// // 						break;
// // 					}
// 				}
// 				
// // 				if(swapped == 0)
// // 				{
// // 					break;
// // 				}
// 				
// 				txWireless[0] = RXFIFO_BURST;
// 						
// 				SPI_DMA_Transfer(rxWireless, txWireless, WIRELESS_BUFFER_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 				
// 				if((int8_t)rxWireless[1] >= -90 && (int8_t)rxWireless[1] <= 90)
// 				{
// 						osSemaphoreWait(wirelessAccId, osWaitForever);
// 						wirelessAngles[0] = rxWireless[1];
// 						osSemaphoreRelease(wirelessAccId);
// 				}
// 				
// 				if((int8_t)rxWireless[2] >= -90 && (int8_t)rxWireless[2] <= 90)
// 				{
// 						osSemaphoreWait(wirelessAccId, osWaitForever);
// 						wirelessAngles[1] = rxWireless[2];
// 						osSemaphoreRelease(wirelessAccId);
// 				}
// 				
// 				if((rxWireless[0] | 0x0F) == RX_OVERFLOW){
// 					strobeCommand[0] = SFRX;
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId);
// 				}
// 				
// 				//Receive State
// 				//Check if we are ready to receive
// 				if((rxWireless[0] |0x0F) != RX_RDY){
// 					strobeCommand[0] = SRX|SINGLEBYTE_WR; //Set for receive mode
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId); 
// 				
// 				break;
// 			case 1:
// 				
// 				while((status[0] | 0x0F) != 0x0F){
// 					strobeCommand[0] = SNOP|SINGLEBYTE_WR; //Set for receive mode
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId);
// 					
// 					if(swapped == 0)
// 					{
// 						break;
// 					}
// 				}
// 				
// 				if(swapped == 0)
// 				{
// 					break;
// 				}
// 				
// 				txWireless[0] = TXFIFO_BURST;
// 				
// 				getACCValues(tempACCValues); //Get current ACC values
// 				
// 				toAngle(tempACCValues, anglesTemp); //Convert to pitch and roll to send
// 				//Cast to signed integer to transmit
//  				anglesTransmit[0] = (int8_t)anglesTemp[0];
//  				anglesTransmit[1] = (int8_t)anglesTemp[1];
// 			
// 				//Pass to transmit buffer
// 				txWireless[1] = anglesTransmit[0];
// 				txWireless[2] = anglesTransmit[1];
// 			
// 				SPI_DMA_Transfer(rxWireless, txWireless, WIRELESS_BUFFER_SIZE, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 				osSignalWait(dmaFlag, osWaitForever);
// 				osMutexRelease(dmaId);
// 				
// 				if((rxWireless[0] | 0x0F) == TX_UNDERFLOW){
// 					strobeCommand[0] = SFTX;
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId);
// 				}
// 			
// 				//Transmit mode
// 				if((rxWireless[0] |0x0F) != TX_RDY){
// 					strobeCommand[0] = STX|SINGLEBYTE_WR; //Setup transmit mode
// 					SPI_DMA_Transfer(status, strobeCommand, 1, WIRELESS_CS_PORT, WIRELESS_CS_PIN);
// 					osSignalWait(dmaFlag, osWaitForever);
// 					osMutexRelease(dmaId);
// 				}
// 			
// 				break;
// 			default:
// 				break;
// 		}
// 	}
// }
/**
*@brief A function that runs the display user interface
*@retval None
*/
void displayUI(void){
	while(1){
		displayPitchRoll();
	}
}

void displayPitchRoll(){
	float acceleration[3];
	//float wirelessAcceleration[3];
	float localAngles[2];
	int8_t remoteAngles[2];
	float rollAngleDiff;
	float pitchAngleDiff;
	
	switch(buttonState){
			case 0: //receive
				
				//get the accelerometer readings of both boards for comparison
				getACCValues(acceleration);
// 				getWirelessACCValues(wirelessAcceleration);
				getWirelessAngles(remoteAngles);
			
				//get the pitch and roll for comparison
				toAngle(acceleration, localAngles);
// 				toAngle(wirelessAcceleration, remoteAngles);
			
				//flash the LEDs if the boards are within a certain threshold of eachother on each axis
				if((localAngles[0] - remoteAngles[0] < THRESHOLD_ANGLE) && (localAngles[0] - remoteAngles[0] > -THRESHOLD_ANGLE) && (localAngles[1] - remoteAngles[1] < THRESHOLD_ANGLE) && (localAngles[1] - remoteAngles[1] > -THRESHOLD_ANGLE)){
						orientationMatch = 1;
				}
				else{
					GPIOD->BSRRH = ORANGE_LED;
 					GPIOD->BSRRH = GREEN_LED;
 					GPIOD->BSRRH = RED_LED;
 					GPIOD->BSRRH = BLUE_LED;
				}
// 				else{
// 					//calculate the difference between the two board's respective pitch and roll
// 					rollAngleDiff = remoteAngles[0] - localAngles[0];
// 					pitchAngleDiff = remoteAngles[1] - localAngles[1];
// 					
// 					GPIOD->BSRRH = ORANGE_LED;
// 					GPIOD->BSRRH = GREEN_LED;
// 					GPIOD->BSRRH = RED_LED;
// 					GPIOD->BSRRH = BLUE_LED;
// 					
// 					if(rollAngleDiff > THRESHOLD_ANGLE){
// 						GPIOD->BSRRH = ORANGE_LED;
// 						GPIOD->BSRRH = GREEN_LED;
// 						GPIOD->BSRRH = RED_LED;
// 						GPIOD->BSRRL = BLUE_LED;
// 					}
// 					else if(rollAngleDiff < -THRESHOLD_ANGLE){
// 						GPIOD->BSRRH = BLUE_LED;
// 						GPIOD->BSRRH = GREEN_LED;
// 						GPIOD->BSRRH = RED_LED;
// 						GPIOD->BSRRL = ORANGE_LED;
// 					}
// 					
// 					if(pitchAngleDiff > THRESHOLD_ANGLE){
// 						GPIOD->BSRRH = GREEN_LED;
// 						GPIOD->BSRRL = RED_LED;
// 					}
// 					else if(pitchAngleDiff < -THRESHOLD_ANGLE){
// 						GPIOD->BSRRH = RED_LED;
// 						GPIOD->BSRRL = GREEN_LED;
// 					}
// 				}
			break;
		
 			case 1: //transmit
				getACCValues(acceleration);
				displayDominantAngle(acceleration);
			break;
		}
	
}

/**
*@brief An interrupt handler for EXTI0
*@retval None
*/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		buttonState = 1 - buttonState;	//Change the current tap state
		EXTI_ClearITPendingBit(EXTI_Line0);	//Clear the EXTI0 interrupt flag
		//swapped = 1 - swapped;
	}
}

/**
*@brief An interrupt handler for Tim3
*@retval None
*/
void TIM3_IRQHandler(void)
{
	osSignalSet(aThread, sampleACCFlag);
	LEDCounter++;
	if(LEDCounter == 25){
		LEDCounter = 0;
		
		if(orientationMatch == 1){
			orientationMatch = 0;
			LEDState = LEDToggle(LEDState);
		}
	}
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //Clear the TIM3 interupt bit
}

/**
*@brief An interrupt handler for DMA2_Stream0
*@retval None
*/
void DMA2_Stream0_IRQHandler(void)
{
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0); //Clear the flag for transfer complete
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);

	//Disable DMA
	DMA_Cmd(DMA2_Stream0, DISABLE); // RX
	DMA_Cmd(DMA2_Stream3, DISABLE); // TX
	
	if(dmaFromAccFlag){
		dmaFromAccFlag = 0;
		GPIO_SetBits(GPIOE, (uint16_t)0x0008);  //Raise CS Line for LIS302DL
		osSignalSet(aThread, dmaFlag);				//Set flag for accelerometer sampling
	}
	if(dmaFromWirelessFlag){
		dmaFromWirelessFlag = 0;
		GPIO_SetBits(WIRELESS_CS_PORT, (uint16_t)WIRELESS_CS_PIN); //Raise CS Line for Wireless
		osSignalSet(wThread, dmaFlag);
	}
}
