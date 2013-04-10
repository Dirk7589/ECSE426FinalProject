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
#include "lcd.h"
#include "keypad.h"
#include "dac.h"
#include "isr.h"
#include "stm32f4_discovery_audio_codec.h"
#include "waveplayer.h"
#include "stm32f4_discovery.h"

/*Defines for compilation*/
#define DEBUG 1
#define TRANSMITTER 0
#define USE_LED_UI 1

/*Defines for variables*/
#define TRANSMIT_WIRELESS 0
#define USER_BTN 0x0001 /*!<Defines the bit location of the user button*/
#define THRESHOLD_ANGLE 10



/*Global Variables*/

//ISR States and Flags
uint8_t buttonState = 1; 
uint8_t tapState = 0; 
uint8_t remoteTapState = 0; 
uint8_t volumeBtnUp = 0;
uint8_t volumeBtnDown = 0;
uint8_t volumeBtnUpFlag = 0;
uint8_t volumeBtnDownFlag = 0;
uint8_t volumeCounterUp = 0;
uint8_t volumeCounterDown = 0;

//OS Signal Masks
uint8_t sampleACCFlag = 0x01; /**<A flag variable for sampling, restricted to a value of 0 or 1*/
uint8_t readKeypadFlag = 0x01; /**<A flag variable that represents the keypad poll*/
uint8_t displayLCDFlag = 0x01; /**<A flag variable that represents the display poll*/
uint8_t wirelessFlag = 0x04; /**<A flag variable that represents the wireless flag*/
uint8_t dmaFlag = 0x02; /**<A flag variable that represent the DMA flag*/

/*********DMA Variables********/
uint8_t tx[7] = {0x29|0x40|0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Transmission buffer for ACC for DMA*/
uint8_t rx[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /**<Receive buffer for ACC for DMA*/

uint8_t const* txptr = &tx[0];
uint8_t* rxptr = &rx[0];
int8_t wirelessAngles[2] = {0,0};


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

//Other variables
float accCorrectedValues[3];
float wirelessAccValues[3] = {0,0,0};
float angles[2];
int32_t accValues[3];
char key = 'E';
uint8_t LEDState = 0; //Led state variable
uint8_t orientationMatch = 0;
uint8_t LEDCounter = 0;
uint8_t volumeUpBtnPressed = 0;
uint8_t volumeDownBtnPressed = 0;
uint8_t audioVolume = 0; // Volume initially muted


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

osSemaphoreDef(audioVolume)
osSemaphoreId vlmId;

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

/**
*@brief A function that controls the volume of the audio device
*@retval None
*/
void volumeControl(void);

/******Thread Prototypes*************/
void accelerometerThread(void const * argument);
void wirelessThread(void const * argument);
void keypadThread(void const * argument);
void lcdThread(void const * argument);

//Thread structure for above thread
osThreadDef(accelerometerThread, osPriorityNormal, 1, 0);
osThreadDef(wirelessThread, osPriorityNormal, 1, 0);
osThreadDef(keypadThread, osPriorityNormal, 1, 0);
osThreadDef(lcdThread, osPriorityNormal, 1, 0);

osThreadId aThread; //Accelerometer thread ID
osThreadId wThread; //Wireless thread ID
osThreadId kThread; //Keypad thread ID
osThreadId lThread; //LCD thread ID


/**
*@brief The main function that creates the processing threads and displays the UI
*@retval An int
*/
int main (void) {	

	//Os semaphore, thread, and mutex creation
	accId = osSemaphoreCreate(osSemaphore(accCorrectedValues), 1); 	//Create necessary semaphores
	dmaId = osMutexCreate(osMutex(dmaMutex)); 	//Create mutex
	
	//Initialization functions
	initIO(); //Enable LEDs and button
	initTim3(); //Enable Tim3 at 100Hz
	initACC(); //Enable the accelerometer
	initDMA(); //Enable DMA for the accelerometer
	initEXTIButton(); //Enable button interrupts via exti0
	
	#if TRANSMITTER
		initEXTIACC();	//Enable tap interrupts via exti1
	#endif
	
	initSPI(); //Enable SPI for wireless
	initWireless(); //Configure the wireless module
	
	#if !TRANSMITTER
		lcd_init(MODE_8_BIT); 
		keypadInit();
	#endif
	
	#if DEBUG
// 		lcd_puts("I'm a stupid LCD, no... really...");
// 		osDelay(2000);
// 		lcd_goto(40);
// 		lcd_puts("Secretly, I'm a");
// 		osDelay(2000);
// 		lcd_clear();
// 		while(1);
// 		char key = 'e';
		WavePlayBack(I2S_AudioFreq_48k);
		while(1){
// 			key = keypadRead();
// 			if (key != 'E'){
// 				lcd_clear();
// 				lcd_write(key);
// 			}
		}

	#endif
		
	// Start threads
	#if !TRANSMITTER
	kThread = osThreadCreate(osThread(keypadThread), NULL);
	lThread = osThreadCreate(osThread(lcdThread), NULL);
	#endif
	aThread = osThreadCreate(osThread(accelerometerThread), NULL);
	wThread = osThreadCreate(osThread(wirelessThread), NULL);

	#if USE_LED_UI
	displayUI(); //Main display function
	#else
	volumeControl();	
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
				
				if((int8_t)rxWireless[2] >= -90 && (int8_t)rxWireless[2] <= 90)
				{
						osSemaphoreWait(wirelessAccId, osWaitForever);
						wirelessAngles[0] = rxWireless[2];
						osSemaphoreRelease(wirelessAccId);
				}
				
				if((int8_t)rxWireless[3] >= -90 && (int8_t)rxWireless[3] <= 90)
				{
						osSemaphoreWait(wirelessAccId, osWaitForever);
						wirelessAngles[1] = rxWireless[3];
						osSemaphoreRelease(wirelessAccId);
				}
				
				remoteTapState = rxWireless[4];
				
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
				
// 				//Prepare TX buffer to transmit
// 				txWireless[0] = TXFIFO_BURST;
// 				txWireless[1] = SMARTRF_SETTING_PKTLEN;
// 				txWireless[2] = SMARTRF_SETTING_ADDR;
// 				txWireless[3] = anglesTransmit[0];
// 				txWireless[4] = anglesTransmit[1];
				
				//Prepare TX buffer to transmit
				txWireless[0] = TXFIFO_BURST;
				txWireless[1] = SMARTRF_SETTING_ADDR;
				txWireless[2] = anglesTransmit[0];
				txWireless[3] = anglesTransmit[1];
				txWireless[4] = tapState;
				
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

void keypadThread(void const * argument){
	while(1){
		osSignalWait(readKeypadFlag, osWaitForever);
		key = keypadRead();
	}
}

void lcdThread(void const * argument){
	while(1){
		osSignalWait(displayLCDFlag, osWaitForever);	
		lcd_write(key);
	}
}

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
	
	switch(buttonState){
			case 0: //receive
				
				//get the accelerometer readings of both boards for comparison
				getACCValues(acceleration);
				getWirelessAngles(remoteAngles);
			
				//get the pitch and roll for comparison
				toAngle(acceleration, localAngles);
			
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
				
			break;
		
 			case 1: //transmit
				getACCValues(acceleration);
				displayDominantAngle(acceleration);
			break;
		}
	
}

void volumeControl(void){
	uint8_t volume = 0;
	
	
}

