/**
*@file common.h
*@author Dirk Dubois, Alain Slak
*@date March 6th, 2013
*@brief A header file that contains the global variables being used
*/

#ifndef __COMMON_H
#define __COMMON_H

#include "cmsis_os.h"
#include "wireless.h"
#include "dac.h"

/**Common Defines**/
#define GREEN_LED 0x1000 /*!<Defines the bit location of the green LED*/
#define ORANGE_LED 0x2000 /**< Defines the bit location of the orange LED*/
#define RED_LED 0x4000 /**< Defines the bit location of the red LED*/
#define BLUE_LED 0x8000	/**< Defines the bit location of the blue LED*/

/*Global Variables*/
extern float temperature; /**<The temperature variable*/
extern float accCorrectedValues[3]; /**<The corrected accelerometer values*/
extern float wirelessAccValues[3]; /**<Accelerometer values from the other board*/
extern float angles[2]; /**<A variable containing the pitch and roll */
extern int8_t wirelessAngles[2];
extern uint8_t audioVolume; /**<The value used to determine the output audio volume*/
extern char key;
extern uint8_t LEDState; /**<A variable containing the current LED state*/
extern uint8_t orientationMatch;
extern uint8_t LEDCounter;
extern uint16_t aTone;
extern uint16_t bTone;
extern uint16_t cTone;
extern uint16_t dTone;
extern uint16_t eTone;
extern uint16_t fTone;
extern uint16_t gTone;
extern uint16_t cStereo;

/*********DMA Variables********/
extern int8_t txWireless[WIRELESS_BUFFER_SIZE]; /**<Transmission buffer for Wireless for DMA*/
extern int8_t rxWireless[WIRELESS_BUFFER_SIZE]; /**<Receive buffer for Wireless for DMA*/
extern uint8_t txWirelessInit[WIRELESS_BUFFER_INIT_SIZE]; /**<Transmission buffer for Wireless initialization for DMA*/
extern uint8_t rxWirelessInit[WIRELESS_BUFFER_INIT_SIZE]; /**<Receive buffer for Wireless initialization for DMA*/
extern uint8_t strobeCommand[1];
extern uint8_t status[1];

/**********Flags**************/
extern uint8_t dmaFromAccFlag; /**<A flag variable that represents whether or not DMA was called from the accelerometer thread*/
extern uint8_t dmaFromWirelessFlag; /**<A flag variable that represents whether or not DMA was called from the wireless thread*/


/**********OS Signal Masks**********/
extern uint8_t sampleACCFlag; /**<A flag variable for sampling, restricted to a value of 0 or 1*/
extern uint8_t readKeypadFlag; /**<A flag variable that represents the keypad poll*/
extern uint8_t displayLCDFlag; /**<A flag variable that represents the display poll*/
extern uint8_t dmaFlag; /**<A flag variable that represents the dma poll*/


/*********DAC Variables**************/
extern uint16_t newPitchBuffer[BUFFER_SIZE]; /**<Buffer containing adjusted pitch values*/
extern uint16_t phase;

/******Thread Prototypes*************/
extern osThreadId aThread; //Accelerometer thread ID
extern osThreadId wThread; //Wireless thread ID
extern osThreadId kThread; //Keypad thread ID
extern osThreadId lThread; //LCD thread ID


extern osSemaphoreId wirelessAccId;
extern osSemaphoreId accId; /**<The id for the accCorrectedValues semaphore*/
extern osSemaphoreId rxId; /**<The id for the rx buffer semaphore*/
extern osSemaphoreId txId; /**<The id for the tx buffer semaphore*/
extern osSemaphoreId vlmId; /**<The id for the audio volume semaphore*/

extern osMutexId dmaId; /**<The id for the DMA mutex*/


//ISR States and Flags
extern uint8_t buttonState; /**<A variable that represents the current state of the button*/
extern uint8_t tapState; /**<A variable that represents the current state of the tap detect*/
extern uint8_t volumeBtnUp;
extern uint8_t volumeBtnDown; /**<A variable that represents the state of the volume up/down button*/
extern uint8_t volumeBtnUpFlag;
extern uint8_t volumeBtnDownFlag;
extern uint8_t volumeCounterUp;
extern uint8_t volumeCounterDown; 
#endif
