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

/*Global Variables*/
extern float temperature; /**<The temperature variable*/
extern float accCorrectedValues[3]; /**<The corrected accelerometer values*/
extern float wirelessAccValues[3]; /**<Accelerometer values from the other board*/
extern float angles[2]; /**<A variable containing the pitch and roll */
extern int8_t wirelessAngles[2];
extern int8_t txWireless[WIRELESS_BUFFER_SIZE]; /**<Transmission buffer for Wireless for DMA*/
extern int8_t rxWireless[WIRELESS_BUFFER_SIZE]; /**<Receive buffer for Wireless for DMA*/
extern uint8_t strobeCommand[1];
extern uint8_t status[1];
extern uint8_t audioVolume; /**<The value used to determine the output audio volume*/

extern uint8_t txWirelessInit[WIRELESS_BUFFER_INIT_SIZE]; /**<Transmission buffer for Wireless initialization for DMA*/
extern uint8_t rxWirelessInit[WIRELESS_BUFFER_INIT_SIZE]; /**<Receive buffer for Wireless initialization for DMA*/

extern uint8_t dmaFromAccFlag; /**<A flag variable that represents whether or not DMA was called from the accelerometer thread*/
extern uint8_t dmaFromWirelessFlag; /**<A flag variable that represents whether or not DMA was called from the wireless thread*/
extern uint8_t wirelessRdy;
extern uint8_t dmaFlag;
//extern uint8_t dmaInitFlag;

extern osSemaphoreId wirelessAccId;
extern osSemaphoreId accId; /**<The id for the accCorrectedValues semaphore*/
extern osSemaphoreId rxId; /**<The id for the rx buffer semaphore*/
extern osSemaphoreId txId; /**<The id for the tx buffer semaphore*/
extern osSemaphoreId vlmId; /**<The id for the audio volume semaphore*/

extern osMutexId dmaId; /**<The id for the DMA mutex*/

extern osTimerId vlmUpId; /**<The id for the audio volume up EXTI timer*/
extern osTimerId vlmDownId; /**<The id for the audio volume down EXTI timer*/

extern char key;
#endif
