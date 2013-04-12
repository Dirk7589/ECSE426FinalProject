/**
*@file access.h
*@author Dirk Dubois, Alain Slak
*@date March 6th, 2013
*@brief A set of functions that provide access to global variables shared accross multiple threads
*/

#ifndef __ACCESS_H
#define __ACCESS_H

#include <stdint.h>

/**
*@brief A function that safely access's the corrected values of the accelerometer
*@param[inout] accValues A pointer to the new location in memory that data is copied to
*@retval None
*/
void getACCValues(float* accValues);

/**
*@brief A function that safely access's the corrected values of the remote accelerometer
*@param[inout] accValues A pointer to the new location in memory that data is copied to
*@retval None
*/
void getWirelessACCValues(float* accValues);
/**
*@brief A function that safely access's the angle values of the remote accelerometer
*@param[inout] angles A pointer to the new location in memory that data is copied to
*@retval None
*/
void getWirelessAngles(int8_t* angles);

/**
*@brief A function that gets the information recieved over wireless to be processed
*@param[inout] rxBuffer is the buffer to which the data is stored
*@param[in] bufferSize The size of the passed in buffer
*@warning bufferSize must be of the same size as WIRELESS_BUFFER_SIZE, see wireless.h for details
*@retval None
*/
void getRecieved(uint8_t* rxBuffer, uint8_t bufferSize);

/**
*@brief A function that sets the information to be transmitted over wireless
*@param[inout] txBuffer is the buffer from which the data is transferred
*@retval None
*/
void setTransmit(uint8_t* txBuffer);

/**
*@brief A function that safely access's the device audio volume
*@param[inout]  A pointer to the new location in memory that data is copied to
*@retval None
*/
void getVolume(uint8_t* vlmToGet);

/**
*@brief A function that safely access's the device audio volume
*@param[inout]  A value to set pointer to the new location in memory that data is copied to
*@retval None
*/
void setVolume(uint8_t vlmToSet);

void increaseVolume(void);

void decreaseVolume(void);

#endif
