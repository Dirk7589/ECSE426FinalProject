 /**
*@file dac.h
*@author Dirk Dubois, Alain Slak
*@date April 9th, 2013
*@brief A set of functions to communicate with the DAC
*/

#ifndef __DAC_H
#define __DAC_H

#include "stm32f4xx.h"

#define SAMPLING_RATE 22050
#define BUFFER_SIZE 2048
#define INVERSE_SAMPLING_RATE 0.00004535147

void playInit(void);

void play(uint16_t* audioTone);

void adjustPitch(uint16_t* sinTable, uint16_t desiredFreq);
#endif
