 /**
*@file dac.h
*@author Dirk Dubois, Alain Slak
*@date April 9th, 2013
*@brief A set of functions to communicate with the DAC
*/

#ifndef __DAC_H
#define __DAC_H

#include "stm32f4xx.h"

void playInit(void);

void play(uint16_t* audioTone);
#endif
