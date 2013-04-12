 /**
*@file dac.c
*@author Dirk Dubois, Alain Slak
*@date April 9th, 2013
*@brief A set of functions to communicate with the DAC
*/

#include "dac.h"
#include "stm32f4_discovery_audio_codec.h"
#include "stm32f4xx.h"

/*Global Variables*/    

void play(uint16_t* audioTone, DMA_InitTypeDef DMA_InitStructure){
	
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)audioTone;
    DMA_InitStructure.DMA_BufferSize = (uint32_t)55125;
    
    /* Configure the DMA Stream with the new parameters */
    DMA_Init(AUDIO_I2S_DMA_STREAM, &DMA_InitStructure);
    
    /* Enable the I2S DMA Stream*/
    DMA_Cmd(AUDIO_I2S_DMA_STREAM, ENABLE);   
}
