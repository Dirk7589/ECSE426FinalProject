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

DMA_InitTypeDef dacStruct;
uint16_t newPitchBuffer[BUFFER_SIZE];
uint16_t phase;

void playInit(void){
	/* Initialize I2S interface */  
	EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);

	EVAL_AUDIO_Init(OUTPUT_DEVICE_AUTO, 50, I2S_AudioFreq_22k);

	dacStruct.DMA_Channel = AUDIO_I2S_DMA_CHANNEL;  
	dacStruct.DMA_PeripheralBaseAddr = AUDIO_I2S_DMA_DREG;
	dacStruct.DMA_Memory0BaseAddr = (uint32_t)0;      /* This field will be configured in play function */
	dacStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dacStruct.DMA_BufferSize = (uint32_t)0xFFFE;      /* This field will be configured in play function */
	dacStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dacStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dacStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dacStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
	dacStruct.DMA_Mode = DMA_Mode_Circular;
	dacStruct.DMA_Priority = DMA_Priority_High;
	dacStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	dacStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dacStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dacStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

	DMA_Init(AUDIO_I2S_DMA_STREAM, &dacStruct);
}

void play(uint16_t* audioTone){
	
	dacStruct.DMA_Memory0BaseAddr = (uint32_t)audioTone; //Select the tone to play
	dacStruct.DMA_BufferSize = (uint32_t)BUFFER_SIZE; //Specify the buffer size
  /* If the I2S peripheral is still not enabled, enable it */
  if ((CODEC_I2S->I2SCFGR & 0x0400) == 0)
  {
    I2S_Cmd(CODEC_I2S, ENABLE);
  }
	
	/* Configure the DMA Stream with the new parameters */
	DMA_Init(AUDIO_I2S_DMA_STREAM, &dacStruct); //Intialize it
	DMA_MemoryTargetConfig(AUDIO_I2S_DMA_STREAM, (uint32_t)audioTone, DMA_Memory_1);
	/* Enable the I2S DMA Stream*/
	DMA_Cmd(AUDIO_I2S_DMA_STREAM, ENABLE); //Turn on DMA
}

void adjustPitch(uint16_t* sinTable, uint16_t desiredFreq) {
	uint16_t i = 0;
	
	float phaseShift = BUFFER_SIZE * (desiredFreq * INVERSE_SAMPLING_RATE);
	for(i = 0; i < BUFFER_SIZE; i++) {
		phase = phase + (uint16_t)phaseShift;
		phase = phase % BUFFER_SIZE;
		newPitchBuffer[i] = sinTable[phase];
	}
}
