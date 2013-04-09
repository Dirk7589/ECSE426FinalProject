 /**
*@file isr.c
*@author Dirk Dubois, Alain Slak
*@date April 9th, 2013
*@brief A file containing all the relevant ISRs for the main application
*
*/

#include "isr.h"
#include "common.h"
#include "stm32f4xx.h"

/**Helper functions**/
/**
*@brief A function that toggles the LEDs
*@param[in] LEDState current state of the LEDs
*@retval The current LEDState
*/
uint8_t LEDToggle(uint8_t LEDState){
    if(!LEDState){
        LEDState = 1; //update state
        //Turn on LEDs
        GPIOD->BSRRL = GREEN_LED;
        GPIOD->BSRRL = ORANGE_LED;
        GPIOD->BSRRL = RED_LED;
        GPIOD->BSRRL = BLUE_LED;
    }
    else{
        LEDState = 0; //update state
        GPIOD->BSRRH = GREEN_LED | ORANGE_LED | RED_LED | BLUE_LED; //Turn off leds
    }
		
		return LEDState;
}

/**
*@brief An interrupt handler for EXTI0
*@retval None
*/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){
		buttonState = 1 - buttonState;	//Change the current button state
		EXTI_ClearITPendingBit(EXTI_Line0);	//Clear the EXTI0 interrupt flag
	}
}

/**
*@brief An interrupt handler for EXTI1
*@retval None
*/

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){
		tapState = 1 - tapState;	//change the current tap state
		EXTI_ClearITPendingBit(EXTI_Line1);	//Clear the EXTI1 interrupt flag
	}
}

/**
*@brief An interrupt handler for EXTI2 for volume down
*@retval None
*/

void EXTI2_IRQHandler(void)
 {
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){
		EXTI->IMR &= EXTI_Line2;//DISABLE INTERUPTS ON EXTI2
		osTimerStart(vlmDownId, 250);//Start Timer
		volumeDownBtn = 1;
		EXTI_ClearITPendingBit(EXTI_Line2);	//Clear the EXTI2 interrupt flag
	}
}

/**
*@brief An interrupt handler for EXTI4 for volume up
*@retval None
*/

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){
		EXTI->IMR &= EXTI_Line4;//DISABLE INTERUPTS ON EXTI4
		osTimerStart(vlmUpId, 250);//Start Timer
		volumeUpBtn = 1;
		EXTI_ClearITPendingBit(EXTI_Line4);	//Clear the EXTI4 interrupt flag
	}
}

/**
*@brief An interrupt handler for Tim3
*@retval None
*/
void TIM3_IRQHandler(void)
{
	osSignalSet(aThread, sampleACCFlag);
	osSignalSet(kThread, readKeypadFlag);
	osSignalSet(lThread, displayLCDFlag);
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


