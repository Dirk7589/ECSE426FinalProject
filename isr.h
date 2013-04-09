 /**
*@file isr.h
*@author Dirk Dubois, Alain Slak
*@date April 9th, 2013
*@brief A file containing all the relevant ISRs for the main application
*
*/
#ifndef __ISR_H
#define __ISR_H

#include <stdint.h>
/**
*@brief A function that toggles the LEDs
*@param[in] LEDState current state of the LEDs
*@retval The current LEDState
*/
uint8_t LEDToggle(uint8_t LEDState);

/**
*@brief An interrupt handler for EXTI0
*@retval None
*/
void EXTI0_IRQHandler(void);

/**
*@brief An interrupt handler for EXTI1
*@retval None
*/
void EXTI1_IRQHandler(void);

/**
*@brief An interrupt handler for EXTI2 for volume down
*@retval None
*/
void EXTI2_IRQHandler(void);

/**
*@brief An interrupt handler for EXTI4 for volume up
*@retval None
*/
void EXTI4_IRQHandler(void);

/**
*@brief An interrupt handler for Tim3
*@retval None
*/
void TIM3_IRQHandler(void);

/**
*@brief An interrupt handler for DMA2_Stream0
*@retval None
*/
void DMA2_Stream0_IRQHandler(void);
#endif


