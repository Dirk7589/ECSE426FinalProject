 /**
*@file keypad.c
*@author Dirk Dubois, Alain Slak
*@date April 4th, 2013
*@brief A set of functions to interface with a 9 digit keypad
*/
#include <stdint.h>
#include "cmsis_os.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "keypad.h"

char keypadRead(void){
	char value = 'E';
	//read from column 0
	GPIO_PORT_KEY->BSRRL = COLUMN_0;
	// Delay for debouncing
	osDelay(5);
	if(GPIO_PORT_KEY->IDR & ROW_0){
		value = '1';
	}
	else if(GPIO_PORT_KEY->IDR & ROW_1){
		value = '4';
	}
	else if(GPIO_PORT_KEY->IDR & ROW_2){
		value = '7';
	}
	else if(GPIO_PORT_KEY->IDR & ROW_3){
		value = '*';
	}
	GPIO_PORT_KEY->BSRRH = COLUMN_0;
	
	
	
	if (value == 'E') {
		//read from column 1
		GPIO_PORT_KEY->BSRRL = COLUMN_1;
		// Delay for debouncing
		osDelay(5);
		if(GPIO_PORT_KEY->IDR & ROW_0){
			value = '2';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_1){
			value = '5';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_2){
			value = '8';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_3){
			value = '0';
		}
		GPIO_PORT_KEY->BSRRH = COLUMN_1;	
	}
	
	if (value == 'E') {
		//read from column 2
		GPIO_PORT_KEY->BSRRL = COLUMN_2;
		// Delay for debouncing
		osDelay(5);
		if(GPIO_PORT_KEY->IDR & ROW_0){
			value = '3';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_1){
			value = '6';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_2){
			value = '9';
		}
		else if(GPIO_PORT_KEY->IDR & ROW_3){
			value = '#';
		}
		GPIO_PORT_KEY->BSRRH = COLUMN_2;
	}
	
	// Delay for debouncing
	osDelay(50);
	
	return value;
}

void keypadInit(void){
	RCC_AHB1PeriphClockCmd(GPIO_CLOCK_KEY, ENABLE); //Setup the peripheral clock
	
	GPIO_InitTypeDef gpio_init_key;			//Create the intialization struct
	GPIO_StructInit(&gpio_init_key);		//Intialize the struct
	
	gpio_init_key.GPIO_Pin	= (COLUMN_0 | COLUMN_1 | COLUMN_2); //Setting all column pins
	gpio_init_key.GPIO_Mode = GPIO_Mode_OUT; //Set as output
	gpio_init_key.GPIO_Speed = GPIO_Speed_100MHz; //Set at max slew rate
	gpio_init_key.GPIO_OType = GPIO_OType_PP; //Push Pull config
	gpio_init_key.GPIO_PuPd = GPIO_PuPd_NOPULL; //Turn off pull ups 
	
	GPIO_Init(GPIO_PORT_KEY, &gpio_init_key); //Intialize port
	
	gpio_init_key.GPIO_Pin	= (ROW_0 | ROW_1 | ROW_2 | ROW_3); //Setting all column pins
	gpio_init_key.GPIO_Mode = GPIO_Mode_IN; //Set as output
	gpio_init_key.GPIO_Speed = GPIO_Speed_100MHz; //Set at max slew rate
	gpio_init_key.GPIO_OType = GPIO_OType_PP; //Push Pull config
	gpio_init_key.GPIO_PuPd = GPIO_PuPd_DOWN; //Turn off pull ups 
	
	GPIO_Init(GPIO_PORT_KEY, &gpio_init_key); //Intialize port
}
