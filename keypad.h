 /**
*@file keypad.h
*@author Dirk Dubois, Alain Slak
*@date April 4th, 2013
*@brief A set of functions to interface with a 9 digit keypad
*This display uses a Hitachi44780 control chip
*/

#ifndef __KEYPAD_H
#define __KEYPAD_H

/*Hardware Defines*/
#define GPIO_PORT_KEY GPIOD /**<Defines the GPIO port*/
#define GPIO_CLOCK_KEY	RCC_AHB1Periph_GPIOD /**<Defines the GPIO peripheral clock*/

#define ROW_0 GPIO_Pin_3
#define ROW_1 GPIO_Pin_6
#define ROW_2 GPIO_Pin_7
#define ROW_3 GPIO_Pin_8
#define COLUMN_0 GPIO_Pin_0
#define COLUMN_1 GPIO_Pin_1
#define COLUMN_2 GPIO_Pin_2

/**
*@brief Reads the value of the button being pushed
*@retval Button character in ASCII
*/
char keypadRead(void);

/**
*@brief Initialize the keypad
*/
void keypadInit(void);
#endif


