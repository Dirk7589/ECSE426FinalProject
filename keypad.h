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
#define GPIO_PORT GPIOE /**<Defines the GPIO port*/
#define GPIO_CLOCK	RCC_AHB1Periph_GPIOE /**<Defines the GPIO peripheral clock*/

#define ROW_0 GPIO_Pin_8
#define ROW_1 GPIO_Pin_9
#define ROW_2 GPIO_Pin_10
#define ROW_3 GPIO_Pin_11
#define COLUMN_0 GPIO_Pin_12
#define COLUMN_1 GPIO_Pin_13
#define COLUMN_2 GPIO_Pin_14

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


