 /**
*@file lcd.h
*@author Dirk Dubois, Alain Slak
*@date April 4th, 2013
*@brief A set of functions to communicate with an LCD display.
*This display uses a Hitachi44780 control chip
*/

#ifndef __LCD_H
#define __LCD_H

/*Hardware Defines*/
#define GPIO_PORT_LCD GPIOD /**<Defines the GPIO port*/
#define GPIO_CLOCK_LCD	RCC_AHB1Periph_GPIOD /**<Defines the GPIO peripheral clock*/
#define LCD_RS GPIO_Pin_10/**<The RS pin*/
#define LCD_RW GPIO_Pin_9/**<The RW pin*/
#define LCD_EN GPIO_Pin_8 /**<The EN pin*/

//The data lines of the LCD should be GPI_Pin defs
#define LCD_D0 GPIO_Pin_0
#define LCD_D1 GPIO_Pin_1
#define LCD_D2 GPIO_Pin_2
#define LCD_D3 GPIO_Pin_3
#define LCD_D4 GPIO_Pin_4
#define LCD_D5 GPIO_Pin_5
#define LCD_D6 GPIO_Pin_6
#define LCD_D7 GPIO_Pin_7

/*Other Defines*/
#define MODE_4_BIT 0x002C /**<Defines the 4 bit mode for configuration*/
#define MODE_8_BIT 0x003C /**<Defines the 8 bit mode for configuration*/

#define LCD_INSTRUCTION 0 /**<Values to assign to RS pin*/
#define LCD_DATA 1 /**<Values to assign to RS pin*/
#define LCD_WRITE 0 /**<values to assign to RW pin*/
#define LCD_READ 1 /**<values to assign to RW pin*/


#define LCD_CLEAR_CMD 0x01 /**<Clears the LCD*/
#define LCD_CURSOR_ON 0x0E /**<Turns the cursor on and blink off*/
#define LCD_CURSOR_LINE_2 40 /**<Index of the first character on the second line of the LCD*/

/**
*@brief Writes a byte to the LCD
*@param[in] c The byte to be written to the display
*@retval None
*/
void lcd_write(char c);

/**
*@brief Clear and home the LCD
*@retval None
*/
void lcd_clear(void);

/**
*@brief Write a string of characters to the LCD
*@param[in] s The char pointer to the characters to be written to the display
*@retval None
*/
void lcd_puts(const char * s);

/**
*@brief Go to the specified position
*@param[in] pos Move the cursor to the position specified
*@retval None
*/
void lcd_goto(unsigned char pos);

/**
*@brief Intialize the LCD in either 4-bit or 8-bit mode
*@param[in] mode Using the MODE defines configures the LCD for 4-bit or 8-bit mode
*@retval None
*@warning This function should be called before anything else
*/
void lcd_init(uint16_t mode);

/**
*@brief Writes a character to the LCD screen
*@param[in] char Writes the character to the screen
*@retval None
*/
void lcd_putch(char);

/**
*@brief Strobes the LCD
*@retval None
*/
void lcd_strobe(void);

#endif
