 /**
*@file lcd.c
*@author Dirk Dubois, Alain Slak
*@date April 4th, 2013
*@brief A set of functions to communicate with an LCD display.
*This display uses a Hitachi44780 control chip
*/

#include <stdint.h>
#include "cmsis_os.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
#include "lcd.h"

static uint8_t mode;

/**
*@brief Writes a byte to the LCD
*@param[in] char The byte to be written to the display
*@retval None
*/
void lcd_write(char c){
	
	if(mode == MODE_8_BIT){
		GPIO_PORT->BSRRL = LCD_RS; //Set RS high
		GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
		GPIO_PORT->ODR = GPIO_PORT->ODR + c; //Load the char into the lower half of PORTD
		lcd_strobe(); //Strobe
	}
	
	if(mode == MODE_4_BIT){
		
	}
	
}

/**
*@brief Clear and home the LCD
*@retval None
*/
void lcd_clear(void){
	GPIO_PORT->BSRRH = LCD_RS;	//set RS low
	
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + LCD_CLEAR_CMD;
	lcd_strobe();	//write the clear command
}

/**
*@brief Write a string of characters to the LCD
*@param[in] s The char pointer to the characters to be written to the display
*@retval None
*/
void lcd_puts(const char * s){
	
	while(*s){
		lcd_write(*s);
		*s++;
	}
}

/**
*@brief Go to the specified position
*@param[in] pos Move the cursor to the position specified
*@retval None
*/
void lcd_goto(unsigned char pos){
	GPIO_PORT->BSRRH = LCD_RS;	//set RS low
	
	pos = pos + 0x80;
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + pos;
	lcd_strobe();	//write the clear command
}

/**
*@brief Intialize the LCD in either 4-bit or 8-bit mode
*@param[in] mode Using the MODE defines configures the LCD for 4-bit or 8-bit mode
*@retval None
*@warning This function should be called before anything else
*/
void lcd_init(uint16_t m){
	mode = m;
	RCC_AHB1PeriphClockCmd(GPIO_CLOCK, ENABLE); //Setup the peripheral clock
	
	osDelay(50);
	
	GPIO_InitTypeDef gpio_init_lcd;			//Create the intialization struct
	GPIO_StructInit(&gpio_init_lcd);		//Intialize the struct
	
	gpio_init_lcd.GPIO_Pin	= (LCD_RS | LCD_RW | LCD_EN | LCD_D0 | LCD_D1 | LCD_D2 | LCD_D3 | LCD_D4| LCD_D5 | LCD_D6 | LCD_D7); //Setting all LCD pins
	gpio_init_lcd.GPIO_Mode = GPIO_Mode_OUT; //Set as output
	gpio_init_lcd.GPIO_Speed = GPIO_Speed_100MHz; //Set at max slew rate
	gpio_init_lcd.GPIO_OType = GPIO_OType_PP; //Push Pull config
	gpio_init_lcd.GPIO_PuPd = GPIO_PuPd_NOPULL; //Turn off pull ups 
	
	GPIO_Init(GPIO_PORT, &gpio_init_lcd); //Intialize port
	
	GPIO_PORT->ODR = 0x30; 
	GPIO_PORT->BSRRH = LCD_RS; //Place in write mode
	GPIO_PORT->BSRRH = LCD_RW; //Place in write mode
	
	lcd_strobe(); //Strobe EN line to latch in values
	osDelay(5); //Delay for settling
	
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + LCD_CURSOR_ON;
	lcd_strobe();
	
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + 0x06;
	lcd_strobe();
	
	osDelay(5);
	
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + mode;
	lcd_strobe();
	
// 	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
// 	GPIO_PORT->ODR = GPIO_PORT->ODR + 0x28;
// 	lcd_strobe();
	
	GPIO_PORT->ODR = GPIO_PORT->ODR & 0xFF00;
	GPIO_PORT->ODR = GPIO_PORT->ODR + 0x0E;
	lcd_strobe();

	lcd_clear();	// Clear screen
}

/**
*@brief Writes a character to the LCD screen
*@param[in] char Writes the character to the screen
*@retval None
*/
void lcd_putch(char);

void lcd_strobe(void){
	
	GPIO_PORT->BSRRL = LCD_EN;
	osDelay(20);
	GPIO_PORT->BSRRH = LCD_EN;	
}


