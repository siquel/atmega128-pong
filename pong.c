#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include "font5x7.h"

typedef enum { 
	LCD_CMD  = 0, 
	LCD_DATA = 1 
} lcd_cmd_data_t;
	

/* Lcd screen size */
#define LCD_X_RES 128
#define LCD_Y_RES 64
#define LCD_CACHE_SIZE ((LCD_X_RES * LCD_Y_RES) / 8)

/* Pinout for LCD */
#define LCD_CLK_PIN 	(1<<PC4)
#define LCD_DATA_PIN 	(1<<PC3)
#define LCD_DC_PIN 		(1<<PC2)
#define LCD_CE_PIN 		(1<<PC1)
#define LCD_RST_PIN 	(1<<PC0)
#define LCD_PORT		PORTC
#define LCD_DDR			DDRC

void lcd_send(uint8_t data, lcd_cmd_data_t cd);

void lcd_clear() 
{
	int i, j;

	for(i=0;i<8;i++)
	{
		lcd_send(0xB0 | i, LCD_CMD);
		lcd_send(0x10, LCD_CMD);
		lcd_send(0x00, LCD_CMD);	// column 0

		for(j=0;j<128;j++)
		{
			lcd_send(0x00, LCD_DATA);
		}
	}    

	lcd_send(0xB0, LCD_CMD);	// page 0
	lcd_send(0x10, LCD_CMD);
	lcd_send(0x00, LCD_CMD);	// column 0
}

void lcd_send(uint8_t data, lcd_cmd_data_t cd)
{
	// Data/DC are outputs for the lcd (all low)
	LCD_DDR |= LCD_DATA_PIN | LCD_DC_PIN;
	
    // Enable display controller (active low)
    LCD_PORT &= ~LCD_CE_PIN;

    // Either command or data
    if(cd == LCD_DATA) 
	{
        LCD_PORT |= LCD_DC_PIN;
    } 
	else 
	{
        LCD_PORT &= ~LCD_DC_PIN;
    }
	
	for(unsigned char i=0;i<8;i++) 
	{
		// Set the DATA pin value
		if((data>>(7-i)) & 0x01) 
		{
			LCD_PORT |= LCD_DATA_PIN;
		} 
		else 
		{
			LCD_PORT &= ~LCD_DATA_PIN;
		}
		
		// Toggle the clock
		LCD_PORT |= LCD_CLK_PIN;
		for(int j=0;j<4;j++); // delay
		LCD_PORT &= ~LCD_CLK_PIN;
	}

	// Disable display controller
	//LCD_PORT &= ~LCD_DC_PIN;
    LCD_PORT |= LCD_CE_PIN;
	
	// Data/DC can be used as button inputs when not sending to LCD (/w pullups)
	LCD_DDR &= ~(LCD_DATA_PIN | LCD_DC_PIN);
	LCD_PORT |= LCD_DATA_PIN | LCD_DC_PIN;
}

void lcd_init(void)
{
	
	//Pull-up on reset pin
    LCD_PORT |= LCD_RST_PIN;	//Reset = 1
	
	//Set output bits on lcd port
	LCD_DDR |= LCD_RST_PIN | LCD_CE_PIN | LCD_DC_PIN | LCD_DATA_PIN | LCD_CLK_PIN;

	//Wait after VCC high for reset (max 30ms)
    _delay_ms(15);
    
    //Toggle display reset pin
    LCD_PORT &= ~LCD_RST_PIN; 	//Reset = 0
	_delay_ms(15);
    LCD_PORT |= LCD_RST_PIN;	//Reset = 1

	_delay_ms(15);

    //Disable LCD controller
    LCD_PORT |= LCD_CE_PIN;

    lcd_send(0xEB, LCD_CMD);  	//LCD bias 
    lcd_send(0x23, LCD_CMD);  	//Set Lines >> 23 = 64
    lcd_send(0x81, LCD_CMD);	//Set Potentiometer
	lcd_send(0x64, LCD_CMD);	//16 >> 64 (Tummuus)
	lcd_send(0xAF, LCD_CMD);  	//Set Display ON
    lcd_send(0xCC, LCD_CMD);  	//Set LCD to RAM mapping
    
    // Clear lcd
    lcd_clear();
	
	//For using printf
	//fdevopen(lcd_chr, 0);
}

void lcd_char(int8_t c)
{
	for (uint8_t i = 0; i < 5; ++i)
	{
		lcd_send(pgm_read_byte(&font5x7[c - 32][i]) << 1, LCD_DATA);
	}
}

int main()
{
	lcd_init();

	int y = 0;
	int x = 0;

	for (;;) 
	{
		lcd_clear();
		if ( x < LCD_X_RES) ++x;
		
		lcd_send(0xB0 | y, LCD_CMD);	// page

		lcd_send(0x00 | (x & 0x0F), LCD_CMD); // what is this?
		lcd_send(0x10 | ((x & 0xF0)>>4), LCD_CMD);	// column

		lcd_char('b');

		lcd_send(0, LCD_DATA); // what is this
		
		_delay_ms(500);
	}
	return 0;
}
