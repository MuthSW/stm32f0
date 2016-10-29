#ifndef __LCD5110_H
#define __LCD5110_H

#include "spi.h"
#include "gpio.h"
#include "lcd_ascii.h"

//The DC pin tells the LCD if we are sending a command or data
#define LCD_COMMAND 0 
#define LCD_DATA 1

//You may find a different size screen, but this one is 84 by 48 pixels
#define LCD_X 84
#define LCD_Y 48

#define PIN_RST GPIO_PIN_0
#define PIN_CE GPIO_PIN_1
#define PIN_DC GPIO_PIN_2

void lcd_init(void);
void lcd_gotoXY(uint8_t X, uint8_t Y);
void lcd_clear(void);
void lcd_char(uint8_t c);
void lcd_set_inverse(uint8_t inverse);
void lcd_string(uint8_t *str);
void lcd_contrast ( uint8_t contrast );

#endif

