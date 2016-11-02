#ifndef __FUNC_LIST_H
#define __FUNC_LIST_H


#include "main.h"

// ----------------------------------------------------------------------------

void lcd_headpiece(void);   // заставка
void rtc_handler(void);     // обработчик часов
void menu_func(void);
void read_alarm(void);
void lcd_led(void);

void read_var_lcd(signed char clock_var);
signed char read_clock_var_lcd( uint8_t ds3231_var, uint8_t addr);


#endif /* __FUNC_LIST_H */
