#include "lcd5110.h" 


void lcd_write(uint8_t data_or_command, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOB, PIN_DC, data_or_command);
	
	//HAL_GPIO_WritePin(GPIOB, PIN_CE, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi1, &data, sizeof(data), 1000);
  if (HAL_SPI_Transmit(&hspi1, &data, sizeof(data), 1000) != HAL_OK)
  {
    Error_Handler();
  }	
	//HAL_GPIO_WritePin(GPIOB, PIN_CE, GPIO_PIN_SET);
}

void lcd_init() 
{
	HAL_GPIO_WritePin(GPIOB, PIN_RST, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, PIN_RST, GPIO_PIN_SET);
	
	lcd_write(LCD_COMMAND, 0x21); 
	lcd_write(LCD_COMMAND, 0xBF); //Contrast
	lcd_write(LCD_COMMAND, 0x04); // Set Temp coefficent. //0x04
	lcd_write(LCD_COMMAND, 0x13); // LCD bias mode 1:48. //0x13
	lcd_write(LCD_COMMAND, 0x0C); // LCD in normal mode
	
	lcd_write(LCD_COMMAND, 0x20); 
	lcd_write(LCD_COMMAND, 0x0C); 
}

void lcd_gotoXY(uint8_t X, uint8_t Y)
{
		lcd_write(LCD_COMMAND, 0x80|X);
		lcd_write(LCD_COMMAND, 0x40|Y);
}

void lcd_clear()
{
	for (int index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
		{
			lcd_write(LCD_DATA, 0x00);
		}
		lcd_gotoXY(0,0);
}

void lcd_set_inverse(uint8_t inverse)
{
	lcd_write(LCD_COMMAND, inverse?0x0D:0x0C);
}

void lcd_char(uint8_t c)
{
	lcd_write(LCD_DATA, 0x00);
	for (int index = 0 ; index < 5 ; index++)
	{
   lcd_write(LCD_DATA, ASCII[c - 0x20][index]);
	} //0x20 is the ASCII character for Space (' '). The font table starts with this character
	lcd_write(LCD_DATA, 0x00); 
	
}

void lcd_string(uint8_t *str)
{
	uint8_t i=0;
	while(str[i])
	{
		lcd_char(str[i]);
		i++;
	}
}

/*--------------------------------------------------------------------------------------------------

  Name         :  LcdContrast

  Description  :  Set display contrast.

  Argument(s)  :  contrast -> Contrast value from 0x00 to 0x7F.

  Return value :  None.

  Notes        :  No change visible at ambient temperature.

--------------------------------------------------------------------------------------------------*/
void lcd_contrast ( uint8_t contrast )
{
    //  LCD Extended Commands.
    lcd_write( LCD_COMMAND, 0x21 );

    // Set LCD Vop (Contrast).
    lcd_write( LCD_COMMAND, 0x80 | contrast );

    //  LCD Standard Commands, horizontal addressing mode.
    lcd_write( LCD_COMMAND, 0x20 );
}

