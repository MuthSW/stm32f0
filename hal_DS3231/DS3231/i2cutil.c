#include "stm32f0xx_hal.h"

//#include "TFT_lcd.h"
#include "i2cutil.h"
#include "rtcutil.h"

//-----------------------------------
uint8_t aTxBuffer[8];
extern char array[50];

//-----------------------------------
void I2C_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf)
{
	while(HAL_I2C_Master_Transmit(&hi, (uint16_t) DEV_ADDR, (uint8_t*) &aTxBuffer, (uint16_t) sizebuf, (uint32_t) 1000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi)!=HAL_I2C_ERROR_AF)
		{
				sprintf(array, "Error I2C Buffer");
	      //drawStrings(array,40,50,2,WHITE,BLACK);
		}
		
	}
}
//--------------------------------------
//--------------------------------------
void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf)
{
  while(HAL_I2C_Master_Receive(&hi, (uint16_t) DEV_ADDR, (uint8_t*) &aTxBuffer, (uint16_t) sizebuf, (uint32_t) 10000)!=HAL_OK)
	{
		if(HAL_I2C_GetError(&hi)!=HAL_I2C_ERROR_AF)
		{
				sprintf(array, "Error I2C Buffer");
	      //drawStrings(array,40,50,2,WHITE,BLACK);
		}
		
	}
}

//--------------------------------------
//--------------------------------------
//--------------------------------------
//--------------------------------------
uint8_t I2C_Write_Byte(I2C_HandleTypeDef hi,uint8_t DEV_ADDR,uint8_t addr, uint8_t data)
{
  uint8_t buf[] = {addr, data};
  uint8_t d;
  while (HAL_I2C_GetState(&hi) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Transmit(&hi, (uint16_t) DEV_ADDR, buf, 2, (uint32_t) 10000);
  if ( d != HAL_OK) {
      return d;
  }
  return HAL_OK;
}
//--------------------------------------
//--------------------------------------
uint8_t I2C_Read_Byte(I2C_HandleTypeDef hi,uint8_t DEV_ADDR,uint8_t addr)
{
  uint8_t data = 0;
  uint8_t d;
  while (HAL_I2C_GetState(&hi) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Transmit(&hi, (uint16_t) DEV_ADDR, &addr, 1, (uint32_t) 10000);
  if ( d != HAL_OK) {
      return d;
  }

  while (HAL_I2C_GetState(&hi) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Receive(&hi, (uint16_t) DEV_ADDR, &data, 1, (uint32_t) 10000);
  if ( d != HAL_OK) {
      return d;
  }
  return data;
}

