/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include <stdbool.h>
#include "i2cutil.h"
#include "rtcutil.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t array[50] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB

char *DOW[] = { "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };


//volatile uint8_t tm_ready = 0;
//volatile uint8_t conf_mode = 0;
extern uint8_t aTxBuffer[8];
uint8_t sec=0,min=0,hour=0,day=0,date=0,month=0,year=0,temp1=0,temp2=0;

// All DS3231 registers
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t alarm1_secconds;
	uint8_t alarm1_minutes;
	uint8_t alarm1_hours;
	uint8_t alarm1_day;
	uint8_t alarm1_date;
	uint8_t alarm2_minutes;
	uint8_t alarm2_hours;
	uint8_t alarm2_day;
	uint8_t alarm2_date;
	uint8_t control;
	uint8_t status;
	uint8_t aging;
	uint8_t msb_temp;
	uint8_t lsb_temp;
} DS3231_registers_TypeDef;

// DS3231 date
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_of_week;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} DS3231_date_TypeDef;

// Human Readable Format date
typedef struct {
	uint8_t  Seconds;
	uint8_t  Minutes;
	uint8_t  Hours;
	uint8_t  Day;
	uint8_t  Month;
	uint16_t Year;
	uint8_t  DOW;
} HRF_date_TypeDef;

struct tm {
	int sec;      // 0 to 59
	int min;      // 0 to 59
	int hour;     // 0 to 23
	int mday;     // 1 to 31
	int mon;      // 1 to 12
	int year;     // year-99
	int wday;     // 1-7

    // 12-hour clock data
    bool am; // true for AM, false for PM
    int twelveHour; // 12 hour clock time
};

// statically allocated 
struct tm _tm;

uint8_t dec2bcd(uint8_t d)
{
  return ((d/10 * 16) + (d % 10));
}

uint8_t bcd2dec(uint8_t b)
{
  return ((b/16 * 10) + (b % 16));
}

/* DS3231 register map
 *
 *  00h-06h: seconds, minutes, hours, day-of-week, date, month, year (all in BCD)
 *       bit 7 should be set to zero: The DS3231 clock is always running
 *  07h: A1M1  Alarm 1 seconds
 *  08h: A1M2  Alarm 1 minutes
 *  09h: A1M3  Alarm 1 hour (bit6 is am/pm flag in 12h mode)
 *  0ah: A1M4  Alarm 1 day/date (bit6: 1 for day, 0 for date)
 *  0bh: A2M2  Alarm 2 minutes
 *  0ch: A2M3  Alarm 2 hour (bit6 is am/pm flag in 12h mode)
 *  0dh: A2M4  Alarm 2 day/data (bit6: 1 for day, 0 for date)
 *       <see data sheet page12 for Alarm register mask bit tables:
 *        for alarm when hours, minutes and seconds match set 1000 for alarm 1>
 *  0eh: control
 *      bit7: !EOSC
 *      bit6: BBSQW
 *      bit5: CONV
 *      bit4: RS2
 *      bit3: RS1
 *      bit2: INTCN
 *      bit1: A2IE
 *      bit0: A1IE
 *  0fh: control/status
 *      bit7: OSF
 *      bit6: 0
 *      bit5: 0
 *      bit4: 0
 *      bit3: EN32kHz
 *      bit2: BSY
 *      bit1: A2F alarm 2 flag
 *      bit0: A1F alarm 1 flag
 * 10h: aging offset (signed)
 * 11h: MSB of temp (signed)
 * 12h: LSB of temp in bits 7 and 6 (0.25 degrees for each 00, 01, 10, 11)
 *
 */

struct tm* rtc_get_time(void)
{
	uint8_t rtc[9];
	uint8_t century = 0;

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
////	twi_begin_transmission(RTC_ADDR);
////	twi_send_byte(0x0);
////	twi_end_transmission();
	/* Format array to send */
	rtc[0] = DS3231_seconds;
	
	/* Send register address */
//	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DS3231_addr, rtc, 1, 10000)!= HAL_OK)
//	while(I2C_Write_Byte(hi2c1, DS3231_addr, DS3231_seconds, 0)!= HAL_OK)
//	while (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DS3231_addr, rtc, 1, 1000) != HAL_OK)	
//	{
//		/* Error_Handler() function is called when Timeout error occurs.
//			 When Acknowledge failure occurs (Slave don't acknowledge its address)
//			 Master restarts communication */
//		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
//		{
//			Error_Handler();
//		}
//	}

		aTxBuffer[0]=0;
	  I2C_WriteBuffer(hi2c1,(uint16_t)0xD0,1);//§’≠’ı”ıﬁ˝ ”ı≠’± æ±=≠Ø⁄±=‘æ
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY)
		{
		}
		I2C_ReadBuffer(hi2c1,(uint16_t)0xD0,7);

		//=========================================================	
    year = aTxBuffer[6];
		year = RTC_ConvertFromDec(year);	
    month = aTxBuffer[5];
		month = RTC_ConvertFromDec(month);
    date = aTxBuffer[4];
		date = RTC_ConvertFromDec(date);
    day = aTxBuffer[3];
		day = RTC_ConvertFromDec(day);		
//=========================================================		
    hour = aTxBuffer[2];
		hour = RTC_ConvertFromDec(hour);		
    min = aTxBuffer[1];
		min = RTC_ConvertFromDec(min);
    sec = aTxBuffer[0];
		sec = RTC_ConvertFromDec(sec);	


////	twi_request_from(RTC_ADDR, 7);

////	for (uint8_t i = 0; i < 7; i++) {
////		rtc[i] = twi_receive();
////	}
////	twi_end_transmission();
  /*##-3- Put I2C peripheral in reception process ############################*/ 
  /* Timeout is set to 10S */ 
  while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DS3231_addr, (uint8_t *)rtc, 7, 10000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }
	
	
////	// Clear clock halt bit from read data
////	// This starts the clock for a DS1307, and has no effect for a DS3231
////	rtc[0] &= ~(_BV(CH_BIT)); // clear bit

	_tm.sec = bcd2dec(rtc[0]);
	_tm.min = bcd2dec(rtc[1]);
	_tm.hour = bcd2dec(rtc[2]);
	_tm.mday = bcd2dec(rtc[4]);
	_tm.mon = bcd2dec(rtc[5] & 0x1F); // returns 1-12
	century = (rtc[5] & 0x80) >> 7;
	_tm.year = century == 1 ? 2000 + bcd2dec(rtc[6]) : 1900 + bcd2dec(rtc[6]); // year 0-99
	_tm.wday = bcd2dec(rtc[3]); // returns 1-7

	if (_tm.hour == 0) {
		_tm.twelveHour = 0;
		_tm.am = 1;
	} else if (_tm.hour < 12) {
		_tm.twelveHour = _tm.hour;
		_tm.am = 1;
	} else {
		_tm.twelveHour = _tm.hour - 12;
		_tm.am = 0;
	}

	return &_tm;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	
	rtc_get_time();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
