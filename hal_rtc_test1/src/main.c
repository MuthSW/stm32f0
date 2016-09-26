/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0308_discovery.h"

static RTC_HandleTypeDef RtcHandle;
RTC_TimeTypeDef   RTC_TimeStructure;


void RTC_Config(void);
static void error(char *str);

int main(void)
{
	uint32_t button_state, button_state_old;
	uint32_t ticks;

	HAL_Init();

	// init RTC
	// see stm32f0xx_hal_rtc.c
	__PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_HSE_DIV32);
	__HAL_RCC_RTC_ENABLE();

//	// Reset Backup domain
//    __HAL_RCC_BACKUPRESET_FORCE();
//    __HAL_RCC_BACKUPRESET_RELEASE();
//    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);


	RtcHandle.Instance = RTC;
	//RtcHandle.Init.AsynchPrediv = 0x7F;
	//RtcHandle.Init.SynchPrediv = 0x0138;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.OutPut = RTC_OUTPUT_ALARMA;	//RTC_OUTPUTSOURCE_NONE
	//RtcHandle.Init.OutPutType =

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_On(LED4);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
	//SysTick_Config(1000000);
	//HAL_SYSTICK_Config(1000000);	//1sec

    if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {
        error("RTC error: RTC initialization failed.");
    }
	HAL_RTC_WaitForSynchro(&RtcHandle);

	RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT_24;
	RTC_TimeStructure.Hours = 0;
	RTC_TimeStructure.Minutes = 50;
	RTC_TimeStructure.Seconds = 0;

    if (HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStructure, RTC_FORMAT_BIN) != HAL_OK) {
        error("RTC error: RTC initialization failed.");
    }
    HAL_RTC_WaitForSynchro(&RtcHandle);

	for(;;){
		button_state = BSP_PB_GetState(BUTTON_USER);
		if(button_state_old != button_state){
			button_state_old = button_state;
			if (HAL_GetTick() - ticks > 10){
				ticks = HAL_GetTick();

				//RTC_TimeStructInit(&RTC_TimeStructure);
				HAL_RTC_GetTime(&RtcHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
				uint8_t hour=RTC_TimeStructure.Hours;
				uint8_t min=RTC_TimeStructure.Minutes;
				uint8_t sec=RTC_TimeStructure.Seconds;

				BSP_LED_Toggle(LED3);
			}
		}
	}


}
/**
  * @brief  Configures the RTC clock source.
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{

}

static void error(char *str)
{
	for(;;){
		HAL_Delay(1000);
		BSP_LED_Toggle(LED3);
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

