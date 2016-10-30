/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

uint16_t ADC_raw[3];
unsigned Vdd;
unsigned Vin;
unsigned temperature;
//uint8_t adc_index = 0;
uint8_t adc_index = 0;

/* Variables for ADC conversions results computation to physical values */
uint16_t   uhADCChannelToDAC_mVolt = 0;
uint16_t   uhVrefInt_mVolt = 0;
 int32_t   wTemperature_DegreeCelsius = 0;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ADC1_COMP_IRQn);

  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef ADC_read(uint32_t* result)
{
	HAL_StatusTypeDef tmp_hal_status = HAL_OK;
	
	tmp_hal_status = HAL_ADC_Start(&hadc);
	if (tmp_hal_status == HAL_OK) {
		tmp_hal_status = HAL_ADC_PollForConversion(&hadc, 500);
		if ( tmp_hal_status == HAL_OK){
			*result = HAL_ADC_GetValue(&hadc);
		}
	}
	return tmp_hal_status;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// 0: ADC_CHANNEL_TEMPSENSOR
	// 1: ADC_CHANNEL_VREFINT
	// 2: ADC_CHANNEL_VBAT
	
//	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
//	{
//		ADC_raw[adc_index] = HAL_ADC_GetValue(hadc);
//		if (adc_index == 1) {
//			Vdd = 3300 * (*VREFINT_CAL_ADDR)/ADC_raw[adc_index];
//		}
//		adc_index++;
//		if (adc_index >= 3){ adc_index = 0;};
//		
//	}	
	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
		{
		ADC_raw[adc_index] = HAL_ADC_GetValue(hadc);
		adc_index++;
		}

	if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
		{
		adc_index = 0;

		Vdd = 3300 * (*VREFINT_CAL_ADDR) / ADC_raw[2];

		temperature = (((int32_t)ADC_raw[1] * Vdd/3300)- (int32_t) *TEMP30_CAL_ADDR );
		temperature = temperature * (int32_t)(110 - 30);
		temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
		temperature = temperature + 30;


		Vin = Vdd*ADC_raw[0]/4095;

		/* Computation of ADC conversions raw data to physical values */
		/* Note: ADC results are transferred into array "ADC_raw"  */
		/*       in the order of their rank in ADC sequencer.                   */
		uhADCChannelToDAC_mVolt    = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_raw[2]);
		uhVrefInt_mVolt            = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_raw[1]);
		wTemperature_DegreeCelsius = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(ADC_raw[0]);
      			
			
		}		
		
		
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
