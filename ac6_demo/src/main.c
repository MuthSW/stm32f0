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
#include "stm32f0308_discovery.h"

void HAL_SYSTICK_Callback(void);

int main(void)
{
	uint32_t button_state, button_state_old;
	uint32_t ticks;

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_On(LED4);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
	//SysTick_Config(1000000);
	HAL_SYSTICK_Config(1000000);	//1sec


	for(;;){
		button_state = BSP_PB_GetState(BUTTON_USER);
		if(button_state_old != button_state){
			button_state_old = button_state;
			if (HAL_GetTick() - ticks > 9){
				ticks = HAL_GetTick();
				BSP_LED_Toggle(LED3);
			}
		}
	}
}

void HAL_SYSTICK_Callback(void){
	uint32_t ticks = HAL_GetTick();
	;
}

