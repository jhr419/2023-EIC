#include "button.h"
#include "commu_task.h"

static uint8_t button_flag = 0;


void EXTI0_IRQHandler(void)
{
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
		{
			button_flag = 1;
			uart
		}
	}
}