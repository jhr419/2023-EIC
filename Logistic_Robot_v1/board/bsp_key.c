#include "bsp_key.h"

uint8_t exit_flag = 0;
uint8_t rising_falling_flag;
void HAL_GPIO_EXIT_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY_Pin)
	{
		if(exit_flag == 0)
		{
			exit_flag = 1;
			rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
		}
	}
}
