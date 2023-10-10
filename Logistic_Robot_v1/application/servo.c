#include "servo.h"
#include "string.h"
#include "tim.h"
#include "main.h"

#define MAX_ANGLE_COMPARE
#define MIN_ANGLE_COMPARE
#define UESR_HTIM1 htim2
#define UESR_HTIM2 htim8
#define COMPARE_PER_ANGLE (fp32)(2500-500)/270

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

servo_t servo[8];

void servo_init(void)
{
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	for(int i=0;i<4;i++)
	{
		servo[i].htim = UESR_HTIM1;
		servo[i].id = i+1;
		servo[i+4].htim = UESR_HTIM2;
		servo[i+4].id = i+5;
	}
	
	servo[0].channel = TIM_CHANNEL_1;
	servo[1].channel = TIM_CHANNEL_2;
	servo[2].channel = TIM_CHANNEL_3;
	servo[3].channel = TIM_CHANNEL_4;
	servo[4].channel = TIM_CHANNEL_1;
	servo[5].channel = TIM_CHANNEL_2;
	servo[6].channel = TIM_CHANNEL_3;
	servo[7].channel = TIM_CHANNEL_4;
}

void single_servo_ctrl(servo_t* servo, uint16_t angle)
{
	TIM_HandleTypeDef htim = servo->htim;
	uint32_t channel = servo->channel;
	__HAL_TIM_SetCompare(&htim, channel, COMPARE_PER_ANGLE * angle);
}

