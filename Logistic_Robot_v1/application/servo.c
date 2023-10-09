#include "servo.h"
#include "string.h"
#include "main.h"

//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 705);		//中 //205 为45度 
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 910);		
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1115);	//左极限
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 295);		//右极限
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 700);		//下极限
//	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 300);		//上极限
#define MAX_ANGLE 180
#define MIN_ANGLE 0
#define PWM_VALUE_PER_X_ANGLE (float)(205/45)
#define PWM_VALUE_PER_Y_ANGLE (float)(400/90)
extern float aX_data_temp;
extern float aY_data_temp;
extern float aZ_data_temp;
extern float wX_data_temp;
extern float wY_data_temp;
extern float wZ_data_temp; 
extern float PitchY_data_temp;
extern float YawZ_data_temp;

float x_angle;
float y_angle;
extern TIM_HandleTypeDef htim2;

void servo_pid_init(void)
{
	 
}

float angle_limiter(float x)
{
	if(x >= MAX_ANGLE)
		return MAX_ANGLE;
	else if(x <= MIN_ANGLE)
		return MIN_ANGLE;
	return x;
}

float x_angle_range(float x)
{
	if(x >180 && x<=360)
		return x-360;
	return x;
}

float y_angle_range(float y)
{
	if(y<=0)
		return 0;
	if(y>=90&&y<=180)
		return 90;
	if(y>180&&y<=360)
		return 0;
	return y;
}
void x_anlge_ctrl(float angle)
{
	uint16_t compare_temp;
	compare_temp = angle *  PWM_VALUE_PER_X_ANGLE;
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, compare_temp+320);
}
void y_anlge_ctrl(float angle)
{
	uint16_t compare_temp;
	compare_temp = angle *  PWM_VALUE_PER_Y_ANGLE;
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 700 - compare_temp);
}
void ctrl_servo(void)
{
	x_angle = x_angle_range(YawZ_data_temp)+90.0;
	y_angle = PitchY_data_temp;
	x_anlge_ctrl(x_angle);
	y_anlge_ctrl(y_angle);
}
