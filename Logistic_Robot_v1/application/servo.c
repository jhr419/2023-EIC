#include "servo.h"
#include "string.h"
#include "tim.h"
#include "main.h"



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

servo_t* get_servo_point(int i){
	return &servo[i];
}
void single_servo_ctrl(servo_t* servo, uint16_t pwm)
{
	TIM_HandleTypeDef htim = servo->htim;
	uint32_t channel = servo->channel;
	//uint32_t pwm = COMPARE_PER_ANGLE * angle;
	__HAL_TIM_SetCompare(&htim, channel, pwm);
}

void arm_ready(void)
{
		single_servo_ctrl(&servo[0], 600);
		single_servo_ctrl(&servo[1],300);
		single_servo_ctrl(&servo[2],930);
  	single_servo_ctrl(&servo[3],1000);
}

void arm_catch(void)
{
		single_servo_ctrl(&servo[0], 580);//抓取
		single_servo_ctrl(&servo[1],350);
		single_servo_ctrl(&servo[2],910);
		single_servo_ctrl(&servo[3],1160);
}

void arm_put_ready(void)
{
			single_servo_ctrl(&servo[0], 1050);//转盘过程点
			single_servo_ctrl(&servo[1],330);
			single_servo_ctrl(&servo[2],450);
			single_servo_ctrl(&servo[3],1160);
}

void arm_up(void)
{
		single_servo_ctrl(&servo[0], 1058); //正上方
		single_servo_ctrl(&servo[1],350);
		single_servo_ctrl(&servo[2],430);
		single_servo_ctrl(&servo[3],1160);
}

void arm_put(void)
{
		single_servo_ctrl(&servo[0], 1058); //正上方
		single_servo_ctrl(&servo[1],350);
		single_servo_ctrl(&servo[2],430);
		single_servo_ctrl(&servo[3],1000);
}

void arm_put_stuff(void)
{
	single_servo_ctrl(&servo[0], 580);
	single_servo_ctrl(&servo[1],600);
	single_servo_ctrl(&servo[2],600);
	single_servo_ctrl(&servo[3],1160);
	
	HAL_Delay(2000);
	
	single_servo_ctrl(&servo[0], 500);//抓取
	single_servo_ctrl(&servo[2],1200);
	HAL_Delay(500);
	single_servo_ctrl(&servo[1],330);

	single_servo_ctrl(&servo[3],1160); 
	
	HAL_Delay(2000);
	
	
	
	single_servo_ctrl(&servo[0], 300);//抓取
	single_servo_ctrl(&servo[1],400);
	single_servo_ctrl(&servo[2],1100);
	single_servo_ctrl(&servo[3],1160); 
	
	HAL_Delay(1000);
	single_servo_ctrl(&servo[0], 300);//抓取
	single_servo_ctrl(&servo[1],400);
	single_servo_ctrl(&servo[2],1100);
	
	single_servo_ctrl(&servo[3],1000);
	
	
}	

void servo_angle_ctrl(servo_t* servo, uint16_t angle)
{
	TIM_HandleTypeDef htim = servo->htim;
	uint32_t channel = servo->channel;
	uint32_t pwm = COMPARE_PER_ANGLE * angle;
	__HAL_TIM_SetCompare(&htim, channel, 500+pwm);
}
