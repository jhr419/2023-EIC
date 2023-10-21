#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "struct_typedef.h"

#define CLAW_MAX_ANGLE_COMPARE 1000 //ÕÅ×¦
#define CLAW_MIN_ANGLE_COMPARE 1200	//ºÏ×¦
#define UESR_HTIM1 htim2
#define UESR_HTIM2 htim8
#define COMPARE_PER_ANGLE (2500 - 500)/270

typedef struct
{
	TIM_HandleTypeDef htim;
	uint32_t channel;
	uint8_t id;
}servo_t;

extern servo_t servo[8];
extern servo_t* get_servo_point(int i);
void servo_init(void);
void single_servo_ctrl(servo_t* servo, uint16_t pwm);
void arm_ready(void);
void arm_catch(void);
void arm_put_ready(void);
void arm_up(void);
void arm_put(void);
void arm_put_stuff(void);
void servo_angle_ctrl(servo_t* servo, uint16_t angle);
#endif
