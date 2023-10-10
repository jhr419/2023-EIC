#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "struct_typedef.h"

#define CLAW_MAX_ANGLE_COMPARE 1000
#define CLAW_MIN_ANGLE_COMPARE 1200
#define UESR_HTIM1 htim2
#define UESR_HTIM2 htim8
#define COMPARE_PER_ANGLE 1

typedef struct
{
	TIM_HandleTypeDef htim;
	uint32_t channel;
	uint8_t id;
}servo_t;

void servo_init(void);
void single_servo_ctrl(servo_t* servo, uint16_t angle);
#endif
