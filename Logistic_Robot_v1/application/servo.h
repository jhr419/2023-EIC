#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "struct_typedef.h"

typedef struct
{
	TIM_HandleTypeDef htim;
	uint32_t channel;
	uint8_t id;
}servo_t;

void servo_init(void);
void single_servo_ctrl(servo_t* servo, uint16_t angle);
#endif
