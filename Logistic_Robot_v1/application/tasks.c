#include "main.h"
#include "tasks.h"
#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "gear_motor_ctrl.h"
#include "servo.h"

#define SERVO_CAM &servo[0]
#define SERVO_CALW &servo[1]
#define CAM_POS_1 120
#define CAM_POS_2 120
#define CLAW_POS_OPEN 120
#define CLAW_POS_CLOSE 120
extern servo_t servo[8];

void task0(void)
{
	servo_init();
	CAN_angleControl(CAN_M1_ID,0);
	refresh_M2006_ctrl();//armµ×ÅÌ
	refresh_M2006_ctrl();//Éý½µ
	servo_angle_ctrl(SERVO_CAM, CAM_POS_1);
}

void task1(void)
{
	set_M2006_rotate_rounds(18,0);
	HAL_Delay(100);
	servo_angle_ctrl(SERVO_CALW, CLAW_POS_OPEN);
	HAL_Delay(100);
}

void task2(void)
{
	servo_angle_ctrl(SERVO_CAM, CAM_POS_2);
	HAL_Delay(100);
	set_M2006_rotate_rounds(0,36);
	HAL_Delay(100);
}

void task3(void)
{
	set_M2006_rotate_rounds(0,-36);
	HAL_Delay(100);
	servo_angle_ctrl(SERVO_CALW, CLAW_POS_CLOSE);
	HAL_Delay(100);
}

void task4(void)
{
	set_M2006_rotate_rounds(0,36);
	set_M2006_rotate_rounds(-18,0);
	HAL_Delay(100);
	set_M2006_rotate_rounds(0,-36);
	servo_angle_ctrl(SERVO_CALW, CLAW_POS_OPEN);
	HAL_Delay(100);
	set_M2006_rotate_rounds(0,36);
	HAL_Delay(100);
}

void task5(void)
{
	CAN_delta_angleControl(CAN_M1_ID,120);
}