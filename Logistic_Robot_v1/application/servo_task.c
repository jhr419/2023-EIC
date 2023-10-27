#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "commu_task.h"
#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "motor_ctrl.h"
#include "gear_motor_ctrl.h"
#include "CAN_cmd_all.h"
extern car_data_s  my_car_data;
extern arm_cmd_t 	my_arm;
extern servo_t servo[8];

static uint8_t cmd_num=0;
//static uint8_t last_arm_cmd;
/*
ARM_RST = 0x01,
ARM_TO_CODE = 0x02,
ARM_TO_STUFF = 0x03,
ARM_GRAB_MATERIAL = 0x04,
ARM_PLACE_GROUND = 0x05,
ARM_GRAB_GROUND = 0x06,
ARM_PLACE_STUFF = 0x07
*/
//优化，刚高过头顶就转向
/*
指令一 重置,锁住先控制电机向上和顺时针旋转，随时检测电流，转至限位处电流大到一定程度，发送重置
*/
void cmd_arm_rst()
{
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	cmd_num = 1;
}

/*
指令二
ARM_TO_CODE
下垂查看二维码
*/
void cmd_arm_to_code()
{
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	HAL_Delay(1000);
	set_M2006_rotate_rounds(1,ROUNDS_TOP_TO_BOTTOM);

	cmd_num = 2;
}

/*
指令三
ARM_TO_STUFF
*/
void cmd_arm_to_stuff()
{
	servo_angle_ctrl(&servo[1],ANGLE_CAMERA_TO_STUFF);
	set_M2006_rotate_rounds(1,ROUNDS_BOTTOM_TO_TOP);
	
	cmd_num = 3;
}

/*
指令四
ARM_GRAB_MATERIAL
垂下来，抓取，升上来，旋转，降，放，升，回转
*/
void cmd_arm_grab_material()
{
	//下降
	set_M2006_rotate_rounds(1,ROUNDS_GRAB_STUFF_MATERIAL);
	HAL_Delay(1000);
	//抓取
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
	//上
	set_M2006_rotate_rounds(1,-ROUNDS_GRAB_STUFF_MATERIAL);
	HAL_Delay(1000);
	//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
	//降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//放
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//升
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//转
	angle_m6020_to_next();
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	
	my_car_data.stuff_num++;
	
	cmd_num = 4;
}

/*
指令五
ARM_PLACE_GROUND
转，下，抓，升，转，下，放，升
*/
void cmd_arm_place_ground()
{
		//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
		//下降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
		//抓取
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
		//上
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	HAL_Delay(1000);
		//降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_GROUND);
	HAL_Delay(2000);
	//放
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_GROUND);
	HAL_Delay(1000);
	//转盘子
	angle_m6020_to_next();
	//升
	
	my_car_data.stuff_num--;
}

/*
指令六
ARM_GRAB_GROUND
*/
void cmd_arm_grab_ground()
{
	//下降
	set_M2006_rotate_rounds(1,ROUNDS_GRAB_STUFF_GROUND);
	HAL_Delay(1500);
	//抓取
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
	//上
	set_M2006_rotate_rounds(1,-ROUNDS_GRAB_STUFF_GROUND);
	HAL_Delay(1000);
	//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1500);
	//降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//放
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//升
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//转盘子
	angle_m6020_to_next();
	//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	my_car_data.stuff_num++;
}

/*
指令七
ARM_PLACE_STUFF
*/
void cmd_arm_place_stuff()
{
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
		//下降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
		//抓取
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
		//上
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//转
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	HAL_Delay(1000);
		//降
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_STUFF);
	HAL_Delay(1000);
	//放
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//转盘子
	angle_m6020_to_next();
	//升
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_STUFF);
	my_car_data.stuff_num--;
}

void servo_task(void const* argument){
	
	while(1){
		
		if(my_arm.act_id!=0)
		{
			switch(my_arm.act_id){
				case ARM_RST:
				{
					cmd_arm_rst();
					break;
				}
				case ARM_TO_CODE:
				{
					cmd_arm_to_code();
					break;
				}
				case ARM_TO_STUFF:
				{
				  cmd_arm_to_stuff();
					break;
				}
				case ARM_GRAB_MATERIAL:
				{
					if(my_car_data.stuff_num<3)
						cmd_arm_grab_material();
					break;
				}
				case ARM_PLACE_GROUND:
				{
					if(my_car_data.stuff_num>0)
						cmd_arm_place_ground();
					break;
				}
				case ARM_GRAB_GROUND:
				{
					if(my_car_data.stuff_num<3)
						cmd_arm_grab_ground();
					break;
				}
				case ARM_PLACE_STUFF:
				{
					if(my_car_data.stuff_num>0)
					 cmd_arm_place_stuff();
					break;
				}
				case ARM_END:
				{
					cmd_arm_place_stuff();
					break;
				}
				default :
					break;
			}
			my_arm.act_id = 0;
	  }
		osDelay(20);
	}
}
