#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "servo.h"
#include "commu_task.h"
#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "gear_motor_ctrl.h"
#include "CAN_cmd_all.h"
extern car_data_s  my_car_data;
extern arm_cmd_t 	my_arm;
extern servo_t servo[8];
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
//�Ż����ո߹�ͷ����ת��
/*
ָ��һ ����,��ס�ȿ��Ƶ�����Ϻ�˳ʱ����ת����ʱ��������ת����λ��������һ���̶ȣ���������
*/
void cmd_arm_rst()
{
	startM2006Monitor();
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
}

/*
ָ���
ARM_TO_CODE
�´��鿴��ά��
*/
void cmd_arm_to_code()
{
	set_M2006_rotate_rounds(1,ROUNDS_TOP_TO_BOTTOM);
	servo_angle_ctrl(&servo[1],ANGLE_CAMERA_TO_CODE);
}

/*
ָ����
ARM_TO_STUFF
*/
void cmd_arm_to_stuff()
{
	servo_angle_ctrl(&servo[1],ANGLE_CAMERA_TO_STUFF);
	set_M2006_rotate_rounds(1,ROUNDS_BOTTOM_TO_TOP);
}

/*
ָ����
ARM_GRAB_MATERIAL
��������ץȡ������������ת�������ţ�������ת
*/
void cmd_arm_grab_material()
{
	//�½�
	set_M2006_rotate_rounds(1,ROUNDS_GRAB_STUFF_MATERIAL);
	HAL_Delay(1000);
	//ץȡ
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
	//��
	set_M2006_rotate_rounds(1,-ROUNDS_GRAB_STUFF_MATERIAL);
	HAL_Delay(1000);
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
	//��
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(2000);
	//��
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//��
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	CAN_angleControl(CAN_M1_ID,120);
	HAL_Delay(2000);
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	my_car_data.stuff_num++;
}

/*
ָ����
ARM_PLACE_GROUND
ת���£�ץ������ת���£��ţ���
*/
void cmd_arm_place_ground()
{
		//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
		//�½�
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
		//ץȡ
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
		//��
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	HAL_Delay(1000);
		//��
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_GROUND);
	HAL_Delay(2000);
	//��
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//ת����
	
	//��
		set_M2006_rotate_rounds(1,-ROUNDS_PLACE_GROUND);
	my_car_data.stuff_num--;
}

/*
ָ����
ARM_GRAB_GROUND
*/
void cmd_arm_grab_ground()
{
	//�½�
	set_M2006_rotate_rounds(1,ROUNDS_GRAB_STUFF_GROUND);
	HAL_Delay(1000);
	//ץȡ
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
	//��
	set_M2006_rotate_rounds(1,-ROUNDS_GRAB_STUFF_GROUND);
	HAL_Delay(1000);
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
	//��
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(2000);
	//��
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//��
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(2000);
	//ת����
	
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	my_car_data.stuff_num++;
}

/*
ָ����
ARM_PLACE_STUFF
*/
void cmd_arm_place_stuff()
{
	set_M2006_rotate_rounds(0,ROUNDS_TURN_IN);
	HAL_Delay(1000);
		//�½�
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
		//ץȡ
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	HAL_Delay(800);
		//��
	set_M2006_rotate_rounds(1,-ROUNDS_PLACE_PLATE);
	HAL_Delay(1000);
	//ת
	set_M2006_rotate_rounds(0,ROUNDS_TURN_OUT);
	HAL_Delay(1000);
		//��
	set_M2006_rotate_rounds(1,ROUNDS_PLACE_STUFF);
	HAL_Delay(2000);
	//��
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_OPEN);
	HAL_Delay(800);
	//ת����
	
	//��
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
					cmd_arm_grab_material();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM_PLACE_GROUND:
				{
					cmd_arm_place_ground();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM_GRAB_GROUND:
				{
					cmd_arm_grab_ground();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM_PLACE_STUFF:
				{
					cmd_arm_place_stuff();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM_END:
				{
					cmd_arm_place_stuff();
					//CAN_delta_angleControl(12000);
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
