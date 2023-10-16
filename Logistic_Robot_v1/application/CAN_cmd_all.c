#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "main.h"

#define MOTOR_4015_CAN hcan1
#define ARM_CAN hcan2
#define CHASSIS_CAN hcan2

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**********************************************************************/
/*电机变量参数*/

/*底盘3508电机变量*/
extern CAN_TxHeaderTypeDef  chassis_tx_message;
extern uint8_t              chassis_can_send_data[8];

/*2006电机变量*/
extern CAN_TxHeaderTypeDef  motor_2006_tx_message;
extern uint8_t              motor_2006_can_send_data[8];

/*4015电机变量*/
extern CAN_TxHeaderTypeDef  motor_tx_message;
extern uint8_t              motor_can_send_data[8];

/**********************************************************************/

/*can数据发送函数实现*/

/*底盘3508*/
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}





/*2006*/
void CAN_cmd_2006(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_2006_tx_message.StdId = 0x1FF;
    motor_2006_tx_message.IDE = CAN_ID_STD;
    motor_2006_tx_message.RTR = CAN_RTR_DATA;
    motor_2006_tx_message.DLC = 0x08;
    motor_2006_can_send_data[0] = motor1 >> 8;
    motor_2006_can_send_data[1] = motor1;
    motor_2006_can_send_data[2] = motor2 >> 8;
    motor_2006_can_send_data[3] = motor2;
    motor_2006_can_send_data[4] = motor3 >> 8;
    motor_2006_can_send_data[5] = motor3;
    motor_2006_can_send_data[6] = motor4 >> 8;
    motor_2006_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&ARM_CAN, &motor_2006_tx_message, motor_2006_can_send_data, &send_mail_box);
}



/*4015*/
void CAN_read_pid(uint32_t id)//0x30
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0x30;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = 0x00;
    motor_can_send_data[5] = 0x00;
    motor_can_send_data[6] = 0x00;
    motor_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_4015_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_set_pid_ROM(uint32_t id, uint8_t* pid)//0x32
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0x32;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = pid[0];
    motor_can_send_data[3] = pid[1];
    motor_can_send_data[4] = pid[2];
    motor_can_send_data[5] = pid[3];
    motor_can_send_data[6] = pid[4];
    motor_can_send_data[7] = pid[5];
    HAL_CAN_AddTxMessage(&MOTOR_4015_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}
void CAN_read_ecdData(uint32_t id)//0x90
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0x90;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = 0x00;
    motor_can_send_data[5] = 0x00;
    motor_can_send_data[6] = 0x00;
    motor_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_4015_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_angleControl(uint32_t id, int16_t angle)
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0xA6;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = *(uint8_t*)(&angle);
    motor_can_send_data[5] = *((uint8_t*)(&angle)+1);
    motor_can_send_data[6] = 0x00;
    motor_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_4015_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_delta_angleControl(uint32_t id, int32_t delta_angle)
{
		delta_angle *=100;
		uint32_t send_mail_box;
    motor_tx_message.StdId = id;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0xA7;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = *(uint8_t*)(&delta_angle);
    motor_can_send_data[5] = *((uint8_t*)(&delta_angle)+1);
    motor_can_send_data[6] = *((uint8_t*)(&delta_angle)+2);
    motor_can_send_data[7] = *((uint8_t*)(&delta_angle)+3);
    HAL_CAN_AddTxMessage(&MOTOR_4015_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}


