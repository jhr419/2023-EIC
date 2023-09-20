#include "CAN_cmd_MS4015.h"
#include "CAN_receive.h"
#include "main.h"
#define MOTOR_CAN hcan1

extern CAN_HandleTypeDef hcan1;

extern CAN_TxHeaderTypeDef  motor_tx_message;
extern uint8_t              motor_can_send_data[8];

void CAN_cmd_read_pid(void)//0x30
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_ID;
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
    HAL_CAN_AddTxMessage(&MOTOR_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_cmd_set_pid_ROM(uint8_t* pid)//0x32
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_ID;
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
    HAL_CAN_AddTxMessage(&MOTOR_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}
void CAN_cmd_read_ecdData(void)//0x90
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_ID;
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
    HAL_CAN_AddTxMessage(&MOTOR_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

//iqControl -2000~2000
void CAN_cmd_iqControl(int16_t iqControl)//0xA1
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_ID;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0xA1;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = *(uint8_t*)(&iqControl);
    motor_can_send_data[5] = *((uint8_t*)(&iqControl)+1);
    motor_can_send_data[6] = 0x00;
    motor_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN_cmd_speedControl(int32_t speedControl)
{
		uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_ID;
    motor_tx_message.IDE   = CAN_ID_STD;
    motor_tx_message.RTR   = CAN_RTR_DATA;
    motor_tx_message.DLC   = 0x08;

    motor_can_send_data[0] = 0xA2;
    motor_can_send_data[1] = 0x00;
    motor_can_send_data[2] = 0x00;
    motor_can_send_data[3] = 0x00;
    motor_can_send_data[4] = *(uint8_t*)(&speedControl);
    motor_can_send_data[5] = *((uint8_t*)(&speedControl)+1);
    motor_can_send_data[6] = *((uint8_t*)(&speedControl)+2);
    motor_can_send_data[7] = *((uint8_t*)(&speedControl)+3);
    HAL_CAN_AddTxMessage(&MOTOR_CAN, &motor_tx_message, motor_can_send_data, &send_mail_box);
}
