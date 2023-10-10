#include "CAN_cmd_2006.h"
#include "main.h"

extern CAN_HandleTypeDef hcan2;


extern CAN_TxHeaderTypeDef  motor_2006_tx_message;
extern uint8_t              motor_2006_can_send_data[8];


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

    HAL_CAN_AddTxMessage(&hcan2, &motor_2006_tx_message, motor_2006_can_send_data, &send_mail_box);
}
