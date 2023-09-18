#include "CAN_receive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

static motor_measure_t motor;
static motor_pid_t motor_pid;
static motor_ecd_data_t motor_ecd_data;


static CAN_TxHeaderTypeDef  motor_tx_message;
static uint8_t              motor_can_send_data[8];

uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
   
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_M1_ID:
        {
					//get_motor_ecd_data(&motor_ecd_data, rx_data);
          get_motor_measure(&motor,rx_data);
					break;
        }
    }
}

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
const motor_measure_t *get_motor_measure_point(void)
{
    return &motor;
}

const motor_pid_t *get_motor_pid_point(void)
{
    return &motor_pid;
}

const motor_ecd_data_t* get_motor_ecd_data_point(void)
{
    return &motor_ecd_data;
}
