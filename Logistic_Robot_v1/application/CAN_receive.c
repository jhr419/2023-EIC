#include "CAN_receive.h"
#include "main.h"
#define MOTOR_CAN hcan1

extern CAN_HandleTypeDef hcan1;

static motor_3508_measure_t		motor_3508[4];
static motor_6020_measure_t 	motor_6020[2];
static motor_2006_measure_t		motor_2006[2];

static motor_4015_measure_t 	motor_4015;
static motor_4015_pid_t 			motor_4015_pid;
static motor_4015_ecd_data_t 	motor_4015_ecd_data;


CAN_TxHeaderTypeDef  chassis_tx_message;
uint8_t              chassis_can_send_data[8];

CAN_TxHeaderTypeDef  motor_6020_tx_message;
uint8_t              motor_6020_can_send_data[8];

CAN_TxHeaderTypeDef  motor_2006_tx_message;
uint8_t              motor_2006_can_send_data[8];

CAN_TxHeaderTypeDef  motor_tx_message;
uint8_t              motor_can_send_data[8];

/*------------------------------------------------------------------------*/
/*
	CAN接收中断回调
*/



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
          get_motor_4015_measure(&motor_4015,rx_data);
					break;
        }
				case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_3508_measure(&motor_3508[i], rx_data);
            break;
        }
				
				case CAN_2006_M1_ID:
        case CAN_2006_M2_ID:        
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_2006_M1_ID;
            get_motor_2006_measure(&motor_2006[i], rx_data);
            break;
        }
				case CAN_6020_M1_ID:
        case CAN_6020_M2_ID:
				{
						static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_6020_M1_ID;
            get_motor_6020_measure(&motor_6020[i], rx_data);
            break;
				}
					
				
        default:
        {
            break;
        }
				
    }
}

/*------------------------------------------------------------------------*/
/*
	3508指针
*/

const motor_3508_measure_t *get_motor_3508_measure_point(uint8_t i)
{
    return &motor_3508[(i & 0x03)];
}

/*
	6020指针
*/

const motor_6020_measure_t *get_motor_6020_measure_point(uint8_t i)
{
    return &motor_6020[(i & 0x03)];
}


/*------------------------------------------------------------------------*/
/*
	2006指针
*/

const motor_2006_measure_t *get_motor_2006_measure_point(uint8_t i)
{
    return &motor_2006[(i & 0x03)];
}

/*------------------------------------------------------------------------*/
/*
	4015指针
*/

const motor_4015_measure_t *get_motor_4015_measure_point(void)
{
    return &motor_4015;
}

const motor_4015_pid_t *get_motor_4015_pid_point(void)
{
    return &motor_4015_pid;
}

const motor_4015_ecd_data_t* get_motor_4015_ecd_data_point(void)
{
    return &motor_4015_ecd_data; 
}
