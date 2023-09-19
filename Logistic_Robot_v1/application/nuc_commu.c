#include "nuc_commu.h"
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "bsp_nuccom.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_info);

//接收到数据
toSTM32_t GimbalRxMsg;

static uint8_t nucinfo_rx_buf[2][NUCINFO_RX_BUF_NUM];

void nuc_control_init(void)
{
    NUC_com_init(nucinfo_rx_buf[0], nucinfo_rx_buf[1], NUCINFO_RX_BUF_NUM);
}



static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_ctrl)
{
	if(sbus_buf[0]==SOF)
	{
		int16_t length = (int16_t)((sbus_buf[2]<<8)+sbus_buf[3]);
		if(sbus_buf[length+4]==END)
		switch (sbus_buf[1])
		{
			case 0x02:{
				nuc_ctrl->x.bytes[0]=sbus_buf[4];
				nuc_ctrl->x.bytes[1]=sbus_buf[5];
				nuc_ctrl->x.bytes[2]=sbus_buf[6];
				nuc_ctrl->x.bytes[3]=sbus_buf[7];
				nuc_ctrl->y.bytes[0]=sbus_buf[8];
				nuc_ctrl->y.bytes[1]=sbus_buf[9];
				nuc_ctrl->y.bytes[2]=sbus_buf[10];
				nuc_ctrl->y.bytes[3]=sbus_buf[11];
				nuc_ctrl->yaw.bytes[0]=sbus_buf[12];
				nuc_ctrl ->yaw.bytes[1]=sbus_buf[13];
				nuc_ctrl ->yaw.bytes[2]=sbus_buf[14];
				nuc_ctrl ->yaw.bytes[3]=sbus_buf[15];
				break;
			};
			case 0x03:{
				nuc_ctrl->cmd=sbus_buf[4];
				break;
			}
		}
	}
}