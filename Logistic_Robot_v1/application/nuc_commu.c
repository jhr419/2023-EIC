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

//void USART6_IRQHandler (void)
//{
//	  
//		if (__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)) // 接收到数据//接受中断
//    {
//			  
//        __HAL_UART_CLEAR_PEFLAG(&huart6);
//    }
//		else if (__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE))
//		{
//			static int this_time_rx_len = 0;
//			__HAL_UART_CLEAR_PEFLAG(&huart6);
//			if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            // disable DMA
//            // 失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            // get receive data length, length = set_data_length - remain_length
//            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            // reset set_data_lenght
//            // 重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

//            // set memory buffer 1
//            // 设定缓冲1
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

//            // enable DMA
//            // 使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
//            {
//								
//                sbus_to_nucCtrl(nucinfo_rx_buf[0], &GimbalRxMsg);
//                usart_printf("%f,%f,%f",GimbalRxMsg.x,GimbalRxMsg.y,GimbalRxMsg.yaw);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            // disable DMA
//            // 失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);

//            // get receive data length, length = set_data_length - remain_length
//            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

//            // reset set_data_lenght
//            // 重新设定数据长度
//            hdma_usart6_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

//            // set memory buffer 0
//            // 设定缓冲??0
//            hdma_usart6_rx.Instance->CR &= ~(DMA_SxCR_CT);

//            // enable DMA
//            // 使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);

//            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
//            {
//                // 处理nuc传来的数
//							//早期处理，通过闪灯检测是否
//                //__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0x00000000);
//								
//                sbus_to_nucCtrl(nucinfo_rx_buf[1], &GimbalRxMsg);
//                usart_printf("%f,%f,%f",GimbalRxMsg.x,GimbalRxMsg.y,GimbalRxMsg.yaw);
//            }
//        }
//    }
//		}

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