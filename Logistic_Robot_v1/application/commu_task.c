#include "commu_task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_usart.h"
//按键中断开始后发送正确的stuffnum，上位机开始发送数据，比赛开始
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6; 
extern UART_HandleTypeDef huart8;

void uart8_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
		
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

		HAL_UART_Transmit_DMA(&huart8, tx_buf, len);

}


move_cmd_t my_move;
arm_cmd_t my_arm;

void my_uart6_send_data(uint8_t *tdata,uint16_t tnum){
        while(HAL_DMA_GetState(&hdma_usart6_tx) == HAL_DMA_STATE_BUSY) HAL_Delay(1);
        HAL_UART_Transmit_DMA(&huart6,tdata,tnum);
}



uint8_t my_uart6_redata[40];
void my_uart6_enable_inpterr(){
    HAL_UART_Receive_DMA(&huart6,my_uart6_redata,34);
    
}
void  encode(uint8_t* a,uint8_t cmd,uint16_t length,float x, float y, float angle, int stuffnum)
{
	a[0]=0xA5;
	memcpy(&a[1],&cmd,1);
	memcpy(&a[2],&length,2);
	memcpy(&a[4],&x,4);
	memcpy(&a[8],&y,4);
	memcpy(&a[12],&angle,4);
	memcpy(&a[16],&stuffnum,2);
	a[18]=0x5A;
}
void decode02(uint8_t* data)
{
	my_move.x_rel.bytes[0]=data[4];
	my_move.x_rel.bytes[1]=data[5];
	my_move.x_rel.bytes[2]=data[6];
	my_move.x_rel.bytes[3]=data[7];
	for(int i=0;i<4;i++)
	{
		my_move.y_rel.bytes[i]=data[8+i];
	}
	memcpy(&(my_move.angle_rel),&(data[12]),4);
}
void decode03(uint8_t* data)
{
	my_arm.act_id=data[4];
}

car_data my_car_data;
uint8_t my_uart8_redata[100];
void decode_action(uint8_t* data){
	memcpy(&(my_car_data.yaw),&data[2],4);
	memcpy(&(my_car_data.pitch),&data[6],4);
	memcpy(&(my_car_data.roll),&data[10],4);
	memcpy(&(my_car_data.x),&data[14],4);
	memcpy(&(my_car_data.y),&data[18],4);
	memcpy(&(my_car_data.w),&data[22],4);
}

void my_uart8_enable_inpterr(){
    HAL_UART_Receive_DMA(&huart8,my_uart8_redata,60);
    
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART8)
    {

			for(int i=0;i<60;i++)
			{
				if(i+27<60)
				{
					 if(my_uart8_redata[i]==0x0D&&my_uart8_redata[i+1]==0x0A&&my_uart8_redata[i+26]==0x0A&&my_uart8_redata[i+27]==0x0D)
					 {
							decode_action(&(my_uart8_redata[i]));
					 }
		    }
				else
				{
					break;
				}
		  }
		}
    if(huart->Instance == USART6)
    {
			for(int i=0;i<34;i++)
			{
				if(my_uart6_redata[i]==0xA5&&(my_uart6_redata[i+1]!=0x5A)&&(my_uart6_redata[i+2]+i+4<34)&&my_uart6_redata[my_uart6_redata[i+2]+i+4]==0x5A)
				{
					
					switch (my_uart6_redata[i+1])
					{
						case 0x02:
						{
							decode02(&my_uart6_redata[i]);
							break;
						}
						case 0x03:
						{
							decode03(&my_uart6_redata[i]);
							break;
						}
					}
				}
			}
		}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    // UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
the HAL_UART_ErrorCallback could be implemented in the user file
*/
    if( huart->ErrorCode & HAL_UART_ERROR_ORE )//Overflow error
   {
       uint32_t temp = huart->Instance->SR;
       temp = huart->Instance->DR;
   }
}

void commu_task(void const* argument){
	my_uart8_enable_inpterr();
	my_uart6_enable_inpterr();
	
	uart8_printf("ACT0");
	while(1){
		
		osDelay(10);
	}
}
