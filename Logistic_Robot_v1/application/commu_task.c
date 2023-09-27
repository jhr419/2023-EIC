/**
 * encoding:GB2312
 * @file commu_task.c
 * @author Brandon
 * @brief  收取全场定位数据并转发給上位机
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2023-9-25     Brandon         1. 增加update函数
 *  V1.0.1     2023-9-26     Brandon         增加按键清零
 */
#include "commu_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "arm_math.h"
#include <stdarg.h>
#include <stdio.h>
#include "bsp_usart.h"
#include "CAN_cmd_3508.h"
//按键中断开始后发送正确的stuffnum，上位机开始发送数据，比赛开始
#define ACTION_DISTANCE_ERROR -57.98275606 //全场定位中心与中心安装的差错长,之后再改
#define ACTION_ANGLE_ERROR    0  //角度差错值

extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6; 
extern UART_HandleTypeDef huart8;

void uart7_printf(const char *fmt,...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	
}

void uart8_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
		
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

		HAL_UART_Transmit(&huart8, tx_buf, len, 100);
    uart8_tx_dma_enable(tx_buf, len);

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
	my_move.vx_err.bytes[0]=data[4];
	my_move.vx_err.bytes[1]=data[5];
	my_move.vx_err.bytes[2]=data[6];
	my_move.vx_err.bytes[3]=data[7];
	my_move.vy_err.bytes[0]=data[8];
	my_move.vy_err.bytes[1]=data[9];
	my_move.vy_err.bytes[2]=data[10];
	my_move.vy_err.bytes[3]=data[11];
	my_move.vw_err.bytes[0]=data[12];
	my_move.vw_err.bytes[1]=data[13];
	my_move.vw_err.bytes[2]=data[14];
	my_move.vw_err.bytes[3]=data[15];
//	for(int i=0;i<4;i++)
//	{
//		my_move.vy_err.bytes[i]=data[8+i];
//	}
//	memcpy(&(my_move.vw_err),&(data[12]),4);
}
void decode03(uint8_t* data)
{
	my_arm.act_id=data[4];
}

action_data my_action_data;
uint8_t my_uart8_redata[100];
void decode_action(uint8_t* data){
	memcpy(&(my_action_data.yaw),&data[2],4);
	memcpy(&(my_action_data.pitch),&data[6],4);
	memcpy(&(my_action_data.roll),&data[10],4);
	memcpy(&(my_action_data.x),&data[14],4);
	memcpy(&(my_action_data.y),&data[18],4);
	memcpy(&(my_action_data.w),&data[22],4);
}

void my_uart8_enable_inpterr(){
    HAL_UART_Receive_DMA(&huart8,my_uart8_redata,60);
}
int action_count=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART8)
    {
			action_count=1;
			//检测完毕
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
//			for(int i=0;i<34;i++)
//			{
//				uart8_printf("%x ", my_uart6_redata[i]);
//			}
//			uart8_printf("\r\n");
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
/*更新Y坐标*/
void Stract(char str[],char source[],int num)
{
	int i=0, j=0;
	while(str[i]!='\0') i++;
	
	for(j=0;j<num;j++)
	{
		str[i++]=source[j];
	}
}
void Update_position(char c,float NEW){
	char Update[8]="ACT";
	if(c=='x'||c=='X')
		Update[3]='X';
	else if(c=='y'||c=='Y')
		Update[3]='Y';
	
	static union
	{
		float f;
		char data[4];
	}NEW_set;
	
	NEW_set.f=NEW;
	
  Stract(Update,NEW_set.data,4);
	
	HAL_UART_Transmit(&huart8, (const uint8_t*)Update , 8,100);
	uart8_tx_dma_enable((uint8_t *)Update,8);
}
car_data my_car_data;
void action_to_car(){
	my_car_data.x = my_action_data.x.data - ACTION_DISTANCE_ERROR * arm_sin_f32(my_action_data.yaw.data*2*PI/360);
	my_car_data.y = my_action_data.y.data + ACTION_DISTANCE_ERROR * arm_cos_f32(my_action_data.yaw.data*2*PI/360);
	my_car_data.yaw = my_action_data.yaw.data;
}

void cmd_packed_tx(UART_HandleTypeDef *huart, uint8_t* raw_data)
{
	HAL_UART_Transmit(huart, raw_data, sizeof(raw_data)/sizeof(uint8_t), 100);
	usart6_tx_dma_enable(raw_data, sizeof(raw_data)/sizeof(uint8_t));
}

uint8_t exit_flag = 0;
uint8_t rising_falling_flag ;
void commu_task(void const* argument){
	my_uart8_enable_inpterr();
	my_uart6_enable_inpterr();
  
	uart8_printf("ACT0");
	Update_position('Y',ACTION_DISTANCE_ERROR);
	uint8_t tx_msg[19];
	while(1){
			if(rising_falling_flag!=HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){
				rising_falling_flag =HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
				if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)==1)
				{
				  uart8_printf("ACT0");
					Update_position('Y',ACTION_DISTANCE_ERROR);
				}
			}
		
		if(action_count)
		{
			HAL_GPIO_WritePin(ACTION_LED_GPIO_Port, ACTION_LED_Pin,GPIO_PIN_SET);
			action_count=0;
			
		}
		
		encode(tx_msg,0x01,14,my_car_data.x,my_car_data.y,my_car_data.yaw,100);
		
		cmd_packed_tx(&huart6, tx_msg);
		
		osDelay(20);
		HAL_GPIO_WritePin(ACTION_LED_GPIO_Port, ACTION_LED_Pin,GPIO_PIN_RESET);
	}
}
