/**
 * encoding:GB2312
 * @file commu_task.c
 * @author Brandon
 * @brief  收取全场定位数据并转发給上位机
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2023-9-2
 Brandon         增加按键清零
 *  V2.0.0     2023-9-28     Brandon         中心矫正初版完成
 */

#include "commu_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "servo.h"
#include "arm_math.h"
#include <stdarg.h>
#include <stdio.h>
#include "servo_task.h"
#include "bsp_usart.h"
#include "gear_motor_ctrl.h"
#include "motor_ctrl.h"
#include "CAN_cmd_all.h"
//按键中断开始后发送正确的stuffnum，上位机开始发送数据，比赛开始
#define ACTION_DISTANCE_ERROR -56.48275606 //全场定位中心与中心安装的差错长,之后再改
#define ACTION_ANGLE_ERROR    0  //角度差错值


uint8_t my_uart8_redata[100];
uint8_t my_uart6_redata[40];

action_data my_action_data;//全场定位数据
move_cmd_t 	my_move;//上位机給车的移动数据
arm_cmd_t 	my_arm;//上位机給车的指令
car_data_s 		my_car_data;//相对车中心的位姿

extern pid_t chassis_v_pid[2];
extern pid_t chassis_w_pid;
extern fp32 v[3];
extern fp32 point_decoded[2];

uint8_t exit_flag = 0;
uint8_t rising_falling_flag ;
uint8_t rising_falling_flag1 ;
int action_count=0;

extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart6; 


void uart7_printf(const char *fmt,...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap, fmt);
	
	len = vsprintf((char* )tx_buf, fmt, ap);
	
	va_end(ap);
	
	HAL_UART_Transmit(&huart7, tx_buf, len, 100);
  uart7_tx_dma_enable(tx_buf, len);
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



void my_uart6_send_data(uint8_t *tdata,uint16_t tnum){
        while(HAL_DMA_GetState(&hdma_usart6_tx) == HAL_DMA_STATE_BUSY) HAL_Delay(1);
        HAL_UART_Transmit_DMA(&huart6,tdata,tnum);
}




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
	my_move.x_goal.bytes[0]=data[4];
	my_move.x_goal.bytes[1]=data[5];
	my_move.x_goal.bytes[2]=data[6];
	my_move.x_goal.bytes[3]=data[7];
	my_move.y_goal.bytes[0]=data[8];
	my_move.y_goal.bytes[1]=data[9];
	my_move.y_goal.bytes[2]=data[10];
	my_move.y_goal.bytes[3]=data[11];
	my_move.w_goal.bytes[0]=data[12];
	my_move.w_goal.bytes[1]=data[13];
	my_move.w_goal.bytes[2]=data[14];
	my_move.w_goal.bytes[3]=data[15];
}
void decode03(uint8_t* data)
{
	my_arm.act_id=data[4];
}


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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART8)
    {
			action_count=1;
			for(int i=0;i<33;i++)
			{
				if(i+27<60)
				{
					 if(my_uart8_redata[i]==0x0D&&my_uart8_redata[i+1]==0x0A&&my_uart8_redata[i+26]==0x0A&&my_uart8_redata[i+27]==0x0D)
					 {
							decode_action(&(my_uart8_redata[i]));
					 }
		    }
				else
					break;
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
//这里无用
void action_to_car(){
//	my_car_data.x = my_action_data.x.data*arm_cos_f32(my_action_data.yaw.data * PI / 180.0) - my_action_data.y.data*arm_sin_f32(my_action_data.yaw.data * PI / 180.0);
//	my_car_data.y = my_action_data.x.data*arm_sin_f32(my_action_data.yaw.data * PI / 180.0) + my_action_data.y.data*arm_cos_f32(my_action_data.yaw.data * PI / 180.0);
	my_car_data.x = my_action_data.x.data;
	my_car_data.y =	my_action_data.y.data;
	my_car_data.yaw = my_action_data.yaw.data;
}


void commu_task_init()
{
	my_uart8_enable_inpterr();
	my_uart6_enable_inpterr();
  uart8_printf("ACT0");
	my_car_data.stuff_num = 100;
}
void commu_task(void const* argument){
	commu_task_init();
	uint8_t tx_msg[19];
	int initangle=120;
	int a=2;
	servo_angle_ctrl(&servo[0],ANGLE_CLAW_CLOSE);
	while(1){
		if(rising_falling_flag!=HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
			rising_falling_flag =HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
			if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)==1)
			{
        uart8_printf("ACT0");
				my_car_data.stuff_num=0;
//				switch(a){
//				case ARM_TO_CODE:
//				{
//					cmd_arm_to_code();
//					break;
//				}
//				case ARM_TO_STUFF:
//				{
//				  cmd_arm_to_stuff();
//					break;
//				}
//				case ARM_GRAB_MATERIAL:
//				{
//					cmd_arm_grab_material();
//					break;
//				}
//				case ARM_PLACE_GROUND:
//				{
//					cmd_arm_place_ground();
//					break;
//				}
//				case ARM_GRAB_GROUND:
//				{
//					cmd_arm_grab_ground();
//					break;
//				}
//				case ARM_PLACE_STUFF:
//				{
//					cmd_arm_place_stuff();
//					break;
//				}
//				case ARM_END:
//				{
//					cmd_arm_place_stuff();
//					break;
//				}
//				default :
//					break;
//			  }
//				a++;
			}
		}
		if(rising_falling_flag1!=HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){
			rising_falling_flag1 =HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)==1)
			{
				startM2006Monitor();
				servo_angle_ctrl(&servo[1],ANGLE_CAMERA_TO_CODE);
				//set_M2006_rotate_rounds(1,ROUNDS_TOP_TO_BOTTOM);
			}
		}
		if(action_count)
		{
			HAL_GPIO_WritePin(ACTION_LED_GPIO_Port, ACTION_LED_Pin,GPIO_PIN_SET);
			encode(tx_msg,0x01,14,my_car_data.x,my_car_data.y,my_car_data.yaw,my_car_data.stuff_num);
		  HAL_UART_Transmit(&huart6, tx_msg , 19 , 100);
		  usart6_tx_dma_enable(tx_msg,19);
			action_count=0;
		}
		
		action_to_car();

//		uart7_printf("\n");
//		uart7_printf("goal: %f, %f, %f\n",my_move.x_goal.data ,my_move.y_goal.data, my_move.w_goal.data);
//		uart7_printf("act:  %f, %f, %f\n",my_action_data.x.data, my_action_data.y.data, my_action_data.yaw.data);
//		uart7_printf("car:  %f, %f, %f\n",my_car_data.x, my_car_data.y, my_car_data.yaw);
//		uart7_printf("err:  %f, %f, %f\n",chassis_v_pid[0].error[0],chassis_v_pid[1].error[0], chassis_w_pid.error[0]);
//		uart7_printf("v:    %f, %f\n",point_decoded[0], point_decoded[1]);
		
		//uart7_printf("%d", );
		osDelay(20);
		HAL_GPIO_WritePin(ACTION_LED_GPIO_Port, ACTION_LED_Pin,GPIO_PIN_RESET);
	}
}
