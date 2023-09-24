#include "chassis_task.h"

#define PI 3.1415926
#define MAX_OUT  12000.0
#define MAX_IOUT 1000.0
#define MOTOR_DISTANCE_TO_CENTER 50.0f //需要改
#define RIDIUS 30.0f									 //需要改
#define	PID_3508_P	5.9
#define PID_3508_I	0.5
#define PID_3508_D	4.0
#define FILTER_NUM 50
#define FILTER_FRAME_PERIOD 1

static fp32 wheel_exp_rpm[4];
static fp32 wheel_set_rpm[4];
extern move_cmd_t my_move;
const motor_3508_measure_t*	chassis_motor[4];
const fp32 pid_k[3]={PID_3508_P, PID_3508_I, PID_3508_D};
first_order_filter_type_t rpm_filter;
pid_t pid[4];

/*
	 	  ^
	 	  |
	 1##x##4
	 #		 #
<--y		 #
	 #		 #
	 2#####3
*/


void chassis_v_to_mecanum_speed(fp32 vx_err, fp32 vy_err, fp32 vw_err)
{
	wheel_exp_rpm[0] = (int)(( vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[1] = (int)(( vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[2] = (int)((-vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[3] = (int)((-vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS)*60);
}

void chassis_init(void)
{
	for(int i=0;i<4;i++)
	{
		PID_init(&pid[i], PID_POSITION, pid_k, MAX_OUT, MAX_IOUT);
		wheel_set_rpm[i] = 0.0;
		chassis_motor[i] = get_motor_3508_measure_point(i);
	}
	first_order_filter_init(&rpm_filter, FILTER_FRAME_PERIOD, FILTER_NUM);
}

void chassis_pid_calc(fp32* wheel_exp_rpm)
{
	chassis_v_to_mecanum_speed(my_move.vx_err.data, my_move.vy_err.data, my_move.vw_err.data);
	//usart_printf("%f\r\n",wheel_exp_rpm[0]);
	for(int i=0; i<4; i++)
	{
		//first_order_filter_cali(&rpm_filter, wheel_exp_rpm[i]);
		
		PID_calc(&pid[i], chassis_motor[i]->speed_rpm, wheel_exp_rpm[i]);
		wheel_set_rpm[i] = pid[i].out;
	}
	
}

void chassis_ctrl(void)
{
		chassis_pid_calc(wheel_exp_rpm);
		CAN_cmd_chassis(wheel_set_rpm[0], wheel_set_rpm[1], wheel_set_rpm[2], wheel_set_rpm[3]);
}

void chassis_task(void const* argument){
	chassis_init();
	while(1){
		chassis_ctrl();
		//uart8_printf("%d,%f\r\n",chassis_motor[0]->speed_rpm,wheel_exp_rpm[0]);
		//usart_printf("out %d\r\n",pid[0].out);
		osDelay(5);
		
	}
}
