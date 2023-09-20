#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "chassis_filter.h"
#include "CAN_receive.h"
#include "CAN_cmd_3508.h"
#include "pid.h"
#include "struct_typedef.h"
#include "commu_task.h"

#define MAX_OUT  8000.0
#define MAX_IOUT 1000.0
#define MOTOR_DISTANCE_TO_CENTER 50.0f //需要改
#define RIDIUS 30.0f									 //需要改
#define	PID_3508_P	1.0
#define PID_3508_I	1.0
#define PID_3508_D	0.0
#define FILTER_NUM 50
#define FILTER_FRAME_PERIOD 1

static fp32 wheel_exp_rpm[4];
static fp32 wheel_set_rpm[4];
extern move_cmd_t my_move;
const motor_3508_measure_t*	chassis_motor[4];
const fp32 pid_k[3]={PID_3508_P, PID_3508_I, PID_3508_D};
first_order_filter_type_t rpm_filter;
pid_t pid[4];


void chassis_v_to_mecanum_speed(fp32 vx_err, fp32 vy_err, fp32 vw_err)
{
	wheel_exp_rpm[0] = (-vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS);
	wheel_exp_rpm[1] = ( vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS);
	wheel_exp_rpm[2] = ( vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS);
	wheel_exp_rpm[3] = (-vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err) / ( 2 * PI * RIDIUS);
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
	
	for(int i=0; i<4; i++)
	{
		first_order_filter_cali(&rpm_filter, wheel_exp_rpm[i]);
		PID_calc(&pid[i], chassis_motor[i]->speed_rpm, rpm_filter.out);
		wheel_set_rpm[i] = pid[i].out;
	}
}

void chassis_ctrl(void)
{
		chassis_pid_calc(wheel_exp_rpm);
		CAN_cmd_chassis(wheel_set_rpm[0], wheel_set_rpm[1], wheel_set_rpm[2], wheel_set_rpm[3]);
}

void chassis_task(){
	while(1){
		
		chassis_ctrl();
		osDelay(1);
	}
}
