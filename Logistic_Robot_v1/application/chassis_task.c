#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassis_filter.h"
#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "commu_task.h"
#include "servo.h"
#include "gear_motor_ctrl.h"
#include "pid.h"
#include "remote_control.h"

#define CHASSIS_RC_DEADLINE 50
#define CHASSIS_VX_RC_SEN 0.006f
#define CHASSIS_VY_RC_SEN 0.005f
#define CHASSIS_WZ_RC_SEN 0.01f
#define MAX_OUT  12000.0
#define MAX_IOUT 1000.0
#define MOTOR_DISTANCE_TO_CENTER 50.0f //需要改
#define RAD_TO_DEGREE 2*PI/360
#define RIDIUS 30.0f									 //需要改
#define	PID_3508_P	5.9
#define PID_3508_I	0.6
#define PID_3508_D	4.0
#define FILTER_NUM 50
#define FILTER_FRAME_PERIOD 1

extern servo_t servo[8];
static fp32 wheel_exp_rpm[4];
static fp32 wheel_set_rpm[4];
extern move_cmd_t my_move;
const motor_3508_measure_t*	chassis_motor[4];
const fp32 pid_k[3]={PID_3508_P, PID_3508_I, PID_3508_D};
pid_t pid[4];
const RC_ctrl_t * rc_p;
int robot_mode;
enum robot_mode
{
	ROBOT_powerless = 1,
	ROBOT_auto = 3,
	ROBOT_manual = 2
};

void chassis_v_to_mecanum_speed(fp32 vx_err, fp32 vy_err, fp32 vw_err)
{
	//vw_err = 500;
	//vx_err = 500;
	//vy_err = 500;
	vx_err*=10;
	vy_err*=10;
	vw_err*=10;
	wheel_exp_rpm[0] = (int)((vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[1] = (int)(( vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[2] = (int)(( -vx_err + vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
	wheel_exp_rpm[3] = (int)((-vx_err - vy_err - MOTOR_DISTANCE_TO_CENTER * vw_err * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
}

void chassis_init(void)
{
	for(int i=0;i<4;i++)
	{
		PID_init(&pid[i], PID_POSITION, pid_k, MAX_OUT, MAX_IOUT);
		wheel_set_rpm[i] = 0.0;
		chassis_motor[i] = get_motor_3508_measure_point(i);
	}
	rc_p=get_remote_control_point();
}

void chassis_pid_calc(fp32* wheel_exp_rpm)
{
	if(robot_mode == ROBOT_auto)
	 chassis_v_to_mecanum_speed(my_move.vx_err.data, my_move.vy_err.data, my_move.vw_err.data);
	else if(robot_mode == ROBOT_manual)
	{
		int vx_channel,vy_channel,w_channel;
		rc_deadband_limit(rc_p->rc.ch[3],vx_channel,CHASSIS_RC_DEADLINE);
		rc_deadband_limit(rc_p->rc.ch[2],vy_channel,CHASSIS_RC_DEADLINE);
		rc_deadband_limit(rc_p->rc.ch[0],w_channel,CHASSIS_RC_DEADLINE);
		float vx  = vx_channel*CHASSIS_VX_RC_SEN;
		float vy  = vy_channel*CHASSIS_VY_RC_SEN;
		float w   = w_channel*CHASSIS_WZ_RC_SEN;
		if(rc_p->rc.ch[1]>500)
			set_M2006_rotate_rounds(1,1);
		else if(rc_p->rc.ch[1]<-500)
			set_M2006_rotate_rounds(1,-1);
		if(rc_p->rc.ch[4]>500)
			set_M2006_rotate_rounds(0,6);
		else if(rc_p->rc.ch[4]<-500)
			set_M2006_rotate_rounds(0,-6);
		chassis_v_to_mecanum_speed(vx/10, vy/10, w/10);
	}
	//usart_printf("%f\r\n",wheel_exp_rpm[0]);
	for(int i=0; i<4; i++)
	{
		PID_calc(&pid[i], chassis_motor[i]->speed_rpm, wheel_exp_rpm[i]);
		wheel_set_rpm[i] = pid[i].out;
	}
	
}

void chassis_ctrl(void)
{
	  
		chassis_pid_calc(wheel_exp_rpm);
	if(robot_mode != ROBOT_powerless)
		CAN_cmd_chassis(wheel_set_rpm[0], wheel_set_rpm[1], wheel_set_rpm[2], wheel_set_rpm[3]);
	else
		CAN_cmd_chassis(0,0,0,0);
}
void get_robot_mode()
{
	 robot_mode=rc_p->rc.s[1];
}
void chassis_task(void const* argument){
	chassis_init();
	while(1){
		get_robot_mode();
		chassis_ctrl();
		osDelay(5);
		
	}
}
