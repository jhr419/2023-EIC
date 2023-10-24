#include "main.h"
#include "motor_ctrl.h"
#include "pid.h"
#include "CAN_receive.h"
#include "CAN_cmd_all.h"
#include "commu_task.h"

#define INIT_ECD 0
		
const motor_6020_measure_t*	motor_6020;
pid_t pid_angle_6020;
pid_t pid_speed_6020;
const fp32 pid_6020_angle[3] = {3.0, 0.0, 0.0};
const fp32 pid_6020_speed[3] = {3.0, 0.0, 0.0};
static uint16_t ecd[3];
uint8_t cnt = 0;

uint16_t ecd_format(uint16_t ecd)
{
	while (ecd >= 8192)
	{
		ecd -= 8192;
	}
	return ecd;
}

uint16_t ecd_limit(uint16_t ref, uint16_t set)
{
	if(set > ref)
	{
		if(set - ref > ref - set + MAX_MOTOR_ECD)
		{
			return set - MAX_MOTOR_ECD;//error = set - MAX_MOTOR_ECD - ref
		}
		else if(set - ref < ref - set + MAX_MOTOR_ECD)
		{
			return set;//error = set -ref
		}
	}
	else if(set < ref)
	{
		if(ref - set > set - ref + MAX_MOTOR_ECD)
		{
			return set + MAX_MOTOR_ECD;//error = 
		}
		else if(ref - set < set - ref + MAX_MOTOR_ECD)
		{
			return set;//error
		}
	}
}

void angle_m6020_init(void)
{
	PID_init(&pid_angle_6020, PID_POSITION, pid_6020_angle, 30000,1000);
	PID_init(&pid_speed_6020, PID_POSITION, pid_6020_speed, 5000, 100);

	motor_6020 = get_motor_6020_measure_point(0);	
	ecd[0] = INIT_ECD;
	ecd[1] = ecd_format(ecd[0] + 2730);
	ecd[2] = ecd_format(ecd[1] + 2731);
}

void angle_m6020_to_next(void)
{
	cnt++;
	if(cnt>=3)
		cnt = 0;

}


fp32 PID_ECD_calc(pid_t *pid, fp32 ref, fp32 set)
{
	PID_calc(pid, ref, ecd_limit(ref, set));
}

void give_pid_current_6020()
{
	PID_ECD_calc(&pid_angle_6020, motor_6020->ecd, ecd[cnt]);
	PID_calc(&pid_speed_6020, motor_6020->speed_rpm, pid_angle_6020.out);
	CAN_cmd_6020(pid_speed_6020.out,pid_speed_6020.out,pid_speed_6020.out,pid_speed_6020.out);
	//uart8_printf("%d,%d\r\n", ecd[cnt], motor_6020->ecd);
}
