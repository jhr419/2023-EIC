#include "main.h"
#include "motor_ctrl.h"
#include "pid.h"
#include "CAN_receive.h"
#include "CAN_cmd_all.h"
#include "commu_task.h"

#define INIT_ECD 6000

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//typedef struct 
//{
//	uint16_t ecd_init;
//	const motor_6020_measure_t*	motor_6020;
//	uint16_t* ecd_p;
//  int16_t* speed_rpm_p;
//}m6020_t;

//m6020_t* motor_6020;
		
const motor_6020_measure_t*	motor_6020;
pid_t pid_angle_6020;
pid_t pid_speed_6020;
const fp32 pid_6020_angle[3] = {3.0, 0.0, 0.0};
const fp32 pid_6020_speed[3] = {3.0, 0.0, 0.0};
static uint16_t ecd[3];
uint8_t cnt = 0;

uint16_t ecd_format(uint16_t ecd)
{
	if(ecd >= 8192)
	{
		return ecd - 8192;
	}
	else if(ecd<0)
	{
		return ecd + 8192;
	}
	return ecd;
}

uint16_t ecd_limit(uint16_t ref, uint16_t set)
{
	if(set > ref)
	{
		if(set - ref > ref - set + MAX_MOTOR_ECD)
		{
			return set - MAX_MOTOR_ECD;
		}
		else if(set - ref < ref - set + MAX_MOTOR_ECD)
		{
			return set;
		}
	}
	else if(set < ref)
	{
		if(ref - set > set - ref + MAX_MOTOR_ECD)
		{
			return set + MAX_MOTOR_ECD;
		}
	}
}

void angle_m6020_init(void)
{
	PID_init(&pid_angle_6020, PID_POSITION, pid_6020_angle, 30000,1000);
	PID_init(&pid_speed_6020, PID_POSITION, pid_6020_speed, 5000, 100);

	motor_6020 = get_motor_6020_measure_point(0);	
	//motor_6020->ecd_init=INIT_ECD;
	//motor_6020->ecd_p = &(motor_6020->motor_6020->ecd);
	//motor_6020->speed_rpm = motor_6020->motor_6020->speed_rpm;
	
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
void give_pid_current_6020()
{
	PID_calc(&pid_angle_6020, motor_6020->ecd, ecd_limit(motor_6020->ecd, ecd[cnt]));
	PID_calc(&pid_speed_6020, motor_6020->speed_rpm, pid_angle_6020.out);
	CAN_cmd_6020(pid_speed_6020.out,pid_speed_6020.out,pid_speed_6020.out,pid_speed_6020.out);
	uart8_printf("%d,%d\r\n", ecd[cnt], motor_6020->ecd);
}
