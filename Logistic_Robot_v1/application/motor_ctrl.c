#include "main.h"
#include "motor_ctrl.h"
#include "pid.h"
#include "CAN_receive.h"

pid_t pid_angle_2006[4];
pid_t pid_speed_2006[4];
const fp32 pid_2006_angle[3] = {0.0, 0.0, 0.0};
const fp32 pid_2006_speed[3] = {0.0, 0.0, 0.0};
const motor_2006_measure_t*	motor_2006[4];

//void angle_m2006_init(void)
//{
//	for(int i=0;i<4;i++)
//	{
//		motor_2006[i]	= get_motor_2006_measure_point(i+4);
//		PID_init(&pid_angle_2006[i], PID_POSITION, pid_2006_angle, 
//	}
//	
//	
//}