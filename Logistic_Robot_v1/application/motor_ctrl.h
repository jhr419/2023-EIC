#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H
#include "pid.h"
#include "struct_typedef.h"
#define MAX_MOTOR_ECD 8191
extern void angle_m6020_to_next(void);
extern void angle_m6020_init(void);
extern void give_pid_current_6020();
extern uint16_t ecd_format(uint16_t ecd);
extern fp32 PID_ECD_calc(pid_t *pid, fp32 ref, fp32 set);
#endif
