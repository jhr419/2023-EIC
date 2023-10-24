#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "struct_typedef.h"
#define MAX_MOTOR_ECD 8191
extern void angle_m6020_to_next(void);
extern void angle_m6020_init(void);
extern void give_pid_current_6020();
#endif
