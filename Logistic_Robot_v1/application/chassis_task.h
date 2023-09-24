#ifndef CHASSIS_TASK
#define  CHASSIS_TASK

#include "main.h"
#include "chassis_filter.h"
#include "CAN_receive.h"
#include "CAN_cmd_3508.h"
#include "pid.h"
#include "struct_typedef.h"
#include "commu_task.h"
#include "cmsis_os.h"

extern void chassis_task(void const* argument);
#endif
