#include "main.h"
#include "motor_ctrl.h"
#include ""

const pid_2006_angle[3] = {0.0, 0.0, 0.0};
const pid_2006_speed[3] = {0.0, 0.0, 0.0};
const motor_2006_measure_t*	motor_2006[4];

void angle_m2006(int32_t)