#ifndef MS4015_H
#define MS4015_H

#include "struct_typedef.h"

extern void CAN_read_pid(void);
extern void CAN_set_pid_ROM(uint8_t* pid);
extern void CAN_read_ecdData(void);//0x90
extern void CAN_iqControl(int16_t iqControl);//0xA1
extern void CAN_speedControl(int32_t speedControl);//0xA2
extern void CAN_angle_angleControl(int16_t angleControl);//0xA6
extern void CAN_delta_angleControl(int32_t angleControl);//0xA7

#endif
