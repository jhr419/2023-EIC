#ifndef MS4015_H
#define MS4015_H

#include "struct_typedef.h"

extern void CAN_read_pid(uint32_t id);
extern void CAN_set_pid_ROM(uint32_t id, uint8_t* pid);
extern void CAN_read_ecdData(uint32_t id);//0x90
extern void CAN_angleControl(uint32_t id, int16_t angleControl);//0xA6
extern void CAN_delta_angleControl(uint32_t id, int32_t angleControl);//0xA7

#endif
