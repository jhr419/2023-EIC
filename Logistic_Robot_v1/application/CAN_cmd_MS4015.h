#ifndef CAN_CMD_MS4015_H
#define CAN_CMD_MS4015_H

#include "struct_typedef.h"

extern void CAN_cmd_read_pid(void);
extern void CAN_cmd_set_pid_ROM(uint8_t* pid);
extern void CAN_cmd_read_ecdData(void);//0x90
extern void CAN_cmd_iqControl(int16_t iqControl);//0xA1
extern void CAN_cmd_speedControl(int32_t speedControl);//0xA2

#endif
