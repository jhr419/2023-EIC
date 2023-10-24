#ifndef __CAN_CMD_ALL_H
#define __CAN_CMD_ALL_H
#include "struct_typedef.h"

/*3508*/
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/*2006*/
extern void CAN_cmd_2006(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/*4015*/
extern void CAN_read_pid(uint32_t id);
extern void CAN_set_pid_ROM(uint32_t id, uint8_t* pid);
extern void CAN_read_ecdData(uint32_t id);//0x90
extern void CAN_angleControl(uint32_t id, int16_t angleControl);//0xA6
extern void CAN_delta_angleControl(uint32_t id, int32_t angleControl);//0xA7
extern void CAN_cmd_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
