#ifndef __CAN_CMD_2006_H
#define __CAN_CMD_2006_H

#include "struct_typedef.h"

void CAN_cmd_2006(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void m2006_task(void const* argument);
extern void set_M2006_rotate_rounds(int k);
#endif
