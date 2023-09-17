#ifndef NUC_COMMU_H
#define NUC_COMMU_H

#include "struct_typedef.h"
#include "main.h"
#define SOF 0xA5
#define END 0x5A

#define CMD_CAR 0x01
#define CMD_MOVE 0x02
#define CMD_PINCH 0x03

//4+4+4+1+4+1=20
#define NUCINFO_RX_BUF_NUM  16u
#define NUCINFO_FRAME_LENGTH 16u

typedef union
{
	fp32 data;
	uint8_t bytes[4];
} RxFP32Data;


typedef struct
{
	RxFP32Data x;
	RxFP32Data y;
	RxFP32Data yaw;
	uint8_t cmd;
}toSTM32_t;

extern void nuc_control_init(void);
#endif
