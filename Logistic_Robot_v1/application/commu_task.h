#ifndef COMMU_TASK
#define COMMU_TASK
#define __PACKED __attribute__((packed))
#include "struct_typedef.h"
typedef union __PACKED
{
	fp32 data;
	uint8_t bytes[4];
} RxFP32Data;
typedef enum
{
	ARM_GRAB_GROUND1 = 0x09   ,
	ARM_GRAB_GROUND2 = 0x0A   ,
	ARM_GRAB_GROUND3 = 0x0B   ,
	ARM_PLACE_GROUND1 = 0x0C  ,
	ARM_PLACE_GROUND2 = 0x0D  ,
	ARM_PLACE_GROUND3 = 0x0E  ,
	ARM_GRAB_GROUND4 = 0x0F   ,
	ARM_GRAB_GROUND5 = 0x10   ,
	ARM_GRAB_GROUND6 = 0x11   ,
	ARM_PLACE_MATERIAL1 = 0x12,
	ARM_PLACE_MATERIAL2 = 0x13,
	ARM_PLACE_MATERIAL3 = 0x14,
	ARM_TO_TARGET = 0x15,
}act_id_e;
typedef struct __PACKED
{
	RxFP32Data yaw;
  RxFP32Data pitch;
	RxFP32Data roll;
	RxFP32Data x;
	RxFP32Data y;
	RxFP32Data w;
}action_data;
typedef struct _PACKED
{
	fp32 x;
	fp32 y;
	fp32 yaw;
	int16_t stuff_num;
}car_data_s;
typedef struct __PACKED
{
 uint8_t sof;
 uint8_t msg_id;
 uint16_t length;
} frame_header_t;

typedef struct __PACKED
{
 RxFP32Data x_abs;
 RxFP32Data y_abs;
 RxFP32Data angle_abs;
 uint16_t stuff_num;
} position_info_t;

typedef struct __PACKED
{
 RxFP32Data x_goal;
 RxFP32Data y_goal;
 RxFP32Data w_goal;
} move_cmd_t;

typedef struct __PACKED
{
 uint8_t act_id;
} arm_cmd_t;
extern void commu_task(void const* argument);
extern void uart8_printf(const char *fmt,...);
extern void uart7_printf(const char *fmt,...);
extern void my_uart8_enable_inpterr(void);
extern void my_uart6_send_data(uint8_t *tdata,uint16_t tnum);

extern void  encode(uint8_t* a,uint8_t cmd,uint16_t length,float x, float y, float angle, int stuffnum);
extern void decode02(uint8_t* data);
extern void decode03(uint8_t* data);
extern void decode_action(uint8_t * data);
#endif
