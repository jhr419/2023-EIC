#ifndef COMMU_TASK
#define COMMU_TASK
#define __PACKED __attribute__((packed))
#include "struct_typedef.h"
typedef union __PACKED
{
	fp32 data;
	uint8_t bytes[4];
} RxFP32Data;
typedef struct __PACKED
{
	RxFP32Data yaw;
  RxFP32Data pitch;
	RxFP32Data roll;
	RxFP32Data x;
	RxFP32Data y;
	RxFP32Data w;
}car_data;

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
 RxFP32Data vx_err;
 RxFP32Data vy_err;
 RxFP32Data vw_err;
} move_cmd_t;

typedef struct __PACKED
{
 uint8_t act_id;
} arm_cmd_t;
extern void commu_task(void const* argument);
extern void uart8_printf(const char *fmt,...);
extern void my_uart8_enable_inpterr(void);
extern void my_uart6_send_data(uint8_t *tdata,uint16_t tnum);

extern void my_uart6_enable_inpterr();
extern void  encode(uint8_t* a,uint8_t cmd,uint16_t length,float x, float y, float angle, int stuffnum);
extern void decode02(uint8_t* data);
extern void decode03(uint8_t* data);
extern void decode_action(uint8_t * data);
#endif
