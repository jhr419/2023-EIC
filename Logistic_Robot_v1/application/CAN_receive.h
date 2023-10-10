#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define get_motor_3508_measure(ptr, data)   														\
    {                                       														\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define get_motor_2006_measure(ptr, data)   														\
    {                                       														\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
#define get_motor_4015_measure(ptr, data)   														\
    {                                       														\
        (ptr)->temperate = (int8_t)   (data)[1];                    		\
        (ptr)->iq        = (int16_t)  ( (data)[3]<<8 | (data)[2] ); 		\
        (ptr)->speed     = (int16_t)  ( (data)[5]<<8 | (data)[4] ); 		\
        (ptr)->ecd       = (uint16_t) ( (data)[7]<<8 | (data)[6] ); 		\
    }

#define get_motor_4015_pid(ptr, data)       														\
    {                                       														\
        (ptr)->anglePidKp = (uint8_t)(data)[2]; 												\
        (ptr)->anglePidKi = (uint8_t)(data)[3]; 												\
        (ptr)->speedPidKp = (uint8_t)(data)[4]; 												\
        (ptr)->speedPidKi = (uint8_t)(data)[5]; 												\
        (ptr)->iqPidKp    = (uint8_t)(data)[6]; 												\
        (ptr)->iqPidKi    = (uint8_t)(data)[7]; 												\
    }

#define get_motor_4015_ecd_data(ptr, data)															\
    {   																																\
        (ptr)->encoder       = (uint16_t) ( (data)[3]<<8 | (data)[2] ); \
        (ptr)->encoderRaw    = (uint16_t) ( (data)[5]<<8 | (data)[4] ); \
        (ptr)->encoderOffset = (uint16_t) ( (data)[7]<<8 | (data)[6] ); \
    }


typedef enum
{
    CAN_M1_ID  = 0x141,
	
		CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
		
		CAN_2006_M1_ID = 0x205,
		CAN_2006_M2_ID = 0x206,
	  CAN_2006_M3_ID = 0x207,
	  CAN_2006_M4_ID = 0x208,
}can_msg_id_e;

typedef struct 
{
    int8_t temperate;
    int16_t iq;
    int16_t speed;
    uint16_t ecd;
}motor_4015_measure_t;

typedef struct 
{
    uint8_t anglePidKp;
    uint8_t anglePidKi;
    uint8_t speedPidKp;
    uint8_t speedPidKi;
    uint8_t iqPidKp;
    uint8_t iqPidKi;
}motor_4015_pid_t;

typedef struct
{
    uint16_t encoder;
    uint16_t encoderRaw;
    uint16_t encoderOffset;
}motor_4015_ecd_data_t;


typedef struct
{
		uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
}motor_3508_measure_t;

typedef struct
{
		uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
}motor_2006_measure_t;


extern const motor_3508_measure_t *get_motor_3508_measure_point(uint8_t i);
extern const motor_2006_measure_t *get_motor_2006_measure_point(uint8_t i);

extern const motor_4015_measure_t *get_motor_4015_measure_point(void);
extern const motor_4015_pid_t *get_motor_4015_pid_point(void);
extern const motor_4015_ecd_data_t* get_motor_4015_ecd_data_point(void);
#endif


