#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define MOTOR_CAN hcan1


#define get_motor_measure(ptr, data)                                \
    {                                                               \
        (ptr)->temperate = (int8_t)   (data)[1];                    \
        (ptr)->iq        = (int16_t)  ( (data)[3]<<8 | (data)[2] ); \
        (ptr)->speed     = (int16_t)  ( (data)[5]<<8 | (data)[4] ); \
        (ptr)->ecd       = (uint16_t) ( (data)[7]<<8 | (data)[6] ); \
    }

#define get_motor_pid(ptr, data)                \
    {                                           \
        (ptr)->anglePidKp = (uint8_t)(data)[2];  \
        (ptr)->anglePidKi = (uint8_t)(data)[3];  \
        (ptr)->speedPidKp = (uint8_t)(data)[4];  \
        (ptr)->speedPidKi = (uint8_t)(data)[5];  \
        (ptr)->iqPidKp    = (uint8_t)(data)[6];  \
        (ptr)->iqPidKi    = (uint8_t)(data)[7];  \
    }

#define get_motor_ecd_data(ptr, data)\
    {   \
        (ptr)->encoder       = (uint16_t) ( (data)[3]<<8 | (data)[2] );   \
        (ptr)->encoderRaw    = (uint16_t) ( (data)[5]<<8 | (data)[4] );   \
        (ptr)->encoderOffset = (uint16_t) ( (data)[7]<<8 | (data)[6] );   \
    }


typedef enum
{
    CAN_ALL_ID = 0x140,
    CAN_M1_ID  = 0x141,
}can_msg_id_e;

typedef struct 
{
    int8_t temperate;
    int16_t iq;
    int16_t speed;
    uint16_t ecd;
}motor_measure_t;

typedef struct 
{
    uint8_t anglePidKp;
    uint8_t anglePidKi;
    uint8_t speedPidKp;
    uint8_t speedPidKi;
    uint8_t iqPidKp;
    uint8_t iqPidKi;
}motor_pid_t;

typedef struct
{
    uint16_t encoder;
    uint16_t encoderRaw;
    uint16_t encoderOffset;
}motor_ecd_data_t;


extern void CAN_cmd_read_pid(void);
extern void CAN_cmd_set_pid_ROM(uint8_t* pid);
extern void CAN_cmd_read_ecdData(void);//0x90
extern void CAN_cmd_iqControl(int16_t iqControl);//0xA1
extern void CAN_cmd_speedControl(int32_t speedControl);//0xA2
extern const motor_measure_t *get_motor_measure_point(void);
extern const motor_pid_t *get_motor_pid_point(void);
extern const motor_ecd_data_t* get_motor_ecd_data_point(void);
#endif
