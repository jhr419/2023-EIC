#ifndef __FILTER_H
#define __FILTER_H


#include "struct_typedef.h"
typedef struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num;       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num);
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
#endif
