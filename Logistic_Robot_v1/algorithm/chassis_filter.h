#ifndef __FILTER_H
#define __FILTER_H


#include "struct_typedef.h"
typedef struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num;       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num);
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
#endif
