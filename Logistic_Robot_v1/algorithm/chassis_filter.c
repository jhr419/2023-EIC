#include "main.h"
#include "chassis_filter.h"

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num)
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num = num;
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = 
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period)	* first_order_filter_type->out + 
		first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}
