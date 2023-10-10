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
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = 
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period)	* first_order_filter_type->out + 
		first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period) * first_order_filter_type->input;
}
