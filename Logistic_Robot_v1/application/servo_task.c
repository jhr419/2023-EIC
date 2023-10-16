#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "commu_task.h"
#include "CAN_cmd_all.h"
#include "struct_typedef.h"
extern car_data_s  my_car_data;
extern arm_cmd_t 	my_arm;
//static uint8_t last_arm_cmd;
void cmd_arm1_grap()
{
	//先糊弄一下，之后改
	osDelay(3000);
	my_car_data.stuff_num++;
	uart7_printf("arm1 already grap，now stuff_num:%d\r",my_car_data.stuff_num);
}
void cmd_arm2_grap()
{
	//先糊弄一下，之后改
	osDelay(3000);
	my_car_data.stuff_num++;
  uart7_printf("arm2 already grap，now stuff_num:%d\r",my_car_data.stuff_num);
}
void cmd_arm2_place1()
{
	//先糊弄一下，之后改
	osDelay(3000);
	my_car_data.stuff_num--;
  uart7_printf("arm2 already place1，now stuff_num:%d\r",my_car_data.stuff_num);
}
void cmd_arm2_place2()
{
	//先糊弄一下，之后改
	osDelay(3000);
	my_car_data.stuff_num--;
  uart7_printf("arm2 already place2，now stuff_num:%d\r",my_car_data.stuff_num);
}
void servo_task(void const* argument){
	
	while(1){
		
		if(my_arm.act_id!=0)
		{
			switch(my_arm.act_id){
				case ARM1_GRAP:
				{
					cmd_arm1_grap();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM2_GRAP:
				{
					cmd_arm2_grap();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM2_PLACE1:
				{
				  cmd_arm2_place1();
					//CAN_delta_angleControl(12000);
					break;
				}
				case ARM2_PLACE2:
				{
					cmd_arm2_place2();
					//CAN_delta_angleControl(12000);
					break;
				}
				
				default :
					break;
			}
			my_arm.act_id = 0;
  		//last_arm_cmd = my_arm.act_id;
	  }
		osDelay(20);
	}
}
