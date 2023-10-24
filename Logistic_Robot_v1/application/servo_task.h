#ifndef SERVO_TASK
#define SERVO_TASK

//向里头转为正转，向外头转为反转
//以最高点为0，向下为负数
//全程48-49圈之间，抓物块21圈下降
// 放盘子上，15圈
//放物块上 24圈
//放地上，42圈+1圈
//张开角度 90°闭合角度1 120°
//对stuff角度 120 对二维码角度 102.5° 
#define ANGLE_CAMERA_TO_CODE 85
#define ANGLE_CAMERA_TO_STUFF 115

#define ANGLE_CLAW_OPEN 90
#define ANGLE_CLAW_CLOSE 120

#define ROUNDS_PLACE_PLATE  -9
#define ROUNDS_TOP_TO_BOTTOM -43
#define ROUNDS_BOTTOM_TO_TOP 43
#define ROUNDS_PLACE_GROUND -43
#define ROUNDS_PLACE_STUFF -21
#define ROUNDS_GRAB_STUFF_MATERIAL -21
#define ROUNDS_GRAB_STUFF_GROUND  -45
#define ROUNDS_TURN_IN 19
#define ROUNDS_TURN_OUT -19

void servo_task(void const* argument);
void cmd_arm_rst();
void cmd_arm_to_code();
void cmd_arm_to_stuff();
void cmd_arm_grab_material();
void cmd_arm_place_ground();
void cmd_arm_grab_ground();
void cmd_arm_place_stuff();
#endif
