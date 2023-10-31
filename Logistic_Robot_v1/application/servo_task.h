#ifndef SERVO_TASK
#define SERVO_TASK

//����ͷתΪ��ת������ͷתΪ��ת
//����ߵ�Ϊ0������Ϊ����
//ȫ��48-49Ȧ֮�䣬ץ���21Ȧ�½�
// �������ϣ�15Ȧ
//������� 24Ȧ
//�ŵ��ϣ�42Ȧ+1Ȧ
//�ſ��Ƕ� 90��պϽǶ�1 120��
//��stuff�Ƕ� 120 �Զ�ά��Ƕ� 102.5�� 
#define ANGLE_CAMERA_TO_CODE 90
#define ANGLE_CAMERA_TO_STUFF 115

#define ANGLE_CLAW_OPEN  10
#define ANGLE_CLAW_CLOSE 47

#define ROUNDS_PLACE_PLATE  -7
#define ROUNDS_TOP_TO_BOTTOM -45
#define ROUNDS_BOTTOM_TO_TOP 45
#define ROUNDS_PLACE_GROUND -45
#define ROUNDS_PLACE_STUFF -24
#define ROUNDS_GRAB_STUFF_MATERIAL -22
#define ROUNDS_GRAB_STUFF_GROUND  -45
#define ROUNDS_TURN_IN -18
#define ROUNDS_TURN_OUT 18

void servo_task(void const* argument);
void cmd_arm_rst();
void cmd_arm_to_code();
void cmd_arm_to_stuff();
void cmd_arm_grab_material();
void cmd_arm_place_ground();
void cmd_arm_grab_ground();
void cmd_arm_place_stuff();
void cmd_arm_end();
void cmd_arm_grab_p2();
void cmd_arm_place_p2();
void cmd_arm_grab_p1();
void cmd_arm_place_p3();
#endif
