/**
 * encoding:GB2312
 * @file M2006_task.c
 * @author Brandon
 * @brief  ��ȡȫ����λ���ݲ�ת���o��λ��
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     10.15           Brandon         done
 */
#include "CAN_cmd_all.h"
#include "main.h"
#include "cmsis_os.h"
#include "commu_task.h"
#include "pid.h"
#include "CAN_receive.h"
//��Ҫ�жϴ���������ʱ�������յ���е�۶�ץȡ�ͷ��õ�ͬʱ���в����������������нo������
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;


extern CAN_TxHeaderTypeDef  motor_2006_tx_message;
extern uint8_t              motor_2006_can_send_data[8];

//���Ʊ������д��޸�
#define Rotate_To_Go 12
#define SPEED_M2006 1000
#define MAX_OUT		12000.0
#define MAX_IOUT  1200.0
float pid_m2006[3]={5,0,0};
#define MILESTONE_NUMBER 3
#define ECD_FULL_ROUND 8192

#define MILESTONE_NEAR_THRESHHOLD 2000


struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};
enum M2006Mode{
	M2006_POWERLESS = 0		,
	M2006_ROTATE_FORWARD 	,
	M2006_ROTATE_BACKWARD ,
	M2006_STOP
};
struct M2006Control_s{
    struct milestoneStack_s mstack; //��̱�ջ
    const uint16_t * ECDPoint;           //���ECD����λ��
    uint16_t    initECD;                //��ʼECD
    uint16_t    nowECD;                 //���ڵ�ECD
    int16_t     nowRounds;              //����ת����Ȧ��
		int16_t     targetRounds;           //Ŀ��ת����Ȧ��
		int16_t	    set_speed;								//Ŀ��ת��
		int16_t*    now_speed;								//��ǰת��
		enum M2006Mode mode;								//��ǰ2006��ģʽ
		pid_t       pid;
};

static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}
extern arm_cmd_t 	my_arm;
//��������޸ģ�����������
struct M2006Control_s M2006Ctrl;
static void initM2006ECDRoundsMonitor()  //��ʼ��������Ȧ�����
{
	  PID_init(&M2006Ctrl.pid, PID_POSITION, pid_m2006, MAX_OUT , MAX_IOUT);
	//��ȡ�Ǹ��������ֵָ��
		M2006Ctrl.now_speed=(int16_t *)&(get_motor_2006_measure_point(0)->speed_rpm);
    M2006Ctrl.ECDPoint=&(get_motor_2006_measure_point(0)->ecd);
    M2006Ctrl.initECD=*(M2006Ctrl.ECDPoint);
		M2006Ctrl.mode=M2006_STOP;
    
}   
static void monitorM2006ECDRound(void)
{
    uint8_t j;
    // ����ECD
    M2006Ctrl.nowECD=*(M2006Ctrl.ECDPoint);
    for(j=0;j<MILESTONE_NUMBER;j++)    //ö��ÿһ����̱�����λ��
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)M2006Ctrl.nowECD-(int16_t)M2006Ctrl.initECD);
        //ʧ��ԭ����0�ıȽϳ���������
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //��ǰλ��������Ӧ��̱�������������
        {
            if(j!=(M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head]))
               //����˵��������һ����λ�ã�������λ�ü���ջ��
            {
                M2006Ctrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  ��usb task���������Խ����Ϣ
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head]=j;
            }
        }
    }
    if(((M2006Ctrl.mstack.head)-2)>=0)
    {
        if(M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head]==M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head-2])
            (M2006Ctrl.mstack.head)-=2;
    }
    if(((M2006Ctrl.mstack.head)-3)>=0)
    {
        if(M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head]==M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head-3])
        {//������һȦ
            if(M2006Ctrl.mstack.stack[1]==1)//������ת����ʱ�룩
                M2006Ctrl.nowRounds +=1;
            else
                M2006Ctrl.nowRounds -=1;
            M2006Ctrl.mstack.head=0;     // ���ջ���ص���ʼΪ0��ʱ��
        }
    }
}
void refresh_M2006_ctrl(){
	if(M2006Ctrl.nowRounds>M2006Ctrl.targetRounds)
		M2006Ctrl.mode=M2006_ROTATE_BACKWARD;
	if(M2006Ctrl.nowRounds<M2006Ctrl.targetRounds)
		M2006Ctrl.mode=M2006_ROTATE_FORWARD;
	if(M2006Ctrl.nowRounds==M2006Ctrl.targetRounds)
	{
		M2006Ctrl.mode=M2006_STOP;
		M2006Ctrl.targetRounds = M2006Ctrl.nowRounds;
	}
}
//�������k Ϊ��ǰ״̬����ת��Ŀ�꣬����һ���ⲿ����������servo_task���ڽ���ǰת��
void set_M2006_rotate_rounds(int k)
{
	if(M2006Ctrl.mode==M2006_STOP)
	{
		M2006Ctrl.targetRounds=M2006Ctrl.nowRounds+k;
		if(k>0)
			M2006Ctrl.mode=M2006_ROTATE_FORWARD;
		else if(k<0)
			M2006Ctrl.mode=M2006_ROTATE_BACKWARD;
		else
			M2006Ctrl.mode=M2006_STOP;
	}
}
//һ��Ҫ��pid��Ϊ�и��ر���
void set_M2006_speed()
{
	if(M2006Ctrl.mode==M2006_ROTATE_FORWARD)
	{
		M2006Ctrl.set_speed=SPEED_M2006;
	}
	else if (M2006Ctrl.mode==M2006_ROTATE_BACKWARD)
	{
		M2006Ctrl.set_speed=-SPEED_M2006;
	}
	else if (M2006Ctrl.mode==M2006_STOP)
	{
		M2006Ctrl.set_speed=0;
	}
}
void set_M2006_current(){
	int current=PID_calc(&M2006Ctrl.pid,*M2006Ctrl.now_speed,M2006Ctrl.set_speed);
	CAN_cmd_2006(current,0,0,0);
}
void m2006_task(void const* argument)
{
	osDelay(300);
	initM2006ECDRoundsMonitor();
	set_M2006_rotate_rounds(2);
	while(1)
	{
		uart8_printf("%d\r\n",M2006Ctrl.nowRounds);
		monitorM2006ECDRound();
		refresh_M2006_ctrl();
		set_M2006_speed();
		set_M2006_current();
		osDelay(1);
	}
}

