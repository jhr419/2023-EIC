/**
 * encoding:GB2312
 * @file M2006_task.c
 * @author Brandon
 * @brief  收取全场定位数据并转发給上位机
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
//需要中断触发，触发时机：在收到机械臂二抓取和放置的同时进行操作，在其他任务中給工作量
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;


extern CAN_TxHeaderTypeDef  motor_2006_tx_message;
extern uint8_t              motor_2006_can_send_data[8];

//控制变量，有待修改
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
    struct milestoneStack_s mstack; //里程碑栈
    const uint16_t * ECDPoint;           //电机ECD所在位置
    uint16_t    initECD;                //初始ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
		int16_t     targetRounds;           //目标转到的圈数
		int16_t	    set_speed;								//目标转速
		int16_t*    now_speed;								//当前转速
		enum M2006Mode mode;								//当前2006的模式
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
//这个酌情修改，可能是数组
struct M2006Control_s M2006Ctrl;
static void initM2006ECDRoundsMonitor()  //初始化拨弹轮圈数监控
{
	  PID_init(&M2006Ctrl.pid, PID_POSITION, pid_m2006, MAX_OUT , MAX_IOUT);
	//获取那个电机的数值指针
		M2006Ctrl.now_speed=(int16_t *)&(get_motor_2006_measure_point(0)->speed_rpm);
    M2006Ctrl.ECDPoint=&(get_motor_2006_measure_point(0)->ecd);
    M2006Ctrl.initECD=*(M2006Ctrl.ECDPoint);
		M2006Ctrl.mode=M2006_STOP;
    
}   
static void monitorM2006ECDRound(void)
{
    uint8_t j;
    // 更新ECD
    M2006Ctrl.nowECD=*(M2006Ctrl.ECDPoint);
    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)M2006Ctrl.nowECD-(int16_t)M2006Ctrl.initECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(M2006Ctrl.mstack.stack[M2006Ctrl.mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                M2006Ctrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
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
        {//到达了一圈
            if(M2006Ctrl.mstack.stack[1]==1)//正向旋转（逆时针）
                M2006Ctrl.nowRounds +=1;
            else
                M2006Ctrl.nowRounds -=1;
            M2006Ctrl.mstack.head=0;     // 清空栈，回到初始为0的时候
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
//输入参数k 为当前状态下旋转的目标，这是一个外部函数，用于servo_task，在接收前转换
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
//一定要用pid因为有负载保持
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

