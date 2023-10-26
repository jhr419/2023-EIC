/**
 * encoding:GB2312
 * @file M2006_task.c
 * @author Brandon
 * @brief  搞定多圈旋转问题
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     10.15           Brandon         done
 */
#include "CAN_cmd_all.h"
#include "main.h"
#include "cmsis_os.h"
#include "motor_ctrl.h"
#include "commu_task.h"
#include "pid.h"
#include "CAN_receive.h"
//需要中断触发，触发时机：在收到机械臂二抓取和放置的同时进行操作，在其他任务中給工作量
//需要锁死在一个ECD，进入stop模式的时候正好锁住ECD
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;


extern CAN_TxHeaderTypeDef  motor_2006_tx_message;
extern uint8_t              motor_2006_can_send_data[8];

//控制变量，有待修改
#define Reduction_ratio_M2006 1/36 
#define SPEED_M2006 2000
#define MAX_SPEED_M2006 1000
#define MAX_IOUT_SPEED_M2006 500
#define MAX_OUT		12000.0
#define MAX_IOUT  1200.0

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
	  uint16_t    targetECD;              //目标ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
		int16_t     targetRounds;           //目标转到的圈数
		int16_t	    set_speed;								//目标转速
		int16_t*    now_speed;								//当前转速
		enum M2006Mode mode;								//当前2006的模式
		pid_t       pid;                    //速度PID
	  pid_t       angle_pid;               //角度PID
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
//开启四个M2006控制
struct M2006Control_s M2006Ctrl[4];
float pid_m2006[2][3]={{5,0,0},{5,0,0}};
float angle_pid_m2006[2][3]={{1,0,0.1},{1,0,0.1}};
static void initM2006ECDRoundsMonitor()  //初始化拨弹轮圈数监控
{
	for(int i=0;i<4;i++)
	{
	  PID_init(&M2006Ctrl[i].pid, PID_POSITION, pid_m2006[i], MAX_OUT , MAX_IOUT);
		PID_init(&M2006Ctrl[i].angle_pid, PID_POSITION, angle_pid_m2006[i], MAX_SPEED_M2006 , MAX_IOUT_SPEED_M2006);
	//获取那个电机的数值指针
		M2006Ctrl[i].now_speed=(int16_t *)&(get_motor_2006_measure_point(i)->speed_rpm);
    M2006Ctrl[i].ECDPoint=&(get_motor_2006_measure_point(i)->ecd);
    M2006Ctrl[i].initECD=*(M2006Ctrl[i].ECDPoint);
		M2006Ctrl[i].mode=M2006_STOP;
		M2006Ctrl[i].targetECD = ecd_format(M2006Ctrl[i].initECD);
	}
}   
void startM2006Monitor()
{
	initM2006ECDRoundsMonitor();
}
static void monitorM2006ECDRound(void)
{
	for(int i=0;i<4;i++)
	{
    uint8_t j;
    // 更新ECD
    M2006Ctrl[i].nowECD=*(M2006Ctrl[i].ECDPoint);
    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)M2006Ctrl[i].nowECD-(int16_t)M2006Ctrl[i].initECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                M2006Ctrl[i].mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head]=j;
            }
        }
    }
    if(((M2006Ctrl[i].mstack.head)-2)>=0)
    {
        if(M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head]==M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head-2])
            (M2006Ctrl[i].mstack.head)-=2;
    }
    if(((M2006Ctrl[i].mstack.head)-3)>=0)
    {
        if(M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head]==M2006Ctrl[i].mstack.stack[M2006Ctrl[i].mstack.head-3])
        {//到达了一圈
            if(M2006Ctrl[i].mstack.stack[1]==1)//正向旋转（逆时针）
                M2006Ctrl[i].nowRounds +=1;
            else
                M2006Ctrl[i].nowRounds -=1;
            M2006Ctrl[i].mstack.head=0;     // 清空栈，回到初始为0的时候
        }
    }
	}
}
void refresh_M2006_ctrl(){
	for(int i=0;i<4;i++)
	{
		if(M2006Ctrl[i].nowRounds>M2006Ctrl[i].targetRounds)
			M2006Ctrl[i].mode=M2006_ROTATE_BACKWARD;
		if(M2006Ctrl[i].nowRounds<M2006Ctrl[i].targetRounds)
			M2006Ctrl[i].mode=M2006_ROTATE_FORWARD;
		if(M2006Ctrl[i].nowRounds==M2006Ctrl[i].targetRounds)
		{
			M2006Ctrl[i].mode=M2006_STOP;
			M2006Ctrl[i].targetRounds = M2006Ctrl[i].nowRounds;
		}
  }
}
//输入参数k 为当前状态下旋转的目标，这是一个外部函数，用于servo_task，在接收前转换
void set_M2006_rotate_rounds(int i,int k)
{
		if(M2006Ctrl[i].mode==M2006_STOP)
		{
			M2006Ctrl[i].targetRounds=M2006Ctrl[i].nowRounds+k;
			if(k>0)
				M2006Ctrl[i].mode=M2006_ROTATE_FORWARD;
			else if(k<0)
				M2006Ctrl[i].mode=M2006_ROTATE_BACKWARD;
			else
				M2006Ctrl[i].mode=M2006_STOP;
		}
}
//一定要用pid因为有负载保持
void set_M2006_speed()
{
	for(int i=0;i<4;i++)
	{
		if(M2006Ctrl[i].mode==M2006_ROTATE_FORWARD)
		{
			M2006Ctrl[i].set_speed=SPEED_M2006;
		}
		else if (M2006Ctrl[i].mode==M2006_ROTATE_BACKWARD)
		{
			M2006Ctrl[i].set_speed=-SPEED_M2006;
		}
		else if (M2006Ctrl[i].mode==M2006_STOP)
		{
			M2006Ctrl[i].set_speed=(int)PID_ECD_calc(&M2006Ctrl[i].angle_pid,*M2006Ctrl[i].ECDPoint,M2006Ctrl[i].targetECD);
		}
  }
}
void set_M2006_current(){
	int current[4];
	for(int i=0;i<4;i++)
		current[i]=PID_calc(&M2006Ctrl[i].pid,*M2006Ctrl[i].now_speed,M2006Ctrl[i].set_speed);
	CAN_cmd_2006(current[0],current[1],current[2],current[3]);
}
void m2006_task(void const* argument)
{
	while(1)
	{
		monitorM2006ECDRound();
		refresh_M2006_ctrl();
		set_M2006_speed();
		set_M2006_current();
		osDelay(1);
	}
}

