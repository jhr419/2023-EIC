#include "rounds.h"
#include "CAN_cmd_2006.h"
#include "CAN_receive.h"

#define MILESTONE_NUMBER 3
#define ECD_FULL_ROUND 8192

extern motor_2006_measure_t		motor_2006[4];

struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};

typedef struct{
    struct milestoneStack_s mstack; //里程碑栈
    uint16_t    ECDPoint;           //电机ECD所在位置
    uint16_t    initECD;                //初始ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
}motor2006Control_s;

motor2006Control_s motor2006Ctrl;

static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}
static void initmotor2006ECDRoundsMonitor()  //初始化拨弹轮圈数监控
{
    motor2006Ctrl.ECDPoint= get_motor2006_motor_measure_point()->ecd;
    motor2006Ctrl.initECD=*(motor2006Ctrl.ECDPoint);
    
}   
static void monitormotor2006ECDRound(void)
{
    uint8_t j;
    // 更新ECD
    motor2006Ctrl.nowECD=*(motor2006Ctrl.ECDPoint);

    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)motor2006Ctrl.nowECD-(int16_t)motor2006Ctrl.initECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                motor2006Ctrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head]=j;
            }
        }
    }
    if(((motor2006Ctrl.mstack.head)-2)>=0)
    {
        if(motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head]==motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head-2])
            (motor2006Ctrl.mstack.head)-=2;
    }
    if(((motor2006Ctrl.mstack.head)-3)>=0)
    {
        if(motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head]==motor2006Ctrl.mstack.stack[motor2006Ctrl.mstack.head-3])
        {//到达了一圈
            if(motor2006Ctrl.mstack.stack[1]==1)//正向旋转（逆时针）
                motor2006Ctrl.nowRounds +=1;
            else
                motor2006Ctrl.nowRounds -=1;
            motor2006Ctrl.mstack.head=0;     // 清空栈，回到初始为0的时候
        }
    }
}

//这里是判断
        if(motor2006Ctrl.nowRounds<=(-nowTimeRoundThreshold)||motor2006Ctrl.nowRounds>=nowTimeRoundThreshold)
        {
            motor2006Mode=motor2006Mode_e_Stop;
            motor2006Ctrl.nowRounds=0;
        }