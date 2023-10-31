#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassis_filter.h"
#include "CAN_cmd_all.h"
#include "CAN_receive.h"
#include "motor_ctrl.h"
#include "commu_task.h"
#include "servo.h"
#include "pid.h"
#include "stdlib.h"

#define MAX_OUT  8000.0
#define MAX_IOUT 4000.0
#define MOTOR_DISTANCE_TO_CENTER 50.0f //需要改
#define RAD_TO_DEGREE 2*PI/360
#define RIDIUS 30.0f									 //需要改
#define	PID_3508_P	5.9
#define PID_3508_I	0.6
#define PID_3508_D	4.0
#define FILTER_NUM 50
#define FILTER_FRAME_PERIOD 1

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		

extern servo_t servo[8];
static fp32 wheel_exp_rpm[4];
static fp32 wheel_set_rpm[4];
extern move_cmd_t my_move;
const motor_3508_measure_t*	chassis_motor[4];
const fp32 pid_k[3]={PID_3508_P, PID_3508_I, PID_3508_D};
pid_t chassis_v_pid[3];
pid_t chassis_w_pid;
const fp32 pid_chassis_v[3]={100.0, 0.0, 5800.0};
const fp32 pid_chassis_w[3]={300.0, 0.0, 50.0};
//500 50 100
const fp32 pid_chassis_dist[3] = {20.0,0.003,0.005};
fp32 v[3];
fp32 v_tmp[3];
fp32 point_decoded[2];
pid_t pid[4];
extern action_data my_action_data;
extern car_data_s 		my_car_data; 
void chassis_pid_init(void)
{
	PID_init(&chassis_v_pid[0], PID_POSITION, pid_chassis_v, 6000, MAX_IOUT);
	PID_init(&chassis_v_pid[1], PID_POSITION, pid_chassis_v, 6000, MAX_IOUT);
	PID_init(&chassis_v_pid[2], PID_POSITION, pid_chassis_dist, 6000, MAX_IOUT);
	PID_init(&chassis_w_pid, PID_POSITION, pid_chassis_w, 6000, MAX_IOUT);
}

fp32 deadbond(fp32 err, fp32 bond, fp32 data)
{
	if(fabsf( err ) <= bond)
	{
		return 0;
	}
	else 
	{
		return data;
	}
}

fp32 angle_limit(fp32 ref, fp32 set)
{
	if(set>0&&set<180&&ref>0&&ref<180)
	{
		return set;
	}
		if(set>0&&set<180&&ref>0&&ref<180)
	{
		return set;
	}
	
	if(set > ref)
	{
		if(set - ref > ref - set + 360.0)
		{
			return set - 360.0;//error = set - MAX_MOTOR_ECD - ref
		}
		else if(set - ref < ref - set + 360.0)
		{
			return set;//error = set -ref
		}
	}
	else if(set < ref){
		if(ref - set > set - ref + 360.0)
		{
			return set + 360.0;//error = 
		}
		else if(ref - set < set - ref + 360.0)
		{
			return set;//error
		}
	}
}

fp32 PID_angle_calc(pid_t *pid, fp32 ref, fp32 set)
{
	PID_calc(pid, ref, angle_limit(ref, set));
}

void goal_to_v(move_cmd_t* move)
{
//	PID_calc(&chassis_v_pid[0], my_action_data.x.data, 	  move->x_goal.data);
//	PID_calc(&chassis_v_pid[1], my_action_data.y.data, 		move->y_goal.data);
//	v_tmp[0] = deadbond(chassis_v_pid[0].error[0] , 3.0, chassis_v_pid[0].out);
//	v_tmp[1] = deadbond(chassis_v_pid[1].error[0] , 3.0, chassis_v_pid[1].out);
////	//计算角度变化量
  PID_angle_calc(&chassis_w_pid, 	  my_action_data.yaw.data,  move->w_goal.data);
//	
	//计算位置变化量
	fp32 dx,dy,dist;
	dx = my_action_data.x.data-move->x_goal.data;
	dy = my_action_data.y.data-move->y_goal.data;

	//计算出绝对距离
	arm_sqrt_f32(dx*dx+dy*dy,&dist);
	dist=deadbond(dist,10,dist);
	
	PID_calc(&chassis_v_pid[2],dist,0);
	
	fp32 v_out = deadbond(chassis_v_pid[2].out,500.0,chassis_v_pid[2].out); 
 if(dist<=10)
 {
	 v_tmp[0] = 0;
	 v_tmp[1] = 0;
 }
 else
 {
	v_tmp[0] = v_out * dx / dist;
	v_tmp[1] = v_out * dy / dist;
 }
	
//	uart7_printf("%f,%f,%f\r\n",chassis_v_pid[2].Ki,chassis_v_pid[2].Iout,chassis_v_pid[2].out);
	v[0] = v_tmp[0]*cos(my_action_data.yaw.data * PI / 180.0) + v_tmp[1]*sin(my_action_data.yaw.data * PI / 180.0);
	v[1] =-v_tmp[0]*sin(my_action_data.yaw.data * PI / 180.0) + v_tmp[1]*cos(my_action_data.yaw.data * PI / 180.0);
	
	v[2] = deadbond(chassis_w_pid.out, 500.0, chassis_w_pid.out);	
	 
  
	

}
void chassis_v_to_mecanum_speed()
{
	//vx_err = 500;
	//vy_err = 500;
//	vw_err = 600;
	//uart7_printf("%f,%f,%f\r\n",chassis_w_pid.out,chassis_w_pid.error[0],v[2]);
	if(my_car_data.stuff_num!=100)
	{
		wheel_exp_rpm[0] = (int)((v[0] - v[1] - MOTOR_DISTANCE_TO_CENTER * v[2] * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
		wheel_exp_rpm[1] = (int)(( v[0] + v[1] - MOTOR_DISTANCE_TO_CENTER * v[2] * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
		wheel_exp_rpm[2] = (int)(( -v[0] + v[1] - MOTOR_DISTANCE_TO_CENTER * v[2] * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
		wheel_exp_rpm[3] = (int)((-v[0] - v[1] - MOTOR_DISTANCE_TO_CENTER * v[2] * RAD_TO_DEGREE) / ( 2 * PI * RIDIUS)*60);
	}
}

void chassis_init(void)
{
	
	for(int i=0;i<4;i++)
	{
		PID_init(&pid[i], PID_POSITION, pid_k, MAX_OUT, MAX_IOUT);
		wheel_set_rpm[i] = 0.0;
		chassis_motor[i] = get_motor_3508_measure_point(i);
	}
}
fp32 PID_limit_calc(pid_t *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		if(pid->error[0]-pid->error[1]>100)
			pid->error[0]=pid->error[0]+100;
		else if(pid->error[0]-pid->error[1]<100)
			pid->error[0]=pid->error[0]-100;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
				if (pid->Iout*pid->error[0] < 0)
				{
					pid->Iout = 0;
				}
        LimitMax(pid->out, pid->max_out);
    }
}
void chassis_pid_calc(fp32* wheel_exp_rpm)
{
  goal_to_v(&my_move);
//	uart7_printf("%f,%f,%f\r\n",v[0],v[1],v[2]);
	
	chassis_v_to_mecanum_speed();
	for(int i=0; i<4; i++)
	{
		//first_order_filter_cali(&rpm_filter, wheel_exp_rpm[i]);
		
		PID_calc(&pid[i], chassis_motor[i]->speed_rpm, wheel_exp_rpm[i]);
		wheel_set_rpm[i] = pid[i].out;
	}
	
}

void chassis_ctrl(void)
{
		chassis_pid_calc(wheel_exp_rpm);
		CAN_cmd_chassis(wheel_set_rpm[0], wheel_set_rpm[1], wheel_set_rpm[2], wheel_set_rpm[3]);
}

void chassis_task(void const* argument){
	chassis_init();
	chassis_pid_init();
	
	while(1){
		chassis_ctrl();
		//计算6020电流
		give_pid_current_6020();
		uart7_printf("%f,%f,%f,%f\r\n",my_move.x_goal.data,my_move.y_goal.data,my_action_data.x.data,my_action_data.y.data);

		osDelay(1);
		
	}
}
