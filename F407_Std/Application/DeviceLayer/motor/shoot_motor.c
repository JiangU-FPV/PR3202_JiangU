#include "shoot_motor.h"
#include "Shoot.h"
#include "motor.h"

M2006_data_t Shot_Motor_BOX;
M3508_data_t Shot_Motor_FRIC[2];
int Shot_Motor_BOX_MaxOutput=8000;
int Shot_Motor_FRIC_MaxOutput=8000;

void Shot_Motor_Init(void)
{
	//拨盘电机
	pid_init(&Shot_Motor_BOX.Spd_PID);
	pid_init(&Shot_Motor_BOX.Deg_PID);
	Shot_Motor_BOX.Spd_PID.f_param_init(&Shot_Motor_BOX.Spd_PID,PID_Speed,   Shot_Motor_BOX_MaxOutput,500,3,0,8000,0,10,0.01,0);//速度环参数
	Shot_Motor_BOX.Spd_PID.f_param_init(&Shot_Motor_BOX.Deg_PID,PID_Position,8000,500,3,0,8000,0,0.4,0,0.3);//角度环参数
	//摩擦轮电机
	for(char i=0;i<2;i++)
	{
		pid_init(&Shot_Motor_FRIC[i].Pid_info);
		Shot_Motor_FRIC[i].Pid_info.f_param_init(&Shot_Motor_FRIC[i].Pid_info,PID_Speed,Shot_Motor_FRIC_MaxOutput,500,3,0,8000,0,6,0.01,0);//速度环参数
	}
}

void Shot_Motor_Cal(uint8_t SHOT_MODE)
{
	/*摩擦轮电机计算*/
	Shot_Motor_FRIC[0].Pid_info.f_cal_pid(&Shot_Motor_FRIC[0].Pid_info,Shot_Motor_FRIC[0].rpm);
	Shot_Motor_FRIC[1].Pid_info.f_cal_pid(&Shot_Motor_FRIC[1].Pid_info,Shot_Motor_FRIC[1].rpm);
	/*拨盘电机计算*/	
	if(SHOT_MODE==SERIAL)
	{
		Shot_Motor_BOX.Spd_PID.f_cal_pid(&Shot_Motor_BOX.Spd_PID,Shot_Motor_BOX.rpm);//内环计算
	}
	else if(SHOT_MODE==SINGLE)
	{
		Shot_Motor_BOX.Deg_PID.f_cal_pid(&Shot_Motor_BOX.Deg_PID,Shot_Motor_BOX.total_angle);//外环计算
		Shot_Motor_BOX.Spd_PID.target = Shot_Motor_BOX.Deg_PID.output;
		Shot_Motor_BOX.Spd_PID.f_cal_pid(&Shot_Motor_BOX.Spd_PID,Shot_Motor_BOX.rpm);//内环计算
	}

}

void Shot_Motor_check(void)
{
	for (int i = 0; i < 2; i++)
	{
		if(Shot_Motor_FRIC[i].off_cnt>=50)
		{
			Shot_Motor_FRIC[i].work_state = DEV_OFFLINE;
		}
		if(Shot_Motor_FRIC[i].work_state == DEV_OFFLINE)
		{
			Shot_Motor_FRIC[i].send_current = 0;
		}
	}
		if(Shot_Motor_BOX.off_cnt>=50)
		{
			Shot_Motor_BOX.work_state = DEV_OFFLINE;
		}
		if(Shot_Motor_BOX.work_state == DEV_OFFLINE)
		{
			Shot_Motor_BOX.send_current = 0;
		}
}

void Shot_Motor_Cut(void)
{
	Shot_Motor_FRIC[0].send_current = 0;
	Shot_Motor_FRIC[1].send_current = 0;
	Shot_Motor_BOX.send_current = 0;
}

void Shot_Motor_BOX_Update(M2006_data_t *Shot_Motor_BOX, uint8_t *rxBuf)
{
	Shot_Motor_BOX->angle		=	CAN_GetMotorAngle(rxBuf);
	Shot_Motor_BOX->rpm			=	CAN_GetMotorSpeed(rxBuf);
	Shot_Motor_BOX->current		=	CAN_GetMotorCurrent(rxBuf);
	Shot_Motor_BOX->off_cnt		=0;
	Shot_Motor_BOX->work_state 	= DEV_ONLINE;
}

void Shot_Motor_FRIC_Update(M3508_data_t *Shot_Motor_FRIC, uint8_t *rxBuf)
{
	Shot_Motor_FRIC->angle		=	CAN_GetMotorAngle(rxBuf);
	Shot_Motor_FRIC->rpm		=	CAN_GetMotorSpeed(rxBuf);
	Shot_Motor_FRIC->current	=	CAN_GetMotorCurrent(rxBuf);
	Shot_Motor_FRIC->off_cnt	=0;
	Shot_Motor_FRIC->work_state = DEV_ONLINE;
}

