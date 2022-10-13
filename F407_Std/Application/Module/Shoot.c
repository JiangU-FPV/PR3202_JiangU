#include "Shoot.h"
#include "shoot_motor.h"
#include "Judge.h"
#include "rc_potocol.h"
#include "rc_sensor.h"
extern rc_sensor_t	rc_sensor;
bool FRIC_OK=false;//摩擦轮就位标志位
uint8_t Last_s2 = RC_SW_MID;
uint8_t FRIC_mode = 0;
uint8_t SHOT_MODE;
uint8_t LAST_SHOT_MODE;
int FRIC_Speed = 1000;
int BOX_Speed = 2000;

void SHOT_CTRL(void)
{
	//判断摩擦轮是否就位
	if(ABS(Shot_Motor_FRIC[0].rpm)>=(FRIC_Speed/1.5f)&&ABS(Shot_Motor_FRIC[0].rpm)>=(FRIC_Speed/1.5f)&&FRIC_mode==1)
	{
		FRIC_OK=true;
	}
	else if(FRIC_mode==0)
	{
		FRIC_OK=false;
	}	
	//跳变开启摩擦轮
	if(rc_sensor.info->s2==RC_SW_DOWN&&Last_s2==RC_SW_MID)
	{
		if(FRIC_mode==0)
		{
			Shot_Motor_FRIC[0].Pid_info.target= -FRIC_Speed;
			Shot_Motor_FRIC[1].Pid_info.target=  FRIC_Speed;
			FRIC_mode = 1;
		}
		else if(FRIC_mode==1)
		{
			Shot_Motor_FRIC[0].Pid_info.target= 0;
			Shot_Motor_FRIC[1].Pid_info.target= 0;
			FRIC_mode = 0;
		}
	}
	//切换单发连发
	if(rc_sensor.info->s1==RC_SW_MID)
	{
		SHOT_MODE=SERIAL;
	}
	else if(rc_sensor.info->s1==RC_SW_DOWN)
	{
		SHOT_MODE=SINGLE;
	}
	//pid清零
	if(LAST_SHOT_MODE!=SHOT_MODE)
	{
		Shot_Motor_BOX.Deg_PID.err=0;
		Shot_Motor_BOX.Deg_PID.all_err=0;
		Shot_Motor_BOX.Spd_PID.err=0; 
		Shot_Motor_BOX.Spd_PID.all_err=0;
		Shot_Motor_BOX.Deg_PID.target=Shot_Motor_BOX.total_angle;
	}
	//发射管理
	if(FRIC_OK==true)
	{
		if(SHOT_MODE==SERIAL)//连发发射
		{
			if(rc_sensor.info->s2==RC_SW_UP)
			{
				SERIAL_SHOT(SHOT_ON);
			}
			else if(rc_sensor.info->s2==RC_SW_MID)
			{
				SERIAL_SHOT(SHOT_OFF);
			}
		}
		else if(SHOT_MODE==SINGLE)//单发发射
		{
			if(rc_sensor.info->s2==RC_SW_UP&&Last_s2==RC_SW_MID)//s2向上拨一下打一发
			{
				SINGAL_SHOT(SHOT_ON);
			}
			else
				SINGAL_SHOT(SHOT_OFF);
		}
	}
	
	Last_s2=rc_sensor.info->s2;
	LAST_SHOT_MODE=SHOT_MODE;
	Shot_Motor_Cal(SHOT_MODE);
}

//连发
void SERIAL_SHOT(uint8_t mode)
{
	if(mode==SHOT_ON)
	{
		Shot_Motor_BOX.Spd_PID.target = -BOX_Speed;
	}
	else if(mode==SHOT_OFF)
	{
		Shot_Motor_BOX.Spd_PID.target = 0;
	}
}
//单发
void SINGAL_SHOT(uint8_t mode)
{	
	if(mode==SHOT_ON)
	{
		Shot_Motor_BOX.Deg_PID.target -= 36864;
	}
	
}
