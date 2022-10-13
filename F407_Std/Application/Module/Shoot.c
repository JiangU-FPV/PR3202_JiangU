#include "Shoot.h"
#include "shoot_motor.h"
#include "Judge.h"
#include "rc_potocol.h"
#include "rc_sensor.h"
extern rc_sensor_t	rc_sensor;
bool FRIC_OK=false;//Ħ���־�λ��־λ
uint8_t Last_s2 = RC_SW_MID;
uint8_t FRIC_mode = 0;
uint8_t SHOT_MODE;
uint8_t LAST_SHOT_MODE;
int FRIC_Speed = 1000;
int BOX_Speed = 2000;

void SHOT_CTRL(void)
{
	//�ж�Ħ�����Ƿ��λ
	if(ABS(Shot_Motor_FRIC[0].rpm)>=(FRIC_Speed/1.5f)&&ABS(Shot_Motor_FRIC[0].rpm)>=(FRIC_Speed/1.5f)&&FRIC_mode==1)
	{
		FRIC_OK=true;
	}
	else if(FRIC_mode==0)
	{
		FRIC_OK=false;
	}	
	//���俪��Ħ����
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
	//�л���������
	if(rc_sensor.info->s1==RC_SW_MID)
	{
		SHOT_MODE=SERIAL;
	}
	else if(rc_sensor.info->s1==RC_SW_DOWN)
	{
		SHOT_MODE=SINGLE;
	}
	//pid����
	if(LAST_SHOT_MODE!=SHOT_MODE)
	{
		Shot_Motor_BOX.Deg_PID.err=0;
		Shot_Motor_BOX.Deg_PID.all_err=0;
		Shot_Motor_BOX.Spd_PID.err=0; 
		Shot_Motor_BOX.Spd_PID.all_err=0;
		Shot_Motor_BOX.Deg_PID.target=Shot_Motor_BOX.total_angle;
	}
	//�������
	if(FRIC_OK==true)
	{
		if(SHOT_MODE==SERIAL)//��������
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
		else if(SHOT_MODE==SINGLE)//��������
		{
			if(rc_sensor.info->s2==RC_SW_UP&&Last_s2==RC_SW_MID)//s2���ϲ�һ�´�һ��
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

//����
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
//����
void SINGAL_SHOT(uint8_t mode)
{	
	if(mode==SHOT_ON)
	{
		Shot_Motor_BOX.Deg_PID.target -= 36864;
	}
	
}
