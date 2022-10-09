/**
 * @file        Chassis.c
 * @author      JiangU
 * @Version     V1.0
 * @date        7-October-2022
 * @brief       Robomaster chassis driver(M3508, Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "Chassis.h"
#include "rc_potocol.h"
#include "rc_sensor.h"
#include "motor.h"
#include "dji_pid.h"
#include "Judge.h"
/* Private variables ---------------------------------------------------------*/
PID_TypeDef	Chassis_Pid_info;
int MaxRPM = 6000;
int M3508Output[4];
extern GM6020_data_t 	GM6020_data[2];
extern rc_sensor_t	rc_sensor;
float  rc_rate;
extern system_t sys;
/* Private functions ---------------------------------------------------------*/
void chassis_init(void)
{
	pid_init(&Chassis_Pid_info);
	Chassis_Pid_info.f_param_init(&Chassis_Pid_info,PID_Speed,400,20,10,0,8000,0,0.5,0,0);
}

void chassis_update(void)
{
	Chassis_Pid_info.target=2047;//YAW机械中值，双枪2047，单枪4777
	
	float speedOutput[4];
	float X_speed = rc_sensor.info->ch2;
	float Y_speed = rc_sensor.info->ch3;
	float z_speed = Chassis_Pid_info.output;
	float maxspeed = 0;
	if(sys.co_mode==CO_GYRO)//陀螺仪模式
	{
		Chassis_Pid_info.f_cal_pid(&Chassis_Pid_info,GM6020_data[0].angle);
		z_speed = Chassis_Pid_info.output;
	}
	if(sys.co_mode==CO_MECH)//机械模式
	{
		z_speed = rc_sensor.info->ch0;
	}
	
	rc_rate = MaxRPM/660;
	z_speed = z_speed * rc_rate;
	
	speedOutput[0] = ( X_speed + Y_speed )*rc_rate;
	speedOutput[1] = ( X_speed - Y_speed )*rc_rate;
	speedOutput[2] = (-X_speed + Y_speed )*rc_rate;
	speedOutput[3] = (-X_speed - Y_speed )*rc_rate;
	
	maxspeed = 0;
	for(char i=0;i<4;i++)
	{
		if(ABS(speedOutput[i]) > ABS(maxspeed))
			maxspeed = speedOutput[i];		
	}
	if((ABS(maxspeed) > (MaxRPM)))
	{
		rc_rate = (MaxRPM) / ABS(maxspeed);
	}
	else
	{
		rc_rate = 1;
	}
	for(char i = 0 ; i < 4 ; i++)
	{
		speedOutput[i] = speedOutput[i] * rc_rate;
	}
	z_speed = z_speed * Get_Symbol(rc_rate);
	for(char i = 0 ; i < 4 ; i++)
	{
		M3508Output[i] = speedOutput[i]+z_speed;
	}
	maxspeed = 0;
	for(char i = 0 ; i < 4 ; i++)
	{
		if(ABS(M3508Output[i]) > ABS(maxspeed))maxspeed = M3508Output[i];		
	}
	if((ABS(maxspeed) > (MaxRPM)))
	{
		rc_rate = (MaxRPM) / ABS(maxspeed);
	}
	else
		rc_rate = 1;
	/*-目标值-*/
	for(char i = 0 ; i < 4 ; i++)
	{
		M3508Output[i] = M3508Output[i]*rc_rate;
	}
}

int Get_Symbol(float num)
{
	int symbol;
	if     (num > 0)symbol = +1;
	else if(num < 0)symbol = -1;
	else if(num== 0)symbol =  0;
	return symbol;
}


