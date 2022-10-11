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
#include "Gimbal.h"
#include "math.h"
/* Private variables ---------------------------------------------------------*/
PID_TypeDef	Chassis_Pid_info;
int MaxRPM = 8000;
int M3508Output[4];
extern GM6020_data_t 	GM6020_data[2];
extern rc_sensor_t	rc_sensor;
float  rc_rate;
extern system_t sys;
extern int MECH_YAW_DEG;
extern bool Top_mode;

/* Private functions ---------------------------------------------------------*/
void chassis_init(void)
{
	pid_init(&Chassis_Pid_info);
	Chassis_Pid_info.f_param_init(&Chassis_Pid_info,PID_Speed,450,20,10,0,8000,0,0.4,0,10);
}

void chassis_update(void)
{
	Chassis_Pid_info.target=0;
	
	float speedOutput[4];
	float X_speed = rc_sensor.info->ch2;
	float Y_speed = rc_sensor.info->ch3;
	float X_speed_Top;
	float Y_speed_Top;
	float z_speed;
	float maxspeed = 0;
	float Top_deg;
	if(sys.co_mode==CO_GYRO)//陀螺仪模式
	{
		if(Top_mode==false)//跟随
		{
			Chassis_Pid_info.f_cal_pid(&Chassis_Pid_info,MECH_YAW_DEG);
			z_speed = Chassis_Pid_info.output;
		}
		else if(Top_mode==true)//小陀螺
		{
			Top_deg=(MECH_YAW_DEG)/4096.0f*3.14159f;
			X_speed_Top = (X_speed *cos(Top_deg)-Y_speed*sin(Top_deg));
			Y_speed_Top = (Y_speed*cos(Top_deg) +X_speed*sin(Top_deg));
			X_speed=X_speed_Top;
			Y_speed=Y_speed_Top;
			z_speed=200; 
		}
	}
	if(sys.co_mode==CO_MECH)//机械模式
	{
		z_speed = rc_sensor.info->ch0*0.7f;
	}
	
	//麦轮解算
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


