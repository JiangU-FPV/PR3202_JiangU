/**
 * @file        Gimbal.c
 * @author      JiangU
 * @Version     V1.0
 * @date        7-October-2022
 * @brief       Robomaster gimbal driver(GM6020, Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "Gimbal.h"
#include "rc_potocol.h"
#include "rc_sensor.h"
#include "motor.h"
#include "dji_pid.h"
#include "imu_sensor.h"
#include "imu_potocol.h"
#include "math.h"
/* Private variables ---------------------------------------------------------*/
int MECH_PIT_MID = 6800;	//PIT机械中值
int MECH_YAW_MID = 2047;	//YAW机械中值，双枪2047，单枪4777
int MECH_YAW_DEG;


float PitRate = 0.00025f;  	//PIT速率
float YawRate = 0.0004f;	//YAW速率
float YawOutput = 0.0f;
float PitOutput = 0.0f;

extern system_t sys;
extern imu_sensor_t imu_sensor;
extern short gyrox, gyroy, gyroz;
extern rc_sensor_t	  rc_sensor;
extern GM6020_data_t  GM6020_data[2];

float MIC_SPD_P=0.0f;
float MIC_SPD_I=0.0f;
float MIC_DEG_P=0.0f;
float GYRO_YAW_DEG;
float imu_deg_del;
/* Private functions ---------------------------------------------------------*/
void gimbal_update(void)
{
	float pit_speed   = -(rc_sensor.info->ch1);
	float yaw_speed   = -(rc_sensor.info->ch0);
	
	PitOutput += pit_speed*PitRate;
	if(PitOutput>MaxPitchDeg)
	   PitOutput=MaxPitchDeg;
	if(PitOutput<MinPitchDeg)
	   PitOutput=MinPitchDeg;
	
	YawOutput += yaw_speed*YawRate;
	
	MECH_YAW_DEG = GM6020_data[0].angle - MECH_YAW_MID;
	
	
	if(MECH_YAW_DEG<-4096)
	{
		MECH_YAW_DEG+=8192;
	}
	if(MECH_YAW_DEG>4096)
	{
		MECH_YAW_DEG-=8192;
	}
	
	GYRO_YAW_DEG = imu_sensor.info->Longtime_imu_Array[1].imu_angle_sum-imu_deg_del;
	
	if(sys.co_mode==CO_GYRO)//陀螺仪模式
	{
		//YAW轴
		GM6020_data[0].Imu_Out_Pid.target = YawOutput;
		GM6020_data[0].Imu_Out_Pid.f_cal_pid(&GM6020_data[0].Imu_Out_Pid,GYRO_YAW_DEG); 
		GM6020_data[0].Imu_Ins_Pid.target   = GM6020_data[0].Imu_Out_Pid.output;
		GM6020_data[0].Imu_Ins_Pid.f_cal_pid(&GM6020_data[0].Imu_Ins_Pid,gyroz); 		
		GM6020_data[0].send_voltage = GM6020_data[0].Imu_Ins_Pid.output;
		//PITCH轴
		GM6020_data[1].Imu_Out_Pid.target = PitOutput;
		GM6020_data[1].Imu_Out_Pid.f_cal_pid(&GM6020_data[1].Imu_Out_Pid,imu_sensor.info->pitch);
		GM6020_data[1].Imu_Ins_Pid.target   = GM6020_data[1].Imu_Out_Pid.output;
		GM6020_data[1].Imu_Ins_Pid.f_cal_pid(&GM6020_data[1].Imu_Ins_Pid, gyroy); 
		GM6020_data[1].send_voltage = GM6020_data[1].Imu_Ins_Pid.output;
	}
	if(sys.co_mode==CO_MECH)//机械模式
	{
		//YAW轴
		GM6020_data[0].Mec_Out_Pid.target = 0;
		GM6020_data[0].Mec_Out_Pid.f_cal_pid(&GM6020_data[0].Mec_Out_Pid,MECH_YAW_DEG); 
		GM6020_data[0].Mec_Ins_Pid.target   = GM6020_data[0].Mec_Out_Pid.output;
		GM6020_data[0].Mec_Ins_Pid.f_cal_pid(&GM6020_data[0].Mec_Ins_Pid,gyroz); 
		GM6020_data[0].send_voltage = GM6020_data[0].Mec_Ins_Pid.output;
		//PITCH轴
		GM6020_data[1].Mec_Out_Pid.target = MECH_PIT_MID+PitOutput*23;
		GM6020_data[1].Mec_Out_Pid.f_cal_pid(&GM6020_data[1].Mec_Out_Pid,GM6020_data[1].angle);
		GM6020_data[1].Mec_Ins_Pid.target   = GM6020_data[1].Mec_Out_Pid.output;
		GM6020_data[1].Mec_Ins_Pid.f_cal_pid(&GM6020_data[1].Mec_Ins_Pid,gyroy); 
		GM6020_data[1].send_voltage = GM6020_data[1].Mec_Ins_Pid.output;
	}
}

void Gimbal_Init(void)
{
	YawOutput = 0.0f;
	//PitOutput = 0.0f;
	imu_deg_del = imu_sensor.info->Longtime_imu_Array[1].imu_angle_sum;
	GIMB_Motor_Pid_Clear();
	//imu_sensor.init(&imu_sensor);
}


