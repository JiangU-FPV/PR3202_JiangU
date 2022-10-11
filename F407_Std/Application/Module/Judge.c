/**
 * @file        Judge.c
 * @author      JiangU
 * @Version     V1.0
 * @date        7-October-2022
 * @brief       Robomaster judge (Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "Judge.h"
#include "motor.h"
#include "rc_potocol.h"
#include "rc_sensor.h"
#include "rp_config.h"
#include "Gimbal.h"
/* Private variables ---------------------------------------------------------*/
system_t sys;
bool lock_ok = false;
bool Top_mode = false;
dev_work_state_t    Last_rc_state = DEV_OFFLINE;//上一次的在线状态
co_mode_t           Last_co_mode;//上一次的车辆状态
co_angle_logic_t co_angle_logic;//头尾状态
extern M3508_data_t 	M3508_data[4];
extern GM6020_data_t 	GM6020_data[2];
extern rc_sensor_t	rc_sensor;
extern int MECH_YAW_DEG;
/* Private functions ---------------------------------------------------------*/
void Rx_check(void)
{
	if(rc_sensor.work_state==DEV_OFFLINE)
	{
		Rx_Loss_Hand();
		
	}
	if(Last_rc_state == DEV_OFFLINE && rc_sensor.work_state==DEV_ONLINE)//从离线到在线跳变
	{
		lock_ok = false;
		Angle_Logic_Judge();
		Gimbal_Init();
	}
	Last_rc_state = rc_sensor.work_state;
}

void Rx_Loss_Hand(void)
{
	Rc_Return_Mid();
	if(lock_ok == false)
	{
		for(int i=0;i<4;i++)
		{
			M3508_data[i].Pid_info.target = 0;//刹车
		}	
	}
	
	if(ABS(M3508_data[0].rpm)<30&&
	   ABS(M3508_data[1].rpm)<30&&
	   ABS(M3508_data[2].rpm)<30&&
	   ABS(M3508_data[3].rpm)<30)//是否刹车完成
	{
		lock_ok = true;
    }
	
	if(lock_ok== true)
	{
	    CHASSIS_Motor_Cut();//刹车完成底盘卸力
		ALL_Motor_Pid_Clear();  //PID清零
	}
	GIMBAL_Motor_Cut();//云台直接卸力
	Gimbal_Init();

}

void Rc_Return_Mid(void)
{
	rc_sensor.info->ch0 = 0;
	rc_sensor.info->ch1 = 0;
	rc_sensor.info->ch2 = 0;
	rc_sensor.info->ch3 = 0;		
	rc_sensor.info->s1 = RC_SW_MID;
	rc_sensor.info->s2 = RC_SW_MID;
	rc_sensor.info->thumbwheel = 0;
}

//头尾判断
void Angle_Logic_Judge(void)
{
	if(MECH_YAW_DEG<2048&&MECH_YAW_DEG>-2048)
		co_angle_logic = LOGIC_FRONT;
	else
		co_angle_logic = LOGIC_BACK;

}

void Mode_Judge(void)
{
	if(rc_sensor.info->s1 == RC_SW_DOWN)
	{
		sys.co_mode=CO_MECH;
	}
	if(rc_sensor.info->s1 == RC_SW_MID)
	{
		sys.co_mode=CO_GYRO;
	}
	if(rc_sensor.info->thumbwheel>330&&sys.co_mode==CO_GYRO)
	{
		Top_mode = true;
	}
	if(rc_sensor.info->thumbwheel<-330)
	{
		Top_mode = false;
	}
	if(sys.co_mode != Last_co_mode)
	{
		GIMB_Motor_Pid_Clear();     //切换模式清除云台PID
		Gimbal_Init();
		Angle_Logic_Judge();
	}
	Last_co_mode = sys.co_mode;
	
}

