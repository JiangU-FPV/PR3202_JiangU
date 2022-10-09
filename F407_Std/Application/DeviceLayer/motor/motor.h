#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "can.h"
#include "main.h"
#include "drv_can.h"
#include "rp_config.h"
#include "rp_can.h"
#include "dji_pid.h"

#define CHASSIS_MOTOR_STD	0x200
#define GIMBAL_MOTOR_STD	0x1ff

#define CHASSIS_MOTOR_FL	0x201U
#define CHASSIS_MOTOR_RL	0x202U
#define CHASSIS_MOTOR_RR	0x203U
#define CHASSIS_MOTOR_FR	0x204U

#define GIMBAL_CAN_ID_PITCH 0x205U
#define GIMBAL_CAN_ID_YAW 	0x206U

#define ABS(x)	( (x>0) ? (x) : (-x) )
typedef struct
{
	int16_t 	send_current;
	
	uint16_t 	angle;		//角度
	int16_t 	rpm;		//转速
	int16_t 	current;	//转矩电流
	
	int32_t 	total_angle;
	int32_t		round_cnt;
	uint16_t 	last_angle;
	uint16_t	offset_angle;
	dev_work_state_t	work_state;
	uint8_t		off_cnt;
	
}M2006_data_t;

typedef struct
{
	int16_t 	send_voltage;
	
	uint16_t 	angle;		//角度
	int16_t 	rpm;		//转速
	int16_t 	current;	//转矩电流
	int16_t 	temp;   	//温度
	
	int32_t 	total_angle;
	int32_t		round_cnt;
	uint16_t 	last_angle;
	uint16_t	offset_angle;
	dev_work_state_t	work_state;
	uint8_t		off_cnt;

//	PID_TypeDef		Pid_info;
//	PID_TypeDef		Pid_info_deg;
	
	PID_TypeDef     Mec_Ins_Pid;
	PID_TypeDef     Mec_Out_Pid;
	
	PID_TypeDef     Imu_Ins_Pid;
	PID_TypeDef     Imu_Out_Pid;	
}GM6020_data_t;

typedef struct
{
	int16_t 	send_current;
	
	uint16_t 	angle;		//角度
	int16_t 	rpm;		//转速
	int16_t 	current;	//转矩电流
	int16_t 	temp;   	//温度
	
	int32_t 	total_angle;
	int32_t		round_cnt;
	uint16_t 	last_angle;
	uint16_t	offset_angle;
	dev_work_state_t	work_state;
	uint8_t		off_cnt;
	
	
	PID_TypeDef		Pid_info;
	PID_TypeDef		Pid_info_deg;
	
}M3508_data_t;

uint16_t CAN_GetMotorAngle(uint8_t *rxData);
int16_t CAN_GetMotorSpeed (uint8_t *rxData);
int16_t CAN_GetMotorCurrent(uint8_t *rxData);
void Motor_Init(void);
void Motor_pid_init(void);
void Motor_pid_update(void);
void CAN_SendSingleData(drv_can_t *drv, int16_t txData);
void CHASSIS_Motor_Update(M3508_data_t *M3508_data, uint8_t *rxBuf);
void GIMBAL_Motor_Update (GM6020_data_t *GM6020_data, uint8_t *rxBuf);
void Motor_Send (void);
void Motor_check(void);
void Motor_TotalAngleCal_M3508 (M3508_data_t *M3508_data);
void Motor_TotalAngleCal_GM6020(GM6020_data_t *GM6020_data);
void CHASSIS_Motor_Cut(void);
void GIMBAL_Motor_Cut (void);
void ALL_Motor_Pid_Clear (void);
void GIMB_Motor_Pid_Clear(void);
void CHAS_Motor_Pid_Clear(void);
//void Motor_Update(int16_t speed,CAN_TxHeaderTypeDef CAN1_TX);
#endif
