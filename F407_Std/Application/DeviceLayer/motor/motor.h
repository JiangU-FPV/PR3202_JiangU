#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "can.h"
#include "main.h"
#include "drv_can.h"
#include "rp_config.h"

#define GIMBAL_CAN_ID_PITCH 0x205U
#define GIMBAL_CAN_ID_YAW 	0x206U
#define GIMBAL_MOTOR_STD	0x1ff
#define CHASSIS_MOTOR_STD	0x200

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
}M3508_data_t;


void Motor_Init(void);
void CAN_SendSingleData(drv_can_t *drv, int16_t txData);
uint16_t CAN_GetMotorAngle(uint8_t *rxData);
int16_t CAN_GetMotorSpeed(uint8_t *rxData);
int16_t CAN_GetMotorCurrent(uint8_t *rxData);
void GIMBAL_Motor_Update(GM6020_data_t *GM6020_data, uint8_t *rxBuf);
//void Motor_Update(int16_t speed,CAN_TxHeaderTypeDef CAN1_TX);
#endif
