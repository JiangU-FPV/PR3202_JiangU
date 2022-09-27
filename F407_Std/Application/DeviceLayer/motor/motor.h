#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "can.h"
#include "main.h"
#include "drv_can.h"
#include "rp_config.h"

#define CHASSIS_MOTOR_STD	0x200
#define GIMBAL_MOTOR_STD	0x1ff

#define CHASSIS_MOTOR_FL	0x201U
#define CHASSIS_MOTOR_RL	0x202U
#define CHASSIS_MOTOR_RR	0x203U
#define CHASSIS_MOTOR_FR	0x204U

#define GIMBAL_CAN_ID_PITCH 0x205U
#define GIMBAL_CAN_ID_YAW 	0x206U



typedef struct
{
	int16_t 	send_current;
	
	uint16_t 	angle;		//�Ƕ�
	int16_t 	rpm;		//ת��
	int16_t 	current;	//ת�ص���
	
	int32_t 	total_angle;
	int32_t		round_cnt;
	uint16_t 	last_angle;
	uint16_t	offset_angle;
}M2006_data_t;

typedef struct
{
	int16_t 	send_voltage;
	
	uint16_t 	angle;		//�Ƕ�
	int16_t 	rpm;		//ת��
	int16_t 	current;	//ת�ص���
	int16_t 	temp;   	//�¶�
	
	int32_t 	total_angle;
	int32_t		round_cnt;
	uint16_t 	last_angle;
	uint16_t	offset_angle;
}GM6020_data_t;

typedef struct
{
	int16_t 	send_current;
	
	uint16_t 	angle;		//�Ƕ�
	int16_t 	rpm;		//ת��
	int16_t 	current;	//ת�ص���
	int16_t 	temp;   	//�¶�
	
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
void CHASSIS_Motor_Update(M3508_data_t *M3508_data, uint8_t *rxBuf);
void GIMBAL_Motor_Update(GM6020_data_t *GM6020_data, uint8_t *rxBuf);
void Motor_Send(void);

//void Motor_Update(int16_t speed,CAN_TxHeaderTypeDef CAN1_TX);
#endif
