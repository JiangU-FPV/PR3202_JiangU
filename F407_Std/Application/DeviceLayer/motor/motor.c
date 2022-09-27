/**
 * @file        motor.c
 * @author      JiangU
 * @Version     V1.0
 * @date        27-September-2022
 * @brief       Robomaster motor device(M3508 and GM6020, Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "stdio.h"
#include "drv_can.h"

/* Private variables ---------------------------------------------------------*/
int16_t Send_CHAS_Array[4]; 
int16_t Send_GIMB_Array[4]; 

GM6020_data_t 	GM6020_data[2];
M3508_data_t 	M3508_data[4];

/* Private functions ---------------------------------------------------------*/

void Motor_Init(void)
{
	CAN1_Init();
	CAN2_Init();
}

/**
 *	@brief	CAN 发送单独数据
 */
void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
{
	int16_t txArr[4] = {0, 0, 0, 0};
	txArr[drv->drv_id] = txData;
	if(drv->type == DRV_CAN1)
		CAN1_SendData(drv->std_id, txArr);
	else if(drv->type == DRV_CAN2)
		CAN2_SendData(drv->std_id, txArr);
}

/**
 *	@brief	接收处理函数
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId==CHASSIS_MOTOR_FL)
	{
		CHASSIS_Motor_Update(&M3508_data[0],rxBuf);
	}
	if(canId==CHASSIS_MOTOR_RL)
	{
		CHASSIS_Motor_Update(&M3508_data[1],rxBuf);
	}
	if(canId==CHASSIS_MOTOR_RR)
	{
		CHASSIS_Motor_Update(&M3508_data[2],rxBuf);
	}
	if(canId==CHASSIS_MOTOR_FR)
	{
		CHASSIS_Motor_Update(&M3508_data[3],rxBuf);
	}
	if(canId==GIMBAL_CAN_ID_PITCH)
	{
		GIMBAL_Motor_Update(&GM6020_data[0],rxBuf);
	}
	if(canId==GIMBAL_CAN_ID_YAW)
	{
		GIMBAL_Motor_Update(&GM6020_data[1],rxBuf);
	}
}	

/**
 *	@brief	电机发送
 */
void Motor_Send(void)
{
	for (int i = 0; i < 4; i++)
	{
		Send_CHAS_Array[i] = M3508_data[i].send_current;
	}
	for (int i = 0; i < 2; i++)
	{
		Send_GIMB_Array[i] = GM6020_data[i].send_voltage;
	}
	CAN_SendData(&hcan1,CHASSIS_MOTOR_STD,Send_CHAS_Array);
	CAN_SendData(&hcan1,GIMBAL_MOTOR_STD,Send_GIMB_Array);
}

/**
 *	@brief	电机回传函数更新
 */
void CHASSIS_Motor_Update(M3508_data_t *M3508_data, uint8_t *rxBuf)
{
	M3508_data->angle		=	CAN_GetMotorAngle(rxBuf);
	M3508_data->rpm			=	CAN_GetMotorSpeed(rxBuf);
	M3508_data->current		=	CAN_GetMotorCurrent(rxBuf);
}
void GIMBAL_Motor_Update(GM6020_data_t *GM6020_data, uint8_t *rxBuf)
{
	GM6020_data->angle		=	CAN_GetMotorAngle(rxBuf);
	GM6020_data->rpm		=	CAN_GetMotorSpeed(rxBuf);
	GM6020_data->current	=	CAN_GetMotorCurrent(rxBuf);
}

/**
 *	@brief	电机获取转子角度、转速、电流
 */
uint16_t CAN_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = ((uint16_t)rxData[0] << 8| rxData[1]);
	return angle;
}
int16_t CAN_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = ((uint16_t)rxData[2] << 8| rxData[3]);
	return speed;
}
int16_t CAN_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = ((int16_t)rxData[4] << 8 | rxData[5]);
	return current;
}




/**
 *	@brief	旧函数
 */
////电机更新函数
//void Motor_Update(int16_t speed,CAN_TxHeaderTypeDef CAN1_TX)
//{
//	uint8_t TxData[8]={0};
//	
//	TxData[0]= speed>>8;
//	TxData[1]= speed;
//	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TX,TxData,0);
//	
//}

//电机接收函数
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	if(hcan->Instance==CAN1)
//	{
//		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,motor_recvBuf);//获取数据
//		
//		dog=0;
//	}
//}	
