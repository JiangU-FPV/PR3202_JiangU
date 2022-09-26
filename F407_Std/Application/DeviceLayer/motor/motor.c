#include "motor.h"
#include "stdio.h"
#include "drv_can.h"
//CAN_TxHeaderTypeDef CAN1_TxHander;
//CAN_RxHeaderTypeDef RxHeader;

//uint8_t motor_recvBuf[8];//接收数组
//int16_t dog;
//int16_t speed=200;


drv_can_t GM6020_can={
	.type=DRV_CAN1,
	.can_id=GIMBAL_CAN_ID_PITCH,
	.tx_data = CAN_SendSingleData,
};

int16_t Send_CHAS_Array[4]; 
int16_t Send_GIMB_Array[4]; 

GM6020_data_t GM6020_data[2];

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

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId==GIMBAL_CAN_ID_PITCH)
	{
		GIMBAL_Motor_Update(&GM6020_data[0],rxBuf);
	}
	if(canId==GIMBAL_CAN_ID_YAW)
	{
		GIMBAL_Motor_Update(&GM6020_data[1],rxBuf);
	}
	
}	


void Motor_Send()
{
	CAN_SendData(&hcan1,CHASSIS_MOTOR_STD,Send_CHAS_Array);
	CAN_SendData(&hcan1,GIMBAL_MOTOR_STD,Send_GIMB_Array);
}

void GIMBAL_Motor_Update(GM6020_data_t *GM6020_data, uint8_t *rxBuf)
{
	GM6020_data->angle		=	CAN_GetMotorAngle(rxBuf);
	GM6020_data->rpm		=	CAN_GetMotorSpeed(rxBuf);
	GM6020_data->current	=	CAN_GetMotorCurrent(rxBuf);
}


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
