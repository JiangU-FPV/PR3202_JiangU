/**
 * @file        motor.c
 * @author      JiangU
 * @Version     V1.0
 * @date        27-September-2022
 * @brief       Robomaster motor device(M3508 and GM6020, Based on HAL).
 */
//机械中值
//GM6020_data[0].Pid_info_deg.target=4777;//单枪
//GM6020_data[0].Pid_info_deg.target=2046;//双枪
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "stdio.h"
#include "drv_can.h"
#include "rp_can.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Judge.h"
/* Private variables ---------------------------------------------------------*/
int16_t Send_CHAS_Array[4]; 
int16_t Send_GIMB_Array[4]; 

M3508_data_t 	M3508_data[4];
GM6020_data_t 	GM6020_data[2];

int chassis_maxoutput = 7000;//8000;
int gimbal_maxoutput  = 20000;//20000;

extern info_pack_t  info_pack;

extern int M3508Output[4];
extern float YawOutput;
extern float PitOutput;
extern float rc_rate;
extern co_mode_t   Last_co_mode;
/* Private functions ---------------------------------------------------------*/


void Motor_Init(void)
{
	CAN1_Init();
	CAN2_Init();
	chassis_init();
	Motor_pid_init();
	Last_co_mode=CO_GYRO;
}

void Motor_pid_init(void)
{
	for(int i=0;i<4;i++)
	{
		pid_init(&M3508_data[i].Pid_info);
		pid_init(&M3508_data[i].Pid_info_deg);
		M3508_data[i].Pid_info.f_param_init(&M3508_data[i].Pid_info,PID_Speed,chassis_maxoutput,3000,3,0,8000,0,6,0.01,0);	//速度环参数
		M3508_data[i].Pid_info_deg.f_param_init(&M3508_data[i].Pid_info_deg,PID_Position,8000,100,10,0,8000,0,0.3,0,10);    //角度环参数
	}
	for(int i=0;i<2;i++)
	{
		pid_init(&GM6020_data[i].Mec_Ins_Pid);
		pid_init(&GM6020_data[i].Mec_Out_Pid);
		pid_init(&GM6020_data[i].Imu_Ins_Pid);
		pid_init(&GM6020_data[i].Imu_Out_Pid);
	}
	//机械模式
	//yaw轴
	GM6020_data[0].Mec_Ins_Pid.f_param_init(&GM6020_data[0].Mec_Ins_Pid,PID_Speed,gimbal_maxoutput,3000,3,0,8000,0,20,0.2,0);		//速度环参数
	GM6020_data[0].Mec_Out_Pid.f_param_init(&GM6020_data[0].Mec_Out_Pid,PID_Position,8000,100,0,0,8000,0,15,0,30); 		//角度环参数	
	//pitch轴
	GM6020_data[1].Mec_Ins_Pid.f_param_init(&GM6020_data[1].Mec_Ins_Pid,PID_Speed,gimbal_maxoutput,3000,3,0,8000,0,20,0.2,0);		//速度环参数
	GM6020_data[1].Mec_Out_Pid.f_param_init(&GM6020_data[1].Mec_Out_Pid,PID_Position,8000,100,0,0,8000,0,15,0,0);        //角度环参数
	
	//陀螺仪模式
	//yaw轴
	GM6020_data[0].Imu_Ins_Pid.f_param_init(&GM6020_data[0].Imu_Ins_Pid,PID_Speed,gimbal_maxoutput,3000,3,0,8000,0,20,0.2,0);		//速度环参数
	GM6020_data[0].Imu_Out_Pid.f_param_init(&GM6020_data[0].Imu_Out_Pid,PID_Position,8000,100,0,0,8000,0,250,0,0); 		//角度环参数		
	//pitch轴
	GM6020_data[1].Imu_Ins_Pid.f_param_init(&GM6020_data[1].Imu_Ins_Pid,PID_Speed,gimbal_maxoutput,3000,3,0,8000,0,20,0.3,0);		//速度环参数
	GM6020_data[1].Imu_Out_Pid.f_param_init(&GM6020_data[1].Imu_Out_Pid,PID_Position,8000,100,0,0,8000,0,260,0,0);        //角度环参数
}

/**
 *	@brief	计算电机输出值
 */
void Motor_info_update(void)
{
	for(int i=0;i<2;i++)
	{
		Motor_TotalAngleCal_GM6020(&GM6020_data[i]);
	}
	gimbal_update();
	chassis_update();
	for(int i=0;i<4;i++)
	{
		//Motor_TotalAngleCal_M3508(&M3508_data[i]);
		//M3508_data[i].Pid_info_deg.f_cal_pid(&M3508_data[i].Pid_info_deg,M3508_data[i].total_angle); //3508角度环
		M3508_data[i].Pid_info.target = M3508Output[i];
		M3508_data[i].Pid_info.f_cal_pid(&M3508_data[i].Pid_info,M3508_data[i].rpm); 
	}
}


/**
 *	@brief	电机发送
 */
void Motor_Send(void)
{            
	Motor_info_update();                                                //计算电机输出值
	for (int i = 0; i < 4; i++)
	{
		M3508_data[i].send_current=	M3508_data[i].Pid_info.output;      //先赋值
	}
	Rx_check();                                                         //遥控器检测
	Motor_check();														//电机检测
	for (int i = 0; i < 4; i++)
	{
		Send_CHAS_Array[i] = M3508_data[i].send_current;
		M3508_data[i].off_cnt++;
	}
	for (int i = 0; i < 2; i++)
	{
		Send_GIMB_Array[i] = GM6020_data[i].send_voltage;
		GM6020_data[i].off_cnt++;
	}
	CAN_SendData(&hcan1,CHASSIS_MOTOR_STD,Send_CHAS_Array);
	CAN_SendData(&hcan1,GIMBAL_MOTOR_STD ,Send_GIMB_Array);
}

/**
 *	@brief	电机卸力
 */
void CHASSIS_Motor_Cut(void)
{
	for(int i=0;i<4;i++)
	{
		M3508_data[i].send_current = 0;
	}
}
void GIMBAL_Motor_Cut(void)
{
	for(int i=0;i<2;i++)
	{
		GM6020_data[i].send_voltage=0;
	}
}

/**
 *	@brief	电机PID清零
 */
void ALL_Motor_Pid_Clear(void)
{
	CHAS_Motor_Pid_Clear();
	GIMB_Motor_Pid_Clear();
}

void CHAS_Motor_Pid_Clear(void)
{
	for(int i=0;i<4;i++)
	{
		M3508_data[i].Pid_info.all_err = 0;
		M3508_data[i].Pid_info.err = 0;
	}
}

void GIMB_Motor_Pid_Clear(void)
{
	for(int i=0;i<2;i++)
	{
		GM6020_data[i].Mec_Ins_Pid.all_err 	= 0;
		GM6020_data[i].Mec_Ins_Pid.err 		= 0;
		GM6020_data[i].Mec_Out_Pid.all_err 	= 0;
		GM6020_data[i].Mec_Out_Pid.err 		= 0;
		GM6020_data[i].Imu_Ins_Pid.all_err 	= 0;
		GM6020_data[i].Imu_Ins_Pid.err 		= 0;
		GM6020_data[i].Imu_Out_Pid.all_err 	= 0;
		GM6020_data[i].Imu_Out_Pid.err 		= 0;		
	}
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
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId==RP_CAN_ID_1)
	{
		RP_CAN_Update(&info_pack,rxBuf);
	}
}


/**
 *	@brief	电机回传数据更新
 */
void CHASSIS_Motor_Update(M3508_data_t *M3508_data, uint8_t *rxBuf)
{
	M3508_data->angle		=	CAN_GetMotorAngle(rxBuf);
	M3508_data->rpm			=	CAN_GetMotorSpeed(rxBuf);
	M3508_data->current		=	CAN_GetMotorCurrent(rxBuf);
	M3508_data->off_cnt=0;
	M3508_data->work_state = DEV_ONLINE;
}
void GIMBAL_Motor_Update(GM6020_data_t *GM6020_data, uint8_t *rxBuf)
{
	GM6020_data->angle		=	CAN_GetMotorAngle(rxBuf);
	GM6020_data->rpm		=	CAN_GetMotorSpeed(rxBuf);
	GM6020_data->current	=	CAN_GetMotorCurrent(rxBuf);
	GM6020_data->off_cnt=0;
	GM6020_data->work_state = DEV_ONLINE;
}

/**
 *	@brief	电机失联检测
 */
void Motor_check(void)
{
	for (int i = 0; i < 4; i++)
	{
		if(M3508_data[i].off_cnt>=50)
		{
			M3508_data[i].work_state = DEV_OFFLINE;
		}
		if(M3508_data[i].work_state == DEV_OFFLINE)
		{
			M3508_data[i].send_current = 0;
		}
	}
	for (int i = 0; i < 2; i++)
	{
		if(GM6020_data[i].off_cnt>=50)
		{
			GM6020_data[i].work_state = DEV_OFFLINE;
		}
		if(GM6020_data[i].work_state == DEV_OFFLINE)
		{
			GM6020_data[i].send_voltage = 0;
		}
	}
}

/**
 *	@brief	电机角度计算
 */
void Motor_TotalAngleCal_M3508(M3508_data_t *M3508_data)
{
	if(M3508_data->angle-M3508_data->last_angle>4096){
		M3508_data->round_cnt--;
	}
	else if (M3508_data->angle-M3508_data->last_angle< -4096){
		M3508_data->round_cnt ++;
	}
	M3508_data->total_angle = M3508_data->round_cnt * 8192 + M3508_data->angle	- M3508_data->offset_angle;
	int res1, res2, delta;
	if(M3508_data->angle < M3508_data->last_angle){			//可能的情况
		res1 = M3508_data->angle + 8192 - M3508_data->last_angle;	//正转，delta=+
		res2 = M3508_data->angle - M3508_data->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = M3508_data->angle - 8192 - M3508_data->last_angle ;//反转	delta -
		res2 = M3508_data->angle - M3508_data->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;
	M3508_data->total_angle += delta;
	M3508_data->last_angle = M3508_data->angle;
}

void Motor_TotalAngleCal_GM6020(GM6020_data_t *GM6020_data)
{
	if(GM6020_data->angle-GM6020_data->last_angle>4096){
		GM6020_data->round_cnt--;
	}
	else if (GM6020_data->angle-GM6020_data->last_angle< -4096){
		GM6020_data->round_cnt ++;
	}
	GM6020_data->total_angle = GM6020_data->round_cnt * 8192 + GM6020_data->angle	- GM6020_data->offset_angle;
	int res1, res2, delta;
	if(GM6020_data->angle < GM6020_data->last_angle){			//可能的情况
		res1 = GM6020_data->angle + 8192 - GM6020_data->last_angle;	//正转，delta=+
		res2 = GM6020_data->angle - GM6020_data->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = GM6020_data->angle - 8192 - GM6020_data->last_angle ;//反转	delta -
		res2 = GM6020_data->angle - GM6020_data->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;
	GM6020_data->total_angle += delta;
	GM6020_data->last_angle = GM6020_data->angle;
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

///**
// *	@brief	CAN 发送单独数据
// */
//void CAN_SendSingleData(drv_can_t *drv, int16_t txData)
//{
//	int16_t txArr[4] = {0, 0, 0, 0};
//	txArr[drv->drv_id] = txData;
//	if(drv->type == DRV_CAN1)
//		CAN1_SendData(drv->std_id, txArr);
//	else if(drv->type == DRV_CAN2)
//		CAN2_SendData(drv->std_id, txArr);
//}
