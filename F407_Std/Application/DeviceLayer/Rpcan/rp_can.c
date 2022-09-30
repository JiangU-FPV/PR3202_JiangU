/**
 * @file        rp_can.c
 * @author      JiangU
 * @Version     V1.0
 * @date        27-September-2022
 * @brief       Robopilots can (RP F407 board, Based on HAL).
 */
 
/* Includes ------------------------------------------------------------------*/

#include "rp_can.h"
#include "drv_can.h"
#include "string.h"
/* Private variables ---------------------------------------------------------*/

info_pack_t  info_pack;
extern CAN_HandleTypeDef hcan1;
/* Private functions ---------------------------------------------------------*/

void PACK_INIT(info_pack_t *info_pack)
{
	info_pack->my_info.age=8;
	info_pack->my_info.height=123.7857f;
}

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
}RPCAN_TxFrameTypeDef;

void RP_Send(void)
{
	PACK_INIT(&info_pack);
	RP_CAN_SendData(&hcan1,RP_CAN_ID_1,&info_pack);
}

uint8_t RP_CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, info_pack_t *info_pack)
{
    uint32_t txMailBox;
	
	RPCAN_TxFrameTypeDef hcan1TxFrame;
    RPCAN_TxFrameTypeDef *txFrame= &hcan1TxFrame;
	
    txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;
//	方法1	
	txFrame->data[0]=(uint8_t)info_pack->my_info.age;
	txFrame->data[1]=(uint8_t)(((int)((info_pack->my_info.height+0.05f)*10))>>24);
	txFrame->data[2]=(uint8_t)(((int)((info_pack->my_info.height+0.05f)*10))>>16);
	txFrame->data[3]=(uint8_t)(((int)((info_pack->my_info.height+0.05f)*10))>>8);
	txFrame->data[4]=(uint8_t)((int)((info_pack->my_info.height+0.05f)*10));
//	方法2	
//    uint8_t *p;
//    p=(uint8_t*)&info_pack->my_info.height;
//    for (int i = 1; i < 5; i++)
//    {
//        txFrame->data[i]=*(p+i-1);
//    }

    if(HAL_CAN_AddTxMessage(hcan, &txFrame->header,txFrame->data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

void RP_CAN_Update(info_pack_t *info_pack,uint8_t *rxBuf)
{	
	
//	方法1	
	
	info_pack->get_info.age=(int8_t)rxBuf[0];
	info_pack->get_info.height=(float)((int)(rxBuf[1]<<24|rxBuf[2]<<16|rxBuf[3]<<8|rxBuf[4]))/10.0f;
	
//	方法2
//	uint8_t *fp;
// 	float height_rec;
//	fp=(uint8_t*)&height_rec;
//	for (int i = 0; i < 4; i++)
//    {
//        *(fp+i)=rxBuf[i+1];
//    }
//	info_pack->get_info.height=height_rec; 
	
//    fp=(float*)&rxBuf[1];
//	info_pack->get_info.height=*fp;
}

