#ifndef _RP_CAN_H_
#define _RP_CAN_H_
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#define	RP_CAN_ID_1 0x123
#define RP_CAN_ID_2 0X124

typedef struct
{
	char age;
	float height;
}info_t;

typedef struct
{
	info_t get_info;
	info_t  my_info;
}info_pack_t;

uint8_t RP_CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, info_pack_t *info_pack);
void PACK_INIT(info_pack_t *info_pack);
void RP_Send(void);
void RP_CAN_Update(info_pack_t *info_pack,uint8_t *rxBuf);
#endif
