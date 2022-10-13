#ifndef _SHOOT_MOTOR_H_
#define _SHOOT_MOTOR_H_
#include "motor.h"

typedef enum {		
	FRIC_L,
	FRIC_R,	
	BOX,
	BARREL,
} shot_motor_cnt_t;


#define Shot_Motor_FRIC_L_ID	0x201
#define Shot_Motor_FRIC_R_ID	0x202
#define Shot_Motor_BOX_ID		0x203
#define SHOT_MOTOR_STD	0x200
void Shot_Motor_Init(void);
void Shot_Motor_BOX_Update(M2006_data_t *Shot_Motor_BOX, uint8_t *rxBuf);
void Shot_Motor_FRIC_Update(M3508_data_t *Shot_Motor_FRIC, uint8_t *rxBuf);
void Shot_Motor_check(void);
void Shot_Motor_Cal(uint8_t SHOT_MODE);
void Shot_Motor_Cut(void);
#endif
