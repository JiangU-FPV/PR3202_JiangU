#ifndef _SHOOT_H_
#define _SHOOT_H_
#include "shoot_motor.h"
#define SHOT_OFF	0
#define SHOT_ON 	1
#define SERIAL		0
#define SINGLE	    1
extern M3508_data_t Shot_Motor_FRIC[2];
extern M2006_data_t Shot_Motor_BOX;
void   SHOT_CTRL(void);
void SERIAL_SHOT(uint8_t mode);
void SINGAL_SHOT(uint8_t mode);
#endif
