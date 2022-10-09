#ifndef __GIMBAL_H
#define __GIMBAL_H

#define MaxPitchDeg  20.0f
#define MinPitchDeg -33.0f

void Gimbal_Reset(void);
void gimbal_update(void);
void Gimbal_Init(void);
#endif
