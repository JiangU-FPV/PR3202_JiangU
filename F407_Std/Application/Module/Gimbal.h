#ifndef __GIMBAL_H
#define __GIMBAL_H

#define MaxPitchDeg  20.0f
#define MinPitchDeg -37.91f
#define	MaxYawDeg    180.f
#define	MinYawDeg   -180.f
void Gimbal_Reset(void);
void gimbal_update(void);
void Gimbal_Init(void);
void PIT_MOTOR_MECHMAX(float pit_speed);
#endif
