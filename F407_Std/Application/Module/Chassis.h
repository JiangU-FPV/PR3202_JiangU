#ifndef __CHASSIS_H
#define __CHASSIS_H

#define ABS(x)	( (x>0) ? (x) : (-x) )

void chassis_update(void);
int Get_Symbol(float num);
void chassis_init(void);
#endif
