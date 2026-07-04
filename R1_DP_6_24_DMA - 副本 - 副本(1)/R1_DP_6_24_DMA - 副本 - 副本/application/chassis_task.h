#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "struct_typedef.h"
#include "math.h"
#include "arm_math.h"
#include "remote_behaviou.h"
#include "motor_can.h"

#define pi 3.141592654
#define pi2 6.2831853084

#define Chassis_motor_N 3

typedef struct{
	uint8_t id[2];
	uint8_t can[2];
	fp32 x_y[2];//0 1
	int16_t original_seat;
}chassis_D_init;

typedef struct{
	int16_t speed;
	int16_t seat;
	int16_t *original_seat;
	fp32 angle;
	fp32 angle_last;
	fp32 speed_in;
	fp32 *x_y;
	fp32 V_X;
	fp32 V_Y;
	fp32 v;
	const uint16_t *motor_ecd;
	const int16_t *motor_speed_rpm;
	
}chassis_D_wheel;

typedef struct{
	remote_rc chassis_remote_recv;
	chassis_D_wheel chassis[Chassis_motor_N];
}chassis_task_D;

void chassis_task_D_init(void);
void chassis_D_task(void);
void chassisMath_D_chassis(chassis_D_wheel *motor,remote_rc *remote);

#endif












