#ifndef __CHASSISMATH_H__
#define __CHASSISMATH_H__

#include "struct_typedef.h"
#include "math.h"
#include "arm_math.h"
#include "remote_behaviou.h"

#define pi 3.141592654

#define Chassis_motor_N 4


typedef struct{
	int8_t *x_y;
	fp32 V_X;
	fp32 V_Y;
	int16_t speed;
	uint8_t *wheel_angle;
}chassis_wheel;

typedef struct{
	int16_t speed;
	int16_t seat;
	int16_t *original_seat;
	fp32 angle;
	fp32 angle_last;
	fp32 speed_in;
	int8_t *x_y;
	fp32 V_X;
	fp32 V_Y;
	const uint16_t *motor_ecd;
}chassis_D_wheel;

void chassisMath_M_chassis(chassis_wheel *motor , remote_rc *remote);
void chassisMath_D_chassis(chassis_D_wheel *motor , remote_rc *remote);

#endif

