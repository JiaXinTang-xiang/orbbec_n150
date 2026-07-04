#ifndef __motor_can_H__
#define __motor_can_H__

#include "struct_typedef.h"

#include "main.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"

//电机个数
#define motor_can_N 6

//创建标识符
enum motor_mode
{
    motor_seat  = 1,
	motor_speed = 2,
	motor_M6020 = 3,
	motor_M3508 = 4
};

//电机的基础参数
typedef struct
{
	uint8_t model;			//motor_M6020 and motor_M3508
	uint8_t can;
	uint8_t id;				//1-11(M6020 1+4 - 7+4) (M3508 1 - 8)
	int16_t *speed_seat;
}motor_send;

//电机需要的全部数据
typedef struct
{
	const motor_measure_t *recv;
	motor_send motor;
	pid_type_def seat;
	pid_type_def speed;
	uint8_t mode;
}motor_can_type_def;

//电机参数的初始化结构体
typedef struct
{
	uint8_t motor_init[4]; // 电机 can id 模式
	fp32 pid_id_seat[3];
	fp32 pid_id_speed[3];
	
}motor_can_init_type_def;

void motor_can_pid(void);

void motor_can_init(void);

void motor_can_speed(uint8_t id,uint8_t can,int16_t *send);

void motor_can_mode(uint8_t id,uint8_t can,uint8_t mode);

const uint16_t *motor_can_return_ecd(uint8_t id,uint8_t can);

const int16_t *motor_can_return_speed(uint8_t id,uint8_t can);

#endif
