/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CAN1_motor hcan1
#define CAN2_motor hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_M1_M4_ID = 0x200,
    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,
	
	CAN_M5_M8_ID = 0x1FF,
    CAN_M5_ID = 0x205,
    CAN_M6_ID = 0x206,
    CAN_M7_ID = 0x207,
	CAN_M8_ID = 0x208,
	
	CAN_M9_MA_ID = 0x2FF,
	CAN_M9_ID = 0x209,
    CAN_MA_ID = 0x20A,
    CAN_MB_ID = 0x20B,
	
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置CAN1和CAN2的3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_motor_reset_ID(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,7]
  * @retval         电机数据指针
  */
extern const motor_measure_t *can1_motor_measure_point(uint8_t i);

extern const motor_measure_t *can2_motor_measure_point(uint8_t i);

extern const motor_measure_t *can_motor_measure_point(uint8_t n,uint8_t can);

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)(0x205,0x206,0x207,0x208)
  * @param[in]      motor1: (0x201,0x202,0x203,0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x205,0x206,0x207,0x208) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN1_cmd_Send(int16_t *can1_motor,uint8_t mode);

extern void CAN2_cmd_Send(int16_t *can2_motor,uint8_t mode);

extern void CAN1_cmd_motor1234(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN1_cmd_motor5678(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
extern void CAN2_cmd_motor1234(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN2_cmd_motor5678(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

// CAN1 电机
extern const motor_measure_t *get_3508_motor1_measure_point(void);
extern const motor_measure_t *get_3508_motor2_measure_point(void);
extern const motor_measure_t *get_3508_motor3_measure_point(void);
extern const motor_measure_t *get_3508_motor4_measure_point(void);
extern const motor_measure_t *get_3508_motor5_measure_point(void);
extern const motor_measure_t *get_3508_motor6_measure_point(void);
extern const motor_measure_t *get_3508_motor7_measure_point(void);
extern const motor_measure_t *get_3508_motor8_measure_point(void);

// CAN2 电机
extern const motor_measure_t *get_CAN2_motor1_measure_point(void);
extern const motor_measure_t *get_CAN2_motor2_measure_point(void);
extern const motor_measure_t *get_CAN2_motor3_measure_point(void);
extern const motor_measure_t *get_CAN2_motor4_measure_point(void);
extern const motor_measure_t *get_CAN2_motor5_measure_point(void);
extern const motor_measure_t *get_CAN2_motor6_measure_point(void);
extern const motor_measure_t *get_CAN2_motor7_measure_point(void);
extern const motor_measure_t *get_CAN2_motor8_measure_point(void);


#endif
