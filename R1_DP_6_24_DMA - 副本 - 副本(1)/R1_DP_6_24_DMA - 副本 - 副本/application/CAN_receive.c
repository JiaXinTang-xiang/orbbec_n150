#include "CAN_receive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);       \
        (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);   \
        (ptr)->temperate = (data)[6];                                   \
    }

motor_measure_t CAN1_motor_chassis[11];
motor_measure_t CAN2_motor_chassis[11];

static CAN_TxHeaderTypeDef  motor_tx_message;
static uint8_t              motor_can_send_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	if(hcan == &hcan1){
		
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		switch (rx_header.StdId)
		{
			case CAN_M1_ID:
			case CAN_M2_ID:
			case CAN_M3_ID:
			case CAN_M4_ID:
			case CAN_M5_ID:
			case CAN_M6_ID:
			case CAN_M7_ID:
			case CAN_M8_ID:
			case CAN_M9_ID:
			case CAN_MA_ID:
			case CAN_MB_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_M1_ID;
				get_motor_measure(&CAN1_motor_chassis[i], rx_data);
				break;
			}

			default:
			{
				break;
			}
		}
	}
	else if(hcan == &hcan2){

		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
	
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		
		switch (rx_header.StdId)
		{
			case CAN_M1_ID:
			case CAN_M2_ID:
			case CAN_M3_ID:
			case CAN_M4_ID:
			case CAN_M5_ID:
			case CAN_M6_ID:
			case CAN_M7_ID:
			case CAN_M8_ID:
			case CAN_M9_ID:
			case CAN_MA_ID:
			case CAN_MB_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_M1_ID;
				get_motor_measure(&CAN2_motor_chassis[i], rx_data);
				break;
			}

			default:
			{
				break;
			}
		}
		
	}
}

void CAN_cmd_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = 0x700;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;
    motor_can_send_data[0] = 0;
    motor_can_send_data[1] = 0;
    motor_can_send_data[2] = 0;
    motor_can_send_data[3] = 0;
    motor_can_send_data[4] = 0;
    motor_can_send_data[5] = 0;
    motor_can_send_data[6] = 0;
    motor_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
    HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN1_cmd_Send(int16_t *can1_motor, uint8_t mode)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_M4_ID;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;
    motor_can_send_data[0] = can1_motor[0] >> 8;
    motor_can_send_data[1] = can1_motor[0];
    motor_can_send_data[2] = can1_motor[1] >> 8;
    motor_can_send_data[3] = can1_motor[1];
    motor_can_send_data[4] = can1_motor[2] >> 8;
    motor_can_send_data[5] = can1_motor[2];
    motor_can_send_data[6] = can1_motor[3] >> 8;
    motor_can_send_data[7] = can1_motor[3];
    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);

    motor_tx_message.StdId = CAN_M5_M8_ID;
    motor_can_send_data[0] = can1_motor[4] >> 8;
    motor_can_send_data[1] = can1_motor[4];
    motor_can_send_data[2] = can1_motor[5] >> 8;
    motor_can_send_data[3] = can1_motor[5];
    motor_can_send_data[4] = can1_motor[6] >> 8;
    motor_can_send_data[5] = can1_motor[6];
    motor_can_send_data[6] = can1_motor[7] >> 8;
    motor_can_send_data[7] = can1_motor[7];
    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);

    if (mode)
    {
        motor_tx_message.StdId = CAN_M9_MA_ID;
        motor_can_send_data[0] = can1_motor[8] >> 8;
        motor_can_send_data[1] = can1_motor[8];
        motor_can_send_data[2] = can1_motor[9] >> 8;
        motor_can_send_data[3] = can1_motor[9];
        motor_can_send_data[4] = can1_motor[10] >> 8;
        motor_can_send_data[5] = can1_motor[10];
        motor_can_send_data[6] = 0;
        motor_can_send_data[7] = 0;
        HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
    }
//    HAL_Delay(2);
}


void CAN2_cmd_Send(int16_t *can2_motor, uint8_t mode)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = CAN_M1_M4_ID;
    motor_can_send_data[0] = can2_motor[0] >> 8;
    motor_can_send_data[1] = can2_motor[0];
    motor_can_send_data[2] = can2_motor[1] >> 8;
    motor_can_send_data[3] = can2_motor[1];
    motor_can_send_data[4] = can2_motor[2] >> 8;
    motor_can_send_data[5] = can2_motor[2];
    motor_can_send_data[6] = can2_motor[3] >> 8;
    motor_can_send_data[7] = can2_motor[3];
    HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);

    motor_tx_message.StdId = CAN_M5_M8_ID;
    motor_can_send_data[0] = can2_motor[4] >> 8;
    motor_can_send_data[1] = can2_motor[4];
    motor_can_send_data[2] = can2_motor[5] >> 8;
    motor_can_send_data[3] = can2_motor[5];
    motor_can_send_data[4] = can2_motor[6] >> 8;
    motor_can_send_data[5] = can2_motor[6];
    motor_can_send_data[6] = can2_motor[7] >> 8;
    motor_can_send_data[7] = can2_motor[7];
    HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);

    if (mode)
    {
        motor_tx_message.StdId = CAN_M9_MA_ID;
        motor_can_send_data[0] = can2_motor[8] >> 8;
        motor_can_send_data[1] = can2_motor[8];
        motor_can_send_data[2] = can2_motor[9] >> 8;
        motor_can_send_data[3] = can2_motor[9];
        motor_can_send_data[4] = can2_motor[10] >> 8;
        motor_can_send_data[5] = can2_motor[10];
        motor_can_send_data[6] = 0;
        motor_can_send_data[7] = 0;
        HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);
    }
//    HAL_Delay(2);
}

const motor_measure_t *can1_motor_measure_point(uint8_t i)
{
    return &CAN1_motor_chassis[i % 12];
}

const motor_measure_t *can2_motor_measure_point(uint8_t i)
{
    return &CAN2_motor_chassis[i % 12];
}

const motor_measure_t *can_motor_measure_point(uint8_t n, uint8_t can)
{
    if (can == 1)
        return can1_motor_measure_point(n);
    return can2_motor_measure_point(n);
}

void CAN1_cmd_motor1234(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = 0x200;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = motor1 >> 8;
    motor_can_send_data[1] = motor1;
    motor_can_send_data[2] = motor2 >> 8;
    motor_can_send_data[3] = motor2;
    motor_can_send_data[4] = motor3 >> 8;
    motor_can_send_data[5] = motor3;
    motor_can_send_data[6] = motor4 >> 8;
    motor_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN1_cmd_motor5678(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = 0x1FF;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = motor5 >> 8;
    motor_can_send_data[1] = motor5;
    motor_can_send_data[2] = motor6 >> 8;
    motor_can_send_data[3] = motor6;
    motor_can_send_data[4] = motor7 >> 8;
    motor_can_send_data[5] = motor7;
    motor_can_send_data[6] = motor8 >> 8;
    motor_can_send_data[7] = motor8;

    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN2_cmd_motor1234(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = 0x200;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = motor1 >> 8;
    motor_can_send_data[1] = motor1;
    motor_can_send_data[2] = motor2 >> 8;
    motor_can_send_data[3] = motor2;
    motor_can_send_data[4] = motor3 >> 8;
    motor_can_send_data[5] = motor3;
    motor_can_send_data[6] = motor4 >> 8;
    motor_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

void CAN2_cmd_motor5678(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box;
    motor_tx_message.StdId = 0x1FF;
    motor_tx_message.IDE = CAN_ID_STD;
    motor_tx_message.RTR = CAN_RTR_DATA;
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = motor5 >> 8;
    motor_can_send_data[1] = motor5;
    motor_can_send_data[2] = motor6 >> 8;
    motor_can_send_data[3] = motor6;
    motor_can_send_data[4] = motor7 >> 8;
    motor_can_send_data[5] = motor7;
    motor_can_send_data[6] = motor8 >> 8;
    motor_can_send_data[7] = motor8;

    HAL_CAN_AddTxMessage(&hcan2, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

// CAN1 电机1 (0x201)
const motor_measure_t *get_3508_motor1_measure_point(void)
{
    return &CAN1_motor_chassis[0];
}

// CAN1 电机2 (0x202)
const motor_measure_t *get_3508_motor2_measure_point(void)
{
    return &CAN1_motor_chassis[1];
}

// CAN1 电机3 (0x203)
const motor_measure_t *get_3508_motor3_measure_point(void)
{
    return &CAN1_motor_chassis[2];
}

// CAN1 电机4 (0x204)
const motor_measure_t *get_3508_motor4_measure_point(void)
{
    return &CAN1_motor_chassis[3];
}

// CAN1 电机5 (0x205)
const motor_measure_t *get_3508_motor5_measure_point(void)
{
    return &CAN1_motor_chassis[4];
}

// CAN1 电机6 (0x206)
const motor_measure_t *get_3508_motor6_measure_point(void)
{
    return &CAN1_motor_chassis[5];
}

// CAN1 电机7 (0x207)
const motor_measure_t *get_3508_motor7_measure_point(void)
{
    return &CAN1_motor_chassis[6];
}

// CAN1 电机8 (0x208)
const motor_measure_t *get_3508_motor8_measure_point(void)
{
    return &CAN1_motor_chassis[7];
}

// CAN2 电机1 (0x201)
const motor_measure_t *get_CAN2_motor1_measure_point(void)
{
    return &CAN2_motor_chassis[0];
}

// CAN2 电机2 (0x202)
const motor_measure_t *get_CAN2_motor2_measure_point(void)
{
    return &CAN2_motor_chassis[1];
}

// CAN2 电机3 (0x203)
const motor_measure_t *get_CAN2_motor3_measure_point(void)
{
    return &CAN2_motor_chassis[2];
}

// CAN2 电机4 (0x204)
const motor_measure_t *get_CAN2_motor4_measure_point(void)
{
    return &CAN2_motor_chassis[3];
}

// CAN2 电机5 (0x205)
const motor_measure_t *get_CAN2_motor5_measure_point(void)
{
    return &CAN2_motor_chassis[4];
}

// CAN2 电机6 (0x206)
const motor_measure_t *get_CAN2_motor6_measure_point(void)
{
    return &CAN2_motor_chassis[5];
}

// CAN2 电机7 (0x207)
const motor_measure_t *get_CAN2_motor7_measure_point(void)
{
    return &CAN2_motor_chassis[6];
}

// CAN2 电机8 (0x208)
const motor_measure_t *get_CAN2_motor8_measure_point(void)
{
    return &CAN2_motor_chassis[7];
}


