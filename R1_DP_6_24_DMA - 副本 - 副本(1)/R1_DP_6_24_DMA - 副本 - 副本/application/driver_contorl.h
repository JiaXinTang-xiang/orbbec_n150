#ifndef DRIVER_CONTROL_H  // 防止头文件重复包含（头文件保护）
#define DRIVER_CONTROL_H

#include "main.h"

typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_COMPLETE
} MotorControlState;

void driver_contorl_m3508_quansu(void); 
void driver_contorl_m3508_jiaodu(void);
void driver_contorl_pid(void);
void driver_contorl_shudu(void);
void driver_contorl_JD(void);
void driver_contorl_QS(void);



void angle_driver_nb_set(uint8_t motor_id, float target_angle);
void quansu_driver_nb_set(uint8_t motor_id, uint32_t target_rotations);
MotorControlState motor_get_state(uint8_t motor_id);
void motor_stop(uint8_t motor_id);
void motor_reset_position(uint8_t motor_id);
void motor_control_process(void);
void motor_control_interrupt_handler(void);

void motor_task_queue_clear(uint8_t motor_id);
uint8_t motor_task_queue_add_position(uint8_t motor_id, float angle);
void motor_task_queue_start(uint8_t motor_id);
uint8_t motor_task_queue_is_running(uint8_t motor_id);
void motor_task_queue_update(uint8_t motor_id);
void angle_limt(void);
	
void driver_task(void);
void driver_task_mode(void);

#endif