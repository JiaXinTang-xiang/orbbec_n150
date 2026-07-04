#include "driver_contorl.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pidd.h"
#include "stdio.h"
#include "main.h"
#include "tim.h"
#include "chassis_task.h"
#include "NRF24L01.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define MOTOR_CONTROL_PERIOD_MS 2
#define ANGLE_TO_POSITION_SCALE 1603.2f
#define FULL_ROTATION_POSITION 577152.0f
#define MAX_TASK_QUEUE_SIZE 10
#define POSITION_TOLERANCE 50.0f

volatile uint32_t driver_delay_tick = 0;

extern float angle_5;
extern float angle_1;
extern float angle_7;
extern float angle_8;

extern RemoteControlData_t rc_data;

typedef struct {
    MotorControlState state;
    float target_pos;
    float current_pos;
    float vel_set;
    int16_t iref;
} MotorController;

typedef struct {
    float positions[MAX_TASK_QUEUE_SIZE];
    uint8_t count;
    uint8_t current_index;
    uint8_t running;
} MotorTaskQueue;

MotorController motor_ctrl[8] = {0};
MotorTaskQueue motor_task_queue[8] = {0};

pid_t driver_pid3508v_1,driver_pid3508pos_1;
pid_t driver_pid3508v_2,driver_pid3508pos_2;
pid_t driver_pid3508v_3,driver_pid3508pos_3;
pid_t driver_pid3508v_4,driver_pid3508pos_4;
pid_t driver_pid3508v_5,driver_pid3508pos_5;
pid_t driver_pid3508v_6,driver_pid3508pos_6;
pid_t driver_pid3508v_7,driver_pid3508pos_7;
pid_t driver_pid3508v_8,driver_pid3508pos_8;

const motor_measure_t *driver_m3508_1;
const motor_measure_t *driver_m3508_2;
const motor_measure_t *driver_m3508_3;
const motor_measure_t *driver_m3508_4;
const motor_measure_t *driver_m3508_5;
const motor_measure_t *driver_m3508_6;
const motor_measure_t *driver_m3508_7;
const motor_measure_t *driver_m3508_8;

int16_t driver_iref_inner_1,driver_iref_inner_2;
int16_t driver_iref_inner_3,driver_iref_inner_4;
int16_t driver_iref_inner_5,driver_iref_inner_6;
int16_t driver_iref_inner_7,driver_iref_inner_8;

float driver_pos_temp_1,driver_pos_set_1=0,driver_vel_set_1=0;
float driver_pos_temp_2,driver_pos_set_2=0,driver_vel_set_2=0;
float driver_pos_temp_3,driver_pos_set_3=0,driver_vel_set_3=0;
float driver_pos_temp_4,driver_pos_set_4=0,driver_vel_set_4=0;
float driver_pos_temp_5,driver_pos_set_5=0,driver_vel_set_5=0;
float driver_pos_temp_6,driver_pos_set_6=0,driver_vel_set_6=0;
float driver_pos_temp_7,driver_pos_set_7=0,driver_vel_set_7=0;
float driver_pos_temp_8,driver_pos_set_8=0,driver_vel_set_8=0;

float driver_angle_1 = 0;
float driver_angle_2 = 0;
float driver_angle_3 = 0;
float driver_angle_4 = 0;
float driver_angle_5 = 0;
float driver_angle_6 = 0;
float driver_angle_7 = 0;
float driver_angle_8 = 0;

float driver_quansu_1 = 0;
float driver_quansu_2 = 0;
float driver_quansu_3 = 0;
float driver_quansu_4 = 0;
float driver_quansu_5 = 0;
float driver_quansu_6 = 0;
float driver_quansu_7 = 0;
float driver_quansu_8 = 0;

fp32 driver_ecd_sum_1,driver_ecd_sum_2,driver_ecd_sum_3,driver_ecd_sum_4;
fp32 driver_ecd_sum_5,driver_ecd_sum_6,driver_ecd_sum_7,driver_ecd_sum_8;

float driver_temp = 0;
int ecd_value;
int angle;
static uint32_t jiaodu_delay_tick = 0;

int driver_map(int x, int in_min, int in_max, int out_min, int out_max)
{
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float driver_ABS(float X)
{
	return driver_temp = X > 0 ? X : -X;
}

float driver_fabs_simple(float value) {
    return (value < 0) ? -value : value;
}

fp32 driver_limit_value(fp32 value, fp32 min, fp32 max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

fp32 driver_pos_set(fp32 pos_set,fp32 max,fp32 min )
{
  if(pos_set > max)
  {
  return 0;
  }
 else if(pos_set < min)
  {
  return 0;
  }
  else{
  return pos_set;
    }
}

void driver_update_ecd_sum(const motor_measure_t *motor_data, fp32 *ecd_sum)
{
    if (driver_ABS(motor_data->ecd - motor_data->last_ecd) > 4095)
		{
        if (motor_data->ecd < motor_data->last_ecd)
				{
            *ecd_sum += motor_data->ecd + (8191 - motor_data->last_ecd);
				} else
				{
            *ecd_sum -= 8191-motor_data->ecd + motor_data->last_ecd;
        }
      } else
		{
        *ecd_sum += motor_data->ecd - motor_data->last_ecd;
        }
    *ecd_sum = driver_limit_value(*ecd_sum, -38912, 38912);
}

void driver_contorl_pid(void)
{
	  driver_m3508_1 = get_CAN2_motor1_measure_point();
      PID_struct_init(&driver_pid3508v_1 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_1 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);

      driver_m3508_2 = get_CAN2_motor2_measure_point();
	  PID_struct_init(&driver_pid3508v_2 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_2 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);

	  driver_m3508_3 = get_CAN2_motor3_measure_point();
	  PID_struct_init(&driver_pid3508v_3 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_3 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);

	  driver_m3508_4 = get_CAN2_motor4_measure_point();
	  PID_struct_init(&driver_pid3508v_4 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_4 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);

	  driver_m3508_5 = get_CAN2_motor5_measure_point();
      PID_struct_init(&driver_pid3508v_5 ,POSITION_PID,10000.f,2000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_5 ,POSITION_PID,10000.f,10000.f,0.05f,0,0.0015f);

      driver_m3508_6 = get_CAN2_motor6_measure_point();
	  PID_struct_init(&driver_pid3508v_6 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_6 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);

	  driver_m3508_7 = get_CAN2_motor7_measure_point();
	  PID_struct_init(&driver_pid3508v_7 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_7 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.0015f);

	  driver_m3508_8 = get_CAN2_motor8_measure_point();
	  PID_struct_init(&driver_pid3508v_8 ,POSITION_PID,10000.f,10000.f,11.f,0.2f,0.0f);
	  PID_struct_init(&driver_pid3508pos_8 ,POSITION_PID,10000.f,10000.f,0.1f,0,0.003f);
}

void driver_contorl_shudu(void)
{
	driver_iref_inner_1=pid_calc(&driver_pid3508v_1,driver_m3508_1->speed_rpm,driver_vel_set_1);

	driver_iref_inner_2=pid_calc(&driver_pid3508v_2,driver_m3508_2->speed_rpm,driver_vel_set_2);

	driver_iref_inner_3=pid_calc(&driver_pid3508v_3,driver_m3508_3->speed_rpm,driver_vel_set_3);

	driver_iref_inner_4=pid_calc(&driver_pid3508v_4,driver_m3508_4->speed_rpm,driver_vel_set_4);

    driver_iref_inner_5=pid_calc(&driver_pid3508v_5,driver_m3508_5->speed_rpm,driver_vel_set_5);

	driver_iref_inner_6=pid_calc(&driver_pid3508v_6,driver_m3508_6->speed_rpm,driver_vel_set_6);

	driver_iref_inner_7=pid_calc(&driver_pid3508v_7,driver_m3508_7->speed_rpm,driver_vel_set_7);

	driver_iref_inner_8=pid_calc(&driver_pid3508v_8,driver_m3508_8->speed_rpm,driver_vel_set_8);

    CAN2_cmd_motor1234(driver_iref_inner_1, driver_iref_inner_2, driver_iref_inner_3, driver_iref_inner_4);
	  CAN2_cmd_motor5678(driver_iref_inner_5, driver_iref_inner_6, driver_iref_inner_7,driver_iref_inner_8);
}

void driver_contorl_JD(void)
{
	     driver_contorl_pid();
		 driver_pos_set_1 += driver_angle_1*50.0f*19.2f*1.1115f;
		 driver_pos_set_2 += driver_angle_2*50.0f*19.2f*1.1115f;
		 driver_pos_set_3 += driver_angle_3*50.0f*19.2f*1.1115f;
		 driver_pos_set_4 += driver_angle_4*50.0f*19.2f*1.1115f;
		 driver_pos_set_5 += driver_angle_5*50.0f*19.2f*1.1115f;
		 driver_pos_set_6 += driver_angle_6*50.0f*19.2f*1.1115f;
		 driver_pos_set_7 += driver_angle_7*50.0f*19.2f*1.1115f;
		 driver_pos_set_8 += driver_angle_8*50.0f*19.2f*1.1115f;
}

void driver_contorl_QS(void)
{
	  driver_contorl_pid();
	  driver_pos_set_1 += driver_quansu_1*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_2 += driver_quansu_2*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_3 += driver_quansu_3*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_4 += driver_quansu_4*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_5 += driver_quansu_5*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_6 += driver_quansu_6*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_7 += driver_quansu_7*360*50.0f*19.2f*1.1115f;
	  driver_pos_set_8 += driver_quansu_8*360*50.0f*19.2f*1.1115f;
}

void driver_contorl_m3508_quansu(void)
{
			if (driver_delay_tick - jiaodu_delay_tick < 2) return;
			jiaodu_delay_tick = driver_delay_tick;

        driver_pos_temp_1+=driver_m3508_1->speed_rpm;
			driver_vel_set_1=pid_calc(&driver_pid3508pos_1,driver_pos_temp_1,driver_pos_set_1);
			driver_vel_set_1 = CLAMP(driver_vel_set_1, -4000.0f, 4000.0f);
			driver_iref_inner_1=pid_calc(&driver_pid3508v_1,driver_m3508_1->speed_rpm,driver_vel_set_1);

			driver_pos_temp_2+=driver_m3508_2->speed_rpm;
			driver_vel_set_2=pid_calc(&driver_pid3508pos_2,driver_pos_temp_2,driver_pos_set_2);
			driver_vel_set_2 = CLAMP(driver_vel_set_2, -4000.0f, 4000.0f);
			driver_iref_inner_2=pid_calc(&driver_pid3508v_2,driver_m3508_2->speed_rpm,driver_vel_set_2);

			driver_pos_temp_3+=driver_m3508_3->speed_rpm;
			driver_vel_set_3=pid_calc(&driver_pid3508pos_3,driver_pos_temp_3,driver_pos_set_3);
			driver_vel_set_3 = CLAMP(driver_vel_set_3, -5000.0f, 5000.0f);
			driver_iref_inner_3=pid_calc(&driver_pid3508v_3,driver_m3508_3->speed_rpm,driver_vel_set_3);

			driver_pos_temp_4+=driver_m3508_4->speed_rpm;
			driver_vel_set_4=pid_calc(&driver_pid3508pos_4,driver_pos_temp_4,driver_pos_set_4);
			driver_vel_set_4 = CLAMP(driver_vel_set_4, -4000.0f, 4000.0f);
			driver_iref_inner_4=pid_calc(&driver_pid3508v_4,driver_m3508_4->speed_rpm,driver_vel_set_4);

			driver_pos_temp_5+=driver_m3508_5->speed_rpm;
			driver_vel_set_5=pid_calc(&driver_pid3508pos_5,driver_pos_temp_5,driver_pos_set_5);
			driver_vel_set_5 = CLAMP(driver_vel_set_5, -4000.0f, 4000.0f);
			driver_iref_inner_5=pid_calc(&driver_pid3508v_5,driver_m3508_5->speed_rpm,driver_vel_set_5);

			driver_pos_temp_6=driver_m3508_6->speed_rpm;
			driver_vel_set_6=pid_calc(&driver_pid3508pos_6,driver_pos_temp_6,driver_pos_set_6);
			driver_vel_set_6 = CLAMP(driver_vel_set_6, -4000.0f, 4000.0f);
			driver_iref_inner_6=pid_calc(&driver_pid3508v_6,driver_m3508_6->speed_rpm,driver_vel_set_6);

			driver_pos_temp_7+=driver_m3508_7->speed_rpm;
			driver_vel_set_7=pid_calc(&driver_pid3508pos_7,driver_pos_temp_7,driver_pos_set_7);
			driver_vel_set_7 = CLAMP(driver_vel_set_7, -5000.0f, 5000.0f);
			driver_iref_inner_7=pid_calc(&driver_pid3508v_7,driver_m3508_7->speed_rpm,driver_vel_set_7);

			driver_pos_temp_8+=driver_m3508_8->speed_rpm;
			driver_vel_set_8=pid_calc(&driver_pid3508pos_8,driver_pos_temp_8,driver_pos_set_8);
			driver_vel_set_8 = CLAMP(driver_vel_set_8, -4000.0f, 4000.0f);
			driver_iref_inner_8=pid_calc(&driver_pid3508v_8,driver_m3508_8->speed_rpm,driver_vel_set_8);

		   CAN2_cmd_motor1234(driver_iref_inner_1, driver_iref_inner_2, driver_iref_inner_3, driver_iref_inner_4);
		   CAN2_cmd_motor5678(driver_iref_inner_5, driver_iref_inner_6, driver_iref_inner_7,driver_iref_inner_8);
}

void driver_contorl_m3508_jiaodu(void)
{
			if (driver_delay_tick - jiaodu_delay_tick < 2) return;
			jiaodu_delay_tick = driver_delay_tick;

        driver_pos_temp_1+=driver_m3508_1->speed_rpm;
			driver_vel_set_1=pid_calc(&driver_pid3508pos_1,driver_pos_temp_1,driver_pos_set_1);
			driver_vel_set_1 = CLAMP(driver_vel_set_1, -4000.0f, 4000.0f);
			driver_iref_inner_1=pid_calc(&driver_pid3508v_1,driver_m3508_1->speed_rpm,driver_vel_set_1);

			driver_pos_temp_2+=driver_m3508_2->speed_rpm;
			driver_vel_set_2=pid_calc(&driver_pid3508pos_2,driver_pos_temp_2,driver_pos_set_2);
			driver_vel_set_2 = CLAMP(driver_vel_set_2, -4000.0f, 4000.0f);
			driver_iref_inner_2=pid_calc(&driver_pid3508v_2,driver_m3508_2->speed_rpm,driver_vel_set_2);

			driver_pos_temp_3+=driver_m3508_3->speed_rpm;
			driver_vel_set_3=pid_calc(&driver_pid3508pos_3,driver_pos_temp_3,driver_pos_set_3);
			driver_vel_set_3 = CLAMP(driver_vel_set_3, -5000.0f, 5000.0f);
			driver_iref_inner_3=pid_calc(&driver_pid3508v_3,driver_m3508_3->speed_rpm,driver_vel_set_3);

			driver_pos_temp_4+=driver_m3508_4->speed_rpm;
			driver_vel_set_4=pid_calc(&driver_pid3508pos_4,driver_pos_temp_4,driver_pos_set_4);
			driver_vel_set_4 = CLAMP(driver_vel_set_4, -4000.0f, 4000.0f);
			driver_iref_inner_4=pid_calc(&driver_pid3508v_4,driver_m3508_4->speed_rpm,driver_vel_set_4);

			driver_pos_temp_5+=driver_m3508_5->speed_rpm;
			driver_vel_set_5=pid_calc(&driver_pid3508pos_5,driver_pos_temp_5,driver_pos_set_5);
			driver_vel_set_5 = CLAMP(driver_vel_set_5, -4000.0f, 4000.0f);
			driver_iref_inner_5=pid_calc(&driver_pid3508v_5,driver_m3508_5->speed_rpm,driver_vel_set_5);

			driver_pos_temp_6=driver_m3508_6->speed_rpm;
			driver_vel_set_6=pid_calc(&driver_pid3508pos_6,driver_pos_temp_6,driver_pos_set_6);
			driver_vel_set_6 = CLAMP(driver_vel_set_6, -4000.0f, 4000.0f);
			driver_iref_inner_6=pid_calc(&driver_pid3508v_6,driver_m3508_6->speed_rpm,driver_vel_set_6);

			driver_pos_temp_7+=driver_m3508_7->speed_rpm;
			driver_vel_set_7=pid_calc(&driver_pid3508pos_7,driver_pos_temp_7,driver_pos_set_7);
			driver_vel_set_7 = CLAMP(driver_vel_set_7, -5000.0f, 5000.0f);
			driver_iref_inner_7=pid_calc(&driver_pid3508v_7,driver_m3508_7->speed_rpm,driver_vel_set_7);

			driver_pos_temp_8+=driver_m3508_8->speed_rpm;
			driver_vel_set_8=pid_calc(&driver_pid3508pos_8,driver_pos_temp_8,driver_pos_set_8);
			driver_vel_set_8 = CLAMP(driver_vel_set_8, -4000.0f, 4000.0f);
			driver_iref_inner_8=pid_calc(&driver_pid3508v_8,driver_m3508_8->speed_rpm,driver_vel_set_8);

		   CAN2_cmd_motor1234(driver_iref_inner_1, driver_iref_inner_2, driver_iref_inner_3, driver_iref_inner_4);
		   CAN2_cmd_motor5678(driver_iref_inner_5, driver_iref_inner_6, driver_iref_inner_7,driver_iref_inner_8);
}



void angle_driver_nb_set(uint8_t motor_id, float target_angle)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;

    motor_ctrl[idx].target_pos = (float)target_angle * ANGLE_TO_POSITION_SCALE;
    // 去掉 current_pos=0 重置，使函数使用上电零点作为绝对参考系
    // 到达目标后 PID 误差归零，电机自动停止维持位置
    motor_ctrl[idx].state = MOTOR_STATE_RUNNING;
}

void motor_reset_position(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;
    
    motor_ctrl[idx].current_pos = 0;
    motor_ctrl[idx].target_pos = 0;
    motor_ctrl[idx].state = MOTOR_STATE_IDLE;
}

void quansu_driver_nb_set(uint8_t motor_id, uint32_t target_rotations)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;
    motor_ctrl[idx].target_pos = target_rotations * FULL_ROTATION_POSITION;
    motor_ctrl[idx].current_pos = 0;
    motor_ctrl[idx].state = MOTOR_STATE_RUNNING;
}

MotorControlState motor_get_state(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return MOTOR_STATE_IDLE;
    return motor_ctrl[motor_id - 1].state;
}

void motor_stop(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return;
    motor_ctrl[motor_id - 1].state = MOTOR_STATE_IDLE;
    motor_ctrl[motor_id - 1].iref = 0;
}

static uint16_t motor_control_counter = 0;

void motor_control_interrupt_handler(void)
{
    motor_control_counter++;
    
    if (motor_control_counter < MOTOR_CONTROL_PERIOD_MS) {
        return;
    }
    motor_control_counter = 0;

    const motor_measure_t *motor_data[8];
    motor_data[0] = get_CAN2_motor1_measure_point();
    motor_data[1] = get_CAN2_motor2_measure_point();
    motor_data[2] = get_CAN2_motor3_measure_point();
    motor_data[3] = get_CAN2_motor4_measure_point();
    motor_data[4] = get_CAN2_motor5_measure_point();
    motor_data[5] = get_CAN2_motor6_measure_point();
    motor_data[6] = get_CAN2_motor7_measure_point();
    motor_data[7] = get_CAN2_motor8_measure_point();

    pid_t *pid_pos[8];
    pid_pos[0] = &driver_pid3508pos_1; pid_pos[1] = &driver_pid3508pos_2;
    pid_pos[2] = &driver_pid3508pos_3; pid_pos[3] = &driver_pid3508pos_4;
    pid_pos[4] = &driver_pid3508pos_5; pid_pos[5] = &driver_pid3508pos_6;
    pid_pos[6] = &driver_pid3508pos_7; pid_pos[7] = &driver_pid3508pos_8;

    pid_t *pid_vel[8];
    pid_vel[0] = &driver_pid3508v_1; pid_vel[1] = &driver_pid3508v_2;
    pid_vel[2] = &driver_pid3508v_3; pid_vel[3] = &driver_pid3508v_4;
    pid_vel[4] = &driver_pid3508v_5; pid_vel[5] = &driver_pid3508v_6;
    pid_vel[6] = &driver_pid3508v_7; pid_vel[7] = &driver_pid3508v_8;

    int16_t iref_values[8] = {0};

    for (int i = 0; i < 8; i++) {
        if (motor_ctrl[i].state != MOTOR_STATE_RUNNING) {
            iref_values[i] = 0;
            continue;
        }

        motor_ctrl[i].current_pos += motor_data[i]->speed_rpm;

        motor_ctrl[i].vel_set = pid_calc(pid_pos[i], motor_ctrl[i].current_pos, motor_ctrl[i].target_pos);
        motor_ctrl[i].vel_set = CLAMP(motor_ctrl[i].vel_set, -2500.0f, 2500.0f);

        motor_ctrl[i].iref = pid_calc(pid_vel[i], motor_data[i]->speed_rpm, motor_ctrl[i].vel_set);
        iref_values[i] = motor_ctrl[i].iref;
        
        float pos_diff = driver_fabs_simple(motor_ctrl[i].current_pos - motor_ctrl[i].target_pos);
        if (pos_diff < POSITION_TOLERANCE && driver_fabs_simple(motor_data[i]->speed_rpm) < 50.0f) {
            motor_ctrl[i].state = MOTOR_STATE_IDLE;
            motor_ctrl[i].iref = 0;
            motor_ctrl[i].vel_set = 0;
            iref_values[i] = 0;
        }
        
        motor_task_queue_update(i + 1);
    }

    CAN2_cmd_motor1234(iref_values[0]/2 , iref_values[1], iref_values[2], iref_values[3]);
    CAN2_cmd_motor5678(iref_values[4]/5 , iref_values[5], iref_values[6]/3, iref_values[7]);
}

void motor_control_process(void)
{
    motor_control_interrupt_handler();
}

void motor_task_queue_clear(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;
    motor_task_queue[idx].count = 0;
    motor_task_queue[idx].current_index = 0;
    motor_task_queue[idx].running = 0;
}

uint8_t motor_task_queue_add_position(uint8_t motor_id, float angle)
{
    if (motor_id < 1 || motor_id > 8) return 0;
    uint8_t idx = motor_id - 1;
    
    if (motor_task_queue[idx].count >= MAX_TASK_QUEUE_SIZE) {
        return 0;
    }
    
    motor_task_queue[idx].positions[motor_task_queue[idx].count++] = angle * ANGLE_TO_POSITION_SCALE;
    return 1;
}

void motor_task_queue_start(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;
    
    if (motor_task_queue[idx].count == 0) return;
    
    motor_task_queue[idx].current_index = 0;
    motor_task_queue[idx].running = 1;
    
    motor_ctrl[idx].target_pos = motor_task_queue[idx].positions[0];
    motor_ctrl[idx].current_pos = 0;
    motor_ctrl[idx].state = MOTOR_STATE_RUNNING;
}

uint8_t motor_task_queue_is_running(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return 0;
    return motor_task_queue[motor_id - 1].running;
}

void motor_task_queue_update(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 8) return;
    uint8_t idx = motor_id - 1;
    
    if (!motor_task_queue[idx].running) return;
    
    if (motor_ctrl[idx].state != MOTOR_STATE_RUNNING) return;
    
    float pos_diff = driver_fabs_simple(motor_ctrl[idx].current_pos - motor_ctrl[idx].target_pos);
    
    if (pos_diff < POSITION_TOLERANCE) {
        motor_task_queue[idx].current_index++;
        
        if (motor_task_queue[idx].current_index >= motor_task_queue[idx].count) {
            motor_task_queue[idx].running = 0;
            motor_ctrl[idx].state = MOTOR_STATE_IDLE;
            motor_ctrl[idx].iref = 0;
        } else {
            motor_ctrl[idx].target_pos = motor_task_queue[idx].positions[motor_task_queue[idx].current_index];
            motor_ctrl[idx].current_pos = 0;
        }
    }
}

void angle_limt(void)
{

		if(angle_5>=280)       { angle_5=280;   }
		else if(angle_5<=-280) { angle_5= -280; }
		
		if(angle_1>=100)       { angle_1=100;   }
		else if(angle_1<=0)    { angle_1= 0 ;   }
		
		if(angle_7>=1800)      { angle_7=1800;  }
		else if(angle_7<=0)    { angle_7= 0 ;   }
		
		if(angle_8>=720)       { angle_8=720;   }
		else if(angle_8<=0)    { angle_8= 0 ;   }
		
}

void driver_task(void)
{
	if(rc_data.key_bg_left == 2 && rc_data.key_bg_right == 3)
	{
		/* 与 remote_behaviou_set 对齐: 中心140, 死区±20, 范围25~255 */
		int16_t raw1 = (int16_t)rc_data.adc_ch1 - 140;
		int16_t raw2 = (int16_t)rc_data.adc_ch2 - 140;
		int16_t raw3 = (int16_t)rc_data.adc_ch3 - 140;
		int16_t raw4 = (int16_t)rc_data.adc_ch4 - 140;

		/* ── 通道1: angle_5  ── */
		if      (raw1 >= 75)  angle_5 += 0.2f;
		else if (raw1 >= 45)  angle_5 += 0.15f;
		else if (raw1 >= 20)  angle_5 += 0.1f;
		else if (raw1 <= -75) angle_5 -= 0.2f;
		else if (raw1 <= -45) angle_5 -= 0.15f;
		else if (raw1 <= -20) angle_5 -= 0.1f;

		/* ── 通道2: angle_1  ── */
		if      (raw2 >= 75)  angle_1 += 0.2f;
		else if (raw2 >= 45)  angle_1 += 0.15f;
		else if (raw2 >= 20)  angle_1 += 0.1f;
		else if (raw2 <= -75) angle_1 -= 0.2f;
		else if (raw2 <= -45) angle_1 -= 0.15f;
		else if (raw2 <= -20) angle_1 -= 0.1f;

		/* ── 通道3: angle_7  ── */
		if      (raw3 >= 75)  angle_7 += 0.1f;
		else if (raw3 >= 45)  angle_7 += 0.05f;
		else if (raw3 >= 20)  angle_7 += 0.02f;
		else if (raw3 <= -75) angle_7 -= 0.1f;
		else if (raw3 <= -45) angle_7 -= 0.05f;
		else if (raw3 <= -20) angle_7 -= 0.02f;

		/* ── 通道4: angle_8  ── */
		if      (raw4 >= 75)  angle_8 += 1.5f;
		else if (raw4 >= 45)  angle_8 += 1.0f;
		else if (raw4 >= 20)  angle_8 += 0.5f;
		else if (raw4 <= -75) angle_8 -= 1.5f;
		else if (raw4 <= -45) angle_8 -= 1.0f;
		else if (raw4 <= -20) angle_8 -= 0.5f;

		/* 角度归零 (按键1-4) */
		if(rc_data.key_number == 1)  angle_1 = 0;
		if(rc_data.key_number == 2)  angle_5 = 0;
		if(rc_data.key_number == 3)  angle_7 = 0;
		if(rc_data.key_number == 4)  angle_8 = 0;
	}

	angle_driver_nb_set(1, angle_1);
	angle_driver_nb_set(5, angle_5);
	angle_driver_nb_set(7, angle_7);
	angle_driver_nb_set(8, angle_8);
}


void driver_task_mode(void)
{
	if(rc_data.key_bg_left == 2 && rc_data.key_bg_right == 4)
	{
		
	
	
	
	
	}

}
