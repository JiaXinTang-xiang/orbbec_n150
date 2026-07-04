#include "chassis_task.h"
#include "NRF24L01.h"
#include "JIDIANQI.h"
#include "driver_contorl.h"
#include "usb_cdc.h"

float angle_5;
float angle_1;
float angle_7;
float angle_8;

extern volatile uint32_t driver_delay_tick;

chassis_D_init chassis_motor_D_init[Chassis_motor_N]={
	{{1,5},{1,1},{1.0f,0.0f},900},{{2,6},{1,1},{-1.0f/2.0f,-1.732f/2.0f},2700},
	{{3,7},{1,1},{-1.0f/2.0f,1.732f/2.0f},0},
};
chassis_task_D chassis_control_D;
int32_t x,y,w;

void chassis_task_D_init(void){
	motor_can_init();
	remote_behaviou_init(&chassis_control_D.chassis_remote_recv);
	for(uint8_t i=0;i<Chassis_motor_N; i++){
		motor_can_speed(chassis_motor_D_init[i].id[0] ,chassis_motor_D_init[i].can[0] ,&chassis_control_D.chassis[i].speed);
		motor_can_speed(chassis_motor_D_init[i].id[1] ,chassis_motor_D_init[i].can[1] ,&chassis_control_D.chassis[i].seat);
		chassis_control_D.chassis[i].x_y = chassis_motor_D_init[i].x_y;
		chassis_control_D.chassis[i].original_seat = &chassis_motor_D_init[i].original_seat;
		chassis_control_D.chassis[i].motor_ecd = motor_can_return_ecd(chassis_motor_D_init[i].id[1] ,chassis_motor_D_init[i].can[1]);
		chassis_control_D.chassis[i].motor_speed_rpm = motor_can_return_speed(chassis_motor_D_init[i].id[1] ,chassis_motor_D_init[i].can[1]);
	}
}

uint8_t speed_bl=0;fp32 speed_Q,speed_Q1;
void chassis_D_task(void){
	/* USB CDC 优先 */
	if (usb_chassis_active) {
		chassis_control_D.chassis_remote_recv.set_x = (int32_t)usb_chassis_vx;
		chassis_control_D.chassis_remote_recv.set_y = (int32_t)usb_chassis_vy;
		chassis_control_D.chassis_remote_recv.set_w = (int32_t)usb_chassis_vw;
	} else {
		remote_behaviou_set(&chassis_control_D.chassis_remote_recv);
	}
	chassisMath_D_chassis(chassis_control_D.chassis ,&chassis_control_D.chassis_remote_recv);
	motor_can_pid();
}

static void chassisMath_D_chassis_angle(chassis_D_wheel *motor);
static void chassis_math_D_angle_mapping(chassis_D_wheel *chassis);

void chassisMath_D_chassis(chassis_D_wheel *motor,remote_rc *remote){
	for(uint8_t i =0 ; i<Chassis_motor_N; ++i){
		motor[i].V_Y = ((fp32)y+(fp32)w*motor[i].x_y[1])/100.0f;
		motor[i].V_X = ((fp32)x+(fp32)w*motor[i].x_y[0])/100.0f;
		arm_sqrt_f32(motor[i].V_Y *motor[i].V_Y  + motor[i].V_X*motor[i].V_X,&motor[i].speed_in);
		motor[i].angle = acos(motor[i].V_X/motor[i].speed_in);
		if(motor[i].V_Y<0)motor[i].angle=pi2-motor[i].angle;
		else if(motor[i].V_Y==0&&motor[i].V_X==0) motor[i].angle=motor[i].angle_last;
		chassis_math_D_angle_mapping(&motor[i]);
	}
}

static fp32 chassis_math_D_angle_set_ref(fp32 ref,fp32 set){
	if(set>ref) return set-ref<=ref+pi2-set ? set-ref : ref+pi2-set;
	return ref-set<=set+pi2-ref ? ref-set : set+pi2-ref;
}

static void chassis_math_D_angle_mapping(chassis_D_wheel *chassis){
	chassis->angle_last = *chassis->motor_ecd - *chassis->original_seat;
	if(chassis->angle_last<0) chassis->angle_last += 8191;
	chassis->angle_last *=(pi2/8191);
	fp32 angle_set_ref = chassis_math_D_angle_set_ref(chassis->angle,chassis->angle_last);
	if(angle_set_ref>pi/2 ){
		chassis->angle -= pi;
		chassis->angle = chassis->angle <0.0 ? 2*pi+chassis->angle : chassis->angle;
		chassis->speed = -chassis->speed_in*19/0.061;
	}else{
		if(angle_set_ref>=pi/6)chassis->speed=0;
		else chassis->speed = chassis->speed_in*19/0.061;
	}
	chassis->seat = chassis->angle*8191/(2*pi) + *chassis->original_seat;
	if(chassis->seat>8191) chassis->seat-=8191;
}

uint16_t tim1_count;
uint16_t chass_flage=0;
uint16_t driver_flage=0;

uint8_t speed_bl;
int32_t k=10,speed_max_up=10,speed_max_zero=40;

void slope(int32_t *ret,int32_t set){
	if(set==0&&(*ret>0-speed_max_zero&&*ret<0+speed_max_zero)){ *ret=0; return; }
	if(set!=0&&(*ret>set-speed_max_zero&&*ret<set+speed_max_zero)){ *ret=set; return; }
	if( *ret>set&&set>=0) *ret-=speed_max_zero;
	else if( *ret<set&&set<=0) *ret+=speed_max_zero;
	if(*ret<set&&set>=0)  *ret+=speed_max_up;
	else if(*ret>set&&set<=0)  *ret-=speed_max_up;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim1)
    {
		NRF24L01_Task();
	   /* USB CDC 继电器 */
		usb_cdc_relay_task();
	   usb_recet_flage();
		relay_task();
		relay_task_mode();

		tim1_count++;
		chass_flage++;
		if(tim1_count==10){
			slope(&x,chassis_control_D.chassis_remote_recv.set_x);
			slope(&y,chassis_control_D.chassis_remote_recv.set_y);
			slope(&w,chassis_control_D.chassis_remote_recv.set_w);
			tim1_count=0;
		}
		if(chass_flage==5){
		   chassis_D_task();
			usb_cdc_tick();       /* USB CDC 反馈 (5ms) */
			chass_flage=0;
		}
		
//		 driver_flage++;
//		if(driver_flage>=1)
//		{
//			driver_task();
//			driver_task_mode();
//			driver_flage=0;
//		
//		}
//		
//		angle_limt();
//        driver_delay_tick++;
//        motor_control_interrupt_handler();
    }
}
