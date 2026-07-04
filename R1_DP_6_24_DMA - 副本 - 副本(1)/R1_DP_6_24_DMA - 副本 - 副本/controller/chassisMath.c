#include "chassisMath.h"

static void chassisMath_D_chassis_angle(chassis_D_wheel *motor);

void chassisMath_D_chassis(chassis_D_wheel *motor , remote_rc *remote){
	for(uint8_t i=0;i<Chassis_motor_N;i++){
		motor[i].V_X = remote->x +remote->w*motor[i].x_y[0];
		motor[i].V_Y = remote->y +remote->w*motor[i].x_y[1];
		arm_sqrt_f32(motor[i].V_X*motor[i].V_X+motor[i].V_Y*motor[i].V_Y,&motor[i].speed_in);
		motor[i].angle = acos(motor[i].V_X/motor[i].speed_in);
		if(motor[i].V_Y<0) motor[i].angle = 2*pi - motor[i].angle;
		else if(motor[i].V_Y==0&&motor[i].V_X==0)motor[i].angle = 0;
		chassisMath_D_chassis_angle(&motor[i]);		
		motor[i].seat = motor[i].angle *8191/(2*pi) + *motor[i].original_seat;
		if(motor[i].seat>8191) motor[i].seat-=8191;
		motor[i].speed = motor[i].speed_in*19;
	}
}

static fp32 angle_set_ref(fp32 set,fp32 ref){
	if(set>ref)
		return set-ref<=pi?set-ref:2*pi-set+ref;
	return ref-set<=pi?ref-set:2*pi-ref+set;
}

void chassisMath_D_chassis_angle(chassis_D_wheel *motor){
	motor->angle_last = *motor->motor_ecd - *motor->original_seat;
	if(motor->angle_last<0) motor->angle_last+=8191;
	fp32 chassisMath_D_angle=angle_set_ref(motor->angle,motor->angle_last);
	
	if(chassisMath_D_angle<=pi/2){
		if(chassisMath_D_angle>=pi*4) motor->speed=0;
		else motor->speed =motor->speed_in;
	}
	else{
		motor->angle-=pi;
		motor->angle<0?motor->angle+=2*pi:motor->angle;
		motor->speed =-motor->speed_in;
	}

}

