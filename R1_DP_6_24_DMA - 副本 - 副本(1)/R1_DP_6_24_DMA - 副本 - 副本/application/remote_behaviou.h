#ifndef __REMOTE_BEHAVIOU_H__
#define __REMOTE_BEHAVIOU_H__

#include "struct_typedef.h"
#include "remote_control.h"
#include "arm_math.h"
#include "tim.h"


typedef enum{
	remote_run = 1,
	remote_run1 = 2,
	remote_run2 = 3


}remote_mode;


typedef struct{
	const RC_ctrl_t *Remote_rc;
	uint8_t control_mode;
	int32_t set_x;
	int32_t set_y;
	int32_t set_w;

}remote_rc;
	
void remote_behaviou_init(remote_rc *remote);
void remote_behaviou_set(remote_rc *remote);

#endif

