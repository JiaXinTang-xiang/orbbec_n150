#include "remote_behaviou.h"
#include "arm_math.h"
#include "NRF24L01.h"

extern RemoteControlData_t rc_data;

int16_t remote_count[3]={2,3,3};
void remote_behaviou_set(remote_rc *remote)
{
	int16_t remote_behaviou_x=0,remote_behaviou_y=0,remote_behaviou_w=0;

	// 从NRF24L01 2.4G接收数据 rc_data 获取遥控值
	// 发射端已统一映射到25~255，中心140，Ch0/Ch1已在发射端翻转
	int16_t raw_x = (int16_t)rc_data.adc_ch1 - 140;  // 右摇杆水平 -> x
	int16_t raw_y = (int16_t)rc_data.adc_ch2 - 140;  // 右摇杆垂直 -> y
	int16_t raw_w = (int16_t)rc_data.adc_ch3 - 140;  // 左摇杆垂直 -> w
	int16_t ch_x = (raw_x > -20 && raw_x < 20) ? 0 : raw_x * 5;
	int16_t ch_y = (raw_y > -20 && raw_y < 20) ? 0 : raw_y * 5;
	int16_t ch_w = (raw_w > -20 && raw_w < 20) ? 0 : raw_w * 5;

	// 使用 key_bg_left 和 key_bg_right 作为模式开关
	if(rc_data.key_bg_left==1&&rc_data.key_bg_right==3){
		remote_behaviou_x=ch_x/90!=0 ? ch_x/10-10 : 0;
		remote_behaviou_y=ch_y/90!=0 ? ch_y/10-10 : 0;
		remote_behaviou_w=ch_w/90!=0 ? ch_w/10-10 : 0;
		remote_behaviou_w*=-1;
		remote->control_mode=remote_run1;
		if(abs(remote_behaviou_x)>abs(remote_behaviou_y)){
			remote_behaviou_y=0;
		}
		else{
			remote_behaviou_x=0;
		}
	}
	else if(rc_data.key_bg_left==1&&rc_data.key_bg_right==4)
	{
		remote_behaviou_x=ch_x/25!=0 ? ch_x/70-1 : 0;
		remote_behaviou_y=ch_y/25!=0 ? ch_y/70-1 : 0;
		remote_behaviou_w=ch_w/25!=0 ? ch_w/70-1 : 0;

		remote->control_mode=remote_run1;
		if(abs(remote_behaviou_x)>abs(remote_behaviou_y)){
			remote_behaviou_y=0;
		}
		else{
			remote_behaviou_x=0;
		}
	}
	else if(rc_data.key_bg_left==2&&rc_data.key_bg_right==4){
		remote_behaviou_x=ch_x/100!=0 ? ch_x/10-10 : 0;
		remote_behaviou_y=ch_y/100!=0 ? ch_y/10-10 : 0;
		remote_behaviou_w=ch_w/100!=0 ? ch_w/10-10 : 0;

		remote->control_mode=remote_run2;

	}

	remote->set_x=remote_behaviou_x*100/5;
	remote->set_y=remote_behaviou_y*100/5;
	remote->set_w=remote_behaviou_w*100/5;

	// 将计算值写入 NRF24L01 回复缓冲区（F1端可接收）
	nrf_reply_buf[0] = 1;  // 帧头(≠9避免F1回环)                           // 帧头
	nrf_reply_buf[1] = (remote_behaviou_x >> 8) & 0xFF;  // x 高字节
	nrf_reply_buf[2] =  remote_behaviou_x       & 0xFF;  // x 低字节
	nrf_reply_buf[3] = (remote_behaviou_y >> 8) & 0xFF;  // y 高字节
	nrf_reply_buf[4] =  remote_behaviou_y       & 0xFF;  // y 低字节
	nrf_reply_buf[5] = (remote_behaviou_w >> 8) & 0xFF;  // w 高字节
	nrf_reply_buf[6] =  remote_behaviou_w       & 0xFF;  // w 低字节
	// nrf_reply_buf[7..11] 保留为0，可扩展

		/* reply sent by NRF24L01_Task */
}

void remote_behaviou_init(remote_rc *remote){
	// NRF24L01 已在 main.c 中初始化，直接初始化遥控结构体
	remote->Remote_rc = 0;
	remote->set_x=0.0f;
	remote->set_y=0.0f;
	remote->set_w=0.0f;
	remote->control_mode=remote_run2;
}

