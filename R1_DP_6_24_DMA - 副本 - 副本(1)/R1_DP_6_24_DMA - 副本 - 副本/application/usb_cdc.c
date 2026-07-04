/**
 * @file usb_cdc.c
 * @brief USB CDC 双协议解析 + 命令分发 + 反馈
 */

#include "usb_cdc.h"
#include "usbd_cdc_if.h"
#include "NRF24L01.h"

extern RemoteControlData_t rc_data;

extern int32_t x, y, w;
extern float angle_5, angle_1, angle_7, angle_8;

/* ================================================================
   目标变量
   ================================================================ */
volatile fp32    usb_chassis_vx, usb_chassis_vy, usb_chassis_vw;
volatile uint8_t usb_chassis_active;

volatile uint8_t usb_relay_flags;
volatile uint8_t usb_relay_updated;

volatile vision_t vision;

/* Debug: 复位标志发送计数 (Watch窗口可看) */
volatile uint32_t usb_reset_sent_cnt = 0;
volatile uint8_t  usb_reset_last_ret = 0;   /* CDC_Transmit_FS 返回值 */

/* ================================================================
   0xAA 协议状态机
   ================================================================ */
enum { AA_SOF, AA_CMD, AA_LEN, AA_DATA, AA_CRC };
static uint8_t  _buf[64], _idx, _st = AA_SOF, _cmd, _dlen, _crc;

/* ================================================================
   0xA5 0x5A 视觉协议状态机
   ================================================================ */
enum { V_SYNC1, V_SYNC2, V_DATA, V_CRC };
static uint8_t  _vst = V_SYNC1, _vidx, _vcrc, _vbuf[14];

/* ================================================================
   CRC
   ================================================================ */
static uint8_t _xor(const uint8_t *d, uint8_t n) {
	uint8_t c = 0; while (n--) c ^= *d++; return c;
}

/* ================================================================
   初始化
   ================================================================ */
void usb_cdc_init(void) {
	_st = AA_SOF; _vst = V_SYNC1;
	usb_chassis_active = 0;
}

/* ================================================================
   接收入口 (CDC_Receive_FS 调用)
   ================================================================ */
void usb_cdc_on_rx(uint8_t *buf, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		uint8_t b = buf[i];

		/* ── 0xA5 0x5A 视觉协议 ── */
		switch (_vst) {
		case V_SYNC1:
			if (b == VIS_SYNC1) { _vcrc = b; _vst = V_SYNC2; }
			break;
		case V_SYNC2:
			if (b == VIS_SYNC2) { _vcrc ^= b; _vidx = 0; _vst = V_DATA; }
			else _vst = V_SYNC1;
			break;
		case V_DATA:
			_vbuf[_vidx++] = b; _vcrc ^= b;
			if (_vidx >= 14) _vst = V_CRC;
			break;
		case V_CRC:
			if (b == _vcrc) {
				vision.class_id = _vbuf[0];
				vision.cx       = _vbuf[1] | ((uint16_t)_vbuf[2] << 8);
				vision.cy       = _vbuf[3] | ((uint16_t)_vbuf[4] << 8);
				vision.x_mm     = _vbuf[5] | ((int16_t)_vbuf[6]  << 8);
				vision.y_mm     = _vbuf[7] | ((int16_t)_vbuf[8]  << 8);
				vision.z_mm     = _vbuf[9] | ((int16_t)_vbuf[10] << 8);
				vision.dist_mm  = _vbuf[11]| ((uint16_t)_vbuf[12]<< 8);
				vision.has_target = (vision.cx || vision.cy);
				vision.has_depth  = (vision.dist_mm != 0xFFFF);
				vision.updated = 1;

				/* 立即反馈: [0xAA][0x81][0x04][CRC] */
				{
					uint8_t fb[4];
					fb[0] = USB_SOF; fb[1] = 0x81;
					fb[2] = 0x04;
					fb[3] = _xor(fb, 3);
					CDC_Transmit_FS(fb, 4);
				}
			}
			_vst = V_SYNC1;
			break;
		}

		/* ── 0xAA 命令协议 ── */
		switch (_st) {
		case AA_SOF:
			if (b == USB_SOF) _st = AA_CMD;
			break;
		case AA_CMD:
			_cmd = b; _crc = b; _dlen = 0;
			switch (b) {
			case USB_CMD_CHASSIS: _dlen = 12; break;       /* 3×fp32 */
			case USB_CMD_DRIVER:  _dlen = 5;  break;       /* u8+fp32 */
			case USB_CMD_RELAY:   _dlen = 1;  break;       /* u8 */
			case USB_CMD_ECHO:
				_st = AA_LEN; continue;
			default:
				_st = AA_SOF; continue;
			}
			_idx = 0; _st = _dlen ? AA_DATA : AA_CRC;
			break;
		case AA_LEN:
			_dlen = b; _crc ^= b; _idx = 0;
			_st = _dlen ? AA_DATA : AA_CRC;
			break;
		case AA_DATA:
			if (_idx < sizeof(_buf)) _buf[_idx++] = b;
			_crc ^= b;
			if (_idx >= _dlen) _st = AA_CRC;
			break;
		case AA_CRC:
			if (b == _crc && _dlen <= sizeof(_buf)) {
				const fp32 *f = (const fp32 *)_buf;
				switch (_cmd) {
				case USB_CMD_CHASSIS:
					usb_chassis_vx = f[0];
					usb_chassis_vy = f[1];
					usb_chassis_vw = f[2];
					usb_chassis_active = 1;
					break;
				case USB_CMD_RELAY:
					usb_relay_flags = _buf[0];
					usb_relay_updated = 1;
					break;
				case USB_CMD_ECHO: {
					uint8_t eb[64], *ep = eb;
					*ep++ = USB_SOF; *ep++ = 0x85;
					*ep++ = _dlen + 2;
					*ep++ = 'O'; *ep++ = 'K';
					for (uint8_t j = 0; j < _dlen; j++) *ep++ = _buf[j];
					uint8_t fl = (uint8_t)(ep - eb);
					*ep = _xor(eb, fl);
					CDC_Transmit_FS(eb, fl + 1);
					break;
				}
				}
			}
			_st = AA_SOF;
			break;
		}
	}
}

/* ================================================================
   继电器输出 (每1ms)
   ================================================================ */
extern void relay_control(uint8_t id, uint8_t state);
void usb_cdc_relay_task(void) {
	if (usb_relay_updated) {
		for (uint8_t i = 0; i < 8; i++)
			relay_control(i + 1, (usb_relay_flags & (1 << i)) ? 0 : 1);
		usb_relay_updated = 0;
	}
}

/* ================================================================
   反馈帧 (每5ms) — chassis/relay 标志
   帧: [0xAA][0x81][flags:u8][CRC]  = 4 bytes
   flags: bit0=chassis_active, bit1=relay_updated
   注意: vision 反馈在收到帧时立即发送，不在这里轮询
   ================================================================ */
void usb_cdc_tick(void) {
	uint8_t tx[4];
	uint8_t flags = 0;

	if (usb_chassis_active) flags |= 0x01;
	if (usb_relay_updated)  flags |= 0x02;

	/* 没变化就不发 */
	static uint8_t last_flags = 0xFF;
	if (flags == last_flags) return;
	last_flags = flags;

	tx[0] = USB_SOF; tx[1] = 0x81;
	tx[2] = flags;
	tx[3] = _xor(tx, 3);
	CDC_Transmit_FS(tx, 4);
}


void usb_recet_flage(void)
{
	if(rc_data.encoder1 >= 20 && (rc_data.key_bg_left==1 && rc_data.key_bg_right==4))
	{
		uint8_t tx[8], *p = tx;
		*p++ = USB_SOF; *p++ = 0x82;
		*p++ = 0x01;
		uint8_t fl = (uint8_t)(p - tx);
		*p = _xor(tx, fl);
		usb_reset_last_ret = CDC_Transmit_FS(tx, fl + 1);
		usb_reset_sent_cnt++;
	}
}
