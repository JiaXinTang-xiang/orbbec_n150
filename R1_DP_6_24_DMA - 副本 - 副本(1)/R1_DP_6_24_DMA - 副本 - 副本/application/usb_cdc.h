/**
 * @file usb_cdc.h
 * @brief USB CDC 通信 (协议: 0xAA CMD + 0xA5 0x5A 视觉, 两套并存)
 *
 * PC → STM32:
 *   [0xAA][CMD][DATA][CRC]  底盘/继电器/驱动器
 *   [0xA5][0x5A][14B][CRC]  视觉数据 (class+cx+cy+xyz+dist)
 *
 * STM32 → PC (每5ms):
 *   [0xAA][0x81][x:i32][y:i32][w:i32][relay:u8]
 *              [angle1:fp32][angle7:fp32][angle5:fp32][angle8:fp32][CRC]
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include "struct_typedef.h"

/* ── 0xAA 协议 ── */
#define USB_SOF         0xAA
#define USB_CMD_CHASSIS 0x01   /* 3×fp32: vx,vy,vw */
#define USB_CMD_DRIVER  0x02   /* u8(id)+fp32(angle) */
#define USB_CMD_RELAY   0x03   /* u8: bit0~7 */
#define USB_CMD_ECHO    0x05   /* [len][data...] */

/* ── 视觉协议 ── */
#define VIS_SYNC1  0xA5
#define VIS_SYNC2  0x5A

/* ── USB CDC 写入的目标值 ── */
extern volatile fp32    usb_chassis_vx, usb_chassis_vy, usb_chassis_vw;
extern volatile uint8_t usb_chassis_active;

extern volatile uint8_t usb_relay_flags;
extern volatile uint8_t usb_relay_updated;

/* 视觉数据 */
typedef struct {
	uint8_t  class_id;
	uint16_t cx, cy;
	int16_t  x_mm, y_mm, z_mm;
	uint16_t dist_mm;
	uint8_t  has_target, has_depth, updated;
} vision_t;
extern volatile vision_t vision;

/* ── API ── */
void usb_cdc_init(void);
void usb_cdc_on_rx(uint8_t *buf, uint32_t len);   /* CDC_Receive_FS 调用 */
void usb_cdc_tick(void);                            /* 反馈发送 (每5ms) */
void usb_cdc_relay_task(void);                      /* 继电器输出 (每1ms) */
void usb_recet_flage(void);
#endif
