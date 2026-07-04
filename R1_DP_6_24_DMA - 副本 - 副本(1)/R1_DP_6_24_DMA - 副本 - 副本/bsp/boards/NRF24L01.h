#ifndef __nRF24L01_API_H
#define __nRF24L01_API_H

#include "main.h"

uint8_t SPI_SwapByte(uint8_t byte);
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Buf_DMA(uint8_t reg,uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Write_Buf_DMA(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_GetRxBuf(uint8_t *rxbuf);
uint8_t NRF24L01_SendTxBuf(uint8_t *txbuf);
uint8_t NRF24L01_Check(void);
void NRF24L01_RT_Init(void);
void NRF24L01_Init(void);
void NRF24L01_SendBuf(uint8_t *Buf);
void NRF24L01_Pin_Init(void);
void NRF24L01_Reset(void);
uint8_t NRF24L01_ReceiveBuf(uint8_t *Buf);
void NRF24L01_Task(void);
void NRF24L01_AutoRx(void);       // 自动接收轮询（定时器ISR调用）
void NRF24L01_ServiceReply(void); // 回复服务（主循环调用）

// 遥控器数据结构体（12字节，与发送端一致）
typedef struct {
    uint8_t  frame_header;     // 帧头 (1字节)
    uint8_t  adc_ch1;          // ADC通道1 (AD_Value[0]>>4, 1字节)
    uint8_t  adc_ch2;          // ADC通道2 (AD_Value[1]>>4, 1字节)
    uint8_t  adc_ch3;          // ADC通道3 (AD_Value[2]>>4, 1字节)
    uint8_t  adc_ch4;          // ADC通道4 (AD_Value[3]>>4, 1字节)
    uint8_t  key_number;       // 按键值 (KeyNum, 1字节)
    uint8_t  key_bg_left;      // 左侧按键 (KeyNum_bg_left, 1字节)
    uint8_t  key_bg_right;     // 右侧按键 (KeyNum_bg_right, 1字节)
    uint8_t encoder1;         // 编码器1 (ecd1, 2字节)
    uint8_t encoder2;         // 编码器2 (ecd2, 2字节)
    uint8_t  encoder1_key;    // 编码器1按键 (Encoder1_Key_Count, 1字节)
    uint8_t  encoder2_key;     // 编码器2按键 (Encoder2_Key_Count, 1字节)
} __attribute__((packed)) RemoteControlData_t;

extern RemoteControlData_t rc_data;
extern uint8_t nrf_reply_buf[12];     // 回复发送缓冲区，计算完值后填入

#endif