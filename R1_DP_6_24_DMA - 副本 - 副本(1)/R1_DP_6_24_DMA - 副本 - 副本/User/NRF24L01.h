#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "NRF24L01_Ins.h"

/**
  * @brief  接收数据结构体
  * @note   与发送端 Bufer[10] 逐字节对应:
  *         Bufer[0] = 9 (协议头)
  *         Bufer[1] = AD_Value[0] >> 4   (摇杆ADC1)
  *         Bufer[2] = AD_Value[1] >> 4   (摇杆ADC2)
  *         Bufer[3] = AD_Value[2] >> 4   (摇杆ADC3)
  *         Bufer[4] = AD_Value[3] >> 4   (摇杆ADC4)
  *         Bufer[5] = KeyNum             (按键1)
  *         Bufer[6] = KeyNum_bg_left     (左拨轮)
  *         Bufer[7] = KeyNum_bg_right    (右拨轮)
  *         Bufer[8] = ecd1               (编码器1)
  *         Bufer[9] = ecd2               (编码器2)
  */
typedef struct {
    uint8_t header;  /* 协议头标识       */
    uint8_t adc1;    /* 摇杆ADC值1       */
    uint8_t adc2;    /* 摇杆ADC值2       */
    uint8_t adc3;    /* 摇杆ADC值3       */
    uint8_t adc4;    /* 摇杆ADC值4       */
    uint8_t key1;    /* 按键值1          */
    uint8_t key2;    /* 按键值2(左拨轮)  */
    uint8_t key3;    /* 按键值3(右拨轮)  */
    uint8_t ecd1;    /* 编码器值1        */
    uint8_t ecd2;    /* 编码器值2        */
} NRF_RxData_t;

/* ---- 底层 SPI 读写 (硬件SPI2 + 软件CSN) ---- */
uint8_t NRF_SPI_RW_Byte(uint8_t byte);
uint8_t NRF24L01_Write_Reg(uint8_t cmd, uint8_t value);
uint8_t NRF24L01_Read_Reg(uint8_t cmd);
uint8_t NRF24L01_Write_Buf(uint8_t cmd, uint8_t *buf, uint8_t len);
uint8_t NRF24L01_Read_Buf(uint8_t cmd, uint8_t *buf, uint8_t len);

/* ---- 应用层 API ---- */
uint8_t NRF24L01_Check(void);                    /* SPI 通信自检                 */
uint8_t NRF24L01_Init(void);                     /* 初始化接收模式                */
void    NRF24L01_read(void);                     /* 主循环查询→缓存数据          */
uint8_t NRF24L01_GetData(NRF_RxData_t *data);    /* 取缓存数据(线程安全)         */
uint8_t NRF24L01_GetRxBuf(uint8_t *rxbuf);       /* 取环形缓冲区最早一帧          */

/* ---- 调试/状态 ---- */
uint8_t NRF24L01_GetStatus(void);                /* 读 STATUS 寄存器              */
uint8_t NRF24L01_GetFIFOStatus(void);            /* 读 FIFO_STATUS 寄存器         */
void    NRF24L01_GetLastRaw(uint8_t *raw);       /* 获取最后一帧原始9字节载荷     */
uint8_t NRF24L01_GetRxBufCount(void);            /* 获取缓冲区待处理帧数          */
extern uint32_t nrf_rx_count;                    /* 接收包计数                   */

#endif
