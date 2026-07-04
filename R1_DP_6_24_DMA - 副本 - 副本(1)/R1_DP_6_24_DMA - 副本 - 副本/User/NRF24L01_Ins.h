#ifndef __NRF24L01_INS_H
#define __NRF24L01_INS_H

#include "main.h"

/**********  NRF24L01 引脚定义  ******************/
/* CSN  = PB12 (软件片选)                        */
/* CE   = PB11 (接收使能，与发送端逻辑对齐)        */
/* SCK  = PB13, MISO = PB14, MOSI = PB15          */
/*        → CubeMX 已配置为 SPI2，见 spi.c        */
/************************************************/

#define NRF_CSN_PORT        GPIOB
#define NRF_CSN_PIN         GPIO_PIN_12
#define NRF_CE_PORT         GPIOB
#define NRF_CE_PIN          GPIO_PIN_11

/**********  引脚快捷操作宏  ***********/
#define CSN_LOW()           HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET)
#define CSN_HIGH()          HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET)
#define CE_LOW()            HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_RESET)
#define CE_HIGH()           HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_SET)

/**********  NRF24L01 寄存器指令定义  ***********/
#define NRF_READ_REG        0x00    /* 读寄存器命令 (低5位=地址)    */
#define NRF_WRITE_REG       0x20    /* 写寄存器命令 (低5位=地址)    */
#define RD_RX_PLOAD         0x61    /* 读RX FIFO有效载荷           */
#define WR_TX_PLOAD         0xA0    /* 写TX FIFO有效载荷           */
#define FLUSH_TX            0xE1    /* 清空TX FIFO                 */
#define FLUSH_RX            0xE2    /* 清空RX FIFO                 */
#define REUSE_TX_PL         0xE3    /* 复用上次发送载荷             */
#define NOP                 0xFF    /* 空操作                      */

/**********  NRF24L01 寄存器地址  *************/
#define CONFIG              0x00
#define EN_AA               0x01
#define EN_RXADDR           0x02
#define SETUP_AW            0x03
#define SETUP_RETR          0x04
#define RF_CH               0x05
#define RF_SETUP            0x06
#define STATUS              0x07
#define OBSERVE_TX          0x08
#define CD                  0x09
#define RX_ADDR_P0          0x0A
#define RX_ADDR_P1          0x0B
#define RX_ADDR_P2          0x0C
#define RX_ADDR_P3          0x0D
#define RX_ADDR_P4          0x0E
#define RX_ADDR_P5          0x0F
#define TX_ADDR             0x10
#define RX_PW_P0            0x11
#define RX_PW_P1            0x12
#define RX_PW_P2            0x13
#define RX_PW_P3            0x14
#define RX_PW_P4            0x15
#define RX_PW_P5            0x16
#define FIFO_STATUS         0x17

/******  STATUS 寄存器 bit 位定义  *******/
#define MAX_TX              0x10
#define TX_OK               0x20
#define RX_OK               0x40

/******  CONFIG 寄存器 bit 位定义  *******/
#define MASK_RX_DR          0x40
#define MASK_TX_DS          0x20
#define MASK_MAX_RT         0x10
#define EN_CRC              0x08
#define CRCO                0x04
#define PWR_UP              0x02
#define PRIM_RX             0x01

#endif
