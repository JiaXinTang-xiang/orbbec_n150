#ifndef __nRF24L01_H
#define __nRF24L01_H

#include "main.h"
extern SPI_HandleTypeDef hspi2;

// 仅保留软件片选 CSN
#define CSN_Port	GPIOB
#define CSN_Pin		GPIO_PIN_12

// 寄存器命令
#define nRF_READ_REG        0x00
#define nRF_WRITE_REG       0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

// 寄存器地址
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define CD              0x09
#define RX_ADDR_P0      0x0A
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define FIFO_STATUS     0x17

// 状态位
#define MAX_TX  	0x10
#define TX_OK   	0x20
#define RX_OK   	0x40

#endif