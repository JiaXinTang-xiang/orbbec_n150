#include "NRF24L01.h"
#include "NRF24L01_Ins.h"
#include "main.h"
#include "spi.h"
#include <string.h>

#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  12
#define RX_PLOAD_WIDTH  12

const uint8_t MY_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF};
const uint8_t REMOTE_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF};

RemoteControlData_t rc_data = {0};
uint8_t Buf[12] = {0};
uint8_t nrf_reply_buf[TX_PLOAD_WIDTH] = {1, 0};
static uint8_t nrf_ready = 0;


void W_SS(uint8_t BitValue)
{
    if(BitValue) HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
    else         HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
}

uint8_t SPI_SwapByte(uint8_t Byte)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi2, &Byte, &rx_data, 1, 100);
    return rx_data;
}

void NRF24L01_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = CSN_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CSN_Port, &GPIO_InitStructure);
    W_SS(1);
}

uint8_t NRF24L01_Write_Reg(uint8_t Reg, uint8_t Value)
{
    uint8_t Status;
    W_SS(0); Status = SPI_SwapByte(Reg); SPI_SwapByte(Value); W_SS(1);
    return Status;
}

uint8_t NRF24L01_Read_Reg(uint8_t Reg)
{
    uint8_t Value;
    W_SS(0); SPI_SwapByte(Reg); Value = SPI_SwapByte(NOP); W_SS(1);
    return Value;
}

uint8_t NRF24L01_Read_Buf_DMA(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t Status;
    static uint8_t dummy_tx[RX_PLOAD_WIDTH];
    memset(dummy_tx, NOP, Len);
    W_SS(0); Status = SPI_SwapByte(Reg);
    HAL_SPI_TransmitReceive_DMA(&hspi2, dummy_tx, Buf, Len);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    W_SS(1);
    return Status;
}

uint8_t NRF24L01_Write_Buf_DMA(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t Status;
    static uint8_t dummy_rx[RX_PLOAD_WIDTH];
    W_SS(0); Status = SPI_SwapByte(Reg);
    HAL_SPI_TransmitReceive_DMA(&hspi2, Buf, dummy_rx, Len);
    while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
    W_SS(1);
    return Status;
}

uint8_t NRF24L01_Read_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t Status, i;
    W_SS(0); Status = SPI_SwapByte(Reg);
    for(i = 0; i < Len; i++) Buf[i] = SPI_SwapByte(NOP);
    W_SS(1); return Status;
}

uint8_t NRF24L01_Write_Buf(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t Status, i;
    W_SS(0); Status = SPI_SwapByte(Reg);
    for(i = 0; i < Len; i++) SPI_SwapByte(*Buf++);
    W_SS(1); return Status;
}

uint8_t NRF24L01_GetRxBuf(uint8_t *Buf)
{
    uint8_t State, FifoSta;
    State = NRF24L01_Read_Reg(STATUS);
    if(State & RX_OK)
    {
        FifoSta = NRF24L01_Read_Reg(FIFO_STATUS);
        if(FifoSta & 0x01) { NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, RX_OK); return 1; }
        NRF24L01_Read_Buf(RD_RX_PLOAD, Buf, RX_PLOAD_WIDTH);
        NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State & (RX_OK | TX_OK | MAX_TX));
        return 0;
    }
    if(State & (TX_OK | MAX_TX))
        NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State & (TX_OK | MAX_TX));
    return 1;
}

uint8_t NRF24L01_SendTxBuf(uint8_t *Buf)
{
    uint8_t State;
    State = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State);
    NRF24L01_Write_Buf(WR_TX_PLOAD, Buf, TX_PLOAD_WIDTH);
    uint32_t timeout = 50000;
    do { State = NRF24L01_Read_Reg(STATUS);
        if(State & MAX_TX) { NRF24L01_Write_Reg(FLUSH_TX, NOP);
            NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, MAX_TX); return MAX_TX; }
        timeout--;
    } while(!(State & TX_OK) && timeout > 0);
    if(State & TX_OK) { NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, TX_OK); return TX_OK; }
    NRF24L01_Write_Reg(FLUSH_TX, NOP);
    return NOP;
}

uint8_t NRF24L01_Check(void)
{
    uint8_t check_in_buf[5] = {0x11,0x22,0x33,0x44,0x55};
    uint8_t check_out_buf[5] = {0};
    W_SS(1);
    NRF24L01_Write_Buf(nRF_WRITE_REG + TX_ADDR, check_in_buf, 5);
    NRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, check_out_buf, 5);
    if(check_out_buf[0]==0x11 && check_out_buf[1]==0x22 && check_out_buf[2]==0x33
       && check_out_buf[3]==0x44 && check_out_buf[4]==0x55) return 0;
    return 1;
}

void NRF24L01_RT_Init(void)
{
    NRF24L01_Write_Reg(FLUSH_RX, NOP);
    NRF24L01_Write_Reg(FLUSH_TX, NOP);
    NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, 0x70);
    NRF24L01_Write_Reg(nRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    NRF24L01_Write_Buf_DMA(nRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)MY_ADDRESS, RX_ADR_WIDTH);
    NRF24L01_Write_Buf_DMA(nRF_WRITE_REG + TX_ADDR, (uint8_t*)REMOTE_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Reg(nRF_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(nRF_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(nRF_WRITE_REG + SETUP_RETR, 0x03);
    NRF24L01_Write_Reg(nRF_WRITE_REG + RF_CH, 0);
    NRF24L01_Write_Reg(nRF_WRITE_REG + RF_SETUP, 0x0F);
    NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0F);
}

void NRF24L01_Init(void)
{
    NRF24L01_Pin_Init();
    while(NRF24L01_Check());
    NRF24L01_RT_Init();
    nrf_ready = 1;
}

void NRF24L01_SendBuf(uint8_t *Buf)
{
    uint8_t State;
    State = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, State);
    NRF24L01_Write_Reg(FLUSH_TX, NOP);
    NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0E);
    NRF24L01_Write_Buf(WR_TX_PLOAD, Buf, TX_PLOAD_WIDTH);
    uint32_t timeout = 2500;
    do { State = NRF24L01_Read_Reg(STATUS);
        if(State & MAX_TX) { NRF24L01_Write_Reg(FLUSH_TX, NOP);
            NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, MAX_TX); break; }
        timeout--;
    } while(!(State & TX_OK) && timeout > 0);
    if(State & TX_OK) NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, TX_OK);
    NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG, 0x0F);
}

uint8_t NRF24L01_ReceiveBuf(uint8_t *Buf)
{
    if(NRF24L01_GetRxBuf(Buf) == 0) return (Buf[0] != 9) ? 1 : 0;
    return 1;
}

void NRF24L01_Reset(void)
{
    if(NRF24L01_GetRxBuf(Buf) == 0) memcpy(&rc_data, Buf, sizeof(RemoteControlData_t));
}

// ─── 收+发（TIM1 ISR 调用）───
void NRF24L01_Task(void)
{
    if (!nrf_ready) return;
    if (NRF24L01_ReceiveBuf((uint8_t *)&rc_data) == 0)
    {
        NRF24L01_SendBuf(nrf_reply_buf);
    }
}

void NRF24L01_AutoRx(void)     { NRF24L01_Task(); }
void NRF24L01_ServiceReply(void) {}
