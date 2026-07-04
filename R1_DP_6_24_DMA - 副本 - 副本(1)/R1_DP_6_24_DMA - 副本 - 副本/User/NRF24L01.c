#include "NRF24L01.h"
#include "spi.h"            /* hspi2 —— CubeMX 生成的 SPI2 句柄 */

/* ================================================================
   协议参数 —— 与 F1 发送端 NRF24L01_RT_Init() 逐项对齐
   ================================================================ */
#define NRF_PAYLOAD_SIZE    9       /* 载荷字节数 (匹配 TX_PLOAD_WIDTH) */
#define NRF_RF_CHANNEL      76      /* 射频通道 2476MHz (匹配 RF_CH)    */
#define NRF_BUFFER_SIZE     8       /* 接收环形缓冲区深度               */

/* 5 字节广播地址 (匹配发送端 TX_ADDRESS / RX_ADDRESS) */
static const uint8_t NRF_ADDR[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ================================================================
   接收环形缓冲区
   ================================================================ */
static NRF_RxData_t RxBuffer[NRF_BUFFER_SIZE];
static uint8_t RxHead = 0;          /* 写指针 */
static uint8_t RxTail = 0;          /* 读指针 */
static uint8_t RxCount = 0;         /* 缓冲帧数 */

NRF_RxData_t nrf_rx_data;           /* 最新一帧缓存 (兼容旧 API)      */
uint8_t      RxDataValid = 0;       /* 数据有效标志 (兼容旧 API)      */
uint32_t     nrf_rx_count = 0;      /* 接收包计数 (调试用)            */
static uint8_t LastRaw[9] = {0};    /* 最后一帧原始数据 (调试用)      */

/* ================================================================
   底层 SPI 驱动 —— 硬件 SPI2 + 软件 CSN/CE (5 线连接)
   CubeMX SPI2:  PB13=SCK, PB14=MISO, PB15=MOSI
   软件 GPIO:    PB12=CSN, PB11=CE
   SPI 模式:     CPOL=0, CPHA=0, MSB First, 8bit
   ================================================================ */

/**
  * @brief  SPI 单字节全双工收发
  * @param  byte: 主机发送字节
  * @retval 从机返回字节 (NRF24L01 的 STATUS 或读取数据)
  */
uint8_t NRF_SPI_RW_Byte(uint8_t byte)
{
    uint8_t rx_byte;
    HAL_SPI_TransmitReceive(&hspi2, &byte, &rx_byte, 1, 50);
    return rx_byte;
}

/**
  * @brief  写 NRF24L01 寄存器
  * @param  cmd:   SPI 命令字节 (调用者拼好: NRF_WRITE_REG | 寄存器地址)
  * @param  value: 写入值
  * @retval STATUS 寄存器值
  */
uint8_t NRF24L01_Write_Reg(uint8_t cmd, uint8_t value)
{
    uint8_t status;
    CSN_LOW();
    status = NRF_SPI_RW_Byte(cmd);
    NRF_SPI_RW_Byte(value);
    CSN_HIGH();
    return status;
}

/**
  * @brief  读 NRF24L01 寄存器
  * @param  cmd: SPI 命令字节 (调用者拼好: NRF_READ_REG | 寄存器地址)
  * @retval 寄存器当前值
  */
uint8_t NRF24L01_Read_Reg(uint8_t cmd)
{
    uint8_t value;
    CSN_LOW();
    NRF_SPI_RW_Byte(cmd);
    value = NRF_SPI_RW_Byte(NOP);
    CSN_HIGH();
    return value;
}

/**
  * @brief  连续写 NRF24L01 内部缓冲区
  * @param  cmd: SPI 命令字节 (调用者拼好: NRF_WRITE_REG | 寄存器地址)
  * @param  buf: 数据指针
  * @param  len: 写入长度
  * @retval STATUS 寄存器值
  */
uint8_t NRF24L01_Write_Buf(uint8_t cmd, uint8_t *buf, uint8_t len)
{
    uint8_t status, i;
    CSN_LOW();
    status = NRF_SPI_RW_Byte(cmd);
    for (i = 0; i < len; i++)
        NRF_SPI_RW_Byte(buf[i]);
    CSN_HIGH();
    return status;
}

/**
  * @brief  连续读 NRF24L01 内部缓冲区
  * @param  cmd: SPI 命令字节 (调用者拼好: NRF_READ_REG | 寄存器地址)
  * @param  buf: 接收数据指针
  * @param  len: 读取长度
  * @retval STATUS 寄存器值
  */
uint8_t NRF24L01_Read_Buf(uint8_t cmd, uint8_t *buf, uint8_t len)
{
    uint8_t status, i;
    CSN_LOW();
    status = NRF_SPI_RW_Byte(cmd);
    for (i = 0; i < len; i++)
        buf[i] = NRF_SPI_RW_Byte(NOP);
    CSN_HIGH();
    return status;
}

/* ================================================================
   NRF24L01 初始化
   ================================================================ */

/**
  * @brief  CSN + CE 引脚 GPIO 初始化
  * @note   SCK/MISO/MOSI 已由 CubeMX 在 MX_SPI2_Init() 中配置
  */
static void NRF_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ---- CSN (PB12) + CE (PB11): 推挽输出 ---- */
    GPIO_InitStruct.Pin   = NRF_CSN_PIN | NRF_CE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(NRF_CSN_PORT, &GPIO_InitStruct);

    /* 初始: CSN=HIGH(SPI空闲), CE=LOW(待机)  —— 与发送端 NRF24L01_Init() 一致 */
    CSN_HIGH();
    CE_LOW();
}

/**
  * @brief  NRF24L01 SPI 通信自检 —— 写读 TX_ADDR 验证
  * @retval 0: SPI 通信正常
  * @retval 1: 通信失败 (检查接线/供电)
  */
uint8_t NRF24L01_Check(void)
{
    uint8_t check[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t out[5]   = {0};

    NRF24L01_Write_Buf(NRF_WRITE_REG | TX_ADDR, check, 5);
    NRF24L01_Read_Buf(NRF_READ_REG | TX_ADDR, out, 5);

    if (out[0] == 0x11 && out[1] == 0x22 &&
        out[2] == 0x33 && out[3] == 0x44 && out[4] == 0x55)
        return 0;
    return 1;
}

/**
  * @brief  NRF24L01 接收模式初始化
  * @note   完全模仿 F1 发送端 NRF24L01_Init() + NRF24L01_RT_Init() 流程:
  *         1. CE=LOW 时配置寄存器
  *         2. 所有寄存器写完后 CE=HIGH 产生上升沿 → 进入 RX 模式
  *
  *         参数与发送端对齐:
  *         地址: 0xFFFFFFFFFF, 载荷: 9 字节
  *         速率: 2Mbps, 功率: 0dBm, 通道: 76
  *         CRC: 2 字节, 自动应答使能
  *
  *         接线: PB12=CSN, PB11=CE, PB13=SCK, PB14=MISO, PB15=MOSI
  * @retval 0: 成功
  */
uint8_t NRF24L01_Init(void)
{
    /* 1. GPIO 初始化: CSN=HIGH, CE=LOW */
    NRF_GPIO_Init();

    /* 2. SPI 通信自检 (与发送端一致: 循环直到通过) */
    while (NRF24L01_Check() != 0);

    /* 3. CE=LOW → Standby-I，确保寄存器配置期间芯片不进入收发状态 */
    CE_LOW();
    CSN_LOW();
    HAL_Delay(1);

    /* 4. 清空 FIFO */
    NRF24L01_Write_Reg(FLUSH_RX, NOP);
    NRF24L01_Write_Reg(FLUSH_TX, NOP);

    /* 5. 寄存器配置 —— 逐项对齐发送端 NRF24L01_RT_Init() */
    NRF24L01_Write_Reg(NRF_WRITE_REG | RX_PW_P0,   NRF_PAYLOAD_SIZE);
    NRF24L01_Write_Buf(NRF_WRITE_REG | TX_ADDR,    (uint8_t *)NRF_ADDR, 5);
    NRF24L01_Write_Buf(NRF_WRITE_REG | RX_ADDR_P0, (uint8_t *)NRF_ADDR, 5);
    NRF24L01_Write_Reg(NRF_WRITE_REG | EN_AA,      0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG | EN_RXADDR,  0x01);
    NRF24L01_Write_Reg(NRF_WRITE_REG | SETUP_RETR, 0x1A);
    NRF24L01_Write_Reg(NRF_WRITE_REG | RF_CH,      NRF_RF_CHANNEL);
    NRF24L01_Write_Reg(NRF_WRITE_REG | RF_SETUP,   0x0F);
    NRF24L01_Write_Reg(NRF_WRITE_REG | CONFIG,     0x0F);   /* RX 模式 */

    /* 6. CSN=HIGH 退出 SPI → CE=HIGH 上升沿 → 芯片进入 RX 监听模式 */
    CSN_HIGH();
    CE_HIGH();              /* 这个上升沿是关键！与发送端 CE_HIGH() 完全对应 */
    HAL_Delay(2);           /* 等待进入 RX (~130us) + 余量 */

    /* 7. 验证 CONFIG 写入 (可删除) */
    {
        uint8_t cfg = NRF24L01_Read_Reg(NRF_READ_REG | CONFIG);
        if (cfg != 0x0F)
            return 1;
    }

    return 0;
}

/* ================================================================
   数据接收 API
   ================================================================ */

/**
  * @brief  查询 RX FIFO 并读取有效载荷
  * @param  buf: 数据缓冲区 (>= NRF_PAYLOAD_SIZE 字节)
  * @retval 0: 有数据，已读入 buf
  * @retval 1: 当前无数据
  */
static uint8_t NRF24L01_PollRxBuf(uint8_t *buf)
{
    uint8_t status = NRF24L01_Read_Reg(NRF_READ_REG | STATUS);

    if (status & RX_OK)             /* RX_DR = 1, FIFO 中有数据 */
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD, buf, NRF_PAYLOAD_SIZE);
        NRF24L01_Write_Reg(NRF_WRITE_REG | STATUS, RX_OK); /* 写 1 清 RX_DR */
        return 0;
    }
    return 1;
}

/**
  * @brief  主循环接收函数 —— 在 while(1) 中周期性调用
  * @note   收到有效数据后存入环形缓冲区，同时更新最新数据缓存。
  *         会做协议头校验: raw[0] 必须等于 9
  *
  *         用法:
  *         while (1) {
  *             NRF24L01_read();
  *             // ...
  *         }
  */
void NRF24L01_read(void)
{
    uint8_t raw[NRF_PAYLOAD_SIZE];

    if (NRF24L01_PollRxBuf(raw) != 0)
        return;

    /* 保存原始数据(调试用) */
    LastRaw[0] = raw[0]; LastRaw[1] = raw[1]; LastRaw[2] = raw[2];
    LastRaw[3] = raw[3]; LastRaw[4] = raw[4]; LastRaw[5] = raw[5];
    LastRaw[6] = raw[6]; LastRaw[7] = raw[7]; LastRaw[8] = raw[8];

    /* 协议头校验 —— 发送端 Bufer[0] = 9
     * 调试阶段先注释，确认能收到数据后再启用 */
    // if (raw[0] != 9)
    //     return;

    /* 存入环形缓冲区 */
    if (RxCount < NRF_BUFFER_SIZE)
    {
        RxBuffer[RxHead].header = raw[0];
        RxBuffer[RxHead].adc1   = raw[1];
        RxBuffer[RxHead].adc2   = raw[2];
        RxBuffer[RxHead].adc3   = raw[3];
        RxBuffer[RxHead].adc4   = raw[4];
        RxBuffer[RxHead].key1   = raw[5];
        RxBuffer[RxHead].key2   = raw[6];
        RxBuffer[RxHead].key3   = raw[7];
        RxBuffer[RxHead].ecd1   = raw[8];
        RxBuffer[RxHead].ecd2   = 0;            /* 发送端 9 字节载荷不含 ecd2 */

        RxHead = (RxHead + 1) % NRF_BUFFER_SIZE;
        RxCount++;
    }

    /* 同步更新最新数据缓存 (兼容旧 API) */
    __disable_irq();
    nrf_rx_data = RxBuffer[(RxHead - 1 + NRF_BUFFER_SIZE) % NRF_BUFFER_SIZE];
    RxDataValid = 1;
    __enable_irq();
}

/**
  * @brief  获取接收数据 (线程安全)，读取后自动清除标志
  * @param  data: 输出数据指针
  * @retval 1: 数据有效，已拷贝到 data
  * @retval 0: 无新数据
  */
uint8_t NRF24L01_GetData(NRF_RxData_t *data)
{
    if (RxDataValid)
    {
        __disable_irq();
        *data = nrf_rx_data;
        RxDataValid = 0;
        __enable_irq();
        return 1;
    }
    return 0;
}

/**
  * @brief  从环形缓冲区取出最早一帧数据 (FIFO 顺序)
  * @param  rxbuf: 接收缓冲区 (>= 10 字节，按结构体字段顺序输出)
  * @retval 1: 取到数据
  * @retval 0: 缓冲区空
  */
uint8_t NRF24L01_GetRxBuf(uint8_t *rxbuf)
{
    if (RxCount == 0)
        return 0;

    __disable_irq();
    rxbuf[0] = RxBuffer[RxTail].header;
    rxbuf[1] = RxBuffer[RxTail].adc1;
    rxbuf[2] = RxBuffer[RxTail].adc2;
    rxbuf[3] = RxBuffer[RxTail].adc3;
    rxbuf[4] = RxBuffer[RxTail].adc4;
    rxbuf[5] = RxBuffer[RxTail].key1;
    rxbuf[6] = RxBuffer[RxTail].key2;
    rxbuf[7] = RxBuffer[RxTail].key3;
    rxbuf[8] = RxBuffer[RxTail].ecd1;
    rxbuf[9] = RxBuffer[RxTail].ecd2;

    RxTail = (RxTail + 1) % NRF_BUFFER_SIZE;
    RxCount--;
    __enable_irq();

    return 1;
}

/**
  * @brief  获取环形缓冲区中待处理的帧数
  * @retval 缓冲区计数 (0 ~ NRF_BUFFER_SIZE)
  */
uint8_t NRF24L01_GetRxBufCount(void)
{
    return RxCount;
}

/* ================================================================
   调试函数
   ================================================================ */

/**
  * @brief  读取 NRF24L01 STATUS 寄存器原始值
  * @retval STATUS (bit6=RX_DR, bit5=TX_DS, bit4=MAX_RT)
  */
uint8_t NRF24L01_GetStatus(void)
{
    return NRF24L01_Read_Reg(NRF_READ_REG | STATUS);
}

/**
  * @brief  读取 NRF24L01 FIFO_STATUS 寄存器
  * @retval FIFO_STATUS (bit0=RX_EMPTY, bit4=TX_FULL, 等)
  */
uint8_t NRF24L01_GetFIFOStatus(void)
{
    return NRF24L01_Read_Reg(NRF_READ_REG | FIFO_STATUS);
}

/**
  * @brief  获取最后一次接收到的原始 9 字节载荷
  * @param  raw: 输出缓冲区 (>=9 字节)
  */
void NRF24L01_GetLastRaw(uint8_t *raw)
{
    raw[0] = LastRaw[0]; raw[1] = LastRaw[1]; raw[2] = LastRaw[2];
    raw[3] = LastRaw[3]; raw[4] = LastRaw[4]; raw[5] = LastRaw[5];
    raw[6] = LastRaw[6]; raw[7] = LastRaw[7]; raw[8] = LastRaw[8];
}
