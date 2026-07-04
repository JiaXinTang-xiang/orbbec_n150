/**
 * vision_task.c
 * USB CDC 视觉数据接收、解析、反馈
 *
 * 协议: 0xA5 0x5A + 14B payload + 1B XOR = 17 bytes
 * payload: class(1) + cx(2) + cy(2) + x(2) + y(2) + z(2) + dist(2) + rsvd(1)
 *
 * 反馈帧: [0xAA][CMD][DATA][CRC]  4 bytes, CRC = XOR(0xAA,CMD,DATA)
 *   CMD=0x81: 状态标志  DATA: bit2=收到视觉帧
 */
#include "vision_task.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "usbd_cdc_if.h"
#include <string.h>

vision_data_t g_vision_data;

static fifo_s_t vision_fifo;
static uint8_t vision_fifo_buf[VISION_FIFO_SIZE];

static vision_unpack_step_e step = VISION_STEP_SYNC1;
static uint8_t  frame_buf[VISION_FRAME_SIZE];
static uint8_t  frame_idx = 0;

void vision_fifo_put(uint8_t *data, uint16_t len)
{
    fifo_s_puts(&vision_fifo, (char *)data, len);
}

static void send_feedback(uint8_t cmd, uint8_t data)
{
    uint8_t buf[4];
    buf[0] = 0xAA;
    buf[1] = cmd;
    buf[2] = data;
    buf[3] = 0xAA ^ cmd ^ data;  // XOR checksum
    CDC_Transmit_FS(buf, 4);
}

static uint8_t vision_parse_frame(uint8_t *buf)
{
    uint8_t i, c = 0;
    for (i = 0; i < VISION_FRAME_SIZE - 1; i++)
        c ^= buf[i];
    if (c != buf[VISION_FRAME_SIZE - 1])
        return 0;

    uint8_t *p = buf + 2;
    g_vision_data.target_class = p[0];
    g_vision_data.cx           = p[1] | (p[2] << 8);
    g_vision_data.cy           = p[3] | (p[4] << 8);
    g_vision_data.x_mm         = (int16_t)(p[5] | (p[6] << 8));
    g_vision_data.y_mm         = (int16_t)(p[7] | (p[8] << 8));
    g_vision_data.z_mm         = (int16_t)(p[9] | (p[10] << 8));
    g_vision_data.dist_mm      = p[11] | (p[12] << 8);
    g_vision_data.reserved     = p[13];

    // 回传反馈：收到视觉帧
    send_feedback(0x81, 0x04);
    return 1;
}

void vision_task(void const * argument)
{
    (void)argument;
    fifo_s_init(&vision_fifo, vision_fifo_buf, VISION_FIFO_SIZE);

    for (;;)
    {
        while (fifo_s_used(&vision_fifo) > 0)
        {
            uint8_t byte = (uint8_t)fifo_s_get(&vision_fifo);

            switch (step)
            {
            case VISION_STEP_SYNC1:
                if (byte == VISION_SYNC1) {
                    frame_buf[0] = byte;
                    frame_idx = 1;
                    step = VISION_STEP_SYNC2;
                }
                break;

            case VISION_STEP_SYNC2:
                if (byte == VISION_SYNC2) {
                    frame_buf[1] = byte;
                    frame_idx = 2;
                    step = VISION_STEP_DATA;
                } else {
                    step = VISION_STEP_SYNC1;
                }
                break;

            case VISION_STEP_DATA:
                frame_buf[frame_idx++] = byte;
                if (frame_idx >= VISION_FRAME_SIZE - 1) {
                    step = VISION_STEP_CHECK;
                }
                break;

            case VISION_STEP_CHECK:
                frame_buf[VISION_FRAME_SIZE - 1] = byte;
                vision_parse_frame(frame_buf);
                step = VISION_STEP_SYNC1;
                frame_idx = 0;
                break;
            }
        }
        osDelay(1);
    }
}
