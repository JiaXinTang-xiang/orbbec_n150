#include "JIDIANQI.h"
#include "main.h"
#include "NRF24L01.h"

extern RemoteControlData_t rc_data;

/* 继电器步间延时 (ms), TIM1 1kHz 下每 tick = 1ms */
#define RELAY_STEP_DELAY_MS  1000

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} RelayConfig_t;

/* 13 */
void relay_task(void)
{
    static uint8_t prev_key = 0;
    static uint8_t relay_on[10] = {0};   // 10路继电器状态: 0=OFF, 1=ON
    uint8_t key = rc_data.key_number;

	if(rc_data.key_bg_left==1&&rc_data.key_bg_right==3)
	{
    // ── 普通按键 1-8 → 继电器 1-8：边沿触发，按一下翻转（低电平触发）──
    for (uint8_t i = 1; i <= 8; i++)
    {
        if (prev_key == 0 && key == i)
        {
            relay_on[i - 1] = !relay_on[i - 1];
            // 低电平触发: ON=GPIO_PIN_RESET(0), OFF=GPIO_PIN_SET(1)
            relay_control(i, relay_on[i - 1] ? 0 : 1);
        }
    }
    prev_key = key;

    // ── 编码器按键 → 继电器 9-10：1=ON 0=OFF ──
    relay_control(9, (rc_data.encoder1_key == 1) ? 1 : 0);
    relay_control(10, (rc_data.encoder2_key == 1) ? 1 : 0);
  }
	
  
}

/*
 * relay_task_1  —  非阻塞继电器时序状态机
 * 触发条件: key_bg_left==1 && key_bg_right==4
 * 按键 1~6 各自对应一组继电器动作序列, 每步间隔 300 ms
 * 本函数在 TIM1 ISR (1 kHz) 中调用, 每次调用只执行一步或计数等待
 */
/* 14 */
void relay_task_mode(void)
{
    /* ── 静态状态变量 ── */
    static uint8_t  seq_key  = 0;    /* 当前触发的按键号 (1~6), 0 = 空闲   */
    static uint8_t  seq_step = 0;    /* 当前步骤索引 (0-based)              */
    static uint16_t seq_tick = 0;    /* 等待计数器 (300 = 300 ms @1kHz)     */
    static uint8_t  prev_key = 0;    /* 上一拍 key_number, 用于上升沿检测   */
    static uint8_t  prev_k7  = 0;    /* 按键7 上一拍, 边沿检测              */
    static uint8_t  prev_k8  = 0;    /* 按键8 上一拍, 边沿检测              */
    static uint8_t  r8_state  = 0;   /* 继电器8 翻转状态 (case1用)          */
    static uint8_t  r7_state  = 0;   /* 继电器7 翻转状态 (case2用)          */

    uint8_t key = rc_data.key_number;

    /* ── 非本模式: 复位状态机 ── */
    if (!(rc_data.key_bg_left == 1 && rc_data.key_bg_right == 4)) {
        seq_key  = 0; seq_step = 0; seq_tick = 0;
        prev_key = 0; prev_k7 = 0; prev_k8 = 0;
        return;
    }

    /* ── 上升沿检测: 按键1~6触发新序列 ── */
    if (key != prev_key && key >= 1 && key <= 6) {
        seq_key  = key;
        seq_step = 0;
        seq_tick = 0;
        prev_k7  = 0;
        prev_k8  = 0;
    }
    prev_key = key;

    /* ── 按键7/8 上升沿 (快速响应, 不受步间延时影响) ── */
    uint8_t k7_up = (key == 7 && prev_k7 == 0) ? 1 : 0;
    uint8_t k8_up = (key == 8 && prev_k8 == 0) ? 1 : 0;
    prev_k7 = (key == 7);
    prev_k8 = (key == 8);

    /* ── case 1 中: 按键7翻转继电器8 ── */
    if (seq_key == 1) {
        if (k7_up) { r8_state = !r8_state; relay_control(8, r8_state); }
        if (seq_step == 0) { relay_control(6, 0); seq_step = 1; } /* 首步执行一次 */
        return;   /* 保持 case 1, 直到切换 case */
    }

    /* ── case 2 中: 按键7→继电器7, 按键8→继电器8 ── */
    if (seq_key == 2) {
        if (k7_up) { r7_state = !r7_state; relay_control(7, r7_state); }
        if (k8_up) { relay_control(8, !r8_state); r8_state = !r8_state; }
    }

    /* ── 无序列在跑 ── */
    if (seq_key == 0) return;

    /* ── 步间延时等待 (第 0 步立即执行) ── */
    if (seq_step > 0) {
        if (seq_tick < RELAY_STEP_DELAY_MS) {
            seq_tick++;
            return;
        }
        seq_tick = 0;
    }

    /* ── 执行当前步骤 (仅 case 2 需要步进序列) ── */
    switch (seq_key) {

    case 2:
        switch (seq_step) {
            case 0: relay_control(8, 1); seq_step = 1; break;
            case 1: relay_control(6, 1); seq_step = 2; break;
            case 2: relay_control(7, 0); seq_key  = 0; break;
        }
        break;

    /* ========== 按键 3: 8 步 ========== */
    case 3:
        switch (seq_step) {
            case 0: relay_control(3, 0); seq_step = 1; break;
            case 1: relay_control(8, 0); seq_step = 2; break;
            case 2: relay_control(4, 0); seq_step = 3; break;
            case 3: relay_control(5, 0); seq_step = 4; break;
            case 4: relay_control(1, 0); seq_step = 5; break;
            case 5: relay_control(3, 1); seq_step = 6; break;
			case 6: relay_control(4, 1); seq_step = 7; break;
            case 7: relay_control(5, 1); seq_step = 8; break;
            case 8: relay_control(7, 1); seq_key  = 0; break;
        }
        break;

    /* ========== 按键 4: 仅 1 步 ========== */
    case 4:
		
	switch (seq_step) {
            case 0: relay_control(6, 0); seq_key = 0; break;
           
        }
        break;
	

    /* ========== 按键 5: 2 步 ========== */
    case 5:
        switch (seq_step) {
            case 0: relay_control(8, 1); seq_step = 1; break;
            case 1: relay_control(6, 1); seq_step  = 2; break;
		    case 2: relay_control(7, 0); seq_key  = 0; break;
        }
        break;

    /* ========== 按键 6: 4 步 ========== */
    case 6:
        switch (seq_step) {
            case 0: relay_control(3, 0); seq_step = 1; break;
            case 1: relay_control(8, 0); seq_step = 2; break;
            case 2: relay_control(4, 0); seq_key = 0; break;
           
        }
        break;

    default:
        seq_key = 0;
        break;
    }
}

const RelayConfig_t relay_config[10] = {
    /* 1-8 low trigger ON=RESET */
    {GPIOF, GPIO_PIN_0},
    {GPIOF, GPIO_PIN_1},
    {GPIOI, GPIO_PIN_6},
    {GPIOC, GPIO_PIN_6},
    {GPIOE, GPIO_PIN_14},
    {GPIOE, GPIO_PIN_13},
    {GPIOE, GPIO_PIN_11},
    {GPIOE, GPIO_PIN_9},
    /* 9-10 high trigger ON=SET */
    {GPIOI, GPIO_PIN_7},
    {GPIOB, GPIO_PIN_7},

};


void relay_control(uint8_t id, uint8_t state)
{
	
    if (id < 1 || id > 10) return;
    
    HAL_GPIO_WritePin(
        relay_config[id - 1].port,
        relay_config[id - 1].pin,
        state ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
	
}

//void JIDIANQI_ON(void)
//{
//    for (uint8_t i = 1; i <= 9; i++) {
//        relay_control(i, 1);
//    }
//    HAL_Delay(1000);
//}

//void JIDIANQI_OFF(void)
//{
//    for (uint8_t i = 1; i <= 9; i++) {
//        relay_control(i, 0);
//    }
//}

//void electyic_relay_1_ON()  { relay_control(1, 1); }
//void electyic_relay_1_OFF() { relay_control(1, 0); }
//void electyic_relay_2_ON()  { relay_control(2, 1); }
//void electyic_relay_2_OFF() { relay_control(2, 0); }
//void electyic_relay_3_ON()  { relay_control(3, 1); }
//void electyic_relay_3_OFF() { relay_control(3, 0); }
//void electyic_relay_4_ON()  { relay_control(4, 1); }
//void electyic_relay_4_OFF() { relay_control(4, 0); }
//void electyic_relay_5_ON()  { relay_control(5, 1); }
//void electyic_relay_5_OFF() { relay_control(5, 0); }
//void electyic_relay_6_ON()  { relay_control(6, 1); }
//void electyic_relay_6_OFF() { relay_control(6, 0); }
//void electyic_relay_7_ON()  { relay_control(7, 1); }
//void electyic_relay_7_OFF() { relay_control(7, 0); }
//void electyic_relay_8_ON()  { relay_control(8, 1); }
//void electyic_relay_8_OFF() { relay_control(8, 0); }
//void electyic_relay_9_ON()  { relay_control(9, 1); }
//void electyic_relay_9_OFF() { relay_control(9, 0); }

void electyic_relay_1_OFF()  {HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);}
void electyic_relay_1_ON()   {HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);}   /* �͵�ƽ���� */

void electyic_relay_2_OFF()  {HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);}
void electyic_relay_2_ON()   {HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);}

void electyic_relay_3_OFF()  {HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);}
void electyic_relay_3_ON()   {HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);}

void electyic_relay_4_OFF()  {HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);}
void electyic_relay_4_ON()   {HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);}

void electyic_relay_5_OFF()  {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);}
void electyic_relay_5_ON()   {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);}

void electyic_relay_6_OFF()  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);}
void electyic_relay_6_ON()   {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);}

void electyic_relay_7_OFF()  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);}
void electyic_relay_7_ON()   {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);}

void electyic_relay_8_OFF()  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);}
void electyic_relay_8_ON()   {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);}

void electyic_relay_9_OFF()  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_RESET);}
void electyic_relay_9_ON()   {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_SET);}   /* �ߵ�ƽ���� */

void electyic_relay_10_OFF()  {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);}
void electyic_relay_10_ON()   {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);}   /* �ߵ�ƽ���� */


void electyic_relay_on(uint16_t open)
{
    relay_control(open, 1);
}

void electyic_relay_off(uint16_t stop)
{
    relay_control(stop, 0);
}


void JIDIANQI_ON(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
 
}

void JIDIANQI_OFF(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7,  GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
	
}

void electyic_Init(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  GPIO_PIN_SET);  
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7,  GPIO_PIN_RESET);

}

void electyic_task(void)
{
	

}
