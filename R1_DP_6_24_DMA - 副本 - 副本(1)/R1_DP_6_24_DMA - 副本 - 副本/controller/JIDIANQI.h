#ifndef JIDIANQI_H
#define JIDIANQI_H
#include "struct_typedef.h"

void relay_control(uint8_t id, uint8_t state);

void JIDIANQI_ON(void);
void JIDIANQI_OFF(void);

void electyic_relay_1_ON();
void electyic_relay_1_OFF();

void electyic_relay_2_ON();
void electyic_relay_2_OFF();

void electyic_relay_3_ON();
void electyic_relay_3_OFF();

void electyic_relay_4_ON();
void electyic_relay_4_OFF();

void electyic_relay_5_ON();
void electyic_relay_5_OFF();

void electyic_relay_6_ON();
void electyic_relay_6_OFF();

void electyic_relay_7_ON();
void electyic_relay_7_OFF();

void electyic_relay_8_ON();
void electyic_relay_8_OFF();

void electyic_relay_9_ON();
void electyic_relay_9_OFF();

void electyic_relay_10_OFF();
void electyic_relay_10_ON();

void electyic_relay_on(uint16_t open);
void electyic_relay_off(uint16_t stop);

void electyic_Init(void);

void electyic_task(void);

void relay_task(void);

void relay_task_mode(void);


#endif
