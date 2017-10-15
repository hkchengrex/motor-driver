#ifndef _ENCODER_H
#define _ENCODER_H

#include "stm32f10x.h"
#include "math.h"
#include "define.h"
#include <stdbool.h>

#define ENCODER_TIM												TIM2
#define ENCODER_TIM_PORT1									GPIO_Pin_0
#define ENCODER_TIM_PORT2									GPIO_Pin_1
#define ENCODER_TIM_GPIOx									GPIOA

#define encoder_gpio_rcc_init()						RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
#define encoder_rcc_init()								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE)

void encoder_init(void);
void encoder_update(void);
bool encoder_malfunction(void);

extern volatile s32 encoder_cnt;
extern volatile s32 encoder_vel, encoder_acc;

#define get_cnt() (encoder_cnt)
#define get_vel() (encoder_vel)
#define get_acc() (encoder_acc)

#endif
