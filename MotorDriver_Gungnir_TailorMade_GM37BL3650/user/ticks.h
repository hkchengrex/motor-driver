#ifndef _TICKS_H
#define _TICKS_H

#include "stm32f10x_tim.h"
#include "system_stm32f10x.h"
#include "misc.h"
#include "define.h"

extern volatile u32 life_signal_freq;

#define TICKS_TIM							TIM4
#define TICKS_RCC							RCC_APB1Periph_TIM4
#define TICKS_IRQn						TIM4_IRQn
#define TICKS_IRQHandler			void TIM4_IRQHandler(void)

extern volatile u32 ms_ticks;
#define get_ticks() (ms_ticks)

extern volatile u32 life_signal_freq;

void ticks_init(void);
void TIM4_IRQHandler(void);

#endif
