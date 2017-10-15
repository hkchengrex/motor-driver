#ifndef _LED_H
#define _LED_H

//Trivial. No comment.

#include "stm32f10x.h"
#include "define.h"

#if MOTOR_CONTROLLER_VERSION == 3
	#define LED_1_GPIOx				GPIOC
	#define LED_2_GPIOx				GPIOC
	
	#define LED_1_Pin				GPIO_Pin_0
	#define LED_2_Pin				GPIO_Pin_1
	#define LED_1_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
	#define LED_2_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
	
#elif MOTOR_CONTROLLER_VERSION == 5
	#define LED_1_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
	#define LED_2_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)

	#define LED_1_GPIOx			GPIOA
	#define LED_1_Pin				GPIO_Pin_2
	#define LED_2_GPIOx			GPIOB
	#define LED_2_Pin				GPIO_Pin_8
#endif

typedef enum {
	LED_1 = 1 << 0,
	LED_2 = 1 << 1,
	LED_BOTH = LED_1 | LED_2
} LED;

void led_init(void);
void led_control(LED led, BitAction state);
void led_blink(LED led);

#endif
