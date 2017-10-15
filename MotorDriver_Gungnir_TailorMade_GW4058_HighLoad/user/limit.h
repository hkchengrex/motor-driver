#ifndef _LIMIT_H
#define _LIMIT_H

//For using two limit switches to limit the movement of the motor.

//For 3.2, UART3 is used. For 5.x, SWH and SWL is used.
//Note the internal pull up of the 5.x board.

#include "stm32f10x.h"
#include "define.h"
#include "can_motor.h"

#if MOTOR_CONTROLLER_VERSION == 3
	#define LM_HI_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
	#define LM_LO_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)

	#define LM_HI_GPIOx				GPIOC
	#define LM_HI_Pin				GPIO_Pin_10
	#define LM_LO_GPIOx				GPIOC
	#define LM_LO_Pin				GPIO_Pin_11
	
	#define LM_INPUT_MODE GPIO_Mode_IPU
	
#elif MOTOR_CONTROLLER_VERSION == 5
	#define LM_HI_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)
	#define LM_LO_RCC_init()	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE)

	#define LM_HI_GPIOx			GPIOA
	#define LM_HI_Pin				GPIO_Pin_5
	#define LM_LO_GPIOx			GPIOA
	#define LM_LO_Pin				GPIO_Pin_4
	
	#define LM_INPUT_MODE GPIO_Mode_IN_FLOATING
#endif

typedef enum{
	LIMIT_LO,
	LIMIT_HI
}LIMIT_ID;

void limit_init(void);
u8 get_limit(LIMIT_ID id);
void limit_update(CloseLoopFlag flag);

#endif
