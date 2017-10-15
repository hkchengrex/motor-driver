#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "ticks.h"
#include "define.h"

#define DEBUG_SPEED (MAX_ORIG_VEL)

#if MOTOR_CONTROLLER_VERSION == 3
	#define DEBUG_SELECT_PIN            GPIO_Pin_10
	#define DEBUG_SELECT_PORT           GPIOC
	#define DEBUG_SELECT_RCC						RCC_APB2Periph_GPIOC
#elif MOTOR_CONTROLLER_VERSION == 5
	#define DEBUG_SELECT_PIN            GPIO_Pin_13
	#define DEBUG_SELECT_PORT           GPIOA
	#define DEBUG_SELECT_RCC						RCC_APB2Periph_GPIOA
#endif

void debug_loop(void);
bool is_debugging(void);
void enter_debug_mode(void);
void config_debug_uart_pin(void);
void detect_debug_pin(void);

#endif
