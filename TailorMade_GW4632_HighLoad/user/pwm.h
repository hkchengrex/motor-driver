#ifndef _PWM_H
#define _PWM_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "math.h"
#include "define.h"

#define MAX_PWM									7999
#define MOTOR_TIM								TIM3
#define MOTOR_TIM_RCC_init()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)

#define MOTOR_MAG_GPIOx					GPIOB
#define MOTOR_MAG_Pin						GPIO_Pin_0

#if MOTOR_CONTROLLER_VERSION == 3
	#define MOTOR_DIR_GPIOx					GPIOC
	#define MOTOR_DIR1_Pin					GPIO_Pin_4
	#define MOTOR_DIR2_Pin					GPIO_Pin_5
	#define MOTOR_DIR_RCC_init()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
	
#elif MOTOR_CONTROLLER_VERSION == 5
	#define MOTOR_DIR_GPIOx					GPIOB
	#define MOTOR_DIR_Pin						GPIO_Pin_1
	#define MOTOR_DIR_RCC_init()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
	
#endif

#define MOTOR_MAG_RCC_init()		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)

#define MAX_POWER_PERCENTAGE 100

typedef enum {
	DIR_POS = 0,
	DIR_NEG = 1,
	DIR_NEU = 2
} Direction;
 
void motor_init(void);
void pwm_init(bool stfu);
void set_pwm(s32 pwm);
void adj_pwm(s32 adj);
s32 get_curr_pwm(void);

#endif
