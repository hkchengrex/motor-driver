#include "pwm.h"
#include "limit.h"

volatile s32 curr_pwm = 0;
volatile bool stfu_mode = false;

void motor_init(){
	GPIO_InitTypeDef GPIO_InitStructure;

	//PWM Pin
	MOTOR_MAG_RCC_init();
	GPIO_InitStructure.GPIO_Pin = MOTOR_MAG_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR_MAG_GPIOx, &GPIO_InitStructure);

	//Direction Pin
	MOTOR_DIR_RCC_init();
	
	#if MOTOR_CONTROLLER_VERSION == 3
		GPIO_InitStructure.GPIO_Pin = MOTOR_DIR1_Pin | MOTOR_DIR2_Pin;
	#elif MOTOR_CONTROLLER_VERSION == 5
		GPIO_InitStructure.GPIO_Pin = MOTOR_DIR_Pin;
	#else
		#error ??
	#endif
	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(MOTOR_DIR_GPIOx, &GPIO_InitStructure);
	
	MOTOR_TIM_RCC_init();
	pwm_init(false);
}

void pwm_init(bool stfu){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_DeInit(MOTOR_TIM);
	
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM;
	if (!stfu){
		TIM_TimeBaseStructure.TIM_Period = MAX_PWM;
	}else{
		//For silence mode, reduce period by half -> doubles frequency
		TIM_TimeBaseStructure.TIM_Period = (MAX_PWM+1)/2 - 1;
	}
	TIM_TimeBaseStructure.TIM_ClockDivision =  TIM_CKD_DIV1;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(MOTOR_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);

	TIM_Cmd(MOTOR_TIM, ENABLE);
	TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE);
	
	stfu_mode = stfu;
	set_pwm(0);
}

static void set_dir(Direction dir){
	#if MOTOR_CONTROLLER_VERSION == 3
		if (dir == DIR_POS) {
			GPIO_SetBits(MOTOR_DIR_GPIOx, MOTOR_DIR2_Pin);
			GPIO_ResetBits(MOTOR_DIR_GPIOx, MOTOR_DIR1_Pin);
			
		} else if (dir == DIR_NEG) {
			GPIO_SetBits(MOTOR_DIR_GPIOx, MOTOR_DIR1_Pin);
			GPIO_ResetBits(MOTOR_DIR_GPIOx, MOTOR_DIR2_Pin);
		}
		
	#elif MOTOR_CONTROLLER_VERSION == 5
		if (dir == DIR_POS) {
			GPIO_SetBits(MOTOR_DIR_GPIOx, MOTOR_DIR_Pin);
		} else if (dir == DIR_NEG) {
			GPIO_ResetBits(MOTOR_DIR_GPIOx, MOTOR_DIR_Pin);
		}
		
	#endif
}

void set_pwm(s32 pwm){
	
	if (pwm > MAX_PWM*MAX_POWER_PERCENTAGE/100){
		pwm = MAX_PWM*MAX_POWER_PERCENTAGE/100;
	}else if(pwm < -MAX_PWM*MAX_POWER_PERCENTAGE/100){
		pwm = -MAX_PWM*MAX_POWER_PERCENTAGE/100;
	}
	
	//Limit the motor's action
	if ((get_limit(LIMIT_HI) && pwm > 0) || (get_limit(LIMIT_LO) && pwm < 0)){
		pwm = 0;
	}
	
	//pwm = -pwm;
	
	if (pwm != 0){
		set_dir(pwm*PWM_DIR > 0 ? DIR_POS : DIR_NEG);
	}
	
	curr_pwm = pwm;
	#if MOTOR_CONTROLLER_VERSION == 3
		pwm = (MAX_PWM - ABS(pwm))*SIGN(pwm);
	#endif
	
	if (!stfu_mode){
		TIM_SetCompare3(MOTOR_TIM, ABS(pwm));
	}else{
		TIM_SetCompare3(MOTOR_TIM, ABS(pwm/2));
	}
}

void adj_pwm(s32 adj){
	set_pwm(curr_pwm + adj);
}

s32 get_curr_pwm(){
	return curr_pwm;
}
