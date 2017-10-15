#include "encoder.h"
#include "pwm.h"
#include "define.h"

volatile s32 encoder_cnt = 0;
volatile s32 encoder_vel = 0;
volatile s32 encoder_acc = 0;

void encoder_init(){
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef encoder_TIM_TimeBaseStructure;

	encoder_gpio_rcc_init();
	
	GPIO_InitStructure.GPIO_Pin = ENCODER_TIM_PORT1 | ENCODER_TIM_PORT2;
	GPIO_InitStructure.GPIO_Mode = ENCODER_INPUT_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENCODER_TIM_GPIOx, &GPIO_InitStructure);

	encoder_rcc_init();
	
	encoder_TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRESCALER - 1;
	encoder_TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	encoder_TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	encoder_TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(ENCODER_TIM, &encoder_TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(ENCODER_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	encoder_cnt = encoder_vel = encoder_acc = 0;
	
	TIM_SetCounter(ENCODER_TIM, 0);
	TIM_Cmd(ENCODER_TIM, ENABLE);
}

static u32 encoder_loose_count = 0;

//To be called at @CONTROL_FREQ
void encoder_update(void){
	const s32 count = (s16)TIM_GetCounter(ENCODER_TIM);
	TIM_SetCounter(ENCODER_TIM, 0);
	
	encoder_acc = count-encoder_vel;
	encoder_vel = count;
	encoder_cnt +=  count;
}

bool encoder_malfunction(){
	if (encoder_cnt == 0){
		//Wait for 0.10 sec if encoder has never worked
		return encoder_loose_count > CONTROL_FREQ/10;
	}else{
		//Wait for 0.25 sec for encoder was working before
		return encoder_loose_count > CONTROL_FREQ/4;
	}
}
