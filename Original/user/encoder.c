#include "encoder.h"
#include "pwm.h"
#include "define.h"

volatile s32 encoder_cnt = 0;
volatile s32 encoder_vel = 0;
volatile s32 encoder_acc = 0;

//My little secret functions

/**
  * @brief  Configure the TI1 as Input.
  * @param  TIMx: where x can be 1 to 17 except 6 and 7 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *   This parameter can be one of the following values:
  *     @arg TIM_ICPolarity_Rising
  *     @arg TIM_ICPolarity_Falling
  * @param  TIM_ICSelection: specifies the input to be used.
  *   This parameter can be one of the following values:
  *     @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
  *     @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
  *     @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *   This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0;
  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC1E);
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
  /* Select the Input and set the filter */
  tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC1S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC1F)));
  tmpccmr1 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
  
  if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
     (TIMx == TIM4) ||(TIMx == TIM5))
  {
    /* Select the Polarity and set the CC1E Bit */
    tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);
  }
  else
  {
    /* Select the Polarity and set the CC1E Bit */
    tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P | TIM_CCER_CC1NP));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);
  }

  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Configure the TI2 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9, 12 or 15 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *   This parameter can be one of the following values:
  *     @arg TIM_ICPolarity_Rising
  *     @arg TIM_ICPolarity_Falling
  * @param  TIM_ICSelection: specifies the input to be used.
  *   This parameter can be one of the following values:
  *     @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
  *     @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
  *     @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *   This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0, tmp = 0;
  /* Disable the Channel 2: Reset the CC2E Bit */
  TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC2E);
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 4);
  /* Select the Input and set the filter */
  tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC2S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC2F)));
  tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);
  tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8);
  
  if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
     (TIMx == TIM4) ||(TIMx == TIM5))
  {
    /* Select the Polarity and set the CC2E Bit */
    tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC2P));
    tmpccer |=  (uint16_t)(tmp | (uint16_t)TIM_CCER_CC2E);
  }
  else
  {
    /* Select the Polarity and set the CC2E Bit */
    tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC2P | TIM_CCER_CC2NP));
    tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC2E);
  }
  
  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}

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
	
	//If the encoder is so fast, you can try reducing the filter 
	TI1_Config(ENCODER_TIM, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x04);
	TI2_Config(ENCODER_TIM, TIM_ICPolarity_Rising, TIM_ICSelection_DirectTI, 0x04);
	
	encoder_cnt = encoder_vel = encoder_acc = 0;
	
	TIM_SetCounter(ENCODER_TIM, 0);
	TIM_Cmd(ENCODER_TIM, ENABLE);
}

static u32 encoder_loose_count = 0;

//To be called at @CONTROL_FREQ
void encoder_update(void){
	const s32 count = (s16)TIM_GetCounter(ENCODER_TIM) * ENC_DIR;
	TIM_SetCounter(ENCODER_TIM, 0);
	
	encoder_acc = count-encoder_vel;
	encoder_vel = count;
	encoder_cnt +=  count;
	
	if (encoder_vel == 0){
		if (ABS(get_curr_pwm()) == MAX_PWM*MAX_POWER_PERCENTAGE/100){
			encoder_loose_count++;
		}
	}else{
		encoder_loose_count = 0;
	}
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
