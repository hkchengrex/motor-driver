#include "led.h"

void led_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LED_1_RCC_init();
	LED_2_RCC_init();
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_InitStructure.GPIO_Pin = LED_1_Pin;
  GPIO_Init(LED_1_GPIOx, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED_2_Pin;
	GPIO_Init(LED_2_GPIOx, &GPIO_InitStructure);
	
	GPIO_WriteBit(LED_1_GPIOx, LED_1_Pin, Bit_RESET);
	GPIO_WriteBit(LED_2_GPIOx, LED_2_Pin, Bit_RESET);
}

void led_control(LED led, BitAction state){
	if (led & LED_1) {
		GPIO_WriteBit(LED_1_GPIOx, LED_1_Pin, state);
	}
	
	if (led & LED_2) {
		GPIO_WriteBit(LED_2_GPIOx, LED_2_Pin, state);
	}
}


void led_blink(LED led){	
	
	/*
		Twinkle, twinkle, little star,
		How we wonder what you are.
		Up above the world so high,
		Like a diamond in the sky.
	*/
	
	if (led & LED_1) {
		GPIO_WriteBit(LED_1_GPIOx, LED_1_Pin, (BitAction) !GPIO_ReadOutputDataBit(LED_1_GPIOx, LED_1_Pin));
	}
	
	if (led & LED_2) {
		GPIO_WriteBit(LED_2_GPIOx, LED_2_Pin, (BitAction) !GPIO_ReadOutputDataBit(LED_2_GPIOx, LED_2_Pin));
	}
}


