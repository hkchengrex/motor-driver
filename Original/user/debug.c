#include "debug.h"
#include "control.h"
#include "path.h"
#include "led.h"

static bool m_is_debugging = false;
static u32 loop_count = 0;

void debug_loop(){
	m_is_debugging = true;
	
	if (loop_count == 0){
		set_max_vel(DEBUG_SPEED);
		led_control(LED_BOTH, Bit_SET);
		set_open_loop_vel(0);
		
	}else if (loop_count == 32){
		led_control(LED_1, Bit_RESET);
		set_close_loop_vel(DEBUG_SPEED);
		
	}else if (loop_count == 32 + 94){
		led_control(LED_2, Bit_RESET);
		set_close_loop_vel(0);
		
	}else if (loop_count == 32 + 94*2){
		led_control(LED_1, Bit_SET);
		setup_path(0);
		
	}else if (loop_count > (32 + 94*2) && !is_path_running() && ABS(get_unit_cnt())<10 && get_unit_vel()==0){
		m_is_debugging = false;
		set_max_vel(MAX_ORIG_VEL);
	}
	
	loop_count++;
}

void config_debug_uart_pin(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(DEBUG_SELECT_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = DEBUG_SELECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(DEBUG_SELECT_PORT, &GPIO_InitStructure);
}

void enter_debug_mode(){
	loop_count = 0;
	m_is_debugging = true;
}

void detect_debug_pin(){
	bool debug_flag = 1;
	for (u16 i=0; i<20; i++){
		debug_flag &= (!GPIO_ReadInputDataBit(DEBUG_SELECT_PORT, DEBUG_SELECT_PIN));
		for (u16 j=0; j<20; j++){
			__asm__("nop");
		}
	}
	
	m_is_debugging = debug_flag;
}

bool is_debugging(){
	return m_is_debugging;
}
