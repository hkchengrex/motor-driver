/**************************************************************************************************
** Motor driver - Project Gungnir
**
** This is a generic motor controller program.
** Please read the files in MotorDriver_Gungnir/wiki for more information.
**
** ROBOCON 2017
** H K U S T
**
** Author:	Rex Cheng
** Contact:	hkchengad@connect.ust.hk
**
** v1.1 Feburary 2017 (Path generation and PID)
** v1.2 March 2017 (Smoother path generation and message filtering)
** v2.0 March 2017 (Improved performance with repesct to MOSFET and H-bridge property
**									, auto-recovery and encoder failure feedback, uart improvement and documentations)
** v2.1 April 2017 (Changed a lot of 64 bit variables to 32 bit.)
** v2.2 April 2017 (bugs fix)
** v2.3 April 2017 (Change CAN ID, separate command and feedback ID, added competition mode)
** v2.4 April 2017 (Added STFU mode under the pressure of my fellows T_T)
** v3.0 June 2017 (Added limit switches and AT mode)
** v3.1 July 2017 (Added real time limit switch monitoring and open loop limit switch handling)
** v3.2 July 2017 (Added encoder filter, AT mode fix) 
**
** “Óðinn á yðr alla!” (“Odin owns all of you!”)
**
** >> "I take it from this that you are an Accelerationist?
** >> "No, " said Sam, "simply an inquirer. I am curious, that's all, as to the reasons."
** >> ~Lord of Light
**
****************************************************************************************************/

/**
* ATTENTION
* You don't really need compile a debug mode version to debug.
* You can flash a normal program and connect a jumper to:
- For 3.2: UART3 (middle two, GND and TX)
- For 5.1: SWD (Rightmost two, GND and IO)
* The program will run at DEBUG_MODE. 
* Defining the DEBUG_MODE below force the program to enter debug mode.
*/

//-----------------
//vvvvvvvvvvvvvvvvv

//#define DEBUG_MODE //<<<<<<<

//^^^^^^^^^^^^^^^^^
//-----------------

#include "stm32f10x.h"
#include "ticks.h"
#include "define.h"
#include "led.h"
#include "limit.h"
#include "can_motor.h"
#include "feedback.h"
#include "control.h"
#include "debug.h"
#include "at_mode.h"

static u32 last_ticks = (u32)-1;

int main(){

	ticks_init(); //He lives as long as he is destined to live.
	can_init(); //Then, when he dies they carry him to be offered in the fire. 
	can_rx_init(); //The fire becomes his fire,
	can_motor_init(); //the smoke his smoke,
	motor_init(); //the flame his flame,
	encoder_init(); //the cinder his cinder,
	uart_init(COM1, 115200); //and the sparks his sparks.
	limit_init();  //In this fire the god offer the man.
	led_init();
	//at_mode_init();
	
	#ifndef DEBUG_MODE
		config_debug_uart_pin();
	#endif
	
	//Wait for some time to denoise
	while(get_ticks() <  1 + this_motor * 2){
		// (ﾒ3[____]
		// ZzzzzZzzzzZzzzzzz
		__asm__("nop");
	}
	
	control_init(); //Out of that offering the man emerges radiant.
	
	#ifdef DEBUG_MODE
		enter_debug_mode();
	#else
		detect_debug_pin();
	#endif
	
	while(1){
		u32 this_ticks = get_ticks();
		if (this_ticks != last_ticks){
			
			static u32 last_encoder_ticks = 0;
			if (this_ticks - last_encoder_ticks >= 4){
				static bool enc_last_state = false;
				bool enc_this_state = encoder_malfunction();
				
				//Handle encoder malfunction and recovery signal
				if (enc_this_state && !enc_last_state){
					//If encoder state change from working to not working
					life_signal_freq = MALF_FLASH_FREQ;
					
				}else if (enc_last_state && !enc_this_state){
					//If encoder state change from not working to working
					life_signal_freq = NORM_FLASH_FREQ;
				}
				
				//CAN feedback to mainboard
				can_feedback(enc_this_state);
				
				enc_last_state = enc_this_state;
				last_encoder_ticks = this_ticks;
			}
			
			static u32 last_uart_ticks = 0;
			if (this_ticks - last_uart_ticks >= 32){
				//UART feedback for debugging
				uart_feedback();
				
				if (is_debugging()){
					debug_loop();
				}
				
				last_uart_ticks = this_ticks;
			}
			
			static u32 last_life_ticks = 0;
			if (!is_debugging() && (this_ticks - last_life_ticks >= life_signal_freq)){
				
				//LED Life signal
				led_blink(LED_2);
				
				last_life_ticks = this_ticks;
			}
			
			//ON -> Command received in last 512 ms via CAN
			if (last_recv_ticks == 0 || ((this_ticks - last_recv_ticks) > 512)){
				led_control(LED_1, Bit_RESET);
			}else{
				led_control(LED_1, Bit_SET);
			}
			
			last_ticks = this_ticks;
		}
	}
}
