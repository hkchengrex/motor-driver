#include "at_mode.h"
#include "uart.h"
#include "feedback.h"
#include "define.h"
#include "control.h"
#include "debug.h"

static char* build_time = __DATE__ " " __TIME__ "\r";
static char at_buf[50] = {0};
static u8 at_pointer = 0;

static inline u8 match_prefix(char* target, u8 start, u8 len){
	u8 end = start+len;
	
	if (end > at_pointer) return 0;
	
	while(end-- > start){
		if (target[end-start] != at_buf[end]){
			return 0;
		}
	}
	return 1;
}

static bool err_occur = false;
static inline s32 resolve_number(u8 start, u8 end){
	err_occur = false;
	s32 num = 0;
	bool neg = false;
	
	if (start >= end){
		err_occur = true;
		return 0;
	}
	
	if (at_buf[start] == '-'){
		neg = true;
		start++;
	}
	
	for (u8 i=start; i<end; i++){
		if (at_buf[i] >= 48 && at_buf[i] <= 57){
			u8 ah = at_buf[i] - 48;
			num = num*10 + ah;
		}else{
			err_occur = true;
			return 0;
		}
	}
	
	return neg?-num:num;
}

#define RESPOND(x) uart_tx(UART_FEEDBACK_COM, x)
#define RESPOND_P(x, y) uart_tx(UART_FEEDBACK_COM, x, y)

static void at_resolve(){
	if (at_pointer >= 2 && match_prefix("AT", 0, 2)){
		//AT matched
		if (at_pointer == 2){
			RESPOND("OK\r");
			
		}else if (match_prefix("+", 2, 1)){
			//AT+ matched
			if (match_prefix("SHUT", 3, 4)){
				//AT+SHUT
				uart_feedback_shut();
				RESPOND("OK\r");
				
			}else if(match_prefix("LIVE", 3, 4)){
				//AT+LIVE
				uart_feedback_live();
				RESPOND("OK\r");
				
			}else if(match_prefix("BUILD?", 3, 6)){
				//AT+BUILD?
				RESPOND(build_time);
				
			}else if(match_prefix("MOTOR_ID", 3, 8)){
				//AT+MOTOR_ID(?=)
				if (at_buf[11] == '?' && at_pointer == 12){
					RESPOND_P("MOTOR_%d\r", this_motor+1);
				}else if(at_buf[11] == '='){
					s32 num = resolve_number(12, at_pointer);
					if (!err_occur && num >= 0 && num <= 15){
						this_motor = num;
						RESPOND("OK\r");
					}else{
						RESPOND("ERROR\r");
					}
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("CONTROL_FREQ?", 3, 13)){
				//AT+CONTROL_FREQ?
				RESPOND_P("%d\r", CONTROL_FREQ);
				
			}else if(match_prefix("LIST_VALUE", 3, 10)){
				//AT+LIST_VALUE
				uart_tx(UART_FEEDBACK_COM, "POS:%d\rVEL:%d\rPWM:%d\rACC:%d\rMAX VEL:%d\r", get_unit_cnt(), get_unit_vel(), get_curr_pwm(), max_accel, max_veloc);
				
			}else if(match_prefix("ACC=", 3, 4)){
				//AT+ACC=
				s32 num = resolve_number(7, at_pointer);
				if (!err_occur){
					set_max_acc(num);
					RESPOND("OK\r");
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("MAX_VEL=", 3, 8)){
				//AT+MAX_VEL=
				s32 num = resolve_number(11, at_pointer);
				if (!err_occur){
					set_max_vel(num);
					RESPOND("OK\r");
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("PATH_POS=", 3, 9)){
				//AT+PATH_POS=
				s32 num = resolve_number(12, at_pointer);
				if (!err_occur){
					Path* p = setup_path(num);
					uart_tx(UART_FEEDBACK_COM, "VT:%d\rT1:%d\rT2:%d\rT3:%d\rT1 PT:%d\rT2 PT:%d\rT3 PT:%d\rDIR:%d\r", 
									p->vt, p->t1, p->t2, p->t3, p->t1_pt, p->t2_pt, p->end_pt, p->dir);
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("PATH_VEL=", 3, 9)){
				//AT+PATH_VEL=
				s32 num = resolve_number(12, at_pointer);
				if (!err_occur){
					Path* p = set_close_loop_vel(num);
					uart_tx(UART_FEEDBACK_COM, "VT:%d\rT1:%d\rT2:%d\rT3:%d\rT1 PT:%d\rT2 PT:%d\rT3 PT:%d\rDIR:%d\r", 
									p->vt, p->t1, p->t2, p->t3, p->t1_pt, p->t2_pt, p->end_pt, p->dir);
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("PWM=", 3, 4)){
				//AT+PWM=
				s32 num = resolve_number(7, at_pointer);
				if (!err_occur && num>=-MAX_PWM && num<=MAX_PWM){
					set_open_loop_vel(num);
					RESPOND("OK\r");
				}else{
					RESPOND("ERROR\r");
				}
				
			}else if(match_prefix("DEBUG", 3, 5)){
				//AT+DEBUG
				enter_debug_mode();
				RESPOND("OK\r");
				
			}else if(match_prefix("LIMIT?", 3, 6)){
				//AT+LIMIT?
				uart_tx(UART_FEEDBACK_COM, "LO ACT:%d STATE:%d\rHI ACT:%d STATE:%d\r", ACTIVATE_LOW_LIMIT_SWITCH, get_limit(LIMIT_LO), ACTIVATE_HIGH_LIMIT_SWITCH, get_limit(LIMIT_HI));
				
			}else{
				RESPOND("ERROR\r");
			}
			
		}else{
			RESPOND("ERROR\r");
		}
			
	}else{
		RESPOND("ERROR\r");
	}
	
	at_pointer = 0;
}
#undef RESPOND

static void at_handler(u8 data){
	if (data == '\r'){
		at_resolve();
	}else{
		at_buf[at_pointer] = data;
		at_pointer = (at_pointer + 1) % 50;
	}
}

void at_mode_init(){
	uart_interrupt_init(UART_FEEDBACK_COM, at_handler);
	at_pointer = 0;
}
