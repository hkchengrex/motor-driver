#include "feedback.h"
#include "pwm.h"
#include "encoder.h"
#include "path.h"
#include "ticks.h"
#include "math.h"
#include "limit.h"
#include "control.h"

static bool uart_shut = false;

void uart_feedback_shut(){
	uart_shut = true;
}

void uart_feedback_live(){
	uart_shut = false;
}

//Send feedback via UART
void uart_feedback(){
	if (!uart_shut){
		uart_tx(UART_FEEDBACK_COM, "%d\t%d\t%d\t%d\t%d\r", 
							encoder_cnt, get_unit_vel(), get_path_pos(), get_path_vel(),
							get_curr_pwm());
	}
	//uart_tx(UART_FEEDBACK_COM, "%d %d\n", CAN_GetReceiveErrorCounter(CAN1), CAN_GetLSBTransmitErrorCounter(CAN1));
}

//Send feedback via CAN
void can_feedback(bool enc_working){
	CanMessage msg;
	msg.id = get_feedback_id(this_motor);
	msg.length = GUN_CAN_ENCODER_FEEDBACK_LENGTH;
	msg.data[0] = GUN_CAN_ENCODER_FEEDBACK_ID;
	
	if (enc_working){
		msg.data[0] |= GUN_CAN_ENCODER_OK_MASK;
	}
	
	if (get_limit(LIMIT_LO)){
		msg.data[0] |= GUN_CAN_ENCODER_LM_LO_MASK;
	}
	
	if (get_limit(LIMIT_HI)){
		msg.data[0] |= GUN_CAN_ENCODER_LM_HI_MASK;
	}
	
	s32 round_cnt = encoder_cnt;
	msg.data[1] = round_cnt;
	msg.data[2] = round_cnt >> 8;
	msg.data[3] = round_cnt >> 16;
	msg.data[4] = round_cnt >> 24;
	
	s32 pwm = get_curr_pwm();
	msg.data[5] = pwm;
	msg.data[6] = pwm >> 8;
	msg.data[7] = ((pwm >> 16) & 127) | ((pwm >> 24) & 128);
	
	can_tx_enqueue(msg);
}

void yield_limit_feedack(LIMIT_ID id, s32 enc){
	CanMessage msg;
	msg.id = get_feedback_id(this_motor);
	msg.length = GUN_CAN_LIMIT_FEEDBACK_LENGTH;
	
	if (id == LIMIT_LO){
		msg.data[0] = GUN_CAN_LIMIT_LO_FEEDBACK;
	}else{
		msg.data[0] = GUN_CAN_LIMIT_HI_FEEDBACK;
	}
	
	msg.data[1] = enc;
	msg.data[2] = enc >> 8;
	msg.data[3] = enc >> 16;
	msg.data[4] = enc >> 24;
	
	can_tx_enqueue(msg);
}

void pt_arrival_feedback(u8 point){
	CanMessage msg;
	msg.id = get_feedback_id(this_motor);
	msg.length = GUN_CAN_ARRIVAL_FEEDBACK_LENGTH;
	
	switch (point){
		case 0:
			msg.data[0] = GUN_CAN_ARRIVAL_PT_1_FEEDBACK;
			break;
		case 1:
			msg.data[0] = GUN_CAN_ARRIVAL_PT_2_FEEDBACK;
			break;
		case 2:
			msg.data[0] = GUN_CAN_ARRIVAL_PT_3_FEEDBACK;
			break;
	}
	
	can_tx_enqueue(msg);
}
